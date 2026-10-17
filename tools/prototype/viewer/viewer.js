(() => {
  'use strict';
  const canvas = document.querySelector('#scene');
  const status = document.querySelector('#status');
  const setStatus = text => { status.textContent = text; };
  const el = (tag, text, className) => {
    const result = document.createElement(tag);
    if (text !== undefined) result.textContent = text;
    if (className) result.className = className;
    return result;
  };
  const length = vector => Math.hypot(...vector);
  const normalize = vector => { const size = length(vector) || 1; return vector.map(x => x / size); };
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const dot = (a, b) => a.reduce((sum, value, i) => sum + value * b[i], 0);
  function multiply(a, b) {
    const result = new Float32Array(16);
    for (let column = 0; column < 4; column++) {
      for (let row = 0; row < 4; row++) {
        for (let k = 0; k < 4; k++) result[column * 4 + row] += a[k * 4 + row] * b[column * 4 + k];
      }
    }
    return result;
  }
  function view(eye, target) {
    const z = normalize(eye.map((value, i) => value - target[i]));
    const x = normalize(cross([0, 1, 0], z)), y = cross(z, x);
    return new Float32Array([x[0], y[0], z[0], 0, x[1], y[1], z[1], 0, x[2], y[2], z[2], 0, -dot(x, eye), -dot(y, eye), -dot(z, eye), 1]);
  }
  function perspective(aspect, near, far) {
    const f = 1 / Math.tan(52 * Math.PI / 360), range = 1 / (near - far);
    return new Float32Array([f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, (far + near) * range, -1, 0, 0, 2 * far * near * range, 0]);
  }
  function materialColour(material) {
    let hash = 0;
    for (const character of material.name) hash = (hash * 31 + character.charCodeAt(0)) >>> 0;
    const palette = [[.56, .61, .65], [.52, .60, .44], [.64, .55, .43], [.56, .49, .47], [.48, .56, .60], [.64, .63, .52]];
    const colour = palette[hash % palette.length];
    const diffuse = Math.max(.3, Math.min(1, material.properties.ksDiffuse?.[0] ?? .7));
    return colour.map(value => value * (.55 + diffuse * .55));
  }
  async function response(url) {
    const result = await fetch(url, {cache: 'no-store'});
    if (!result.ok) throw new Error(`${url}: HTTP ${result.status}`);
    return result;
  }
  async function start() {
    const gl = canvas.getContext('webgl2', {antialias: true, preserveDrawingBuffer: true});
    if (!gl) throw new Error('WebGL 2 is unavailable. Enable browser hardware acceleration or use a supported browser.');
    function shader(type, text) {
      const value = gl.createShader(type);
      gl.shaderSource(value, text); gl.compileShader(value);
      if (!gl.getShaderParameter(value, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(value));
      return value;
    }
    const program = gl.createProgram();
    gl.attachShader(program, shader(gl.VERTEX_SHADER, `#version 300 es
      layout(location=0) in vec3 position;
      layout(location=1) in vec3 normal;
      layout(location=2) in vec2 uv;
      uniform mat4 world;
      uniform mat4 camera;
      out vec3 surfaceNormal;
      out vec2 surfaceUV;
      void main() {
        gl_Position = camera * world * vec4(position, 1.0);
        mat3 basis = mat3(world);
        surfaceNormal = abs(determinant(basis)) > 0.000001 ? transpose(inverse(basis)) * normal : normal;
        surfaceUV = uv;
      }`));
    gl.attachShader(program, shader(gl.FRAGMENT_SHADER, `#version 300 es
      precision highp float;
      in vec3 surfaceNormal;
      in vec2 surfaceUV;
      uniform vec3 colour;
      uniform bool selected;
      uniform sampler2D diffuseTexture;
      uniform bool useTexture;
      uniform bool textureEnabled;
      uniform float alphaCutoff;
      uniform float materialOpacity;
      uniform bool alphaEnabled;
      out vec4 pixel;
      void main() {
        vec3 n = length(surfaceNormal) > 0.00001 ? normalize(surfaceNormal) : vec3(0.0, 1.0, 0.0);
        if (!gl_FrontFacing) n = -n;
        float light = 0.48 + 0.52 * max(0.0, dot(n, normalize(vec3(0.4, 0.85, 0.3))));
        vec4 texel = useTexture && textureEnabled ? texture(diffuseTexture, surfaceUV) : vec4(colour, 1.0);
        float alpha = alphaEnabled ? texel.a * materialOpacity : 1.0;
        if (alpha < alphaCutoff || alpha < 0.001) discard;
        vec3 base = selected ? mix(texel.rgb, vec3(1.0, 0.15, 0.22), 0.65) : texel.rgb;
        pixel = vec4(base * light, alpha);
      }`));
    gl.linkProgram(program);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
    gl.useProgram(program);
    const uniforms = Object.fromEntries(['world', 'camera', 'colour', 'selected', 'diffuseTexture', 'useTexture', 'textureEnabled', 'alphaCutoff', 'materialOpacity', 'alphaEnabled'].map(name => [name, gl.getUniformLocation(program, name)]));
    gl.uniform1i(uniforms.diffuseTexture, 0);
    const scene = await (await response('/api/scene')).json();
    document.querySelector('#track').textContent = `${scene.track}${scene.layout ? ' / ' + scene.layout : ''}`;
    const stats = document.querySelector('#stats');
    for (const [name, value] of [['Visible meshes', scene.meshes.length], ['Triangles', scene.triangles], ['Materials', scene.materials.length], ['Geometry', `${(scene.geometry_bytes / 1048576).toFixed(1)} MiB`]]) {
      stats.append(el('dt', name), el('dd', value.toLocaleString()));
    }
    const diagnostics = document.querySelector('#diagnostics');
    document.querySelector('#diagnostic-title').textContent = `Diagnostics (${scene.diagnostics.length})${scene.has_errors ? ' — partial scene with errors' : ''}`;
    for (const item of scene.diagnostics) {
      const li = el('li', undefined, item.severity === 'error' ? 'error' : '');
      li.append(el('span', item.code, 'diagnostic-code'), document.createTextNode(`${item.message} [${item.source}${item.byte_offset !== undefined ? ', byte ' + item.byte_offset : item.line ? ':' + item.line : ''}]`));
      diagnostics.append(li);
    }
    const materials = document.querySelector('#materials');
    for (const material of scene.materials) {
      const details = el('details');
      details.append(el('summary', `${material.name} · ${material.shader}`), el('pre', JSON.stringify(material, null, 2)));
      materials.append(details);
    }
    const uploaded = new Map(), decoders = new Set();
    const forceSoftware = new URLSearchParams(location.search).has('software-textures');
    for (const info of scene.textures) {
      setStatus(`Loading local diffuse textures: ${uploaded.size}/${scene.textures.length}…`);
      try {
        const data = await (await response(`/textures/${info.id}.bin`)).arrayBuffer();
        const result = OPENASC_DDS.upload(gl, info, data, forceSoftware);
        uploaded.set(info.id, result.texture); decoders.add(result.decoder);
      } catch (error) {
        const li = el('li', undefined, 'error');
        li.append(el('span', 'texture_upload_failed', 'diagnostic-code'), document.createTextNode(`${info.name}: ${error.message} Diagnostic colour will be used.`));
        diagnostics.append(li);
      }
      await new Promise(requestAnimationFrame);
    }
    document.querySelector('#diagnostic-title').textContent = `Diagnostics (${diagnostics.children.length})${scene.has_errors ? ' — partial scene with errors' : ''}`;
    stats.append(el('dt', 'Diffuse textures'), el('dd', `${uploaded.size}/${scene.textures.length}`), el('dt', 'Texture path'), el('dd', [...decoders].join(', ') || 'Fallback colours'));
    canvas.dataset.textureCount = String(uploaded.size);
    canvas.dataset.textureDecoders = [...decoders].join(', ');
    const white = gl.createTexture(); gl.bindTexture(gl.TEXTURE_2D, white);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, 1, 1, 0, gl.RGBA, gl.UNSIGNED_BYTE, new Uint8Array([255, 255, 255, 255]));
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    setStatus('Loading local geometry into WebGL…');
    const bytes = await (await response('/geometry.bin')).arrayBuffer();
    if (bytes.byteLength !== scene.geometry_bytes) throw new Error('Geometry byte length does not match the scene manifest.');
    const meshes = scene.meshes.map((mesh, index) => {
      const vao = gl.createVertexArray(); gl.bindVertexArray(vao);
      const vertices = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, vertices);
      gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(bytes, mesh.vertex_offset, mesh.vertex_bytes / 4), gl.STATIC_DRAW);
      gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 44, 0);
      gl.enableVertexAttribArray(1); gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 44, 12);
      gl.enableVertexAttribArray(2); gl.vertexAttribPointer(2, 2, gl.FLOAT, false, 44, 24);
      const indices = gl.createBuffer(); gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indices);
      gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, new Uint16Array(bytes, mesh.index_offset, mesh.index_count), gl.STATIC_DRAW);
      const material = scene.materials[mesh.material];
      return {...mesh, index, vao, colour: materialColour(material), world: new Float32Array(mesh.world),
        texture: uploaded.get(material.preview_texture), materialDefinition: material};
    });
    let driveState = scene.drive ? {world: scene.drive.initial_world} : null;
    let followCar = Boolean(scene.drive);
    for (const mesh of meshes) if (mesh.dynamic === 'car') mesh.localWorld = mesh.world;
    if (gl.getError() !== gl.NO_ERROR) throw new Error('WebGL could not allocate the scene buffers.');
    gl.enable(gl.DEPTH_TEST); gl.depthFunc(gl.LEQUAL);
    gl.disable(gl.CULL_FACE); // Diagnostic preview is deliberately double-sided.
    let target = [0, 0, 0], distance = 100, yaw = .6, pitch = .85, selected = -1, pending = false, textureEnabled = true;
    const fullRadius = Math.max(1, length(scene.bounds[1].map((value, i) => value - scene.bounds[0][i])) / 2);
    const roads = scene.road_focus_bounds ? [{bounds: scene.road_focus_bounds}] : [];
    function releaseFollow() {
      if (followCar && driveState) {
        target = [...driveState.world.slice(12, 15)]; distance = 10;
        yaw = Math.atan2(driveState.world[8], driveState.world[10]) + Math.PI; pitch = .4;
      }
      followCar = false;
      document.querySelector('#drive-follow').setAttribute('aria-pressed', 'false');
    }
    function fit(values) {
      if (scene.drive && !document.querySelector('#drive-panel').hidden) releaseFollow();
      const low = [Infinity, Infinity, Infinity], high = [-Infinity, -Infinity, -Infinity];
      for (const mesh of values) {
        let bounds = mesh.bounds;
        if (mesh.dynamic === 'car' && driveState) {
          const p = driveState.world, points = [];
          for (const x of [0, 1]) for (const y of [0, 1]) for (const z of [0, 1]) {
            const point = [bounds[x][0], bounds[y][1], bounds[z][2]];
            points.push([0, 1, 2].map(i => p[i] * point[0] + p[4 + i] * point[1] + p[8 + i] * point[2] + p[12 + i]));
          }
          bounds = [0, 1].map(side => [0, 1, 2].map(i => (side ? Math.max : Math.min)(...points.map(point => point[i]))));
        }
        for (let axis = 0; axis < 3; axis++) {
          low[axis] = Math.min(low[axis], bounds[0][axis]);
          high[axis] = Math.max(high[axis], bounds[1][axis]);
        }
      }
      target = low.map((value, i) => (value + high[i]) / 2);
      distance = Math.max(.1, length(high.map((value, i) => value - low[i])) / 2) / Math.sin(52 * Math.PI / 360) * 1.15;
      invalidate();
    }
    function invalidate() {
      if (!pending) { pending = true; requestAnimationFrame(draw); }
    }
    function draw() {
      pending = false;
      const ratio = Math.min(devicePixelRatio || 1, 2);
      canvas.width = Math.max(1, Math.round(canvas.clientWidth * ratio));
      canvas.height = Math.max(1, Math.round(canvas.clientHeight * ratio));
      gl.viewport(0, 0, canvas.width, canvas.height);
      gl.clearColor(.065, .078, .10, 1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.useProgram(program);
      let eye = [target[0] + distance * Math.cos(pitch) * Math.sin(yaw), target[1] + distance * Math.sin(pitch), target[2] + distance * Math.cos(pitch) * Math.cos(yaw)];
      let cameraTarget = target;
      if (driveState) {
        const pose = driveState.world;
        if (followCar) {
          cameraTarget = [pose[12], pose[13] + .5, pose[14]];
          eye = [0, 1, 2].map(i => pose[12 + i] - pose[8 + i] * 8 + pose[4 + i] * 3);
        }
        for (const mesh of meshes) if (mesh.dynamic === 'car') {
          mesh.world = multiply(pose, mesh.localWorld);
          const center = mesh.bounds[0].map((v, i) => (v + mesh.bounds[1][i]) / 2);
          mesh.dynamicCenter = [0, 1, 2].map(i => pose[i] * center[0] + pose[4 + i] * center[1] + pose[8 + i] * center[2] + pose[12 + i]);
        }
      }
      const camera = multiply(perspective(canvas.width / canvas.height, Math.max(.02, (followCar ? 8 : distance) / 5000), Math.max(fullRadius * 12, distance * 4)), view(eye, cameraTarget));
      gl.uniformMatrix4fv(uniforms.camera, false, camera);
      gl.uniform1i(uniforms.textureEnabled, textureEnabled);
      const opaque = meshes.filter(mesh => mesh.materialDefinition.preview_blend !== 1);
      const transparent = meshes.filter(mesh => mesh.materialDefinition.preview_blend === 1);
      const meshDistance = mesh => length(mesh.bounds[0].map((value, axis) => (mesh.dynamicCenter?.[axis] ?? (value + mesh.bounds[1][axis]) / 2) - eye[axis]));
      transparent.sort((a, b) => meshDistance(b) - meshDistance(a));
      [...opaque, ...transparent].forEach(mesh => {
        const material = mesh.materialDefinition;
        if (material.preview_blend === 1) { gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA); }
        else gl.disable(gl.BLEND);
        if (material.preview_depth === 2) gl.disable(gl.DEPTH_TEST);
        else gl.enable(gl.DEPTH_TEST);
        gl.depthMask(material.preview_depth === 0);
        if (material.preview_depth === 1) { gl.enable(gl.POLYGON_OFFSET_FILL); gl.polygonOffset(-1, -1); }
        else gl.disable(gl.POLYGON_OFFSET_FILL);
        gl.activeTexture(gl.TEXTURE0); gl.bindTexture(gl.TEXTURE_2D, mesh.texture || white);
        gl.uniform1i(uniforms.useTexture, Boolean(mesh.texture));
        gl.uniform1f(uniforms.alphaCutoff, material.preview_alpha_test ? .5 : 0);
        gl.uniform1i(uniforms.alphaEnabled, material.preview_alpha_test || material.preview_blend === 1);
        gl.uniform1f(uniforms.materialOpacity, Math.max(0, Math.min(1, material.properties.alpha?.[0] ?? 1)));
        gl.bindVertexArray(mesh.vao);
        gl.uniformMatrix4fv(uniforms.world, false, mesh.world);
        gl.uniform3fv(uniforms.colour, mesh.colour);
        gl.uniform1i(uniforms.selected, mesh.index === selected);
        gl.drawElements(gl.TRIANGLES, mesh.index_count, gl.UNSIGNED_SHORT, 0);
      });
      gl.depthMask(true); gl.disable(gl.BLEND); gl.disable(gl.POLYGON_OFFSET_FILL);
      if (gl.getError() !== gl.NO_ERROR) {
        canvas.dataset.error = 'true'; setStatus('WebGL reported a drawing error.'); return;
      }
      canvas.dataset.ready = 'true'; canvas.dataset.drawCalls = String(meshes.length);
      canvas.dataset.carDrawCalls = String(meshes.filter(mesh => mesh.dynamic === 'car').length);
      canvas.dataset.textureMode = textureEnabled ? 'textured' : 'colours';
      setStatus(`${scene.has_errors ? 'Partial scene · ' : ''}${meshes.length} meshes · ${scene.triangles.toLocaleString()} triangles · ${textureEnabled ? uploaded.size + ' diffuse textures · approximate materials' : 'diagnostic colours'}`);
    }
    document.querySelector('#fit-all').addEventListener('click', () => fit(meshes));
    const roadButton = document.querySelector('#fit-road');
    roadButton.title = 'Focus bounds of active ROAD-named meshes, including non-renderable meshes. This is a name heuristic, not surface validation.';
    roadButton.addEventListener('click', () => fit(roads));
    document.querySelector('#top').addEventListener('click', () => { if (scene.drive) releaseFollow(); pitch = Math.PI / 2 - .001; invalidate(); });
    for (const button of document.querySelectorAll('.toolbar button')) button.disabled = false;
    roadButton.disabled = !roads.length;
    const textureToggle = document.querySelector('#texture-toggle');
    textureToggle.disabled = !uploaded.size;
    if (!uploaded.size) {
      textureEnabled = false;
      textureToggle.textContent = 'No textures';
      textureToggle.setAttribute('aria-pressed', 'false');
    }
    textureToggle.addEventListener('click', () => {
      textureEnabled = !textureEnabled;
      textureToggle.setAttribute('aria-pressed', String(textureEnabled));
      textureToggle.textContent = textureEnabled ? 'Textures on' : 'Textures off';
      invalidate();
    });
    function updateList() {
      const filter = document.querySelector('#filter').value.toLowerCase();
      const list = document.querySelector('#mesh-list'); list.replaceChildren();
      meshes.forEach((mesh, index) => {
        const material = scene.materials[mesh.material];
        if (!`${mesh.name} ${material.name}`.toLowerCase().includes(filter)) return;
        const button = el('button', `${mesh.name} · ${(mesh.index_count / 3).toLocaleString()} triangles`, 'mesh-button');
        button.type = 'button'; button.setAttribute('aria-pressed', String(index === selected));
        button.addEventListener('click', () => {
          selected = index; fit([mesh]); updateList();
          document.querySelector('#selection').textContent = `${mesh.name} · ${material.name} · ${material.shader} · ${mesh.model}`;
        });
        list.append(button);
      });
    }
    document.querySelector('#filter').addEventListener('input', updateList);
    updateList();
    let drag;
    canvas.addEventListener('contextmenu', event => event.preventDefault());
    canvas.addEventListener('pointerdown', event => {
      if (scene.drive) releaseFollow();
      drag = {x: event.clientX, y: event.clientY, pan: event.button !== 0 || event.shiftKey};
      canvas.setPointerCapture(event.pointerId); canvas.focus();
    });
    canvas.addEventListener('pointermove', event => {
      if (!drag) return;
      const dx = event.clientX - drag.x, dy = event.clientY - drag.y;
      if (drag.pan) {
        const scale = distance / Math.max(1, canvas.clientHeight);
        target[0] -= dx * Math.cos(yaw) * scale; target[2] += dx * Math.sin(yaw) * scale; target[1] += dy * scale;
      } else {
        yaw -= dx * .006; pitch = Math.max(-1.5, Math.min(1.56, pitch + dy * .006));
      }
      drag.x = event.clientX; drag.y = event.clientY; invalidate();
    });
    canvas.addEventListener('pointerup', () => { drag = null; });
    canvas.addEventListener('pointercancel', () => { drag = null; });
    canvas.addEventListener('lostpointercapture', () => { drag = null; });
    canvas.addEventListener('wheel', event => {
      if (scene.drive) releaseFollow();
      event.preventDefault(); distance = Math.max(.05, Math.min(fullRadius * 30, distance * Math.exp(event.deltaY * .001))); invalidate();
    }, {passive: false});
    canvas.addEventListener('keydown', event => {
      if (scene.drive && event.key.startsWith('Arrow')) return;
      if (event.key === 'ArrowLeft') yaw -= .1;
      else if (event.key === 'ArrowRight') yaw += .1;
      else if (event.key === 'ArrowUp') pitch = Math.min(1.56, pitch + .1);
      else if (event.key === 'ArrowDown') pitch = Math.max(-1.5, pitch - .1);
      else if (event.key === '+' || event.key === '=') distance = Math.max(.05, distance * .85);
      else if (event.key === '-') distance = Math.min(fullRadius * 30, distance / .85);
      else return;
      event.preventDefault(); invalidate();
    });
    canvas.addEventListener('webglcontextlost', event => { event.preventDefault(); canvas.dataset.error = 'true'; setStatus('WebGL context lost. Reload the page to restore the preview.'); });
    new ResizeObserver(invalidate).observe(canvas);
    fit(roads.length ? roads : meshes);
    if (scene.drive) await OPENASC_DRIVE.attach({canvas,
      onState: value => { driveState = value; invalidate(); },
      onFollow: value => { if (value) followCar = true; else releaseFollow(); invalidate(); }});
  }
  start().catch(error => { canvas.dataset.error = 'true'; setStatus(`Cannot display scene: ${error.message}`); console.error(error); });
})();
