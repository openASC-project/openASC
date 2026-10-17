/* Original bounded DDS pixel decoding and optional S3TC upload for the local viewer. */
((root) => {
  'use strict';
  function levelView(info, buffer, index) {
    const level = info.levels[index];
    if (!(buffer instanceof ArrayBuffer) || !level || !Number.isInteger(level.width) || !Number.isInteger(level.height) ||
        level.width < 1 || level.height < 1 || level.width > 8192 || level.height > 8192 || level.width * level.height > 16777216 ||
        !Number.isInteger(level.offset) || !Number.isInteger(level.length) || level.offset < 0 || level.length < 0 || level.offset + level.length > buffer.byteLength) {
      throw new Error('Invalid DDS level bounds.');
    }
    return {level, bytes: new Uint8Array(buffer, level.offset, level.length)};
  }
  function colour565(value) {
    const r = value >>> 11, g = (value >>> 5) & 63, b = value & 31;
    return [(r << 3) | (r >>> 2), (g << 2) | (g >>> 4), (b << 3) | (b >>> 2), 255];
  }
  function decodeLevel(info, buffer, index = 0) {
    const {level, bytes} = levelView(info, buffer, index);
    const width = level.width, height = level.height;
    const output = new Uint8Array(width * height * 4);
    if (['BC1', 'BC2', 'BC3'].includes(info.format)) {
      const stride = info.format === 'BC1' ? 8 : 16;
      if (bytes.length !== Math.ceil(width / 4) * Math.ceil(height / 4) * stride) throw new Error('Invalid compressed DDS block length.');
      const data = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      let offset = 0;
      for (let by = 0; by < height; by += 4) {
        for (let bx = 0; bx < width; bx += 4, offset += stride) {
          const colourOffset = offset + (stride === 16 ? 8 : 0);
          const c0 = data.getUint16(colourOffset, true), c1 = data.getUint16(colourOffset + 2, true);
          const palette = [colour565(c0), colour565(c1)];
          if (info.format === 'BC1' && c0 <= c1) {
            palette.push(palette[0].map((value, channel) => channel === 3 ? 255 : Math.floor((value + palette[1][channel]) / 2)), [0, 0, 0, 0]);
          } else {
            palette.push(palette[0].map((value, channel) => channel === 3 ? 255 : Math.floor((2 * value + palette[1][channel]) / 3)));
            palette.push(palette[0].map((value, channel) => channel === 3 ? 255 : Math.floor((value + 2 * palette[1][channel]) / 3)));
          }
          const colours = data.getUint32(colourOffset + 4, true);
          let alphaValues, alphaBits;
          if (info.format === 'BC3') {
            const a = bytes[offset], b = bytes[offset + 1];
            alphaValues = [a, b];
            const steps = a > b ? 7 : 5;
            for (let j = 1; j < steps; j++) alphaValues.push(Math.floor(((steps - j) * a + j * b) / steps));
            if (steps === 5) alphaValues.push(0, 255);
            alphaBits = 0;
            for (let j = 0; j < 6; j++) alphaBits += bytes[offset + 2 + j] * 2 ** (8 * j);
          }
          for (let k = 0; k < 16; k++) {
            const x = bx + (k & 3), y = by + (k >>> 2);
            if (x >= width || y >= height) continue;
            const pixel = (y * width + x) * 4, colour = palette[(colours >>> (k * 2)) & 3];
            output.set(colour, pixel);
            if (info.format === 'BC2') output[pixel + 3] = ((bytes[offset + (k >>> 1)] >>> ((k & 1) * 4)) & 15) * 17;
            if (info.format === 'BC3') output[pixel + 3] = alphaValues[Math.floor(alphaBits / 2 ** (k * 3)) % 8];
          }
        }
      }
    } else if (info.format === 'RGB' || info.format === 'LUMINANCE') {
      if (![8, 16, 24, 32].includes(info.bits) || !Array.isArray(info.masks) || info.masks.length !== 4) throw new Error('Invalid DDS raw format.');
      const stride = info.bits / 8, pitch = level.row_pitch;
      if (!Number.isInteger(pitch) || pitch < width * stride || pitch * height !== bytes.length) throw new Error('Invalid DDS row pitch.');
      const channels = info.masks.map(mask => {
        if (!mask) return null;
        const low = (mask & -mask) >>> 0, shift = Math.log2(low), maximum = mask >>> shift;
        return value => Math.round(((value & mask) >>> shift) * 255 / maximum);
      });
      for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
          const start = y * pitch + x * stride, pixel = (y * width + x) * 4;
          let packed = 0;
          for (let b = 0; b < stride; b++) packed += bytes[start + b] * 2 ** (8 * b);
          const red = channels[0] ? channels[0](packed) : 0;
          output[pixel] = red;
          output[pixel + 1] = info.format === 'LUMINANCE' ? red : channels[1](packed);
          output[pixel + 2] = info.format === 'LUMINANCE' ? red : channels[2](packed);
          output[pixel + 3] = channels[3] ? channels[3](packed) : 255;
        }
      }
    } else throw new Error(`Unsupported DDS format ${info.format}.`);
    return output;
  }
  function upload(gl, info, buffer, forceSoftware = false) {
    if (Math.max(info.width, info.height) > gl.getParameter(gl.MAX_TEXTURE_SIZE)) throw new Error('Texture exceeds this GPU dimension limit.');
    const extension = !forceSoftware && gl.getExtension('WEBGL_compressed_texture_s3tc');
    const compatible = info.levels.every(level => [level.width, level.height].every(size => size % 4 === 0 || size === 1 || size === 2));
    const compressed = extension && compatible && ['BC1', 'BC2', 'BC3'].includes(info.format);
    const texture = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
    try {
      info.levels.forEach((level, index) => {
        if (compressed) {
          const formats = {BC1: extension.COMPRESSED_RGBA_S3TC_DXT1_EXT, BC2: extension.COMPRESSED_RGBA_S3TC_DXT3_EXT, BC3: extension.COMPRESSED_RGBA_S3TC_DXT5_EXT};
          gl.compressedTexImage2D(gl.TEXTURE_2D, index, formats[info.format], level.width, level.height, 0, levelView(info, buffer, index).bytes);
        } else {
          gl.texImage2D(gl.TEXTURE_2D, index, gl.RGBA8, level.width, level.height, 0, gl.RGBA, gl.UNSIGNED_BYTE, decodeLevel(info, buffer, index));
        }
      });
      if (info.generate_mips) gl.generateMipmap(gl.TEXTURE_2D);
      else gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAX_LEVEL, info.levels.length - 1);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, info.generate_mips || info.levels.length > 1 ? gl.LINEAR_MIPMAP_LINEAR : gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.REPEAT);
      if (gl.getError() !== gl.NO_ERROR) throw new Error('GPU texture upload failed.');
      return {texture, decoder: compressed ? 'S3TC' : 'RGBA software'};
    } catch (error) {
      gl.deleteTexture(texture);
      for (let i = 0; i < 8 && gl.getError() !== gl.NO_ERROR; i++) { /* Clear errors before a bounded fallback attempt. */ }
      if (compressed) return upload(gl, info, buffer, true);
      throw error;
    }
  }
  const api = {decodeLevel, upload};
  if (typeof module !== 'undefined' && module.exports) module.exports = api;
  else root.OPENASC_DDS = api;
})(globalThis);
