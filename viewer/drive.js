/* Keyboard controller and telemetry; simulation remains in Python at 120 Hz. */
window.OPENASC_DRIVE = {
  async attach({canvas, onState, onFollow}) {
    const panel = document.querySelector('#drive-panel');
    panel.hidden = false;
    document.querySelector('#drive-hud').hidden = false;
    const state = await (await fetch('/api/drive', {cache: 'no-store'})).json();
    const token = state.token;
    let sequence = state.sequence, gear = state.gear, current = state;
    let queue = Promise.resolve(), inputPending = false, following = true, connected = true;
    const keys = new Set();
    const status = document.querySelector('#drive-status');
    const toggle = document.querySelector('#drive-toggle');
    const follow = document.querySelector('#drive-follow');
    document.querySelector('#drive-car').textContent = `${state.car} · first drive`;
    document.querySelector('.header-label').textContent = 'LOCAL FIRST-DRIVE PROTOTYPE';
    document.querySelector('.inspector > h1').textContent = 'First drive';
    document.querySelector('.inspector > .notice').textContent = 'Approximate diffuse materials and independent prototype dynamics. Partial scene/catalog errors remain visible below. Original simulator behavior is not reproduced.';
    function show(value) {
      if (value.sequence < current.sequence || (value.sequence === current.sequence && value.tick < current.tick)) return;
      current = value;
      canvas.dataset.driveTick = String(value.tick);
      canvas.dataset.driveSpeed = String(value.speed_kmh);
      canvas.dataset.drivePaused = String(value.paused);
      document.querySelector('#drive-speed').textContent = `${value.speed_kmh.toFixed(0)} km/h`;
      document.querySelector('#drive-gear').textContent = value.gear === -1 ? 'R' : value.gear === 0 ? 'N' : String(value.gear);
      document.querySelector('#drive-rpm').textContent = `${value.rpm.toFixed(0)} rpm`;
      document.querySelector('#drive-hud').textContent = `openASC · prototype · ${value.speed_kmh.toFixed(0)} km/h · ${value.gear === -1 ? 'R' : value.gear === 0 ? 'N' : value.gear}`;
      status.textContent = value.paused ? value.reason : `Running · 120 Hz · ${value.simulation_seconds.toFixed(1)} s${value.lag_events ? ' · clock lag recorded' : ''}`;
      toggle.textContent = value.paused ? 'Resume' : 'Pause';
      onState(value);
    }
    function enqueue(action, controls) {
      const run = async () => {
        const result = await fetch('/api/drive', {method: 'POST', headers: {'Content-Type': 'application/json'},
          body: JSON.stringify({token, sequence: ++sequence, action, ...(controls ? {controls} : {})}),
          signal: AbortSignal.timeout(2000)});
        const value = await result.json();
        if (!result.ok) throw new Error(value.diagnostics?.[0]?.message || `Controller HTTP ${result.status}`);
        if (action === 'reset') gear = value.gear;
        show(value);
      };
      queue = queue.then(run).catch(error => {
        keys.clear(); gear = current.gear;
        status.textContent = `Controller: ${error.message}`;
      });
      return queue;
    }
    const controls = () => ({throttle: keys.has('KeyW') || keys.has('ArrowUp') ? 1 : 0,
      brake: keys.has('KeyS') || keys.has('ArrowDown') ? 1 : 0,
      steer: Number(keys.has('KeyA') || keys.has('ArrowLeft')) - Number(keys.has('KeyD') || keys.has('ArrowRight')),
      handbrake: keys.has('Space') ? 1 : 0, gear});
    function pause() { keys.clear(); enqueue('pause'); }
    function reset() { keys.clear(); enqueue('reset'); }
    function togglePause() { keys.clear(); enqueue(current.paused ? 'resume' : 'pause'); canvas.focus(); }
    toggle.addEventListener('click', togglePause);
    document.querySelector('#drive-reset').addEventListener('click', reset);
    follow.addEventListener('click', () => {
      following = follow.getAttribute('aria-pressed') !== 'true';
      follow.setAttribute('aria-pressed', String(following));
      onFollow(following);
    });
    canvas.addEventListener('keydown', event => {
      if (!['KeyW', 'KeyS', 'KeyA', 'KeyD', 'Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'KeyE', 'KeyQ', 'KeyP', 'KeyR'].includes(event.code)) return;
      event.preventDefault();
      if (event.repeat) return;
      if (event.code === 'KeyE') gear = Math.min(state.gears, gear + 1);
      else if (event.code === 'KeyQ') gear = Math.max(-1, gear - 1);
      else if (event.code === 'KeyP') togglePause();
      else if (event.code === 'KeyR') reset();
      else keys.add(event.code);
    });
    window.addEventListener('keyup', event => keys.delete(event.code));
    window.addEventListener('blur', pause);
    document.addEventListener('visibilitychange', () => { if (document.hidden) pause(); });
    window.addEventListener('pagehide', pause);
    show(state);
    setInterval(() => {
      if (inputPending || document.hidden || !connected) return;
      inputPending = true;
      enqueue('input', controls()).finally(() => { inputPending = false; });
    }, 50);
    async function poll() {
      try {
        const result = await fetch('/api/drive', {cache: 'no-store', signal: AbortSignal.timeout(2000)});
        if (!result.ok) throw new Error(`Telemetry HTTP ${result.status}`);
        connected = true;
        show(await result.json());
      } catch (error) {
        connected = false;
        keys.clear(); enqueue('pause');
        status.textContent = `Telemetry unavailable: ${error.message}. Pausing; the input watchdog also protects against connection loss.`;
      } finally { setTimeout(poll, 33); }
    }
    poll();
  }
};
