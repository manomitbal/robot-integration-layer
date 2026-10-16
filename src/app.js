// ---------------------------------------------------------------------------
// APPLICATION — stands in for the orchestration core + operator console.
// It only uses ril.submit / ril.tasks / ril.robots[*].state / audit events.
// It never references an OEM, a controller name, or a ROS endpoint.
// ---------------------------------------------------------------------------
(function () {
  const $ = (s, el = document) => el.querySelector(s);
  const el = (tag, cls, txt) => { const e = document.createElement(tag); if (cls) e.className = cls; if (txt != null) e.textContent = txt; return e; };
  const sleep = ms => new Promise(r => setTimeout(r, ms));
  const parseXML = s => new DOMParser().parseFromString(s, 'text/xml');
  const DATA = window.ROBOT_DATA;
  const STATE_CLASS = { IDLE: 'ok', EXECUTING: 'active', LEASED: 'active', CONNECTING: 'muted', RECOVERING: 'warn', FAULTED: 'warn', OFFLINE: 'err', ESTOP: 'err' };
  const TASK = { skill: 'pick_place', object: 'part_1', from: 'station_a', to: 'station_b' };

  // ---- boot the integration layer ----------------------------------------
  const world = new RIL.Platform.World(RIL.Site.world);
  const ril = new RIL.Platform.RIL({ world, parseXML });
  const graphs = {}, traceLines = [];
  const trace = (id) => (dir, line) => { traceLines.push({ t: Date.now(), id, dir, line }); renderTrace(); };
  Object.keys(RIL.Manifests).forEach(id => {
    const m = RIL.Manifests[id];
    const kin = new RIL.Kin.KinModel(parseXML(DATA[id].urdf), m.description.root_link);
    const names = kin.chain(m.tool_frame.link).filter(j => j.type !== 'fixed').map(j => j.name);
    graphs[id] = new RIL.SimRos2.SimRobotGraph({ ns: '/' + id, vendor: m.oem === 'FANUC' ? 'fanuc' : 'ur', jointNames: names, q0: m.home, onTrace: trace(id) });
    ril.register({ manifest: m, urdf: DATA[id].urdf, site: RIL.Site.robots[id], adapter: new RIL.Adapters.byName[m.adapter]({ manifest: m, graph: graphs[id] }) });
  });

  // ---- 3D scene ------------------------------------------------------------
  const css = getComputedStyle(document.documentElement);
  const view = $('#view');
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio));
  renderer.outputEncoding = THREE.sRGBEncoding;
  view.appendChild(renderer.domElement);
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(38, 1, 0.05, 50);
  camera.up.set(0, 0, 1); camera.position.set(0.1, -2.6, 1.7);
  const controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0.3, 0.3); controls.enableDamping = true; controls.update();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x8a9096, 0.85));
  const sun = new THREE.DirectionalLight(0xffffff, 0.75); sun.position.set(1.5, -2, 3); scene.add(sun);
  const fill = new THREE.DirectionalLight(0xffffff, 0.25); fill.position.set(-2, 2, 1.5); scene.add(fill);

  const table = new THREE.Mesh(new THREE.BoxGeometry(2.0, 1.4, 0.04), new THREE.MeshStandardMaterial({ color: 0x9aa3a8, roughness: 0.9 }));
  table.position.set(0, 0.3, -0.02); scene.add(table);
  const grid = new THREE.GridHelper(2.0, 20, 0x7b858b, 0x8c959a); grid.rotation.x = Math.PI / 2; grid.position.set(0, 0.3, 0.001); scene.add(grid);
  Object.entries(world.stations).forEach(([k, p]) => {
    const pad = new THREE.Mesh(new THREE.BoxGeometry(0.16, 0.16, 0.004), new THREE.MeshStandardMaterial({ color: 0x3c4a57, roughness: 0.7 }));
    pad.position.set(p[0], p[1], 0.002); scene.add(pad);
  });
  const part = new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.05), new THREE.MeshStandardMaterial({ color: 0xe0a21a, roughness: 0.5 }));
  scene.add(part);

  function geometryFrom(g) {
    const bin = s => Uint8Array.from(atob(s), c => c.charCodeAt(0)).buffer;
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(bin(g.pos)), 3));
    geo.setIndex(new THREE.BufferAttribute(g.idx32 ? new Uint32Array(bin(g.idx)) : new Uint16Array(bin(g.idx)), 1));
    geo.computeVertexNormals();
    return geo;
  }
  const m4 = (T) => new THREE.Matrix4().set(...T);
  const robots3d = {};
  Object.values(ril.robots).forEach(r => {
    const links = {};
    Object.values(r.kin.links).forEach(L => {
      const grp = new THREE.Group(); grp.matrixAutoUpdate = false;
      L.visuals.forEach(v => {
        (DATA[r.id].meshes[v.mesh] || []).forEach(g => {
          const mesh = new THREE.Mesh(geometryFrom(g), new THREE.MeshStandardMaterial({ color: new THREE.Color(g.color).convertSRGBToLinear(), roughness: 0.55, metalness: 0.1 }));
          mesh.matrixAutoUpdate = false;
          mesh.matrix.copy(m4(v.T)).multiply(new THREE.Matrix4().makeScale(...v.scale));
          grp.add(mesh);
        });
      });
      scene.add(grp); links[L.name] = grp;
    });
    // generic parallel gripper, oriented by the manifest's approach axis
    const tf = r.manifest.tool_frame, grip = new THREE.Group();
    grip.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), new THREE.Vector3(...tf.approach));
    const dark = new THREE.MeshStandardMaterial({ color: 0x2b3238, roughness: 0.6 });
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.075, 0.045, 0.08), dark); body.position.z = 0.04; grip.add(body);
    const fingers = [-1, 1].map(s => { const f = new THREE.Mesh(new THREE.BoxGeometry(0.012, 0.03, 0.075), dark); f.position.set(s * 0.045, 0, 0.1175); grip.add(f); return f; });
    links[tf.link].add(grip);
    const ring = new THREE.Mesh(new THREE.RingGeometry(0.17, 0.2, 48), new THREE.MeshBasicMaterial({ color: 0x2e7d4f, transparent: true, opacity: 0.85 }));
    ring.position.set(...RIL.Site.robots[r.id].xyz); ring.position.z = 0.003; scene.add(ring);
    robots3d[r.id] = { links, fingers, ring };
  });

  // ---- labels projected from 3D --------------------------------------------
  const labelLayer = $('#labels');
  const labels = {};
  Object.values(ril.robots).forEach(r => {
    const d = el('div', 'tag'); d.append(el('span', 'tag-name', r.manifest.display), el('span', 'pill', ''));
    labelLayer.append(d); labels[r.id] = d;
  });
  Object.keys(world.stations).forEach(k => { const d = el('div', 'tag station', k === 'station_a' ? 'Station A' : 'Station B'); labelLayer.append(d); labels[k] = d; });
  function place(div, p) {
    const v = new THREE.Vector3(...p).project(camera), w = view.clientWidth, h = view.clientHeight;
    div.style.transform = `translate(-50%,-100%) translate(${(v.x + 1) / 2 * w}px, ${(1 - v.y) / 2 * h}px)`;
    div.style.visibility = v.z < 1 ? 'visible' : 'hidden';
  }

  const ringColor = { ok: 0x2e7d4f, active: 0x2457a6, warn: 0xd9a400, err: 0xb3261e, muted: 0x8a949b };
  function frame() {
    const w = view.clientWidth, h = view.clientHeight;
    if (renderer.domElement.width !== Math.floor(w * renderer.getPixelRatio())) { renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
    Object.values(ril.robots).forEach(r => {
      const T = r.kin.fk(graphs[r.id].readJointStates(), r.baseT), R3 = robots3d[r.id];
      Object.entries(R3.links).forEach(([n, g]) => { if (T[n]) { g.matrix.copy(m4(T[n])); g.matrixWorldNeedsUpdate = true; } });
      const io = graphs[r.id].ioOut; const closed = !!(io[1] || io[0]);
      R3.fingers.forEach((f, i) => { const tgt = (i ? 1 : -1) * (closed ? 0.031 : 0.045); f.position.x += (tgt - f.position.x) * 0.25; });
      R3.ring.material.color.setHex(ringColor[STATE_CLASS[r.state]] || ringColor.muted);
      const base = RIL.Site.robots[r.id].xyz; place(labels[r.id], [base[0], base[1] - 0.25, 1.05]);
    });
    const o = world.objects.part_1;
    if (o.heldBy) {
      const r = ril.robots[o.heldBy], T = r.kin.fk(graphs[r.id].readJointStates(), r.baseT)[r.manifest.tool_frame.link];
      part.position.set(...RIL.Kin.apply(T, r.manifest.tool_frame.offset));
    } else part.position.set(...o.pos);
    Object.entries(world.stations).forEach(([k, p]) => place(labels[k], [p[0], p[1] + 0.12, 0.02]));
    controls.update(); renderer.render(scene, camera);
    requestAnimationFrame(frame);
  }

  // ---- UI state ------------------------------------------------------------
  let selected = 'auto', direction = 'ab', activeTask = null, scenarioBusy = false;
  const taskSpec = () => direction === 'ab' ? { ...TASK } : { ...TASK, from: 'station_b', to: 'station_a' };

  function renderCode() {
    const s = taskSpec();
    $('#code').textContent = `ril.submit(\n  { skill: '${s.skill}', object: '${s.object}',\n    from: '${s.from}', to: '${s.to}' },\n  { robot: '${selected}' }\n)`;
  }
  document.querySelectorAll('[data-robot]').forEach(b => b.addEventListener('click', () => {
    selected = b.dataset.robot; document.querySelectorAll('[data-robot]').forEach(x => x.setAttribute('aria-pressed', x === b)); renderCode();
  }));
  $('#dir').addEventListener('change', e => { direction = e.target.value; renderCode(); });

  function syncDirectionToPart() {
    const o = world.objects.part_1; if (o.heldBy) return;
    const atA = Math.hypot(o.pos[0] - world.stations.station_a[0], o.pos[1] - world.stations.station_a[1]) < 0.01;
    direction = atA ? 'ab' : 'ba'; $('#dir').value = direction; renderCode();
  }

  async function runTask(robot) {
    const t = ril.submit(taskSpec(), { robot });
    activeTask = t; renderAll();
    await t.done; renderAll(); syncDirectionToPart();
    return t;
  }
  $('#run').addEventListener('click', () => { if (!busy()) runTask(selected); });
  const busy = () => scenarioBusy || (activeTask && !['SUCCEEDED', 'FAILED', 'CANCELED'].includes(activeTask.state));

  function resetCell() {
    world.reset(); syncDirectionToPart();
    Object.values(graphs).forEach(g => { g.restoreLink(); });
    Object.values(ril.robots).forEach(r => { if (r.failure === 'PROTECTIVE_STOP') ril.operatorRecover(r.id); });
  }
  $('#reset').addEventListener('click', () => { if (!busy()) resetCell(); });

  const say = (msg) => { $('#narration').textContent = msg; };
  const scenarios = {
    async pick() {
      say(`Running pick and place on ${selected === 'auto' ? 'whichever arm the platform allocates' : ril.robots[selected].manifest.display}.`);
      const t = await runTask(selected); say(`${t.id} ${t.state.toLowerCase()} on ${t.robot}.`);
    },
    async switch() {
      resetCell(); await sleep(300);
      say('Same task, robot: crx10ia. Only the robot argument changes between runs.');
      setSel('crx10ia'); const a = await runTask('crx10ia');
      say(`FANUC run ${a.state.toLowerCase()}. Resetting the part and re-running on robot: ur10e.`);
      await sleep(900); world.reset(); syncDirectionToPart(); setSel('ur10e');
      const b = await runTask('ur10e');
      say(`Both runs ${a.state === 'SUCCEEDED' && b.state === 'SUCCEEDED' ? 'succeeded' : 'finished'} through one call path. Compare the two vendors in the adapter traffic panel.`);
    },
    async failover() {
      resetCell(); await sleep(300);
      setSel('crx10ia');
      say('Starting on the FANUC arm. Its connection drops early in the approach.');
      const p = runTask('crx10ia');
      while (!activeTask || activeTask.current !== 'approach') await sleep(30);
      await sleep(700); graphs.crx10ia.injectLinkLoss();
      say(`FANUC link lost. The platform waits ${RIL.Platform.LIVENESS_TIMEOUT_MS / 1000} s for heartbeat before declaring it offline, then checks the cell is clear and fails over.`);
      const t = await p;
      say(t.state === 'SUCCEEDED' ? `Recovered: ${t.id} completed on ${t.robot} after failover from crx10ia. Restore the FANUC link from its card.` : `${t.id} ended ${t.state}.`);
    },
  };
  function setSel(id) { selected = id; document.querySelectorAll('[data-robot]').forEach(x => x.setAttribute('aria-pressed', x.dataset.robot === id)); renderCode(); }
  document.querySelectorAll('[data-scenario]').forEach(b => b.addEventListener('click', async () => {
    if (busy()) return; scenarioBusy = true; renderAll();
    try { await scenarios[b.dataset.scenario](); } finally { scenarioBusy = false; renderAll(); }
  }));

  // ---- robot cards ------------------------------------------------------------
  const cards = {};
  Object.values(ril.robots).forEach(r => {
    const m = r.manifest, c = el('article', 'robot');
    c.innerHTML = `
      <header><div><h3>${m.display}</h3><p class="sub">${m.oem} · ${m.adapter}</p></div><span class="pill"></span></header>
      <p class="detail"></p>
      <dl>
        <div><dt>Reach</dt><dd>${m.limits.reach_m} m</dd></div>
        <div><dt>Payload</dt><dd>${m.limits.payload_kg} kg</dd></div>
        <div><dt>Heartbeat</dt><dd class="hb">–</dd></div>
        <div><dt>Driver</dt><dd>${m.transport.driver}</dd></div>
      </dl>
      <div class="row">
        <button class="ghost" data-act="link">Drop connection</button>
        <button class="ghost" data-act="pstop">Protective stop</button>
        <button class="ghost" data-act="recover">Recover</button>
      </div>`;
    c.querySelector('[data-act=link]').addEventListener('click', () => { const g = graphs[r.id]; g.linkUp ? g.injectLinkLoss() : g.restoreLink(); renderAll(); });
    c.querySelector('[data-act=pstop]').addEventListener('click', () => { graphs[r.id].injectProtectiveStop(); });
    c.querySelector('[data-act=recover]').addEventListener('click', () => ril.operatorRecover(r.id));
    $('#robots').append(c); cards[r.id] = c;
  });

  function renderAll() {
    const now = Date.now();
    Object.values(ril.robots).forEach(r => {
      const c = cards[r.id], cls = STATE_CLASS[r.state] || 'muted';
      [c.querySelector('.pill'), labels[r.id].querySelector('.pill')].forEach(p => { p.textContent = r.state; p.className = 'pill ' + cls; });
      c.querySelector('.detail').textContent = r.failure ? `${r.failure}${r.detail ? ': ' + r.detail : ''}` : (r.task ? `Running ${r.task}` : 'Ready');
      c.querySelector('.detail').className = 'detail ' + (r.failure ? cls : '');
      const age = (now - r.lastSeen) / 1000;
      c.querySelector('.hb').textContent = graphs[r.id].linkUp ? `${Math.max(0, age).toFixed(1)} s ago` : `${age.toFixed(1)} s ago`;
      c.querySelector('[data-act=link]').textContent = graphs[r.id].linkUp ? 'Drop connection' : 'Restore connection';
      c.querySelector('[data-act=recover]').disabled = !(r.failure && r.failure !== 'CONNECTIVITY');
    });
    $('#run').disabled = busy(); $('#reset').disabled = busy();
    document.querySelectorAll('[data-scenario]').forEach(b => { b.disabled = busy(); });
    // task progress
    const t = activeTask, steps = $('#steps');
    steps.replaceChildren();
    if (t) {
      $('#task-title').textContent = `${t.id} · ${t.robot || 'allocating'} · ${t.state}`;
      $('#task-title').className = 'task-title ' + ({ SUCCEEDED: 'ok', FAILED: 'err', CANCELED: 'err', BLOCKED_ON_OPERATOR: 'err', PREEMPTED: 'warn' }[t.state] || 'active');
      (t.steps || []).forEach((s, i) => {
        const li = el('li', i < t.step ? 'done' : (i === t.step && t.current ? 'now' : ''), s.name); steps.append(li);
      });
      const op = $('#operator');
      op.hidden = t.state !== 'BLOCKED_ON_OPERATOR';
      if (!op.hidden) $('#operator-msg').textContent = t.held
        ? `${t.robot} stopped while holding ${t.spec.object}. Restore the robot, then resume, or abort and clear the part by hand.`
        : `${t.robot} stopped inside the shared workspace, so the platform will not send another arm in. Restore it and resume, or abort.`;
    }
  }
  $('#op-resume').addEventListener('click', () => activeTask && ril.operatorDecision(activeTask.id, 'resume'));
  $('#op-abort').addEventListener('click', () => activeTask && ril.operatorDecision(activeTask.id, 'abort'));

  // ---- event + trace panes -------------------------------------------------------
  const fmt = t => new Date(t).toLocaleTimeString([], { hour12: false }) + '.' + String(t % 1000).padStart(3, '0').slice(0, 1);
  ril.audit.on(e => {
    const li = el('li', 'lvl-' + e.level);
    li.append(el('time', '', fmt(e.t)), el('span', 'etype', e.type), el('span', 'emsg', e.msg));
    const ul = $('#events'); ul.prepend(li); while (ul.children.length > 200) ul.lastChild.remove();
    renderAll();
  });
  let traceQueued = false;
  function renderTrace() {
    if (traceQueued) return; traceQueued = true;
    requestAnimationFrame(() => {
      traceQueued = false; const ul = $('#trace'); ul.replaceChildren();
      traceLines.slice(-120).reverse().forEach(x => {
        const li = el('li', 'oem-' + x.id);
        li.append(el('time', '', fmt(x.t)), el('span', 'who', x.id), el('span', 'dir', x.dir), el('span', 'line', x.line));
        ul.append(li);
      });
    });
  }
  ril.onChange(() => renderAll());
  setInterval(renderAll, 250);
  renderCode(); renderAll(); requestAnimationFrame(frame);
})();
