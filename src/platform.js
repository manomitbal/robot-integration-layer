// ---------------------------------------------------------------------------
// SHARED PLATFORM — everything here is OEM-agnostic and is written once.
// Registry · health/liveness · capability matching · skill composition ·
// execution with local recovery, failover and operator escalation · audit.
// ---------------------------------------------------------------------------
RIL.Platform = (() => {
  const C = RIL.Contract, K = RIL.Kin;
  const RS = C.RobotState, TS = C.TaskState;
  const LIVENESS_TIMEOUT_MS = 3000, HEALTH_PERIOD_MS = 500;
  const MAX_LOCAL_RECOVERIES = 2, RECOVERY_WINDOW_MS = 10 * 60 * 1000;
  const HOVER_M = 0.15, SPEED_FACTOR = 0.45, CLEARANCE_M = 0.30;

  class Audit {
    constructor() { this.events = []; this.subs = []; }
    emit(e) { const ev = { t: Date.now(), level: 'info', ...e }; this.events.push(ev); this.subs.forEach(f => f(ev)); return ev; }
    on(f) { this.subs.push(f); }
  }

  class RIL_ {
    constructor({ world, parseXML }) {
      this.robots = {}; this.tasks = {}; this.world = world; this.parseXML = parseXML;
      this.audit = new Audit(); this.listeners = []; this._seq = 0;
      this._hm = setInterval(() => this._healthTick(), HEALTH_PERIOD_MS);
    }
    onChange(f) { this.listeners.push(f); }
    _changed() { this.listeners.forEach(f => f()); }

    // ---- registry: adding a robot = manifest + URDF + adapter class + site pose
    register({ manifest, urdf, adapter, site }) {
      const kin = new K.KinModel(this.parseXML(urdf), manifest.description.root_link);
      const baseT = K.fromXYZRPY(site.xyz, site.rpy);
      const r = { id: manifest.id, manifest, kin, baseT, site, adapter, state: RS.CONNECTING, failure: null, detail: '', lastSeen: Date.now(), task: null, recoveries: [], watch: [] };
      this.robots[r.id] = r;
      adapter.connect().then((c) => {
        if (c && c.ok === false) { r.state = RS.FAULTED; r.failure = 'OEM_INTERNAL'; r.detail = `connect failed at ${c.failed}`; this.audit.emit({ type: 'robot.connect_failed', level: 'error', robot: r.id, msg: `${r.id} handshake failed at ${c.failed}` }); }
        else { r.state = RS.IDLE; this.audit.emit({ type: 'robot.registered', robot: r.id, msg: `${manifest.display} registered (${manifest.oem}, adapter ${manifest.adapter})` }); }
        this._changed();
      });
      return r;
    }

    // Live world transform of a robot's root link. Fixed robots use the site
    // pose; mobile robots follow the adapter's odometry.
    baseT(robotId) {
      const r = this.robots[robotId], mob = r.manifest.mobility;
      if (!mob) return r.baseT;
      const b = r.adapter.basePose();
      return K.fromXYZRPY([b.x, b.y, mob.body_height_m], [0, 0, b.yaw]);
    }

    // ---- health & liveness ------------------------------------------------
    _healthTick() {
      const now = Date.now();
      Object.values(this.robots).forEach(r => {
        if (r.state === RS.CONNECTING) return;   // adapter is mid-handshake (e.g. lease, power-on)
        const h = r.adapter.health();
        if (h.alive) {
          if (r.state === RS.OFFLINE) this.audit.emit({ type: 'robot.reconnected', robot: r.id, msg: `${r.id} heartbeat restored` });
          r.lastSeen = now;
          if (r.state === RS.RECOVERING) return;
          if (h.failure) {
            if (r.state !== RS.FAULTED && r.state !== RS.ESTOP) this.audit.emit({ type: 'robot.fault', level: 'warn', robot: r.id, msg: `${r.id} → ${h.failure}`, data: { oem: h.detail } });
            r.state = h.state; r.failure = h.failure; r.detail = h.detail || '';
          } else {
            r.failure = null; r.detail = '';
            r.state = r.task ? RS.EXECUTING : RS.IDLE;
          }
        } else if (now - r.lastSeen > LIVENESS_TIMEOUT_MS && r.state !== RS.OFFLINE) {
          r.state = RS.OFFLINE; r.failure = 'CONNECTIVITY'; r.detail = `no heartbeat for ${LIVENESS_TIMEOUT_MS / 1000}s`;
          this.audit.emit({ type: 'robot.offline', level: 'error', robot: r.id, msg: `${r.id} OFFLINE — no heartbeat for ${LIVENESS_TIMEOUT_MS / 1000} s` });
          r.watch.forEach(f => f('CONNECTIVITY'));
        }
      });
      this._changed();
    }

    // ---- capability matching + skill composition ---------------------------
    plan(robotId, spec) {
      const r = this.robots[robotId], m = r.manifest, w = this.world;
      const req = C.Skills[spec.skill];
      if (!req) return { ok: false, failure: 'UNSUPPORTED', reason: `unknown skill ${spec.skill}` };
      if (!m.skills.includes(spec.skill)) return { ok: false, failure: 'UNSUPPORTED', reason: `${m.id} does not declare ${spec.skill}` };
      if (!m.tools.some(t => req.requires.tool.includes(t.type))) return { ok: false, failure: 'UNSUPPORTED', reason: `no ${req.requires.tool.join(' / ')}` };
      const obj = w.objects[spec.object];
      if (obj.mass_kg > m.limits.payload_kg) return { ok: false, failure: 'PAYLOAD_EXCEEDED', reason: `${obj.mass_kg} kg > ${m.limits.payload_kg} kg` };
      const src = obj.pos.slice(), dst = w.stations[spec.to].slice();
      dst[2] = src[2];
      const tool = m.tool_frame, down = [0, 0, -1], mob = m.mobility;
      const poses = [
        ...(mob ? [{ name: 'walk_to_pick', nav: src }] : []),
        { name: 'approach', p: [src[0], src[1], src[2] + HOVER_M] },
        { name: 'descend', p: src },
        { name: 'grasp', gripper: true },
        { name: 'lift', p: [src[0], src[1], src[2] + HOVER_M] },
        ...(mob ? [{ name: 'walk_to_place', nav: dst }] : []),
        { name: 'transfer', p: [dst[0], dst[1], dst[2] + HOVER_M] },
        { name: 'place', p: dst },
        { name: 'release', gripper: false },
        { name: 'retreat', p: [dst[0], dst[1], dst[2] + HOVER_M] },
        { name: 'home', q: m.home },
        ...(mob ? [{ name: 'return_to_dock', dock: true }] : []),
      ];
      // Mobile: each arm pose is solved from the stand pose the base will be at by then.
      let base = mob && r.adapter.basePose(), baseT = mob ? null : r.baseT;
      const setBase = (b) => { base = b; baseT = K.fromXYZRPY([b.x, b.y, mob.body_height_m], [0, 0, b.yaw]); };
      if (mob) setBase(base);
      let q = r.adapter.jointState();
      const steps = [];
      for (const s of poses) {
        if (s.gripper !== undefined) { steps.push({ name: s.name, kind: 'gripper', close: s.gripper }); continue; }
        if (s.q) { q = Object.assign({}, s.q); steps.push({ name: s.name, kind: 'move_joints', q }); continue; }
        if (s.nav || s.dock) {
          let pose;
          if (s.dock) pose = { x: r.site.xyz[0], y: r.site.xyz[1], yaw: r.site.rpy[2] };
          else {
            // Stand standoff_m short of the target on the line from where the base is now, facing it.
            const dx = s.nav[0] - base.x, dy = s.nav[1] - base.y, d = Math.hypot(dx, dy) || 1;
            pose = { x: s.nav[0] - dx / d * mob.standoff_m, y: s.nav[1] - dy / d * mob.standoff_m, yaw: Math.atan2(dy, dx) };
          }
          setBase(pose); steps.push({ name: s.name, kind: 'navigate', pose }); continue;
        }
        const sol = K.solveIK(r.kin, tool, Object.assign({}, m.home, q), { p: s.p, approach: down }, baseT);
        if (!sol.ok) return { ok: false, failure: 'UNREACHABLE', reason: `${s.name} at [${s.p.map(v => v.toFixed(2))}] outside workspace` };
        q = sol.q; steps.push({ name: s.name, kind: 'move_pose', q, p: s.p });
      }
      return { ok: true, steps };
    }

    rank(spec, exclude = []) {
      return Object.values(this.robots)
        .filter(r => !exclude.includes(r.id) && r.state === RS.IDLE && !r.task)
        .map(r => ({ r, plan: this.plan(r.id, spec) }))
        .filter(x => x.plan.ok)
        .map(x => x.r.id);
    }

    // ---- tasks ---------------------------------------------------------------
    submit(spec, { robot = 'auto', allowFailover = true } = {}) {
      const id = `task-${String(++this._seq).padStart(3, '0')}`;
      const task = { id, spec, state: TS.ACCEPTED, robot: null, step: 0, steps: [], held: false, allowFailover, history: [], preferred: robot };
      this.tasks[id] = task;
      this.audit.emit({ type: 'task.accepted', task: id, msg: `${id} accepted: ${spec.skill}(${spec.object}: ${spec.from} → ${spec.to}) on ${robot}` });
      task.done = this._run(task, robot === 'auto' ? null : robot);
      return task;
    }

    async _run(task, robotId, exclude = []) {
      if (!robotId) {
        const ranked = this.rank(task.spec, exclude);
        if (!ranked.length) return this._finish(task, TS.FAILED, 'NO_CAPABLE_ROBOT', 'no available robot can perform this task');
        robotId = ranked[0];
      }
      const r = this.robots[robotId];
      if (r.state !== RS.IDLE || r.task) {
        if (task.allowFailover) { this.audit.emit({ type: 'task.reassign', task: task.id, level: 'warn', msg: `${robotId} unavailable (${r.state}); finding another robot` }); return this._run(task, null, [...exclude, robotId]); }
        return this._finish(task, TS.FAILED, 'ROBOT_UNAVAILABLE', `${robotId} is ${r.state}`);
      }
      const plan = this.plan(robotId, task.spec);
      if (!plan.ok) {
        this.audit.emit({ type: 'task.rejected', task: task.id, robot: robotId, level: 'warn', msg: `${robotId} rejected: ${plan.failure} — ${plan.reason}` });
        if (task.allowFailover) return this._run(task, null, [...exclude, robotId]);
        return this._finish(task, TS.FAILED, plan.failure, plan.reason);
      }
      task.robot = robotId; task.steps = plan.steps; task.step = 0; task.state = TS.EXECUTING; r.task = task.id; r.state = RS.EXECUTING;
      task.history.push({ robot: robotId, at: Date.now() });
      this.audit.emit({ type: 'task.assigned', task: task.id, robot: robotId, msg: `${task.id} → ${robotId} (${plan.steps.length} steps, plan validated against manifest)` });
      this._changed();
      return this._execute(task, r, exclude);
    }

    async _execute(task, r, exclude) {
      while (task.step < task.steps.length) {
        const s = task.steps[task.step];
        task.current = s.name; this._changed();
        const res = await this._runStep(r, s);
        if (res.ok) {
          if (s.kind === 'gripper' && s.close) { this.world.attach(task.spec.object, r.id); task.held = true; }
          if (s.kind === 'gripper' && !s.close) { this.world.detach(task.spec.object, task.spec.to); task.held = false; }
          task.step++; continue;
        }
        if (res.failure === 'CANCELED') return this._finish(task, TS.CANCELED, 'CANCELED', 'canceled', r);
        const decision = await this._handleFailure(task, r, res.failure, exclude);
        if (decision === 'resume') continue;
        return decision; // terminal promise from failover / finish
      }
      return this._finish(task, TS.SUCCEEDED, null, `${task.id} completed on ${r.id}`, r);
    }

    _runStep(r, s) {
      return new Promise((resolve) => {
        let settled = false;
        const done = (v) => { if (!settled) { settled = true; r.watch = r.watch.filter(f => f !== wd); resolve(v); } };
        const wd = (failure) => { r.adapter.cancel(); done({ ok: false, failure }); };
        r.watch.push(wd);
        if (r.state === RS.OFFLINE) return wd('CONNECTIVITY');
        if (s.kind === 'gripper') r.adapter.gripper(s.close).then(done);
        else if (s.kind === 'navigate') r.adapter.navigate(s.pose).then(done);
        else r.adapter.followJointTrajectory(this._trajectory(r, s.q)).then(done);
      });
    }

    _trajectory(r, target) {
      const q0 = r.adapter.jointState(), names = Object.keys(target);
      const vmax = Math.min(...names.map(n => r.kin.joints[n].velocity)) * SPEED_FACTOR;
      const dmax = Math.max(...names.map(n => Math.abs(target[n] - q0[n])));
      const T = Math.max(0.5, dmax / vmax * 1.5), N = 24, pts = [];
      for (let i = 1; i <= N; i++) {
        const u = i / N, s = 0.5 - 0.5 * Math.cos(Math.PI * u);      // smooth start/stop
        const q = {}; names.forEach(n => { q[n] = q0[n] + (target[n] - q0[n]) * s; });
        pts.push({ q, t: T * u });
      }
      return pts;
    }

    async _handleFailure(task, r, failure, exclude) {
      const info = C.Failure[failure] || C.Failure.OEM_INTERNAL;
      this.audit.emit({ type: 'task.step_failed', level: 'warn', task: task.id, robot: r.id, msg: `${task.id} step "${task.current}" failed on ${r.id}: ${failure}${r.detail ? ' (' + r.detail + ')' : ''}` });
      // 1) local recovery on the same robot (bounded)
      const now = Date.now(); r.recoveries = r.recoveries.filter(t => now - t < RECOVERY_WINDOW_MS);
      if (info.localRecovery && r.recoveries.length < MAX_LOCAL_RECOVERIES) {
        r.recoveries.push(now); r.state = RS.RECOVERING; this._changed();
        this.audit.emit({ type: 'robot.recovering', task: task.id, robot: r.id, msg: `auto-recovery ${r.recoveries.length}/${MAX_LOCAL_RECOVERIES} on ${r.id} for ${failure}` });
        const rec = await r.adapter.recover(failure);
        r.state = rec.ok ? RS.EXECUTING : RS.FAULTED;
        if (rec.ok) { this.audit.emit({ type: 'robot.recovered', task: task.id, robot: r.id, msg: `${r.id} recovered; resuming at "${task.current}"` }); return 'resume'; }
      }
      // 2) failover — only if the object is not in this robot's gripper
      if (!task.held && task.allowFailover) {
        r.task = null; task.state = TS.PREEMPTED;
        this.audit.emit({ type: 'task.preempted', level: 'warn', task: task.id, robot: r.id, msg: `${task.id} preempted on ${r.id}; object still at source → eligible for failover` });
        const clr = this._clearance(r, task.spec);
        const next = clr.ok ? this.rank(task.spec, [...exclude, r.id]) : [];
        if (!clr.ok) this.audit.emit({ type: 'task.failover_blocked', level: 'error', task: task.id, robot: r.id, msg: `failover blocked: ${r.id} last known TCP is ${clr.d.toFixed(2)} m from ${clr.station} (< ${CLEARANCE_M} m) — shared workspace not provably clear` });
        if (next.length) {
          this.audit.emit({ type: 'task.failover', level: 'warn', task: task.id, robot: next[0], msg: `failover ${r.id} → ${next[0]} (re-planned from start)` });
          return this._run(task, next[0], [...exclude, r.id]);
        }
        if (clr.ok) return this._finish(task, TS.FAILED, failure, 'no healthy robot available for failover', null);
        r.task = task.id;
      }
      // 3) escalate to a human
      task.state = TS.BLOCKED_ON_OPERATOR; this._changed();
      this.audit.emit({ type: 'task.blocked', level: 'error', task: task.id, robot: r.id, msg: `${task.id} BLOCKED_ON_OPERATOR — ${task.held ? r.id + ' holds ' + task.spec.object : r.id + ' may obstruct the cell'}; needs a human decision` });
      const choice = await new Promise(res => { task.operator = res; this._changed(); });
      task.operator = null;
      this.audit.emit({ type: 'operator.intervention', task: task.id, robot: r.id, msg: `operator chose "${choice}" for ${task.id}` });
      if (choice === 'resume') { task.state = TS.EXECUTING; return 'resume'; }
      return this._finish(task, TS.CANCELED, failure, 'aborted by operator', r);
    }

    // A dead robot's last known pose must be clear of the stations the next
    // robot will work in. Real deployments add zone interlocks via Traffic &
    // Resource Management; this is the minimum the RIL itself must check.
    _clearance(r, spec) {
      const T = r.kin.fk(r.adapter.jointState(), this.baseT(r.id))[r.manifest.tool_frame.link];
      const tcp = K.apply(T, r.manifest.tool_frame.offset);
      let worst = { ok: true, d: Infinity };
      [spec.from, spec.to].forEach(st => {
        const p = this.world.stations[st], d = Math.hypot(tcp[0] - p[0], tcp[1] - p[1]);
        if (d < worst.d) worst = { ok: d >= CLEARANCE_M, d, station: st };
      });
      return worst;
    }

    operatorDecision(taskId, choice) { const t = this.tasks[taskId]; if (t && t.operator) t.operator(choice); }

    async operatorRecover(robotId) {
      const r = this.robots[robotId];
      if (!r.failure) return;
      r.state = RS.RECOVERING; this._changed();
      this.audit.emit({ type: 'operator.intervention', robot: r.id, msg: `operator requested recovery of ${r.id} (${r.failure})` });
      const rec = await r.adapter.recover(r.failure);
      this.audit.emit({ type: rec.ok ? 'robot.recovered' : 'robot.recover_failed', level: rec.ok ? 'info' : 'error', robot: r.id, msg: rec.ok ? `${r.id} recovered` : `${r.id} recovery failed — needs on-site attention` });
      r.state = rec.ok ? (r.task ? RS.EXECUTING : RS.IDLE) : RS.FAULTED; this._changed();
    }

    cancel(taskId) { const t = this.tasks[taskId]; if (t && t.robot) this.robots[t.robot].adapter.cancel(); }

    _finish(task, state, failure, msg, r) {
      task.state = state; task.failure = failure; task.current = null;
      if (r) { r.task = null; if (r.state === RS.EXECUTING) r.state = RS.IDLE; }
      this.audit.emit({ type: `task.${state.toLowerCase()}`, level: state === TS.SUCCEEDED ? 'info' : 'error', task: task.id, robot: r && r.id, msg: `${task.id} ${state}${failure && state !== TS.SUCCEEDED ? ' (' + failure + ')' : ''} — ${msg}` });
      this._changed();
      return task;
    }
  }

  // Minimal world model. In the full platform this lives in the Fleet State
  // Engine / Site World Model, not in the RIL; it's here so the demo is
  // self-contained.
  class World {
    constructor(def) { this.def = def; this.reset(); }
    reset() {
      this.stations = JSON.parse(JSON.stringify(this.def.stations));
      this.objects = {};
      Object.entries(this.def.objects).forEach(([k, o]) => { this.objects[k] = { ...o, pos: this.stations[o.at].slice(), heldBy: null }; });
    }
    attach(obj, robot) { this.objects[obj].heldBy = robot; }
    detach(obj, station) { const o = this.objects[obj]; o.heldBy = null; o.pos = this.stations[station].slice(); }
  }

  return { RIL: RIL_, World, LIVENESS_TIMEOUT_MS };
})();
