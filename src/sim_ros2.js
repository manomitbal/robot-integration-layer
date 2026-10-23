// ---------------------------------------------------------------------------
// SIMULATION — stand-in for "OEM ROS 2 driver + robot controller".
// Plays the role of ur_robot_driver (use_mock_hardware:=true),
// fanuc_driver (use_mock:=true) and spot_driver (mock_enable:=true). It
// exposes ROS-shaped endpoints (actions,
// services, status topics) with each vendor's own status vocabulary, so the
// adapters have to do real translation work. Swap this for rclnodejs /
// rosbridge and the adapters do not change.
// ---------------------------------------------------------------------------
RIL.SimRos2 = (() => {
  const TICK_MS = 20;
  const SPOT_LEGS = ['front_left', 'front_right', 'rear_left', 'rear_right'];
  const SPOT_FINGER = { joint: 'arm_f1x', open: -1.0, closed: -0.3 };   // closed = resting on a 5 cm part
  const spotIdle = () => ({ estop: 'STATE_NOT_ESTOPPED', motor_power_state: 'STATE_OFF', lease_owner: '', behavior_faults: [] });

  class SimRobotGraph {
    // vendor: 'ur' | 'fanuc' | 'spot' (selects status vocabulary only)
    // base: initial odometry {x, y, yaw} for mobile robots; stand: leg joint positions while standing.
    constructor({ ns, vendor, jointNames, q0, base, stand, onTrace }) {
      this.ns = ns; this.vendor = vendor; this.jointNames = jointNames;
      this.q = Object.assign({}, stand, q0);
      this.stand = stand; this.base = base && { ...base };
      this.linkUp = true;           // network / driver connection
      this.goal = null;
      this.ioOut = {};
      this.trace = onTrace || (() => {});
      this.status = vendor === 'ur' ? { robot_mode: 'RUNNING', safety_mode: 'NORMAL', program_running: true }
        : vendor === 'spot' ? spotIdle()
        : { servo_ready: true, in_error: false, alarm: '', motion_possible: true };
      if (vendor === 'spot') this.q[SPOT_FINGER.joint] = 0;
      this._faultSeq = 0;
      this._timer = setInterval(() => this._tick(), TICK_MS);
    }

    // --- action servers ------------------------------------------------------
    // ur / fanuc: /<controller>/follow_joint_trajectory with points.
    // spot: /robot_command (arm joint move, points) or /trajectory (body, {target_pose, duration}).
    sendGoal(controller, payload) {
      return new Promise((resolve) => {
        if (!this.linkUp) return; // silence: goal never acknowledged
        if (this.vendor === 'spot' && !this._spotCanMove()) { this.trace('←', `action ${this.ns}/${controller}  goal REJECTED (${this._spotBlocker()})`); return resolve({ code: 'ABORTED' }); }
        if (controller === 'trajectory') {
          const p = payload.target_pose;
          this.trace('→', `action ${this.ns}/trajectory  send_goal  target_pose(odom)=[${p.x.toFixed(2)}, ${p.y.toFixed(2)}, yaw ${p.yaw.toFixed(2)}]  duration=${payload.duration.toFixed(1)}s`);
          this.goal = { kind: 'base', T: payload.duration, t: 0, from: { ...this.base }, to: { ...p }, resolve, controller };
          return;
        }
        const T = payload[payload.length - 1].t;
        this.trace('→', this.vendor === 'spot'
          ? `action ${this.ns}/robot_command  send_goal  arm_joint_move_command  points=${payload.length}  T=${T.toFixed(2)}s`
          : `action ${this.ns}/${controller}/follow_joint_trajectory  send_goal  points=${payload.length}  T=${T.toFixed(2)}s`);
        this.goal = { kind: 'joints', points: payload, t: 0, start: Object.assign({}, this.q), resolve, controller };
      });
    }
    _actionName(g) { return this.vendor === 'spot' ? `${this.ns}/${g.controller}` : `${this.ns}/${g.controller}/follow_joint_trajectory`; }
    cancelGoal() {
      if (!this.linkUp) return false;          // request never reaches the robot
      if (this.goal) { this.trace('→', `action ${this._actionName(this.goal)}  cancel_goal`); this.goal.resolve({ code: 'CANCELED' }); this.goal = null; }
      return true;
    }
    _spotCanMove() { return !this._spotBlocker(); }
    _spotBlocker() {
      const s = this.status;
      if (s.estop !== 'STATE_NOT_ESTOPPED') return 'estopped';
      if (s.behavior_faults.length) return 'behavior fault active';
      if (s.motor_power_state !== 'STATE_ON') return 'motors off';
      if (!s.lease_owner) return 'no body lease';
      return '';
    }

    // --- services -------------------------------------------------------------
    call(service, req) {
      return new Promise((resolve) => {
        if (!this.linkUp) return; // hangs; caller must time out
        this.trace('→', `srv  ${this.ns}${service}  ${JSON.stringify(req)}`);
        setTimeout(() => resolve(this._service(service, req)), 120);
      });
    }
    _service(service, req) {
      if (this.vendor === 'ur') {
        if (service === '/io_and_status_controller/set_io') { this.ioOut[req.pin] = !!req.state; return { success: true }; }
        if (service === '/dashboard_client/unlock_protective_stop') {
          if (this.status.safety_mode !== 'PROTECTIVE_STOP') return { success: false, answer: 'Robot is not in protective stop' };
          this.status.safety_mode = 'NORMAL'; this.status.program_running = false; return { success: true, answer: 'Protective stop releasing' };
        }
        if (service === '/io_and_status_controller/resend_robot_program') { this.status.program_running = true; return { success: true }; }
      } else if (this.vendor === 'spot') {
        // std_srvs/Trigger-shaped: { success, message }
        const s = this.status, ok = (message = '') => ({ success: true, message }), no = (message) => ({ success: false, message });
        if (service === '/claim') { s.lease_owner = 'ril'; return ok('Success'); }
        if (service === '/power_on') { if (s.estop !== 'STATE_NOT_ESTOPPED') return no('Robot is estopped'); s.motor_power_state = 'STATE_ON'; return ok(); }
        if (service === '/stand') return this._spotCanMove() ? ok() : no(this._spotBlocker());
        if (service === '/open_gripper' || service === '/close_gripper') {
          if (!this._spotCanMove()) return no(this._spotBlocker());
          this.ioOut[0] = service === '/close_gripper'; return ok();
        }
        if (service === '/clear_behavior_fault') {
          const f = s.behavior_faults.find(x => x.behavior_fault_id === req.id);
          if (!f) return no(`no behavior fault ${req.id}`);
          if (f.status !== 'STATUS_CLEARABLE') return no('fault is not clearable');
          s.behavior_faults = s.behavior_faults.filter(x => x !== f); return ok();
        }
      } else {
        if (service === '/fanuc_gpio_controller/set_bool_io') { this.ioOut[req.index] = !!req.value; return { result: 0 }; }
        if (service === '/fanuc_gpio_controller/reset') {
          this.status = { servo_ready: true, in_error: false, alarm: '', motion_possible: true }; return { result: 0 };
        }
      }
      return { success: false, result: -1 };
    }

    // --- status topics (what the adapter subscribes to) ------------------------
    readStatus() {
      if (!this.linkUp) return null;
      const s = this.status;
      return { stamp: Date.now(), ...s, ...(s.behavior_faults && { behavior_faults: s.behavior_faults.map(f => ({ ...f })) }) };
    }
    readJointStates() { return Object.assign({}, this.q); }
    // spot: /odometry (nav_msgs/Odometry), flattened to planar x, y, yaw.
    readOdometry() { return { ...this.base }; }

    // --- fault injection (test harness only) ----------------------------------
    injectLinkLoss() { this.linkUp = false; }
    restoreLink() { this.linkUp = true; }
    injectProtectiveStop() {
      if (this.vendor === 'ur') { this.status.safety_mode = 'PROTECTIVE_STOP'; this.status.program_running = false; }
      else if (this.vendor === 'spot') this.status.behavior_faults.push({ behavior_fault_id: ++this._faultSeq, cause: 'CAUSE_FALL', status: 'STATUS_CLEARABLE' });
      else this.status = { servo_ready: false, in_error: true, alarm: 'SRVO-050 Collision Detect alarm', motion_possible: false };
      if (this.goal) { const g = this.goal; this.goal = null; this._settleLegs(); this.trace('←', `action ${this._actionName(g)}  result=ABORTED`); g.resolve({ code: 'ABORTED' }); }
    }

    _tick() {
      if (this.vendor === 'spot') { const f = SPOT_FINGER, tgt = this.ioOut[0] ? f.closed : f.open; this.q[f.joint] += (tgt - this.q[f.joint]) * 0.2; }
      if (!this.goal || !this.linkUp) return; // on link loss the robot holds position (controller-side behaviour)
      const g = this.goal, now = performance.now(); g.t += Math.min(0.1, (now - (g.last || now - TICK_MS)) / 1000); g.last = now;  // wall-clock, robust to timer throttling
      if (g.kind === 'base') return this._tickBase(g);
      const pts = g.points; const T = pts[pts.length - 1].t;
      let i = 0; while (i < pts.length - 1 && pts[i + 1].t < g.t) i++;
      const a = i === 0 && g.t < pts[0].t ? { q: g.start, t: 0 } : pts[i];
      const b = pts[Math.min(i + 1, pts.length - 1)];
      const s = b.t > a.t ? Math.min(1, Math.max(0, (g.t - a.t) / (b.t - a.t))) : 1;
      this.jointNames.forEach(n => { this.q[n] = a.q[n] + (b.q[n] - a.q[n]) * s; });
      if (g.t >= T) {
        this.jointNames.forEach(n => { this.q[n] = pts[pts.length - 1].q[n]; });
        this.goal = null;
        this.trace('←', `action ${this._actionName(g)}  result=SUCCEEDED`);
        g.resolve({ code: 'SUCCEEDED' });
      }
    }

    // Body trajectory: holonomic glide in odom with a cosmetic diagonal-pair trot.
    // The gait is not a locomotion model; it only animates the reported leg joints.
    _tickBase(g) {
      const u = Math.min(1, g.t / g.T), s = 0.5 - 0.5 * Math.cos(Math.PI * u);
      const dyaw = Math.atan2(Math.sin(g.to.yaw - g.from.yaw), Math.cos(g.to.yaw - g.from.yaw));
      this.base = { x: g.from.x + (g.to.x - g.from.x) * s, y: g.from.y + (g.to.y - g.from.y) * s, yaw: g.from.yaw + dyaw * s };
      const env = Math.min(1, g.t / 0.3, (g.T - g.t) / 0.3), ph = g.t * 2 * Math.PI * 2.2;   // ~2.2 Hz stride
      SPOT_LEGS.forEach(l => {
        const w = Math.sin(ph + (l === 'front_left' || l === 'rear_right' ? 0 : Math.PI));
        this.q[l + '_hip_y'] = this.stand[l + '_hip_y'] + 0.22 * env * w;
        this.q[l + '_knee'] = this.stand[l + '_knee'] - 0.45 * env * Math.max(0, w);   // flex only during swing
      });
      if (u >= 1) {
        this.base = { ...g.to }; this._settleLegs(); this.goal = null;
        this.trace('←', `action ${this.ns}/trajectory  result=SUCCEEDED`);
        g.resolve({ code: 'SUCCEEDED' });
      }
    }
    _settleLegs() { if (this.stand) Object.assign(this.q, this.stand); }
  }
  return { SimRobotGraph };
})();
