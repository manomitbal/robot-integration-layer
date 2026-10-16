// ---------------------------------------------------------------------------
// SIMULATION — stand-in for "OEM ROS 2 driver + robot controller".
// Plays the role of ur_robot_driver (use_mock_hardware:=true) and
// fanuc_driver (use_mock:=true). It exposes ROS-shaped endpoints (actions,
// services, status topics) with each vendor's own status vocabulary, so the
// adapters have to do real translation work. Swap this for rclnodejs /
// rosbridge and the adapters do not change.
// ---------------------------------------------------------------------------
RIL.SimRos2 = (() => {
  const TICK_MS = 20;

  class SimRobotGraph {
    // vendor: 'ur' | 'fanuc' (selects status vocabulary only)
    constructor({ ns, vendor, jointNames, q0, onTrace }) {
      this.ns = ns; this.vendor = vendor; this.jointNames = jointNames;
      this.q = Object.assign({}, q0);
      this.linkUp = true;           // network / driver connection
      this.goal = null;
      this.ioOut = {};
      this.trace = onTrace || (() => {});
      this.status = vendor === 'ur'
        ? { robot_mode: 'RUNNING', safety_mode: 'NORMAL', program_running: true }
        : { servo_ready: true, in_error: false, alarm: '', motion_possible: true };
      this._timer = setInterval(() => this._tick(), TICK_MS);
    }

    // --- action server: /<controller>/follow_joint_trajectory -------------
    sendGoal(controller, points) {
      return new Promise((resolve) => {
        if (!this.linkUp) return; // silence: goal never acknowledged
        this.trace('→', `action ${this.ns}/${controller}/follow_joint_trajectory  send_goal  points=${points.length}  T=${points[points.length-1].t.toFixed(2)}s`);
        this.goal = { points, t: 0, start: Object.assign({}, this.q), resolve, controller };
      });
    }
    cancelGoal() {
      if (!this.linkUp) return false;          // request never reaches the robot
      if (this.goal) { this.trace('→', `action ${this.ns}/${this.goal.controller}/follow_joint_trajectory  cancel_goal`); this.goal.resolve({ code: 'CANCELED' }); this.goal = null; }
      return true;
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
      } else {
        if (service === '/fanuc_gpio_controller/set_bool_io') { this.ioOut[req.index] = !!req.value; return { result: 0 }; }
        if (service === '/fanuc_gpio_controller/reset') {
          this.status = { servo_ready: true, in_error: false, alarm: '', motion_possible: true }; return { result: 0 };
        }
      }
      return { success: false, result: -1 };
    }

    // --- status topics (what the adapter subscribes to) ------------------------
    readStatus() { return this.linkUp ? { stamp: Date.now(), ...this.status } : null; }
    readJointStates() { return Object.assign({}, this.q); }

    // --- fault injection (test harness only) ----------------------------------
    injectLinkLoss() { this.linkUp = false; }
    restoreLink() { this.linkUp = true; }
    injectProtectiveStop() {
      if (this.vendor === 'ur') { this.status.safety_mode = 'PROTECTIVE_STOP'; this.status.program_running = false; }
      else this.status = { servo_ready: false, in_error: true, alarm: 'SRVO-050 Collision Detect alarm', motion_possible: false };
      if (this.goal) { const g = this.goal; this.goal = null; this.trace('←', `action ${this.ns}/${g.controller}/follow_joint_trajectory  result=ABORTED`); g.resolve({ code: 'ABORTED' }); }
    }

    _tick() {
      if (!this.goal || !this.linkUp) return; // on link loss the robot holds position (controller-side behaviour)
      const g = this.goal, now = performance.now(); g.t += Math.min(0.1, (now - (g.last || now - TICK_MS)) / 1000); g.last = now;  // wall-clock, robust to timer throttling
      const pts = g.points; const T = pts[pts.length - 1].t;
      let i = 0; while (i < pts.length - 1 && pts[i + 1].t < g.t) i++;
      const a = i === 0 && g.t < pts[0].t ? { q: g.start, t: 0 } : pts[i];
      const b = pts[Math.min(i + 1, pts.length - 1)];
      const s = b.t > a.t ? Math.min(1, Math.max(0, (g.t - a.t) / (b.t - a.t))) : 1;
      this.jointNames.forEach(n => { this.q[n] = a.q[n] + (b.q[n] - a.q[n]) * s; });
      if (g.t >= T) {
        this.jointNames.forEach(n => { this.q[n] = pts[pts.length - 1].q[n]; });
        this.goal = null;
        this.trace('←', `action ${this.ns}/${g.controller}/follow_joint_trajectory  result=SUCCEEDED`);
        g.resolve({ code: 'SUCCEEDED' });
      }
    }
  }
  return { SimRobotGraph };
})();
