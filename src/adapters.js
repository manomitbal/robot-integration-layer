// ---------------------------------------------------------------------------
// ADAPTER SDK — reusable base for any arm driven through ros2_control's
// FollowJointTrajectory. Most modern arm drivers (UR, FANUC CRX, KUKA, ABB
// via abb_ros2, Doosan, Kinova) fit this shape, so a new arm adapter is
// usually a subclass of ~40 lines: names, status mapping, IO, recovery.
// ---------------------------------------------------------------------------
RIL.Adapters = (() => {
  const C = RIL.Contract;
  const withTimeout = (p, ms) => Promise.race([p, new Promise(r => setTimeout(() => r({ timeout: true }), ms))]);

  class Ros2ArmAdapter {
    constructor({ manifest, graph }) {
      this.manifest = manifest;
      this.graph = graph;           // in production: rclnodejs node / rosbridge connection
      this.lastStatus = null;
    }
    async connect() { return { ok: true }; }
    jointState() { return this.graph.readJointStates(); }

    health() {
      const raw = this.graph.readStatus();
      if (!raw) return { alive: false };
      if (this.staleGoal && this.graph.cancelGoal() !== false) this.staleGoal = false;  // reconnect handshake: never let a pre-outage goal resume
      this.lastStatus = raw;
      const n = this.normalizeStatus(raw);           // OEM-specific
      return { alive: true, ...n, oem: raw };
    }

    async followJointTrajectory(points) {
      const ctl = this.manifest.transport.trajectory_controller;
      const res = await this.graph.sendGoal(ctl, points);
      if (res.code === 'SUCCEEDED') return { ok: true };
      if (res.code === 'CANCELED') return { ok: false, failure: 'CANCELED' };
      const h = this.health();                         // classify via status topic, not result code
      return { ok: false, failure: h.failure || 'OEM_INTERNAL', oem: h.oem };
    }
    cancel() { if (this.graph.cancelGoal() === false) this.staleGoal = true; }

    async gripper(close) {
      const r = await withTimeout(this.setGripperIO(close), 2000);   // OEM-specific
      if (r.timeout) return { ok: false, failure: 'CONNECTIVITY' };
      return r.ok ? { ok: true } : { ok: false, failure: 'TOOL_FAULT', oem: r };
    }
    async recover(failure) {
      if (failure !== 'PROTECTIVE_STOP') return { ok: false };
      const r = await withTimeout(this.recoverProtectiveStop(), 4000);  // OEM-specific
      return { ok: !r.timeout && r.ok };
    }
  }

  // ---- Universal Robots (ur_robot_driver) ---------------------------------
  class URAdapter extends Ros2ArmAdapter {
    normalizeStatus(s) {
      if (s.safety_mode === 'PROTECTIVE_STOP') return { state: C.RobotState.FAULTED, failure: 'PROTECTIVE_STOP', detail: 'safety_mode=PROTECTIVE_STOP' };
      if (s.safety_mode === 'EMERGENCY_STOP' || s.safety_mode === 'ROBOT_EMERGENCY_STOP') return { state: C.RobotState.ESTOP, failure: 'ESTOP' };
      if (!s.program_running) return { state: C.RobotState.FAULTED, failure: 'OEM_INTERNAL', detail: 'External Control program not running' };
      return { state: C.RobotState.IDLE };
    }
    async setGripperIO(close) {
      const pin = this.manifest.tools[0].io.pin;
      const r = await this.graph.call('/io_and_status_controller/set_io', { fun: 1, pin, state: close ? 1 : 0 });
      return { ok: r.success };
    }
    async recoverProtectiveStop() {
      // UR requires the protective stop to be held >= 5 s before unlock on real hardware.
      const a = await this.graph.call('/dashboard_client/unlock_protective_stop', {});
      if (!a.success) return { ok: false };
      const b = await this.graph.call('/io_and_status_controller/resend_robot_program', {});
      return { ok: b.success };
    }
  }

  // ---- FANUC (fanuc_driver, CRX series) -----------------------------------
  class FanucAdapter extends Ros2ArmAdapter {
    normalizeStatus(s) {
      if (s.in_error && /SRVO-050|SRVO-0?13|collision/i.test(s.alarm)) return { state: C.RobotState.FAULTED, failure: 'PROTECTIVE_STOP', detail: s.alarm };
      if (s.in_error && /SRVO-001|SRVO-002|E-?STOP/i.test(s.alarm)) return { state: C.RobotState.ESTOP, failure: 'ESTOP', detail: s.alarm };
      if (s.in_error) return { state: C.RobotState.FAULTED, failure: 'OEM_INTERNAL', detail: s.alarm };
      if (!s.servo_ready || !s.motion_possible) return { state: C.RobotState.FAULTED, failure: 'OEM_INTERNAL', detail: 'servo not ready' };
      return { state: C.RobotState.IDLE };
    }
    async setGripperIO(close) {
      const index = this.manifest.tools[0].io.index;
      const r = await this.graph.call('/fanuc_gpio_controller/set_bool_io', { io_type: 'DO', index, value: close });
      return { ok: r.result === 0 };
    }
    async recoverProtectiveStop() {
      const r = await this.graph.call('/fanuc_gpio_controller/reset', {});
      return { ok: r.result === 0 };
    }
  }

  return { Ros2ArmAdapter, URAdapter, FanucAdapter, byName: { URAdapter, FanucAdapter } };
})();
