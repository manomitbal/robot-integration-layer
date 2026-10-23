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

    followJointTrajectory(points) { return this._goal(this.manifest.transport.trajectory_controller, points); }
    async _goal(action, payload) {
      const res = await this.graph.sendGoal(action, payload);
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

  // ---- Boston Dynamics Spot + Arm (spot_driver) -----------------------------
  // Not ros2_control: arm moves go through the robot_command action, the body
  // through the trajectory action, and every command needs the body lease.
  class SpotAdapter extends Ros2ArmAdapter {
    async connect() {
      for (const srv of ['/claim', '/power_on', '/stand']) {
        const r = await withTimeout(this.graph.call(srv, {}), 4000);
        if (r.timeout || !r.success) return { ok: false, failed: srv };
      }
      return { ok: true };
    }
    normalizeStatus(s) {
      if (s.estop !== 'STATE_NOT_ESTOPPED') return { state: C.RobotState.ESTOP, failure: 'ESTOP', detail: `status/estop=${s.estop}` };
      const f = s.behavior_faults[0];
      if (f && f.status === 'STATUS_CLEARABLE') return { state: C.RobotState.FAULTED, failure: 'PROTECTIVE_STOP', detail: `behavior_fault ${f.behavior_fault_id} ${f.cause}` };
      if (f) return { state: C.RobotState.FAULTED, failure: 'OEM_INTERNAL', detail: `unclearable behavior_fault ${f.cause}` };
      if (s.motor_power_state !== 'STATE_ON') return { state: C.RobotState.FAULTED, failure: 'OEM_INTERNAL', detail: 'motor power off' };
      if (!s.lease_owner) return { state: C.RobotState.FAULTED, failure: 'OEM_INTERNAL', detail: 'body lease not held' };
      return { state: C.RobotState.IDLE };
    }
    basePose() { return this.graph.readOdometry(); }
    navigate(pose) {
      const mob = this.manifest.mobility, b = this.basePose();
      const dyaw = Math.abs(Math.atan2(Math.sin(pose.yaw - b.yaw), Math.cos(pose.yaw - b.yaw)));
      const duration = Math.max(1, Math.hypot(pose.x - b.x, pose.y - b.y) / mob.max_speed_mps, dyaw / mob.max_yaw_rate) * 1.2;
      return this._goal(this.manifest.transport.base_controller, { target_pose: pose, duration });
    }
    async setGripperIO(close) {
      const io = this.manifest.tools[0].io;
      const r = await this.graph.call(close ? io.close : io.open, {});
      return { ok: r.success };
    }
    async recoverProtectiveStop() {
      for (const f of (this.lastStatus && this.lastStatus.behavior_faults) || []) {
        const r = await this.graph.call('/clear_behavior_fault', { id: f.behavior_fault_id });
        if (!r.success) return { ok: false };
      }
      const st = await this.graph.call('/stand', {});
      return { ok: st.success };
    }
  }

  return { Ros2ArmAdapter, URAdapter, FanucAdapter, SpotAdapter, byName: { URAdapter, FanucAdapter, SpotAdapter } };
})();
