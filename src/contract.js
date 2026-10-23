// ---------------------------------------------------------------------------
// CONTRACT — the only thing orchestration (and this demo's UI) is allowed to
// depend on. Versioned; no OEM names, controller names, or vendor codes.
// ---------------------------------------------------------------------------
RIL.Contract = Object.freeze({
  VERSION: '0.4.0',

  RobotState: Object.freeze({
    OFFLINE: 'OFFLINE', CONNECTING: 'CONNECTING', IDLE: 'IDLE', LEASED: 'LEASED',
    EXECUTING: 'EXECUTING', FAULTED: 'FAULTED', RECOVERING: 'RECOVERING', ESTOP: 'ESTOP',
  }),

  TaskState: Object.freeze({
    ACCEPTED: 'ACCEPTED', EXECUTING: 'EXECUTING', SUCCEEDED: 'SUCCEEDED', FAILED: 'FAILED',
    CANCELED: 'CANCELED', PREEMPTED: 'PREEMPTED', BLOCKED_ON_OPERATOR: 'BLOCKED_ON_OPERATOR',
  }),

  // Normalized failure taxonomy. Adapters MUST map every OEM fault into one of
  // these; the raw OEM code travels alongside for humans, never for decisions.
  Failure: Object.freeze({
    CONNECTIVITY:     { retryable: true,  localRecovery: false, operator: false },
    PROTECTIVE_STOP:  { retryable: true,  localRecovery: true,  operator: false },
    ESTOP:            { retryable: false, localRecovery: false, operator: true  },
    UNREACHABLE:      { retryable: false, localRecovery: false, operator: false },
    PAYLOAD_EXCEEDED: { retryable: false, localRecovery: false, operator: false },
    TOOL_FAULT:       { retryable: true,  localRecovery: true,  operator: true  },
    OEM_INTERNAL:     { retryable: false, localRecovery: false, operator: true  },
  }),

  // Tier 1: skills. Orchestration submits these.
  Skills: Object.freeze({
    pick_place: { params: ['object', 'from', 'to'], requires: { kind: ['arm', 'mobile_manipulator'], tool: ['parallel_gripper', 'claw_gripper'] } },
  }),

  // Tier 2: motion primitives. Gated behind a lease + privileged role in
  // production; the platform composes skills out of these.
  Primitives: Object.freeze(['move_joints', 'move_pose', 'gripper', 'set_io', 'navigate']),
});

/* Adapter interface (what every OEM plugin implements). Everything else is
 * shared platform code.
 *
 *   manifest                     -> declarative capability description
 *   connect()                    -> Promise<void>
 *   health()                     -> { alive:bool, state:RobotState, failure?:code, oem:{...raw} }
 *   followJointTrajectory(pts)   -> Promise<{ ok, failure?, oem? }>   (points: [{q:{name:rad}, t:s}])
 *   gripper(close:bool)          -> Promise<{ ok, failure?, oem? }>
 *   cancel()                     -> void
 *   recover(failureCode)         -> Promise<{ ok }>
 *   jointState()                 -> {name: rad}
 *
 * Mobile robots (manifest.mobility present) additionally implement:
 *   basePose()                   -> { x, y, yaw }   (site frame, m / rad)
 *   navigate(pose)               -> Promise<{ ok, failure?, oem? }>
 */
