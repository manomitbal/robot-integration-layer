// ---------------------------------------------------------------------------
// CAPABILITY MANIFESTS — data, not code. Values from OEM datasheets and the
// URDFs shipped in UniversalRobots/Universal_Robots_ROS2_Description,
// FANUC-CORPORATION/fanuc_description and rai-opensource/spot_description.
// Adding a robot model starts here.
// ---------------------------------------------------------------------------
RIL.Manifests = {
  crx10ia: {
    id: 'crx10ia', display: 'FANUC CRX-10iA', oem: 'FANUC', kind: 'arm', adapter: 'FanucAdapter',
    description: { urdf: 'fanuc_crx_description/robot/crx10ia.urdf.xacro', root_link: 'base_link' },
    // FANUC flange frame: +X points out of the flange.
    tool_frame: { link: 'flange', offset: [0.15, 0, 0], approach: [1, 0, 0] },
    limits: { reach_m: 1.249, payload_kg: 10, dof: 6 },
    skills: ['pick_place'],
    tools: [{ id: 'gripper0', type: 'parallel_gripper', length_m: 0.15, io: { kind: 'DO', index: 1 } }],
    recovery: { PROTECTIVE_STOP: 'srv /fanuc_gpio_controller/reset' },
    transport: { kind: 'ros2', driver: 'fanuc_driver', trajectory_controller: 'joint_trajectory_controller', controller_type: 'fanuc_controllers/ScaledJointTrajectoryController', status: '/fanuc_gpio_controller/robot_status' },
    home: { J1: 2.344, J2: -0.182, J3: -0.267, J4: 0, J5: -1.485, J6: 0 },
  },
  ur10e: {
    id: 'ur10e', display: 'Universal Robots UR10e', oem: 'Universal Robots', kind: 'arm', adapter: 'URAdapter',
    description: { urdf: 'ur_description/urdf/ur.urdf.xacro (ur_type:=ur10e)', root_link: 'base_link' },
    // REP-199 tool0: +Z points out of the flange.
    tool_frame: { link: 'tool0', offset: [0, 0, 0.15], approach: [0, 0, 1] },
    limits: { reach_m: 1.3, payload_kg: 12.5, dof: 6 },
    skills: ['pick_place'],
    tools: [{ id: 'gripper0', type: 'parallel_gripper', length_m: 0.15, io: { kind: 'standard_digital_out', pin: 0 } }],
    recovery: { PROTECTIVE_STOP: 'srv /dashboard_client/unlock_protective_stop → /io_and_status_controller/resend_robot_program' },
    transport: { kind: 'ros2', driver: 'ur_robot_driver', trajectory_controller: 'joint_trajectory_controller', controller_type: 'joint_trajectory_controller/JointTrajectoryController', status: '/io_and_status_controller/safety_mode' },
    home: { shoulder_pan_joint: 0.744, shoulder_lift_joint: -2.006, elbow_joint: 2.094, wrist_1_joint: -1.66, wrist_2_joint: -1.571, wrist_3_joint: 0 },
  },
  spot: {
    id: 'spot', display: 'Boston Dynamics Spot + Arm', oem: 'Boston Dynamics', kind: 'mobile_manipulator', adapter: 'SpotAdapter',
    description: { urdf: 'spot_description/urdf/spot.urdf.xacro (arm:=true)', root_link: 'body' },
    // ~SDK "hand" frame: +X out of the wrist, between the jaws.
    tool_frame: { link: 'arm_link_wr1', offset: [0.2, 0, 0], approach: [1, 0, 0] },
    limits: { reach_m: 0.984, payload_kg: 5, dof: 6 },
    // Arm targets are planned from a stand pose standoff_m short of the station, facing it.
    mobility: {
      kind: 'legged', body_height_m: 0.485, standoff_m: 0.7, max_speed_mps: 0.8, max_yaw_rate: 1.0,
      stand: {
        front_left_hip_x: 0, front_left_hip_y: 0.8, front_left_knee: -1.6, front_right_hip_x: 0, front_right_hip_y: 0.8, front_right_knee: -1.6,
        rear_left_hip_x: 0, rear_left_hip_y: 0.8, rear_left_knee: -1.6, rear_right_hip_x: 0, rear_right_hip_y: 0.8, rear_right_knee: -1.6,
      },
    },
    skills: ['pick_place'],
    tools: [{ id: 'gripper0', type: 'claw_gripper', length_m: 0.2, io: { kind: 'service', open: '/open_gripper', close: '/close_gripper' } }],
    recovery: { PROTECTIVE_STOP: 'srv /clear_behavior_fault → /stand' },
    transport: { kind: 'ros2', driver: 'spot_driver', trajectory_controller: 'robot_command', base_controller: 'trajectory', controller_type: 'spot_msgs/action/RobotCommand', status: '/status/behavior_faults' },
    // Stowed: arm folded along the back, jaw resting on the front of the body.
    home: { arm_sh0: 0, arm_sh1: -3.11, arm_el0: 3.13, arm_el1: 0, arm_wr0: 1.57, arm_wr1: 0 },
  },
};

// SITE PROFILE — per-deployment configuration. Site differences live here,
// never in adapter code.
RIL.Site = {
  name: 'Demo cell A',
  robots: {
    crx10ia: { xyz: [-0.55, 0, 0], rpy: [0, 0, 0] },
    ur10e: { xyz: [0.55, 0, 0], rpy: [0, 0, 0] },
    // Mobile robots: dock pose, returned to after every task.
    spot: { xyz: [-0.3, 2.1, 0], rpy: [0, 0, -Math.PI / 2] },
  },
  world: {
    // station_c is out of reach of both fixed arms; only a mobile robot can serve it.
    stations: { station_a: [-0.25, 0.55, 0.025], station_b: [0.25, 0.55, 0.025], station_c: [1.7, 1.9, 0.025] },
    objects: { part_1: { at: 'station_a', size_m: 0.05, mass_kg: 1.2 } },
  },
};
