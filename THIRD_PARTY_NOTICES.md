# Third-party notices

Robot descriptions and meshes embedded in `assets/` and `index.html` are derived from:

| Robot | Source | License |
| --- | --- | --- |
| FANUC CRX-10iA | [FANUC-CORPORATION/fanuc_description](https://github.com/FANUC-CORPORATION/fanuc_description) (fanuc_crx_description) | Apache-2.0 |
| UR10e | [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) | BSD-3-Clause |
| Boston Dynamics Spot + Arm | [rai-opensource/spot_description](https://github.com/rai-opensource/spot_description) (submodule of [spot_ros2](https://github.com/rai-opensource/spot_ros2)) | MIT |

Modifications: xacro expanded to URDF; collision/inertial/ros2_control tags removed; visual meshes decimated and re-encoded. Spot OBJ meshes are coloured from the `Kd` values in `spot.mtl`.

Runtime libraries loaded from CDNs: three.js r128 (MIT), OrbitControls (MIT). Fonts: Barlow, JetBrains Mono (SIL OFL 1.1, Google Fonts).

Endpoint names used by the simulated drivers follow [FANUC-CORPORATION/fanuc_driver](https://github.com/FANUC-CORPORATION/fanuc_driver) , [UniversalRobots/Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) and [rai-opensource/spot_ros2](https://github.com/rai-opensource/spot_ros2) (spot_driver).
