# Robot Integration Layer — multi-OEM prototype

A FANUC CRX-10iA, a Universal Robots UR10e and a Boston Dynamics Spot with arm driven through one contract. The application calls

```js
ril.submit({ skill: "pick_place", object: "part_1", from: "station_a", to: "station_b" },
           { robot: "crx10ia" | "ur10e" | "spot" | "auto" })
```

and never names an OEM. Real URDFs and meshes from the official ROS 2 description repos; simulated ROS 2 drivers using each vendor's real endpoint names and status vocabulary.

**Live demo:** `https://manomitbal.github.io/robot-integration-layer/` (after enabling Pages, below)

## Layout

```
index.html          built, self-contained page served by GitHub Pages
src/
  contract.js       versioned contract: robot/task states, failure taxonomy, skills
  kinematics.js     shared: URDF parser, FK, damped-least-squares IK
  platform.js       shared: registry, liveness, planning, allocation, recovery/failover, audit
  adapters.js       adapter SDK base (ros2_control arms) + URAdapter + FanucAdapter + SpotAdapter
  manifests.js      capability manifests + site profile (data only)
  sim_ros2.js       simulated vendor ROS 2 drivers (stand-in for mock hardware)
  app.js            operator UI / orchestration stand-in (three.js)
web/shell.html      page template and styles
assets/robots.json  decimated meshes + URDFs embedded into index.html
assets/urdf/        xacro-expanded URDFs
tools/build.py      bundles src + assets -> index.html
tools/convert_meshes.py   regenerates assets/robots.json from vendor repos
test/scenarios.test.js    headless run of all seven scenarios
```

## Host on GitHub Pages

1. Create a repo and push this folder to `main`.
2. Settings → Pages → Source: *Deploy from a branch* → `main` / `(root)` → Save.
3. Open `https://manomitbal.github.io/robot-integration-layer/` after about a minute.

## Develop

```bash
npm install                 # only needed for tests
npm test                    # 7 scenarios: nominal x2, failover, protective-stop recovery, operator escalation,
                            #   mobile allocation (A→C), Spot behavior-fault recovery
npm run build               # rebuild index.html after editing src/ or web/
npm run serve               # http://localhost:8000
```

## Adding a third arm

1. Add its URDF to `assets/urdf/`, run `tools/convert_meshes.py`.
2. Add a manifest in `src/manifests.js` (tool frame, limits, home, IO, controller) and a placement in the site profile.
3. Subclass `Ros2ArmAdapter`: `normalizeStatus`, `setGripperIO`, `recoverProtectiveStop` (~20 lines).
4. `npm test`, then `npm run build`.

No changes to the planner, allocator, failover logic, or UI.

## Mobile manipulators (Spot)

Spot shows what a robot that is *not* a ros2_control arm costs:

- **Manifest** declares `kind: 'mobile_manipulator'` and a `mobility` block (stand-off distance, body height, speed, standing leg pose). The site profile gives its dock pose.
- **Planner** (one contained change in `plan()`): when a manifest has `mobility`, it inserts `walk_to_pick` / `walk_to_place` / `return_to_dock` navigate steps and solves each arm pose from the stand pose the base will be at. Allocation, failover and escalation are unchanged.
- **SpotAdapter** speaks spot_driver: lease/power/stand handshake, arm moves via the `robot_command` action, body moves via the `trajectory` action, `open_gripper` / `close_gripper`, `status/behavior_faults` → `PROTECTIVE_STOP`, recovered with `clear_behavior_fault` + `stand`.
- `station_c` is outside both arms' workspace, so `{ from: 'station_a', to: 'station_c' }` with `robot: 'auto'` is allocated to Spot by elimination.

Simplifications: the base glides holonomically; the trot is a cosmetic animation of the reported leg joints, not a locomotion model. There is no collision checking of Spot's body against the arms, and failover clearance checks only the TCP.

## Regenerating meshes (optional)

```bash
mkdir vendor && cd vendor
git clone --depth 1 https://github.com/FANUC-CORPORATION/fanuc_description.git
git clone --depth 1 https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
git clone --depth 1 https://github.com/rai-opensource/spot_description.git
cd ..
pip install xacro trimesh pycollada fast_simplification
export PYTHONPATH=$PWD/tools/ament_shim
xacro vendor/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro ur_type:=ur10e name:=ur10e -o assets/urdf/ur10e.urdf
xacro vendor/fanuc_description/fanuc_crx_description/robot/crx10ia.urdf.xacro -o assets/urdf/crx10ia.urdf
xacro vendor/spot_description/spot_description/urdf/spot.urdf.xacro arm:=true -o assets/urdf/spot.urdf
python3 tools/convert_meshes.py && python3 tools/build.py
```

`convert_meshes.py spot` regenerates only the named robot and merges it into the existing `assets/robots.json`.

See `THIRD_PARTY_NOTICES.md` for licenses of embedded robot assets.
