# Robot Integration Layer — multi-OEM prototype

A FANUC CRX-10iA and a Universal Robots UR10e driven through one contract. The application calls

```js
ril.submit({ skill: "pick_place", object: "part_1", from: "station_a", to: "station_b" },
           { robot: "crx10ia" | "ur10e" | "auto" })
```

and never names an OEM. Real URDFs and meshes from the official ROS 2 description repos; simulated ROS 2 drivers using each vendor's real endpoint names and status vocabulary.

**Live demo:** `https://<username>.github.io/ril-demo/` (after enabling Pages, below)

## Layout

```
index.html          built, self-contained page served by GitHub Pages
src/
  contract.js       versioned contract: robot/task states, failure taxonomy, skills
  kinematics.js     shared: URDF parser, FK, damped-least-squares IK
  platform.js       shared: registry, liveness, planning, allocation, recovery/failover, audit
  adapters.js       adapter SDK base (ros2_control arms) + URAdapter + FanucAdapter
  manifests.js      capability manifests + site profile (data only)
  sim_ros2.js       simulated vendor ROS 2 drivers (stand-in for mock hardware)
  app.js            operator UI / orchestration stand-in (three.js)
web/shell.html      page template and styles
assets/robots.json  decimated meshes + URDFs embedded into index.html
assets/urdf/        xacro-expanded URDFs
tools/build.py      bundles src + assets -> index.html
tools/convert_meshes.py   regenerates assets/robots.json from vendor repos
test/scenarios.test.js    headless run of all five scenarios
```

## Host on GitHub Pages

1. Create a repo and push this folder to `main`.
2. Settings → Pages → Source: *Deploy from a branch* → `main` / `(root)` → Save.
3. Open `https://<username>.github.io/ril-demo/` after about a minute.

## Develop

```bash
npm install                 # only needed for tests
npm test                    # 5 scenarios: nominal x2, failover, protective-stop recovery, operator escalation
npm run build               # rebuild index.html after editing src/ or web/
npm run serve               # http://localhost:8000
```

## Adding a third arm

1. Add its URDF to `assets/urdf/`, run `tools/convert_meshes.py`.
2. Add a manifest in `src/manifests.js` (tool frame, limits, home, IO, controller) and a placement in the site profile.
3. Subclass `Ros2ArmAdapter`: `normalizeStatus`, `setGripperIO`, `recoverProtectiveStop` (~20 lines).
4. `npm test`, then `npm run build`.

No changes to the planner, allocator, failover logic, or UI.

## Regenerating meshes (optional)

```bash
mkdir vendor && cd vendor
git clone --depth 1 https://github.com/FANUC-CORPORATION/fanuc_description.git
git clone --depth 1 https://github.com/UniversalRobots/Universal_Robots_ROS2_Description.git
cd ..
pip install xacro trimesh pycollada fast_simplification
export PYTHONPATH=$PWD/tools/ament_shim
xacro vendor/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro ur_type:=ur10e name:=ur10e -o assets/urdf/ur10e.urdf
xacro vendor/fanuc_description/fanuc_crx_description/robot/crx10ia.urdf.xacro -o assets/urdf/crx10ia.urdf
python3 tools/convert_meshes.py && python3 tools/build.py
```

See `THIRD_PARTY_NOTICES.md` for licenses of embedded robot assets.
