"""Bundle src/*.js + assets/robots.json into a single self-contained index.html."""
import re, pathlib
ROOT = pathlib.Path(__file__).resolve().parent.parent
order = ["contract", "kinematics", "sim_ros2", "adapters", "manifests", "platform", "app"]
src = {f: (ROOT / "src" / f"{f}.js").read_text() for f in order}
code = lambda s: [l for l in s.splitlines() if l.strip() and not l.strip().startswith(("//", "*", "/*"))]
cls = lambda n: re.search(r"(  class " + n + r".*?\n  \}\n)", src["adapters"], re.S).group(1)
rows = [("Shared platform", len(code(src["contract"] + src["kinematics"] + src["platform"])), ""),
        ("Adapter SDK (ros2_control arms)", len(code(cls("Ros2ArmAdapter"))), ""),
        ("URAdapter", len(code(cls("URAdapter"))), "oem"), ("FanucAdapter", len(code(cls("FanucAdapter"))), "oem"),
        ("Manifests + site profile (data)", len(code(src["manifests"])), ""),
        ("Simulated ROS 2 drivers", len(code(src["sim_ros2"])), ""), ("Operator UI", len(code(src["app"])), "")]
mx = max(r[1] for r in rows)
bars = "".join(f"<div class=\"bar {c}\"><span>{n}</span><div class=\"track\"><div class=\"fill\" style=\"width:{v/mx*100:.1f}%\"></div></div><span class=\"n\">{v}</span></div>" for n, v, c in rows)
data = (ROOT / "assets" / "robots.json").read_text().replace("</", "<\\/")
version = re.search(r"VERSION: \x27([\d.]+)\x27", src["contract"]).group(1)
html = ((ROOT / "web" / "shell.html").read_text().replace("__BARS__", bars).replace("__DATA__", data)
        .replace("__SRC__", "\n".join(src[f] for f in order)).replace("__VERSION__", version))
(ROOT / "index.html").write_text(html)
print(f"index.html written ({len(html)/1e6:.2f} MB)")
for r in rows: print(f"  {r[0]:<34}{r[1]:>5} lines")
