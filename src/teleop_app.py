"""Phase 4 -- slider teleop app for models/full_scene.xml.

Reproduces the reference video's interface: EE pose target sliders (X/Y/Z/Roll/Pitch/Yaw)
per hand driving the Phase 3 IK (src/ik.py) + Phase 3 arm torque control (src/arm_control.py),
grasp/thumb sliders per hand driving the Phase 2 synergy (src/grasp.py), a joint position
monitor, and an HUD (ik_err, sim/wall time, loop freq).

**GUI toolkit note**: dearpygui segfaults on import in this environment (Python 3.14, no
prebuilt wheel compatible with this ABI) and no working PyQt/Tk/GLFW+imgui context could be
established here either (see NOTES.md "Phase 4"), so the slider panel is a small static HTML
page served over loopback HTTP instead of an in-process GUI toolkit. This sidesteps the
GLFW/EGL context-sharing problem PLAN.md warns about entirely (the browser is a separate
process; there is no shared GL context to conflict with MuJoCo's own viewer window) and still
satisfies the same architectural constraint PLAN.md asks for: the browser thread only ever
writes into `TeleopState`'s target fields and reads its readout fields, never touches `data`
directly. The physics loop is the only thing that steps the simulation or writes `data.ctrl`.

Run: `python3 src/teleop_app.py` (from anywhere), then open http://localhost:8000 in a
browser. The MuJoCo viewer window opens separately for the 3D view.
Keyboard (focus the MuJoCo window): R = reset can (+-2cm random), G = toggle contact
force/point visualization, C = cycle camera preset (overview / right-hand close-up). The
same three actions also have buttons on the HTML page.
"""

import http.server
import json
import math
import pathlib
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))
MODEL_PATH = REPO_ROOT / "models" / "full_scene.xml"

import arm_control  # noqa: E402
import grasp  # noqa: E402
import ik  # noqa: E402

ARM_R = [f"arm_r_joint{i}" for i in range(1, 8)]
ARM_L = [f"arm_l_joint{i}" for i in range(1, 8)]
# Reused from tests/test_phase_3.py / tests/test_phase_4.py -- see NOTES.md "Phase 4" for
# why these carry over unchanged onto full_scene.xml (pure rigid translation, no rotation,
# between arm_hand.xml's fixed arm_base and this scene's lift-jointed one).
HOME_Q_R = np.array([-0.225, -0.394, 0.682, -2.613, -0.704, 0.843, -1.218])
HOME_Q_L = np.array([-0.2222, 0.3763, -0.4512, -1.2252, 0.8006, 0.9576, 0.0270])
LIFT_RANGE = (-0.5, 0.0)
MONITOR_JOINTS = (
    [f"arm_r_joint{i}" for i in range(1, 8)] + [f"arm_l_joint{i}" for i in range(1, 8)]
    + [f"finger_r_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + [f"finger_l_joint{i}" for i in (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)]
    + ["lift_joint", "head_joint1", "head_joint2"]
)

LOOP_HZ = 25.0
IK_MAX_ITER = 30

HTTP_PORT = 8000


def rpy_deg_to_quat(rpy_deg):
    r, p, y = (math.radians(v) for v in rpy_deg)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_rpy_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


class TeleopState:
    """Shared target/readout state. GUI (HTTP thread) only ever writes `pos`/`rpy_deg`/
    `grasp`/`thumb`/`lift`/`camera_preset`/`contact_viz` and reads the `*_readout` fields
    below; the physics loop is the mirror image -- this keeps the "GUI writes targets,
    physics writes readouts" one-way structure PLAN.md asks for explicit."""

    def __init__(self, pos, rpy_deg, lift):
        self._lock = threading.Lock()
        self.pos = {"l": list(pos["l"]), "r": list(pos["r"])}
        self.rpy_deg = {"l": list(rpy_deg["l"]), "r": list(rpy_deg["r"])}
        self.grasp = {"l": 0.0, "r": 0.0}
        self.thumb = {"l": 0.0, "r": 0.0}
        self.lift = lift
        self.camera_preset = 0
        self.contact_viz = False
        self._reset_can = False
        # readouts
        self.ik_err_mm = {"l": 0.0, "r": 0.0}
        self.sim_time = 0.0
        self.wall_time = 0.0
        self.freq_hz = 0.0
        self.joint_deg = {}

    def set_target(self, data):
        with self._lock:
            for side in ("l", "r"):
                if f"pos_{side}" in data:
                    self.pos[side] = [float(v) for v in data[f"pos_{side}"]]
                if f"rpy_{side}" in data:
                    self.rpy_deg[side] = [float(v) for v in data[f"rpy_{side}"]]
                if f"grasp_{side}" in data:
                    self.grasp[side] = float(np.clip(data[f"grasp_{side}"], 0.0, 1.0))
                if f"thumb_{side}" in data:
                    self.thumb[side] = float(np.clip(data[f"thumb_{side}"], 0.0, 1.0))
            if "lift" in data:
                self.lift = float(np.clip(data["lift"], *LIFT_RANGE))
            if "camera_preset" in data:
                self.camera_preset = int(data["camera_preset"])
            if "contact_viz" in data:
                self.contact_viz = bool(data["contact_viz"])

    def request_reset_can(self):
        with self._lock:
            self._reset_can = True

    def pop_reset_can(self):
        with self._lock:
            v = self._reset_can
            self._reset_can = False
            return v

    def toggle_contact_viz(self):
        with self._lock:
            self.contact_viz = not self.contact_viz

    def cycle_camera(self):
        with self._lock:
            self.camera_preset = 1 - self.camera_preset

    def snapshot_targets(self):
        with self._lock:
            return dict(
                pos={k: list(v) for k, v in self.pos.items()},
                rpy_deg={k: list(v) for k, v in self.rpy_deg.items()},
                grasp=dict(self.grasp), thumb=dict(self.thumb),
                lift=self.lift, camera_preset=self.camera_preset,
                contact_viz=self.contact_viz,
            )

    def update_readout(self, ik_err_mm, sim_time, wall_time, freq_hz, joint_deg):
        with self._lock:
            self.ik_err_mm = dict(ik_err_mm)
            self.sim_time = sim_time
            self.wall_time = wall_time
            self.freq_hz = freq_hz
            self.joint_deg = dict(joint_deg)

    def snapshot_all(self):
        with self._lock:
            return dict(
                pos={k: list(v) for k, v in self.pos.items()},
                rpy_deg={k: list(v) for k, v in self.rpy_deg.items()},
                grasp=dict(self.grasp), thumb=dict(self.thumb),
                lift=self.lift, camera_preset=self.camera_preset,
                contact_viz=self.contact_viz,
                ik_err_mm=dict(self.ik_err_mm), sim_time=self.sim_time,
                wall_time=self.wall_time, freq_hz=self.freq_hz,
                joint_deg=dict(self.joint_deg),
            )


INDEX_HTML = """<!doctype html><html><head><meta charset="utf-8">
<title>FFW-SH5 Teleop</title>
<style>
body{font-family:monospace;background:#1a1a1a;color:#ddd;margin:0;padding:12px;}
h2{color:#7ec8ff;margin:6px 0}
.cols{display:flex;gap:24px;flex-wrap:wrap}
.col{background:#242424;border-radius:8px;padding:10px 14px;min-width:280px}
.row{display:flex;align-items:center;gap:8px;margin:4px 0}
.row label{width:60px;color:#aaa}
.row input[type=range]{flex:1}
.row span{width:70px;text-align:right;color:#7ec8ff}
button{background:#333;color:#ddd;border:1px solid #555;border-radius:4px;padding:6px 10px;margin:4px 4px 4px 0;cursor:pointer}
button:hover{background:#3a5;}
#hud{background:#242424;border-radius:8px;padding:10px 14px;margin-bottom:12px}
#hud span{color:#7ec8ff;margin-right:18px}
#monitor{max-height:340px;overflow-y:auto;font-size:12px;background:#1c1c1c;padding:6px;border-radius:4px;margin-top:6px}
#monitor div{display:flex;justify-content:space-between;color:#999}
</style></head><body>
<h2>FFW-SH5 Slider Teleop (Phase 4)</h2>
<div id="hud">
  <span id="hud_sim">sim: 0.0s</span>
  <span id="hud_wall">wall: 0.0s</span>
  <span id="hud_freq">0.0 Hz</span>
  <span id="hud_ikl">IK-L: 0.0mm</span>
  <span id="hud_ikr">IK-R: 0.0mm</span>
</div>
<div class="cols">
  <div class="col" id="col_l"></div>
  <div class="col" id="col_r"></div>
  <div class="col">
    <h2>Lift / Utils</h2>
    <div class="row"><label>lift</label><input type="range" id="lift" min="-0.5" max="0" step="0.001"><span id="lift_v"></span></div>
    <button onclick="postAction('/reset_can')">Reset Can (R)</button>
    <button onclick="postAction('/toggle_contact')">Toggle Contact Viz (G)</button>
    <button onclick="postAction('/camera')">Cycle Camera (C)</button>
    <h2>Joint Monitor</h2>
    <div id="monitor"></div>
  </div>
</div>
<script>
const AXES = [["x","X",-0.2,1.0,0.01],["y","Y",-0.6,0.6,0.01],["z","Z",0.5,1.7,0.01],
              ["roll","Roll",-180,180,1],["pitch","Pitch",-180,180,1],["yaw","Yaw",-180,180,1]];
function buildCol(side, label){
  const col = document.getElementById("col_"+side);
  col.innerHTML = "<h2>"+label+"</h2>";
  for (const [key,name,lo,hi,step] of AXES){
    const id = side+"_"+key;
    col.innerHTML += `<div class="row"><label>${name}</label>
      <input type="range" id="${id}" min="${lo}" max="${hi}" step="${step}">
      <span id="${id}_v"></span></div>`;
  }
  for (const key of ["grasp","thumb"]){
    const id = side+"_"+key;
    col.innerHTML += `<div class="row"><label>${key}</label>
      <input type="range" id="${id}" min="0" max="1" step="0.01">
      <span id="${id}_v"></span></div>`;
  }
}
buildCol("l","Left Arm/Hand");
buildCol("r","Right Arm/Hand");

let dragging = new Set();
function wire(id, onInput){
  const el = document.getElementById(id);
  el.addEventListener("pointerdown", ()=>dragging.add(id));
  el.addEventListener("pointerup", ()=>dragging.delete(id));
  el.addEventListener("input", onInput);
}
for (const side of ["l","r"]){
  for (const [key] of AXES) wire(side+"_"+key, ()=>pushTarget(side));
  wire(side+"_grasp", ()=>pushTarget(side));
  wire(side+"_thumb", ()=>pushTarget(side));
}
wire("lift", ()=>{
  fetch("/target", {method:"POST", headers:{"Content-Type":"application/json"},
    body: JSON.stringify({lift: parseFloat(document.getElementById("lift").value)})});
});

function pushTarget(side){
  const g = id=>parseFloat(document.getElementById(id).value);
  const body = {};
  body["pos_"+side] = [g(side+"_x"), g(side+"_y"), g(side+"_z")];
  body["rpy_"+side] = [g(side+"_roll"), g(side+"_pitch"), g(side+"_yaw")];
  body["grasp_"+side] = g(side+"_grasp");
  body["thumb_"+side] = g(side+"_thumb");
  fetch("/target", {method:"POST", headers:{"Content-Type":"application/json"}, body: JSON.stringify(body)});
}
function postAction(path){ fetch(path, {method:"POST"}); }

async function poll(){
  try {
    const r = await fetch("/state");
    const s = await r.json();
    for (const side of ["l","r"]){
      const p = s.pos[side], rp = s.rpy_deg[side];
      const vals = {x:p[0], y:p[1], z:p[2], roll:rp[0], pitch:rp[1], yaw:rp[2],
                    grasp:s.grasp[side], thumb:s.thumb[side]};
      for (const [k,v] of Object.entries(vals)){
        const id = side+"_"+k;
        if (!dragging.has(id)) document.getElementById(id).value = v;
        document.getElementById(id+"_v").textContent = (typeof v === "number") ? v.toFixed(3) : v;
      }
    }
    if (!dragging.has("lift")) document.getElementById("lift").value = s.lift;
    document.getElementById("lift_v").textContent = s.lift.toFixed(3);
    document.getElementById("hud_sim").textContent = "sim: "+s.sim_time.toFixed(1)+"s";
    document.getElementById("hud_wall").textContent = "wall: "+s.wall_time.toFixed(1)+"s";
    document.getElementById("hud_freq").textContent = s.freq_hz.toFixed(1)+" Hz";
    document.getElementById("hud_ikl").textContent = "IK-L: "+s.ik_err_mm.l.toFixed(2)+"mm";
    document.getElementById("hud_ikr").textContent = "IK-R: "+s.ik_err_mm.r.toFixed(2)+"mm";
    const mon = document.getElementById("monitor");
    mon.innerHTML = Object.entries(s.joint_deg).map(([n,d])=>`<div><span>${n}</span><span>${d.toFixed(1)}</span></div>`).join("");
  } catch(e) {}
  setTimeout(poll, 150);
}
poll();
</script>
</body></html>"""


def make_handler(state):
    class Handler(http.server.BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send_json(self, obj, code=200):
            body = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/":
                body = INDEX_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            elif self.path == "/state":
                self._send_json(state.snapshot_all())
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b""
            if self.path == "/target":
                try:
                    data = json.loads(raw) if raw else {}
                except json.JSONDecodeError:
                    data = {}
                state.set_target(data)
                self._send_json({"ok": True})
            elif self.path == "/reset_can":
                state.request_reset_can()
                self._send_json({"ok": True})
            elif self.path == "/toggle_contact":
                state.toggle_contact_viz()
                self._send_json({"ok": True})
            elif self.path == "/camera":
                state.cycle_camera()
                self._send_json({"ok": True})
            else:
                self.send_response(404)
                self.end_headers()

    return Handler


def _set_camera_preset(cam, preset):
    if preset == 0:  # overview
        cam.lookat[:] = [0.3, 0.0, 1.0]
        cam.distance = 2.2
        cam.azimuth = 120
        cam.elevation = -20
    else:  # right-hand close-up
        cam.lookat[:] = [0.5055, 0.0, 0.85]
        cam.distance = 0.5
        cam.azimuth = 90
        cam.elevation = -15


def _apply_contact_viz(viewer_opt, on):
    viewer_opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = on
    viewer_opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = on


def _reset_can_random(model, data, rng):
    """The one qpos write in this file outside of the initial keyframe reset -- resetting a
    freely-placed object's spawn pose is the explicit exception PLAN.md's rule 1 carves out,
    not a kinematic override of the robot itself."""
    can_jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "can_free")
    qadr = model.jnt_qposadr[can_jid]
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    home_can_pos = model.key_qpos[key_id][qadr:qadr + 3].copy()
    data.qpos[qadr:qadr + 3] = home_can_pos + rng.uniform(-0.02, 0.02, size=3)
    data.qpos[qadr + 3:qadr + 7] = [1, 0, 0, 0]
    dof = model.jnt_dofadr[can_jid]
    data.qvel[dof:dof + 6] = 0.0


def main():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)

    solver_r = ik.InverseKinematics(model, "grasp_target_r", ARM_R)
    solver_l = ik.InverseKinematics(model, "grasp_target_l", ARM_L)
    ctrl_r = arm_control.ArmTorqueController(model, ARM_R)
    ctrl_l = arm_control.ArmTorqueController(model, ARM_L)
    lift_aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "lift_joint")
    monitor_qposadr = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)]
                        for n in MONITOR_JOINTS}
    rng = np.random.default_rng()

    site_r = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_r")
    site_l = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "grasp_target_l")
    init_quat_r = np.zeros(4)
    mujoco.mju_mat2Quat(init_quat_r, data.site_xmat[site_r])
    init_quat_l = np.zeros(4)
    mujoco.mju_mat2Quat(init_quat_l, data.site_xmat[site_l])

    state = TeleopState(
        pos={"l": data.site_xpos[site_l].tolist(), "r": data.site_xpos[site_r].tolist()},
        rpy_deg={"l": quat_to_rpy_deg(init_quat_l), "r": quat_to_rpy_deg(init_quat_r)},
        lift=float(data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "lift_joint")]]),
    )

    httpd = http.server.ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), make_handler(state))
    http_thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    http_thread.start()
    print(f"Teleop control panel: http://localhost:{HTTP_PORT}")

    q_des_r = HOME_Q_R.copy()
    q_des_l = HOME_Q_L.copy()
    frame_dt = 1.0 / LOOP_HZ
    steps_per_frame = max(1, round(frame_dt / model.opt.timestep))
    freq_ema = LOOP_HZ
    wall_start = time.perf_counter()

    def on_key(keycode):
        if keycode == ord("R"):
            state.request_reset_can()
        elif keycode == ord("G"):
            state.toggle_contact_viz()
        elif keycode == ord("C"):
            state.cycle_camera()

    with mujoco.viewer.launch_passive(model, data, key_callback=on_key) as viewer:
        _set_camera_preset(viewer.cam, 0)
        while viewer.is_running():
            t0 = time.perf_counter()
            targets = state.snapshot_targets()

            if state.pop_reset_can():
                _reset_can_random(model, data, rng)
                mujoco.mj_forward(model, data)

            ctx = data.qpos.copy()
            quat_r = rpy_deg_to_quat(targets["rpy_deg"]["r"])
            quat_l = rpy_deg_to_quat(targets["rpy_deg"]["l"])
            q_des_r, perr_r, _ = solver_r.solve_pose(
                q_des_r, np.array(targets["pos"]["r"]), quat_r,
                max_iter=IK_MAX_ITER, context_qpos=ctx)
            q_des_l, perr_l, _ = solver_l.solve_pose(
                q_des_l, np.array(targets["pos"]["l"]), quat_l,
                max_iter=IK_MAX_ITER, context_qpos=ctx)

            for _ in range(steps_per_frame):
                ctrl_r.apply(data, q_des_r)
                ctrl_l.apply(data, q_des_l)
                data.ctrl[lift_aid] = targets["lift"]
                grasp.apply_grasp(model, data, grasp=targets["grasp"]["r"], thumb=targets["thumb"]["r"], side="r")
                grasp.apply_grasp(model, data, grasp=targets["grasp"]["l"], thumb=targets["thumb"]["l"], side="l")
                mujoco.mj_step(model, data)

            _set_camera_preset(viewer.cam, targets["camera_preset"])
            _apply_contact_viz(viewer.opt, targets["contact_viz"])
            viewer.sync()

            elapsed = time.perf_counter() - t0
            freq_ema = 0.9 * freq_ema + 0.1 * (1.0 / max(elapsed, 1e-6))
            joint_deg = {n: math.degrees(float(data.qpos[a])) for n, a in monitor_qposadr.items()}
            state.update_readout(
                ik_err_mm={"l": perr_l * 1000.0, "r": perr_r * 1000.0},
                sim_time=data.time, wall_time=time.perf_counter() - wall_start,
                freq_hz=freq_ema, joint_deg=joint_deg,
            )

            sleep_time = frame_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    httpd.shutdown()


if __name__ == "__main__":
    main()
