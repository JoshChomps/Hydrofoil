"""
The hydrofoil simulator app: native MuJoCo window with live 3-D view, sliders, plots and actuator loads.
(Replaces both the old Unity visualizer and the browser dashboard.)

MuJoCo is used purely as a renderer and UI: hydrofoil.vehicle.VehiclePhysics remains the only
physics authority. Each frame we write its pose into the MuJoCo model (qpos) and call
mj_forward -- mj_step is never called.

Run:
    python -m hydrofoil.viewer [--waves] [--wave-dir 90] [--speed 1.0] [--duration 600]

Sliders: open the right-hand panel's "Control" section. Each slider is a zero-gain MuJoCo actuator
used only as a UI control; its value is read every frame:
    height_ref, bank_ref, speed_ref                 pilot references (bank holds when set here)
    wave_amp, wave_len, wave_dir                    Airy wave (amp 0 = calm); applied live
    kp/ki/kd_heave, kp/kd_pitch, kp/kd_roll         PID gains, applied live (no restart)
    gear_ratio, v_bus                               drive model used for the load read-out

Keys (click the viewer window first):
    Up / Down        climb / dive   (height_ref, holds)
    Left / Right     bank left / right (bank_ref, +-8 deg, springs back to 0)
    [  /  ]          speed_ref down / up (holds)
    R                reset to rest

Left of the window: plots (height vs target, roll/pitch, surface angles, drive currents).
Top-left text: state, and per drive the hinge moment, motor current and bus power (docs/17).

Frames: the sim is NED/body (x fwd, y stbd, z down). MuJoCo is Z-up, so world
coordinates are mapped with D = diag(1,-1,-1) (a proper 180 deg rotation about x):
    p_mj = D p_ned,   R_mj = D R_ned B   with B = diag(1,-1,-1) mapping the
MuJoCo body frame (x fwd, y left, z up) onto the sim's body frame.
"""

from __future__ import annotations

import argparse
import time
from collections import deque

import numpy as np
import mujoco
import mujoco.viewer

from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.dynamics.kinematics import R_body_to_ned
from hydrofoil.waves.airy import AiryWave, CalmWater
from hydrofoil.actuation.drive import DriveSpec, electrical_demand
from hydrofoil.actuation.loads import DRIVES, surface_models, drive_angles, steady_moments

# Pilot limits -- keep identical to the verified/capped ranges used elsewhere.
HEIGHT_MIN, HEIGHT_MAX, HEIGHT_STEP = -0.3, 1.0, 0.02   # m, per key event
SPEED_MIN, SPEED_MAX, SPEED_STEP = 0.0, 10.0, 0.25       # m/s, per key event
BANK_MIN, BANK_MAX, BANK_STEP = -8.0, 8.0, 1.0           # deg, per key event
BANK_RETURN_RATE = 10.0                                   # deg/s spring-back

# UI sliders: (name, param key or None, lo, hi). Order = MuJoCo actuator / ctrl index.
SLIDERS = [
    ("height_ref", "flying_height", HEIGHT_MIN, HEIGHT_MAX),
    ("bank_ref", "phi_ref", BANK_MIN, BANK_MAX),
    ("speed_ref", "target_speed", SPEED_MIN, SPEED_MAX),
    ("wave_amp", "wave_amplitude", 0.0, 0.20),
    ("wave_len", "wave_wavelength", 1.0, 20.0),
    ("wave_dir", "wave_direction", -180.0, 180.0),
    ("kp_heave", "kp_heave", 0.0, 100.0),
    ("ki_heave", "ki_heave", 0.0, 20.0),
    ("kd_heave", "kd_heave", 0.0, 30.0),
    ("kp_pitch", "kp_pitch", 0.0, 5.0),
    ("kd_pitch", "kd_pitch", 0.0, 1.0),
    ("kp_roll", "kp_roll", 0.0, 10.0),
    ("kd_roll", "kd_roll", 0.0, 1.0),
    ("gear_ratio", "gear_ratio", 5.0, 50.0),
    ("v_bus", "v_bus", 10.0, 44.0),
]
IDX = {name: i for i, (name, *_rest) in enumerate(SLIDERS)}

_D = np.diag([1.0, -1.0, -1.0])   # NED <-> Z-up world
_B = np.diag([1.0, -1.0, -1.0])   # sim body frame <-> MuJoCo body frame


def build_mjcf(p: dict) -> str:
    """Generate the MJCF for hull + struts + main foil halves (with flaps) + elevator + UI sliders."""
    hull_off = p["hull_offset"]
    mc, ms = p["main_chord"], p["main_span"]
    ec, es = p["elev_chord"], p["elev_span"]
    lat = p["main_lateral_offset"]
    mx, mz = p["main_pos_x"], p["main_pos_z"]
    ex, ez = p["elev_pos_x"], p["elev_pos_z"]
    flap_frac = p.get("main_flap_frac", 0.4)

    def half(name, y):
        return f"""
        <body name="{name}" pos="{mx} {y} {-mz}">
          <geom type="box" size="{mc*(1-flap_frac)/2} {ms/2} 0.008" pos="{mc*flap_frac/2} 0 0" material="foil"/>
          <body name="{name}_flap" pos="{-mc*(1-flap_frac)/2 + mc*flap_frac/2} 0 0">
            <joint name="{name}_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>
            <geom type="box" size="{mc*flap_frac/2} {ms/2} 0.008" pos="{-mc*flap_frac/2} 0 0" material="flap"/>
          </body>
        </body>"""

    # Zero-gain actuators: they only exist to put sliders in the viewer's Control panel.
    sliders = "\n".join(
        f'    <general name="{n}" joint="elev_hinge" gainprm="0" biasprm="0 0 0" ctrllimited="true" ctrlrange="{lo} {hi}"/>'
        for n, _k, lo, hi in SLIDERS)

    strut_len = hull_off + mz
    return f"""
<mujoco model="hydrofoil">
  <option gravity="0 0 0"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <headlight ambient="0.45 0.45 0.45" diffuse="0.6 0.6 0.6"/>
    <map znear="0.01" zfar="500"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".12 .20 .28" rgb2=".16 .26 .36" width="256" height="256" mark="edge" markrgb=".4 .5 .6"/>
    <material name="grid" texture="grid" texrepeat="6 6" texuniform="true" reflectance="0.05"/>
    <material name="hull" rgba="0.92 0.92 0.95 1"/>
    <material name="strut" rgba="0.35 0.38 0.42 1"/>
    <material name="foil" rgba="0.10 0.35 0.75 1"/>
    <material name="flap" rgba="0.95 0.45 0.10 1"/>
    <material name="water" rgba="0.25 0.55 0.85 0.35"/>
  </asset>
  <worldbody>
    <light pos="0 0 6" dir="0 0 -1" directional="true"/>
    <geom name="seabed" type="plane" pos="0 0 -3" size="0 0 0.1" material="grid"/>
    <body name="water" mocap="true" pos="0 0 0">
      <geom type="box" size="60 60 0.01" material="water" contype="0" conaffinity="0"/>
    </body>
    <body name="vehicle" pos="0 0 0">
      <freejoint name="root"/>
      <geom name="hull" type="box" size="{p["hull_length"]/2} {p["hull_beam"]/2} 0.08" pos="{p["hull_x"]} 0 {hull_off}" material="hull"/>
      <geom name="main_strut" type="box" size="0.015 0.04 {strut_len/2}" pos="{mx} 0 {hull_off - strut_len/2}" material="strut"/>
      <geom name="cross_bar" type="box" size="0.015 {lat} 0.01" pos="{mx} 0 {-mz}" material="strut"/>
      <geom name="elev_strut" type="box" size="0.012 0.03 {(hull_off+ez)/2}" pos="{ex} 0 {hull_off - (hull_off+ez)/2}" material="strut"/>
      <geom name="cog" type="sphere" size="0.03" pos="0 0 0" rgba="1 0 0 1" contype="0" conaffinity="0"/>
      {half("left_foil", lat)}
      {half("right_foil", -lat)}
      <body name="elevator" pos="{ex} 0 {-ez}">
        <joint name="elev_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>
        <geom type="box" size="{ec/2} {es/2} 0.006" material="foil"/>
      </body>
    </body>
  </worldbody>
  <actuator>
{sliders}
  </actuator>
</mujoco>"""


class MujocoPose:
    """Maps a sim state + flap commands onto the MuJoCo model's qpos."""

    def __init__(self, params: dict):
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(params))
        self.data = mujoco.MjData(self.model)
        m = self.model
        self.vehicle_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "vehicle")
        j = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
        self.q_left, self.q_right, self.q_elev = j("left_foil_hinge"), j("right_foil_hinge"), j("elev_hinge")
        self.water_mocap = m.body_mocapid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "water")]

    def apply(self, state: np.ndarray, cmds: dict, wave_elev: float = 0.0):
        eta = state[:6]
        R_mj = _D @ R_body_to_ned(eta[3], eta[4], eta[5]) @ _B
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_mj.ravel())
        d = self.data
        d.qpos[0:3] = _D @ eta[:3]
        d.qpos[3:7] = quat
        # Positive sim flap deflection = more AoA = trailing edge down. In the MuJoCo
        # body frame (y left) a positive hinge rotation lowers the *leading* edge, so negate.
        ang = drive_angles(cmds)
        d.qpos[self.q_left] = -np.radians(ang["left"])
        d.qpos[self.q_right] = -np.radians(ang["right"])
        d.qpos[self.q_elev] = -np.radians(ang["elev"])
        d.mocap_pos[self.water_mocap] = [eta[0], -eta[1], wave_elev]
        mujoco.mj_forward(self.model, d)


class Pilot:
    """Keyboard stick. Writes into the slider values (ctrl) so keys and sliders stay one source."""

    def __init__(self, ctrl: np.ndarray):
        self.ctrl = ctrl
        self.reset_flag = False
        self.bank_spring = False   # True after a key bank input; a slider-set bank holds
        self.key_bank_value = None

    def _nudge(self, name, step):
        _n, _k, lo, hi = SLIDERS[IDX[name]]
        self.ctrl[IDX[name]] = float(np.clip(self.ctrl[IDX[name]] + step, lo, hi))

    def key(self, code: int):
        if code == 265:      # up
            self._nudge("height_ref", HEIGHT_STEP)
        elif code == 264:    # down
            self._nudge("height_ref", -HEIGHT_STEP)
        elif code in (262, 263):    # right / left -> bank right / left (springs back)
            self._nudge("bank_ref", BANK_STEP if code == 262 else -BANK_STEP)
            self.bank_spring = True
            self.key_bank_value = float(self.ctrl[IDX["bank_ref"]])
        elif code == 93:     # ]
            self._nudge("speed_ref", SPEED_STEP)
        elif code == 91:     # [
            self._nudge("speed_ref", -SPEED_STEP)
        elif code in (ord("R"), ord("r")):
            self.reset_flag = True

    def spring_back(self, dt: float):
        if not self.bank_spring:
            return
        i = IDX["bank_ref"]
        phi, step = self.ctrl[i], BANK_RETURN_RATE * dt
        self.ctrl[i] = max(0.0, phi - step) if phi > 0 else min(0.0, phi + step)
        if self.ctrl[i] == 0.0:
            self.bank_spring = False


class DriveMonitor:
    """Per-step hinge moment / motor current / bus power for the three physical drives (docs/17)."""

    def __init__(self, params: dict):
        self.hinges = surface_models(params)
        self.spec = DriveSpec()
        self.prev = None          # (angles rad, rates rad/s) of the previous step
        self.out = {d: {"M": 0.0, "I": 0.0, "P": 0.0, "tau": 0.0} for d in DRIVES}

    def reset(self):
        self.prev = None

    def update(self, vp, state, cmds, env, dt: float, gear_ratio: float, v_bus: float):
        self.spec.gear.ratio, self.spec.ctrl.v_bus = gear_ratio, v_bus
        ang = {k: np.radians(v) for k, v in drive_angles(cmds).items()}
        m = steady_moments(vp, state, cmds, env, self.hinges)
        if self.prev is None:
            rates = {d: 0.0 for d in DRIVES}
            acc = {d: 0.0 for d in DRIVES}
        else:
            a0, r0 = self.prev
            rates = {d: (ang[d] - a0[d]) / dt for d in DRIVES}
            acc = {d: (rates[d] - r0[d]) / dt for d in DRIVES}
        self.prev = (ang, rates)
        for d in DRIVES:
            h = self.hinges[d]
            tau = ((h.g.j_struct + h.i_added) * acc[d] + h.damping(m[d + "_V"]) * rates[d]
                   + self.spec.gear.friction_out * np.sign(rates[d]) - m[d])
            e = electrical_demand(self.spec, rates[d], acc[d], tau)
            self.out[d] = {"M": m[d], "tau": float(tau), "I": float(e["i_q"]), "P": float(e["p_bus"]),
                           "V": float(e["v_needed"])}
        return self.out

    def flags(self) -> str:
        c = self.spec.ctrl
        bad = [d for d in DRIVES if abs(self.out[d]["I"]) > c.i_peak or self.out[d].get("V", 0) > c.k_mod * c.v_bus]
        return "LIMIT: " + ",".join(bad) if bad else "within limits"


class Plots:
    """Rolling MuJoCo figures drawn on the left edge of the window."""

    SPECS = [
        ("height above water [m]", ("height", "target")),
        ("roll / pitch [deg]", ("roll", "pitch")),
        ("surface angle [deg]", ("left", "right", "elev")),
        ("drive current [A]", ("I_left", "I_right", "I_elev")),
    ]
    COLORS = [(0.2, 0.7, 1.0), (1.0, 0.6, 0.2), (0.6, 1.0, 0.4)]

    def __init__(self, window_s: float = 15.0, rate_hz: float = 30.0):
        self.n = int(window_s * rate_hz)
        self.min_dt = 1.0 / rate_hz
        self.last_t = -1e9
        self.buf = {k: deque(maxlen=self.n) for _t, keys in self.SPECS for k in keys}
        self.t = deque(maxlen=self.n)
        self.figs = []
        for title, keys in self.SPECS:
            f = mujoco.MjvFigure()
            mujoco.mjv_defaultFigure(f)
            f.title = title
            f.flg_extend = 1
            f.figurergba[3] = 0.6
            f.gridsize = [3, 3]
            for i, k in enumerate(keys):
                f.linename[i] = k
                f.linergb[i] = self.COLORS[i]
            self.figs.append(f)

    def clear(self):
        for b in self.buf.values():
            b.clear()
        self.t.clear()
        self.last_t = -1e9

    def push(self, t: float, values: dict):
        if t - self.last_t < self.min_dt:
            return
        self.last_t = t
        self.t.append(t)
        for k, v in values.items():
            self.buf[k].append(v)

    def figures(self, viewport):
        if viewport is None or len(self.t) < 2:
            return []
        w = max(200, viewport.width // 4)
        h = viewport.height // len(self.figs)
        tt = np.asarray(self.t)
        out = []
        for j, ((_title, keys), f) in enumerate(zip(self.SPECS, self.figs)):
            for i, k in enumerate(keys):
                y = np.asarray(self.buf[k])
                n = len(y)
                f.linepnt[i] = n
                f.linedata[i][0:2 * n:2] = tt[-n:]
                f.linedata[i][1:2 * n:2] = y
            rect = mujoco.MjrRect(0, viewport.height - (j + 1) * h, w, h)
            out.append((rect, f))
        return out


def _wave_from(p: dict):
    if p["wave_amplitude"] <= 1e-4:
        return CalmWater()
    return AiryWave(amplitude=p["wave_amplitude"], wavelength=p["wave_wavelength"],
                    direction=np.radians(p["wave_direction"]))


def _apply_gains(vp: VehiclePhysics, p: dict):
    c = vp.controller
    c.heave_pid.Kp, c.heave_pid.Ki, c.heave_pid.Kd = p["kp_heave"], p["ki_heave"], p["kd_heave"]
    c.pitch_pid.Kp, c.pitch_pid.Kd = p["kp_pitch"], p["kd_pitch"]
    c.roll_pid.Kp, c.roll_pid.Kd = p["kp_roll"], p["kd_roll"]


def run(speed: float = 1.0, wave_cfg: dict | None = None, duration: float = 600.0):
    params = dict(DEFAULT_PARAMS)
    params["v_bus"] = DriveSpec().ctrl.v_bus
    if wave_cfg:
        params.update(wave_cfg)
    else:
        params["wave_amplitude"] = 0.0
    dt = params["dt"]

    vp = VehiclePhysics(params)
    pose = MujocoPose(params)
    ctrl = pose.data.ctrl
    for i, (_n, key, _lo, _hi) in enumerate(SLIDERS):
        ctrl[i] = params[key]
    pilot = Pilot(ctrl)
    drives = DriveMonitor(params)
    plots = Plots()

    wave = _wave_from(params)
    wave_key = (params["wave_amplitude"], params["wave_wavelength"], params["wave_direction"])
    state = vp.rest_state()
    t = 0.0
    cmds = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    thrust, draft, wave_elev = 0.0, 0.0, 0.0
    pose.apply(state, cmds)
    last_bank_written = ctrl[IDX["bank_ref"]]

    with mujoco.viewer.launch_passive(pose.model, pose.data, key_callback=pilot.key) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = pose.vehicle_id
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 4.5, 140.0, -18.0

        wall0 = time.perf_counter()
        last_wall = wall0
        while v.is_running() and t < duration:
            now = time.perf_counter()
            with v.lock():
                # A bank value that changed without a key press came from the slider: hold it.
                b = float(ctrl[IDX["bank_ref"]])
                if abs(b - last_bank_written) > 1e-9 and b != pilot.key_bank_value:
                    pilot.bank_spring = False
                pilot.spring_back(now - last_wall)
                last_bank_written = ctrl[IDX["bank_ref"]]
                for i, (_n, key, _lo, _hi) in enumerate(SLIDERS):
                    params[key] = float(ctrl[i])
            last_wall = now
            _apply_gains(vp, params)
            wk = (params["wave_amplitude"], params["wave_wavelength"], params["wave_direction"])
            if wk != wave_key:
                wave, wave_key = _wave_from(params), wk

            if pilot.reset_flag:
                pilot.reset_flag = False
                vp.controller.reset(); vp.surge.reset(); drives.reset(); plots.clear()
                state, t = vp.rest_state(), 0.0
                wall0 = now

            target_t = (now - wall0) * speed
            steps = 0
            while t < target_t and steps < 50:     # cap catch-up so a slow frame can't spiral
                state, cmds, thrust, draft, wave_elev = vp.step(state, t, wave)
                t += dt
                steps += 1
                env = vp.wave_sample(state[:6], t, wave)
                dm = drives.update(vp, state, cmds, env, dt, params["gear_ratio"], params["v_bus"])
                ang = drive_angles(cmds)
                plots.push(t, {
                    "height": -state[2] + params["hull_offset"], "target": params["flying_height"],
                    "roll": np.degrees(state[3]), "pitch": np.degrees(state[4]),
                    "left": ang["left"], "right": ang["right"], "elev": ang["elev"],
                    "I_left": dm["left"]["I"], "I_right": dm["right"]["I"], "I_elev": dm["elev"]["I"],
                })
            if steps == 50:
                wall0 = now - t / speed             # drop the backlog instead of racing to catch up

            with v.lock():
                pose.apply(state, cmds, wave_elev)
            dm = drives.out
            mode = "FOILBORNE" if draft <= 1e-6 else "DISPLACEMENT"
            v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
                         "t / mode / speed (target)\nheight above water (target)\nroll / pitch (bank ref)\n"
                         "thrust / draft\n\ndrive  hinge M / motor I / bus P\n  left\n  right\n  elev\n"
                         f"N = {params['gear_ratio']:.0f}:1, V_bus = {params['v_bus']:.0f} V",
                         f"{t:6.1f}s  {mode}  {state[6]:.2f} ({params['target_speed']:.2f}) m/s\n"
                         f"{-(state[2]) + params['hull_offset']:.2f} ({params['flying_height']:.2f}) m\n"
                         f"{np.degrees(state[3]):.1f} / {np.degrees(state[4]):.1f} deg ({params['phi_ref']:.1f})\n"
                         f"{thrust:.1f} N / {draft:.3f} m\n\n\n"
                         + "".join(f"{dm[d]['M']:+6.2f} Nm  {dm[d]['I']:+6.1f} A  {dm[d]['P']:6.1f} W\n" for d in DRIVES)
                         + drives.flags()))
            v.set_figures(plots.figures(v.viewport))
            v.sync()
            time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))


def main():
    ap = argparse.ArgumentParser(description="Hydrofoil simulator (native MuJoCo window)")
    ap.add_argument("--speed", type=float, default=1.0, help="sim speed multiplier")
    ap.add_argument("--duration", type=float, default=600.0, help="run length [sim s]")
    ap.add_argument("--waves", action="store_true", help="start with an Airy wave (amp/len/dir also on sliders)")
    ap.add_argument("--wave-dir", type=float, default=0.0, help="wave travel direction [deg from +x]; try 90 for beam seas")
    a = ap.parse_args()
    wave_cfg = dict(wave_amplitude=0.05, wave_wavelength=5.0, wave_direction=a.wave_dir) if a.waves else None
    run(speed=a.speed, wave_cfg=wave_cfg, duration=a.duration)


if __name__ == "__main__":
    main()
