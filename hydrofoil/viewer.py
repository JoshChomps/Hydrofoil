"""
The hydrofoil simulator app: native MuJoCo window with live 3-D view, sliders, plots and actuator loads.
(Replaces both the old Unity visualizer and the browser dashboard.)

MuJoCo is used purely as a renderer and UI: hydrofoil.vehicle.VehiclePhysics remains the only
physics authority. Each frame we write its pose into the MuJoCo model (qpos) and call
mj_forward -- mj_step is never called.

Run:
    python -m hydrofoil.viewer [--waves] [--wave-dir 90] [--speed 1.0] [--duration 600]
                               [--controller lqr|pid] [--sensors ideal|noisy] [--preset model|manned]
                               [--supervisor] [--windowed] [--api-port 8766]

The window opens maximised (Windows) with MuJoCo's own menus hidden: 3-D view, a status panel
(bottom left; shortened in small windows), the flight mode (top left) and four rolling plots (right).
The water is a heightfield following the boat, refilled from the wave field every frame.

Live controls: a local HTTP API on 127.0.0.1:<api-port> (GET /state, GET /sliders, POST /set) that a
dashboard page can use (the gitignored local/ launcher serves one on :8765). Slider values are kept in
zero-gain MuJoCo actuators (ctrl), shared by the keys and the API, and read every frame:
    height_ref, bank_ref, speed_ref                 pilot references (bank holds when set here)
    wave_amp, wave_len, wave_dir                    Airy wave (amp 0 = calm); applied live
    kp/ki/kd_heave, kp/kd_pitch, kp/kd_roll         PID gains, applied live (no restart)
    gear_ratio, v_bus                               drive model used for the load read-out

Keys (click the viewer window first):
    Up / Down        climb / dive   (height_ref, holds)
    Left / Right     bank left / right (bank_ref, +-8 deg, springs back to 0)
    [  /  ]          speed_ref down / up (holds)
    R                reset to rest
    C                switch PID <-> LQR (restarts from rest; the PID always flies takeoff)
    F                toggle Foil / Land request (when --supervisor is enabled)
    U                show speeds in m/s or km/h

The PID gain sliders act on the PID loops, which fly takeoff in both modes and everything in PID mode.

Per drive the status panel shows the hinge moment, motor current and bus power (docs/17).
With actuator_model = "servo" (default) these are the simulated drives' own values and the drives'
limits act on the flight; with "kinematic" they are estimated from the flap trajectory.

Frames: the sim is NED/body (x fwd, y stbd, z down). MuJoCo is Z-up, so world
coordinates are mapped with D = diag(1,-1,-1) (a proper 180 deg rotation about x):
    p_mj = D p_ned,   R_mj = D R_ned B   with B = diag(1,-1,-1) mapping the
MuJoCo body frame (x fwd, y left, z up) onto the sim's body frame.
"""

from __future__ import annotations

from pathlib import Path

import argparse
import json
import time
from collections import deque

import numpy as np
import mujoco
import mujoco.viewer

from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.dynamics.kinematics import R_body_to_ned
from hydrofoil.waves.airy import AiryWave, CalmWater
from hydrofoil.actuation.drive import DriveSpec, electrical_demand
from hydrofoil.actuation.loads import DRIVES, surface_models, drive_angles, steady_moments

# Pilot limits -- keep identical to the verified/capped ranges used elsewhere.
DASHBOARD_URL = "http://127.0.0.1:8765"
HEIGHT_MIN, HEIGHT_MAX, HEIGHT_STEP = -0.3, 1.0, 0.02   # m, per key event
SPEED_MIN, SPEED_MAX, SPEED_STEP = 0.0, 10.0, 0.25       # m/s, per key event
BANK_MIN, BANK_MAX, BANK_STEP = -8.0, 8.0, 1.0           # deg, per key event
BANK_RETURN_RATE = 10.0                                   # deg/s spring-back

# UI sliders: (name, param key or None, lo, hi). Order = MuJoCo actuator / ctrl index.
SLIDERS = [
    ("height_ref", "flying_height", HEIGHT_MIN, HEIGHT_MAX),
    ("bank_ref", "phi_ref", BANK_MIN, BANK_MAX),
    ("speed_ref", "target_speed", SPEED_MIN, SPEED_MAX),
    ("wave_amp", "wave_amplitude", 0.0, 0.30),
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

WATER_N, WATER_HALF, WATER_RANGE = 128, 30.0, 0.6   # water heightfield: grid points, half-size [m], +- elevation [m]

_D = np.diag([1.0, -1.0, -1.0])   # NED <-> Z-up world
_B = np.diag([1.0, -1.0, -1.0])   # sim body frame <-> MuJoCo body frame


def _hull_vertices(L: float, B: float, D: float) -> str:
    """A simple planing hull as a convex point set (MuJoCo takes the convex hull): square transom, hard chines,
    a raked bow. Origin at the middle of the bottom, x forward, z up."""
    pts = [(-L / 2, y * B / 2, D) for y in (-1, 1)] + [(-L / 2, y * 0.36 * B, 0.0) for y in (-1, 1)]
    pts += [(0.18 * L, y * B / 2, D) for y in (-1, 1)] + [(0.18 * L, y * 0.36 * B, 0.0) for y in (-1, 1)]
    pts += [(0.38 * L, y * 0.33 * B, D) for y in (-1, 1)] + [(0.30 * L, y * 0.16 * B, 0.12 * D) for y in (-1, 1)]
    pts += [(L / 2, 0.0, 1.08 * D), (0.44 * L, 0.0, 0.3 * D)]
    return " ".join(f"{x:.4f} {y:.4f} {z:.4f}" for x, y, z in pts)


def build_mjcf(p: dict) -> str:
    """Generate the MJCF for hull + struts + main foil halves (with flaps) + elevator + UI sliders."""
    hull_off = p["hull_offset"]
    mc, ms = p["main_chord"], p["main_span"]
    ec, es = p["elev_chord"], p["elev_span"]
    lat = p["main_lateral_offset"]
    mx, mz = p["main_pos_x"], p["main_pos_z"]
    ex, ez = p["elev_pos_x"], p["elev_pos_z"]
    flap_frac = p.get("main_flap_frac", 0.4)

    fs = ms * p.get("main_flap_span_frac", 1.0)      # flap span (inboard part of each half)

    def half(name, y):
        # y is the half's centre (MuJoCo y = left); the flap sits on the INBOARD side (toward y = 0)
        inboard = -1.0 if y > 0 else 1.0
        fy = inboard * (ms - fs) / 2                   # flap centre relative to the half's centre
        oy = -inboard * fs / 2                         # fixed trailing-edge strip on the outboard part
        return f"""
        <body name="{name}" pos="{mx} {y} {-mz}">
          <geom type="box" size="{mc*(1-flap_frac)/2} {ms/2} 0.008" pos="{mc*flap_frac/2} 0 0" material="foil"/>
          <geom type="box" size="{mc*flap_frac/2} {(ms-fs)/2 + 1e-4} 0.008" pos="{-mc*(1-flap_frac)/2} {oy} 0" material="foil"/>
          <body name="{name}_flap" pos="{-mc*(1-flap_frac)/2 + mc*flap_frac/2} {fy} 0">
            <joint name="{name}_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>
            <geom type="box" size="{mc*flap_frac/2} {fs/2} 0.008" pos="{-mc*flap_frac/2} 0 0" material="flap"/>
          </body>
        </body>"""

    # Zero-gain actuators: they only exist to put sliders in the viewer's Control panel.
    sliders = "\n".join(
        f'    <general name="{n}" joint="elev_hinge" gainprm="0" biasprm="0 0 0" ctrllimited="true" ctrlrange="{lo} {hi}"/>'
        for n, _k, lo, hi in SLIDERS)

    strut_len = hull_off + mz
    L, B = p["hull_length"], p["hull_beam"]
    D = max(0.12, 0.09 * L)                              # hull depth, visual only
    # Struts drawn at their real chord (fore-aft) and ~12 % thickness, meeting each foil at a streamlined pod;
    # the two main-foil halves are joined across the centre gap. If the elevator strut sits aft of the transom
    # (as on the manned preset: elevator 2.0 m aft, transom 1.8 m), a bracket carries it from the transom.
    mc_s, ec_s = p.get("main_strut_chord", 0.12), p.get("elev_strut_chord", 0.08)
    gap = max(lat - ms / 2, 0.0)
    pod_r = max(0.025, 0.16 * mc)
    transom = p["hull_x"] - L / 2
    struts = f"""<geom name="main_strut" type="box" size="{mc_s / 2} {0.06 * mc_s} {strut_len / 2}" pos="{mx} 0 {hull_off - strut_len / 2}" material="strut"/>
      <geom name="main_pod" type="capsule" fromto="{mx - 0.9 * mc} 0 {-mz} {mx + 0.7 * mc} 0 {-mz}" size="{pod_r}" material="strut"/>
      <geom name="main_root" type="box" size="{mc / 2} {gap + 0.01} 0.008" pos="{mx} 0 {-mz}" material="foil"/>
      <geom name="elev_strut" type="box" size="{ec_s / 2} {0.06 * ec_s} {(hull_off + ez) / 2}" pos="{ex} 0 {hull_off - (hull_off + ez) / 2}" material="strut"/>
      <geom name="elev_pod" type="capsule" fromto="{ex - 0.8 * ec} 0 {-ez} {ex + 0.6 * ec} 0 {-ez}" size="{max(0.02, 0.16 * ec)}" material="strut"/>"""
    if ex - ec_s / 2 < transom:
        struts += f"""
      <geom name="elev_bracket" type="box" size="{(transom - ex) / 2 + ec_s / 2} {0.08 * B} 0.03" pos="{(transom + ex) / 2} 0 {hull_off + 0.03}" material="strut"/>"""
    rider = ""
    if p["mass"] > 100.0:                                # manned preset: a seated rider for scale
        rx, deck = p["hull_x"] - 0.12 * L, hull_off + D
        rider = f"""
      <geom name="rider_thigh_l" type="capsule" fromto="{rx + 0.02} 0.10 {deck + 0.10} {rx + 0.40} 0.11 {deck + 0.30}" size="0.06" material="rider_dark"/>
      <geom name="rider_thigh_r" type="capsule" fromto="{rx + 0.02} -0.10 {deck + 0.10} {rx + 0.40} -0.11 {deck + 0.30}" size="0.06" material="rider_dark"/>
      <geom name="rider_shin_l" type="capsule" fromto="{rx + 0.40} 0.11 {deck + 0.30} {rx + 0.56} 0.12 {deck + 0.05}" size="0.05" material="rider_dark"/>
      <geom name="rider_shin_r" type="capsule" fromto="{rx + 0.40} -0.11 {deck + 0.30} {rx + 0.56} -0.12 {deck + 0.05}" size="0.05" material="rider_dark"/>
      <geom name="rider_arm_l" type="capsule" fromto="{rx - 0.02} 0.19 {deck + 0.58} {rx + 0.30} 0.17 {deck + 0.36}" size="0.04" material="rider_vest"/>
      <geom name="rider_arm_r" type="capsule" fromto="{rx - 0.02} -0.19 {deck + 0.58} {rx + 0.30} -0.17 {deck + 0.36}" size="0.04" material="rider_vest"/>
      <geom name="rider_body" type="capsule" fromto="{rx} 0 {deck + 0.14} {rx - 0.05} 0 {deck + 0.62}" size="0.15" material="rider_vest"/>
      <geom name="rider_head" type="sphere" pos="{rx - 0.05} 0 {deck + 0.86}" size="0.11" material="rider_dark"/>"""
    return f"""
<mujoco model="hydrofoil">
  <option gravity="0 0 0"/>
  <visual>
    <global offwidth="1920" offheight="1080" fovy="40"/>
    <headlight ambient="0.35 0.37 0.40" diffuse="0.55 0.55 0.55" specular="0.1 0.1 0.1"/>
    <map znear="0.01" zfar="500" haze="0.25"/>
    <quality shadowsize="4096"/>
    <rgba haze="0.70 0.80 0.82 1" fog="0.70 0.80 0.82 1"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.72 0.82 0.84" rgb2="0.10 0.27 0.33" width="512" height="512"/>
    <texture name="seabed" type="2d" builtin="checker" rgb1=".05 .17 .21" rgb2=".06 .20 .24" width="256" height="256"/>
    <material name="seabed" texture="seabed" texrepeat="12 12" texuniform="true"/>
    <material name="hull" rgba="0.94 0.95 0.97 1" specular="0.4" shininess="0.6"/>
    <material name="deck" rgba="0.06 0.23 0.28 1"/>
    <material name="rider_vest" rgba="0.95 0.76 0.19 1"/>
    <material name="rider_dark" rgba="0.08 0.12 0.14 1"/>
    <mesh name="hull" vertex="{_hull_vertices(L, B, D)}"/>
    <material name="strut" rgba="0.62 0.72 0.75 1" specular="0.3"/>
    <material name="foil" rgba="0.95 0.76 0.19 1" specular="0.5"/>
    <material name="flap" rgba="0.36 0.81 0.76 1" specular="0.5"/>
    <material name="water" rgba="0.09 0.36 0.44 0.42" specular="0.7" shininess="0.9"/>
    <hfield name="water" nrow="{WATER_N}" ncol="{WATER_N}" size="{WATER_HALF} {WATER_HALF} {2 * WATER_RANGE} 0.02"/>
  </asset>
  <worldbody>
    <light pos="0 0 8" dir="0.3 0.2 -1" directional="true" castshadow="true" diffuse="0.7 0.7 0.7"/>
    <geom name="seabed" type="plane" pos="0 0 -4" size="0 0 0.1" material="seabed" contype="0" conaffinity="0"/>
    <geom name="far_water" type="plane" pos="0 0 -0.03" size="0 0 0.1" material="water" contype="0" conaffinity="0"/>
    <body name="water" mocap="true" pos="0 0 {-WATER_RANGE}">
      <geom type="hfield" hfield="water" material="water" contype="0" conaffinity="0"/>
    </body>
    <body name="vehicle" pos="0 0 0">
      <freejoint name="root"/>
      <geom name="hull" type="mesh" mesh="hull" pos="{p["hull_x"]} 0 {hull_off}" material="hull" contype="0" conaffinity="0"/>
      <geom name="cockpit" type="box" size="{0.22 * L} {0.36 * B} 0.006" pos="{p["hull_x"] - 0.08 * L} 0 {hull_off + D}" material="deck" contype="0" conaffinity="0"/>{rider}
      {struts}
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
        self.hfield_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_HFIELD, "water")
        g = np.linspace(-WATER_HALF, WATER_HALF, WATER_N)
        self._gx, self._gy = np.meshgrid(g, g)            # rows along MuJoCo y, columns along x
        self._calm_drawn = False

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
        mujoco.mj_forward(self.model, d)

    def water(self, state, wave, t: float) -> bool:
        """Move the water patch under the boat and refill its heights from the wave field. Returns True if
        the heightfield changed (the caller then uploads it with the viewer's update_hfield)."""
        x0, y0 = round(float(state[0]) / 0.5) * 0.5, round(float(-state[1]) / 0.5) * 0.5   # snapped
        self.data.mocap_pos[self.water_mocap] = [x0, y0, -WATER_RANGE]
        calm = isinstance(wave, CalmWater)
        if calm and self._calm_drawn:
            return False
        self._calm_drawn = calm
        eta = 0.0 if calm else wave.elevation(x0 + self._gx, -(y0 + self._gy), t)
        eta = np.broadcast_to(eta, self._gx.shape)
        a = self.model.hfield_adr[self.hfield_id]
        self.model.hfield_data[a:a + WATER_N * WATER_N] = np.clip((eta + WATER_RANGE) / (2 * WATER_RANGE), 0, 1).ravel()
        return True


class Pilot:
    """Keyboard stick. Writes into the slider values (ctrl) so keys and sliders stay one source."""

    def __init__(self, ctrl: np.ndarray):
        self.ctrl = ctrl
        self.reset_flag = False
        self.toggle_ctrl = False
        self.toggle_foil = False
        self.kmh = False           # U key: show speeds in km/h instead of m/s (display only)
        self.focus = False         # set by the dashboard's "Show 3D view"
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
        elif code in (ord("C"), ord("c")):
            self.toggle_ctrl = True
        elif code in (ord("F"), ord("f")):
            self.toggle_foil = True
        elif code in (ord("U"), ord("u")):
            self.kmh = not self.kmh
        elif code in (ord("D"), ord("d")):        # open the dashboard (served by the local launcher)
            import webbrowser
            webbrowser.open(DASHBOARD_URL)

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
        bad = [d for d in DRIVES if self.out[d].get("limited")
               or abs(self.out[d]["I"]) > c.i_peak or self.out[d].get("V", 0) > c.k_mod * c.v_bus]
        return "AT LIMIT: " + ",".join(bad) if bad else "within limits"


class Plots:
    """Rolling MuJoCo figures drawn on the left edge of the window."""

    SPECS = [
        ("Hull height above water [m]", ("height", "target")),
        ("Roll / pitch [deg]", ("roll", "pitch")),
        ("Surface angle [deg]", ("left", "right", "elev")),
        ("Drive current [A]", ("I_left", "I_right", "I_elev")),
    ]
    COLORS = [(0.95, 0.76, 0.19), (0.36, 0.81, 0.76), (0.89, 0.93, 0.94)]   # signal yellow, aqua, spray (local/page.html)

    def __init__(self, window_s: float = 15.0, rate_hz: float = 30.0, ranges=None):
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
            f.flg_legend = 1
            f.figurergba[:] = (0.04, 0.17, 0.22, 0.90)
            f.panergba[:] = (0.03, 0.13, 0.16, 0.95)
            f.gridrgb[:] = (0.12, 0.32, 0.38)
            f.textrgb[:] = (0.89, 0.93, 0.94)
            f.gridsize = [4, 5]
            f.gridwidth = 1.0
            f.linewidth = 2.0
            f.legendrgba[:] = (0.04, 0.17, 0.22, 0.7)
            for i, k in enumerate(keys):
                f.linename[i] = k
                f.linergb[i] = self.COLORS[i]
            if ranges is not None:                      # fixed y range (no jumping axes); x follows time
                f.range[1][:] = ranges[len(self.figs)]
                f.flg_extend = 0
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

    @staticmethod
    def width(viewport) -> int:
        return int(min(460, max(280, viewport.width * 0.27)))

    def figures(self, viewport):
        if viewport is None or len(self.t) < 2:
            return []
        w = Plots.width(viewport)
        h = viewport.height // len(self.figs)
        tt = np.asarray(self.t)
        out = []
        span = self.n * self.min_dt
        for j, ((_title, keys), f) in enumerate(zip(self.SPECS, self.figs)):
            f.range[0][:] = (tt[-1] - span, tt[-1])          # scrolling time window
            f.flg_legend = int(h >= 170)                      # legends cover the lines in short plots
            for i, k in enumerate(keys):
                y = np.asarray(self.buf[k])
                n = len(y)
                f.linepnt[i] = n
                f.linedata[i][0:2 * n:2] = tt[-n:]
                f.linedata[i][1:2 * n:2] = y
            rect = mujoco.MjrRect(viewport.width - w, viewport.height - (j + 1) * h, w, h)
            out.append((rect, f))
        return out


def _maximize_window(title_prefix: str):
    """Windows only: maximise the viewer window once it exists (best effort, silently skipped elsewhere)."""
    import sys
    if sys.platform != "win32":
        return
    import ctypes
    import threading

    def go():
        u = ctypes.windll.user32
        found = []
        cb_t = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

        def cb(hwnd, _):
            n = u.GetWindowTextLengthW(hwnd)
            if n:
                b = ctypes.create_unicode_buffer(n + 1)
                u.GetWindowTextW(hwnd, b, n + 1)
                if b.value.startswith(title_prefix) and u.IsWindowVisible(hwnd):
                    found.append(hwnd)
            return True
        for _ in range(50):
            u.EnumWindows(cb_t(cb), 0)
            if found:
                u.ShowWindow(found[0], 3)         # SW_MAXIMIZE
                u.SetForegroundWindow(found[0])
                return
            time.sleep(0.1)
    threading.Thread(target=go, daemon=True).start()


_FONT_DIR = Path(__file__).resolve().parent / "assets" / "fonts"
_C = {"deep": (10, 44, 55), "trench": (7, 32, 41), "rule": (30, 81, 97), "mist": (142, 178, 187), "spray": (227, 238, 240),
      "signal": (242, 194, 48), "aqua": (92, 207, 194), "flag": (215, 38, 61)}


class Hud:
    """Instrument strip along the bottom of the 3-D view, drawn with Pillow in the dashboard's type and colours
    (MuJoCo's own overlay text is a single fixed font). Saira fonts: hydrofoil/assets/fonts (SIL OFL)."""

    def __init__(self):
        from PIL import ImageFont
        self._IF = ImageFont
        self._cache = {}

    def _font(self, kind: str, size: int):
        key = (kind, size)
        if key not in self._cache:
            name = {"num": "SairaExtraCondensed-SemiBold.ttf", "ui": "SairaSemiCondensed-Regular.ttf",
                    "ui5": "SairaSemiCondensed-Medium.ttf"}[kind]
            try:
                self._cache[key] = self._IF.truetype(str(_FONT_DIR / name), size)
            except OSError:
                self._cache[key] = self._IF.load_default()
        return self._cache[key]

    def render(self, w: int, h: int, d: dict) -> np.ndarray:
        from PIL import Image, ImageDraw
        im = Image.new("RGB", (w, h), _C["deep"])
        g = ImageDraw.Draw(im)
        bar = max(18, h // 6)                                    # key-hint bar along the top of the strip
        al = d.get("alert")
        if al:                                                   # the observer's worst alert replaces the key help
            col = _C["flag"] if al[0] == "danger" else _C["signal"]
            g.rectangle([0, 0, w, bar], fill=col)
            g.text((14, bar // 2), ("Danger: " if al[0] == "danger" else "Caution: ") + al[1],
                   font=self._font("ui5", int(bar * 0.66)), fill=(10, 30, 36), anchor="lm")
        else:
            g.rectangle([0, 0, w, bar], fill=_C["trench"])
            g.text((14, bar // 2), d["keys"], font=self._font("ui", int(bar * 0.62)), fill=_C["mist"], anchor="lm")
        body = h - bar
        cells = [0.22, 0.2, 0.2, 0.2, 0.18]
        x = 0
        lab, big, sml = self._font("ui", int(body * 0.17)), self._font("num", int(body * 0.42)), self._font("ui", int(body * 0.15))
        pad = 16
        for i, frac in enumerate(cells):
            cw = int(w * frac)
            if i:
                g.line([x, bar + 8, x, h - 8], fill=_C["rule"], width=1)
            y0 = bar + int(body * 0.08)
            if i == 0:
                g.text((x + pad, y0), d["time"], font=lab, fill=_C["mist"])
                g.text((x + pad, y0 + int(body * 0.2)), d["mode"], font=self._font("ui5", int(body * 0.3)),
                       fill=_C["signal"] if d["mode"] == "Foilborne" else _C["spray"])
                g.text((x + pad, y0 + int(body * 0.62)), d["ctl"], font=sml, fill=_C["mist"])
            elif i == 3:                                          # heel dial + numbers
                r = int(body * 0.34)
                cx, cy = x + pad + r, bar + body // 2
                g.ellipse([cx - r, cy - r, cx + r, cy + r], fill=_C["trench"], outline=_C["rule"])
                for a in (-30, -20, -10, 0, 10, 20, 30):
                    t = np.radians(a)
                    g.line([cx + np.sin(t) * r * 0.8, cy - np.cos(t) * r * 0.8, cx + np.sin(t) * r, cy - np.cos(t) * r],
                           fill=_C["mist"], width=2 if a == 0 else 1)
                tb = np.radians(d["bank_ref"])
                g.ellipse([cx + np.sin(tb) * r * 0.9 - 3, cy - np.cos(tb) * r * 0.9 - 3, cx + np.sin(tb) * r * 0.9 + 3,
                           cy - np.cos(tb) * r * 0.9 + 3], fill=_C["aqua"])
                t = np.radians(d["roll"])
                dx, dy = np.cos(t) * r * 0.65, np.sin(t) * r * 0.65
                g.line([cx - dx, cy - dy, cx + dx, cy + dy], fill=_C["signal"], width=3)
                g.line([cx, cy, cx + np.sin(t) * r * 0.6, cy - np.cos(t) * r * 0.6], fill=_C["signal"], width=2)
                tx = cx + r + 12
                g.text((tx, y0), "Heel / trim", font=lab, fill=_C["mist"])
                g.text((tx, y0 + int(body * 0.18)), f"{d['roll']:+.1f}°", font=big, fill=_C["signal"])
                g.text((tx, y0 + int(body * 0.66)), f"trim {d['pitch']:+.1f}°", font=sml, fill=_C["mist"])
            else:
                k, v, unit, sub_, warn = d["cells"][i]
                g.text((x + pad, y0), k, font=lab, fill=_C["mist"])
                g.text((x + pad, y0 + int(body * 0.18)), v, font=big, fill=_C["signal"])
                vw = g.textlength(v, font=big)
                g.text((x + pad + vw + 6, y0 + int(body * 0.18) + int(body * 0.2)), unit, font=sml, fill=_C["spray"])
                g.text((x + pad, y0 + int(body * 0.66)), sub_, font=sml, fill=_C["flag"] if warn else _C["mist"])
            x += cw
        return np.ascontiguousarray(np.asarray(im))               # rows top-down: set_images flips them itself


def _find_window(title_prefix: str):
    """Windows only: handle of the first visible top-level window whose title starts with title_prefix."""
    import ctypes
    u = ctypes.windll.user32
    found = []
    cb_t = ctypes.WINFUNCTYPE(ctypes.c_bool, ctypes.c_void_p, ctypes.c_void_p)

    def cb(hwnd, _):
        n = u.GetWindowTextLengthW(hwnd)
        if n:
            b = ctypes.create_unicode_buffer(n + 1)
            u.GetWindowTextW(hwnd, b, n + 1)
            if b.value.startswith(title_prefix) and u.IsWindowVisible(hwnd):
                found.append(hwnd)
        return True
    u.EnumWindows(cb_t(cb), 0)
    return found[0] if found else None


def _focus_window(title_prefix: str):
    """Bring the viewer window to the front (dashboard's "Show 3D view"). Windows only; elsewhere a no-op."""
    import sys
    if sys.platform != "win32":
        return
    import ctypes
    u = ctypes.windll.user32
    h = _find_window(title_prefix)
    if h:
        u.ShowWindow(h, 9 if u.IsIconic(h) else 5)               # SW_RESTORE / SW_SHOW
        u.SetForegroundWindow(h)


class ControlAPI:
    """Local HTTP API (127.0.0.1 only) so the launcher page can show live telemetry and move the sliders.
    GET /state -> telemetry JSON; GET /sliders -> slider ranges; POST /set {"name": value, ...} or
    {"action": "reset" | "toggle_ctrl"}. The slider values live in the MuJoCo ctrl array, as for the keys."""

    def __init__(self, port: int, ctrl, pilot, lock):
        import json
        import threading
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
        self.telemetry: dict = {}
        api = self

        class H(BaseHTTPRequestHandler):
            def _json(self, obj, code=200):
                b = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

            def do_OPTIONS(self):
                self._json({})

            def do_GET(self):
                if self.path.startswith("/state"):
                    self._json(api.telemetry)
                elif self.path.startswith("/sliders"):
                    self._json([{"name": n, "lo": lo, "hi": hi, "value": float(ctrl[i])}
                                for i, (n, _k, lo, hi) in enumerate(SLIDERS)])
                else:
                    self._json({"error": "not found"}, 404)

            def do_POST(self):
                try:
                    body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                except ValueError:
                    return self._json({"error": "bad json"}, 400)
                with lock():
                    act = body.pop("action", None)
                    if act == "reset":
                        pilot.reset_flag = True
                    elif act == "toggle_ctrl":
                        pilot.toggle_ctrl = True
                    elif act == "toggle_foil":
                        pilot.toggle_foil = True
                    elif act == "focus":
                        pilot.focus = True
                    for name, val in body.items():
                        if name in IDX:
                            _n, _k, lo, hi = SLIDERS[IDX[name]]
                            ctrl[IDX[name]] = float(np.clip(float(val), lo, hi))
                            if name == "bank_ref":
                                pilot.bank_spring = False          # a set bank holds
                self._json({"ok": True})

            def log_message(self, *a):
                pass

        try:
            self.srv = ThreadingHTTPServer(("127.0.0.1", port), H)
        except OSError:
            self.srv = None                                         # port busy: run without the API
            return
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()

    def close(self):
        if self.srv is not None:
            self.srv.shutdown()


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


def _ctrl_label(vp: VehiclePhysics) -> str:
    if vp.lqr is None:
        return "PID"
    return "LQR" if vp.lqr.engaged else "LQR (PID takeoff)"


def run(speed: float = 1.0, wave_cfg: dict | None = None, duration: float = 600.0,
        controller: str | None = None, sensors: str | None = None, preset: str = "model", api_port: int = 8766, maximize: bool = True,
        overrides: dict | None = None, supervisor: bool = False):
    from hydrofoil.design import apply_design
    from hydrofoil.monitor import FlightMonitor
    params = apply_design(dict(MANNED_PARAMS if preset == "manned" else DEFAULT_PARAMS), overrides or {})
    params.setdefault("v_bus", DriveSpec().ctrl.v_bus)
    if controller:
        params["controller"] = controller
    if sensors:
        params["sensors"] = sensors
    if supervisor:
        params["supervisor"] = True
    if wave_cfg:
        params.update(wave_cfg)
    else:
        params["wave_amplitude"] = 0.0
    dt = params["dt"]

    vp = VehiclePhysics(params)
    lqr_spare = None            # the LQR not in use, kept so the C key can swap without a redesign
    pose = MujocoPose(params)
    ctrl = pose.data.ctrl
    for i, (_n, key, _lo, _hi) in enumerate(SLIDERS):
        ctrl[i] = params[key]
    if vp.supervisor is not None:
        vp.supervisor.pilot["foil"] = True
        ctrl[IDX["height_ref"]] = float(vp.supervisor.pilot["height"])
    pilot = Pilot(ctrl)
    drives = DriveMonitor(params)
    fh = params["flying_height"]
    monitor = FlightMonitor(params)
    alerts = []
    plots = Plots(ranges=[(min(-0.1, fh - 0.6), fh + 0.4), (-6.0, 6.0), (-25.0, 25.0), (-40.0, 40.0)])

    wave = _wave_from(params)
    wave_key = (params["wave_amplitude"], params["wave_wavelength"], params["wave_direction"])
    state = vp.rest_state()
    t = 0.0
    cmds = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    thrust, draft, wave_elev = 0.0, 0.0, 0.0
    pose.apply(state, cmds)
    last_bank_written = ctrl[IDX["bank_ref"]]

    size = max(params["hull_length"], 2 * params["main_lateral_offset"] + params["main_span"])
    hud, hud_img, hud_n = Hud(), None, 0
    with mujoco.viewer.launch_passive(pose.model, pose.data, key_callback=pilot.key,
                                      show_left_ui=False, show_right_ui=False) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = pose.vehicle_id
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 1.6 * size, 145.0, -12.0
        v.opt.flags[mujoco.mjtVisFlag.mjVIS_LIGHT] = 0
        api = ControlAPI(api_port, ctrl, pilot, v.lock)
        pose.water(state, wave, t)
        v.update_hfield(pose.hfield_id)
        api_line = "http://127.0.0.1:8765" if api.srv else "off (API port busy)"
        if maximize:
            _maximize_window("MuJoCo : hydrofoil")

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
                pilot_keys = {"flying_height", "phi_ref", "target_speed"}
                for i, (_n, key, _lo, _hi) in enumerate(SLIDERS):
                    if vp.supervisor is not None and key in pilot_keys:
                        continue
                    params[key] = float(ctrl[i])
                if vp.supervisor is not None:
                    vp.supervisor.pilot["height"] = float(ctrl[IDX["height_ref"]])
                    vp.supervisor.pilot["bank"] = float(ctrl[IDX["bank_ref"]])
                    vp.supervisor.pilot["speed"] = float(ctrl[IDX["speed_ref"]])
                    if pilot.toggle_foil:
                        pilot.toggle_foil = False
                        vp.supervisor.pilot["foil"] = not vp.supervisor.pilot["foil"]
                else:
                    pilot.toggle_foil = False
            last_wall = now
            _apply_gains(vp, params)
            wk = (params["wave_amplitude"], params["wave_wavelength"], params["wave_direction"])
            if wk != wave_key:
                wave, wave_key = _wave_from(params), wk

            if pilot.toggle_ctrl:          # switch PID <-> LQR; restarts from rest (no mid-flight handover)
                pilot.toggle_ctrl = False
                if vp.lqr is not None:
                    vp.lqr, lqr_spare = None, vp.lqr
                elif vp.actuator_model == "servo":
                    if lqr_spare is None:
                        vp._build_lqr()    # ~3 s: designs the gains
                    else:
                        vp.lqr, lqr_spare = lqr_spare, None
                pilot.reset_flag = True
            if pilot.reset_flag:
                pilot.reset_flag = False
                vp.reset(); drives.reset(); plots.clear(); monitor.reset()
                state, t = vp.rest_state(), 0.0
                wall0 = now

            target_t = (now - wall0) * speed
            steps = 0
            while t < target_t and steps < 50:     # cap catch-up so a slow frame can't spiral
                state, cmds, thrust, draft, wave_elev = vp.step(state, t, wave)
                t += dt
                steps += 1
                if "drive" in cmds:     # servo model: the drives' own simulated current/power
                    dm = drives.out = cmds["drive"]
                else:                   # kinematic model: estimate loads from the trajectory
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
            if steps:
                alerts = monitor.update(t, vp, state, cmds, wave, cmds.get("drive"), draft <= 1e-6)

            with v.lock():
                pose.apply(state, cmds, wave_elev)
                hf = pose.water(state, wave, t)
            if hf:
                v.update_hfield(pose.hfield_id)
            dm = drives.out
            mode = vp.supervisor.mode if vp.supervisor is not None else (
                "FOILBORNE" if draft <= 1e-6 else ("TAKING OFF" if state[6] > 1.0 else "DISPLACEMENT")
            )
            sup_stat = vp.supervisor.status if vp.supervisor is not None else None
            hud_alert = (alerts[0]["level"], alerts[0]["text"]) if alerts else None
            if sup_stat is not None:
                if sup_stat["faults"]:
                    hud_alert = ("danger", "SUP FAULT: " + ", ".join(sup_stat["faults"])
                                 + (" (GNSS vz)" if sup_stat["using_gnss"] else ""))
                elif sup_stat["warnings"] and hud_alert is None:
                    hud_alert = ("caution", "SUP WARN: " + ", ".join(sup_stat["warnings"]))
            height = -state[2] + params["hull_offset"]
            limits = drives.flags()
            ctl = f"{_ctrl_label(vp)}, {'noisy sensors + Kalman' if vp.sensors is not None else 'ideal sensors'}"
            sea = (f"{100 * params['wave_amplitude']:.0f} cm / {params['wave_wavelength']:.0f} m / {params['wave_direction']:.0f} deg"
                   if params["wave_amplitude"] > 1e-4 else "calm")
            if pilot.focus:
                pilot.focus = False
                _focus_window("MuJoCo : hydrofoil")
            vp_ = v.viewport
            if vp_ is not None and vp_.width > 0:
                k, su = (3.6, "km/h") if pilot.kmh else (1.0, "m/s")
                pw = Plots.width(vp_)
                sw, sh = max(320, vp_.width - pw), int(np.clip(vp_.height * 0.17, 104, 168))
                hud_n += 1
                akey = hud_alert
                if hud_img is None or hud_n % 6 == 0 or hud_img.shape[:2] != (sh, sw) or akey != getattr(hud, "_akey", None):
                    hud._akey = akey    # ~10 Hz redraw (a draw takes ~14 ms)
                    ok = limits == "within limits"
                    key_bar = ("Up / Down ride height     Left / Right bank     [ ] speed     "
                               + ("F foil/land     " if vp.supervisor is not None else "")
                               + "R rest     C PID / LQR     U units"
                               + ("     D dashboard" if api.srv else ""))
                    hud_img = hud.render(sw, sh, {
                        "time": f"{t:6.1f} s", "mode": mode.capitalize(),
                        "ctl": f"{_ctrl_label(vp)}, {'noisy' if vp.sensors is not None else 'ideal'} sensors"
                               + (" [sup]" if vp.supervisor is not None else ""),
                        "roll": float(np.degrees(state[3])), "pitch": float(np.degrees(state[4])), "bank_ref": params["phi_ref"],
                        "alert": hud_alert,
                        "keys": key_bar,
                        "cells": {1: ("Speed", f"{k * state[6]:.2f}", su,
                                      f"{state[6] * 1.944:.1f} kn, target {k * params['target_speed']:.2f} {su}", False),
                                  2: ("Ride height", f"{height:.2f}", "m", f"target {params['flying_height']:.2f} m", False),
                                  4: ("Thrust", f"{thrust:.0f}", "N",
                                      "drives within limits" if ok else limits.lower(), not ok)},
                    })
                v.set_images([(mujoco.MjrRect(0, 0, sw, sh), hud_img)])
            v.set_texts([])
            v.set_figures(plots.figures(v.viewport))
            v.sync()
            api.telemetry = {
                "t": t, "mode": mode, "preset": preset, "mass": params["mass"], "x": float(state[0]),
                "supervisor": sup_stat,
                "geom": {k: params[k] for k in ("hull_length", "hull_x", "hull_offset", "main_pos_x", "main_pos_z",
                                                "main_chord", "elev_pos_x", "elev_pos_z", "elev_chord")},
                "wave": {"amp": params["wave_amplitude"], "len": params["wave_wavelength"], "dir": params["wave_direction"]},
                "speed": float(state[6]), "speed_ref": params["target_speed"],
                "height": float(height), "height_ref": params["flying_height"],
                "roll": float(np.degrees(state[3])), "pitch": float(np.degrees(state[4])), "bank_ref": params["phi_ref"],
                "thrust": float(thrust), "draft": float(draft), "controller": _ctrl_label(vp),
                "sensors": "noisy" if vp.sensors is not None else "ideal", "limits": limits,
                "surfaces": {k: float(a) for k, a in drive_angles(cmds).items()},
                "drives": {d: {k: float(dm[d].get(k, 0.0)) for k in ("M", "I", "P")} for d in DRIVES},
                "sliders": {n: float(ctrl[i]) for i, (n, *_r) in enumerate(SLIDERS)},
                "alerts": alerts, "alert_log": monitor.log[-40:],
                "design": {k: params[k] for k in ("main_chord", "main_span", "main_incidence_deg", "elev_chord", "elev_span",
                                                  "elev_pos_x", "flying_height", "target_speed")} | {"cg_offset": list(params.get("cg_offset", (0, 0)))},
            }
            time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))
        api.close()


def main():
    ap = argparse.ArgumentParser(description="Hydrofoil simulator (native MuJoCo window)")
    ap.add_argument("--speed", type=float, default=1.0, help="sim speed multiplier")
    ap.add_argument("--duration", type=float, default=600.0, help="run length [sim s]")
    ap.add_argument("--waves", action="store_true", help="start with an Airy wave (amp/len/dir also on sliders)")
    ap.add_argument("--wave-dir", type=float, default=0.0, help="wave travel direction [deg from +x]; try 90 for beam seas")
    ap.add_argument("--controller", choices=("lqr", "pid"), help="flight controller (default: params)")
    ap.add_argument("--sensors", choices=("ideal", "noisy"), help="ideal truth, or noisy sensors + Kalman")
    ap.add_argument("--windowed", action="store_true", help="do not maximise the window on start")
    ap.add_argument("--overrides", help="JSON file of design edits (boat editor), see hydrofoil/design.py")
    ap.add_argument("--api-port", type=int, default=8766, help="local live-control API port (launcher page)")
    ap.add_argument("--preset", choices=("model", "manned"), default="model",
                    help="model = 25 kg DEFAULT_PARAMS, manned = 250-300 kg MANNED_PARAMS (docs/19)")
    ap.add_argument("--supervisor", action="store_true",
                    help="enable flight-mode supervisor (takeoff/landing envelope & fault protection; press F to foil/land)")
    a = ap.parse_args()
    wave_cfg = (dict(wave_amplitude=0.15, wave_wavelength=10.0, wave_direction=a.wave_dir) if a.preset == "manned"
                else dict(wave_amplitude=0.05, wave_wavelength=5.0, wave_direction=a.wave_dir)) if a.waves else None
    run(speed=a.speed, wave_cfg=wave_cfg, duration=a.duration, controller=a.controller, sensors=a.sensors,
        preset=a.preset, api_port=a.api_port, maximize=not a.windowed, supervisor=a.supervisor,
        overrides=json.loads(Path(a.overrides).read_text()) if a.overrides and Path(a.overrides).exists() else None)


if __name__ == "__main__":
    main()
