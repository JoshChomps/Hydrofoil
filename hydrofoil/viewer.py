"""
Native MuJoCo 3D viewer for the hydrofoil sim (replaces the old Unity visualizer).

MuJoCo is used purely as a renderer: hydrofoil.vehicle.VehiclePhysics remains the
only physics authority, and each frame we write its pose into the MuJoCo model
(qpos) and call mj_forward -- mj_step is never called.

Run:
    python -m hydrofoil.viewer [--waves] [--speed 1.0] [--duration 600]

Keys (click the viewer window first):
    Up / Down        climb / dive   (flying_height target, holds)
    Left / Right     bank left / right (phi_ref, +-8 deg, springs back to 0)
    [  /  ]          target speed down / up (holds)
    R                reset to rest

Frames: the sim is NED/body (x fwd, y stbd, z down). MuJoCo is Z-up, so world
coordinates are mapped with D = diag(1,-1,-1) (a proper 180 deg rotation about x):
    p_mj = D p_ned,   R_mj = D R_ned B   with B = diag(1,-1,-1) mapping the
MuJoCo body frame (x fwd, y left, z up) onto the sim's body frame.
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import mujoco
import mujoco.viewer

from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.dynamics.kinematics import R_body_to_ned
from hydrofoil.waves.airy import AiryWave, CalmWater

# Pilot limits -- keep identical to the verified/capped ranges used elsewhere.
HEIGHT_MIN, HEIGHT_MAX, HEIGHT_STEP = -0.3, 1.0, 0.02   # m, per key event
SPEED_MIN, SPEED_MAX, SPEED_STEP = 0.0, 10.0, 0.25       # m/s, per key event
BANK_MIN, BANK_MAX, BANK_STEP = -8.0, 8.0, 1.0           # deg, per key event
BANK_RETURN_RATE = 10.0                                   # deg/s spring-back

_D = np.diag([1.0, -1.0, -1.0])   # NED <-> Z-up world
_B = np.diag([1.0, -1.0, -1.0])   # sim body frame <-> MuJoCo body frame


def build_mjcf(p: dict) -> str:
    """Generate the MJCF for hull + struts + main foil halves (with flaps) + elevator."""
    hull_off = p["hull_offset"]
    mc, ms = p["main_chord"], p["main_span"]
    ec, es = p["elev_chord"], p["elev_span"]
    lat = p["main_lateral_offset"]
    mx, mz = p["main_pos_x"], p["main_pos_z"]
    ex, ez = p["elev_pos_x"], p["elev_pos_z"]
    flap_frac = 0.4   # matches the 40 % chord plain flap assumed by MAIN_FOIL_FLAP_EFFECTIVENESS

    def half(name, y):
        return f"""
        <body name="{name}" pos="{mx} {y} {-mz}">
          <geom type="box" size="{mc*(1-flap_frac)/2} {ms/2} 0.008" pos="{mc*flap_frac/2} 0 0" material="foil"/>
          <body name="{name}_flap" pos="{-mc*(1-flap_frac)/2 + mc*flap_frac/2} 0 0">
            <joint name="{name}_hinge" type="hinge" axis="0 1 0" limited="true" range="-60 60"/>
            <geom type="box" size="{mc*flap_frac/2} {ms/2} 0.008" pos="{-mc*flap_frac/2} 0 0" material="flap"/>
          </body>
        </body>"""

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
        d.qpos[self.q_left] = -np.radians(cmds["delta_main"] - cmds["delta_roll"])
        d.qpos[self.q_right] = -np.radians(cmds["delta_main"] + cmds["delta_roll"])
        d.qpos[self.q_elev] = -np.radians(cmds["delta_elev"])
        d.mocap_pos[self.water_mocap] = [eta[0], -eta[1], wave_elev]
        mujoco.mj_forward(self.model, d)


class Pilot:
    """In-process replacement for the old Unity UDP stick."""

    def __init__(self, params: dict):
        self.p = params
        self.reset_flag = False

    def key(self, code: int):
        p = self.p
        if code == 265:      # up
            p["flying_height"] = min(HEIGHT_MAX, p["flying_height"] + HEIGHT_STEP)
        elif code == 264:    # down
            p["flying_height"] = max(HEIGHT_MIN, p["flying_height"] - HEIGHT_STEP)
        elif code == 262:    # right -> bank right
            p["phi_ref"] = min(BANK_MAX, p["phi_ref"] + BANK_STEP)
        elif code == 263:    # left
            p["phi_ref"] = max(BANK_MIN, p["phi_ref"] - BANK_STEP)
        elif code == 93:     # ]
            p["target_speed"] = min(SPEED_MAX, p["target_speed"] + SPEED_STEP)
        elif code == 91:     # [
            p["target_speed"] = max(SPEED_MIN, p["target_speed"] - SPEED_STEP)
        elif code in (ord("R"), ord("r")):
            self.reset_flag = True

    def spring_back(self, dt: float):
        phi = self.p["phi_ref"]
        step = BANK_RETURN_RATE * dt
        self.p["phi_ref"] = max(0.0, phi - step) if phi > 0 else min(0.0, phi + step)


def run(speed: float = 1.0, wave=None, duration: float = 600.0):
    params = dict(DEFAULT_PARAMS)
    dt = params["dt"]
    wave = wave or CalmWater()

    vp = VehiclePhysics(params)
    pose = MujocoPose(params)
    pilot = Pilot(params)

    state = vp.rest_state()
    t = 0.0
    cmds = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    thrust, draft, wave_elev = 0.0, 0.0, 0.0
    pose.apply(state, cmds)

    with mujoco.viewer.launch_passive(pose.model, pose.data, key_callback=pilot.key) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        v.cam.trackbodyid = pose.vehicle_id
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 4.5, 140.0, -18.0

        wall0 = time.perf_counter()
        last_wall = wall0
        while v.is_running() and t < duration:
            now = time.perf_counter()
            pilot.spring_back(now - last_wall)
            last_wall = now

            if pilot.reset_flag:
                pilot.reset_flag = False
                vp.controller.reset(); vp.surge.reset()
                state, t = vp.rest_state(), 0.0
                wall0 = now

            target_t = (now - wall0) * speed
            steps = 0
            while t < target_t and steps < 50:     # cap catch-up so a slow frame can't spiral
                state, cmds, thrust, draft, wave_elev = vp.step(state, t, wave)
                t += dt
                steps += 1
            if steps == 50:
                wall0 = now - t / speed             # drop the backlog instead of racing to catch up

            pose.apply(state, cmds, wave_elev)
            mode = "FOILBORNE" if draft <= 1e-6 else "DISPLACEMENT"
            v.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                         "t / mode / speed (target)\nheight above water (target)\nroll / pitch\n"
                         "flaps main/elev/roll\nthrust / draft\nphi_ref",
                         f"{t:6.1f}s  {mode}  {state[6]:.2f} ({params['target_speed']:.2f}) m/s\n"
                         f"{-(state[2]) + params['hull_offset']:.2f} ({params['flying_height']:.2f}) m\n"
                         f"{np.degrees(state[3]):.1f} / {np.degrees(state[4]):.1f} deg\n"
                         f"{cmds['delta_main']:.1f} / {cmds['delta_elev']:.1f} / {cmds['delta_roll']:.1f} deg\n"
                         f"{thrust:.1f} N / {draft:.3f} m\n"
                         f"{params['phi_ref']:.1f} deg"))
            v.sync()
            time.sleep(max(0.0, 1 / 60 - (time.perf_counter() - now)))


def main():
    ap = argparse.ArgumentParser(description="Hydrofoil sim -> native MuJoCo viewer")
    ap.add_argument("--speed", type=float, default=1.0, help="sim speed multiplier")
    ap.add_argument("--duration", type=float, default=600.0, help="run length [sim s]")
    ap.add_argument("--waves", action="store_true", help="enable an Airy wave (period from deep-water dispersion)")
    ap.add_argument("--wave-dir", type=float, default=0.0, help="wave travel direction [deg from +x]; try 90 for beam seas")
    a = ap.parse_args()
    wave = AiryWave(amplitude=0.05, wavelength=5.0, direction=np.radians(a.wave_dir)) if a.waves else CalmWater()
    run(speed=a.speed, wave=wave, duration=a.duration)


if __name__ == "__main__":
    main()
