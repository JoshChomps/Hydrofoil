"""
Canonical vehicle physics, shared by the browser dashboard (dashboard.py)
and the Unity-streaming realtime sim (sim_realtime.py) so they can't drift
apart the way sim_realtime.py fell behind dashboard.py's physics this
session (constant-volume buoyancy, force-fed speed, inert wave config).

Everything here — hull buoyancy-vs-draft, wave-coupled foil flow, closed-loop
surge/thrust control, the differential left/right main foil — is exactly
what dashboard.py's SimRunner._run() had grown, extracted so it has one
source of truth. sim.py's simpler model stays as-is for quick trim/plotting
checks; this module is for anything that needs the full picture.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from hydrofoil.constants import RHO_WATER
from hydrofoil.dynamics.rigid_body import RigidBody, strip_added_mass_heave_pitch
from hydrofoil.dynamics.kinematics import J, R_body_to_ned
from hydrofoil.forces.foil import (
    Foil, FoilGeometry, FoilTables,
    MAIN_FOIL_FLAP_EFFECTIVENESS, ELEVATOR_FLAP_EFFECTIVENESS, main_flap_effectiveness, MAIN_FOIL_OSWALD_EFFICIENCY,
)
from hydrofoil.forces.buoyancy import gravity_force
from hydrofoil.forces.hull import HullHydrostatics
from hydrofoil.actuation.hinge import thin_airfoil_coeffs
from hydrofoil.control.pid import HydrofoilController
from hydrofoil.control.surge import SurgeController
from hydrofoil.control.actuator import DEFAULT_ACTUATOR_PARAMS
from hydrofoil.actuation.drive import DriveSpec
from hydrofoil.actuation.loads import surface_models, flow_models
from hydrofoil.actuation.servo import ServoDrive, DEFAULT_SERVO_PARAMS


# Safety bounds for VehiclePhysics.step() — this drives a real-time interactive
# sim, and rapid multi-axis pilot input (banking while also changing height/
# speed) can excite gyroscopic roll<->yaw<->pitch coupling (real physics —
# Fossen's C_RB matrix — not a bug) faster than the independent SISO PIDs can
# track, which can run away to inf/NaN (verified: reproducible within 15-60s
# of randomized combined stick input, even with bank capped at +-8 deg).
# Proper fix is a coupled/MIMO controller (Phase 3: LQR/MPC per the project
# roadmap). Until then, every step clamps state to physically-plausible
# bounds so a runaway can never reach Unity as NaN — worst case it looks like
# an aggressive, ugly maneuver instead of crashing the receiver.
MAX_ATTITUDE_RAD = np.radians(70.0)   # keeps clear of T_euler's +-90 deg singularity
MAX_LINEAR_VEL = 15.0                  # m/s
MAX_ANGULAR_VEL = 3.0                  # rad/s (~172 deg/s)


@dataclass
class WaveSample:
    """
    Wave environment frozen for one integration step, sampled where it matters:
    at the hull (draft) and at each foil's own world position (relative flow and
    free-surface submergence). Sampling per foil is what lets oblique/beam seas
    load the left and right halves differently and so excite roll.
    """
    elevation: float = 0.0                       # at the body origin [m] (for display)
    hull_surface_z: float = 0.0                  # NED z of the surface at the body origin (display)
    hull_cell_surface_z: object = 0.0            # NED z of the surface under every hull footprint cell (N,) or scalar
    foil_surface_z: dict = field(default_factory=dict)   # name -> NED z of the surface at that foil
    foil_vel_body: dict = field(default_factory=dict)    # name -> wave orbital velocity in body frame [m/s]


class VehiclePhysics:
    """
    Builds the vehicle, foils, and controllers from a params dict, and
    advances the full closed-loop simulation one RK4 step at a time.

    params must contain: mass, ixx, iyy, izz, heave_damping, main_chord,
    main_span, main_pos_x, main_pos_z, main_lateral_offset, elev_chord,
    elev_span, elev_pos_x, elev_pos_z, kp/ki/kd_heave, kp/ki/kd_pitch,
    kp/ki/kd_roll, flying_height, phi_ref, theta_ref, target_speed,
    kp/ki/kd_speed, max_thrust, hull_length, hull_beam, hull_x, max_draft, hull_offset,
    hull_drag_coeff, dt. Optional: actuator_params (defaults to
    DEFAULT_ACTUATOR_PARAMS). params is kept live (not copied) — mutating
    it externally (e.g. a live "stick" update) takes effect on the next step().
    """

    def __init__(self, params: dict):
        self.params = params
        p = params

        # Hydrodynamic added mass (heave + pitch) from the vehicle's own
        # lifting surfaces — see dynamics/rigid_body.py::strip_added_mass_heave_pitch.
        # Left/right main-foil halves sit at the same (main_pos_x, main_pos_z)
        # and only differ in lateral offset, which doesn't matter for this
        # heave/pitch-only calculation.
        added_mass_heave, added_inertia_pitch = strip_added_mass_heave_pitch([
            (p["main_chord"], p["main_span"], p["main_pos_x"]),   # left half
            (p["main_chord"], p["main_span"], p["main_pos_x"]),   # right half
            (p["elev_chord"], p["elev_span"], p["elev_pos_x"]),
        ])
        # Distributed hull hydrostatics (cross-referenced against Fossen ch.4 GM_T / GM_L, see docs/15).
        self.hull = HullHydrostatics(p["hull_length"], p["hull_beam"], p["hull_x"],
                                     p["hull_offset"], p["max_draft"])
        self.waterplane_area = self.hull.waterplane_area
        self.rigid_body = RigidBody(
            mass=p["mass"], I_b=np.diag([p["ixx"], p["iyy"], p["izz"]]),
            added_mass_heave=added_mass_heave, added_inertia_pitch=added_inertia_pitch,
        )

        polar = FoilTables.naca0009()
        # Main wing modeled as two independently-flapped halves, offset
        # laterally — this is what lets delta_roll produce a rolling moment
        # (asymmetric lift x lateral offset); the existing 6-DOF dynamics
        # turn that into an actual bank. Closed-loop via self.controller's
        # roll_pid — see control/pid.py.
        # Flap section pitching moment (thin-airfoil Cm_delta x inboard span fraction x viscous factor).
        # Lift effectiveness follows the flap geometry (forces/foil.py::main_flap_effectiveness; 0.21 at the
        # 40 % chord / half-span reference); `main_flap_effectiveness` in params overrides it.
        self.main_flap_eff = (p.get("main_flap_effectiveness")
                              or (main_flap_effectiveness(p["main_flap_frac"], p["main_flap_span_frac"])
                                  if p.get("main_actuation", "flap") == "flap" else MAIN_FOIL_FLAP_EFFECTIVENESS))
        flap_cm = (thin_airfoil_coeffs(p["main_flap_frac"])["cm_delta"] * p["main_flap_span_frac"]
                   * p.get("flap_cm_factor", 0.8)) if p.get("flap_moment", True) else 0.0
        self.left_foil = Foil(
            FoilGeometry(chord=p["main_chord"], span=p["main_span"], aoa_offset=p.get("main_incidence_deg", 0.0)),
            polar,
            r_attach=np.array([p["main_pos_x"], -p["main_lateral_offset"], p["main_pos_z"]]),
            flap_effectiveness=self.main_flap_eff,
            oswald_efficiency=MAIN_FOIL_OSWALD_EFFICIENCY, flap_cm_delta=flap_cm,
        )
        self.right_foil = Foil(
            FoilGeometry(chord=p["main_chord"], span=p["main_span"], aoa_offset=p.get("main_incidence_deg", 0.0)),
            polar,
            r_attach=np.array([p["main_pos_x"], p["main_lateral_offset"], p["main_pos_z"]]),
            flap_effectiveness=self.main_flap_eff,
            oswald_efficiency=MAIN_FOIL_OSWALD_EFFICIENCY, flap_cm_delta=flap_cm,
        )
        self.elev_foil = Foil(
            FoilGeometry(chord=p["elev_chord"], span=p["elev_span"]),
            polar,
            r_attach=np.array([p["elev_pos_x"], 0.0, p["elev_pos_z"]]),
            flap_effectiveness=ELEVATOR_FLAP_EFFECTIVENESS,
        )

        # Vertical struts (hull -> foil), normal axis = body y, so AoA = side-slip and they
        # carry side force; wetted span follows the foil's depth below the surface.
        self.main_strut = Foil(FoilGeometry(chord=p["main_strut_chord"], span=1.0), polar,
                               r_attach=np.array([p["main_pos_x"], 0.0, p["main_pos_z"]]),
                               normal_axis=np.array([0.0, 1.0, 0.0]))
        self.elev_strut = Foil(FoilGeometry(chord=p["elev_strut_chord"], span=1.0), polar,
                               r_attach=np.array([p["elev_pos_x"], 0.0, p["elev_pos_z"]]),
                               normal_axis=np.array([0.0, 1.0, 0.0]))

        self._r_nominal = {n: f.r_attach.copy() for n, f in self._foils().items()}
        self.set_cg_offset(p.get("cg_offset", (0.0, 0.0)))

        pid_gains = {
            "heave": {"Kp": p["kp_heave"], "Ki": p["ki_heave"], "Kd": p["kd_heave"]},
            "pitch": {"Kp": p["kp_pitch"], "Ki": p["ki_pitch"], "Kd": p["kd_pitch"]},
            "roll":  {"Kp": p["kp_roll"],  "Ki": p["ki_roll"],  "Kd": p["kd_roll"]},
        }
        actuator_params = p.get("actuator_params") or DEFAULT_ACTUATOR_PARAMS
        self.controller = HydrofoilController(pid_gains, dt=p["dt"], actuator_params=actuator_params)
        self._dw_t, self._dw_eps, self._eps_used = [], [], 0.0   # wing->tail downwash history (transport lag)
        self.surge = SurgeController(Kp=p["kp_speed"], Ki=p["ki_speed"], Kd=p["kd_speed"],
                                      dt=p["dt"], max_thrust=p["max_thrust"])

        # Surface drives. "servo" (default): three PHYSICAL drives (left flap, right flap, elevator) after the
        # mixer, each a moteus position loop + BLDC + gearbox against the hinge moment (actuation/servo.py,
        # docs/17). "kinematic": the legacy delay -> rate limit -> lag on the three virtual channels.
        self.actuator_model = p.get("actuator_model", "servo")
        self.hinges = surface_models(p)
        self.lqr = None
        self.sensors = None
        self._last_cmds = dict.fromkeys(("delta_main", "delta_elev", "delta_roll"), 0.0)
        if p.get("sensors", "ideal") == "noisy":
            self._build_estimation()
        if p.get("controller", "pid") == "lqr":
            if self.actuator_model != "servo":
                import warnings
                warnings.warn("controller='lqr' needs actuator_model='servo'; falling back to the PID loops")
            else:
                self._build_lqr()
        sp = dict(DEFAULT_SERVO_PARAMS, **(p.get("servo_params") or {}))
        self.drives = {}
        self._last_target = None
        self._slew_last = None                              # command slew limiter state (cmd_slew_deg_s)
        for name, stop in (("left", p["flap_stop_deg"]), ("right", p["flap_stop_deg"]), ("elev", p["elev_stop_deg"])):
            spec = DriveSpec()
            spec.gear.ratio = p["gear_ratio"]
            spec.ctrl.v_bus = p["v_bus"]
            if p.get("supply_i_max") is not None:
                spec.ctrl.supply_i_max = p["supply_i_max"]
            if p.get("drive_i_cont") is not None:
                spec.ctrl.i_cont = p["drive_i_cont"]
            self.drives[name] = ServoDrive(self.hinges[name], spec, p["dt"], stop, **sp,
                                           twist_k=self.twist_stiffness(name))
        # Flight-mode supervisor (control/supervisor.py): off unless params["supervisor"] is True or a config dict.
        self.supervisor = None
        if p.get("supervisor"):
            from hydrofoil.control.supervisor import Supervisor
            self.supervisor = Supervisor(self, p["supervisor"] if isinstance(p["supervisor"], dict) else None)

    def set_cg_offset(self, offset):
        """Move the centre of gravity by (dx forward, dy to starboard) [m] relative to the nominal layout, e.g. a
        rider shifting their weight. The body frame stays at the CG, so every foil and strut moves by -offset.
        Can be called mid-flight. Not shifted: the hull footprint (matters only while floating), the inertias and
        the added-mass terms; mass is set by params['mass'] at construction."""
        d = np.array([float(offset[0]), float(offset[1]), 0.0])
        for n, f in self._foils().items():
            f.r_attach = self._r_nominal[n] - d

    def _build_lqr(self):
        """Scheduled LQR (control/lqr.py), designed on a PID-configured copy of this plant."""
        from hydrofoil.control.lqr import LQRController
        # designed on the NOMINAL CG: the controller cannot know where the rider sits (cg_offset is the truth)
        p = dict(self.params, controller="pid", cg_offset=(0.0, 0.0), supervisor=False)
        # design latency: the loop delay the gains are computed for. Defaults to the drive model's command
        # delay; on hardware set lqr_delay_s to the MEASURED sensor-to-surface latency.
        delay = p.get("lqr_delay_s")
        if delay is None:
            delay = dict(DEFAULT_SERVO_PARAMS, **(p.get("servo_params") or {}))["delay_s"]
        self.lqr = LQRController(VehiclePhysics(p), p["dt"], weights=p.get("lqr_weights"), delay_s=delay,
                                 speeds=tuple(p.get("lqr_speeds", (3.0, 4.0, 5.0, 6.0, 7.0))),
                                 engage_speed=p.get("lqr_engage_speed", 3.2))

    def _build_estimation(self):
        """Noisy sensors + Kalman estimator (params['sensors'] = 'noisy'); controllers then see estimates."""
        from hydrofoil.sensors.models import SensorSuite
        from hydrofoil.estimation.kalman import KalmanEstimator, HeaveFilter
        p = dict(self.params, controller="pid", sensors="ideal", cg_offset=(0.0, 0.0), supervisor=False)   # nominal CG model
        self.sensors = SensorSuite(seed=p.get("sensor_seed", 0), scale=p.get("sensor_noise_scale", 1.0))
        self.estimator = KalmanEstimator(VehiclePhysics(p), p["dt"], process=p.get("kf_process"),
                                         speeds=tuple(p.get("lqr_speeds", (3.0, 4.0, 5.0, 6.0, 7.0))))
        self.heave = HeaveFilter(p["dt"], wave=p.get("kf_wave"), **(p.get("kf_heave") or {}))
        self._est_ready = False
        self._u_meas = 0.0

    def _control_state(self, state, t: float, surface_elev: float = 0.0) -> np.ndarray:
        """The state the controllers see: truth when sensors are 'ideal'; otherwise measurements (below the
        foilborne speed) or the Kalman estimate (above it). Physics always uses the true state."""
        if self.sensors is None:
            return state
        p = self.params
        meas = self.sensors.measure(state, t, p["dt"], surface_elev)
        if "u" in meas:
            self._u_meas = float(meas["u"])
        # Height: the ranging sensor is slow, late and sees the local wave surface; the heave filter turns it
        # (with the accelerometer) into height above the mean surface and a vertical speed, from t = 0 on.
        if "z" in meas:
            if not self.heave.ready:
                self.heave.init(meas["z"])
            self.heave.step(meas.get("az"), meas["z"], "z" in self.sensors.fresh)
            meas = dict(meas, z=self.heave.z)
        if not all(k in meas for k in ("z", "phi", "theta", "p", "q", "r")) and not self._est_ready:
            return state                                   # sensors not all up yet (first few ms)
        c = state.copy()
        z_trim = p["hull_offset"] - p["flying_height"]
        if self._u_meas < p.get("est_engage_speed", 3.0) and not self._est_ready:
            for k, i in (("z", 2), ("phi", 3), ("theta", 4), ("p", 9), ("q", 10), ("r", 11)):
                if k in meas:
                    c[i] = meas[k]
            c[6] = self._u_meas
            return c
        if not self._est_ready:
            self.estimator.init(meas, z_trim)
            self._est_ready = True
        last = self._last_cmds
        est = self.estimator.update(meas, [last["delta_main"], last["delta_elev"], last["delta_roll"]],
                                    self._u_meas, z_trim)
        c[[2, 3, 4, 7, 8, 9, 10, 11]] = est
        c[6] = self._u_meas
        # heave velocity from the kinematic filter (it sees wave and payload forces through the accelerometer,
        # which the model-based filter cannot): vz = -u sin(theta) + w cos(theta) cos(phi)
        c[2] = self.heave.z
        c[8] = (self.heave.vz + self._u_meas * np.sin(c[4])) / (np.cos(c[4]) * np.cos(c[3]))
        return c

    def reset(self):
        """Reset controllers and drive states (e.g. on a sim restart)."""
        if getattr(self, "sensors", None) is not None:
            self.sensors.reset(); self.estimator.reset(); self.heave.reset(); self._est_ready = False; self._last_cmds = dict.fromkeys(("delta_main", "delta_elev", "delta_roll"), 0.0)
        if getattr(self, "lqr", None) is not None:
            self.lqr.reset()
        self._dw_t, self._dw_eps, self._eps_used = [], [], 0.0
        self.controller.reset()
        self.surge.reset()
        if getattr(self, "supervisor", None) is not None:
            self.supervisor.reset()
        for d in self.drives.values():
            d.reset()
        self._slew_last = None

    def twist_stiffness(self, name: str) -> float | None:
        """Spanwise twist stiffness of a surface at its drive [N m/rad], for the span-mean deflection under a
        uniform hinge load: 3 GJ / L for a flap driven at one end (L = flapped span); the all-moving elevator is
        driven at mid-span through its strut, two halves of L/2 in parallel: 12 GJ / L. None = rigid."""
        p = self.params
        if name in ("left", "right"):
            gj = p.get("main_flap_GJ")
            L = p["main_span"] * (p.get("main_flap_span_frac", 1.0) if p.get("main_actuation", "flap") == "flap" else 1.0)
            return None if gj is None else 3.0 * gj / L
        gj = p.get("elev_GJ")
        return None if gj is None else 12.0 * gj / p["elev_span"]

    def _servo_commands(self, raw: dict, state, env) -> dict:
        """Mix the virtual PID channels onto the physical drives and advance the drive dynamics."""
        p = self.params
        u = max(float(state[6]), 0.1)
        vs = p.get("q_schedule_speed", 0.0)
        k = float(np.clip((vs / u) ** 2, 0.25, 1.0)) if vs > 0 else 1.0   # gain-schedule flap commands by 1/q above vs
        main, roll, elev = k * raw["delta_main"], k * raw["delta_roll"], k * raw["delta_elev"]
        target = {"left": main - roll, "right": main + roll, "elev": elev}
        if self.supervisor is not None:
            target = self.supervisor.mix(target)          # jammed-flap reconfiguration
        slew = p.get("cmd_slew_deg_s")
        if slew:                                           # rate-limit each drive's command (takeoff steps)
            step = slew * p["dt"]
            if self._slew_last is None:                    # start from where the surfaces are, not the first command
                self._slew_last = {n: float(np.degrees(self.drives[n].delta)) for n in target}
            target = {n: float(np.clip(v, self._slew_last[n] - step, self._slew_last[n] + step))
                      for n, v in target.items()}
            self._slew_last = dict(target)
        self._last_target = target
        flows = flow_models(self, state, env, self.hinges)
        self._last_flows = flows                          # hinge model at this flow (supervisor fault residuals)
        for d in self.drives.values():                     # live gear ratio / bus voltage (UI sliders)
            d.spec.gear.ratio, d.spec.ctrl.v_bus = p["gear_ratio"], p["v_bus"]
        ang = {n: self.drives[n].update(target[n], flows[n]) for n in self.drives}
        return {"delta_main": 0.5 * (ang["left"] + ang["right"]),
                "delta_roll": 0.5 * (ang["right"] - ang["left"]),
                "delta_elev": ang["elev"],
                "target": target,
                "drive": {n: dict(self.drives[n].telemetry) for n in self.drives}}

    def _downwash_qs(self) -> float:
        """Quasi-steady downwash angle at the tail [rad] from the main foils' latest corrected CL:
        eps = k * 2 CL / (pi AR_wing), AR on the full two-panel span."""
        l, r = self.left_foil, self.right_foil
        ar_w = (l.geometry.span + r.geometry.span) / l.geometry.chord
        return self.params.get("downwash_factor", 1.0) * (l.last_CL + r.last_CL) / (np.pi * ar_w)

    def _downwash_lagged(self, t: float, u: float) -> float:
        """Downwash the tail sees NOW: the wing's value from l/u seconds ago (the wake must be convected
        the wing->tail distance). params['downwash_lag'] = False uses the instantaneous value."""
        if not self.params.get("downwash_lag", True) or not self._dw_t:
            return self._downwash_qs() if not self.params.get("downwash_lag", True) else 0.0
        l = abs(self.elev_foil.r_attach[0] - self.left_foil.r_attach[0])
        return float(np.interp(t - l / max(float(u), 0.5), self._dw_t, self._dw_eps))

    def _trim_ff(self, u: float) -> float:
        """Speed-scheduled trim flap angle [deg], added to the heave PID output so the integrator no longer
        has to ramp up to the weight-supporting angle. Lift ~ u^2, so the angle of attack the wing needs goes
        as 1/u^2; the fixed incidence supplies part of it and the flap the rest:
            eff * delta_trim(u) + incidence = (eff * d0 + incidence) * (vs / u)^2."""
        p = self.params
        d0, vs = p.get("trim_ff_deg", 0.0), p.get("trim_ff_speed", 0.0)
        if d0 == 0.0 or vs <= 0.0:
            return 0.0
        inc = p.get("main_incidence_deg", 0.0) / self.main_flap_eff    # incidence in flap degrees
        return float(np.clip((d0 + inc) * (vs / max(float(u), 0.5 * vs)) ** 2 - inc, -20.0, 20.0))

    def rest_state(self) -> np.ndarray:
        """Initial state: at rest, floating at the hull's buoyant equilibrium draft."""
        p = self.params
        rest_draft = min(p["mass"] / (RHO_WATER * self.waterplane_area), p["max_draft"])
        z0 = p["hull_offset"] + rest_draft
        eta0 = np.array([0.0, 0.0, z0, 0.0, np.radians(2.0), 0.0])
        nu0 = np.zeros(6)
        return np.concatenate([eta0, nu0])

    def _foils(self):
        return {"left": self.left_foil, "right": self.right_foil, "elev": self.elev_foil,
                "main_strut": self.main_strut, "elev_strut": self.elev_strut}

    def wave_sample(self, eta, t, wave) -> WaveSample:
        """Sample the wave field at the hull and at every foil's world position."""
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        smp = WaveSample()
        smp.elevation = wave.elevation(eta[0], eta[1], t)
        smp.hull_surface_z = -smp.elevation          # NED: surface risen -> smaller (more negative) z
        cells = self.hull.cell_world_xy(eta)         # where each hull footprint cell actually is in the world
        elev_cells = np.zeros(self.hull.n) + wave.elevation(cells[:, 0], cells[:, 1], t)
        smp.hull_cell_surface_z = -elev_cells
        for name, foil in self._foils().items():
            pos = eta[:3] + R @ foil.r_attach        # foil AC in the NED world frame
            elev = wave.elevation(pos[0], pos[1], t)
            smp.foil_surface_z[name] = -elev
            # orbital_velocity wants depth below the still surface as a NEGATIVE z (z <= 0)
            v_ned = wave.orbital_velocity(pos[0], pos[1], -pos[2], t)
            smp.foil_vel_body[name] = R.T @ v_ned
        return smp

    @staticmethod
    def calm_sample() -> WaveSample:
        z3 = np.zeros(3)
        names = ("left", "right", "elev", "main_strut", "elev_strut")
        return WaveSample(0.0, 0.0, 0.0, {n: 0.0 for n in names}, {n: z3 for n in names})

    def _submergence_over_chord(self, foil, eta, water_surface_z):
        """
        This foil's depth below the (possibly wave-displaced) local water
        surface, divided by its chord — feeds Foil.generalized_force's
        free-surface correction. Approximates the foil AC's world-frame z by
        rotating its body-frame r_attach by the current attitude, same
        convention hull_draft() uses for the hull. Can go negative if the
        foil is above the local surface; free_surface_lift_factor() clamps
        that to 0 internally.
        """
        R = R_body_to_ned(eta[3], eta[4], eta[5])
        world_z = eta[2] + (R @ foil.r_attach)[2]
        return (world_z - water_surface_z) / foil.geometry.chord

    def physics_rhs(self, state, cmds, thrust, env: WaveSample | None = None):
        """
        Pure nonlinear dynamics d(state)/dt. env is a WaveSample (None = calm).
        Each foil sees the body velocity minus the wave orbital velocity AT ITS OWN
        position; hull drag and heave damping use the raw body velocity.
        """
        p = self.params
        env = env or self.calm_sample()
        eta, nu = state[:6], state[6:]

        def foil_force(name, delta, downwash=0.0):
            foil = self._foils()[name]
            nu_rel = nu.copy()
            nu_rel[:3] -= env.foil_vel_body[name]
            return foil.generalized_force(
                nu_rel, delta=delta,
                submergence_over_chord=self._submergence_over_chord(foil, eta, env.foil_surface_z[name]),
                downwash_rad=downwash)

        def strut_force(name):
            strut = self._foils()[name]
            nu_rel = nu.copy()
            nu_rel[:3] -= env.foil_vel_body[name]
            h = self._submergence_over_chord(strut, eta, env.foil_surface_z[name]) * strut.geometry.chord
            strut_len = p["hull_offset"] + strut.r_attach[2]
            wet = float(np.clip(h, 0.0, strut_len))
            r = strut.r_attach - np.array([0.0, 0.0, 0.5 * wet])   # AC at mid wetted length
            return strut.generalized_force(nu_rel, span=wet, r_attach=r)

        tau = np.zeros(6)
        tau += strut_force("main_strut")
        tau += strut_force("elev_strut")
        tau += foil_force("left", cmds["delta_main"] - cmds["delta_roll"])
        tau += foil_force("right", cmds["delta_main"] + cmds["delta_roll"])
        # Wing -> tail downwash: self._eps_used (transport-lagged, see _downwash_lagged)
        tau += foil_force("elev", cmds["delta_elev"], downwash=self._eps_used)
        tau += self.hull.buoyancy(eta, env.hull_cell_surface_z)
        tau += gravity_force(p["mass"], eta)
        tau += self.hull.drag(nu, self.hull.mean_draft(eta, env.hull_cell_surface_z), p["hull_drag_coeff"], p["hull_rot_damping"])
        tau[0] += thrust
        tau[2] -= p["heave_damping"] * nu[2]
        return np.concatenate([J(eta) @ nu, self.rigid_body.nu_dot(nu, tau)])

    def step(self, state: np.ndarray, t: float, wave, cmds_override: dict | None = None):
        """
        Advance one full RK4 step: evaluate the controller and surge
        controller once (held fixed across the RK4 sub-stages, same as
        wave state), integrate, and return everything a caller needs to
        broadcast/log.

        cmds_override : optional dict of delta_main/delta_elev/delta_roll [deg] that
                        bypasses the controller (open-loop experiments).

        Returns
        -------
        new_state, cmds, thrust, draft, wave_elev   (draft is measured on new_state)
        """
        p = self.params
        dt = p["dt"]
        eta, nu = state[:6], state[6:]

        # phi_ref/theta_ref are stored in degrees (UI convention); eta's
        # angles are radians (Fossen convention) — convert before compare.
        env = self.wave_sample(eta, t, wave)
        cstate = self._control_state(state, t, env.elevation)   # what the controllers see (== state if sensors ideal)
        if self.supervisor is not None and cmds_override is None:
            self.supervisor.update(t, cstate)                  # mode, references, limits (control/supervisor.py)
        z_ref = p["hull_offset"] - p["flying_height"]
        ceta, cnu = cstate[:6], cstate[6:]
        self._eps_used = self._downwash_lagged(t, nu[0])      # frozen over the step; drive loads use it too
        if cmds_override is None and self.actuator_model == "servo":
            ff = self._trim_ff(cnu[0])
            self.controller.heave_pid.output_min = -20.0 - ff   # anti-windup judges the TOTAL flap command
            self.controller.heave_pid.output_max = 20.0 - ff
            raw = self.controller.pid_commands(
                z_ref,                       ceta[2],
                np.radians(p["phi_ref"]),    ceta[3],
                np.radians(p["theta_ref"]),  ceta[4],
            )
            raw["delta_main"] += ff
            if self.lqr is not None:
                dl, dr, de = (self.drives[n] for n in ("left", "right", "elev"))   # drive encoders: angle, rate
                act = ([0.5 * np.degrees(dl.delta + dr.delta), np.degrees(de.delta), 0.5 * np.degrees(dr.delta - dl.delta)],
                       [0.5 * np.degrees(dl.rate + dr.rate), np.degrees(de.rate), 0.5 * np.degrees(dr.rate - dl.rate)])
                raw = self.lqr.update(cstate, z_ref, np.radians(p["phi_ref"]), np.radians(p["theta_ref"]), raw, act)
            if self.supervisor is not None:
                raw = self.supervisor.shape_raw(raw, cstate)
            cmds = self._servo_commands(raw, state, env)
        elif cmds_override is None:
            cmds = self.controller.update(
                z_ref,                       eta[2],
                np.radians(p["phi_ref"]),    eta[3],
                np.radians(p["theta_ref"]),  eta[4],
            )
        else:
            cmds = dict(cmds_override)
        thrust = self.surge.update(p["target_speed"], cnu[0])
        self._last_cmds = {k: cmds[k] for k in ("delta_main", "delta_elev", "delta_roll")}
        wave_elev = env.elevation

        def rhs(s):
            return self.physics_rhs(s, cmds, thrust, env)

        k1 = rhs(state)
        k2 = rhs(state + dt / 2 * k1)
        k3 = rhs(state + dt / 2 * k2)
        k4 = rhs(state + dt * k3)
        new_state = state + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

        if not np.all(np.isfinite(new_state)):
            # Recovered, not ignored: snap back to the last valid state with
            # zero velocity/rates and reset the controllers, so the pilot
            # regains control instead of the app streaming NaN forever.
            new_state = state.copy()
            new_state[6:] = 0.0
            self.reset()
        else:
            new_state[3] = np.clip(new_state[3], -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD)
            new_state[4] = np.clip(new_state[4], -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD)
            new_state[6:9] = np.clip(new_state[6:9], -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
            new_state[9:12] = np.clip(new_state[9:12], -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

        # last_CL is from the final RK stage (~ end of step): record it for the transport delay
        self._dw_t.append(t + dt); self._dw_eps.append(self._downwash_qs())
        while len(self._dw_t) > 1 and self._dw_t[1] < t + dt - 5.0:
            del self._dw_t[0], self._dw_eps[0]
        draft = self.hull.mean_draft(new_state[:6], env.hull_cell_surface_z)   # footprint-averaged draft
        return new_state, cmds, thrust, draft, wave_elev
