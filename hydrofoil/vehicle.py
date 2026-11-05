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
    MAIN_FOIL_FLAP_EFFECTIVENESS, ELEVATOR_FLAP_EFFECTIVENESS, MAIN_FOIL_OSWALD_EFFICIENCY,
)
from hydrofoil.forces.buoyancy import gravity_force
from hydrofoil.forces.hull import hull_buoyancy_force, hull_drag_force, hull_draft
from hydrofoil.control.pid import HydrofoilController
from hydrofoil.control.surge import SurgeController
from hydrofoil.control.actuator import DEFAULT_ACTUATOR_PARAMS


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
    hull_surface_z: float = 0.0                  # NED z of the surface at the hull
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
    kp/ki/kd_speed, max_thrust, waterplane_area, max_draft, hull_offset,
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
        self.left_foil = Foil(
            FoilGeometry(chord=p["main_chord"], span=p["main_span"]),
            polar,
            r_attach=np.array([p["main_pos_x"], -p["main_lateral_offset"], p["main_pos_z"]]),
            flap_effectiveness=MAIN_FOIL_FLAP_EFFECTIVENESS,
            oswald_efficiency=MAIN_FOIL_OSWALD_EFFICIENCY,
        )
        self.right_foil = Foil(
            FoilGeometry(chord=p["main_chord"], span=p["main_span"]),
            polar,
            r_attach=np.array([p["main_pos_x"], p["main_lateral_offset"], p["main_pos_z"]]),
            flap_effectiveness=MAIN_FOIL_FLAP_EFFECTIVENESS,
            oswald_efficiency=MAIN_FOIL_OSWALD_EFFICIENCY,
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

        pid_gains = {
            "heave": {"Kp": p["kp_heave"], "Ki": p["ki_heave"], "Kd": p["kd_heave"]},
            "pitch": {"Kp": p["kp_pitch"], "Ki": p["ki_pitch"], "Kd": p["kd_pitch"]},
            "roll":  {"Kp": p["kp_roll"],  "Ki": p["ki_roll"],  "Kd": p["kd_roll"]},
        }
        actuator_params = p.get("actuator_params") or DEFAULT_ACTUATOR_PARAMS
        self.controller = HydrofoilController(pid_gains, dt=p["dt"], actuator_params=actuator_params)
        self.surge = SurgeController(Kp=p["kp_speed"], Ki=p["ki_speed"], Kd=p["kd_speed"],
                                      dt=p["dt"], max_thrust=p["max_thrust"])

    def rest_state(self) -> np.ndarray:
        """Initial state: at rest, floating at the hull's buoyant equilibrium draft."""
        p = self.params
        rest_draft = min(p["mass"] / (RHO_WATER * p["waterplane_area"]), p["max_draft"])
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
        return WaveSample(0.0, 0.0, {n: 0.0 for n in names}, {n: z3 for n in names})

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

        def foil_force(name, delta):
            foil = self._foils()[name]
            nu_rel = nu.copy()
            nu_rel[:3] -= env.foil_vel_body[name]
            return foil.generalized_force(
                nu_rel, delta=delta,
                submergence_over_chord=self._submergence_over_chord(foil, eta, env.foil_surface_z[name]))

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
        tau += foil_force("elev", cmds["delta_elev"])
        tau += hull_buoyancy_force(eta, p["hull_offset"], p["waterplane_area"], p["max_draft"],
                                    water_surface_z=env.hull_surface_z)
        tau += gravity_force(p["mass"], eta)
        tau += hull_drag_force(nu, eta, p["hull_offset"], p["max_draft"], p["hull_drag_coeff"],
                                water_surface_z=env.hull_surface_z)
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

        z_ref = p["hull_offset"] - p["flying_height"]
        # phi_ref/theta_ref are stored in degrees (UI convention); eta's
        # angles are radians (Fossen convention) — convert before compare.
        if cmds_override is None:
            cmds = self.controller.update(
                z_ref,                       eta[2],
                np.radians(p["phi_ref"]),    eta[3],
                np.radians(p["theta_ref"]),  eta[4],
            )
        else:
            cmds = dict(cmds_override)
        thrust = self.surge.update(p["target_speed"], nu[0])
        env = self.wave_sample(eta, t, wave)
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
            self.controller.reset()
            self.surge.reset()
        else:
            new_state[3] = np.clip(new_state[3], -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD)
            new_state[4] = np.clip(new_state[4], -MAX_ATTITUDE_RAD, MAX_ATTITUDE_RAD)
            new_state[6:9] = np.clip(new_state[6:9], -MAX_LINEAR_VEL, MAX_LINEAR_VEL)
            new_state[9:12] = np.clip(new_state[9:12], -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

        draft = hull_draft(new_state[:6], p["hull_offset"], p["max_draft"], env.hull_surface_z)
        return new_state, cmds, thrust, draft, wave_elev
