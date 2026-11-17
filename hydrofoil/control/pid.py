"""
Cascade PID controller for a fully submerged T-foil.

Three independent loops:
    heave  : altitude error → main foil flap command (delta_main)
    pitch  : pitch error    → elevator command        (delta_elev)
    roll   : roll error     → differential main flap  (delta_roll)

Each loop is a standard discrete-time PID with anti-windup clamping.
"""

import numpy as np

from hydrofoil.control.actuator import Actuator, DEFAULT_ACTUATOR_PARAMS


class PID:
    """
    Single-axis discrete-time PID with integrator anti-windup.

    Parameters
    ----------
    Kp, Ki, Kd  : gains
    dt           : sample time [s]
    output_min,
    output_max   : output clamp [same units as setpoint]
    """

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        dt: float,
        output_min: float = -np.inf,
        output_max: float = np.inf,
    ):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_measurement = None   # None = no derivative on the next call

    def reset(self):
        self._integral = 0.0
        self._prev_measurement = None

    def update(self, error: float, measurement: float, integrate: bool = True) -> float:
        """
        Compute control output for one timestep.

        Parameters
        ----------
        error       : the caller's error signal (sign convention is the
                      caller's choice — this class doesn't assume one)
        measurement : the raw process value error was derived from, used
                      ONLY for the derivative term (see below)
        integrate   : if False, hold the integrator state (used when this loop is
                      on standby behind LQR or a mode override)

        Derivative-on-measurement, not derivative-on-error: a sudden
        setpoint change (e.g. a pilot stick input) makes `error` jump
        discontinuously even though the plant hasn't moved, which would
        otherwise spike the D-term ("derivative kick") into a huge one-shot
        command on the very step the setpoint changes. Deriving from the
        measurement instead means the D-term only reacts to how fast the
        plant is actually moving, immune to setpoint jumps. Requires error
        to vary the same direction as measurement (true for every caller in
        this codebase's convention); reset() clears the reference so the
        first call after a restart also produces no spurious kick.

        Returns
        -------
        output : clamped control signal
        """
        if self._prev_measurement is None:
            derivative = 0.0
        else:
            derivative = (measurement - self._prev_measurement) / self.dt
        candidate = self._integral + error * self.dt if integrate else self._integral
        output = self.Kp * error + self.Ki * candidate + self.Kd * derivative
        clamped = float(np.clip(output, self.output_min, self.output_max))

        # Conditional-integration anti-windup: freeze the integrator only while the
        # output is saturated AND the error is driving it further into saturation.
        # (The old rule froze it whenever the output was clamped, even if the error
        # was pulling the output back out of saturation, which delayed recovery.)
        if integrate:
            pushing_deeper = (output > self.output_max and error > 0.0) or \
                             (output < self.output_min and error < 0.0)
            if not pushing_deeper:
                self._integral = candidate

        self._prev_measurement = measurement
        return clamped


class HydrofoilController:
    """
    Cascade PID for a T-foil with main foil + elevator.

    Outputs:
        delta_main  : symmetric main foil flap angle [deg]
        delta_elev  : elevator angle [deg]
        delta_roll  : differential main foil angle (left - right) [deg]

    Usage:
        ctrl = HydrofoilController(gains, dt=0.02)
        cmds = ctrl.update(z_ref, z_meas, phi_ref, phi_meas, theta_ref, theta_meas)
    """

    def __init__(self, gains: dict, dt: float, actuator_params: dict = None):
        """
        Parameters
        ----------
        gains           : dict with keys 'heave', 'pitch', 'roll', each containing
                          {'Kp': ..., 'Ki': ..., 'Kd': ...}
        dt              : sample time [s]
        actuator_params : kwargs for control.actuator.Actuator (rate_limit_deg_s,
                          tau, delay_steps), applied to every flap channel.
                          Defaults to DEFAULT_ACTUATOR_PARAMS (BLDC placeholder).
        """
        lim = 20.0   # ± 20 deg flap limit — adjust to your servo range
        self.dt = dt

        self.heave_pid = PID(dt=dt, output_min=-lim, output_max=lim,
                             **gains['heave'])
        self.pitch_pid = PID(dt=dt, output_min=-lim, output_max=lim,
                             **gains['pitch'])
        self.roll_pid  = PID(dt=dt, output_min=-lim, output_max=lim,
                             **gains['roll'])

        ap = dict(actuator_params or DEFAULT_ACTUATOR_PARAMS)
        # Latency is specified in seconds (delay_s) so it does not silently change with
        # dt; an explicit delay_steps (legacy / tests) still wins.
        delay_s = ap.pop("delay_s", 0.0)
        if "delay_steps" not in ap:
            ap["delay_steps"] = int(round(delay_s / dt))
        self.main_actuator = Actuator(dt=dt, **ap)
        self.elev_actuator = Actuator(dt=dt, **ap)
        self.roll_actuator = Actuator(dt=dt, **ap)

    def update(
        self,
        z_ref: float,     z_meas: float,
        phi_ref: float,   phi_meas: float,
        theta_ref: float, theta_meas: float,
    ) -> dict:
        """
        Compute foil deflection commands.

        Units: z in metres; every angle (phi, theta and their references) in
        RADIANS (Fossen convention, same as the state vector). The roll and pitch
        errors are converted to DEGREES before the PID so the gains are
        degrees-of-flap per degree-of-error; the heave gains are degrees per metre.
        (Previously radian errors were fed to a degree-output PID, making the
        angle gains 57.3x weaker than their numerical values suggested.)

        Sign convention: positive delta → increased AoA → increased lift.
        NED frame: z positive downward, so altitude error = z_meas - z_ref
        (deeper = more positive = we want to rise = positive lift needed).

        Returns
        -------
        dict with 'delta_main', 'delta_elev', 'delta_roll' in [deg] — actual
        (actuator-filtered) flap angles, not raw PID output.
        """
        raw = self.pid_commands(z_ref, z_meas, phi_ref, phi_meas, theta_ref, theta_meas)
        return {
            'delta_main': self.main_actuator.update(raw['delta_main']),
            'delta_elev': self.elev_actuator.update(raw['delta_elev']),
            'delta_roll': self.roll_actuator.update(raw['delta_roll']),
        }

    def pid_commands(self, z_ref, z_meas, phi_ref, phi_meas, theta_ref, theta_meas,
                     integrate: bool = True) -> dict:
        """Raw PID outputs [deg] on the three virtual channels, before any actuator model
        (used directly by the servo drive model, which mixes them onto the physical drives)."""
        heave_err = z_meas - z_ref      # +ve: too deep → need more lift to rise
        pitch_err = np.degrees(theta_meas - theta_ref)  # +ve: nose up → need upward rear force → –M_y → nose down
        roll_err  = np.degrees(phi_meas - phi_ref)      # +ve: rolled right → need upward left force

        cmd_main = self.heave_pid.update(heave_err, z_meas, integrate=integrate)
        cmd_elev = self.pitch_pid.update(pitch_err, np.degrees(theta_meas), integrate=integrate)
        cmd_roll = self.roll_pid.update(roll_err, np.degrees(phi_meas), integrate=integrate)

        return {'delta_main': cmd_main, 'delta_elev': cmd_elev, 'delta_roll': cmd_roll}

    def reset(self):
        self.heave_pid.reset()
        self.pitch_pid.reset()
        self.roll_pid.reset()
        self.main_actuator.reset()
        self.elev_actuator.reset()
        self.roll_actuator.reset()
