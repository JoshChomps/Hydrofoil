"""
Flight-mode supervisor: the operating protocol that takes the boat from displacement to foiling and back, keeps it
inside a speed / height envelope while it flies, and answers faults (docs/19, "Operating protocol").

It sits above the existing controllers and only moves their references and limits:

    params["target_speed"], params["flying_height"], params["theta_ref"], params["phi_ref"]  (references)
    surge thrust limits (no reverse thrust; thrust cut)                                        (limits)
    a mixer override for a jammed flap (the healthy flap balances roll about the jammed one)    (reconfiguration)

so the LQR / PID / estimator are unchanged. Everything it decides on is measurable on the boat: the estimated
state the controllers see (height above the mean surface, attitude, speed), the drive encoders and telemetry, and
the sensors' fresh-sample flags. It never reads the true state.

Modes
    DISPLACEMENT  hull in the water. Flaps held at `disp_flap_deg` (no lift wanted), speed = pilot request capped at
                  `v_disp_max`. Foil request -> TAKEOFF if no fault is latched.
    TAKEOFF       speed ref -> v_takeoff. Height ref held at the hull's floating height until u >= v_liftoff, then
                  ramped up at `climb_rate` to the requested clearance. -> FOILING when the hull is clear and the
                  height error is small; -> LANDING (abort) if liftoff is not reached in `takeoff_timeout` or an
                  attitude limit is exceeded.
    FOILING       pilot speed clamped to [v_min_foil, v_max]; pilot height clamped to [h_min + wave_margin x wave amplitude, h_max];
                  references rate-limited. Land request -> LANDING.
    LANDING       height ref ramped down at `sink_rate` (speed held >= v_min_foil so the foils keep authority) until
                  the hull is in the water, then speed ref -> pilot displacement speed and -> DISPLACEMENT.
    FAULT modes   see `_fault_response`.

Faults detected (drive checks only while flying above v_disp_max)
    The drive residual is |encoder - command - M_hinge / kp| [deg]: the position error left after the static
    deflection the hinge load explains (M_hinge from the hinge model at the measured flow). A drive is lost when the
    residual exceeds `fault_err_deg` and either the encoder has not moved over `still_time` (jammed), or it persists
    for `fault_time` (`elev_fault_time` for the elevator) while the drive is NOT torque-limited (power loss), or for
    `fault_time_limited` regardless.
    flap_left / flap_right a main-flap drive lost; the angle it stopped at is kept in `jam`
    elev_loss              elevator drive lost
    height_lost            no fresh height sample for `sensor_timeout` s (noisy-sensor mode only)
    speed_lost             no fresh speed sample for `sensor_timeout` s (noisy-sensor mode only)
    attitude               |roll| > roll_limit or |pitch| > pitch_limit while foiling
"""

from __future__ import annotations

import numpy as np

DEFAULT_SUPERVISOR = {
    "v_disp_max": 4.0,          # m/s: displacement speed cap (the hull hump makes faster displacement running wasteful)
    "v_liftoff": 5.5,           # m/s: start raising the height ref (trim flap about +13 deg at 5 m/s, 275 kg)
    "v_takeoff": 6.5,           # m/s: speed ref during takeoff (hull clears at 5.6-6.3 m/s)
    "v_min_foil": 6.0,          # m/s: speed floor while foiling (trim flap about +7 deg: margin for waves both ways)
    "v_max": 9.5,               # m/s: cavitation number about 2.3, trim flap about -7 deg
    "h_min": 0.15,              # m: lowest hull clearance in calm water
    "h_max": 0.6,               # m: highest (strut 1.4 m; foil stays >= 0.8 m deep)
    "h_default": 0.25,          # m: clearance used if the pilot asks for nothing else
    "wave_margin": 1.2,         # the lowest clearance is h_min + wave_margin x estimated wave amplitude
    "climb_rate": 0.10,         # m/s: height-ref ramp up
    "sink_rate": 0.08,          # m/s: height-ref ramp down when landing
    "speed_rate": 0.5,          # m/s^2: speed-ref ramp
    "takeoff_timeout": 25.0,    # s: abort takeoff if liftoff speed not reached
    "disp_flap_deg": 0.0,       # flap angle held in displacement mode
    "roll_limit": 15.0,         # deg: attitude fault while foiling
    "pitch_limit": 8.0,
    "fault_err_deg": 4.0,       # encoder vs command error that counts as a drive fault
    "fault_time": 0.30,         # s it must persist (flaps)
    "elev_fault_time": 0.15,    # s (elevator: the boat diverges in about 1 s without it)
    "still_time": 0.20,         # s: a drive whose encoder moved less than still_deg in this window while off its
    "still_deg": 0.2,           # command by > fault_err_deg is jammed
    "fault_time_limited": 1.5,  # s: same error while the drive reports torque / voltage limiting (a jam stalls the
                                # motor at its limit; a big hinge moment does too, briefly)
    "sensor_timeout": 0.5,      # s without a fresh sample
    "touchdown_clearance": 0.0, # m: hull clearance at or below which the hull is taken to be in the water
    "jam_land_thrust": 0.0,     # N: thrust while landing after a flap jam
}

MODES = ("DISPLACEMENT", "TAKEOFF", "FOILING", "LANDING", "EMERGENCY_LANDING")


class Supervisor:
    def __init__(self, vp, cfg: dict | None = None):
        self.vp = vp
        self.c = dict(DEFAULT_SUPERVISOR, **(cfg or {}))
        p = vp.params
        # pilot requests (the UI / tests write these)
        self.pilot = {"foil": False, "speed": float(p.get("target_speed", 0.0)),
                      "height": float(self.c["h_default"]), "bank": 0.0}
        self._lqr_engage = vp.lqr.engage_speed if vp.lqr is not None else None
        self._kp0 = {n: vp.drives[n].kp for n in ("left", "right", "elev")}   # configured servo gains
        self.reset()

    # ------------------------------------------------------------------ state
    def reset(self):
        self.mode = "DISPLACEMENT"
        self.t_mode = 0.0
        self.faults: dict[str, float] = {}           # name -> time detected (latched)
        self.fault_info: dict = {}
        self._err_t = {"left": 0.0, "right": 0.0, "elev": 0.0}
        self._lim_t = {"left": 0.0, "right": 0.0, "elev": 0.0}
        self._pos_hist = {"left": [], "right": [], "elev": []}
        self._tgt_hist = {"left": [], "right": [], "elev": []}
        self._since = {"z": 0.0, "u": 0.0}
        self._last_h_val = None
        self._h_same_t = 0.0
        self._last_reject_events = 0
        self.speed_ref = 0.0
        self.height_ref = None
        self.jam = None                              # ("left"|"right", angle deg) when a main-flap drive is lost
        self.wave_amp = 0.0
        self._wave_hist = []
        self.log = []                                # (t, from, to, reason)
        self._down_t = 0.0
        self._down_h_t = 0.0

    def _set_mode(self, t, mode, reason=""):
        if mode != self.mode:
            self.log.append((round(t, 2), self.mode, mode, reason))
            self.mode, self.t_mode = mode, t

    @property
    def status(self) -> dict:
        """What a UI shows: mode, references, envelope and latched faults."""
        return {"mode": self.mode, "speed_ref": self.speed_ref, "height_ref": self.height_ref,
                "v_min": self.c["v_min_foil"], "v_max": self.c["v_max"], "h_min": self._h_min(),
                "h_max": self.c["h_max"], "faults": sorted(self.faults), "jam": self.jam,
                "wave_amp": self.wave_amp, "log": list(self.log[-5:])}

    # -------------------------------------------------------------- helpers
    def _hull_clearance(self, cstate) -> float:
        """Hull bottom above the mean surface [m] from the controllers' height estimate."""
        return float(self.vp.params["hull_offset"] - cstate[2])

    def _h_min(self) -> float:
        return self.c["h_min"] + self.c["wave_margin"] * self.wave_amp

    def _ramp(self, cur, target, rate, dt):
        return cur + float(np.clip(target - cur, -rate * dt, rate * dt))

    def _update_wave_estimate(self, t, dt):
        """Wave amplitude under the boat [m] while FOILING: sqrt(2) x 10 s std of (raw range - estimated craft depth).
        Subtracting the IMU-driven heave depth removes takeoff climbs and pilot height changes (which otherwise
        booked calm climbs as large waves)."""
        sens = self.vp.sensors
        if sens is None or sens.channels["height"].value is None or "z" not in sens.fresh:
            return
        if self.mode != "FOILING":
            return
        h_raw = float(np.ravel(sens.channels["height"].value)[0])
        val = h_raw - self.vp.heave.z if self.vp.heave.ready else h_raw
        self._wave_hist.append(val)
        n = int(10.0 * 20.0)                         # 10 s of 20 Hz samples
        del self._wave_hist[:-n]
        if len(self._wave_hist) > n // 2:
            self.wave_amp = float(np.sqrt(2.0) * np.std(self._wave_hist))

    # ------------------------------------------------------- fault detection
    def _detect(self, t, dt, cstate):
        vp, c = self.vp, self.c
        tgt = getattr(vp, "_last_target", None)
        flying = self.mode in ("TAKEOFF", "FOILING", "LANDING") and float(cstate[6]) > c["v_disp_max"]
        if tgt is not None:
            for n in ("left", "right", "elev"):
                d = vp.drives[n]
                th = self._tgt_hist[n]
                if not th and d.delay_steps > 0:
                    th.extend([0.0] * d.delay_steps)
                th.append(float(tgt[n]))
                del th[:-(d.delay_steps + 1)]
        if tgt is not None and flying:
            for n in ("left", "right", "elev"):
                d = vp.drives[n]
                # Compare against the command that actually reached the drive after its transport delay
                cmd_del = self._tgt_hist[n][0] if self._tgt_hist[n] else tgt[n]
                cmd = float(np.clip(cmd_del, -np.degrees(d.stop), np.degrees(d.stop)))
                # residual: the position error left after the static deflection the hinge load explains
                # (kp * err = M at rest). M comes from the hinge model at the measured flow and encoder angle,
                # which the flight computer can evaluate too.
                flows = getattr(vp, "_last_flows", None)
                m_exp = float(flows[n](d.delta)) if flows is not None else 0.0
                err = abs(np.degrees(d.delta) - cmd - np.degrees(m_exp / self._kp0[n]))
                bad = err > c["fault_err_deg"]
                limited = bool(d.telemetry.get("limited", False))
                # a drive at its torque / voltage limit is still working (big hinge moment, fast command): only a
                # long-lasting error counts. A drive with no torque (power loss) or that fails to follow while NOT
                # limited is lost quickly.
                # stuck: the encoder has not moved (< still_deg) over the last still_time while it should have
                hist = self._pos_hist[n]
                hist.append(np.degrees(d.delta))
                del hist[:-max(int(c["still_time"] / dt), 2)]
                self._err_t[n] = self._err_t[n] + dt if bad else 0.0
                # stuck only if the error has lasted the whole window too: right after a command step a healthy
                # drive has not moved yet either (this tripped on the LQR handover step)
                stuck = (bad and len(hist) >= int(c["still_time"] / dt) and (max(hist) - min(hist)) < c["still_deg"]
                         and self._err_t[n] >= c["still_time"])
                self._lim_t[n] = self._lim_t[n] + dt if (bad and not limited) else 0.0
                fast = c["elev_fault_time"] if n == "elev" else c["fault_time"]
                if stuck or self._lim_t[n] > fast or self._err_t[n] > c["fault_time_limited"]:
                    i_drv = abs(float(d.telemetry.get("I", 0.0)))
                    mode_tag = "limp" if i_drv < 0.5 else ("jam" if stuck else "fault")
                    self.fault_info[n] = {"t": t, "err": err, "stuck": bool(stuck), "limited": limited,
                                          "mode": mode_tag, "err_t": self._err_t[n], "free_t": self._lim_t[n]}
                    key = "elev_loss" if n == "elev" else f"flap_{n}"
                    if key not in self.faults and (n == "elev" or self.jam is None):
                        self.faults[key] = t
                        if n != "elev":
                            self.jam = (n, float(np.degrees(d.delta)))
        if vp.sensors is not None:
            fresh = vp.sensors.fresh
            for k, name in (("z", "height_lost"), ("u", "speed_lost")):
                self._since[k] = 0.0 if k in fresh else self._since[k] + dt
                if self._since[k] > c["sensor_timeout"] and name not in self.faults:
                    self.faults[name] = t
            if "z" in fresh and vp.sensors.channels["height"].value is not None:
                h_val = float(np.ravel(vp.sensors.channels["height"].value)[0])
                if self._last_h_val is None or abs(h_val - self._last_h_val) >= 1e-6:
                    self._h_same_t = 0.0
                self._last_h_val = h_val
            if self.mode in ("TAKEOFF", "FOILING", "LANDING"):
                if self._last_h_val is not None:
                    self._h_same_t += dt
                    if self._h_same_t > c["sensor_timeout"] and (
                        abs(vp.heave.vz) > 0.05 or abs(self._last_h_val - vp.heave.z) > 0.05
                    ):
                        self.faults.setdefault("height_lost", t)
                if vp.heave.reject_events > self._last_reject_events:
                    self.faults.setdefault("height_lost", t)
            self._last_reject_events = vp.heave.reject_events
        if self.mode in ("FOILING", "TAKEOFF") and (abs(np.degrees(cstate[3])) > c["roll_limit"]
                                                    or abs(np.degrees(cstate[4])) > c["pitch_limit"]):
            self.faults.setdefault("attitude", t)

    # ------------------------------------------------------------ main step
    def update(self, t: float, cstate) -> None:
        """Called by VehiclePhysics.step before the controllers: sets references and limits."""
        vp, c, p = self.vp, self.c, self.vp.params
        dt = p["dt"]
        if self.height_ref is None:
            self.height_ref = self._hull_clearance(cstate)
        self._update_wave_estimate(t, dt)
        self._detect(t, dt, cstate)
        u = float(cstate[6])
        clear = self._hull_clearance(cstate)
        airborne = clear > c["touchdown_clearance"] + 0.03

        if self.faults and self.mode in ("TAKEOFF", "FOILING", "LANDING"):
            self._set_mode(t, "EMERGENCY_LANDING", ",".join(sorted(self.faults)))

        # the LQR is designed for the foilborne plant: keep the PID on takeoff until the hull is clear
        if vp.lqr is not None:
            vp.lqr.engage_speed = self._lqr_engage if (airborne and self.mode != "DISPLACEMENT") else np.inf
            # per-phase controller choice (params["phase_controllers"], docs/21): "pid" / "lqr" force it for the
            # phase, "auto" (or no entry) keeps the automatic handover; DISPLACEMENT also takes "fixed"
            choice = self._phase_choice()
            vp.lqr.force = {"pid": False, "fixed": False, "lqr": True}.get(choice)

        mode = self.mode
        pv = self.pilot
        h_target = float(np.clip(pv["height"], self._h_min(), c["h_max"]))
        thrust_max = p["max_thrust"]
        if mode == "DISPLACEMENT":
            spd = min(pv["speed"], c["v_disp_max"])
            self.height_ref = clear                                      # follow the hull: nothing to track
            if pv["foil"] and not self.faults:
                self._set_mode(t, "TAKEOFF", "pilot")
        elif mode == "TAKEOFF":
            spd = max(c["v_takeoff"], min(pv["speed"], c["v_max"]))
            if u >= c["v_liftoff"]:
                self.height_ref = self._ramp(self.height_ref, h_target, c["climb_rate"], dt)
            else:
                self.height_ref = min(self.height_ref, clear)
            if not pv["foil"]:
                self._set_mode(t, "LANDING", "pilot")
            elif airborne and abs(clear - h_target) < 0.03 and self.height_ref >= h_target - 1e-6:
                self._set_mode(t, "FOILING", "airborne")
            elif t - self.t_mode > c["takeoff_timeout"] and u < c["v_liftoff"]:
                self._set_mode(t, "LANDING", "takeoff timeout")
        elif mode == "FOILING":
            spd = float(np.clip(pv["speed"], c["v_min_foil"], c["v_max"]))
            self.height_ref = self._ramp(self.height_ref, h_target, c["climb_rate"], dt)
            if not pv["foil"]:
                self._set_mode(t, "LANDING", "pilot")
        elif mode == "LANDING":
            if airborne:
                spd = max(min(pv["speed"], c["v_max"]), c["v_min_foil"])
                self.height_ref = self._ramp(self.height_ref, -0.10, c["sink_rate"], dt)
            else:
                spd = min(pv["speed"], c["v_disp_max"])
                self._set_mode(t, "DISPLACEMENT", "hull down")
                self.landed_reset()
        else:                                                            # EMERGENCY_LANDING
            spd, thrust_max = self._fault_response(t, dt, clear, airborne)
            self._down_t = self._down_t + dt if not airborne else 0.0
            self._down_h_t = self._down_h_t + dt if ("height_lost" in self.faults and u < c["v_disp_max"]) else 0.0
            if (not airborne and (u < c["v_disp_max"] or ("speed_lost" in self.faults and self._down_t > 2.0))) or self._down_h_t > 2.0:
                self._set_mode(t, "DISPLACEMENT", "hull down (fault latched)")
                self.landed_reset()

        self.speed_ref = self._ramp(self.speed_ref, spd, c["speed_rate"], dt) if self.mode != "EMERGENCY_LANDING" \
            else spd
        p["target_speed"] = self.speed_ref
        p["flying_height"] = float(self.height_ref)
        p["phi_ref"] = float(np.clip(pv["bank"], -8.0, 8.0)) if self.mode == "FOILING" else 0.0
        pid = vp.surge.pid
        pid.output_min, pid.output_max = 0.0, thrust_max                 # a propeller gives no useful reverse thrust

    def _fault_response(self, t, dt, clear, airborne):
        """Speed ref and thrust limit in EMERGENCY_LANDING; height and mixer handled through refs / `mix`."""
        c, p = self.c, self.vp.params
        f = self.faults
        if "elev_loss" in f:
            # no control-surface cure on this layout: cut thrust at once; height ref down so the main flaps shed lift
            self.height_ref = self._ramp(self.height_ref, -0.10, 0.5, dt)
            return 0.0, 0.0
        if self.jam is not None:
            # healthy flap matches the failed one for roll (see `mix`); cut thrust so the lift falls with speed
            self.height_ref = self._ramp(self.height_ref, -0.10, c["sink_rate"], dt)
            return 0.0, c["jam_land_thrust"]
        if "height_lost" in f:
            # no reliable height feedback: cut thrust so the craft settles onto the hull while IMU stabilizes attitude
            self.height_ref = self._ramp(self.height_ref, -0.10, max(c["sink_rate"], 0.25), dt)
            return 0.0, 0.0
        # speed or attitude fault: normal landing profile, keep foil authority until the hull is down
        if airborne:
            self.height_ref = self._ramp(self.height_ref, -0.10, c["sink_rate"], dt)
            return max(c["v_min_foil"], min(self.speed_ref, c["v_max"])), p["max_thrust"]
        if "speed_lost" in f:
            return 0.0, 0.0
        return min(self.pilot["speed"], c["v_disp_max"]), p["max_thrust"]

    # ---------------------------------------------------------------- hooks
    def _phase_choice(self) -> str:
        pc = self.vp.params.get("phase_controllers") or {}
        mode = "LANDING" if self.mode == "EMERGENCY_LANDING" else self.mode
        return pc.get(mode, "auto")

    def shape_raw(self, raw: dict, cstate) -> dict:
        """Virtual-channel commands after the controllers: displacement flaps."""
        if self.mode == "DISPLACEMENT" and self._phase_choice() == "fixed":
            # no stability system: every surface held at its displacement angle
            return {"delta_main": self.c["disp_flap_deg"], "delta_elev": 0.0, "delta_roll": 0.0}
        if self.mode == "DISPLACEMENT":
            # no lift wanted from the main flaps; the elevator and differential flap keep damping pitch and roll
            return {"delta_main": self.c["disp_flap_deg"], "delta_elev": raw["delta_elev"], "delta_roll": raw["delta_roll"]}
        return raw

    def mix(self, target: dict) -> dict:
        """Physical drive targets: with a failed flap the healthy one carries the whole roll command about its
        live encoder angle (handles both a fixed jam and a free-trailing limp flap)."""
        if self.jam is None:
            return target
        side, a0 = self.jam
        a = float(np.degrees(self.vp.drives[side].delta)) if side in self.vp.drives else a0
        roll = 0.5 * (target["right"] - target["left"])
        t = dict(target)
        if side == "left":
            t["left"], t["right"] = a, a + 2.0 * roll
        else:
            t["right"], t["left"] = a, a - 2.0 * roll
        return t

    def landed_reset(self):
        """Hand the controllers back to takeoff state once the hull is down (the LQR handover re-latches)."""
        vp = self.vp
        if vp.lqr is not None:
            vp.lqr.reset()
        vp.controller.reset()
