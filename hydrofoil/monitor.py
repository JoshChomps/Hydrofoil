"""
Flight observer: watches a running simulation and says when it is getting bad for foiling, and why.

It is an analysis tool for the sim (it reads the true state and the wave field), not a flight function: the
supervisor (control/supervisor.py) is the part that acts, on measurable signals only. Limits come from the
supervisor's envelope where one exists, so the two agree.

Each check gives "ok", "caution" or "danger" with a plain reason:

    hull           hull clearance to the local water surface (a hull strike at speed is the main crash mode)
    breach         main-foil depth below the local surface, in chords (near 1 chord the foil loses lift; at the
                   surface it ventilates and lift collapses)
    heel / pitch   against the supervisor's attitude limits
    authority      a flap held near its +-20 deg command limit: no control margin left
    drives         drives torque- or voltage-limited for much of the last second
    speed          below the minimum foiling speed (the flap cannot trim the boat any slower)
    oscillation    height or heel swinging more over the last 3 s than over the 3 s before
    ride           vertical acceleration felt by the rider (rms over 2 s, and peaks)
    cavitation     cavitation number at the foil
    height         height error from the target

`update` returns the current alerts (worst first); `log` keeps every change of level with its time.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from hydrofoil.constants import RHO_WATER, G

LEVELS = {"ok": 0, "caution": 1, "danger": 2}


class FlightMonitor:
    def __init__(self, params: dict, rate_hz: float = 50.0):
        from hydrofoil.control.supervisor import DEFAULT_SUPERVISOR
        sup = dict(DEFAULT_SUPERVISOR, **(params.get("supervisor") if isinstance(params.get("supervisor"), dict) else {}))
        self.p = params
        manned = params["mass"] > 100.0
        self.lim = {
            "roll": sup["roll_limit"] if manned else 20.0, "pitch": sup["pitch_limit"] if manned else 10.0,
            "h_min": sup["h_min"] if manned else 0.05,
            "v_min": params.get("v_min_foil") or (sup["v_min_foil"] if manned else 0.75 * params["target_speed"]),
        }
        self.dt = 1.0 / rate_hz
        n = int(6.0 * rate_hz)
        self.buf = {k: deque(maxlen=n) for k in ("t", "dz", "roll", "vz", "lim")}
        self.state = {}
        self.log: list[dict] = []
        self.last_t = -1e9
        self.alerts: list[dict] = []
        self.air_since = None          # time the hull left the water
        self.established = False       # foilborne and settled near the target height once: the in-flight checks apply

    def reset(self):
        for b in self.buf.values():
            b.clear()
        self.state, self.log, self.alerts, self.last_t = {}, [], [], -1e9
        self.air_since, self.established = None, False

    def _set(self, out, key, level, text, value=None):
        out.append({"id": key, "level": level, "text": text, "value": value})

    def update(self, t, vp, state, cmds, wave, drives: dict | None, foilborne: bool) -> list[dict]:
        if t - self.last_t < self.dt:
            return self.alerts
        self.last_t = t
        p = self.p
        eta = state[:6]
        out = []
        vz = (-state[6] * np.sin(state[4]) + state[7] * np.cos(state[4]) * np.sin(state[3])
              + state[8] * np.cos(state[4]) * np.cos(state[3]))
        hull_off = float(vp.hull_offset) if hasattr(vp, "hull_offset") else (float(p["hull_offset"]) - float(p.get("cg_z", 0.0)))
        zref = hull_off - p["flying_height"]
        lim = any(v.get("limited") for v in (drives or {}).values())
        for k, v in (("t", t), ("dz", state[2] - zref), ("roll", np.degrees(state[3])), ("vz", vz), ("lim", lim)):
            self.buf[k].append(v)
        tt = np.asarray(self.buf["t"])
        dz_now = state[2] - zref
        if not foilborne:
            self.air_since, self.established = None, False
        else:
            self.air_since = t if self.air_since is None else self.air_since
            if not self.established and t - self.air_since > 2.0 and abs(dz_now) < 0.15 * max(p["flying_height"], 0.2):
                self.established = True
        flying = foilborne and self.established          # checks that only make sense once settled

        # hull clearance over the local wave surface, and shallowest main-foil depth (center & outer wingtip)
        env = vp.wave_sample(eta, t, wave)
        clearance = -state[2] + hull_off + env.hull_surface_z          # surface z is NED (up = negative)
        foil_hc = min(
            min(vp._submergence_over_chord(f, eta, env.foil_surface_z[n]),
                vp._foil_tip_submergence_over_chord(f, eta, env.foil_surface_z[n]))
            for n, f in (("left", vp.left_foil), ("right", vp.right_foil))
        )
        if flying:
            if clearance < 0.04:
                self._set(out, "hull", "danger", f"Hull {clearance * 100:.0f} cm off the water: a strike at speed", clearance)
            elif clearance < 0.6 * self.lim["h_min"]:
                self._set(out, "hull", "caution", f"Hull only {clearance * 100:.0f} cm above the waves", clearance)
            if foil_hc < 1.0:
                self._set(out, "breach", "danger", f"Main foil {foil_hc:.1f} chords deep: about to breach and lose lift", foil_hc)
            elif foil_hc < 2.0:
                self._set(out, "breach", "caution", f"Main foil shallow ({foil_hc:.1f} chords): losing lift to the surface", foil_hc)

        # attitude
        for key, ang, L in (("heel", np.degrees(state[3]), self.lim["roll"]), ("pitch", np.degrees(state[4]), self.lim["pitch"])):
            a = abs(ang)
            if a > L:
                self._set(out, key, "danger", f"{key.capitalize()} {ang:+.1f}° beyond the {L:.0f}° limit", ang)
            elif a > 0.6 * L:
                self._set(out, key, "caution", f"{key.capitalize()} {ang:+.1f}° (limit {L:.0f}°)", ang)

        # control authority: live physical flap against its dynamic safe envelope
        from hydrofoil.actuation.loads import drive_angles
        da = drive_angles(cmds)
        ang = {"left": da["left"], "right": da["right"], "elev": da["elev"]}
        alloc = getattr(vp, "_last_alloc_status", None) or {}
        bnds = alloc.get("bounds") or {}
        cmd_def = float(p.get("flap_cmd_max_deg", 20.0))
        def _margin_frac(key: str) -> tuple[float, float, float]:
            a_val = float(ang[key])
            lo, hi = bnds.get(key, (-cmd_def, cmd_def))
            lim_mag = abs(hi) if a_val >= 0.0 else abs(lo)
            lim_mag = max(lim_mag, 1.0)
            return abs(a_val) / lim_mag, a_val, lim_mag
        worst_k = max(ang, key=lambda k: _margin_frac(k)[0])
        frac_w, val_w, lim_w = _margin_frac(worst_k)
        label_w = "Elevator" if worst_k == "elev" else f"{worst_k.capitalize()} flap"
        if foilborne and frac_w > 0.95:
            self._set(out, "authority", "danger", f"{label_w} at {val_w:+.1f}° (limit ±{lim_w:.0f}°): no control left", val_w)
        elif foilborne and frac_w > 0.78:
            self._set(out, "authority", "caution", f"{label_w} at {val_w:+.1f}° ({frac_w:.0%} of safe limit)", val_w)

        # drives
        recent = np.asarray(self.buf["lim"])[tt > t - 1.0]
        if len(recent):
            f = recent.mean()
            if f > 0.6:
                self._set(out, "drives", "danger", f"Drives at their torque limit {f:.0%} of the last second", f)
            elif f > 0.2:
                self._set(out, "drives", "caution", f"Drives limited {f:.0%} of the last second", f)

        # speed
        u = state[6]
        if foilborne and u < self.lim["v_min"] - 0.5:
            self._set(out, "speed", "danger", f"{u:.1f} m/s: too slow to stay on the foils", u)
        elif foilborne and u < self.lim["v_min"]:
            self._set(out, "speed", "caution", f"{u:.1f} m/s, under the {self.lim['v_min']:.1f} m/s foiling minimum", u)

        # sideslip (strut ventilation / skid risk in banked turns)
        if flying and u > 2.0:
            beta_deg = float(np.degrees(np.arctan2(state[7], u)))
            te = getattr(vp, "_last_turn_envelope", None) or {}
            b_safe = float(te.get("sideslip_max_deg") or min(float(p.get("strut_vent_deg", 8.5)) - 1.5, 7.0))
            b_vent = float(p.get("strut_vent_deg", 8.5))
            if abs(beta_deg) >= b_vent - 0.5:
                self._set(out, "sideslip", "danger", f"Sideslip {beta_deg:+.1f}° (vent onset {b_vent:.1f}°): strut ventilation risk", beta_deg)
            elif abs(beta_deg) > 0.85 * b_safe:
                self._set(out, "sideslip", "caution", f"Sideslip {beta_deg:+.1f}° (safe limit {b_safe:.1f}°)", beta_deg)

        # growing oscillation (height, heel): rms of the last 3 s against the 3 s before
        if tt[-1] - tt[0] > 5.5 and flying and t - self.air_since > 8.0:
            new, old = tt > t - 3.0, (tt <= t - 3.0)

            def wobble(ts, x):                             # rms about a straight line: a steady climb is not a wobble
                return float(np.std(x - np.polyval(np.polyfit(ts, x, 1), ts))) if len(x) > 5 else 0.0
            for key, unit, small in (("dz", "m", 0.02), ("roll", "°", 1.0)):
                x = np.asarray(self.buf[key])
                a1, a0 = wobble(tt[new], x[new]), wobble(tt[old], x[old])
                if a1 > small and a1 > 1.6 * max(a0, 1e-6):
                    name = "Height" if key == "dz" else "Heel"
                    lvl = "danger" if a1 > 3 * small and a1 > 2.5 * a0 else "caution"
                    self._set(out, "oscillation", lvl, f"{name} oscillation growing ({a0:.2f} to {a1:.2f} {unit} rms)", a1)
                    break

        # ride: vertical acceleration felt aboard
        v = np.asarray(self.buf["vz"])
        if len(v) > 10 and flying and np.all(np.diff(tt) > 1e-6):
            acc = np.gradient(v, tt)
            a2 = acc[tt > t - 2.0]
            if len(a2) >= 2:
                rms, pk = a2.std() / G, np.abs(a2).max() / G
                if pk > 0.5:
                    self._set(out, "ride", "danger", f"Slam: {pk:.1f} g vertical", pk)
                elif rms > 0.1:
                    self._set(out, "ride", "caution", f"Rough ride: {rms:.2f} g rms vertical", rms)

        # cavitation
        depth = max(foil_hc * p["main_chord"], 0.0)
        sigma = (101325 + RHO_WATER * G * depth - 2340) / max(0.5 * RHO_WATER * u * u, 1e-6)
        if foilborne and sigma < 1.2:
            self._set(out, "cavitation", "danger", f"Cavitation likely (sigma {sigma:.1f})", sigma)
        elif foilborne and sigma < 2.0:
            self._set(out, "cavitation", "caution", f"Cavitation margin small (sigma {sigma:.1f})", sigma)

        # height error
        dz = dz_now
        if flying and abs(dz) > 0.3 * max(p["flying_height"], 0.2) * 2:
            self._set(out, "height", "danger", f"{abs(dz) * 100:.0f} cm {'below' if dz > 0 else 'above'} the target height", dz)
        elif flying and abs(dz) > 0.3 * max(p["flying_height"], 0.2):
            self._set(out, "height", "caution", f"{abs(dz) * 100:.0f} cm {'below' if dz > 0 else 'above'} the target height", dz)

        out.sort(key=lambda a: -LEVELS[a["level"]])
        # log changes of level per check
        seen = {a["id"]: a for a in out}
        for k in set(self.state) | set(seen):
            new_l = seen[k]["level"] if k in seen else "ok"
            if self.state.get(k, "ok") != new_l:
                self.log.append({"t": round(t, 2), "id": k, "level": new_l, "text": seen[k]["text"] if k in seen else f"{k}: back to normal"})
                self.log = self.log[-200:]
            self.state[k] = new_l
        self.alerts = out
        return out

    @property
    def level(self) -> str:
        return self.alerts[0]["level"] if self.alerts else "ok"
