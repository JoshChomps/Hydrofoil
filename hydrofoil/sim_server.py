"""
Web launcher, boat editor, and autotuner dashboard for the sim: a page at http://127.0.0.1:8765 that starts / stops
the MuJoCo viewer (`python -m hydrofoil.viewer`) with the chosen options.

Run:  python -m hydrofoil.sim_server [--port 8765]
"""

from __future__ import annotations

import argparse
import html
import json
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

ASSETS = Path(__file__).resolve().parent / "assets" / "launcher"
PAGE = ASSETS / "page.html"
FONTS = ASSETS / "fonts"

STATE_DIR = ROOT / "local"                     # gitignored runtime directory for logs & saved overrides
LOG = STATE_DIR / "viewer.log"
OVERRIDES = STATE_DIR / "overrides.json"       # boat-editor edits and tuned gains, per preset: {"manned": {...}, "model": {...}}
TUNE_PROGRESS = STATE_DIR / "tune_progress.json"

tune_proc: subprocess.Popen | None = None
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
proc: subprocess.Popen | None = None
last_cmd: list[str] = []

FIELDS = {   # name: (choices or None for free numeric, default)
    "preset": (("model", "manned"), "manned"),
    "supervisor": (("off", "on"), "on"),
    "controller": (("lqr", "pid"), "lqr"),
    "sensors": (("ideal", "noisy"), "ideal"),
    "waves": (("off", "on"), "off"),
    "wave_dir": (None, "0"),
    "speed": (None, "1.0"),
}

LABELS = {
    "preset": "Boat",
    "supervisor": "Supervisor",
    "controller": "Flight controller",
    "sensors": "Sensors",
    "waves": "Waves",
    "wave_dir": "Wave direction (deg)",
    "speed": "Sim speed",
}

API_PORT = 8766
last_form: dict = {}


def _ensure_state_dir() -> Path:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    return STATE_DIR


def running() -> bool:
    return proc is not None and proc.poll() is None


def start(form: dict) -> None:
    global proc, last_cmd
    stop()
    _ensure_state_dir()
    v = {k: form.get(k, [d])[0] for k, (_c, d) in FIELDS.items()}
    for k, (choices, d) in FIELDS.items():
        if choices and v[k] not in choices:
            v[k] = d
        if not choices:
            try:
                v[k] = str(float(v[k]))
            except ValueError:
                v[k] = d
    cmd = [sys.executable, "-m", "hydrofoil.viewer", "--preset", v["preset"], "--controller", v["controller"],
           "--sensors", v["sensors"], "--speed", v["speed"], "--wave-dir", v["wave_dir"]]
    if v["supervisor"] == "on":
        cmd.append("--supervisor")
    if v["waves"] == "on":
        cmd.append("--waves")
    cmd += ["--duration", "36000", "--api-port", str(API_PORT)]
    ov = load_overrides().get(v["preset"])
    if ov:
        f = STATE_DIR / f"overrides_{v['preset']}.json"
        f.write_text(json.dumps(ov))
        cmd += ["--overrides", str(f)]
    last_cmd = cmd
    last_form.update(v)
    proc = subprocess.Popen(cmd, cwd=ROOT, stdout=open(LOG, "w"), stderr=subprocess.STDOUT, creationflags=NO_WINDOW)


def stop() -> None:
    global proc
    if running():
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
    proc = None


def load_overrides() -> dict:
    try:
        return json.loads(OVERRIDES.read_text())
    except (OSError, ValueError):
        return {}


def save_overrides(d: dict):
    _ensure_state_dir()
    OVERRIDES.write_text(json.dumps(d, indent=1))


EDIT_KEYS = ["main_chord", "main_span", "main_incidence_deg", "main_flap_frac", "elev_chord", "elev_span", "elev_pos_x",
             "flying_height", "target_speed", "max_thrust"]


def design_info(preset: str) -> dict:
    """Preset values for the editor fields, and what is saved for this preset."""
    from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
    p = MANNED_PARAMS if preset == "manned" else DEFAULT_PARAMS
    base = {k: p[k] for k in EDIT_KEYS}
    base["strut_length"] = p["hull_offset"] + p["main_pos_z"]
    geom = {k: p[k] for k in ("hull_length", "hull_beam", "hull_x", "main_lateral_offset")}
    return {"preset": preset, "base": base, "payloads": p.get("default_payloads", []), "geom": geom,
            "saved": load_overrides().get(preset, {})}


def design_preview(preset: str, overrides: dict) -> dict:
    from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
    from hydrofoil.design import apply_design, derived
    p = dict(MANNED_PARAMS if preset == "manned" else DEFAULT_PARAMS)
    design_only = {k: v for k, v in overrides.items() if k not in ("lqr_weights",) and not k.startswith(("kp_", "ki_", "kd_"))}
    e = apply_design(p, design_only)
    return {"base": derived(p), "edited": derived(e),
            "edited_geom": {k: e[k] for k in ("main_chord", "main_span", "main_lateral_offset", "elev_chord", "elev_span",
                                               "elev_pos_x", "hull_length", "hull_beam", "hull_x")}}


def tune_start(preset: str, target: str):
    global tune_proc
    if tune_proc is not None and tune_proc.poll() is None:
        return False
    _ensure_state_dir()
    ov = load_overrides().get(preset)
    f = STATE_DIR / f"overrides_{preset}.json"
    f.write_text(json.dumps(ov or {}))
    TUNE_PROGRESS.write_text(json.dumps({"status": "starting", "preset": preset, "target": target}))
    cmd = [sys.executable, "-m", "hydrofoil.tuning.autotune", "--preset", preset, "--target", target,
           "--overrides", str(f), "--progress", str(TUNE_PROGRESS)]
    tune_proc = subprocess.Popen(cmd, cwd=ROOT, stdout=open(STATE_DIR / "tune.log", "w"), stderr=subprocess.STDOUT,
                                 creationflags=NO_WINDOW)
    return True


def tune_status() -> dict:
    try:
        d = json.loads(TUNE_PROGRESS.read_text())
    except (OSError, ValueError):
        return {"status": "idle"}
    if tune_proc is not None and tune_proc.poll() not in (None, 0) and d.get("status") != "done":
        d["status"] = "failed"
        log_path = STATE_DIR / "tune.log"
        d["log"] = log_path.read_text(errors="replace")[-1500:] if log_path.exists() else ""
    return d


def tune_apply() -> dict:
    d = tune_status()
    if not d.get("best"):
        return {"ok": False}
    allov = load_overrides()
    ov = allov.setdefault(d["preset"], {})
    vals = d["best"]["values"]
    if "lqr_weights" in vals:
        ov["lqr_weights"] = vals["lqr_weights"]
    else:
        ov.update(vals)
    save_overrides(allov)
    return {"ok": True, "applied": vals}


def page() -> str:
    def select(name):
        choices, d = FIELDS[name]
        cur = last_form.get(name, d)
        opts = "".join(f'<option{" selected" if c == cur else ""}>{c}</option>' for c in choices)
        return f'<label>{LABELS[name]}<select name="{name}">{opts}</select></label>'

    def number(name):
        return (f'<label>{LABELS[name]}<input type="text" name="{name}" '
                f'value="{html.escape(last_form.get(name, FIELDS[name][1]))}" inputmode="decimal"></label>')

    status = "running" if running() else ("stopped" if proc is None else f"exited ({proc.returncode})")
    log = html.escape(LOG.read_text(errors="replace")[-3000:]) if LOG.exists() else ""
    fields = "".join(select(n) if FIELDS[n][0] else number(n) for n in FIELDS)
    return (PAGE.read_text(encoding="utf-8").replace("__STATUS__", status).replace("__FIELDS__", fields)
            .replace("__LOG__", log or "no output yet").replace("__CMD__", html.escape(" ".join(last_cmd)))
            .replace("__APIPORT__", str(API_PORT)))


class Handler(BaseHTTPRequestHandler):
    def _send(self, body: str, code: int = 200):
        b = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _json(self, obj, code: int = 200):
        b = json.dumps(obj, default=float).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self._send(page())
        if self.path.startswith("/api/design"):
            preset = parse_qs(self.path.partition("?")[2]).get("preset", [last_form.get("preset", "manned")])[0]
            return self._json(design_info(preset))
        if self.path.startswith("/api/tune/status"):
            return self._json(tune_status())
        name = self.path.split("?")[0].removeprefix("/fonts/")
        f = FONTS / name
        if self.path.startswith("/fonts/") and "/" not in name and f.is_file():   # bundled fonts, no internet needed
            b = f.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", {".css": "text/css", ".woff2": "font/woff2"}.get(f.suffix, "text/plain"))
            self.send_header("Cache-Control", "max-age=86400")
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)
            return
        self._send("not found", 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(n).decode()
        if self.path.startswith("/api/"):
            try:
                body = json.loads(raw or "{}")
            except ValueError:
                return self._json({"error": "bad json"}, 400)
            preset = body.get("preset") or last_form.get("preset", "manned")
            try:
                if self.path == "/api/design/preview":
                    return self._json(design_preview(preset, body.get("overrides", {})))
                if self.path == "/api/design/save":
                    allov = load_overrides()
                    keep = {k: v for k, v in allov.get(preset, {}).items()
                            if k == "lqr_weights" or k.startswith(("kp_", "ki_", "kd_"))}      # tuned gains survive
                    allov[preset] = {**keep, **body.get("overrides", {})}
                    if body.get("reset"):
                        allov[preset] = keep
                    save_overrides(allov)
                    return self._json({"ok": True})
                if self.path == "/api/relaunch":
                    form = {k: [v] for k, v in last_form.items()} or {"preset": [preset]}
                    form["preset"] = [preset]
                    start(form)
                    return self._json({"ok": True})
                if self.path == "/api/tune/start":
                    return self._json({"ok": tune_start(preset, body.get("target", "pid"))})
                if self.path == "/api/tune/apply":
                    return self._json(tune_apply())
                if self.path == "/api/tune/clear":
                    allov = load_overrides()
                    allov[preset] = {k: v for k, v in allov.get(preset, {}).items()
                                     if not (k == "lqr_weights" or k.startswith(("kp_", "ki_", "kd_")))}
                    save_overrides(allov)
                    return self._json({"ok": True})
            except Exception as e:                                  # report, do not kill the launcher
                return self._json({"error": f"{type(e).__name__}: {e}"}, 500)
            return self._json({"error": "not found"}, 404)
        form = parse_qs(raw)
        if self.path == "/start":
            start(form)
        elif self.path == "/stop":
            stop()
        self.send_response(303)
        self.send_header("Location", "/")
        self.end_headers()

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true", help="do not open the dashboard in the default browser")
    a = ap.parse_args()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    print(f"Hydrofoil sim launcher on http://127.0.0.1:{a.port}", flush=True)
    try:
        if not a.no_browser:
            import webbrowser
            webbrowser.open(f"http://127.0.0.1:{a.port}")
        srv.serve_forever()
    finally:
        stop()
        if tune_proc is not None and tune_proc.poll() is None:
            tune_proc.terminate()
