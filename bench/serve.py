#!/usr/bin/env python3
"""LAN results server + browser viewer for the cart-pole demo. Standard library only.

  python3 bench/serve.py [--port 8080] [--host 0.0.0.0]   (bench tool, not part of the flight core)

Runs the real C++ binary (fsw/build/cartpole_demo) and streams CSV rows to the
browser over Server-Sent Events. Playback is paced by the browser in simulated time.
"""
import argparse, glob, json, math, os, re, socket, subprocess, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEMO = os.path.join(ROOT, "fsw", "build", "cartpole_demo")
SCEN = os.path.join(ROOT, "sim", "scenarios")
OUT = os.path.join(ROOT, "sim", "out")
BUILD_INFO = os.path.join(ROOT, "fsw", "build", "build_info.json")
COLS = ["tick", "t", "x", "xdot", "theta", "thetadot", "u", "fault",
        "u_x", "u_xd", "u_th", "u_thd", "u_i", "integ", "sat"]
INT_COLS = {"tick", "fault", "sat", "mode"}
NAME_RE = re.compile(r"^[a-z0-9_-]+$")

# name: (flag, default, lo, hi, kind); lo exclusive for kind 'pos' (must be > lo)
PARAMS = {
    "theta0": ("--theta0", 0.2, -3.2, 3.2, "f"),
    "x0": ("--x0", 0.5, -2.4, 2.4, "f"),
    "seconds": ("--seconds", 8.0, 0.0, 60.0, "pos"),
    "dt": ("--dt", 0.01, 0.0, 0.2, "pos"),
    "substeps": ("--substeps", 1, 1, 100, "i"),
    "cart_mass": ("--cart-mass", 1.0, 0.0, 1000.0, "pos"),
    "pole_mass": ("--pole-mass", 0.1, 0.0, 1000.0, "pos"),
    "half_len": ("--half-len", 0.5, 0.0, 100.0, "pos"),
}


def build_args(q):
    """Validate/clamp query params into a demo argv tail. Raises ValueError on garbage."""
    argv = []
    for name, (flag, dflt, lo, hi, kind) in PARAMS.items():
        raw = q.get(name, [None])[0]
        if raw is None or raw == "":
            v = dflt
        else:
            v = float(raw)
            if not math.isfinite(v):
                raise ValueError(f"{name} not finite")
        if kind == "pos":
            if v <= lo:
                raise ValueError(f"{name} must be > 0")
            v = min(v, hi)
        elif kind == "i":
            v = int(max(lo, min(hi, round(v))))
        else:
            v = max(lo, min(hi, v))
        argv += [flag, str(v)]
    return argv


def scenario_names():
    return sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(SCEN, "*.json")))


def parse_row(line, cols):
    """Parse one CSV data row using the header the binary printed (so new columns never break the viewer)."""
    parts = line.strip().split(",")
    if len(parts) != len(cols):
        return None
    try:
        return {c: (int(p) if c in INT_COLS else float(p)) for c, p in zip(cols, parts)}
    except ValueError:
        return None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "cartpole-serve"

    def log_message(self, fmt, *args):
        sys.stderr.write("%s %s\n" % (self.address_string(), fmt % args))

    # ---- helpers
    def send_bytes(self, code, ctype, body):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def send_json(self, obj, code=200):
        self.send_bytes(code, "application/json", json.dumps(obj).encode())

    def sse_start(self):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.send_header("Connection", "close")
        self.end_headers()
        self.close_connection = True

    def sse(self, event, data):
        self.wfile.write(("event: %s\ndata: %s\n\n" % (event, data)).encode())
        self.wfile.flush()

    def stream_proc(self, cmd, event, on_line):
        """Run cmd, forward each stdout line via on_line(line)->(event,data)|None. Returns (count, exit)."""
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT if event == "line" else subprocess.DEVNULL,
                                universal_newlines=True, bufsize=1, cwd=ROOT)
        n = 0
        try:
            for line in proc.stdout:
                ev = on_line(line)
                if ev:
                    n += 1
                    self.sse(*ev)
            return n, proc.wait()
        finally:  # client vanished or finished: never leave the child running
            if proc.poll() is None:
                proc.kill()
                proc.wait()
            proc.stdout.close()

    # ---- routes
    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        try:
            if u.path in ("/", "/index.html"):
                return self.route_index()
            if u.path == "/run":
                return self.route_run(q)
            if u.path == "/scenarios":
                return self.route_scenarios()
            if u.path == "/backtest":
                return self.route_backtest(q)
            if u.path.startswith("/map/"):
                return self.route_map(u.path[5:])
            if u.path == "/build_info":
                return self.route_build_info()
            if u.path.startswith("/docs/"):
                return self.route_docs(u.path[6:])
            self.send_bytes(404, "text/plain; charset=utf-8", b"not found\n")
        except (BrokenPipeError, ConnectionResetError):
            pass

    def route_docs(self, name):
        """Serve the single-file HTML docs (specs/*.html) on the LAN. Name only, no paths."""
        import re
        if not re.fullmatch(r"[a-z0-9_-]+\.html", name):
            return self.send_bytes(404, "text/plain; charset=utf-8", b"not found\n")
        for d in (os.path.join(ROOT, "specs"),):
            p = os.path.join(d, name)
            if os.path.exists(p):
                with open(p, "rb") as f:
                    return self.send_bytes(200, "text/html; charset=utf-8", f.read())
        self.send_bytes(404, "text/plain; charset=utf-8", b"not found\n")

    def route_index(self):
        if not os.path.exists(DEMO):
            msg = ("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width'>"
                   "<body style='background:#080808;color:#f2f2f2;font:15px ui-monospace,Menlo,monospace;padding:24px'>"
                   "<h3 style='color:#ff6b35'>cartpole_demo not built</h3>"
                   "<p>Missing <code>fsw/build/cartpole_demo</code>. Run <code>scripts/build_demo.sh</code> "
                   "on this machine, then reload.</p>")
            return self.send_bytes(503, "text/html; charset=utf-8", msg.encode())
        with open(os.path.join(HERE, "viewer.html"), "rb") as f:
            self.send_bytes(200, "text/html; charset=utf-8", f.read())

    def route_run(self, q):
        if not os.path.exists(DEMO):
            return self.send_json({"error": "cartpole_demo not built; run scripts/build_demo.sh"}, 503)
        try:
            argv = build_args(q)
        except ValueError as e:
            return self.send_json({"error": str(e)}, 400)
        self.sse_start()
        state = {"cols": None}

        def on_line(line):
            if state["cols"] is None:
                state["cols"] = line.strip().split(",")   # first line is the CSV header
                return None
            row = parse_row(line, state["cols"])
            return ("row", json.dumps(row, separators=(",", ":"))) if row else None
        n, code = self.stream_proc([DEMO] + argv, "row", on_line)
        self.sse("done", json.dumps({"rows": n, "exit": code}))

    def route_scenarios(self):
        out = []
        for n in scenario_names():
            try:
                note = json.load(open(os.path.join(SCEN, n + ".json"))).get("note", "")
            except (OSError, ValueError):
                note = ""
            out.append({"name": n, "note": note})
        self.send_json(out)

    def route_backtest(self, q):
        name = q.get("scenario", [""])[0]
        if name not in scenario_names():
            return self.send_json({"error": "unknown scenario"}, 400)
        try:
            jobs = max(1, min(64, int(q.get("jobs", [os.cpu_count() or 4])[0])))
        except ValueError:
            return self.send_json({"error": "bad jobs"}, 400)
        self.sse_start()
        cmd = [sys.executable, "-u", os.path.join(ROOT, "sim", "backtest.py"),
               os.path.join(SCEN, name + ".json"), "--jobs", str(jobs)]
        _, code = self.stream_proc(cmd, "line", lambda l: ("line", json.dumps(l.rstrip("\n"))))
        self.sse("done", json.dumps({"exit": code}))

    def route_map(self, name):
        if not name.endswith(".png") or not NAME_RE.match(name[:-4]):
            return self.send_bytes(404, "text/plain", b"not found\n")
        p = os.path.join(OUT, name)
        if not os.path.isfile(p):
            return self.send_bytes(404, "text/plain", b"no map\n")
        with open(p, "rb") as f:
            self.send_bytes(200, "image/png", f.read())

    def route_build_info(self):
        try:
            with open(BUILD_INFO, "rb") as f:
                body = f.read()
            json.loads(body)
        except (OSError, ValueError):
            body = b"{}"
        self.send_bytes(200, "application/json", body)


def lan_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))  # no packet is sent for UDP connect
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--host", default="0.0.0.0")
    a = ap.parse_args()
    ThreadingHTTPServer.daemon_threads = True
    srv = ThreadingHTTPServer((a.host, a.port), Handler)
    print("cart-pole viewer: http://%s:%d/   (local: http://127.0.0.1:%d/)" % (lan_ip(), a.port, a.port), flush=True)
    if not os.path.exists(DEMO):
        print("warning: %s missing; run scripts/build_demo.sh" % DEMO, file=sys.stderr)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
