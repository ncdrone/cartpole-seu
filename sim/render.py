#!/usr/bin/env python3
"""Cart-pole replay renderer. Reads the CSV emitted by fsw/build/cartpole_demo
(or any CSV with the same columns) and animates it in simulated time.

  python3 sim/render.py --theta0 0.2 --x0 0.5 --seconds 5        # runs the demo, then plays it
  python3 sim/render.py --csv run.csv                              # replays a saved trace
  python3 sim/render.py ... --speed 0.5                            # half speed

Optional column `event` (string) flashes the pole when non-empty; the C++ side
does not emit it yet. Nothing here touches the flight code.
"""
import argparse, csv, io, os, subprocess, sys
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import animation, patches

HERE = os.path.dirname(os.path.abspath(__file__))
DEMO = os.path.join(HERE, "..", "fsw", "build", "cartpole_demo")

# NOIR ident tokens
BG, PANEL, LINE, INK, MUTE, DIM, ORANGE = "#080808", "#0d0d10", "#242429", "#f2f2f2", "#9a9aa2", "#55555c", "#ff6b35"
TRACK = 2.4          # rail half-length [m]
TERMS = ["u_x", "u_xd", "u_th", "u_thd", "u_i"]
TERM_LABEL = {"u_x": "x · Kx  (P on cart)", "u_xd": "ẋ · Kẋ  (D on cart)", "u_th": "θ · Kθ  (P on pole)",
              "u_thd": "θ̇ · Kθ̇  (D on pole)", "u_i": "∫x · Ki  (I on cart)"}
TERM_COLOR = {"u_x": "#9a9aa2", "u_xd": "#55555c", "u_th": "#ff6b35", "u_thd": "#c8d4e0", "u_i": "#33ff33"}
POLE_LEN = 1.0       # drawn pole length = 2 * half_len [m]
DT = 0.01


def load_rows(args):
    if args.csv:
        text = open(args.csv).read()
    else:
        if not os.path.exists(DEMO):
            sys.exit(f"demo binary not found: {DEMO}\nbuild it: cmake -B fsw/build -S fsw && cmake --build fsw/build")
        cmd = [DEMO, "--theta0", str(args.theta0), "--x0", str(args.x0), "--seconds", str(args.seconds)]
        text = subprocess.run(cmd, check=True, capture_output=True, text=True).stdout
    rows = list(csv.DictReader(io.StringIO(text)))
    if not rows:
        sys.exit("empty trace")
    for r in rows:
        for k in ("t", "x", "xdot", "theta", "thetadot", "u"):
            r[k] = float(r[k])
        r["fault"] = int(r.get("fault", 0))
        r["event"] = r.get("event", "") or ""
        for k in TERMS:                       # optional per-term columns (older CSVs lack them)
            r[k] = float(r[k]) if r.get(k) not in (None, "") else 0.0
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv")
    ap.add_argument("--theta0", type=float, default=0.2)
    ap.add_argument("--x0", type=float, default=0.5)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    ap.add_argument("--save", help="write an .mp4/.gif instead of opening a window")
    args = ap.parse_args()
    rows = load_rows(args)
    n = len(rows)
    t = [r["t"] for r in rows]

    plt.rcParams.update({
        "figure.facecolor": BG, "axes.facecolor": PANEL, "axes.edgecolor": LINE,
        "axes.labelcolor": MUTE, "xtick.color": DIM, "ytick.color": DIM,
        "text.color": INK, "font.family": "Helvetica", "axes.grid": True,
        "grid.color": LINE, "grid.linewidth": 0.6, "axes.spines.top": False, "axes.spines.right": False,
    })
    fig = plt.figure(figsize=(11, 8.6), num="cart-pole · fsw baseline")
    gs = fig.add_gridspec(4, 3, height_ratios=[2.4, 1, 1, 1.2], hspace=0.6, wspace=0.35)
    ax = fig.add_subplot(gs[0, :])
    ax_th = fig.add_subplot(gs[1, :])
    ax_x = fig.add_subplot(gs[2, :2])
    ax_u = fig.add_subplot(gs[2, 2])
    ax_terms = fig.add_subplot(gs[3, :])

    # scene
    ax.set_xlim(-TRACK - 0.4, TRACK + 0.4); ax.set_ylim(-0.5, 1.6); ax.set_aspect("equal")
    ax.grid(False); ax.set_yticks([]); ax.set_xlabel("x [m]")
    ax.plot([-TRACK, TRACK], [0, 0], color=LINE, lw=2)
    for s in (-TRACK, TRACK):
        ax.plot([s, s], [-0.12, 0.12], color=DIM, lw=2)
    cart = patches.Rectangle((0, 0), 0.4, 0.2, facecolor=INK, edgecolor="none")
    ax.add_patch(cart)
    pole, = ax.plot([], [], color=ORANGE, lw=4, solid_capstyle="round")
    bob, = ax.plot([], [], "o", color=ORANGE, ms=8)
    force = ax.annotate("", xy=(0, 0.1), xytext=(0, 0.1), arrowprops=dict(arrowstyle="-|>", color=MUTE, lw=1.5))
    readout = ax.text(0.01, 0.95, "", transform=ax.transAxes, family="Menlo", size=10, color=MUTE, va="top")
    ev_txt = ax.text(0.99, 0.95, "", transform=ax.transAxes, family="Menlo", size=10, color=ORANGE, va="top", ha="right")
    fig.text(0.01, 0.975, "●", color=ORANGE, size=9, ha="left", va="top")
    fig.text(0.03, 0.975, "CART-POLE · SEU · LQR BASELINE · SIM TIME REPLAY", family="Menlo", size=9, color=MUTE, va="top")

    # traces
    def trace(a, key, label, ylim=None):
        a.plot(t, [r[key] for r in rows], color=DIM, lw=1)
        live, = a.plot([], [], color=INK, lw=1.4)
        cur = a.axvline(0, color=ORANGE, lw=1)
        a.set_ylabel(label); a.set_xlim(t[0], t[-1])
        if ylim: a.set_ylim(*ylim)
        return live, cur
    th_live, th_cur = trace(ax_th, "theta", "θ [rad]")
    ax_th.axhspan(-0.0873, 0.0873, color=ORANGE, alpha=0.06, lw=0)  # ±5° success band
    x_live, x_cur = trace(ax_x, "x", "x [m]")
    u_live, u_cur = trace(ax_u, "u", "u [N]", (-22, 22))
    ax_u.axhline(20, color=LINE, lw=1); ax_u.axhline(-20, color=LINE, lw=1)
    ax_x.set_xlabel("t [s]"); ax_u.set_xlabel("t [s]")

    # per-term force contributions (u = sum of terms, before clamp)
    has_terms = any(abs(r[k]) > 0 for r in rows for k in TERMS)
    term_live = {}
    for k in TERMS:
        if k == "u_i" and not any(abs(r["u_i"]) > 1e-9 for r in rows):
            lbl = TERM_LABEL[k] + "  = 0 (Ki off)"
        else:
            lbl = TERM_LABEL[k]
        ax_terms.plot(t, [r[k] for r in rows], color=TERM_COLOR[k], lw=0.8, alpha=0.35)
        term_live[k], = ax_terms.plot([], [], color=TERM_COLOR[k], lw=1.6, label=lbl)
    terms_cur = ax_terms.axvline(0, color=ORANGE, lw=1)
    ax_terms.axhline(0, color=LINE, lw=1)
    ax_terms.set_xlim(t[0], t[-1]); ax_terms.set_ylabel("force terms [N]"); ax_terms.set_xlabel("t [s]")
    ax_terms.legend(loc="upper right", frameon=False, fontsize=8, labelcolor=MUTE, ncol=5)
    if not has_terms:
        ax_terms.text(0.5, 0.5, "no per-term columns in this CSV", transform=ax_terms.transAxes, ha="center", color=DIM, family="Menlo", size=9)

    def frame(i):
        r = rows[i]
        cart.set_xy((r["x"] - 0.2, 0.0))
        import math
        px, py = r["x"], 0.2
        tx, ty = px + POLE_LEN * math.sin(r["theta"]), py + POLE_LEN * math.cos(r["theta"])
        pole.set_data([px, tx], [py, ty]); bob.set_data([tx], [ty])
        flash = bool(r["event"]) or r["fault"]
        pole.set_color("#ff3355" if r["fault"] else ORANGE)
        force.set_position((px, 0.1)); force.xy = (px + r["u"] / 40.0, 0.1)
        readout.set_text(f"tick {r['tick']:>5}  t {r['t']:5.2f}s\nθ {math.degrees(r['theta']):+6.2f}°  x {r['x']:+5.2f} m\n"
                         f"u {r['u']:+6.2f} N  fault {r['fault']}\n"
                         f"P {r['u_x'] + r['u_th']:+6.2f}  D {r['u_xd'] + r['u_thd']:+6.2f}  I {r['u_i']:+6.2f}"
                         + (f"\nmode {'BALANCE' if str(r.get('mode', '')) == '1' else 'SWING-UP'}" if r.get('mode', '') != '' else ""))
        ev_txt.set_text(r["event"] if flash else "")
        sl = slice(0, i + 1)
        th_live.set_data(t[sl], [q["theta"] for q in rows[sl]]); th_cur.set_xdata([r["t"], r["t"]])
        x_live.set_data(t[sl], [q["x"] for q in rows[sl]]); x_cur.set_xdata([r["t"], r["t"]])
        u_live.set_data(t[sl], [q["u"] for q in rows[sl]]); u_cur.set_xdata([r["t"], r["t"]])
        for k in TERMS:
            term_live[k].set_data(t[sl], [q[k] for q in rows[sl]])
        terms_cur.set_xdata([r["t"], r["t"]])
        return cart, pole, bob, force, readout, ev_txt, th_live, th_cur, x_live, x_cur, u_live, u_cur, terms_cur

    interval_ms = max(1, int(1000 * DT / args.speed))
    anim = animation.FuncAnimation(fig, frame, frames=n, interval=interval_ms, blit=False, repeat=True)
    if args.save:
        anim.save(args.save, fps=int(1 / DT * args.speed), dpi=110)
        print("wrote", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
