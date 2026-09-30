#!/usr/bin/env python3
"""Scenario-driven back-test harness for the cart-pole controller.

  python3 sim/backtest.py sim/scenarios/grid.json          # one scenario
  python3 sim/backtest.py --all                            # every sim/scenarios/*.json, stats after each
  python3 sim/backtest.py sim/scenarios/grid.json --show   # also open the basin map
  python3 sim/backtest.py sim/scenarios/rate.json --set seconds=6 --set dt=0.005,0.01,0.02

A scenario is a JSON object of AXES. Every axis is a list (or a {start,stop,step}
range); the harness runs the cartesian product and prints stats after the run,
overall and broken down by each axis that has more than one value.

  {
    "name": "grid", "seconds": 10,
    "theta0_deg": {"start": -180, "stop": 180, "step": 2},
    "x0_m": [-1, -0.5, 0, 0.5, 1],
    "dt": [0.01], "substeps": [1],
    "cart_mass": [1.0], "pole_mass": [0.1], "half_len": [0.5]
  }

Every run is the real controller binary (fsw/build/cartpole_demo). Nothing here
touches the flight code. The mildest failure prints a replay command for sim/render.py.
"""
import argparse, csv, glob, io, itertools, json, math, os, statistics, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DEMO = os.path.join(ROOT, "fsw", "build", "cartpole_demo")
OUT = os.path.join(HERE, "out")
SCEN = os.path.join(HERE, "scenarios")

AXES = ["theta0_deg", "x0_m", "dt", "substeps", "cart_mass", "pole_mass", "half_len"]
FLAG = {"theta0_deg": "--theta0", "x0_m": "--x0", "dt": "--dt", "substeps": "--substeps",
        "cart_mass": "--cart-mass", "pole_mass": "--pole-mass", "half_len": "--half-len"}
DEFAULT = {"theta0_deg": [0.0], "x0_m": [0.0], "dt": [0.01], "substeps": [1],
           "cart_mass": [1.0], "pole_mass": [0.1], "half_len": [0.5]}

# Success predicate (research/01 §9): upright AND centred, held, inside the rail, no faults.
THETA_OK = math.radians(5.0)
X_OK = 0.10
HOLD_S = 2.0
TRACK = 2.4
FELL = math.radians(90.0)
FORCE_LIMIT = 20.0
OUTCOMES = ["OK", "TRACK_EXIT", "NOT_SETTLED", "FAULT", "NAN"]


def wrap(a):
    return math.remainder(a, 2 * math.pi)


def expand(v):
    if isinstance(v, dict) and "min" in v:      # sampled range; sampled_cases() draws from it
        return [float(v["min"]), float(v["max"])]
    if isinstance(v, dict):
        n = int(round((v["stop"] - v["start"]) / v["step"]))
        return [round(v["start"] + i * v["step"], 6) for i in range(n + 1)]
    if isinstance(v, (int, float)):
        return [v]
    return list(v)


def splitmix64(seed):
    """Deterministic 64-bit PRNG, identical on every platform (no std:: distributions). The C++ injector
    will use the same algorithm so fault lists match between the harness and the flight-side hooks."""
    state = seed & 0xFFFFFFFFFFFFFFFF
    while True:
        state = (state + 0x9E3779B97F4A7C15) & 0xFFFFFFFFFFFFFFFF
        z = state
        z = ((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9) & 0xFFFFFFFFFFFFFFFF
        z = ((z ^ (z >> 27)) * 0x94D049BB133111EB) & 0xFFFFFFFFFFFFFFFF
        yield z ^ (z >> 31)


def uniform(gen, lo, hi):
    return lo + (next(gen) >> 11) * (1.0 / (1 << 53)) * (hi - lo)


def load_scenario(path, overrides):
    sc = json.load(open(path))
    sc.setdefault("name", os.path.splitext(os.path.basename(path))[0])
    sc.setdefault("seconds", 10.0)
    for k, v in overrides.items():
        sc[k] = v
    axes = {k: expand(sc.get(k, DEFAULT[k])) for k in AXES}
    return sc, axes


def sampled_cases(sc, axes):
    """If the scenario has "samples" and "seed", draw that many cases: any axis given as {"min","max"}
    is sampled uniformly; list axes are sampled by index; scalar axes are fixed."""
    gen = splitmix64(int(sc["seed"]))
    cases = []
    for _ in range(int(sc["samples"])):
        c = {}
        for k in AXES:
            spec = sc.get(k, DEFAULT[k])
            if isinstance(spec, dict) and "min" in spec:
                v = uniform(gen, float(spec["min"]), float(spec["max"]))
                c[k] = round(v, 6) if k != "substeps" else int(round(v))
            else:
                vals = axes[k]
                c[k] = vals[next(gen) % len(vals)] if len(vals) > 1 else vals[0]
        cases.append(c)
    return cases


def run_case(case, seconds):
    cmd = [DEMO, "--seconds", str(seconds)]
    for k in AXES:
        v = case[k]
        if k == "theta0_deg":
            v = math.radians(v)
        cmd += [FLAG[k], f"{v:.6g}"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return {"outcome": "FAULT", "note": r.stderr.strip()[:80], "t_settle": None,
                "max_x": float("nan"), "max_theta_deg": float("nan"), "max_u": float("nan"), "sat_frac": float("nan"), "faults": 0}
    rows = list(csv.DictReader(io.StringIO(r.stdout)))
    raw = r.stdout
    dt = case["dt"]
    th = [float(q["theta"]) for q in rows]
    x = [float(q["x"]) for q in rows]
    u = [float(q["u"]) for q in rows]
    faults = sum(int(q["fault"]) for q in rows)
    theta0 = math.radians(case["theta0_deg"])
    if any(math.isnan(v) or math.isinf(v) for v in th + x):
        outcome = "NAN"
    elif faults:
        outcome = "FAULT"
    elif max(abs(v) for v in x) > TRACK:
        outcome = "TRACK_EXIT"
    else:
        hold = max(1, int(HOLD_S / dt))
        settled = all(abs(wrap(v)) < THETA_OK for v in th[-hold:]) and all(abs(v) < X_OK for v in x[-hold:])
        outcome = "OK" if settled else "NOT_SETTLED"
    t_settle = None
    if outcome == "OK":
        ok = [abs(wrap(a)) < THETA_OK and abs(b) < X_OK for a, b in zip(th, x)]
        k = len(ok)
        while k > 0 and ok[k - 1]:
            k -= 1
        t_settle = k * dt
    return {"outcome": outcome, "t_settle": t_settle, "max_x": max(abs(v) for v in x),
            "max_theta_deg": math.degrees(max(abs(wrap(v)) for v in th)), "max_u": max(abs(v) for v in u),
            "sat_frac": sum(1 for v in u if abs(v) >= FORCE_LIMIT - 1e-6) / len(u), "faults": faults, "note": "", "raw": raw,
            "swung": int(max(abs(wrap(v)) for v in th) > FELL and abs(wrap(theta0)) <= FELL)}


def pct(vals, p):
    if not vals:
        return float("nan")
    s = sorted(vals)
    return s[min(len(s) - 1, int(round(p * (len(s) - 1))))]


def stats_block(name, cases, results, axes, seconds, wall, jobs, sampled=False):
    n = len(results)
    counts = {k: sum(1 for r in results if r["outcome"] == k) for k in OUTCOMES}
    ok = [r for r in results if r["outcome"] == "OK"]
    ts = [r["t_settle"] for r in ok]
    print(f"\n== {name}: {n} runs · {seconds:g} s each · {wall:.1f} s wall · {jobs} jobs")
    print("   outcomes   " + "  ".join(f"{k} {v}" for k, v in counts.items() if v))
    print(f"   success    {counts['OK']}/{n} = {100.0 * counts['OK'] / n:.1f}%")
    if ok:
        print(f"   t_settle   mean {statistics.mean(ts):.2f}s  median {pct(ts, .5):.2f}s  p90 {pct(ts, .9):.2f}s  max {max(ts):.2f}s")
        print(f"   max|x|     mean {statistics.mean(r['max_x'] for r in ok):.2f} m  worst {max(r['max_x'] for r in ok):.2f} m  (rail {TRACK} m)")
        print(f"   max|u|     mean {statistics.mean(r['max_u'] for r in ok):.1f} N  saturated ticks {100 * statistics.mean(r['sat_frac'] for r in ok):.1f}% of run (limit {FORCE_LIMIT:g} N)")
        print(f"   swing-up   {sum(1 for r in ok if r.get('swung'))} of the successes passed through horizontal on the way up")
    varying = [k for k in AXES if len(axes[k]) > 1]
    for k in varying:
        if len(axes[k]) > 12:
            continue  # continuous / fine axes: basin edges below or the CSV
        parts = []
        for v in axes[k]:
            sub = [r for c, r in zip(cases, results) if c[k] == v]
            s_ok = sum(1 for r in sub if r["outcome"] == "OK")
            parts.append(f"{v:g}:{100 * s_ok / len(sub):.0f}%")
        print(f"   by {k:<10} " + "  ".join(parts))
    if "theta0_deg" in varying and len(axes["theta0_deg"]) > 12 and not sampled:
        others = [k for k in varying if k != "theta0_deg"]
        combos = list(itertools.product(*[axes[k] for k in others])) if others else [()]
        shown = 0
        for combo in combos:
            sub = [(c, r) for c, r in zip(cases, results) if all(c[k] == v for k, v in zip(others, combo))]
            pos = sorted((c["theta0_deg"], r["outcome"]) for c, r in sub if c["theta0_deg"] >= 0)
            neg = sorted((-c["theta0_deg"], r["outcome"]) for c, r in sub if c["theta0_deg"] <= 0)
            if not any(d > 0 for d, _ in neg):
                neg = []
            def edge(seq):
                last = None
                for deg, o in seq:
                    if o != "OK":
                        return last if last is not None else 0.0
                    last = deg
                return last
            tag = " ".join(f"{k}={v:g}" for k, v in zip(others, combo)) or "nominal"
            ep, en = edge(pos), edge(neg)
            s = f"   basin      {tag:<32} "
            s += f"+{ep:.0f}°" if pos else ""
            s += f" / -{en:.0f}°" if neg else ""
            print(s)
            shown += 1
            if shown >= 12 and len(combos) > 12:
                print(f"   ... {len(combos) - 12} more combos in the CSV")
                break
    fails = [(c, r) for c, r in zip(cases, results) if r["outcome"] != "OK"]
    if fails:
        c, r = min(fails, key=lambda cr: abs(cr[0]["theta0_deg"]))
        flags = " ".join(f"{FLAG[k]} {(math.radians(c[k]) if k == 'theta0_deg' else c[k]):.6g}" for k in AXES if c[k] != DEFAULT[k][0] or k in ("theta0_deg", "x0_m"))
        print(f"   replay     mildest failure = {r['outcome']} at θ₀ {c['theta0_deg']:g}°, x₀ {c['x0_m']:g} m:")
        print(f"              python3 sim/render.py --seconds {seconds:g} {flags}")


def write_csv(path, cases, results):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(AXES + ["outcome", "t_settle_s", "max_x_m", "max_theta_deg", "max_u_N", "sat_frac", "faults", "note"])
        for c, r in zip(cases, results):
            w.writerow([c[k] for k in AXES] + [r["outcome"], "" if r["t_settle"] is None else f"{r['t_settle']:.2f}",
                                                f"{r['max_x']:.3f}", f"{r['max_theta_deg']:.1f}", f"{r['max_u']:.2f}",
                                                f"{r['sat_frac']:.3f}", r["faults"], r["note"]])


def draw_map(name, cases, results, axes, png, show):
    """theta0 x x0 outcome map; one panel per combination of the other varying axes (max 6)."""
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import patches
    BG, PANEL, LINE, INK, MUTE, DIM, ORANGE = "#080808", "#0d0d10", "#242429", "#f2f2f2", "#9a9aa2", "#55555c", "#ff6b35"
    COL = {"OK": "#33ff33", "TRACK_EXIT": "#4a5568", "NOT_SETTLED": "#ff3355", "FAULT": "#ff6b35", "NAN": "#ff6b35"}
    plt.rcParams.update({"figure.facecolor": BG, "axes.facecolor": PANEL, "axes.edgecolor": LINE, "axes.labelcolor": MUTE,
                         "xtick.color": DIM, "ytick.color": DIM, "text.color": INK, "font.family": "Helvetica"})
    others = [k for k in AXES if k not in ("theta0_deg", "x0_m") and len(axes[k]) > 1]
    combos = list(itertools.product(*[axes[k] for k in others]))[:6] if others else [()]
    ths, xs = axes["theta0_deg"], axes["x0_m"]
    step = (ths[1] - ths[0]) if len(ths) > 1 else 5.0
    fig, axs = plt.subplots(len(combos), 1, figsize=(11, 1.4 + len(combos) * (0.9 + 0.45 * len(xs))), squeeze=False, num=f"basin map · {name}")
    for ax, combo in zip(axs[:, 0], combos):
        sub = [(c, r) for c, r in zip(cases, results) if all(c[k] == v for k, v in zip(others, combo))]
        for c, r in sub:
            ax.add_patch(patches.Rectangle((c["theta0_deg"] - step / 2, xs.index(c["x0_m"]) - 0.4), step, 0.8, facecolor=COL[r["outcome"]], edgecolor="none"))
        ax.set_xlim(min(ths) - step / 2, max(ths) + step / 2); ax.set_ylim(-0.6, len(xs) - 0.4)
        ax.set_yticks(range(len(xs))); ax.set_yticklabels([f"{x:+.1f}" for x in xs]); ax.set_ylabel("x₀ [m]")
        ax.set_xticks(range(int(min(ths)), int(max(ths)) + 1, 30)); ax.grid(False)
        s_ok = sum(1 for _, r in sub if r["outcome"] == "OK")
        tag = " ".join(f"{k}={v:g}" for k, v in zip(others, combo)) or "nominal plant · dt 0.01"
        ax.set_title(f"{tag} · success {s_ok}/{len(sub)} ({100 * s_ok / len(sub):.0f}%)", family="Menlo", size=9, color=MUTE, loc="left")
    axs[-1, 0].set_xlabel("θ₀ [deg]  (0 = upright, ±180 = hanging down)")
    ok = sum(1 for r in results if r["outcome"] == "OK")
    fig.text(0.01, 0.985, "●", color=ORANGE, size=9, va="top")
    fig.text(0.03, 0.985, f"ORCA · CART-POLE · {name.upper()} · SUCCESS {ok}/{len(results)} ({100 * ok / len(results):.0f}%) · NON-MOVING STARTS", family="Menlo", size=9, color=MUTE, va="top")
    handles = [patches.Patch(color=COL[k], label=k) for k in OUTCOMES if any(r["outcome"] == k for r in results)]
    axs[0, 0].legend(handles=handles, loc="upper right", frameon=False, fontsize=8, labelcolor=MUTE, ncol=len(handles))
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.savefig(png, dpi=130, facecolor=BG)
    if show:
        plt.show()


def run_scenario(path, a):
    overrides = {}
    for s in a.set:
        k, v = s.split("=", 1)
        overrides[k] = float(v) if k == "seconds" else [float(x) for x in v.split(",")]
    sc, axes = load_scenario(path, overrides)
    if "samples" in sc:
        cases = sampled_cases(sc, axes)
        for k in AXES:  # axes for stats grouping = the distinct sampled values
            axes[k] = sorted({c[k] for c in cases})
    else:
        cases = [dict(zip(AXES, combo)) for combo in itertools.product(*[axes[k] for k in AXES])]
    t0 = time.time()
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        results = list(ex.map(lambda c: run_case(c, sc["seconds"]), cases))
    wall = time.time() - t0
    import hashlib
    h = hashlib.sha256()
    for c, r in zip(cases, results):
        h.update(json.dumps(c, sort_keys=True).encode()); h.update(r.get("raw", "").encode())
    digest = h.hexdigest()
    if a.quiet:
        print(digest)
        return sc["name"], len(results), sum(1 for r in results if r["outcome"] == "OK")
    stats_block(sc["name"], cases, results, axes, sc["seconds"], wall, a.jobs, sampled="samples" in sc)
    if a.hash or "samples" in sc:
        print(f"   result hash {digest}   (sha256 over every case + its raw CSV; same software constraint => same hash)")
        gold = sorted(glob.glob(os.path.join(HERE, "golden", f"{sc['name']}.*.sha256")))
        for g in gold:
            tag = os.path.basename(g)[len(sc["name"]) + 1:-7]
            same = open(g).read().strip() == digest
            print(f"   {'match ' if same else 'differs'}    golden {tag}" + ("" if same else "  (different software constraint, or drift)"))
    os.makedirs(OUT, exist_ok=True)
    csv_path = os.path.join(OUT, f"{sc['name']}.csv")
    write_csv(csv_path, cases, results)
    print(f"   csv        {os.path.relpath(csv_path, ROOT)}")
    if len(axes["theta0_deg"]) > 1 and "samples" not in sc:
        png = os.path.join(OUT, f"{sc['name']}.png")
        draw_map(sc["name"], cases, results, axes, png, a.show)
        print(f"   map        {os.path.relpath(png, ROOT)}")
    return sc["name"], len(results), sum(1 for r in results if r["outcome"] == "OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("scenario", nargs="*", help="scenario JSON file(s)")
    ap.add_argument("--all", action="store_true", help="run every sim/scenarios/*.json")
    ap.add_argument("--set", action="append", default=[], help="override an axis, e.g. --set dt=0.005,0.02 or --set seconds=6")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    ap.add_argument("--show", action="store_true", help="open maps in a window")
    ap.add_argument("--hash", action="store_true", help="print the sha256 result hash and compare with sim/golden")
    ap.add_argument("--quiet", action="store_true", help="print only the result hash (for scripts)")
    a = ap.parse_args()
    if not os.path.exists(DEMO):
        sys.exit(f"build first: cmake -B fsw/build -S fsw && cmake --build fsw/build  (missing {DEMO})")
    paths = sorted(glob.glob(os.path.join(SCEN, "*.json"))) if a.all else a.scenario
    if not paths:
        ap.error("give a scenario file or --all")
    summary = [run_scenario(p, a) for p in paths]
    if len(summary) > 1:
        print("\n== summary")
        for name, n, ok in summary:
            print(f"   {name:<12} {ok:>5}/{n:<5} {100 * ok / n:5.1f}%")


if __name__ == "__main__":
    main()
