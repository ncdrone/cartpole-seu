#!/usr/bin/env python3
"""Verification campaign (SPEC-02 §4.2-4.5): every protected word, every bit, through BOTH builds, plus double bits,
check bits, the CRC word, compute-window (local) flips, sensor flips, state flips and stalls. Prints a build x set x
outcome table and exits non-zero if the protected build shows silent data corruption in the sets it claims to cover,
or if the baseline shows none (which would mean the injector is broken).

  python3 sim/campaign.py            # full table (about 1,600 short runs, a few seconds)
  python3 sim/campaign.py --quick    # one bit class per word

Outcome vocabulary (SPEC-02 §5): masked | corrected | recovered | SDC | DUE | detected-but-failed.
"""
import argparse, csv, io, math, os, subprocess, sys, time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
BIN = {"protected": os.path.join(ROOT, "fsw", "build", "cartpole_demo"), "baseline": os.path.join(ROOT, "fsw", "build", "cartpole_baseline")}
PFIELDS = "k0 k1 k2 k3 ki ilim alpha swke swamax swamin swkx swkv sweref swenter swrate swexit mM mm ml mg".split()
SFIELDS = "integ thdf tick flags".split()
DET = {0: "none", 1: "SEC", 2: "DED", 3: "CRC", 4: "RANGE", 5: "MISMATCH", 6: "NONFINITE", 7: "STALL", 8: "GOLDEN"}
THETA0, X0, SECONDS, TICK = 0.05, 0.0, 6.0, 100


def run(build, extra):
    cmd = [BIN[build], "--theta0", str(THETA0), "--x0", str(X0), "--seconds", str(SECONDS), "--flip-tick", str(TICK)] + extra
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return {"plant": "CRASH", "det": "none", "reload": 0, "fdir": 0, "sec": 0}
    rows = list(csv.DictReader(io.StringIO(r.stdout)))
    th = [math.remainder(float(q["theta"]), 2 * math.pi) for q in rows]; x = [float(q["x"]) for q in rows]
    hold = int(2.0 / 0.01)
    settled = all(abs(a) < 0.0873 for a in th[-hold:]) and all(abs(b) < 0.1 for b in x[-hold:])
    plant = "OK" if settled and max(abs(b) for b in x) <= 2.4 else "FAIL"
    dets = [int(q["det"]) for q in rows[TICK:]]
    first = next((d for d in dets if d != 0), 0)
    strongest = max(dets) if dets else 0
    return {"plant": plant, "det": DET[first], "strongest": DET[strongest], "reload": int(rows[-1]["reload"]),
            "fdir": int(rows[-1]["fdir"]), "sec": int(rows[-1]["sec"]), "fault": max(int(q["fault"]) for q in rows)}


def classify(r):
    if r["plant"] == "CRASH":
        return "DUE"
    detected = r["det"] not in ("none",)
    corrected = r["det"] == "SEC" and r["reload"] == 0
    if r["fdir"] == 3:
        return "DUE" if r["plant"] == "FAIL" else "recovered"       # latched SAFE with the pole still up counts as recovered here
    if r["plant"] == "OK":
        if r["fdir"] == 2: return "recovered"                          # DEGRADED, pole still up
        if not detected: return "masked"
        if corrected: return "corrected"
        return "recovered"
    return "SDC" if not detected else "detected-but-failed"


def sets(quick):
    bits_all = list(range(32)); bits_q = [31, 30, 23, 0]
    bits = bits_q if quick else bits_all
    S = {}
    S["A single-bit Params"] = [(b, ["--flip", f"{f}:{bit}"]) for b in ("baseline", "protected") for f in PFIELDS for bit in bits]
    S["B double-bit Params"] = [(b, ["--flip", f"{f}:{p}+{q}"]) for b in ("baseline", "protected") for f in PFIELDS for p, q in ((30, 31), (0, 1), (15, 16))]
    S["C check bits"] = [("protected", ["--flip-check", f"{f}:{bit}"]) for f in PFIELDS for bit in (range(7) if not quick else (0, 6))]
    S["D CRC word"] = [("protected", ["--flip-crc", str(bit)]) for bit in (range(0, 32, 4) if not quick else (0, 31))]
    S["E compute window (lane 1)"] = [("protected", ["--flip-local", f"{f}:{bit}"]) for f in PFIELDS for bit in (31, 30)]
    S["F sensor sample"] = [(b, ["--flip-input", f"{fld}:{bit}"]) for b in ("baseline", "protected") for fld in ("theta", "thetadot", "x") for bit in (31, 30, 23)]
    S["G state words"] = [(b, ["--flip-state", f"{f}:{bit}"]) for b in ("baseline", "protected") for f in SFIELDS for bit in (31, 30, 0)]
    S["H stall (ticks)"] = [(b, ["--stall", str(k)]) for b in ("baseline", "protected") for k in (5, 10, 20, 40, 80)]
    return S


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--quick", action="store_true"); ap.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    a = ap.parse_args()
    for b, p in BIN.items():
        if not os.path.exists(p): sys.exit(f"missing {p}: run scripts/build_demo.sh")
    S = sets(a.quick); t0 = time.time(); results = {}
    with ThreadPoolExecutor(max_workers=a.jobs) as ex:
        for name, cases in S.items():
            results[name] = list(zip(cases, ex.map(lambda c: run(c[0], c[1]), cases)))
    cats = ["masked", "corrected", "recovered", "SDC", "DUE", "detected-but-failed"]
    print(f"\n== verification campaign: {sum(len(v) for v in S.values())} runs · {time.time() - t0:.1f} s · θ₀ {THETA0} rad, flip at tick {TICK}")
    print(f"   {'set':<28} {'build':<10} {'n':>4}  " + "  ".join(f"{c:>9}" for c in cats))
    bad = []
    for name, rows in results.items():
        for build in ("baseline", "protected"):
            sub = [r for (c, r) in rows if c[0] == build]
            if not sub: continue
            counts = {c: 0 for c in cats}
            for r in sub: counts[classify(r)] += 1
            print(f"   {name:<28} {build:<10} {len(sub):>4}  " + "  ".join(f"{counts[c]:>9}" for c in cats))
            if build == "protected" and name[0] in "ABCDE" and counts["SDC"] > 0:
                bad.append(f"protected build has {counts['SDC']} SDC in set {name}")
            if build == "baseline" and name[0] == "A" and counts["SDC"] + counts["DUE"] == 0:
                bad.append("baseline shows no failures under single-bit flips: injector broken?")
    # mechanism breakdown for the protected build
    print("\n   protected build, first detector per set:")
    for name, rows in results.items():
        sub = [r for (c, r) in rows if c[0] == "protected"]
        if not sub: continue
        mech = {}
        for r in sub: mech[r["det"]] = mech.get(r["det"], 0) + 1
        print(f"   {name:<28} " + "  ".join(f"{k}={v}" for k, v in sorted(mech.items(), key=lambda kv: -kv[1])))
    # worst offenders list (protected failures) for replay
    fails = [(c, r) for rows in results.values() for (c, r) in rows if c[0] == "protected" and classify(r) in ("SDC", "detected-but-failed", "DUE")]
    if fails:
        print(f"\n   protected failures ({len(fails)}), first 8:")
        for c, r in fails[:8]:
            print(f"     {' '.join(c[1]):<28} plant={r['plant']} det={r['det']} reload={r['reload']} fdir={r['fdir']}")
    if bad:
        print("\n   FAIL: " + "; ".join(bad)); sys.exit(1)
    print("\n   campaign thresholds met")


if __name__ == "__main__":
    main()
