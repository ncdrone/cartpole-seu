# Cart-pole soft-fault controller: build, run, reproduce

Three ways to run the same code. The flight core (`fsw/`) is identical in all three; only the toolchain around it changes.

| Path | Where | What it proves | Command |
|---|---|---|---|
| **Metal** | Mac, Raspberry Pi 5, Jetson Orin | Runs natively on a laptop and on an OBC-class board (validated on all three) | `bash scripts/build_demo.sh` |
| **Metal + LAN viewer** | Pi or Jetson on the bench | Live viewer from any browser on the LAN | `bash scripts/build_demo.sh --serve` then open the printed URL |
| **Docker** | Anywhere Docker runs | A pinned software constraint: same image on Mac and Jetson gives the same result hash | `docker build -t cartpole . && docker run --rm cartpole` |

## Metal (Mac, Pi, Jetson)

```
bash scripts/build_demo.sh                  # everything: detect -> deps (offers installs, y/N) -> build -> verify -> stamp
bash scripts/build_demo.sh --force-default  # unattended: every y/N is yes (fresh Pi or Jetson)
bash scripts/build_demo.sh --no-install     # never install; print the commands and stop
bash scripts/build_demo.sh --visual         # ...then open the matplotlib window
bash scripts/build_demo.sh --demo           # ...then the right visual for this host: Mac window, Pi/Jetson LAN viewer
bash scripts/build_demo.sh --serve          # ...then start the LAN viewer on :8080
bash scripts/build_demo.sh --docker         # ...also build the pinned image and verify inside it
bash scripts/build_demo.sh --explain        # what it checks and why
bash scripts/run_tests.sh [--quick] [--docker]
bash scripts/clean_up.sh [--docker] [--remote] [--all] [--force-default] [--dry-run]   # remove everything generated
```

The default build is **pure C++** and emits CSV; nothing visual is required. `--visual`, `--demo` and `--serve` are additive and are the only paths that need Python.

**Do not run the script with sudo.** It builds as your user and only prefixes `sudo` on the apt-get lines, so sudo prompts for your password once when a package is actually installed (on a stock Pi OS user, not at all). Run as root anyway and it warns, drops the prefixes and continues, leaving a root-owned build tree. Homebrew itself and Docker are never installed by the script; it prints the links.

What it checks, in order (`--explain` prints the same):

1. **detect** host, arch, board (Pi, Jetson from the device tree), libc, and derives an *environment class* (`arm64-darwin`, `aarch64-linux`).
2. **deps**: compiles and runs a three-line file rather than trusting `command -v` (a clean Mac has a `clang++` shim that only prompts for Command Line Tools); `cmake`; `python3` and `matplotlib` only when a flag needs them; Docker only with `--docker`. Missing items are offered one at a time, or all at once with `--force-default`.
3. **build** Release with `-std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti`, warnings as errors.
4. **verify**: unit tests, a 1 s demo with the CSV schema checked, and the golden comparison. Same environment class: bit-exact or fail. No golden yet: per-column max delta against the other classes.
5. **stamp** `fsw/build/build_info.json` (host, arch, envclass, compiler, flags, libc, git SHA, time, container flag).

`run_tests.sh` then runs: unit tests, demo schema, golden bit-exact, injector sanity (a force-limit sign flip must drop the pole on the unprotected baseline and a mantissa flip must be masked), the repro hash against its golden, and the back-test scenarios. Record a golden for a new environment once with `bash scripts/make_golden.sh`.

## Docker (the pinned constraint)

```
bash bench/pin_image.sh                         # once: pin debian:bookworm-slim by digest
docker build -t cartpole .                      # build_demo + verify run at image build time
docker run --rm cartpole                        # unit tests + envclass + repro hash
docker run --rm cartpole tests                  # full run_tests.sh inside the container
docker run --rm -p 8080:8080 cartpole serve     # LAN viewer from the container
docker run --rm cartpole backtest sim/scenarios/grid.json
docker run --rm cartpole demo --theta0 0.6 --x0 -0.5 --seconds 8 > run.csv
docker build --platform linux/amd64 -t cartpole:amd64 .   # x86 variant, on an Apple Silicon Mac
```

Docker Desktop on an Apple Silicon Mac runs arm64 Linux containers, the same architecture, compiler family and libm as a Pi or Jetson. So Docker-on-Mac and Docker-on-Jetson from the same image are the *same software constraint* and must produce the same result hash.

## What "reproducible" means here

Three things determine the numbers: the fault/start list, floating-point codegen, and the math library.

- **Start and fault lists** come from an in-repo `splitmix64` PRNG (`sim/backtest.py`), never `std::` distributions, so they are identical on every platform including Mac native.
- **Codegen** is pinned by `-ffp-contract=off` and a compile-time ban on fast-math (`fsw/include/fsw/FswTypes.hpp`). Without contraction, IEEE semantics fix the arithmetic for a given architecture.
- **libm** (`sin`, `cos` in the plant) is the residual: it differs between Apple's libm and glibc, and can differ by an ULP between glibc versions. Pinning the container image pins libm.

| Comparison | Start list | Trajectories | Check |
|---|---|---|---|
| Same image, Docker-on-Mac vs Docker-on-Jetson (arm64) | identical | bit-exact | same repro hash |
| Docker vs metal on the same Jetson or Pi | identical | bit-exact (measured: glibc 2.35, 2.36 and 2.41 agree, so one `aarch64-linux` golden serves all three) | same repro hash |
| Mac metal vs any Linux | identical | ULP-level (measured: 1 case of 64 differs in one integrator digit at tick 477) | verify's delta line |

**Measured on 2026-09-30** with the same source and flags (protected build: swing-up, on-board LQR, encoded store, FDIR):

| Environment | envclass | repro hash (first 16) | demo-1s golden |
|---|---|---|---|
| Mac, Apple clang 17, metal | `arm64-darwin` | `f741d78219d438ca` | identical to Linux at printed precision |
| Raspberry Pi 5, Pi OS (glibc 2.41), g++, metal | `aarch64-linux` | `6b7e831fb898b98e` | bit-exact |
| Jetson Orin Nano, L4T (glibc 2.35), g++, metal | `aarch64-linux` | `6b7e831fb898b98e` | bit-exact |
| Docker `debian:bookworm-slim` (pinned digest, glibc 2.36), g++ | `aarch64-linux` | `6b7e831fb898b98e` | bit-exact |

So on aarch64 Linux the result is bit-exact whether it runs in the container or on the metal of either board. The Mac differs from Linux in exactly one of 64 probe cases, in the last printed digit of the integrator, from libm rounding accumulated over 477 ticks; outcomes are identical (45 of 64 succeed everywhere).

The probe is `sim/scenarios/repro.json`: 64 seeded random non-moving starts. `python3 sim/backtest.py sim/scenarios/repro.json --hash` prints a sha256 over every case and its raw CSV and compares against `sim/golden/repro.<envclass>.sha256`. Same constraint, same hash.

## Sim tools

```
python3 sim/render.py --theta0 0.6 --x0 -0.5            # matplotlib window: cart, pole, traces, force terms
python3 sim/backtest.py --all                            # every scenario, stats after each
python3 sim/backtest.py sim/scenarios/grid.json --show   # one scenario + basin map window
python3 sim/backtest.py sim/scenarios/rate.json --set dt=0.005,0.02 --set seconds=6
python3 bench/serve.py                                   # bench: LAN browser viewer, stdlib only, http://<host>:8080
```

Scenario files are JSON axes (`theta0_deg`, `x0_m`, `dt`, `substeps`, `cart_mass`, `pole_mass`, `half_len`); lists or `{start, stop, step}` ranges sweep the cartesian product, `{min, max}` with `samples` and `seed` draws random cases. Stats after each run: outcome counts, success rate, settle time (mean, median, p90), peak |x| and |u|, saturated ticks, per-axis breakdown, basin edges, a replay command for the mildest failure, CSV and map paths.

## Controller gains are solved on board

`fsw/src/Lqr.cpp` linearizes the cart-pole about upright, discretizes it with an exact zero-order hold, and iterates the discrete Riccati equation at init (bounded loop, no heap, 4×4 arrays). The gain that results is the optimization in the binary: Q = diag(1, 1, 10, 1), R = 0.1 are the design parameters, K is derived. `fsw/tools/compute_lqr_gains.py` solves the same problem with scipy and the unit test requires agreement to 1e-4 (measured: 1e-7, one float32 ULP on three gains). `--gains table` uses the offline table instead, which is also the fallback if the solve fails to converge.

## Fault injection (sim side, baseline)

```
fsw/build/cartpole_baseline --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100 > flip.csv   # sign flip on the pole gain, unprotected: falls at 2.1 s
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100 > ok.csv     # same flip, protected: corrected at tick 100
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:30+31 --flip-tick 100           # double flip: DED -> verified golden reload
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip-local k2:31 --flip-tick 100        # compute window: dual-execution mismatch
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --stall 40 --flip-tick 100                # controller stalled 0.4 s: actuator fallback, reload
python3 sim/render.py --csv flip.csv
```

Params fields `k0 k1 k2 k3 ki ilim alpha swke swamax swamin swkx swkv sweref swenter swrate swexit mM mm ml mg`, bits 0–31, `+` for multi-bit; also `--flip-state`, `--flip-check`, `--flip-crc`, `--flip-input`. The injector lives in the sim-side `main.cpp`; the flight core has no knowledge of it and the flight object code is identical with or without it. The CSV carries `det` (0 none, 1 SEC, 2 DED, 3 CRC, 4 RANGE, 5 MISMATCH, 6 NONFINITE, 7 STALL, 8 GOLDEN, 9 INPUT), `fdir` (0 NOMINAL, 1 RECOVERING, 2 DEGRADED, 3 SAFE), `sec` and `reload` counters. `docs/how-it-works.html` walks through eight real flips and what each protection layer does with them; `docs/system-diagram.html` is the functional diagram. Both are single files and are also served by the LAN viewer at `/docs/<name>.html`.

## Layout

```
fsw/        flight core: Controller, Lqr, Protect (SECDED + CRC + store), Runtime (decode → dual step → guard → FDIR), Plant, harness main (+ injector), tests   (C++14, no deps, no heap, no exceptions)
sim/        render.py (window viewer), backtest.py, campaign.py (verification campaign), scenarios/ (grid, basin, mistune, repro), golden/
bench/      bench tooling, not part of the core: serve.py + viewer.html (LAN browser viewer, used by --serve/--demo), pin_image.sh, rate.json
scripts/    build_demo.sh, run_tests.sh, clean_up.sh, make_golden.sh, docker-entry.sh
docs/       SPEC-01, SPEC-02, GLOSSARY.md, how-it-works.html, system-diagram.html
research_summary.html   synthesis of five independent design reviews (the reviews themselves are internal, not shipped)
Dockerfile  pinned software constraint
```

## Status

The controller is energy-shaping swing-up with a hysteresis handoff to a discrete LQR whose gains are solved on board. Measured with `sim/backtest.py` (success = |θ| < 5° and |x| < 0.1 m held for the last 2 s of a 10 s run, |x| ≤ 2.4 m throughout, no fault):

| Scenario | Runs | Success | Notes |
|---|---|---|---|
| grid: θ₀ −180°…180° every 2° × x₀ {−1, −0.5, 0, 0.5, 1} m | 905 | **100%** | includes hanging-down; worst cart travel 2.15 m of the 2.4 m rail; settle median 5.3 s, max 7.2 s |
| basin: θ₀ 0…180° every 1°, x₀ = 0 | 181 | **100%** | 59 starts pass through horizontal on the way up |
| mistune: ±10% cart mass, pole mass, half-length (27 corners) × θ₀ {5, 15, 25, 35}° | 108 | **100%** | the swing-up handoff, not the LQR, was the sensitive part: a 2.5 rad/s entry limit stalled at +10% pole length; 4.0 rad/s catches every corner |
| hanging-down, nominal | 1 | balance at 2.4 s | 1.19 m of rail used |

Before swing-up the same grid was 23%.

## The protected build, measured

Two binaries from one source: `cartpole_baseline` (no protection) and `cartpole_demo` (the ladder: Hsiao SECDED(39,32) on every word of `Params` and `State` in a volatile encoded store → CRC-32C over the corrected plaintext → range table → dual execution of `step()` → output guard from constants → 3-state FDIR with (v, ~v) state word and a verified golden reload). `sim/campaign.py` pushes the same faults through both, from θ₀ = 0.05 rad with the fault at tick 100 (1,640 runs, a few seconds):

| Injection set | Build | Runs | Masked | Corrected | Recovered | SDC |
|---|---|---|---|---|---|---|
| A. single bit, every Params word × 32 bits | baseline | 640 | 598 | | | **42** |
| | protected | 640 | | **640** | | 0 |
| B. two bits in one word | baseline | 60 | 54 | | | 6 |
| | protected | 60 | | | **60** (DED → reload) | 0 |
| C. SECDED check bits | protected | 140 | | 140 | | 0 |
| D. the CRC word itself | protected | 8 | | | 8 | 0 |
| E. compute window (one lane's decoded copy) | protected | 40 | 27 | | 13 (dual-exec mismatch) | 0 |
| F. one sensor sample | both | 9 | 9 | | | 0 |
| G. State words | protected | 12 | | 12 | | 0 |
| H. controller stalled 5–80 ticks | both | 5 | | | 5 (actuator fallback + reload) | 0 |

Reading it: on the unprotected build 42 of 640 single-bit flips (6.6%) are silent data corruption, all sign or exponent bits. The protected build corrects every one of them in place and never applies a double-flipped word. The 27 masked compute-window flips hit swing-up parameters while the controller was balancing, so both lanes agreed; they are masked by use, not by luck. A single corrupted sensor sample is absorbed by feedback in both builds, which is the "feedback tolerates transient faults" point from the research. `scripts/run_tests.sh` runs this campaign and fails if the protected build ever shows an SDC in sets A–E.
