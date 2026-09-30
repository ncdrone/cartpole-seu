# Cart-pole soft-fault controller

A C++14 cart-pole controller (energy-shaping swing-up, then an LQR whose gains are solved on board) with a software-only soft-error protection ladder: SECDED-coded parameters and state, CRC, range table, dual execution, output guard, sensor plausibility guard, and a four-state FDIR. The same flight core (`fsw/`) builds on a Mac, a Raspberry Pi 5, a Jetson Orin and in a pinned Docker image; a baseline binary without protection runs beside it so every fault result is a comparison.

## Requirements and evidence

| # | Requirement | Evidence |
|---|---|---|
| R1 | C++ controller, no dependencies | `fsw/` is 1,123 lines of C++14 (sources and headers), no heap, no exceptions, no RTTI, built `-O2 -ffp-contract=off -fno-exceptions -fno-rtti` with warnings as errors; `test_controller` |
| R2 | Optimization-based | On-board discrete Riccati solve at init (Q = diag(1, 1, 10, 1), R = 0.1); scipy cross-check agrees to 1e-7 |
| R3 | Stabilizes from any non-moving start | Grid 905/905 including hanging-down; basin 181/181; worst rail use 2.15 m of 2.4 m (settle median 5.3 s, max 7.2 s; 59 basin starts pass through horizontal); hanging-down nominal balances at 2.4 s using 1.19 m of rail; the same grid was 23% before swing-up |
| R4 | Tolerates small parameter error | Mistune 108/108 at ±10% cart mass, pole mass, half-length; the swing-up handoff was the sensitive part (2.5 rad/s entry limit stalled at +10% length, 4.0 rad/s catches every corner) |
| R5 | Software-only radiation robustness (the definition is part of the ask) | Fault model: single- and double-bit SEUs in stored parameters and state, compute-window flips, sensor-sample corruption, and stalls. Required behaviour: never apply an untrusted parameter, correct what can be proven, detect the rest, recover within the time-to-criticality. Campaign: baseline 42/640 SDC, protected 0 SDC in every set |

Stall deadline (protected build, stall at tick 100, `python3 sim/campaign.py --deadline`): the longest recoverable stall is 130 ticks (1300 ms) at 1 deg, 100 (1000 ms) at 5 deg, 80 (800 ms) at 15 deg and 60 (600 ms) at 30 deg. The 300 ms budget is met at every angle.

## What is built and what is not

Built:

- Energy-shaping swing-up with hysteresis handoff to a discrete LQR; gains solved on board, offline table as fallback (`--gains table`).
- Hsiao SECDED(39,32) on every word of `Params` and `State` in a volatile encoded store, CRC-32C over the corrected plaintext, range table, dual execution of `step()`, output guard from constants.
- Sensor-sample plausibility guard (`det` 9 INPUT): rejects non-finite values, |x| > 2.9 m, |thetadot| > 25 rad/s, or a theta step inconsistent with thetadot by more than 0.05 rad. First reject holds the last good sample without a reload, second consecutive reject reloads, third latches SAFE.
- Four-state FDIR (NOMINAL, RECOVERING, DEGRADED, SAFE) with a (v, ~v) state word and a verified golden reload. DEGRADED inhibits swing-up and halves the clamp; it is entered on a second detection within the 200-tick window. SAFE latches on the third, or on any detection while DEGRADED.
- Fault injector (sim side only), verification campaign `sim/campaign.py` (sets A-H plus the stall deadline sweep), back-test scenarios, LAN viewer.
- Tests: `test_controller`, `test_protect` (exhaustive codec), `test_fdir` (a)-(m); demo schema, golden, injector sanity and repro-hash checks in `scripts/run_tests.sh`.
- Metal builds on Mac, Pi 5 and Jetson Orin; pinned Docker image; measured reproducibility across them.

Not built (documented only):

- Command sequence plus CRC (belongs on the F´ page below).
- Sampled assessment campaign.
- Friction, actuator lag and sensor bias plant axes.
- Docker on the boards as a required path.
- Hardware EDAC assumptions (SPEC-01 section 8 lists what would change).

## How to run

Three ways to run the same code; only the toolchain around `fsw/` changes.

| Path | Where | Command |
|---|---|---|
| Metal | Mac, Raspberry Pi 5, Jetson Orin (validated on all three) | `bash scripts/build_demo.sh` |
| Metal + LAN viewer | Pi or Jetson on the bench | `bash scripts/build_demo.sh --serve` then open the printed URL |
| Docker | Anywhere Docker runs: same image on Mac and Jetson gives the same result hash | `docker build -t cartpole . && docker run --rm cartpole` |

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

The default build is pure C++ and emits CSV; `--visual`, `--demo` and `--serve` are additive and are the only paths that need Python. Do not run the script with sudo: it builds as your user and prefixes `sudo` only on the apt-get lines. Homebrew and Docker are never installed by the script; it prints the links.

`build_demo.sh` checks, in order: detect (host, arch, board, libc, environment class `arm64-darwin` or `aarch64-linux`); deps (compiles and runs a three-line file rather than trusting `command -v`); build (Release, `-std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti`, warnings as errors); verify (unit tests, a 1 s demo with the CSV schema checked, golden comparison: bit-exact within an environment class); stamp `fsw/build/build_info.json`. `run_tests.sh` adds the injector sanity check (a force-limit sign flip must drop the pole on the baseline, a mantissa flip must be masked), the repro hash, the back-test scenarios and the fault campaign, and fails if the protected build shows an SDC in sets A-E. Record a golden for a new environment once with `bash scripts/make_golden.sh`.

Docker (the pinned constraint):

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

Fault injection (baseline vs protected, from theta0 = 0.05 rad, fault at tick 100):

```
fsw/build/cartpole_baseline --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100 > flip.csv   # sign flip on the pole gain, unprotected: falls at 2.1 s
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100 > ok.csv     # same flip, protected: corrected at tick 100
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:30+31 --flip-tick 100           # double flip: DED -> verified golden reload
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip-local k2:31 --flip-tick 100        # compute window: dual-execution mismatch
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --stall 40 --flip-tick 100                # controller stalled 0.4 s: actuator fallback, reload
python3 sim/render.py --csv flip.csv
python3 sim/campaign.py --deadline                    # stall deadline sweep, protected build
```

Params fields `k0 k1 k2 k3 ki ilim alpha swke swamax swamin swkx swkv sweref swenter swrate swexit mM mm ml mg`, bits 0-31, `+` for multi-bit; also `--flip-state`, `--flip-check`, `--flip-crc`, `--flip-input`. The injector lives in the sim-side `main.cpp`; the flight object code is identical with or without it. The CSV carries `det` (0 none, 1 SEC, 2 DED, 3 CRC, 4 RANGE, 5 MISMATCH, 6 NONFINITE, 7 STALL, 8 GOLDEN, 9 INPUT), `fdir` (0 NOMINAL, 1 RECOVERING, 2 DEGRADED, 3 SAFE), `sec` and `reload` counters. `docs/how-it-works.html` walks through eight real flips; `docs/system-diagram.html` is the functional diagram; both are also served by the LAN viewer at `/docs/<name>.html`.

Sim tools:

```
```
python3 sim/render.py --theta0 0.6 --x0 -0.5            # matplotlib window: cart, pole, traces, force terms
python3 sim/backtest.py --all                            # every scenario, stats after each
python3 sim/backtest.py sim/scenarios/grid.json --show   # one scenario + basin map window
python3 sim/backtest.py sim/scenarios/rate.json --set dt=0.005,0.02 --set seconds=6
python3 bench/serve.py                                   # bench: LAN browser viewer, stdlib only, http://<host>:8080
```

Scenario files are JSON axes (`theta0_deg`, `x0_m`, `dt`, `substeps`, `cart_mass`, `pole_mass`, `half_len`); lists or `{start, stop, step}` ranges sweep the cartesian product, `{min, max}` with `samples` and `seed` draws random cases. Success = |theta| < 5 deg and |x| < 0.1 m held for the last 2 s of a 10 s run, |x| <= 2.4 m throughout, no fault. `fsw/src/Lqr.cpp` linearizes about upright, discretizes with an exact zero-order hold and iterates the Riccati equation at init (bounded loop, no heap, 4x4 arrays); `fsw/tools/compute_lqr_gains.py` solves the same problem with scipy and the unit test requires agreement to 1e-4 (measured 1e-7, one float32 ULP on three gains).

## Reproducibility

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

Campaign results (`sim/campaign.py`, 1,640 runs, from theta0 = 0.05 rad, fault at tick 100):

| Injection set | Build | Runs | Masked | Corrected | Recovered | SDC |
|---|---|---|---|---|---|---|
| A. single bit, every Params word × 32 bits | baseline | 640 | 598 | | | **42** |
| | protected | 640 | | **640** | | 0 |
| B. two bits in one word | baseline | 60 | 54 | | | 6 |
| | protected | 60 | | | **60** (DED → reload) | 0 |
| C. SECDED check bits | protected | 140 | | 140 | | 0 |
| D. the CRC word itself | protected | 8 | | | 8 | 0 |
| E. compute window (one lane's decoded copy) | protected | 40 | 27 | | 13 (dual-exec mismatch) | 0 |
| F. one sensor sample | baseline | 9 | 9 | | | 0 |
| | protected | 9 | 6 | | 3 (input guard) | 0 |
| G. State words | protected | 12 | | 12 | | 0 |
| H. controller stalled 5–80 ticks | both | 5 | | | 5 (actuator fallback + reload) | 0 |

On the unprotected build 42 of 640 single-bit flips (6.6%) are silent data corruption, all sign or exponent bits. The protected build corrects every one in place and never applies a double-flipped word. The 27 masked compute-window flips hit swing-up parameters while the controller was balancing, so both lanes agreed. Set F on the protected build is 6 masked and 3 recovered: the sensor guard fires on the exponent-bit-30 flips of theta, thetadot and x, while sign and mantissa flips are sub-tolerance and absorbed by feedback.

## Where this fails

- **A flip in a register mid-computation, without dual execution.** Memory codes never see it; the wrong value is written back and re-encoded as valid. Dual execution catches one lane; a flip in both lanes in the same tick is undetectable by construction.
- **Three or more flipped bits in one word.** SECDED miscorrects 60–69% of triple-bit patterns and reports success. The CRC and range table catch the miscorrection; a pattern that passes all three is silent. MBUs are >5% of space memory errors and we do not model adjacency.
- **The pole is already past the basin when recovery finishes.** Zero force loses the pole in 0.3–1.3 s depending on angle. Any detection-plus-reload chain longer than ~300 ms at a large angle fails even though every layer "worked".
- **Faults in what we do not protect: code, stack, OS, bus, the plant model.** A flipped instruction, a corrupted return address, a wrong sensor calibration or an unmodelled friction term are outside every layer here. Real systems answer these with hardware EDAC, image CRC at boot, bus CRCs and a hardware watchdog.
- **The controller's model sensitivity.** Swing-up reads pole energy through its own model; a ±10% length error made it stall or overshoot the catch until a minimum pump authority and a wider entry rate were added. It now passes every mistune corner, but friction, actuator lag and sensor bias are not modelled yet and the slide says so.

## F´ mapping (one paragraph)

The controller becomes a passive F´ component invoked from a 100 Hz rate group. Input port `SensorFrame` carries x, xdot, theta, thetadot, tick and validity bits; output port `ActuatorCmd` carries force, a sequence number and a CRC over both, verified by the actuator component. Telemetry channels are `sec_total`, `reload_total`, `det_total` and `fdir_state`, with one throttled event per FDIR transition. Parameters (Q, R, model constants) come from `Svc::PrmDb`, whose CRC-checked file is the golden image. The health ping is answered only while FDIR is not SAFE, so `Svc::Health` stops stroking the hardware watchdog when the controller has given up.

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

## Hours and what was cut

Hours: TBD (author fills in)

Cut, documented but not built: command sequence + CRC (F´ page), sampled assessment campaign, friction/lag/bias plant axes, Docker on the boards, hardware EDAC assumptions.
