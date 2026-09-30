# Cart-pole soft-fault controller

A C++14 cart-pole controller (energy-shaping swing-up, then an LQR whose gains are solved on board) with a software-only soft-error protection ladder: SECDED-coded parameters and state, CRC, range table, input plausibility guard, dual execution, output guard, and a four-state FDIR. The same flight core (`fsw/`) builds on metal on a Mac, a Raspberry Pi 5 and a Jetson Orin, and in a pinned Docker image (same image validated on the Mac; boards run metal). A baseline binary without protection is built from the same source, so every fault result is a comparison.

## Run it

```
bash scripts/build_demo.sh    # detect -> deps -> build -> stamp -> verify   (about 3 s)
bash scripts/run_tests.sh     # 7 gated steps, including the 1,640-run campaign (about 8 s)
```

About 11 s for both on an Apple Silicon Mac (clean export of this commit: 3.2 s + 7.4 s). Prerequisites: a C++14 compiler (Xcode Command Line Tools on a Mac), CMake 3.13 or newer (`brew install cmake`), and python3 with the standard library only (the macOS system python3 is enough) for the campaign, back-tests and repro hash. matplotlib is optional (window viewer and PNG maps). Docker instead of the metal toolchain: `docker build -t cartpole . && docker run --rm cartpole`.

## Requirements and evidence

| # | Requirement | Evidence |
|---|---|---|
| R1 | C++ controller, no dependencies | `fsw/src` + `fsw/include` are 1,177 lines of C++14 (`wc -l`, tests excluded), no heap, no exceptions, no RTTI, built `-O2 -ffp-contract=off -fno-exceptions -fno-rtti` with warnings as errors; `test_controller` |
| R2 | Optimization-based | On-board discrete Riccati solve at init (Q = diag(1, 1, 10, 1), R = 0.1); scipy cross-check agrees to 1e-7 |
| R3 | Stabilizes from any non-moving start | Grid 905/905 including hanging-down; basin 181/181; worst rail use 2.15 m of 2.4 m; settle median 4.9 s, max 6.7 s (basin 4.8 s / 6.7 s); 300 grid starts and 59 basin starts pass through horizontal on the way up; hanging-down nominal balances at 2.44 s using 1.19 m of rail; the same grid was 23% on the first baseline, before swing-up |
| R4 | Tolerates small parameter error | Mistune 108/108 at ±10% cart mass, pole mass, half-length; the swing-up handoff was the sensitive part (a 2.5 rad/s entry limit stalled at +10% length, 4.0 rad/s catches every corner) |
| R5 | Software-only radiation robustness (the definition is part of the ask) | Radiation robustness here means that under single- and double-bit SEUs in stored parameters and state, compute-window flips, sensor-sample corruption and stalls, the controller never applies an untrusted parameter, corrects what can be proven, detects the rest, and recovers within the time-to-criticality. Campaign: baseline 42/640 SDC, protected 0 SDC in every set |

Soft errors are simulated in software. The demo harness includes a software bit-flip injector for evaluation only; the flight core has no knowledge of it.

Stall deadline (protected build, `python3 sim/campaign.py --deadline`): starting from θ₀ = 1°…30°, the controller stops at tick 100 (t = 1 s); the actuator holds the last command for 10 ticks, then applies 0 N. The longest recoverable stall is 130 ticks (1,300 ms) from 1°, 100 (1,000 ms) from 5°, 80 (800 ms) from 15° and 60 (600 ms) from 30°. The 300 ms budget is met at every swept angle, and `run_tests.sh` step 7 fails if it is not.

## What is built and what is not

Built:

- Energy-shaping swing-up with a hysteresis handoff to a discrete LQR. The gains are solved on board at init; the offline scipy table is the unit-test cross-check and the range-table reference, and if the on-board solve does not converge, init refuses to start (exit 3).
- Hsiao SECDED(39,32) on every word of `Params` (20 × F32, 80 B) and `State` (4 words) in a volatile encoded store. The 32 data columns are the first 32 weight-3 7-bit values in ascending order, a `static const` table. CRC-32C over the corrected plaintext, stored with its complement; range table; dual execution of `step()`; output guard from compile-time constants (`cfg::FORCE_LIMIT_N` = 20 N, `cfg::SAFE_FORCE_N` = 0 N).
- Golden reload: the golden `Params` is rebuilt from the nominal model by the same on-board Riccati solve and accepted only if its CRC-32C matches the value pinned at init (0x9E919ECA). A mismatch goes to SAFE (`det` 8 GOLDEN).
- Input plausibility guard (`det` 9 INPUT): rejects a non-finite field, |x| > 2.9 m, |xdot| > 10 m/s (`PLAUS_MAX_XDOT`), |thetadot| > 25 rad/s (`PLAUS_MAX_RATE`), or a theta step that differs from the trapezoid of thetadot × dt by more than 0.05 rad (`PLAUS_THETA_TOL`). The first reject holds the last accepted sample without a reload (RECOVERING), the second consecutive reject reloads the golden (DEGRADED), and the third latches SAFE.
- Four-state FDIR (NOMINAL, RECOVERING, DEGRADED, SAFE). A single-bit correction is counted and never changes state. Any other detection reloads the golden (except the first input reject) and enters RECOVERING, which returns to NOMINAL after 50 clean ticks. A second detection within the 200-tick window while still RECOVERING enters DEGRADED (swing-up inhibited, clamp halved to 10 N), which returns to NOMINAL after 200 clean ticks; if RECOVERING has already returned to NOMINAL, the second detection re-enters RECOVERING. The third detection in the window, or any detection while DEGRADED, latches SAFE: 0 N, no exit. The state word and the window, count and clean-tick words are stored as (v, ~v); a mismatch reads as SAFE.
- Stall handling: an actuator freshness monitor holds a stale command for 10 ticks (`STALE_MAX_TICKS`), then applies 0 N; the stall is a detection when the controller resumes, with a golden reload.
- Fault injector (sim side only), verification campaign `sim/campaign.py` (sets A-H plus the stall deadline sweep), back-test scenarios, LAN viewer.
- Tests: `test_controller`, `test_protect` (every 1- and 2-bit pattern of 8 sample words, a 3-bit data-pattern census, CRC-32C, the store), `test_fdir` (a)-(o); `ctest` in `fsw/build` runs all three. `scripts/run_tests.sh` gates seven steps (below).
- Metal builds on Mac, Pi 5 and Jetson Orin; a pinned Docker image validated on the Mac; measured reproducibility across them.

Not built (documented only):

- Command sequence plus CRC (belongs on the F´ port, see below).
- Sampled assessment campaign.
- Friction, actuator lag and sensor bias plant axes.
- Docker on the boards as a required path (same image validated on the Mac; boards run metal).
- Hardware EDAC assumptions (SPEC-01 section 8 lists what would change).
- The flight/sim library split: `Plant.cpp` is still linked into `libfsw.a` and the harness is `fsw/src/main.cpp`.

## How to run

Two ways to run the same code; only the toolchain around `fsw/` changes.

| Path | Where | Command |
|---|---|---|
| Metal | Mac, Raspberry Pi 5, Jetson Orin (the boards' evidence is a full re-run at 703f03b on 2026-09-30 that matched the committed `aarch64-linux` goldens bit-exact) | `bash scripts/build_demo.sh` |
| Docker | Anywhere Docker runs; same image validated on the Mac, boards run metal | `docker build -t cartpole . && docker run --rm cartpole` |

```
bash scripts/build_demo.sh                  # everything: detect -> deps (offers installs, y/N) -> build -> stamp -> verify
bash scripts/build_demo.sh --force-default  # unattended: every y/N is yes (fresh Pi or Jetson)
bash scripts/build_demo.sh --no-install     # never install; print install commands for anything missing
bash scripts/build_demo.sh --visual         # ...then open the matplotlib window
bash scripts/build_demo.sh --demo           # ...then the right visual for this host: Mac window, Pi/Jetson LAN viewer
bash scripts/build_demo.sh --serve          # ...then start the LAN viewer on :8080
bash scripts/build_demo.sh --docker         # ...also build the pinned image and verify inside it
bash scripts/build_demo.sh --clean          # wipe fsw/build first
bash scripts/build_demo.sh --explain        # what it checks and why
bash scripts/run_tests.sh [--quick] [--docker]
bash scripts/clean_up.sh [--docker] [--remote] [--all] [--force-default] [--dry-run]   # remove everything generated
```

The build needs no Python and emits CSV. `run_tests.sh` uses python3 (standard library only) for the campaign, the back-tests and the repro hash, and skips those steps with a note if python3 is absent. `--visual` and `--demo` on a Mac need matplotlib; `--serve` needs python3. Do not run the script with sudo: it builds as your user and prefixes `sudo` only on the apt-get lines (run as root, it warns and drops sudo). Homebrew and Docker are never installed by the script; it prints the links. `clean_up.sh --remote` acts only on the hosts named in `REMOTE_HOSTS`; there is no default.

`build_demo.sh` runs, in order: detect (host, arch, board, libc, environment class `arm64-darwin` or `aarch64-linux`); deps (compiles and runs a three-line file rather than trusting `command -v` for the compiler; CMake by presence); build (Release, `-std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti`, warnings as errors); stamp `fsw/build/build_info.json`; verify (unit tests, a 1 s demo with the CSV schema checked, golden comparison: bit-exact within an environment class).

`run_tests.sh` gates seven steps and exits non-zero on the first failure: (1) the three unit tests; (2) demo CSV schema; (3) the bit-exact golden for this environment class (a class with no golden only warns); (4) injector sanity: `k2:31` must drop the pole on the baseline and be corrected by SEC at tick 100 on the protected build, and `k2:30+31` must give DED and a verified golden reload; (5) the full 1,640-run campaign, which fails if the protected build shows an SDC in sets A-E or the baseline shows no failure; (6) the repro hash against this environment class's golden (again, a missing golden only warns), then the basin and mistune back-tests, whose stats are printed but not gated (skipped with `--quick`); (7) the stall deadline sweep, which fails if any swept angle misses the 300 ms budget. Record a golden for a new environment once with `bash scripts/make_golden.sh`.

Docker (the pinned constraint). Docker Desktop on an Apple Silicon Mac runs arm64 Linux containers. The image (Debian bookworm pinned by tag and digest, glibc 2.36, g++ 12.2) reproduces the `aarch64-linux` golden that the Pi 5 (glibc 2.41) and the Jetson (glibc 2.35) produce on metal. Docker on the boards was not run: same image validated on the Mac; boards run metal.

```
docker build -t cartpole .                      # build_demo + verify run at image build time
docker run --rm cartpole                        # verify = run_tests.sh --quick (all 7 steps; basin/mistune back-tests skipped), about 4 s
docker run --rm cartpole tests                  # full run_tests.sh inside the container
docker run --rm cartpole campaign --deadline    # any campaign.py flags
docker run --rm cartpole backtest sim/scenarios/grid.json   # no matplotlib in the image: the map is skipped
docker run --rm cartpole demo --theta0 3.14159 --seconds 8 > run.csv
docker build --platform linux/amd64 -t cartpole:amd64 .     # x86 variant, on an Apple Silicon Mac
```

Bench (author's hardware, not needed for review): `bash scripts/build_demo.sh --serve` on a Pi or Jetson starts the LAN viewer (`bench/serve.py`) and prints the URL; `docker run --rm -p 8080:8080 cartpole serve` does the same from the container. `bash bench/pin_image.sh` re-pins the Debian base image by digest (maintainer step).

Fault injection (baseline vs protected, from theta0 = 0.05 rad, fault at tick 100):

```
fsw/build/cartpole_baseline --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100 > flip.csv   # sign flip on the pole gain, unprotected: falls at 2.1 s
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:31 --flip-tick 100 > ok.csv     # same flip, protected: corrected at tick 100
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:30+31 --flip-tick 100           # double flip: DED -> verified golden reload
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip k2:21+22+23 --flip-tick 100        # triple flip: SECDED miscorrects, the CRC catches it -> reload
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --flip-local k2:31 --flip-tick 100        # compute window: dual-execution mismatch
fsw/build/cartpole_demo     --theta0 0.05 --seconds 6 --stall 40 --flip-tick 100                # controller stalled 0.4 s: 10-tick hold, then 0 N; reload on resume
python3 sim/render.py --csv flip.csv
python3 sim/campaign.py --deadline                    # stall deadline sweep, protected build
```

Params fields `k0 k1 k2 k3 ki ilim alpha swke swamax swamin swkx swkv sweref swenter swrate swexit mM mm ml mg`, bits 0-31, `+` for multi-bit; also `--flip-state`, `--flip-check`, `--flip-crc`, `--flip-input`. The injector lives in the sim-side `main.cpp`; the flight object code is identical with or without it. The CSV carries `mode` (0 SWING, 1 BALANCE; SAFE and stall ticks report 0), `det` (0 none, 1 SEC, 2 DED, 3 CRC, 4 RANGE, 5 MISMATCH, 6 NONFINITE, 7 STALL, 8 GOLDEN, 9 INPUT), `fdir` (0 NOMINAL, 1 RECOVERING, 2 DEGRADED, 3 SAFE), `sec` and `reload` counters. `specs/how-it-works.html` walks through single, double and triple flips with the numbers from `cartpole_baseline`, `cartpole_demo` and the shipped codec; `specs/system-diagram.html` is the functional diagram; on metal both are also served by the LAN viewer at `/docs/<name>.html` (the image carries only `specs/GLOSSARY.md`).

Sim tools:

```
python3 sim/render.py --theta0 3.14159                 # matplotlib window: cart, pole, traces, force terms
python3 sim/backtest.py --all                            # every scenario, stats after each
python3 sim/backtest.py sim/scenarios/grid.json --show   # one scenario + basin map window
python3 sim/backtest.py bench/rate.json --set dt=0.005,0.02 --set seconds=6
python3 bench/serve.py                                   # bench: LAN browser viewer, stdlib only, http://<host>:8080
```

Scenario files are JSON axes (`theta0_deg`, `x0_m`, `dt`, `substeps`, `cart_mass`, `pole_mass`, `half_len`); lists or `{start, stop, step}` ranges sweep the cartesian product, `{min, max}` with `samples` and `seed` draws random cases. Success = |theta| < 5 deg and |x| < 0.1 m held for the last 2 s of a 10 s run, |x| <= 2.4 m throughout, no fault. `fsw/src/Lqr.cpp` linearizes about upright, discretizes with an exact zero-order hold and iterates the Riccati equation at init (bounded loop, no heap, 4x4 arrays); `fsw/tools/compute_lqr_gains.py` solves the same problem with scipy and the unit test requires agreement to 1e-4 (measured 1e-7, one float32 ULP on three gains).

## Reproducibility

Three things determine the numbers: the start and fault lists, floating-point codegen, and the math library.

- **Start lists** come from an in-repo `splitmix64` PRNG (`sim/backtest.py`), never `std::` distributions, so they are identical on every platform including Mac native. The campaign's fault lists are fixed enumerations in `sim/campaign.py`.
- **Codegen** is pinned by `-ffp-contract=off` and a compile-time ban on fast-math (`fsw/include/fsw/FswTypes.hpp`). Without contraction, IEEE semantics fix the arithmetic for a given architecture.
- **libm** (`sin`, `cos` in the plant) is the residual: it differs between Apple's libm and glibc, and can differ by an ULP between glibc versions. Pinning the container image pins libm.

| Comparison | Start list | Trajectories | Check |
|---|---|---|---|
| Docker on the Mac (arm64 Linux, glibc 2.36) vs Pi 5 (2.41) and Jetson (2.35) metal | identical | bit-exact, so one `aarch64-linux` golden serves all three | same repro hash |
| Mac metal vs aarch64 Linux | identical | last printed digit in 3 of 64 probe cases | different repro hash, identical outcomes |

**Measured on 2026-09-30** with the same source and flags (protected build: swing-up, on-board LQR, encoded store, FDIR):

| Environment | envclass | repro hash (first 16) | demo-1s golden |
|---|---|---|---|
| Mac, Apple clang 17, metal | `arm64-darwin` | `f741d78219d438ca` | byte-identical to the Linux golden |
| Raspberry Pi 5, Pi OS (glibc 2.41), g++, metal | `aarch64-linux` | `6b7e831fb898b98e` | bit-exact |
| Jetson Orin Nano, L4T (glibc 2.35), g++, metal | `aarch64-linux` | `6b7e831fb898b98e` | bit-exact |
| Docker `debian:bookworm-slim` (tag + digest, glibc 2.36), g++ 12.2, on the Mac | `aarch64-linux` | `6b7e831fb898b98e` | bit-exact |

The Pi 5 and the Jetson Orin were rebuilt from 703f03b on 2026-09-30 and ran the full `run_tests.sh` suite and the campaign on metal: both matched the `aarch64-linux` goldens recorded at 9a26e5f bit-exact and printed identical campaign and deadline tables. Nominal output did not change between those two commits, and the commits since 703f03b change only documents, comments and the CMake minimum version.

So on aarch64 Linux the result is bit-exact in the container on the Mac and on the metal of both boards. The two `demo-1s` goldens are byte-identical (same sha256): 1 s is too short for libm drift to reach the printed precision, so the repro hash is the check that tells the environments apart. Over the 64 probe starts the Mac differs from Linux in 3 cases, in the last printed digit (at most 1e-5 in a state column, 1e-4 N in the force columns): case 37 from tick 202 (46 rows), case 44 at tick 767 and case 59 at tick 672 (one row each). Outcomes are identical: 64 of 64 succeed on both.

The probe is `sim/scenarios/repro.json`: 64 seeded random non-moving starts. `python3 sim/backtest.py sim/scenarios/repro.json --hash` prints a sha256 over every case and its raw CSV and compares it against every committed `sim/golden/repro.*.sha256`, printing match or differs for each. `run_tests.sh` step 6 fails if it differs from the golden for this environment class; with no golden for the class it only warns. Same constraint, same hash.

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
| | protected | 9 | 6 | | 3 (input plausibility guard) | 0 |
| G. State words | baseline | 12 | 12 | | | 0 |
| | protected | 12 | | 12 | | 0 |
| H. controller stalled 5–80 ticks | baseline | 5 | | | 5 (actuator hold, then 0 N; no reload) | 0 |
| | protected | 5 | | | 5 (actuator hold, then 0 N; reload on resume) | 0 |

On the unprotected build 42 of 640 single-bit flips (6.6%) are silent data corruption, all sign or exponent bits (24-31). The protected build corrects every one in place and never applies a double-flipped word. The 27 masked compute-window flips hit fields that do not change the balance command (swing-up and model constants, the disabled integrator, a raised exit threshold), so both lanes agreed; 4 of the 27 (ki:31, ilim:30, ilim:31, swexit:30) are read and leave the command unchanged. Set F on the protected build is 6 masked and 3 recovered: the input guard rejects the bit-30 flips of theta, thetadot and x (hold, no reload), while the sign (bit 31) and lowest-exponent (bit 23) flips stay within the plausibility limits and feedback absorbs them. In set H the baseline counts as recovered only because the harness marks stall ticks `det` = STALL; it reloads nothing.

## Where this fails

- **A flip in a register mid-computation, without dual execution.** Memory codes never see it; the wrong value is written back and re-encoded as valid. Dual execution catches one lane; a flip in both lanes in the same tick is undetectable by construction.
- **Three or more flipped bits in one word.** The shipped Hsiao code reports SEC on 5,500 of the 9,139 three-bit patterns of a 39-bit word (60%; extended Hamming would be 69%) and hands back a wrong word. The CRC-32C over the corrected plaintext has Hamming distance 6 at this 640-bit block, so a miscorrected triple (at most four wrong bits) cannot pass it; a pattern with six or more wrong bits that also matches the CRC and the range table would be silent. MBUs are >5% of space memory errors and we do not model adjacency.
- **The pole is already far from upright when the fault lands.** Measured: a stall that begins at t = 1 s is recoverable for 0.6 s (from θ₀ 30°) to 1.3 s (from θ₀ 1°). A stall that begins at a large pole angle has less margin and is not swept.
- **Faults in what we do not protect: code, stack, OS, bus, the plant model.** A flipped instruction, a corrupted return address, a wrong sensor calibration or an unmodelled friction term are outside every layer here. The input guard's reference sample and reject counter and the telemetry counters are plain RAM. Real systems answer these with hardware EDAC, image CRC at boot, bus CRCs and a hardware watchdog.
- **The controller's model sensitivity.** Swing-up reads pole energy through its own model; a ±10% length error made it stall or overshoot the catch until a minimum pump authority and a wider entry rate were added. It now passes every mistune corner, but friction, actuator lag and sensor bias are not modelled yet.
- **The injection model itself.** Software flips are not radiation (no LET, flux or cross-section); faults land at tick boundaries in memory we chose to mark; a green campaign on a Mac does not qualify flight code.

## F´ mapping (one paragraph)

The controller becomes a passive F´ component invoked from a 100 Hz rate group. Input port `SensorFrame` carries x, xdot, theta, thetadot, tick and validity bits; output port `ActuatorCmd` carries force, a sequence number and a CRC over both, verified by the actuator component. Telemetry channels are `sec_total`, `reload_total`, `det_total` and `fdir_state`, with one throttled event per FDIR transition. Parameters (Q, R, model constants) come from `Svc::PrmDb`, whose CRC-checked file is the golden image. The health ping is answered only while FDIR is not SAFE, so `Svc::Health` stops stroking the hardware watchdog when the controller has given up.

## Layout

```
fsw/        flight core: Controller, Lqr, Protect (SECDED + CRC + store), Runtime (decode → input guard → dual step → output guard → FDIR), Math, Plant, harness main (+ injector), tests   (C++14, no deps, no heap, no exceptions)
sim/        render.py (window viewer), backtest.py, campaign.py (verification campaign), scenarios/ (grid, basin, mistune, repro), golden/
bench/      bench tooling, not part of the core: serve.py + viewer.html (LAN browser viewer, used by --serve/--demo), pin_image.sh, rate.json
scripts/    build_demo.sh, run_tests.sh, clean_up.sh, make_golden.sh, docker-entry.sh
specs/      SPEC-01-system-build.md, SPEC-02-softfault-testing.md, GLOSSARY.md, how-it-works.html, system-diagram.html
research_summary.html   synthesis of five independent reviews of the plan as of 2026-09-29 (historical; the reviews themselves are internal, not shipped)
Dockerfile  pinned software constraint
```

## What was cut

Cut, documented but not built: command sequence + CRC (F´ port), sampled assessment campaign, friction/lag/bias plant axes, Docker on the boards as a required path (same image validated on the Mac; boards run metal), hardware EDAC assumptions, the flight/sim library split.
