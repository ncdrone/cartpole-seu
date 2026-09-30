# Spec 01 — Cart-pole soft-fault-aware control system (build)

**Status:** implemented · **rev 3 (2026-09-30):** brought in line with the code at this commit (protected build = full stack) · rev 2 (2026-09-29): environment tiers for Pi/Jetson metal + Docker, C++14, pure-C++ default build with additive `--visual`, `build_demo.sh` / `run_tests.sh`  
**Audience:** builder (implementation) / reviewer (summary context)  
**Deliverable shape:** small C++ system in a zip; one bootstrap script; two compile-time builds that show the hardening story.

---

## 1. Goal

Ship a **small, runnable system** (not a one-file toy) that:

1. Stabilizes an inverted cart-pole from any **non-moving** initial condition (per the assignment).
2. Shows a clear **software-only** soft-error / “radiation robustness” story via a baseline build and a protected build.
3. On detected memory corruption in control state, **recovers safely**: reloads the verified golden parameters and reseeds dynamic state without a command jump.
4. Runs on a reviewer Mac **without requiring Docker**; optional Docker; Pi 5 and Jetson metal are measured, not required; F´ is stretch only.

**Honesty constraints**

- Soft errors are **simulated in software**. Say so in README, slides, and build notes.
- This is not flight FSW and not a claim that SECDED on a laptop equals space-grade EDAC.
- The ask was an **optimization-based** controller. The balance controller is discrete LQR with the gain **solved on board from the Riccati equation at init** (Q, R and the plant model are the design inputs; K is derived). Swing-up from hanging-down uses energy shaping, which is Lyapunov-based, not optimization-based, and the summary says so. iLQR is the stretch rung of the degradation ladder, not the baseline.

---

## 2. Environment (exact target)

| Tier | Environment | Status | Notes |
|------|-------------|--------|-------|
| **A — Primary (metal)** | macOS on Apple Silicon (measured; Intel expected, untested); Apple Clang; CMake ≥ 3.13; **C++14** | Required | `bash scripts/build_demo.sh`: detect, dependency check, build, stamp, verify. Missing packages are offered with y/N confirmation (`--force-default` answers yes; `--no-install` only prints). `--visual` opens the window viewer. |
| **B — Boards (metal)** | Raspberry Pi 5 (Pi OS) and NVIDIA Jetson Orin (L4T), aarch64, g++ | Measured, not required | Same script; `--demo` ends by starting the LAN viewer (`bench/serve.py`) so a browser on the bench can drive it. Both boards were rebuilt from 703f03b on 2026-09-30, ran the full `run_tests.sh` suite and the campaign on metal, and reproduced the `aarch64-linux` goldens (recorded at 9a26e5f) bit-exact (README table). |
| **C — Pinned constraint (Docker)** | `Dockerfile`, Debian bookworm pinned by tag and digest | Optional appendix | `docker build -t cartpole . && docker run --rm cartpole tests`. Docker on the Mac reproduces the `aarch64-linux` golden that the boards produce on metal; Docker on the boards was not run (same image validated on the Mac; boards run metal). Never installed by our scripts. |
| **D — Stretch** | F´ component on the Pi | One paragraph | The controller becomes a passive F´ component invoked from a 100 Hz rate group. Input port `SensorFrame` carries x, xdot, theta, thetadot, tick and validity bits; output port `ActuatorCmd` carries force, a sequence number and a CRC over both, verified by the actuator component. Telemetry channels are `sec_total`, `reload_total`, `det_total` and `fdir_state`, with one throttled event per FDIR transition. Parameters (Q, R, model constants) come from `Svc::PrmDb`, whose CRC-checked file is the golden image. The health ping is answered only while FDIR is not SAFE, so `Svc::Health` stops stroking the hardware watchdog when the controller has given up. |

**C++ standard:** the flight core (`fsw/`) is **C++14** (F´ v4 requires C++14; the code is also C++11-clean). No exceptions, no RTTI, no heap after init, no STL containers in the hot path. Sim and harness code may use anything.

**Package:** git repository (zip export of the same tree):

```
cartpole-seu/
  README.md                 # run it, R1–R5 with a number each, how to run, reproducibility, limitations
  fsw/                      # flight core: Controller (pure step, range table), Lqr (on-board design), Protect (SECDED, CRC, store),
                            #   Runtime (decode → input guard → dual step → output guard → FDIR), Math, Config; Plant and the harness
                            #   main (+ injector) are sim-side code in this tree; tests/, tools/ (scipy cross-check)
  sim/                      # render.py (window viewer), backtest.py + scenarios/ (grid, basin, mistune, repro), campaign.py, golden/
  scripts/                  # build_demo.sh, run_tests.sh, clean_up.sh, make_golden.sh, docker-entry.sh
  specs/                    # SPEC-01, SPEC-02, GLOSSARY, how-it-works.html, system-diagram.html
  bench/                    # bench tooling, not part of the core: serve.py + viewer.html (LAN viewer), pin_image.sh, rate.json
  Dockerfile                # pinned software constraint
```

**Build contract (`scripts/build_demo.sh`)**

1. Detect OS / arch / board and derive the environment class (`arm64-darwin`, `aarch64-linux`).
2. Check deps: the C++14 compiler by compiling and running a file (not `command -v`: a clean Mac has a `clang++` shim that only prompts for the CLT); CMake by presence. Python + matplotlib are optional (viewer, back-tests). On a miss: print the exact install command and offer to run it (y/N; `--force-default` = yes to all; `--no-install` = print only). apt-get lines carry `sudo`; the script should not be run as root (it warns and drops `sudo`).
3. Build Release: `-std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror`.
4. Stamp `fsw/build/build_info.json`.
5. Verify: unit tests; 1 s demo with schema check; golden comparison (bit-exact within the environment class).

The default build is pure C++ and emits CSV. `--visual`, `--demo` (host-aware: Mac window, boards LAN viewer), `--serve` and `--docker` are additive; `--explain` prints every check.

## 3. Physics + control requirements (aligned to the assignment)

**Plant (sim):** classic cart-pole (Florian equations, frictionless), RK4 at dt = 10 ms (`--dt`, `--substeps` adjustable).

**Success (control), the one definition used everywhere (README, `sim/backtest.py`, §9):** from any initial state with zero velocities, |θ| < 5° **and** |x| < 0.1 m held for the final 2 s of a 10 s run, |x| ≤ 2.4 m throughout (rail), |u| ≤ 20 N, no fault flag. "Any non-moving start" includes hanging-down (θ₀ = π), so swing-up is part of the requirement, not an extra. IC grid: θ₀ ∈ [−180°, 180°] every 2° × x₀ ∈ {−1, −0.5, 0, 0.5, 1} m (`sim/scenarios/grid.json`). Report the success fraction and the basin edges.

**Param robustness:** mass and length mistuned by ±10% (`sim/scenarios/mistune.json`) still succeed; friction is not modelled.

**Controller core (iteration 1):**

- Full-state discrete LQR, u = −K·[x, ẋ, θ, θ̇], K from the on-board DARE solve (`fsw/src/Lqr.cpp`), θ wrapped to (−π, π]. The offline scipy gains are the cross-check (`test_controller`) and the range-table reference; if the on-board solve does not converge, `runtime_init` fails and the harness exits 3. There is no runtime table fallback.
- Energy-shaping swing-up with a hysteresis handoff to LQR: enter balance below 31.5° when |θ̇| < 4 rad/s, fall back above 43°, a deterministic kick when |θ̇·cos θ| ≤ 1e-3 (for example at rest at π), and a minimum pump authority so a model error cannot stall the swing below the top. Status: **built**; grid, basin and ±10% mistune sweeps all 100% (README table).
- Gains (derived from Q, R), swing-up settings, integrator/filter settings and the controller's model constants live in a POD `Params` block (20 × F32, 80 B). Q, R and the force limits are compile-time constants. No lookup table: LQR needs none, and a table that exists only to be protected is padding.

## 4. Build iterations (what we ship)

Two **compile-time builds** from one source (`FSW_PROTECT=0/1`, set per target in `fsw/CMakeLists.txt`, which builds both), because a runtime mode word is itself an unprotected single point of failure. The middle "checksum-only" tier of rev 1 is dropped: it existed only to lose to the full tier on the same fault class.

| Build | Name | What it is |
|-------|------|------------|
| `cartpole_baseline` (`FSW_PROTECT=0`) | Iteration 1 · baseline | Plant + swing-up + LQR balance (gains solved on board). Plain `Params`/`State` in RAM, `step()` once, no detection, no FDIR. The `--flip` injector shows what a single bit does. |
| `cartpole_demo` (`FSW_PROTECT=1`) | Iteration 2 · protected | One coherent ladder, each layer catching what the previous one misses: **SECDED** Hsiao(39,32) on every word of `Params` and `State` → **CRC-32C** over the corrected plaintext → **range table** per parameter → **input plausibility guard** → **dual execution** of `step()` with compare → **output guard** (clamp and safe force from compile-time constants) → **DEGRADED law** → **FDIR** (NOMINAL → RECOVERING → DEGRADED → SAFE, latched); sim side, an **actuator freshness monitor**. On any store or compute detection: verified golden reload + bumpless reseed; the first input reject holds the last sample without a reload. |

**Status: both builds exist**; the verification campaign in `sim/campaign.py` is the evidence (README table).

**Plus (the protected build):**

- **Output guard:** `isfinite(u)`, `|u| ≤ FORCE_LIMIT_N` (20 N), from constants outside `Params`. There is no rate limit on Δu.
- **Input plausibility guard (`det` 9 INPUT):** a non-finite field, |x| > 2.9 m (rail + 0.5 m), |ẋ| > `PLAUS_MAX_XDOT` (10 m/s), |θ̇| > `PLAUS_MAX_RATE` (25 rad/s), or a θ step that differs from the trapezoid of θ̇ × dt by more than `PLAUS_THETA_TOL` (0.05 rad) is rejected. The first reject holds the last accepted sample and is a detection without a reload; a second consecutive reject reloads the golden and runs both lanes on it; a third latches SAFE through the persistence count. With no accepted sample to hold (first tick, or after a stall or a store/compute reload), a non-finite sample skips `step()` and commands 0 N, and a finite one has x, ẋ and θ̇ clamped to the limits.
- **Actuator-side freshness (watchdog):** command age in simulated ticks, never wall-clock. A stale command is held for `STALE_MAX_TICKS` = 10 ticks (100 ms), then the actuator applies 0 N; when the controller resumes, the stall is a detection (`det` 7 STALL) and the golden is reloaded. Recovery deadline ≤ 300 ms; `campaign.py --deadline` measures the longest recoverable stall (600–1,300 ms from θ₀ = 30°…1°). Zero force is an interim command, never a "safe state" (an inverted pendulum has none). An in-process heartbeat cannot see its own hang and is not used.
- **Command integrity:** in flight the command port carries a sequence number and CRC (F´ port + bus CRC). Between two structs in one process there is no link to corrupt, so it is documented in the F´ mapping, not modelled here.
- **Recovery policy (required):** on any detection other than a SEC correction (DED, CRC fail, range fail, dual-execution mismatch, non-finite or oversized output, stall, repeated input reject): rebuild `Params` from the nominal model (the defaults plus the on-board Riccati K, `params_design()`) and accept it only if it reproduces the CRC-32C pinned at init; re-encode the store; reseed `State` (the integrator is disabled, `KI_X` = 0, and reseed zeroes it; the θ̇ filter re-seeds from the next measurement, bumpless; the mode is re-picked from the measured angle). If the rebuilt golden does not match the pinned CRC, go to SAFE (`det` 8 GOLDEN). The CSV logs, per tick, the first detector's code, the FDIR state and the `sec` and `reload` counters; the injected field is printed by the harness on stderr.
- **FDIR states:** four, in `Runtime.cpp`. A SEC correction is counted and never changes state; a SEC tick counts as clean. Every other detection counts in a persistence window that opens at the first detection and lasts `FDIR_WINDOW_TICKS` = 200 ticks; a detection after the window has run out opens a new one. The third detection in the window (`FDIR_MAX_DETECTIONS` = 3) latches SAFE. Otherwise, a detection while DEGRADED latches SAFE; a detection while RECOVERING that is the second in the window enters DEGRADED; any other detection enters RECOVERING. RECOVERING returns to NOMINAL after `FDIR_CLEAN_TICKS` = 50 consecutive clean ticks, DEGRADED after `FDIR_DEGRADED_CLEAN_TICKS` = 200. DEGRADED inhibits swing-up (a swing-mode tick commands 0 N and sets the fault flag) and halves the clamp to 10 N. SAFE commands 0 N with the fault flag set every tick, reports mode 0, does not run `step()` (decode hands the caller compile-time defaults, never the store), and has no exit. The FDIR state word is stored as (v, ~v) with codes at pairwise Hamming distance ≥ 16; the detection count, window start and clean-tick count are also stored as (v, ~v). A mismatch in any of them, or an unknown state code, reads as SAFE.

**Rad-hard note (docs + slides, not fake HW):** gains and model constants live in a `Params` region that on the laptop is protected in software; in flight it would sit in EDAC / scrubbed memory (see §8, ideal with hardware). The force limits are compile-time constants.

## 5. Soft-fault awareness as a **system**

Not a single `.cpp`. Minimum modules:

| Module | Responsibility |
|--------|----------------|
| `fsw/` (flight core) | `Controller` (pure `step()`, swing-up + LQR, range table), `Lqr` (on-board design), `Protect` (SECDED, CRC-32C, encoded store), `Runtime` (decode, input guard, dual execution, output guard, DEGRADED law, FDIR, golden reload), `Math` (wrap, clamp). No I/O, no clock, no heap, no knowledge of the injector. |
| Sim side | `fsw/src/Plant.cpp` (RK4) and the harness `fsw/src/main.cpp` (actuator freshness monitor, `--flip` and stall injection, CSV) are sim-side code that still lives in the `fsw/` tree; `Plant.cpp` is linked into `libfsw.a` (the flight/sim split is not built). `sim/`: back-tests, campaign, viewers. |

**Data that must be protectable (priority order):**

1. Controller gains and model constants (the force limits are constants)  
2. Integrator state  
3. Derivative / filter memory  
4. Last command / watchdog counters  

Sensors/plant state: the input plausibility guard checks each sample; the primary story is **controller memory**.

---

## 6. SECDED + checksum (standards for this project)

**Checksum:** CRC-32C (Castagnoli) over the 80-byte `Params` plaintext, stored with its complement. The reference is pinned at init (golden CRC 0x9E919ECA) and the store CRC is rewritten only after a verified reload. `State` has its own CRC and is re-encoded every tick after `step()`. HD = 6 at this block length; one of four independent layers.

**SECDED:** Hsiao(39,32) on every 32-bit word. `Params` is 20 F32 words; `State` is 4 words (two F32, a U32 tick, and four U8: `filt_init`, `mode`, `pad[2]`). The 32 data columns are the first 32 weight-3 7-bit values in ascending order, a `static const` table in `Protect.cpp`. `test_protect` flips every single bit and every pair of bits (all 39 positions) of 8 sample words, runs the census of all 4,960 data-bit triples, checks a CRC-32C known answer and pushes triple-bit patterns through the store. Document:

- Correctable: fix bit, write back (scrub), count `corrected` once per event, continue.
- Detectable uncorrectable: **do not** trust data → recovery policy (§4).
- Limitation: this code reports SEC on 60% of 3-bit patterns in a word (5,500 of 9,139; extended Hamming would be 69%) and hands back a wrong word. The CRC exists to catch that: at HD = 6 a miscorrected triple (at most four wrong bits) cannot pass it (see specs/how-it-works.html §3).

**Ordering:** decode and correct every word → CRC-32C over the corrected plaintext → range table → use. The CSV carries one `det` code per tick, named by the first detector other than SEC: a SEC code is overwritten by a later detector in the same tick (a miscorrected triple logs `det` 3 CRC and still increments `sec`). Several detectors can fire in one tick (for example a dual-execution mismatch followed by a non-finite output); each calls `on_detection()`, so the detection counters can rise by 2 in that tick.

## 7. Build flags / "include simulated error in the build"

CMake:

- `FSW_PROTECT` = `0` (baseline) | `1` (protected), a compile definition per target; `fsw/CMakeLists.txt` builds `cartpole_baseline`, `cartpole_demo`, `test_controller`, `test_protect` and `test_fdir` (`ctest` runs the three tests). No runtime mode word.
- The injector is sim-side only (`--flip field:bit --flip-tick N`, stall injection), so the flight object code is identical whether or not injection is used. No `ENABLE_FAULT_INJECTION` gate in the core.

README **must** say: "The demo harness includes a software bit-flip injector for evaluation only; the flight core has no knowledge of it." (It does, under R1–R5.)

## 8. Out of scope (explicit), and the ideal with hardware support

Out of scope:

- Real radiation / beam testing  
- Multi-board HW diversity  
- Full F´ deployment on the Pi (stretch writeup only)  
- Claiming any specific flight SoC  
- RL as the primary controller  
- Control-flow signatures (CFCSS), Reed-Solomon, MBU adjacency models, bus CRCs, OS/kernel faults: documented in the failure-modes table, not built  

**The ideal, with hardware support.** This package assumes no help from the hardware, which is the honest assumption for COTS compute. On a rad-tolerant OBC the picture changes and the software should be written to exploit it:

| Hardware feature | What it replaces here | What software still owns |
|---|---|---|
| EDAC SRAM / memory controller with SECDED and a readable syndrome | Software SECDED on the store (the protected build) | Reacting to the syndrome: correction counts to telemetry, DED → the same FDIR path |
| Hardware scrubber | Per-tick re-encode and write-back | Scrub rate vs. exposure of rarely-read data (golden image) |
| Lockstep or dual-core compare | Dual execution of `step()` | The compare policy: what to do on mismatch |
| Hardware watchdog (WDT) | Actuator-side freshness monitor | Stroking it only while every health check passes (F´ `Svc::Health` pattern) |
| EDAC-protected flash / NVM | Golden rebuilt from the model and checked against the CRC pinned at init | Verifying the image at boot anyway |
| Bus-level CRC (1553 parity, SpaceWire RMAP, CAN CRC) | Command encoding with seq + CRC (documented, not built) | Freshness and authority checks; ECC is not authentication |

With that hardware the software layers that stay valuable are the compute-window compare, the range table and plausibility guard, the output guard from constants, the command sequence check, and the FDIR ladder. The layers that become redundant (software SECDED, software scrub) would stay in the code behind a compile-time policy so the same core builds for both cases; that policy is not built (`FSW_PROTECT` is all-or-nothing today).

## 9. Acceptance (build)

- [x] `scripts/build_demo.sh` succeeds on a Mac with Xcode CLT + CMake; `scripts/run_tests.sh` passes (all 7 steps)  
- [x] Two builds (`FSW_PROTECT=0/1`) run and log CSV with the same schema  
- [x] Control: success fraction over the full θ₀ × x₀ grid incl. hanging-down (905/905); ±10% plant mistune (108/108); criterion from §3  
- [x] Codec: every single flip corrected, every double flip DED, never a silent "fix" of a double; 3-bit data-pattern census (59% of data triples miscorrected)  
- [x] Single-bit flip in every `Params` word through the protected build: corrected, command unchanged (640/640)  
- [x] Double-bit flip: DED → verified golden reload → RECOVERING → NOMINAL (60/60); the unprotected build fails the same flips (6/60 SDC)  
- [ ] Range table on its own: not tested with the codes disabled (no such build). Measured offline with the shipped `params_in_range()`: it rejects 38 of the 42 baseline SDC flips and misses alpha bits 26–29, which stay in (0, 1]  
- [x] Dual execution: a corruption of the decoded local copy between decode and `step()` is detected (13 of 40; the other 27 hit fields that do not change the balance command: swing-up and model constants, the disabled integrator, a raised exit threshold; 4 of them, ki:31, ilim:30, ilim:31 and swexit:30, are read and leave the command unchanged)  
- [x] Stall: k skipped ticks with ZOH; measured recovery deadline reported against the 300 ms budget (`campaign.py --deadline`)  
- [x] FDIR transitions unit-tested (`test_fdir` (a)–(o)); state word and counters (v, ~v). The transitions are an if/else in `on_detection()`, not a table  
- [x] README states the fault model, the limitations, the criterion and what was cut  

## 10. Open decisions (resolved 2026-09-29)

1. **Controller.** Full-state LQR with **K solved on board** from the discrete Riccati equation at init (Q, R and the plant model are the parameters; the gain is derived, and is re-derived from the model on every reload). **Energy-shaping swing-up** with a hysteresis handoff to LQR covers hanging-down starts. **iLQR** is the stretch: if built it becomes the top rung of the degradation ladder (iLQR → LQR → safe) with a fixed iteration cap and a cold start on any detection; it never replaces the LQR rung.
2. **Checksum.** **CRC-32C** (Castagnoli; HD=6 at this block size; hardware instruction on ARMv8 and x86). It is one of four independent layers with SECDED, the range table and dual execution; nothing relies on it alone.
3. **SECDED.** 32-bit words → **Hsiao(39,32)** with the first 32 weight-3 7-bit values in ascending order as the data columns, an exhaustive 1- and 2-bit codec test and a 3-bit census. Not hand-rolled Hamming.
4. **Golden params.** **Not a duplicated RAM copy** (equally exposed; ~8.5%/year chance of a latent flip at LEO rates if unread). The golden is re-derived at each reload from the nominal model (the compile-time defaults plus the on-board Riccati K, `params_design()`) and accepted only if it reproduces the CRC-32C pinned at init; a mismatch is SAFE. The model itself sits in RAM, so a corrupt model fails the pinned CRC rather than producing a wrong golden. In flight this is the EDAC-protected image; here it is the closest software equivalent.
