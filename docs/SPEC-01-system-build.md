# Spec 01 — Cart-pole soft-fault-aware control system (build)

**Status:** draft for implementation · **rev 2 (2026-09-29):** environment tiers for Pi/Jetson metal + Docker, C++14, iteration 3 = full stack, pure-C++ default build with additive `--visual`, `build_demo.sh` / `run_tests.sh`  
**Audience:** builder (implementation) / reviewer (summary context)  
**Deliverable shape:** small C++ system in a zip; one bootstrap script; staged builds that show the hardening story.

---

## 1. Goal

Ship a **small, runnable system** (not a one-file toy) that:

1. Stabilizes an inverted cart-pole from any **non-moving** initial condition (per the assignment).
2. Shows a clear **software-only** soft-error / “radiation robustness” story via staged hardening.
3. On detected memory corruption in control state, **recovers safely**: reloads the verified golden parameters and reseeds dynamic state without a command jump.
4. Runs on a reviewer Mac **without requiring Docker**; optional Docker; F´/RPi is stretch only.

**Honesty constraints**

- Soft errors are **simulated in software**. Say so in README, slides, and build notes.
- This is not flight FSW and not a claim that SECDED on a laptop equals space-grade EDAC.
- The ask was an **optimization-based** controller. The balance controller is discrete LQR with the gain **solved on board from the Riccati equation at init** (Q, R and the plant model are the design inputs; K is derived). Swing-up from hanging-down uses energy shaping, which is Lyapunov-based, not optimization-based, and the summary says so. iLQR is the stretch rung of the degradation ladder, not the baseline.

---

## 2. Environment (exact target)

| Tier | Environment | Status | Notes |
|------|-------------|--------|-------|
| **A — Primary (metal)** | macOS Apple Silicon or Intel; Apple Clang; CMake ≥ 3.10; **C++14** | Required | `bash scripts/build_demo.sh`: detect, compile-a-file dependency check, build, verify, stamp. Missing packages are offered with y/N confirmation (`--force-default` answers yes; `--no-install` only prints). `--visual` opens the window viewer. |
| **B — Boards (metal)** | Raspberry Pi 5 (Pi OS) and NVIDIA Jetson Orin (L4T), aarch64, g++ | Measured, not required | Same script; `--demo` ends by starting the LAN viewer (`bench/serve.py`) so a browser on the bench can drive it. Both boards reproduce the `aarch64-linux` golden bit-exact (README table). |
| **C — Pinned constraint (Docker)** | `Dockerfile`, Debian bookworm pinned by digest | Optional appendix | `docker build -t cartpole . && docker run --rm cartpole tests`. Same image on any arm64 host gives the same result hash. Never installed by our scripts. |
| **D — Stretch** | F´ component on the Pi | One paragraph | The controller becomes a passive F´ component invoked from a 100 Hz rate group. Input port `SensorFrame` carries x, xdot, theta, thetadot, tick and validity bits; output port `ActuatorCmd` carries force, a sequence number and a CRC over both, verified by the actuator component. Telemetry channels are `sec_total`, `reload_total`, `det_total` and `fdir_state`, with one throttled event per FDIR transition. Parameters (Q, R, model constants) come from `Svc::PrmDb`, whose CRC-checked file is the golden image. The health ping is answered only while FDIR is not SAFE, so `Svc::Health` stops stroking the hardware watchdog when the controller has given up. |

**C++ standard:** the flight core (`fsw/`) is **C++14** (F´ v4 requires C++14; the code is also C++11-clean). No exceptions, no RTTI, no heap after init, no STL containers in the hot path. Sim and harness code may use anything.

**Package:** git repository (zip export of the same tree):

```
orca-cartpole-softfault/
  README.md                 # R1–R5 with a number each, how to run, reproducibility, limitations, hours
  fsw/                      # flight core: Controller (pure step), Lqr (on-board design), Plant, harness main (+ --flip injector), tests
  sim/                      # render.py (window viewer, presentation), backtest.py + scenarios/ (grid, basin, mistune, repro), golden/
  scripts/                  # build_demo.sh, run_tests.sh, clean_up.sh, make_golden.sh, docker-entry.sh
  docs/                     # SPEC-01, SPEC-02, GLOSSARY, how-it-works.html, system-diagram.html
  bench/                    # bench tooling, not part of the core: bench_setup.sh, LAN viewer, image pinning
  Dockerfile                # pinned software constraint
```

**Build contract (`scripts/build_demo.sh`)**

1. Detect OS / arch / board and derive the environment class (`arm64-darwin`, `aarch64-linux`).
2. Check deps by compiling a file (not `command -v`): C++14 compiler, CMake. Python + matplotlib are optional (viewer, back-tests). On a miss: print the exact install command and offer to run it (y/N; `--force-default` = yes to all; `--no-install` = print only). apt-get lines carry `sudo`; the script itself is never run as root.
3. Build Release: `-std=c++14 -O2 -ffp-contract=off -fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror`.
4. Verify: unit tests; 1 s demo with schema check; golden comparison (bit-exact within the environment class).
5. Stamp `build_info.json`.

The default build is pure C++ and emits CSV. `--visual`, `--demo` (host-aware: Mac window, boards LAN viewer), `--serve` and `--docker` are additive; `--explain` prints every check.

## 3. Physics + control requirements (aligned to the assignment)

**Plant (sim):** classic cart-pole, continuous → discrete RK4 or semi-implicit Euler at fixed `dt` (e.g. 1–10 ms).

**Success (control), the one definition used everywhere (README, `sim/backtest.py`, §9):** from any initial state with zero velocities, |θ| < 5° **and** |x| < 0.1 m held for the final 2 s of a 10 s run, |x| ≤ 2.4 m throughout (rail), |u| ≤ 20 N, no fault flag. "Any non-moving start" includes hanging-down (θ₀ = π), so swing-up is part of the requirement, not an extra. IC grid: θ₀ ∈ [−180°, 180°] every 2° × x₀ ∈ {−1, −0.5, 0, 0.5, 1} m (`sim/scenarios/grid.json`). Report the success fraction and the basin edges.

**Param robustness:** small mistuning of mass/length/friction still succeeds (document sweep ranges).

**Controller core (iteration 1):**

- Full-state discrete LQR, u = −K·[x, ẋ, θ, θ̇], K from the on-board DARE solve (`fsw/src/Lqr.cpp`), θ wrapped to (−π, π]. Offline scipy gains are the cross-check and the fallback table.
- Energy-shaping swing-up with a hysteresis handoff to LQR: enter balance below 31.5° when |θ̇| < 4 rad/s, fall back above 43°, deterministic kick at exactly θ = π, and a minimum pump authority so a model error cannot stall the swing below the top. Status: **built**; grid, basin and ±10% mistune sweeps all 100% (README table).
- Design parameters (Q, R, model constants, limits) in a POD `Params` block. No lookup table: LQR needs none, and a table that exists only to be protected is padding.

## 4. Build iterations (what we ship)

Two **compile-time builds** from one source (`-DPROTECT=0/1`), because a runtime mode word is itself an unprotected single point of failure. The middle "checksum-only" tier of rev 1 is dropped: it existed only to lose to the full tier on the same fault class. The table keeps the iteration names for the narrative.

| Build | Name | What it is |
|-------|------|------------|
| `PROTECT=0` | Iteration 1 · baseline | Plant + swing-up + LQR balance (gains solved on board). No protection. The `--flip` injector shows what a single bit does. |
| `PROTECT=1` | Iteration 2 · protected | One coherent ladder, each layer catching what the previous one misses: **SECDED** Hsiao(39,32) on every word of `Params` and `State` → **CRC-32C** over the corrected `Params` plaintext (fixed at init) → **range table** per parameter → **output guard** (clamp and safe force from compile-time constants) → **dual execution** of `step()` with compare → **actuator-side freshness** (stale command for N ticks → fallback) → **FDIR** (NOMINAL → RECOVERING → SAFE latched; DEGRADED added when swing-up gives it a distinct law). On any detection: verified golden reload + bumpless reseed. |

The table keeps "Iteration 1 / 2" for the narrative; there is no middle tier. **Status: both builds exist** (`cartpole_baseline`, `cartpole_demo`); the verification campaign in `sim/campaign.py` is the evidence (README table).

**Plus (the protected build):**

- **Output guard:** `isfinite(u)`, `|u| ≤ FORCE_LIMIT_N`, from constants outside `Params`. Any rate limit on Δu is examined for basin effect before it ships.
- **Actuator-side freshness (watchdog):** command age in simulated ticks, never wall-clock. Stale for N ticks → fallback to a freshly rebuilt golden LQR. Recovery deadline ≤ 300 ms; zero force is an interim command, never a "safe state" (an inverted pendulum has none). Hold-last is never used. An in-process heartbeat cannot see its own hang and is not used.
- **Command integrity:** in flight the command port carries a sequence number and CRC (F´ port + bus CRC). Between two structs in one process there is no link to corrupt, so it is documented in the F´ mapping, not modelled here.
- **Recovery policy (required):** on any detection (DED, CRC fail, range fail, dual-execution mismatch): reload `Params` from the verified golden (`params_default()` literals, CRC-checked), re-encode the store, reseed the θ̇ filter from the current measurement (bumpless), enter RECOVERING; NOMINAL after M clean ticks; SAFE (latched) on persistence. Never zero an integrator blindly, never hold the last command. Log mechanism, field, tick, action, counters.
- **FDIR states:** NOMINAL → RECOVERING → SAFE (latched) now; DEGRADED (= swing-up inhibited, tighter clamp) is added when swing-up exists and gives it a distinct law. State word stored as (v, ~v) with Hamming-distance-16 codes; `default:` → SAFE.

**Rad-hard note (docs + slides, not fake HW):** gains and limits live in a `Params` region that on the laptop is protected in software; in flight it would sit in EDAC / scrubbed memory (see §8, ideal with hardware).

## 5. Soft-fault awareness as a **system**

Not a single `.cpp`. Minimum modules:

| Module | Responsibility |
|--------|----------------|
| `fsw/` (flight core) | `Controller` (pure `step()`, swing-up + LQR), `Lqr` (on-board design), `Protect` (store, SECDED, CRC, range table), `Fdir` (states, counters, reload). No I/O, no clock, no heap, no knowledge of the injector. |
| `sim/` | Plant (RK4), `--flip` injector and stall injection in the harness `main.cpp`, CSV, back-test harness, viewers. |

**Data that must be protectable (priority order):**

1. Controller gains, limits, model constants  
2. Integrator state  
3. Derivative / filter memory  
4. Last command / watchdog counters  

Sensors/plant state: optionally checksum for demo realism; primary story is **controller memory**.

---

## 6. SECDED + checksum (standards for this project)

**Checksum:** CRC-32C (Castagnoli) over the 36-byte `Params` plaintext, computed at init and never recomputed at runtime; `State` is re-encoded each tick only after its invariants pass. HD = 6 at this block length; one of four independent layers.

**SECDED:** Hsiao(39,32) on every 32-bit word (all fields are F32), OpenTitan parity masks, exhaustive 1-, 2- and 3-bit codec test. Document:

- Correctable: fix bit, write back (scrub), count `corrected` once per event, continue.
- Detectable uncorrectable: **do not** trust data → recovery policy (§4).
- Limitation: 3+ bits in a word are miscorrected 60–69% of the time by any SECDED; the CRC and range table exist to catch that (see docs/how-it-works.html §3).

**Ordering:** decode and correct every word → CRC-32C over the corrected plaintext → range table → use. Log `{secded_status, crc_status, range_status}` per event; the first detector names the outcome so nothing is double-counted.

## 7. Build flags / "include simulated error in the build"

CMake:

- `PROTECT` = `0` (baseline) | `1` (protected). Compile-time; no runtime mode word.
- The injector is sim-side only (`--flip field:bit --flip-tick N`, stall injection), so the flight object code is identical whether or not injection is used. No `ENABLE_FAULT_INJECTION` gate in the core.

README **must** say: "The demo harness includes a software bit-flip injector for evaluation only; the flight core has no knowledge of it."

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
| EDAC SRAM / memory controller with SECDED and a readable syndrome | Software SECDED on the store (iteration 3, stage 2) | Reacting to the syndrome: correction counts to telemetry, DED → the same FDIR path |
| Hardware scrubber | Per-tick re-encode and write-back | Scrub rate vs. exposure of rarely-read data (golden image) |
| Lockstep or dual-core compare | Dual execution of `step()` | The compare policy: what to do on mismatch |
| Hardware watchdog (WDT) | Actuator-side freshness monitor | Stroking it only while every health check passes (F´ `Svc::Health` pattern) |
| EDAC-protected flash / NVM | Golden literals verified by CRC | Verifying the image at boot anyway |
| Bus-level CRC (1553 parity, SpaceWire RMAP, CAN CRC) | Command encoding with seq + CRC | Freshness and authority checks; ECC is not authentication |

With that hardware the software layers that stay valuable are the compute-window compare, the range table and plausibility guard, the output guard from constants, the command sequence check, and the FDIR ladder. The layers that become redundant (software SECDED, software scrub) stay in the code behind a compile-time policy so the same core builds for both cases.

## 9. Acceptance (build)

- [ ] `scripts/build_demo.sh` succeeds on a clean Mac with Xcode CLT + CMake; `scripts/run_tests.sh` passes  
- [ ] Two builds (`PROTECT=0/1`) run and log CSV with the same schema  
- [ ] Control: success fraction over the full θ₀ × x₀ grid incl. hanging-down; ±10% plant mistune; criterion from §3  
- [ ] Codec: exhaustive 1-, 2-, 3-bit test on Hsiao(39,32); every single flip corrected, every double flip DED, never a silent "fix"  
- [ ] Single-bit flip in every `Params` word through the protected build: corrected, command unchanged  
- [ ] Double-bit flip: DED → verified golden reload → RECOVERING → NOMINAL; the unprotected build fails the same flip  
- [ ] Range table: the §2 flips of docs/how-it-works.html (sign and exponent) are rejected even with codes disabled  
- [ ] Dual execution: a corruption of the decoded local copy between decode and `step()` is detected  
- [ ] Stall: k skipped ticks with ZOH; measured recovery deadline reported against the 300 ms budget  
- [ ] FDIR transitions table-driven and unit-tested; state word (v, ~v)  
- [ ] README states the fault model, the limitations, the criterion, hours spent and what was cut  

## 10. Open decisions (resolved 2026-09-29)

1. **Controller.** Full-state LQR with **K solved on board** from the discrete Riccati equation at init (Q, R and the plant model are the parameters; the gain is derived, and can be re-derived from golden on recovery). **Energy-shaping swing-up** with a hysteresis handoff to LQR covers hanging-down starts. **iLQR** is the stretch: if built it becomes the top rung of the degradation ladder (iLQR → LQR → safe) with a fixed iteration cap and a cold start on any detection; it never replaces the LQR rung.
2. **Checksum.** **CRC-32C** (Castagnoli; HD=6 at this block size; hardware instruction on ARMv8 and x86). It is one of four independent layers with SECDED, the range table and dual execution; nothing relies on it alone.
3. **SECDED.** 32-bit words (every field is F32) → **Hsiao(39,32)** using the OpenTitan parity masks (Apache-2.0), with an exhaustive 1-, 2- and 3-bit codec test. Not hand-rolled Hamming.
4. **Golden params.** **Not a duplicated RAM copy** (equally exposed; ~8.5%/year chance of a latent flip at LEO rates if unread). The golden is the compile-time literals in `params_default()`, which live in read-only pages, **verified against a pinned CRC before every reload**. In flight this is the EDAC-protected image; here it is the closest software equivalent.

---
*Cross-verified and adjusted against a Fable 5.1 agent review (research/06-spec-complexity-review.md), 2026-09-29.*
