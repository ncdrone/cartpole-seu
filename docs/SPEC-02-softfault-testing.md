# Spec 02 — Soft-fault (software bit-flip) testing

**Status:** implemented (campaign in `sim/campaign.py`; rev 2026-09-30 matches the code at this commit)  
**Companion:** SPEC-01-system-build.md  
**Purpose:** how we simulate bit flips, what we measure, and what we must **not** claim.

---

## 1. Goal

Prove the **system** (SECDED + CRC + range table + input plausibility guard + dual execution + output guard + actuator freshness monitor + FDIR with verified golden reload) behaves as designed under **software-injected** faults, with a clear statement of **limitations** of that injection model.

This is the evaluation story the assignment left open (“radiation robustness… definition is part of the ask”).

---

## 2. Threat model we are approximating

| Real phenomenon (space / soft error) | What we approximate in software |
|--------------------------------------|----------------------------------|
| SEU flipping a bit in SRAM / register | Flip one bit in a chosen memory region of the process |
| Multi-bit upset (MBU) | Flip 2+ bits in one word (`--flip k2:30+31`); adjacency is not modelled |
| Silent data corruption in controller state | Corrupt a `Params` or `State` word, a check byte, the CRC word, a decoded working copy or a sensor sample, without crashing |
| Hang / missed deadline | Stall the control tick past the actuator's freshness limit |

We are **not** approximating: total ionizing dose, latchup, FPGA configuration SEU, bus SEFI, OS/kernel faults, or true radiation transport.

---

## 3. Injection method (software-only)

**Mechanism (built):**

1. The protected build holds `Params` (20 words) and `State` (4 words) in two encoded stores (words + check bytes + CRC and ~CRC); the baseline holds plain structs in RAM.
2. At `--flip-tick` the sim-side **fault harness** (`fsw/src/main.cpp`):
   - Takes the target from the command line: field and bit(s), e.g. `--flip k2:30+31`. There is no random mode; the campaign enumerates its fault lists.
   - XOR-flips the bit(s) **in place** at the start of that tick: after the previous tick's encode and before this tick's decode. The flip stays until something rewrites the word.
3. The next decode runs SECDED → CRC → range table → correct, detect, or miss.

Other injection flags: `--flip-state`, `--flip-check` (a check bit), `--flip-crc` (the CRC word), `--flip-local` (lane 1's decoded copy for one tick, the compute window), `--flip-input` (one sensor sample), `--stall k` (the controller skips k ticks).

**Do not:** rely on undefined behavior, `/proc` self-mem hacks, or debugger breakpoints as the primary story (fragile on Mac review).

**Build:** the injector lives in the sim-side harness; the flight core has no knowledge of it and its object code is identical with or without injection.

---

## 4. Campaign design (what tests we run)

### 4.1 Functional goldens (no faults)

- Sweep non-moving ICs: θ₀ ∈ [−180°, 180°], x₀ ∈ {−1, −0.5, 0, 0.5, 1} m, ω = ẋ = 0 (`grid.json`, `basin.json`).
- Metrics: time-to-upright, peak |u|, max |x|, success/fail.
- Mistuned plant params (±10% cart mass, pole mass, half-length; `mistune.json`) — still succeed within bounds.

### 4.2 Single-bit SEU on protected region

- Every `Params` word (`k0`–`k3`, `ki`, `ilim`, `alpha`, `swke` … `mg`) × 32 bits, through both builds on the same fault list.
- Builds: `baseline`, `protected`.
- Expect:
  - **Protected:** single-bit → corrected by SECDED, continue; control still succeeds.
  - **Baseline:** classify outcome (masked / SDC / DUE).

### 4.3 Double-bit / uncorrectable

- Two bits in one SECDED word → detect, **not** silent correct.
- Expect recovery policy (verified golden reload + bumpless reseed → RECOVERING), not “fake fix.”

### 4.4 Watchdog (actuator-side freshness) and command encoding

- Inject "controller skipped k ticks" in simulated time (never wall-clock); the plant keeps integrating during the stall. The actuator holds the last command for 10 ticks, then applies 0 N; on resume the stall is a detection and the golden is reloaded. Expect the pole to be caught if the stall is shorter than the recovery deadline (≤ 300 ms). `campaign.py --deadline` sweeps k and reports the longest recoverable stall per start angle.
- Compute-window flips: corrupt the decoded local copy from the sim wrapper between decode and `step()` (no hooks inside the core, so the flight object code stays identical with or without injection). Expect: dual-execution mismatch → detected → reload. This is the case memory codes cannot see.
- Command encoding (sequence number + CRC) is not built; it belongs on the F´ port.

### 4.5 Negative control

- The baseline build on the same fault list. The campaign exits non-zero if the baseline shows no failure under single-bit flips (the injector would be broken).

---

## 5. Outcome taxonomy (standard masked / SDC / DUE vocabulary)

Use the standard architectural-vulnerability vocabulary (Mukherjee et al.). Each campaign run is 6 s from θ₀ = 0.05 rad with the fault at tick 100; "pole OK" is the success check (final 2 s inside |θ| < 5°, |x| < 0.1 m, rail never exceeded); "detected" means a non-zero `det` code on or after tick 100.

| Tag | Rule in `sim/campaign.py` |
|-----|----------------------|
| **Masked** | Pole OK, nothing detected |
| **Corrected** | Pole OK, first detection SEC, no reload |
| **Recovered** | Pole OK after any other detection (reload, an input reject held without reload, a stall), or ended DEGRADED, or latched SAFE with the pole still up |
| **SDC** | Pole not OK, nothing detected |
| **DUE** | The binary crashed, or latched SAFE with the pole down |
| **Detected-but-failed** | Detected and reloaded, pole still not OK |

The baseline has no detectors; its stall runs count as recovered only because the harness marks stall ticks `det` = STALL. Report counts + example traces (CSV + short plots).

---

## 6. Limitations of software bit flips (must appear in README + slides)

Be explicit; this is part of the grade story. The README carries the short form under "Where this fails".

1. **Not radiation.** No LET, no flux, no device cross-section. We measure **logic** of detect/correct/recover, not silicon SER.
2. **Spatial correlation.** Real MBUs cluster; our fixed in-word bit pairs are a cartoon unless we add adjacency models.
3. **Timing.** We inject at tick boundaries we choose; real SEUs are async to the CPU pipeline and may hit mid-instruction / caches differently.
4. **Coverage.** Only regions we **mark** can be protected or deliberately hit. Uninstrumented heap/stack/OS remain out of scope.
5. **Compiler/optimizations.** Aggressive optimization might keep state in registers; the protected state is forced through a `volatile` store the harness sees, or results would lie.
6. **Checksum/SECDED ≠ OS or bus integrity.** Separate problem (CRC on links, F´/cFS patterns) — mention, don’t fake.
7. **False confidence.** A green SECDED campaign on a Mac does not qualify flight code; it **demonstrates engineering judgment** for a software-only ask.

Optional one-liner for slides: *“We treat soft errors as a measurable software property: inject → classify → harden → re-measure.”*

---

## 7. Automation & packaging

| Artifact | Role |
|----------|------|
| `fsw/tests/test_controller` | Control law, on-board gains vs scipy, θ wrap, guards, POD layout, swing-up from π |
| `fsw/tests/test_protect` | SECDED (every 1- and 2-bit pattern of 8 sample words, 3-bit data-pattern census), CRC-32C known answer, the encoded store (triple-bit patterns rejected by decode + CRC) |
| `fsw/tests/test_fdir` | FDIR (a)–(o): SEC, DED recovery, DEGRADED law, SAFE latch, state word, untrusted golden, stall, window rollover, plausibility rules, SAFE outputs, mirrored counters |
| `sim/backtest.py` + `sim/scenarios/` | IC grid, basin, mistune, repro hash; stats after every run |
| `sim/campaign.py` | Sets A–H through both builds; `--deadline` stall sweep; exits non-zero on a protected SDC in sets A–E or a baseline with no failure |
| `scripts/run_tests.sh` | Everything above in order, non-zero exit on the first failure; injector sanity (`k2:31` must drop the pole on the baseline and be corrected on the protected build; `k2:30+31` must give DED and a reload) |

**Campaign scope.** The verification campaign ships: exhaustive over all 20 `Params` words × 32 bits in both builds, plus double-bit, check-bit, CRC-word, compute-window, sensor, `State` and stall sets (1,640 runs). A sampled assessment campaign over all controller-owned state with confidence intervals is deferred; two of its ideas cost nothing and are adopted: the same fault list for every build, and a fixed post-injection observation window instead of a fixed run length.

Exit codes: 0 = campaign thresholds met; non-zero = regression (a protected SDC in sets A–E, or a baseline that never fails).

**Mac without Docker:** all of the above via CMake binaries.  
**With Docker:** the same tests inside the pinned image (`docker run --rm cartpole tests`); Docker on the Mac reproduces the `aarch64-linux` golden that the Pi 5 and Jetson produce on metal (same image validated on the Mac; boards run metal).

---

## 8. Acceptance (testing)

- [x] Documented injector algorithm + when it runs in the tick (§3)  
- [x] Limitations section in README (short form of §6)  
- [x] Campaign produces a table: build × set × outcome counts  
- [x] At least one plot or CSV excerpt in the summary pack (`docs/how-it-works.html` §2–§3, from the binaries' CSV)  
- [x] Hardened mode shows fewer SDC than baseline under the same single-bit schedule (0 vs 42 of 640)  
- [x] Double-bit path never “corrects” silently under SECDED (60/60 DED → reload)  

---

## 9. Stretch (explicitly not required)

- Map campaign to F´ unit/integration test on RPi  
- Fault injection into a real EDAC DRAM scrubber  
- Comparing Jetson vs MCU targets  

One paragraph in slides is enough if mentioned at all.
