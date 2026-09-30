# Spec 02 — Soft-fault (software bit-flip) testing

**Status:** draft for implementation  
**Companion:** SPEC-01-system-build.md  
**Purpose:** how we simulate bit flips, what we measure, and what we must **not** claim.

---

## 1. Goal

Prove the **system** (SECDED + CRC + range table + dual execution + output guard + command encoding + actuator watchdog + FDIR with verified golden reload) behaves as designed under **software-injected** faults, with a clear statement of **limitations** of that injection model.

This is the evaluation story the assignment left open (“radiation robustness… definition is part of the ask”).

---

## 2. Threat model we are approximating

| Real phenomenon (space / soft error) | What we approximate in software |
|--------------------------------------|----------------------------------|
| SEU flipping a bit in SRAM / register | Flip one bit in a chosen memory region of the process |
| Multi-bit upset (MBU) | Optionally flip 2+ bits in one word or adjacent words |
| Silent data corruption in controller state | Corrupt gains, I, D, or command without crashing |
| Hang / missed deadline | Stall the control tick past watchdog window |

We are **not** approximating: total ionizing dose, latchup, FPGA configuration SEU, bus SEFI, OS/kernel faults, or true radiation transport.

---

## 3. Injection method (software-only)

**Mechanism (preferred for this assignment):**

1. Control loop owns a `ProtectedState` blob (gains, I, D, meta).
2. At a scheduled tick, the sim-side **fault harness**:
   - Picks target: field + bit index (or random within region).
   - XOR-flips bit(s) **in place** after last “good” encode, or before decode — document which.
3. Next protect-check runs SECDED / checksum → correct, detect, or miss (if we corrupted unprotected memory on purpose).

**Alternative (optional, same API):** mmap / memcpy of a mirror buffer then flip — useful if we want “inject between scrub cycles.”

**Do not:** rely on undefined behavior, `/proc` self-mem hacks, or debugger breakpoints as the primary story (fragile on Mac review).

**Build:** the injector lives in the sim-side harness (`--flip field:bit --flip-tick N`); the flight core has no knowledge of it and its object code is identical with or without injection.

---

## 4. Campaign design (what tests we run)

### 4.1 Functional goldens (no faults)

- Sweep non-moving ICs: θ₀ ∈ [−θ_max, θ_max], x₀ ∈ range, ω=ẋ=0.
- Metrics: time-to-upright, peak |u|, max |x|, success/fail.
- Mistuned plant params (±5–10% m, ℓ, …) — still succeed within bounds.

### 4.2 Single-bit SEU on protected region

- For each critical field (Kp, Ki, Kd, I, d_prev, …): flip each bit (or N random bits per field).
- Modes: `baseline` (expect bad behavior / SDC), `checksum`, `secded`.
- Expect:
  - **SECDED:** single-bit → correct, continue; control still succeeds (or recovers within bound).
  - **Checksum-only:** detect → verified golden reload + bumpless reseed; may have a transient, then recover.
  - **Baseline:** classify outcome (masked / SDC / DUE-like abort).

### 4.3 Double-bit / uncorrectable

- Two bits in one SECDED word → detect, **not** silent correct.
- Expect recovery policy (verified golden reload + bumpless reseed → RECOVERING), not “fake fix.”

### 4.4 Watchdog (actuator-side freshness) and command encoding

- Inject "controller skipped k ticks" in simulated time (never wall-clock); the plant keeps integrating with ZOH on the last command during the stall. Expect: after N stale ticks the actuator applies the fallback, and the pole is still caught if the stall is shorter than the recovery deadline (≤ 300 ms). Sweep k to find the deadline and report it.
- Compute-window flips: corrupt the decoded local copy from the sim wrapper between decode and `step()` (no hooks inside the core, so the flight object code stays identical with or without injection). Expect: dual-execution mismatch → detected → reload. This is the case memory codes cannot see.

### 4.5 Negative / unprotected (limitation demo)

- Inject into **unprotected** stack variable or plant state → show checksum/SECDED do not help; motivates “what we chose to protect.”

---

## 5. Outcome taxonomy (standard masked / SDC / DUE vocabulary)

Use the standard architectural-vulnerability vocabulary (Mukherjee et al.):

| Tag | Meaning in this demo |
|-----|----------------------|
| **Masked** | Fault present but metrics still within success bounds; no recovery event |
| **Corrected** | SECDED fixed a single-bit; run succeeds |
| **Recovered** | Detect (CRC / DED / range / mismatch) → golden reload + reseed; run still meets success after transient |
| **SDC** | Silent wrong control / fails success criteria without detection |
| **DUE-like** | Abort, NaN, assert, or watchdog permanent fail-safe |

Report counts + example traces (CSV + short plots).

---

## 6. Limitations of software bit flips (must appear in README + slides)

Be explicit; this is part of the grade story.

1. **Not radiation.** No LET, no flux, no device cross-section. We measure **logic** of detect/correct/recover, not silicon SER.
2. **Spatial correlation.** Real MBUs cluster; our injector’s “random two bits” is a cartoon unless we add adjacency models.
3. **Timing.** We inject at tick boundaries we choose; real SEUs are async to the CPU pipeline and may hit mid-instruction / caches differently.
4. **Coverage.** Only regions we **mark** can be protected or deliberately hit. Uninstrumented heap/stack/OS remain out of scope.
5. **Compiler/optimizations.** Aggressive optimization might keep state in registers; we must force protected state through the blob the harness sees (volatile / explicit memory barriers as needed) or results lie.
6. **Checksum/SECDED ≠ OS or bus integrity.** Separate problem (CRC on links, F´/cFS patterns) — mention, don’t fake.
7. **False confidence.** A green SECDED campaign on a Mac does not qualify flight code; it **demonstrates engineering judgment** for a software-only ask.

Optional one-liner for slides: *“We treat soft errors as a measurable software property: inject → classify → harden → re-measure.”*

---

## 7. Automation & packaging

| Artifact | Role |
|----------|------|
| `fsw/tests/test_controller` | Control law, on-board gains vs scipy, θ wrap, guards, POD layout |
| `fsw/tests/test_protect` (planned with the protected build) | Exhaustive codec; single/double bit through the pipeline; range table; golden reload; FDIR transitions |
| `sim/backtest.py` + `sim/scenarios/` | IC grid, basin, mistune, repro hash; stats after every run |
| `scripts/run_tests.sh` | Everything above in order, non-zero exit on the first failure; injector sanity (the unprotected build must fail a sign flip) |

**Campaign scope.** The Verification campaign (exhaustive over the 9 protected words, the double-bit set, the negative control) ships. A sampled Assessment campaign over all controller-owned state with confidence intervals is deferred; two of its ideas cost nothing and are adopted: the same fault list for every build, and a fixed post-injection observation window instead of a fixed run length.

Exit codes: 0 = campaign thresholds met; non-zero = regression (e.g. SDC rate above budget in hardened mode).

**Mac without Docker:** all of the above via CMake binaries.  
**With Docker:** the same tests inside the pinned image (`docker run --rm cartpole tests`); same image on any arm64 host reproduces the result hash.

---

## 8. Acceptance (testing)

- [ ] Documented injector algorithm + when it runs in the tick  
- [ ] Limitations section in README (copy from §6, shortened)  
- [ ] Campaign produces a table: mode × outcome counts  
- [ ] At least one plot or CSV excerpt in the summary pack  
- [ ] Hardened mode shows fewer SDC than baseline under the same single-bit schedule  
- [ ] Double-bit path never “corrects” silently under SECDED  

---

## 9. Stretch (explicitly not required)

- Map campaign to F´ unit/integration test on RPi  
- Fault injection into a real EDAC DRAM scrubber  
- Comparing Jetson vs MCU targets  

One paragraph in slides is enough if mentioned at all.

---
*Cross-verified and adjusted against a Fable 5.1 agent review (research/06-spec-complexity-review.md), 2026-09-29.*
