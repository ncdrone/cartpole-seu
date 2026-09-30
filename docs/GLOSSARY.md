# Glossary: terms and acronyms used in this project

**Scope:** every acronym and term of art that appears in `docs/`, `research/`, `fsw/` and `sim/`. Grouped by topic. Where a term names a specific design decision in this project, the entry says what we do with it.

---

## 1. Radiation and soft errors

| Term | Expansion | Meaning here |
|---|---|---|
| **SEU** | Single-Event Upset | One bit flips in memory or a register because a charged particle deposited energy. The fault model this project approximates in software. Not permanent; the next write clears it. |
| **MBU** | Multi-Bit Upset | One particle flips two or more bits, usually physically adjacent. Defeats SECDED if both bits are in the same word. Called out as a limitation; not modelled. |
| **SEFI** | Single-Event Functional Interrupt | An upset that hangs or wedges a device or task (stuck state machine, lost tick). We approximate it with a "skipped k ticks" injection. |
| **SET** | Single-Event Transient | A short voltage glitch in combinational logic. Out of scope for software. |
| **SEL** | Single-Event Latch-up | A parasitic short that can destroy a part; needs a power cycle. Hardware problem; out of scope. |
| **TID** | Total Ionizing Dose | Slow cumulative damage that drifts analog parameters and clocks over months. Not modelled; mentioned as a reason plant parameters drift. |
| **Soft error** | | A transient, non-destructive error (SEU, SET). "Soft-fault" in our spec titles means the same thing. |
| **Hard error** | | Permanent damage (TID, SEL, stuck bit). |
| **LET** | Linear Energy Transfer | Energy a particle deposits per unit path length; the x-axis of a device's upset cross-section curve. Only relevant to beam testing, which we do not do. |
| **SER** | Soft Error Rate | Upsets per bit per unit time for real silicon. A software injector cannot measure it; the report says so explicitly. |
| **Cross-section** | | Upset probability per particle fluence. Beam-test quantity. Not measured here. |
| **LEO** | Low Earth Orbit | Below roughly 2,000 km. Report 02 uses a measured LEO rate of 4.76e-7 upsets/bit/day (JASON-2) for a back-of-envelope on golden-copy latency. |
| **COTS** | Commercial Off-The-Shelf | Non-radiation-hardened parts. Software fault tolerance matters most on COTS. |
| **RHBD** | Radiation-Hardened By Design | Circuit-level hardening. The alternative to software mitigation; out of scope. |
| **Rad-hard / rad-tolerant** | | Hardened silicon / tolerant-enough silicon. The specs say "in flight, params would live in rad-hard or EDAC memory". |
| **Scrubbing** | | Periodically reading and rewriting protected memory so single-bit errors are corrected before a second one lands in the same word. Report 02 gives the cadence math. |
| **Latent fault** | | A corruption that has happened but has not yet been read, so nothing has detected it. The golden-copy-in-RAM problem. |
| **ARGOS** | Advanced Research and Global Observation Satellite | 1999 experiment that flew software-implemented EDAC on COTS processors. The citation for "software SECDED is a real flight technique". |
| **Beam test** | | Exposing hardware to a particle accelerator to measure upset rates. What we are *not* doing. |

## 2. Error detection and correction

| Term | Expansion | Meaning here |
|---|---|---|
| **EDAC** | Error Detection And Correction | Umbrella term for hardware or software that detects and fixes memory errors. |
| **ECC** | Error-Correcting Code | Any code that can correct errors, e.g. Hamming, Hsiao, Reed-Solomon. Often used loosely for "ECC RAM". |
| **SEC** | Single Error Correction | The code can fix one flipped bit per word. |
| **DED** | Double Error Detection | The code can notice, but not fix, two flipped bits per word. |
| **SECDED** | Single Error Correction, Double Error Detection | The standard EDAC flavour. One bit: fix silently and count. Two bits: flag and refuse to trust the word. Our iteration-3 protection on each 32-bit float of `Params`/`State`. |
| **Hamming code** | | The classic SEC code. Extended Hamming adds one parity bit for DED. Hamming(39,32) = 32 data bits + 7 check bits. |
| **Hsiao code** | | A SECDED code with odd-weight columns; same size as extended Hamming but fewer miscorrections on triple-bit errors and cheaper hardware. Report 02 recommends Hsiao(39,32) using the OpenTitan encoder masks. |
| **(39,32) / (72,64)** | | Code notation: (total bits, data bits). (39,32) protects a 32-bit word; (72,64) protects a 64-bit word. We use 32-bit floats, so (39,32). |
| **Syndrome** | | The result of re-encoding the data and XOR-ing with the stored check bits. Zero means no error; a non-zero value points at the flipped bit. |
| **Miscorrection** | | The decoder "fixes" the wrong bit because the error pattern (3+ bits) aliases to a single-bit syndrome. Why a CRC should follow SECDED. |
| **Hamming distance (HD)** | | Minimum number of bit flips needed to turn one valid codeword into another. HD=4 for SECDED. For a CRC, "HD=6 up to N bits" means every error of 5 or fewer bits in a block of N bits is detected. Also used for choosing far-apart enum values (e.g. `0x5A5A5A5A` vs `0xA5A5A5A5`). |
| **Checksum** | | A cheap integrity value over a block. Detects, never corrects. |
| **CRC** | Cyclic Redundancy Check | A checksum based on polynomial division. Strong detection guarantees for a given block length. |
| **CRC-32C** | CRC-32 Castagnoli | The CRC-32 variant with the best Hamming distance at our block size (HD=6 at 416 bits), with a hardware instruction on ARMv8 and x86. Report 02's recommendation over CRC-32/IEEE and Fletcher. |
| **Fletcher-32 / Adler-32** | | Simple additive checksums. Faster to describe than CRC but weaker (HD=3) and, with modern CPUs, not faster to run. Rejected. |
| **TMR** | Triple Modular Redundancy | Keep three copies and majority-vote on read. Corrects anything confined to one copy, including in-word MBUs, at 3× memory. The software-native alternative to SECDED for a 52-byte blob. |
| **DWC / DMR** | Duplication With Compare / Dual Modular Redundancy | Two copies or two executions, compared; a mismatch is detected but cannot be resolved without a third. What "dual-execute the control step" means. |
| **Golden copy** | | A reference copy of parameters used to reload after corruption is detected. Must itself be verified (CRC) before use. In this code the golden source is `params_default()`, built from compile-time literals. |
| **Reed-Solomon (RS)** | | A block code strong against bursts. Overkill for a per-tick 52-byte struct; suited to flash images. Documented, not built. |
| **Parity** | | One bit recording whether the number of 1s is even or odd. Detects a single flip; the building block of Hamming codes. |
| **Write-back** | | After a SEC correction, writing the corrected word back to storage so the error does not get "corrected" again every tick. |

## 3. Software fault tolerance (the wider literature)

| Term | Expansion | Meaning here |
|---|---|---|
| **SIHFT** | Software-Implemented Hardware Fault Tolerance | The research field: tolerating hardware faults with software only. Our whole second requirement. |
| **EDDI** | Error Detection by Duplicated Instructions | Compiler duplicates every instruction and compares before stores. Precedent for "compare before it becomes state". |
| **SWIFT** | Software Implemented Fault Tolerance | Refinement of EDDI that also checks control flow. |
| **CFCSS** | Control-Flow Checking by Software Signatures | Each basic block carries a signature; a wrong jump is detected. Literature finds poor return on investment; we document but do not build it. |
| **COAST** | Compiler-Assisted Software fault Tolerance | BYU's LLVM passes that automate DWC/TMR. Cited as the "what I'd do with a week" comparison. |
| **ABFT** | Algorithm-Based Fault Tolerance | Checks built into the maths itself (e.g. checksummed matrices). Mentioned for linear-algebra-heavy controllers; not used. |
| **N-version programming** | | Independent implementations voted against each other. Out of scope. |
| **Recovery block** | | Try the primary; if its acceptance test fails, run the alternate. Related to the degradation ladder. |
| **Simplex architecture** | | A complex high-performance controller guarded by a simple verified fallback and a decision module. Our "iLQR/swing-up → LQR → safe" ladder. |
| **Plausibility / analytic-redundancy check** | | Rejecting a sensor value that violates physics, e.g. \|Δθ − θ̇·dt\| too large. Independent of ECC. |
| **Control-flow error** | | The program jumps somewhere it should not (corrupted PC or branch). Not covered by data ECC. |
| **Silent window / window of vulnerability** | | The time between decoding a protected value and writing it back, during which a flip in a register or local is invisible to memory ECC. Report 03's "compute window". |
| **Laundering** | | (Report 03's term) A corrupted working value gets re-encoded with valid check bits, so the corruption is now "verified". |
| **Register promotion** | | The optimizer keeps a variable in a CPU register instead of memory. Report 02 verified this removes the "protected blob" from RAM at -O2 unless storage is `volatile`. |
| **`volatile`** | | C/C++ qualifier that forces every read and write to go to memory. Used on the encoded storage words so the checker reads what an SEU would hit. |
| **`-ffast-math` / `-ffinite-math-only`** | | Compiler flags that assume no NaN/Inf exist and may delete `isfinite` checks. Forbidden in this project. |
| **FMA / `-ffp-contract`** | Fused Multiply-Add | A single instruction computing a·b+c with one rounding. Clang enables contraction by default on Apple Silicon, so trajectories differ from x86 in the last bits. We build with `-ffp-contract=off`. |
| **NaN / Inf** | Not a Number / Infinity | IEEE-754 special values. A bit flip in the exponent can produce them; `isfinite` catches those but not the far more common "finite but wrong" flips. |
| **Subnormal (denormal)** | | Tiny floats below the normal range. Slow on some x86 parts; a flipped exponent can create one. |

## 4. Fault injection and measurement

| Term | Expansion | Meaning here |
|---|---|---|
| **Fault injection** | | Deliberately corrupting state to observe the system's response. Our injector XORs bits in the protected blob (and, per report 03, should also hit working values and inputs). |
| **Campaign** | | A batch of injection runs with a defined fault space, sampling scheme and success predicate. |
| **Verification campaign** | | Exhaustive injection into the protected words to prove the codec: 100% correct/detect expected. Can gate CI. |
| **Assessment campaign** | | Sampled injection across all controller-owned state to estimate how much robustness the system gained. Reported with confidence intervals; never gated. |
| **Fault space** | | The set of (location, bit, time) an injection can target. Must be stated for any rate to mean anything. |
| **Masked** | | Fault happened, nothing detected it, and the outcome still met success bounds (feedback absorbed it, or the bit was never read). |
| **Corrected** | | SECDED fixed a single-bit error and the run succeeded. |
| **Recovered** | | Detected (checksum or DED), recovery action fired (reload/reset), run still met success after a transient. |
| **SDC** | Silent Data Corruption | Wrong result with no detection. The worst outcome. |
| **DUE** | Detected Unrecoverable Error | Detected but could not continue: abort, NaN, latched safe state. "DUE-like" in our taxonomy. |
| **AVF** | Architectural Vulnerability Factor | Fraction of time a bit matters to the final result. The idea behind weighting a campaign by exposure, not just by bit count. |
| **Dead bit** | | A bit that is never read or is overwritten before use; injecting into it always looks "masked". Report 03 counted roughly half of our candidate bits as dead under default config. |
| **Activated** | | The corrupted value was actually read after injection. Only activated faults should count in rates. |
| **Latent (at end of run)** | | Corruption still present when the run ended without having caused failure. Distinct from masked. |
| **Negative control** | | Injecting where protection does not reach, to show the mechanism is not magic. |
| **Oracle** | | The thing that decides pass/fail. Must sit outside the fault domain: compare against a fault-free twin run, not the device's own counters. |
| **Fault-free twin** | | The same initial condition and seed run with no injection; the reference trajectory. |
| **Descriptor** | | The file that fully specifies a campaign (git SHA, compiler, flags, fault model, RNG seed, N) so it can be replayed. |
| **Wilson interval** | | A confidence interval for a proportion that behaves well at small counts. Default choice for reported rates. |
| **Clopper-Pearson** | | The conservative exact interval for a proportion. |
| **Rule of three** | | Zero failures in n trials gives an upper 95% bound of about 3/n. Showing "<1%" needs n ≥ 299. |
| **McNemar test** | | The test for comparing two modes that were run on the *same* fault list (paired design). |
| **Paired design** | | Every mode sees the identical fault list, so differences are due to the mode, not the sample. |
| **Exposure weighting** | | Weighting fault sampling by how long each value is live and read, not uniformly over bits. |
| **PRNG / splitmix64 / xoshiro** | Pseudo-Random Number Generator | Deterministic random sources. We use an in-repo one because `std::` distributions differ between libc++ and libstdc++. |
| **Seed** | | The PRNG starting value; needed for replay. |
| **Replay** | | Re-running one injection row exactly from its descriptor. |
| **IronFrame** | | Dan's prior SEU-injection project on edge-GPU inference. Source of the inject → classify → harden → re-measure discipline. |

## 5. Flight software and fault management

| Term | Expansion | Meaning here |
|---|---|---|
| **FSW** | Flight Software | Software that runs on the spacecraft. The `fsw/` directory holds the flight-style core. |
| **F´ / F Prime** | | NASA JPL's component-based flight software framework. Orca's bus FSW today. Our controller is shaped to become an F´ component. |
| **cFS / cFE** | core Flight System / core Flight Executive | NASA Goddard's flight software framework and its executive layer. Orca's likely future framework. |
| **FPP** | F Prime Prime | F´'s component modelling language. Report 04 includes an FPP sketch. |
| **Component (F´)** | | A unit with typed ports. *Passive* runs on the caller's thread, *active* has its own thread, *queued* has a queue drained by a caller. |
| **Port (F´)** | | A typed connection between components. |
| **Rate group** | | An F´ scheduler pattern: components invoked at a fixed rate (e.g. 100 Hz). Where the control loop would run. |
| **PrmDb** | Parameter Database (F´ `Svc::PrmDb`) | Loads parameters from a CRC-checked file. The flight home for golden gains. |
| **Svc::Health** | | F´ component that pings other components and strokes the hardware watchdog only while all reply. |
| **HS / LC / CS** | Health & Safety / Limit Checker / Checksum (cFS apps) | HS monitors app execution and services the watchdog; LC evaluates limits with persistence counts; CS background-checksums memory and code. Precedents cited in reports 02 and 04. |
| **FM** | Fault Management | The discipline of detecting, isolating and responding to faults. NASA-HDBK-1002 is the handbook. |
| **FDIR** | Fault Detection, Isolation and Recovery | The runtime state machine that acts on faults. Ours: NOMINAL → RECOVERING → DEGRADED → SAFE. DEGRADED = swing-up inhibited, output clamp halved. |
| **Persistence (three-strike)** | | Requiring a fault indication to repeat N times before acting, so noise or a single SEU in the detector does not trigger a response. |
| **Escalation** | | Stronger responses on repeated faults: retry → degrade → safe mode → reset. |
| **Hysteresis** | | Requiring a different (stricter) condition to leave a state than to enter it, so the system does not chatter. |
| **Safe mode / safe state** | | A configuration that is safe regardless of the fault. For an inverted pendulum there is no static safe command, which is a key finding. |
| **TTC** | Time To Criticality | How long after a fault before the situation becomes unrecoverable. 0.3–1.3 s for our pole. Mitigations must act faster than this. |
| **Latched** | | A state that stays until explicitly cleared by command. |
| **RTS** | Relative Time Sequence (cFS) | A stored command sequence LC can trigger on a limit violation. |
| **Watchdog (WDT)** | | A timer that resets the system if not "petted" in time. Hardware in flight; simulated here as an actuator-side freshness check in ticks. |
| **Deadline monitor** | | Report 04's honest name for the in-process check that notices a missed tick after the fact. |
| **Cycle slip** | | F´'s term for a rate-group overrun. |
| **Heartbeat / stroke / pet** | | The periodic signal that keeps a watchdog from firing. |
| **ZOH** | Zero-Order Hold | Holding the last command constant between updates. Used for discretization and for "what the actuator does during a stall". |
| **Fail-operational / fail-safe** | | Keep working vs. stop safely. This loop must be fail-operational because stopping drops the pole. |
| **Single point of failure** | | One unprotected word (the mode word, a counter) whose corruption defeats everything. |
| **Command freshness / sequence number** | | Stamping each command so the actuator can reject stale ones. |
| **JPL P10 (Power of Ten)** | | Holzmann's ten rules for safety-critical C: bounded loops, no recursion, no heap after init, check every return, etc. |
| **JPL Institutional Coding Standard** | | The fuller JPL C standard; Rule 16 (sanity checks trigger recovery) is cited. |
| **MISRA / AUTOSAR C++** | | Industry coding standards for safety-critical C/C++. |
| **NPR 7150.2** | NASA Procedural Requirements for software | SWE-134 requires detecting memory modification and recovering to a known safe state. |
| **NASA-HDBK-1002** | NASA Fault Management Handbook | The reference for detection sub-functions, persistence and TTC. |
| **ECSS** | European Cooperation for Space Standardization | European equivalents; ECSS-Q-HB-60-02A covers radiation mitigation techniques. |
| **WCET** | Worst-Case Execution Time | Upper bound on how long a tick takes. Bounded loops and no heap make it computable. LQR has O(1) WCET; an online optimizer needs an iteration cap. |
| **POD** | Plain Old Data | A struct with no constructors, virtuals or pointers, so it can be copied and checksummed byte-wise. All our controller structs. |
| **Trivially copyable** | | The C++ property that makes a struct safe to `memcpy`. Checked with `static_assert`. |
| **Padding** | | Bytes the compiler inserts for alignment. Made explicit (`pad[3]`) so checksums are deterministic. |
| **ICD** | Interface Control Document | Specification of the sensor/actuator boundary: units, sign conventions, validity bits. Missing; report 04 asks for one. |
| **Requirements trace** | | A table mapping each requirement to its verification and evidence. Report 05's R1–R6 table. |
| **OBC** | On-Board Computer | The spacecraft's main computer. |
| **C&DH** | Command and Data Handling | The bus subsystem that runs the OBC and FSW. |
| **GNC** | Guidance, Navigation and Control | Jake's background. GN&C is the same thing. |
| **ADCS** | Attitude Determination and Control System | The real-world analogue of this control loop. |
| **HIL / SIL** | Hardware-in-the-Loop / Software-in-the-Loop | Testing with real hardware or with the flight software against a simulated plant. Our sim is SIL. |
| **Companion computer** | | A non-flight-critical computer (e.g. Jetson) that proposes; the bus FSW disposes. |
| **Lockstep / dual-lane** | | Two processors executing the same instructions and comparing (Dragon's approach). |
| **CAN / RS-422 / SpaceWire / MIL-STD-1553** | | Spacecraft data buses. Each has its own CRC/parity; mentioned as out of scope. |
| **LUT** | Lookup Table | A gain-schedule table. In the spec as "would live in rad-hard memory"; report 01 recommends dropping it. |
| **Telemetry (TLM)** | | Data downlinked for monitoring. Correction counts and FDIR transitions should be telemetry. |
| **Event (EVR)** | Event Report | A logged discrete occurrence with a reason and timestamp. |
| **Throttled event** | | An event limited to N per window so a stuck fault does not flood the log. |

## 6. Control theory

| Term | Expansion | Meaning here |
|---|---|---|
| **Cart-pole / inverted pendulum** | | The plant: a cart on a rail with a pole hinged on top; force on the cart is the only input. |
| **Upright / hanging-down** | | θ = 0 (unstable equilibrium) / θ = π (stable equilibrium). |
| **State** | | [x, ẋ, θ, θ̇]: cart position, cart velocity, pole angle from upright, pole angular rate. |
| **Non-moving initial condition** | | ẋ = θ̇ = 0 at t = 0, any x and θ. Jake's requirement. Includes hanging down. |
| **Equilibrium** | | A state where the derivative is zero. Upright and hanging-down are the two. |
| **Basin of attraction** | | The set of initial states from which the controller reaches the goal. Ours is |θ₀| ≤ 43° at 20 N, measured by `sim/backtest.py`. |
| **Swing-up** | | Pumping energy into a hanging pendulum until it reaches upright. Needed for θ₀ beyond the basin. |
| **Energy shaping** | | A swing-up law (Åström–Furuta) that drives the pendulum's energy toward the upright energy. Lyapunov-based, not optimization-based. |
| **Catch / handoff** | | Switching from swing-up to the balance controller near upright, with hysteresis. |
| **PID** | Proportional-Integral-Derivative | The classical single-loop law. The specs' original plan; report 01 shows θ-only PID cannot regulate cart position. |
| **P / I / D terms** | | Contributions proportional to error, its integral, and its derivative. In the renderer, "P" = x·Kx + θ·Kθ, "D" = ẋ·Kẋ + θ̇·Kθ̇, "I" = ∫x·Ki. |
| **Full-state feedback** | | u = −K·state, using all four states. What the baseline does. |
| **LQR** | Linear-Quadratic Regulator | Optimal full-state feedback for a linear model with a quadratic cost. Solved via the Riccati equation. Our balance controller. |
| **LQI** | LQR with Integral action | LQR on a state augmented with ∫x. How to add an integrator properly. |
| **LQG** | Linear-Quadratic-Gaussian | LQR plus a Kalman estimator. Has no guaranteed margins (Doyle 1978). |
| **DARE** | Discrete Algebraic Riccati Equation | The matrix equation whose solution gives the LQR gain. Currently solved offline in scipy; report 01 says solve it in C++ at init. |
| **Q / R** | | LQR cost weights on state and input. Ours: Q = diag(1, 1, 10, 1), R = 0.1. |
| **Gain vector K** | | The four numbers in u = −K·state. Ours: [−2.96, −5.56, −47.37, −12.28]. |
| **Gain margin / phase margin** | | How much the loop gain or phase can change before instability. Discrete LQR here has input-gain margin [0.375, 15.95]. |
| **MPC / NMPC** | (Nonlinear) Model Predictive Control | Re-solving a finite-horizon optimal control problem every tick, with constraints. The "optimization-based" answer a controls reviewer expects. |
| **iLQR / DDP** | iterative LQR / Differential Dynamic Programming | Trajectory optimizers for nonlinear systems; the usual way to do optimization-based swing-up. |
| **Receding horizon** | | Solve over the next N steps, apply the first move, repeat. |
| **Warm start** | | Seeding an optimizer with last tick's solution. Becomes SEU-sensitive state. |
| **Terminal cost** | | The cost on the final state of a finite horizon; using the DARE solution makes unconstrained MPC identical to LQR. |
| **QP** | Quadratic Program | The optimization a constrained linear MPC solves each tick. |
| **Linearization** | | Approximating the nonlinear plant by a linear model about an equilibrium. LQR is designed on the linearization about upright. |
| **Discretization** | | Converting continuous dynamics to a fixed-step update. We use ZOH at dt = 0.01 s for the design and RK4 for the plant. |
| **RK4** | 4th-order Runge-Kutta | The integrator used for the simulated plant. Explicit Euler is avoided because it adds energy to a pendulum. |
| **Semi-implicit Euler** | | A cheaper integrator that conserves energy better than explicit Euler. Mentioned in SPEC-01 as an option. |
| **Non-minimum-phase** | | A system that initially moves the "wrong" way (the cart must move away from centre to tip the pole back). Why cart-position control on a cart-pole is finicky. |
| **RHP zero / RHP pole** | Right-Half-Plane | Unstable pole (≈3.97 rad/s here) and the "wrong-way" zero (≈3.8 rad/s). They are close together, which limits cascaded PID. |
| **Bandwidth** | | How fast a loop responds. Must sit between the RHP zero and pole constraints. |
| **Anti-windup** | | Limiting the integrator so saturation does not leave it holding a huge value. |
| **Derivative kick** | | A spike in output when the derivative term sees a step (e.g. after zeroing its memory). Why "reset D memory" is harmful. |
| **Bumpless transfer** | | Switching or reseeding a controller so the output does not jump. Our filter reseeds from the current measurement. |
| **Saturation / clamp** | | Limiting the command to ±20 N. |
| **Track / rail limit** | | ±2.4 m. A run that leaves it is a TRACK_EXIT in the back-test. |
| **Settling time** | | Time until the state stays inside the success band (|θ| < 5°, |x| < 0.1 m) for the rest of the run. |
| **Limit cycle / hunting** | | Sustained oscillation, e.g. from Coulomb friction with no integral action. |
| **Coulomb friction** | | Constant-magnitude friction opposing motion. Not in our plant; report 01 shows it causes hunting. |
| **Florian equations** | | Florian (2005), "Correct equations for the dynamics of the cart-pole system". Our plant model. |
| **Plant** | | The physical system being controlled (the cart-pole sim). |
| **Mistune** | | Running the controller designed for nominal parameters on a plant with different mass or length. |
| **Monte Carlo** | | Randomly sampling parameters or starts and reporting success rates. |
| **Wrap (angle)** | | Mapping θ to (−π, π]. The baseline does not do this yet. |
| **Observer / estimator** | | Software that reconstructs unmeasured states (e.g. velocities) from sensors. Assumed perfect here. |

## 7. Project-specific names

| Term | Meaning |
|---|---|
| **SPEC-01 / SPEC-02** | The build spec and the soft-fault testing spec in `docs/`. |
| **Iteration 1 / 2 / 3** or **baseline / checksum / secded** | The staged demo modes in SPEC-01. Research recommends re-staging by fault class (none / data / data+compute / full). |
| **`Params`** | The POD struct of gains and limits; the "protected blob". |
| **`State`** | The POD struct of mutable controller memory (integrator, filtered θ̇, tick). |
| **`step()`** | The pure control-law function `step(const Params&, const Input&, State&, Output&)`. |
| **`params_default()`** | Builds `Params` from compile-time constants; the golden source. |
| **`state_reseed()`** | Zeroes the integrator and re-seeds the θ̇ filter from the next measurement; used after a verified golden reload, never as a recovery by itself. |
| **`sim/render.py`** | Replay viewer: reads the demo CSV, animates cart, pole, traces and force terms. |
| **`sim/backtest.py`** | Scenario runner: cartesian sweeps through the C++ binary with stats and basin maps. |
| **`sim/scenarios/*.json`** | Parameter sets: `basin`, `grid`, `mistune`, `rate`. |
| **OK / TRACK_EXIT / FELL / NOT_SETTLED / FAULT / NAN** | Back-test outcome classes. |
| **Orca** | The company (AI spacecraft operator; F´ bus FSW). |
| **Jake** | Jacob Ososke, Orca CEO, former Lockheed GNC; the reviewer. |
