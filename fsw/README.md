# fsw: flight core and harness

C++14 flight core plus the sim-side harness. One source, two builds: `cartpole_demo` (protected, `FSW_PROTECT=1`) and `cartpole_baseline` (no protection, `FSW_PROTECT=0`). Start from the top-level [README](../README.md); build and test through `scripts/build_demo.sh` and `scripts/run_tests.sh`.

**Files.** `Controller` pure `step()` (energy swing-up, hysteresis handoff to LQR), the 80-byte `Params` (20 × F32) and 16-byte `State`, the range table · `Lqr` on-board Riccati solve at init (`tools/compute_lqr_gains.py` is the scipy cross-check) · `Protect` Hsiao SECDED(39,32) from a `static const` column table, CRC-32C, the encoded store · `Runtime` the protected tick and FDIR · `Math.hpp` angle wrap and clamp · `Config.hpp` every constant · `Plant` RK4 cart-pole, Florian (2005) equations, frictionless (sim side, still linked into `libfsw`) · `main.cpp` harness, actuator monitor and injector (sim side).

**One protected tick** (`runtime_decode` then `runtime_control`): decode every stored word (SECDED corrects 1 bit and scrubs, flags 2) → CRC-32C over the corrected plaintext against its (v, ~v) copy → range table → input plausibility guard → `step()` twice, compare → output guard from constants → DEGRADED law → FDIR bookkeeping → re-encode `State`. The baseline reads plain `Params`/`State` and calls `step()` once.

**Rules and how each is enforced.**
- No exceptions or RTTI, warnings are errors: `-fno-exceptions -fno-rtti -Wall -Wextra -Wpedantic -Werror` in `CMakeLists.txt`. No heap, recursion or unbounded loops: by construction (fixed arrays, loops bounded by constants such as `LQR_MAX_ITER`), checked in review.
- No fast-math, no FMA contraction: `-ffp-contract=off`, and `FswTypes.hpp` fails the build if fast-math is on.
- No mutable globals in the core: its only static data are `const` tables and constants.
- The output clamp (20 N) and the fault-time force (0 N) are `cfg::` constants, never `Params` words.
- The golden `Params` is rebuilt from the nominal model by the on-board solve and used only if its CRC-32C equals the value pinned at init; otherwise SAFE.
- FDIR state, detection count, window start and clean-tick count are stored as (v, ~v); a mismatch reads as SAFE. SAFE has no exit.
- The injector is a runtime flag of `main.cpp`; the flight object code is the same with or without it.
- POD structs: sizes by `static_assert` in `Controller.hpp`, trivially copyable by `static_assert` in `test_controller`.

**Build and test** (CMake 3.13 or newer):

    cmake -S . -B build && cmake --build build
    ./build/test_controller && ./build/test_protect && ./build/test_fdir     # or: cd build && ctest
    ./build/cartpole_demo --theta0 0.2 --x0 0.5 --seconds 3   # CSV: tick,t,x,xdot,theta,thetadot,u,fault,u_x,u_xd,u_th,u_thd,u_i,integ,sat,mode,det,fdir,sec,reload
