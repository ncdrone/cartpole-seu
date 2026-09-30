# fsw: cart-pole FSW controller baseline

Small flight-software-style C++14 cart-pole controller plus plant sim. This is the
"baseline" iteration from SPEC-01 (no checksum, SECDED, or fault injection).

## Design
- Primary law: full-state LQR feedback `u = -K [x, xdot, theta, thetadot]`, clamped to +/-20 N.
- Optional integrator on cart position (`KI_X`, default 0) with anti-windup clamp, plus a
  first-order filter on thetadot (`DFILT_ALPHA`, default 1). Both are `State` memory that a
  recovery reseeds after a verified golden reload (`state_reseed()`).
- `step(const Params&, const Input&, State&, Output&)` is pure and re-entrant (no statics), ready
  to wrap as an F Prime passive component. `Params` is the future ProtectedState blob;
  `// TODO(protect)` in `Controller.cpp` marks where it would be verified.
- Non-finite input/param/intermediate: force = safe value (0 N), `Output.fault = 1`.
- Plant: RK4, fixed dt, Florian (2007) equations, frictionless, theta = 0 upright.
- Gains: `tools/compute_lqr_gains.py` (scipy DARE, ZOH, Q=diag(1,1,10,1), R=0.1).

## Constraints obeyed
C++14, `-Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti`; no STL containers, no
dynamic allocation, no recursion, no unbounded loops, no mutable globals; F Prime style
fixed-width typedefs; POD structs asserted trivially copyable in tests. `<cstdio>`/`<cstdlib>`
only in `main.cpp`.

## Build / run
    cmake -B build && cmake --build build
    ./build/test_controller
    ./build/cartpole_demo --theta0 0.2 --x0 0.0 --seconds 3   # CSV: tick,t,x,xdot,theta,thetadot,u,fault

Success criterion used in tests: from theta0=0.2 rad, x0=0.5 m at rest, |theta|<0.05 rad and
|x|<0.2 m at 5 s.

## Not here yet
Checksum, SECDED, golden-copy reload, watchdog, fault injection, mass/length/friction
robustness sweeps, F Prime component wrapper.
