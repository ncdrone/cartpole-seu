/** @file Lqr.hpp  @brief On-board discrete LQR design: linearize, discretize (ZOH), solve the DARE, K = (R+B'PB)^-1 B'PA.
 *
 * Everything is 4x4 / 4x1 fixed arrays in F64, no heap, bounded loops. Call once at init (or on recovery to
 * re-derive gains from golden plant parameters and cost weights). The result is the "optimization" in the binary:
 * K minimizes sum(x'Qx + u'Ru) for the linearized plant at the control rate.
 */
#ifndef FSW_LQR_HPP
#define FSW_LQR_HPP
#include "fsw/FswTypes.hpp"
#include "fsw/Plant.hpp"

namespace fsw {

/** LQR cost weights and rate. POD. */
struct LqrDesign {
    F64 q[4];     /**< diag(Q) on [x, xdot, theta, thetadot] */
    F64 r;        /**< R on u */
    F64 dt;       /**< control period [s] */
};

/** @brief Defaults from Config.hpp (Q = diag(1,1,10,1), R = 0.1, dt = 0.01). */
void lqr_design_default(LqrDesign& d);

/**
 * @brief Solve for the discrete LQR gain of the cart-pole linearized about upright.
 * @param pp        plant parameters (the model the controller believes)
 * @param d         cost weights and period
 * @param k_out     gain, u = -k.[x, xdot, theta, thetadot]
 * @param max_iter  Riccati iteration cap (bounded loop; 2000 is ample at dt = 10 ms)
 * @param tol       convergence: max |P_{k+1} - P_k| below tol
 * @return true on convergence with finite K; false otherwise. The table is the offline cross-check (test_controller f);
 *         the on-board solve is the only runtime source and init refuses to start if it does not converge.
 */
bool lqr_solve(const PlantParams& pp, const LqrDesign& d, F32 k_out[4], U32 max_iter, F64 tol);

/** @brief Continuous-time linearization about upright: xdot = A x + B u. Exposed for tests. */
void cartpole_linearize(const PlantParams& pp, F64 A[4][4], F64 B[4]);

/** @brief Exact ZOH discretization by truncated series (bounded, deterministic). Exposed for tests. */
void zoh_discretize(const F64 A[4][4], const F64 B[4], F64 dt, F64 Ad[4][4], F64 Bd[4]);

} // namespace fsw
#endif
