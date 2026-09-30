/** @file Lqr.cpp  @brief On-board discrete LQR design (see Lqr.hpp). */
#include "fsw/Lqr.hpp"
#include <cmath>

namespace fsw {

void lqr_design_default(LqrDesign& d) {
    d.q[0] = cfg::LQR_Q_X; d.q[1] = cfg::LQR_Q_XDOT; d.q[2] = cfg::LQR_Q_THETA; d.q[3] = cfg::LQR_Q_THDOT;
    d.r = cfg::LQR_R;
    d.dt = cfg::DT_S;
}

/* Florian (2005) cart-pole, theta from upright, frictionless:
 *   thdd = (g sin th - cos th * tmp) / (l (4/3 - m cos^2 th / M_t)),  tmp = (u + m l thd^2 sin th) / M_t
 *   xdd  = tmp - m l thdd cos th / M_t
 * Linearized at th = 0, thd = 0, u = 0 (sin th ~ th, cos th ~ 1, thd^2 ~ 0):
 *   thdd = (g th - u / M_t) / D,  D = l (4/3 - m / M_t)
 *   xdd  = u / M_t - (m l / M_t) thdd
 */
void cartpole_linearize(const PlantParams& pp, F64 A[4][4], F64 B[4]) {
    const F64 Mt = pp.cart_mass + pp.pole_mass;
    const F64 D  = pp.half_len * (4.0 / 3.0 - pp.pole_mass / Mt);
    const F64 a_th_th = pp.gravity / D;            // d(thdd)/d(th)
    const F64 b_th    = -1.0 / (Mt * D);            // d(thdd)/d(u)
    const F64 c       = pp.pole_mass * pp.half_len / Mt;
    for (U32 i = 0; i < 4U; ++i) { B[i] = 0.0; for (U32 j = 0; j < 4U; ++j) A[i][j] = 0.0; }
    A[0][1] = 1.0;                       // x' = xdot
    A[1][2] = -c * a_th_th;              // xdd = -(m l / Mt) thdd
    A[2][3] = 1.0;                       // th' = thdot
    A[3][2] = a_th_th;                   // thdd
    B[1] = 1.0 / Mt - c * b_th;          // xdd from u
    B[3] = b_th;                         // thdd from u
}

static void mat_mul(const F64 X[4][4], const F64 Y[4][4], F64 Z[4][4]) {
    for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) {
        F64 s = 0.0; for (U32 k = 0; k < 4U; ++k) s += X[i][k] * Y[k][j]; Z[i][j] = s;
    }
}

/* Ad = exp(A dt) = sum_{n>=0} (A dt)^n / n!,  Bd = (sum_{n>=1} A^{n-1} dt^n / n!) B.  Truncated at N terms:
 * at dt = 10 ms and |A| ~ 15 rad/s^2 the terms fall below 1e-18 well before N = 20. Bounded loop, no heap. */
void zoh_discretize(const F64 A[4][4], const F64 B[4], F64 dt, F64 Ad[4][4], F64 Bd[4]) {
    static const U32 N_TERMS = 24U;
    F64 term[4][4], next[4][4], S[4][4];   // term = (A dt)^n / n!,  S = sum_{n>=1} A^{n-1} dt^n / n!
    for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) {
        term[i][j] = (i == j) ? 1.0 : 0.0; Ad[i][j] = term[i][j]; S[i][j] = (i == j) ? dt : 0.0;
    }
    F64 Adt[4][4];
    for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) Adt[i][j] = A[i][j] * dt;
    F64 fac = 1.0;
    for (U32 n = 1; n < N_TERMS; ++n) {
        mat_mul(term, Adt, next);                             // (A dt)^n / (n-1)!
        fac = static_cast<F64>(n);
        for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) {
            term[i][j] = next[i][j] / fac;                    // (A dt)^n / n!
            Ad[i][j] += term[i][j];
            S[i][j]  += term[i][j] * dt / static_cast<F64>(n + 1);   // A^n dt^{n+1} / (n+1)!
        }
    }
    for (U32 i = 0; i < 4U; ++i) { F64 s = 0.0; for (U32 k = 0; k < 4U; ++k) s += S[i][k] * B[k]; Bd[i] = s; }
}

bool lqr_solve(const PlantParams& pp, const LqrDesign& d, F32 k_out[4], U32 max_iter, F64 tol) {
    F64 A[4][4], B[4], Ad[4][4], Bd[4];
    cartpole_linearize(pp, A, B);
    zoh_discretize(A, B, d.dt, Ad, Bd);

    F64 P[4][4];
    for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) P[i][j] = (i == j) ? d.q[i] : 0.0;

    bool converged = false;
    for (U32 it = 0; it < max_iter && !converged; ++it) {
        // PB = P Bd,  s = R + Bd'PBd (scalar),  PA = P Ad,  BtPA = Bd' P Ad (1x4)
        F64 PB[4], PA[4][4], BtPA[4];
        for (U32 i = 0; i < 4U; ++i) { F64 s = 0.0; for (U32 k = 0; k < 4U; ++k) s += P[i][k] * Bd[k]; PB[i] = s; }
        F64 s_ = d.r; for (U32 k = 0; k < 4U; ++k) s_ += Bd[k] * PB[k];
        if (!(s_ > 0.0) || !std::isfinite(s_)) return false;
        mat_mul(P, Ad, PA);
        for (U32 j = 0; j < 4U; ++j) { F64 s = 0.0; for (U32 k = 0; k < 4U; ++k) s += Bd[k] * PA[k][j]; BtPA[j] = s; }
        // P_next = Q + Ad'PAd - (Ad'PBd)(BtPA)/s_
        F64 Pn[4][4]; F64 diff = 0.0;
        for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) {
            F64 atpa = 0.0; for (U32 k = 0; k < 4U; ++k) atpa += Ad[k][i] * PA[k][j];   // (Ad'PAd)_ij
            F64 atpb = 0.0; for (U32 k = 0; k < 4U; ++k) atpb += Ad[k][i] * PB[k];      // (Ad'PBd)_i
            Pn[i][j] = ((i == j) ? d.q[i] : 0.0) + atpa - atpb * BtPA[j] / s_;
            const F64 e = std::fabs(Pn[i][j] - P[i][j]); if (e > diff) diff = e;
        }
        for (U32 i = 0; i < 4U; ++i) for (U32 j = 0; j < 4U; ++j) P[i][j] = Pn[i][j];
        if (!std::isfinite(diff)) return false;
        converged = (diff < tol);
    }
    if (!converged) return false;
    // K = (R + Bd'PBd)^-1 Bd'PAd
    F64 PB[4], PA[4][4];
    for (U32 i = 0; i < 4U; ++i) { F64 s = 0.0; for (U32 k = 0; k < 4U; ++k) s += P[i][k] * Bd[k]; PB[i] = s; }
    F64 s_ = d.r; for (U32 k = 0; k < 4U; ++k) s_ += Bd[k] * PB[k];
    mat_mul(P, Ad, PA);
    for (U32 j = 0; j < 4U; ++j) {
        F64 s = 0.0; for (U32 k = 0; k < 4U; ++k) s += Bd[k] * PA[k][j];
        const F64 kj = s / s_;
        if (!std::isfinite(kj)) return false;
        k_out[j] = static_cast<F32>(kj);    // standard LQR gain; the law applies u = -k.s
    }
    return true;
}

} // namespace fsw
