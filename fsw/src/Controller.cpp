/** @file Controller.cpp */
#include "fsw/Controller.hpp"
#include "fsw/Lqr.hpp"
#include <cmath>
#include <cstring>

namespace fsw {

static F32 clampf(F32 v, F32 lim) {
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

void params_default(Params& p) {
    std::memset(&p, 0, sizeof(p));
    p.k[0] = cfg::GAIN_X;
    p.k[1] = cfg::GAIN_XDOT;
    p.k[2] = cfg::GAIN_THETA;
    p.k[3] = cfg::GAIN_THDOT;
    p.ki_x = cfg::KI_X;
    p.integ_limit = cfg::INTEG_LIMIT;
    p.dfilt_alpha = cfg::DFILT_ALPHA;
    p.sw_k_energy = cfg::SW_K_ENERGY; p.sw_a_max = cfg::SW_A_MAX; p.sw_a_min = cfg::SW_A_MIN; p.sw_k_x = cfg::SW_K_X; p.sw_k_v = cfg::SW_K_V;
    p.sw_e_ref = cfg::SW_E_REF; p.sw_enter_rad = cfg::SW_ENTER_RAD; p.sw_enter_rate = cfg::SW_ENTER_RATE;
    p.sw_exit_rad = cfg::SW_EXIT_RAD;
    p.m_cart = cfg::CART_MASS_KG; p.m_pole = cfg::POLE_MASS_KG; p.half_len = cfg::POLE_HALF_LEN_M; p.gravity = cfg::GRAVITY_MS2;
}

bool params_design(Params& p, const PlantParams& model) {
    params_default(p);
    LqrDesign d; lqr_design_default(d);
    F32 k[4];
    if (!lqr_solve(model, d, k, cfg::LQR_MAX_ITER, cfg::LQR_TOL)) return false;   // keep table gains
    for (U32 i = 0; i < 4U; ++i) p.k[i] = k[i];
    return true;
}

void state_reset(State& st) { std::memset(&st, 0, sizeof(st)); }

static bool band(F32 v, F32 ref, F32 lo_mul, F32 hi_mul) {   // same sign as ref, magnitude within [lo_mul, hi_mul] x |ref|
    if (!std::isfinite(v) || ref == 0.0f) return std::isfinite(v) && v == ref;
    if ((v < 0.0f) != (ref < 0.0f)) return false;
    const F32 av = (v < 0.0f) ? -v : v, ar = (ref < 0.0f) ? -ref : ref;
    return av >= lo_mul * ar && av <= hi_mul * ar;
}
bool params_in_range(const Params& p, const Params& r) {
    for (U32 i = 0; i < 4U; ++i) if (!band(p.k[i], r.k[i], 0.5f, 2.0f)) return false;
    if (!(std::isfinite(p.ki_x) && p.ki_x <= 0.0f && p.ki_x >= -1.0f)) return false;
    if (!(std::isfinite(p.integ_limit) && p.integ_limit > 0.0f && p.integ_limit <= 10.0f)) return false;
    if (!(std::isfinite(p.dfilt_alpha) && p.dfilt_alpha > 0.0f && p.dfilt_alpha <= 1.0f)) return false;
    if (!band(p.sw_k_energy, r.sw_k_energy, 0.5f, 2.0f) || !band(p.sw_a_max, r.sw_a_max, 0.5f, 2.0f) ||
        !band(p.sw_a_min, r.sw_a_min, 0.5f, 2.0f) || !band(p.sw_k_x, r.sw_k_x, 0.5f, 2.0f) ||
        !band(p.sw_k_v, r.sw_k_v, 0.5f, 2.0f) || !band(p.sw_e_ref, r.sw_e_ref, 0.5f, 2.0f) ||
        !band(p.sw_enter_rad, r.sw_enter_rad, 0.5f, 2.0f) || !band(p.sw_enter_rate, r.sw_enter_rate, 0.5f, 2.0f) ||
        !band(p.sw_exit_rad, r.sw_exit_rad, 0.5f, 2.0f)) return false;
    if (!(p.sw_a_min <= p.sw_a_max && p.sw_enter_rad < p.sw_exit_rad && p.sw_exit_rad < 3.1416f)) return false;
    if (!band(p.m_cart, r.m_cart, 0.5f, 2.0f) || !band(p.m_pole, r.m_pole, 0.5f, 2.0f) ||
        !band(p.half_len, r.half_len, 0.5f, 2.0f) || !band(p.gravity, r.gravity, 0.5f, 2.0f)) return false;
    return true;
}

void state_reseed(State& st) {
    st.integ = 0.0f;
    st.thdot_filt = 0.0f;
    st.filt_init = 0;
}

void step(const Params& p, const Input& in, State& st, Output& out) {
    out.fault = 0; out.saturated = 0; out.pad[0] = 0; out.pad[1] = 0;
    st.tick++;

    // TODO(protect): checksum/SECDED verify of p and st here; recover via verified golden reload + state_reseed().

    bool ok = std::isfinite(in.x) && std::isfinite(in.xdot) && std::isfinite(in.theta) &&
              std::isfinite(in.thetadot) && std::isfinite(in.dt) && in.dt > 0.0f &&
              std::isfinite(st.integ) && std::isfinite(st.thdot_filt);
    for (U32 i = 0; i < 4U; ++i) { ok = ok && std::isfinite(p.k[i]); }
    ok = ok && std::isfinite(p.ki_x) && std::isfinite(p.integ_limit) &&
         std::isfinite(p.dfilt_alpha) &&
         std::isfinite(p.sw_k_energy) && std::isfinite(p.sw_a_max) && std::isfinite(p.sw_a_min) && std::isfinite(p.sw_k_x) &&
         std::isfinite(p.sw_k_v) && std::isfinite(p.sw_e_ref) && std::isfinite(p.sw_enter_rad) &&
         std::isfinite(p.sw_enter_rate) && std::isfinite(p.sw_exit_rad) && std::isfinite(p.m_cart) &&
         std::isfinite(p.m_pole) && std::isfinite(p.half_len) && std::isfinite(p.gravity);
    if (!ok) {
        out.force = cfg::SAFE_FORCE_N;
        out.fault = 1;
        return;
    }

    // Wrap theta to (-pi, pi] so a full rotation (or theta0 = -pi vs +pi) is not a 360 deg error.
    const F32 theta = static_cast<F32>(std::remainder(static_cast<F64>(in.theta), 2.0 * 3.14159265358979323846));
    const F32 abs_th = (theta < 0.0f) ? -theta : theta;
    const F32 abs_thd = (in.thetadot < 0.0f) ? -in.thetadot : in.thetadot;

    // Derivative-memory: first-order low-pass on thetadot. First tick also picks the mode from the measured angle.
    if (st.filt_init == 0) {
        st.thdot_filt = in.thetadot; st.filt_init = 1;
        st.mode = (abs_th < p.sw_enter_rad && abs_thd < p.sw_enter_rate) ? static_cast<U8>(MODE_BALANCE) : static_cast<U8>(MODE_SWING);
    } else {
        st.thdot_filt += p.dfilt_alpha * (in.thetadot - st.thdot_filt);
        // Hysteresis: enter BALANCE only near upright and slow; leave it only well outside the basin.
        if (st.mode == MODE_SWING && abs_th < p.sw_enter_rad && abs_thd < p.sw_enter_rate) st.mode = MODE_BALANCE;
        else if (st.mode == MODE_BALANCE && abs_th > p.sw_exit_rad) st.mode = MODE_SWING;
    }

    // Integrator with anti-windup clamp.
    st.integ = clampf(st.integ + in.x * in.dt, p.integ_limit);

    F32 u;
    if (st.mode == MODE_BALANCE) {
        u = -(p.k[0] * in.x + p.k[1] * in.xdot + p.k[2] * theta + p.k[3] * st.thdot_filt) - p.ki_x * st.integ;
    } else {
        // Energy shaping (Astrom & Furuta 2000). Pole energy about upright, I_pivot = 4/3 m l^2 (Florian model):
        //   E = 1/2 I thdot^2 + m g l (cos th - 1); E = 0 upright at rest, -2 m g l hanging at rest.
        // dE/dt from a cart acceleration a is proportional to -thdot cos(th) a, so
        //   a = k_e (E - E_ref) sign(thdot cos th) raises E while E < E_ref. At sign(0) (exactly hanging, at rest) kick +1.
        const F32 I = (4.0f / 3.0f) * p.m_pole * p.half_len * p.half_len;
        const F32 E = 0.5f * I * st.thdot_filt * st.thdot_filt + p.m_pole * p.gravity * p.half_len * (std::cos(theta) - 1.0f);
        const F32 s = st.thdot_filt * std::cos(theta);
        const F32 sgn = (s > cfg::SW_KICK_EPS) ? 1.0f : ((s < -cfg::SW_KICK_EPS) ? -1.0f : 1.0f);
        const F32 err = p.sw_e_ref - E;                        // > 0: the pole lacks energy, pump it in
        F32 mag = p.sw_k_energy * ((err > 0.0f) ? err : -err);
        if (err > 0.0f && mag < p.sw_a_min) mag = p.sw_a_min;  // deficit side only: a model error must not stall the pump
        if (mag > p.sw_a_max) mag = p.sw_a_max;
        F32 a = (err > 0.0f) ? -mag * sgn : mag * sgn;         // = k_e (E - E_ref) sgn with the floor on the deficit side
        a -= p.sw_k_x * in.x + p.sw_k_v * in.xdot;      // keep the cart near the centre of the rail while pumping
        u = (p.m_cart + p.m_pole) * a;
    }

    if (!std::isfinite(u)) {
        out.force = cfg::SAFE_FORCE_N;
        out.fault = 1;
        return;
    }
    const F32 uc = clampf(u, cfg::FORCE_LIMIT_N);
    out.saturated = (uc != u) ? 1 : 0;
    out.force = uc;
}

} // namespace fsw
