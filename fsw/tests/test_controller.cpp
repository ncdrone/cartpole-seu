/** @file test_controller.cpp  @brief Framework-free tests (plain asserts; NDEBUG is undefined). */
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <type_traits>
#include "fsw/Controller.hpp"
#include "fsw/Plant.hpp"
#include "fsw/Lqr.hpp"

using namespace fsw;

static_assert(std::is_trivially_copyable<Params>::value, "Params must be trivially copyable");
static_assert(std::is_trivially_copyable<State>::value, "State must be trivially copyable");
static_assert(std::is_trivially_copyable<Input>::value, "Input must be trivially copyable");
static_assert(std::is_trivially_copyable<Output>::value, "Output must be trivially copyable");
static_assert(std::is_trivially_copyable<PlantState>::value, "PlantState must be trivially copyable");
static_assert(std::is_standard_layout<Params>::value && std::is_standard_layout<State>::value,
              "must be standard layout");

/** Run closed loop; returns max |x| and fills final state and first-fault flag. */
static void run(double x0, double th0, double secs, PlantState& ps, double& max_abs_u) {
    PlantParams pp; plant_params_default(pp);
    ps.x = x0; ps.xdot = 0.0; ps.theta = th0; ps.thetadot = 0.0;
    Params p; params_default(p);
    State st; state_reset(st);
    max_abs_u = 0.0;
    const U32 n = static_cast<U32>(secs / cfg::DT_S);
    for (U32 k = 0; k < n; ++k) {
        Input in = { static_cast<F32>(ps.x), static_cast<F32>(ps.xdot), static_cast<F32>(ps.theta),
                     static_cast<F32>(ps.thetadot), cfg::DT_S };
        Output out;
        step(p, in, st, out);
        assert(out.fault == 0);
        if (std::fabs(out.force) > max_abs_u) max_abs_u = std::fabs(out.force);
        plant_step(pp, ps, out.force, cfg::DT_S);
    }
}

int main() {
    PlantState ps; double mu;

    {   // (f) on-board Riccati solve reproduces the offline scipy gains (same Q, R, dt, plant)
        PlantParams model; plant_params_default(model);
        Params pd; const bool okd = params_design(pd, model);
        Params pt; params_default(pt);
        assert(okd);
        for (U32 i = 0; i < 4U; ++i) {
            const double rel = std::fabs(static_cast<double>(pd.k[i]) - pt.k[i]) / std::fabs(pt.k[i]);
            std::printf("K[%u] dare=%.6f table=%.6f rel=%.1e%s\n", static_cast<unsigned>(i),
                        static_cast<double>(pd.k[i]), static_cast<double>(pt.k[i]), rel,
                        (pd.k[i] == pt.k[i]) ? "  bit-identical" : "");
            assert(rel < 1e-4);
        }
        // linearization sanity: upright is unstable, pole eigenvalue ~ sqrt(g/D) = 3.97 rad/s
        F64 A[4][4], B[4]; cartpole_linearize(model, A, B);
        assert(std::fabs(std::sqrt(A[3][2]) - 3.97) < 0.05 && B[3] < 0.0 && B[1] > 0.0);
    }
    {   // (g) theta wrap: 2*pi + 0.01 must act like +0.01, and -pi like +pi
        Params p; params_default(p); State s1; state_reset(s1); State s2; state_reset(s2); Output o1, o2;
        Input a = { 0.f, 0.f, 0.01f, 0.f, cfg::DT_S };
        Input b = { 0.f, 0.f, static_cast<F32>(2.0 * 3.14159265358979323846 + 0.01), 0.f, cfg::DT_S };
        step(p, a, s1, o1); step(p, b, s2, o2);
        assert(std::fabs(o1.force - o2.force) < 1e-3f);
    }

    {   // (h) swing-up: hanging down at rest -> balanced upright within 10 s, never off the rail
        PlantParams pp; plant_params_default(pp); Params p; params_default(p); State st; state_reset(st);
        PlantState s = { 0.0, 0.0, 3.14159265358979323846, 0.0 }; double maxx = 0.0; double t_bal = -1.0;
        for (U32 k = 0; k < 1000U; ++k) {
            Input in = { static_cast<F32>(s.x), static_cast<F32>(s.xdot), static_cast<F32>(s.theta), static_cast<F32>(s.thetadot), cfg::DT_S };
            Output o; step(p, in, st, o); assert(o.fault == 0);
            if (st.mode == MODE_BALANCE && t_bal < 0.0) t_bal = k * cfg::DT_S;
            plant_step(pp, s, o.force, cfg::DT_S);
            if (std::fabs(s.x) > maxx) maxx = std::fabs(s.x);
        }
        const double th_end = std::remainder(s.theta, 2.0 * 3.14159265358979323846);
        std::printf("swing-up from pi: balance at %.2f s, max|x|=%.2f m, end theta=%.4f x=%.4f\n", t_bal, maxx, th_end, s.x);
        assert(t_bal > 0.0 && maxx <= cfg::TRACK_LIMIT_M && std::fabs(th_end) < 0.05 && std::fabs(s.x) < 0.1);
    }
    run(0.0, 0.0, 5.0, ps, mu);                       // (a) upright stays upright
    assert(std::fabs(ps.theta) < 1e-4 && std::fabs(ps.x) < 1e-4);

    run(0.5, 0.2, 5.0, ps, mu);                       // (b) recovery
    std::printf("recover: x=%.4f theta=%.5f max|u|=%.2f\n", ps.x, ps.theta, mu);
    assert(std::fabs(ps.theta) < 0.05 && std::fabs(ps.x) < 0.2);
    assert(mu <= cfg::FORCE_LIMIT_N);                 // (d) clamp respected in closed loop

    {   // (c) NaN / Inf input -> safe output + fault
        Params p; params_default(p); State st; state_reset(st); Output out;
        Input in = { std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f, 0.f, cfg::DT_S };
        step(p, in, st, out);
        assert(out.fault == 1 && out.force == cfg::SAFE_FORCE_N);
        Input in2 = { 0.f, 0.f, std::numeric_limits<float>::infinity(), 0.f, cfg::DT_S };
        step(p, in2, st, out);
        assert(out.fault == 1 && out.force == cfg::SAFE_FORCE_N);
    }
    {   // (d) clamp with huge state
        Params p; params_default(p); State st; state_reset(st); Output out;
        Input in = { 100.f, 0.f, 1.f, 0.f, cfg::DT_S };
        step(p, in, st, out);
        assert(out.fault == 0 && out.saturated == 1 && std::fabs(out.force) == cfg::FORCE_LIMIT_N);
    }
    {   // integrator anti-windup
        Params p; params_default(p); p.ki_x = 1.0f; State st; state_reset(st); Output out;
        Input in = { 1.f, 0.f, 0.f, 0.f, 1.0f };
        for (int i = 0; i < 100; ++i) step(p, in, st, out);
        assert(std::fabs(st.integ) <= p.integ_limit);
        state_reseed(st);
        assert(st.integ == 0.0f && st.thdot_filt == 0.0f);
    }
    std::printf("ALL TESTS PASSED\n");
    return 0;
}
