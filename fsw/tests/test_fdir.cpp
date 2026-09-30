/** @file test_fdir.cpp  @brief FDIR state machine tests, driving decode/control directly (plain asserts). */
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include "fsw/Runtime.hpp"
#include "fsw/Plant.hpp"

using namespace fsw;

static Input make_in(F32 theta) { Input in = { 0.0f, 0.0f, theta, 0.0f, cfg::DT_S }; return in; }

/** One tick = decode + control (same copy in both lanes). Returns decode's result. */
static bool tick(Runtime& rt, F32 theta, Output& out, TickTlm& t) {
    Params p; State s;
    const bool ok = runtime_decode(rt, p, s, t);
    runtime_control(rt, p, p, s, make_in(theta), out, t);
    return ok;
}
/** Tests that step theta discontinuously drop the plausibility reference first (a fresh sample, not a glitch). */
static void forget_last(Runtime& rt) { rt.have_last = 0U; }
/** Closed-loop plant driven by the runtime (same plant model the sim uses). */
struct Plant {
    PlantParams pp; PlantState ps;
    void reset(double th0) { plant_params_default(pp); ps.x = 0.0; ps.xdot = 0.0; ps.theta = th0; ps.thetadot = 0.0; }
    void tick(Runtime& rt, Output& out, TickTlm& t) {
        Input in = { static_cast<F32>(ps.x), static_cast<F32>(ps.xdot), static_cast<F32>(ps.theta), static_cast<F32>(ps.thetadot), cfg::DT_S };
        Params p; State s;
        runtime_decode(rt, p, s, t); runtime_control(rt, p, p, s, in, out, t);
        plant_step(pp, ps, out.force, cfg::DT_S);
    }
    void coast(F32 force) { plant_step(pp, ps, force, cfg::DT_S); }   // controller skipped: actuator holds the last force
};
static void init(Runtime& rt) {
    PlantParams m; plant_params_default(m);
    bool ok = runtime_init(rt, m); assert(ok); (void)ok;
}
static void double_flip(Runtime& rt, U32 word) { rt.pstore.words[word] ^= (1U << 30) | (1U << 31); }
static void clean_ticks(Runtime& rt, U32 n, F32 theta) {
    Output o; TickTlm t;
    for (U32 i = 0; i < n; ++i) { tick(rt, theta, o, t); assert(t.det == DET_NONE); }
}
/** NOMINAL -> RECOVERING -> DEGRADED via two double flips on consecutive ticks. */
static void reach_degraded(Runtime& rt) {
    Output o; TickTlm t;
    init(rt); tick(rt, 0.02f, o, t);
    double_flip(rt, 2); tick(rt, 0.02f, o, t); assert(runtime_fdir(rt) == FDIR_RECOVERING);
    double_flip(rt, 3); tick(rt, 0.02f, o, t); assert(runtime_fdir(rt) == FDIR_DEGRADED);
}

int main() {
    Output o; TickTlm t;

    { // (a) SEC: single-bit flip is corrected and scrubbed, not a detection for transitions
        Runtime rt; init(rt);
        const U32 orig = rt.pstore.words[2];
        rt.pstore.words[2] ^= (1U << 3);
        Params p; State s;
        assert(runtime_decode(rt, p, s, t));
        assert(t.det == DET_SEC);
        assert(runtime_fdir(rt) == FDIR_NOMINAL);
        assert(rt.sec_total == 1U);
        assert(rt.pstore.words[2] == orig);
        std::printf("(a) SEC ok\n");
    }
    { // (b) DED -> RECOVERING -> NOMINAL after 50 clean ticks
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        double_flip(rt, 2);
        tick(rt, 0.02f, o, t);
        assert(t.det == DET_DED);
        assert(runtime_fdir(rt) == FDIR_RECOVERING);
        assert(rt.reload_total == 1U);
        clean_ticks(rt, cfg::FDIR_CLEAN_TICKS, 0.02f);
        assert(runtime_fdir(rt) == FDIR_NOMINAL);
        std::printf("(b) DED recover ok\n");
    }
    { // (c) second detection inside the window -> DEGRADED; halved clamp; 200 clean ticks -> NOMINAL
        Runtime rt; reach_degraded(rt);
        assert(rt.reload_total == 2U);
        clean_ticks(rt, 5U, 0.02f);                                  // reseed re-picks BALANCE near upright
        const F32 lim = 0.5f * cfg::FORCE_LIMIT_N;
        bool saw_sat = false;
        forget_last(rt);
        for (U32 i = 0; i < 20U; ++i) {                              // 0.6 rad: BALANCE holds (exit 0.75), raw law saturates
            tick(rt, 0.6f, o, t);
            assert(runtime_fdir(rt) == FDIR_DEGRADED);
            assert(t.mode == MODE_BALANCE);
            assert(std::fabs(o.force) <= lim);
            if (o.saturated && std::fabs(o.force) == lim) saw_sat = true;
        }
        assert(saw_sat);
        forget_last(rt);
        clean_ticks(rt, 5U, 0.02f);
        // remaining clean ticks to complete DEGRADED_CLEAN_TICKS (30 already counted)
        clean_ticks(rt, cfg::FDIR_DEGRADED_CLEAN_TICKS - 30U - 1U, 0.02f);
        assert(runtime_fdir(rt) == FDIR_DEGRADED);
        clean_ticks(rt, 1U, 0.02f);
        assert(runtime_fdir(rt) == FDIR_NOMINAL);
        std::printf("(c) DEGRADED ok\n");
    }
    { // (d) third detection -> SAFE latched
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        for (U32 w = 2; w < 5; ++w) { double_flip(rt, w); tick(rt, 0.02f, o, t); }
        assert(runtime_fdir(rt) == FDIR_SAFE);
        for (U32 i = 0; i < 1000U; ++i) {
            tick(rt, 0.02f, o, t);
            assert(runtime_fdir(rt) == FDIR_SAFE);
            assert(o.force == cfg::SAFE_FORCE_N && o.fault == 1U);
        }
        std::printf("(d) SAFE latch ok\n");
    }
    { // (e) corrupt state word reads as SAFE
        Runtime rt; init(rt);
        rt.fdir_state ^= 1U;
        assert(runtime_fdir(rt) == FDIR_SAFE);
        std::printf("(e) state word ok\n");
    }
    { // (f) untrusted golden -> SAFE
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        rt.golden_crc ^= 1U;
        double_flip(rt, 2);
        tick(rt, 0.02f, o, t);
        assert(t.det == DET_GOLDEN);
        assert(runtime_fdir(rt) == FDIR_SAFE);
        std::printf("(f) golden ok\n");
    }
    { // (g) stall -> RECOVERING
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        const U32 before = rt.reload_total;
        runtime_notify_stall(rt);
        tick(rt, 0.02f, o, t);
        assert(t.det == DET_STALL);
        assert(runtime_fdir(rt) == FDIR_RECOVERING);
        assert(rt.reload_total == before + 1U);
        std::printf("(g) stall ok\n");
    }
    { // (h) DEGRADED inhibits swing-up
        Runtime rt; reach_degraded(rt);
        forget_last(rt);
        for (U32 i = 0; i < 5U; ++i) {
            tick(rt, 2.5f, o, t);
            assert(runtime_fdir(rt) == FDIR_DEGRADED);
            assert(t.mode == MODE_SWING);
            assert(o.force == cfg::SAFE_FORCE_N && o.fault == 1U);
        }
        std::printf("(h) swing inhibit ok\n");
    }
    { // (i) window rollover: a detection long after an earlier one is a fresh first detection, not SAFE
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        double_flip(rt, 2); tick(rt, 0.02f, o, t); assert(runtime_fdir(rt) == FDIR_RECOVERING);
        clean_ticks(rt, 300U, 0.02f);
        assert(runtime_fdir(rt) == FDIR_NOMINAL);
        double_flip(rt, 2); tick(rt, 0.02f, o, t);
        assert(t.det == DET_DED);
        assert(runtime_fdir(rt) == FDIR_RECOVERING);
        std::printf("(i) window rollover ok\n");
    }
    { // (j) first fault after a DEGRADED -> NOMINAL exit is RECOVERING, not SAFE
        Runtime rt; reach_degraded(rt);
        clean_ticks(rt, cfg::FDIR_DEGRADED_CLEAN_TICKS, 0.02f);
        assert(runtime_fdir(rt) == FDIR_NOMINAL);
        double_flip(rt, 2); tick(rt, 0.02f, o, t);
        assert(t.det == DET_DED);
        assert(runtime_fdir(rt) == FDIR_RECOVERING);
        std::printf("(j) post-degraded fault ok\n");
    }
    { // (k) closed loop from 0.3 rad for 500 ticks: the guard never fires on a real trajectory
        Runtime rt; init(rt); Plant pl; pl.reset(0.3);
        for (U32 i = 0; i < 500U; ++i) { pl.tick(rt, o, t); assert(t.det != DET_INPUT); }
        assert(rt.det_total == 0U && runtime_fdir(rt) == FDIR_NOMINAL);
        std::printf("(k) closed loop, no DET_INPUT ok\n");
    }
    { // (l) stall then resume: the plant coasts on the held force, the first fresh sample is a STALL, never an INPUT reject
        Runtime rt; init(rt); Plant pl; pl.reset(0.3);
        for (U32 i = 0; i < 30U; ++i) pl.tick(rt, o, t);
        runtime_notify_stall(rt);
        for (U32 i = 0; i < 20U; ++i) pl.coast(o.force);
        pl.tick(rt, o, t);
        assert(t.det == DET_STALL);
        for (U32 i = 0; i < 50U; ++i) { pl.tick(rt, o, t); assert(t.det != DET_INPUT); }
        assert(runtime_fdir(rt) == FDIR_NOMINAL);
        std::printf("(l) stall resume ok\n");
    }
    { // (m) one case per plausibility rule; first reject holds without reload, a second consecutive reject reloads
        const F32 nan = std::nanf("");
        const Input bads[5] = { { 0.0f, 0.0f, nan, 0.0f, cfg::DT_S }, { 3.0f, 0.0f, 0.0f, 0.0f, cfg::DT_S },
                                { 0.0f, 0.0f, 0.0f, 26.0f, cfg::DT_S }, { 0.0f, 0.0f, 1.0f, 0.0f, cfg::DT_S },
                                { 0.0f, 11.0f, 0.0f, 0.0f, cfg::DT_S } };
        for (U32 c = 0; c < 5U; ++c) {
            Runtime rt; init(rt); tick(rt, 0.0f, o, t); tick(rt, 0.0f, o, t);
            Params p; State s;
            runtime_decode(rt, p, s, t); runtime_control(rt, p, p, s, bads[c], o, t);
            assert(t.det == DET_INPUT);
            assert(rt.reload_total == 0U);
            assert(runtime_fdir(rt) == FDIR_RECOVERING);
            if (c == 4U) assert(std::fabs(o.force) <= cfg::FORCE_LIMIT_N && o.fault == 0U && o.saturated == 0U);   // xdot held, not clamped
            if (c == 3U) {
                runtime_decode(rt, p, s, t); runtime_control(rt, p, p, s, bads[c], o, t);
                assert(t.det == DET_INPUT);
                assert(rt.reload_total == 1U);
            }
        }
        { // first-ever sample non-finite while the pole hangs: no invented upright sample, safe command, no reload
            Runtime rt; init(rt); Params p; State s;
            const Input nan_in = { 0.0f, 0.0f, nan, 0.0f, cfg::DT_S };
            runtime_decode(rt, p, s, t); runtime_control(rt, p, p, s, nan_in, o, t);
            assert(o.force == cfg::SAFE_FORCE_N && o.fault == 1U);
            assert(t.det == DET_INPUT && rt.reload_total == 0U && rt.have_last == 0U);
        }
        { // persistent finite offset: third consecutive reject -> SAFE (the input guard's own reload keeps its reference)
            Runtime rt; init(rt); tick(rt, 0.02f, o, t); tick(rt, 0.02f, o, t);
            for (U32 i = 0; i < 3U; ++i) { tick(rt, 1.02f, o, t); assert(t.det == DET_INPUT); }
            assert(runtime_fdir(rt) == FDIR_SAFE);
        }
        std::printf("(m) plausibility rules ok\n");
    }
    { // (n) after SAFE nothing reads uninitialised memory: safe command and a valid mode on every tick
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        for (U32 w = 2; w < 5; ++w) { double_flip(rt, w); tick(rt, 0.02f, o, t); }
        assert(runtime_fdir(rt) == FDIR_SAFE);
        for (U32 i = 0; i < 100U; ++i) {
            Params p; State s;
            std::memset(&p, 0xA5, sizeof p); std::memset(&s, 0xA5, sizeof s);   // a caller's garbage must not reach the output
            const bool ok = runtime_decode(rt, p, s, t);
            assert(!ok);
            assert(s.mode == 0U && p.k[0] == cfg::GAIN_X);   // decode_safe filled p and s
            runtime_control(rt, p, p, s, make_in(0.02f), o, t);
            assert(o.force == cfg::SAFE_FORCE_N && o.fault == 1U && t.mode <= 1U);
        }
        std::printf("(n) SAFE reads no garbage ok\n");
    }
    { // (o) a flipped clean_ticks mirror while RECOVERING reads as SAFE
        Runtime rt; init(rt); tick(rt, 0.02f, o, t);
        double_flip(rt, 2); tick(rt, 0.02f, o, t);
        assert(runtime_fdir(rt) == FDIR_RECOVERING);
        rt.clean_ticks ^= 1U;
        tick(rt, 0.02f, o, t);
        assert(runtime_fdir(rt) == FDIR_SAFE && t.fdir == FDIR_SAFE);
        assert(o.force == cfg::SAFE_FORCE_N && o.fault == 1U);
        tick(rt, 0.02f, o, t);
        assert(o.force == cfg::SAFE_FORCE_N && o.fault == 1U);
        std::printf("(o) clean_ticks mirror ok\n");
    }
    std::printf("ALL FDIR TESTS PASSED\n");
    return 0;
}
