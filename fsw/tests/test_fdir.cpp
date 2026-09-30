/** @file test_fdir.cpp  @brief FDIR state machine tests, driving decode/control directly (plain asserts). */
#undef NDEBUG
#include <cassert>
#include <cmath>
#include <cstdio>
#include "fsw/Runtime.hpp"

using namespace fsw;

static Input make_in(F32 theta) { Input in = { 0.0f, 0.0f, theta, 0.0f, cfg::DT_S }; return in; }

/** One tick = decode + control (same copy in both lanes). Returns decode's result. */
static bool tick(Runtime& rt, F32 theta, Output& out, TickTlm& t) {
    Params p; State s;
    const bool ok = runtime_decode(rt, p, s, t);
    runtime_control(rt, p, p, s, make_in(theta), out, t);
    return ok;
}
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
        for (U32 i = 0; i < 20U; ++i) {                              // 0.6 rad: BALANCE holds (exit 0.75), raw law saturates
            tick(rt, 0.6f, o, t);
            assert(runtime_fdir(rt) == FDIR_DEGRADED);
            assert(t.mode == MODE_BALANCE);
            assert(std::fabs(o.force) <= lim);
            if (o.saturated && std::fabs(o.force) == lim) saw_sat = true;
        }
        assert(saw_sat);
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
    std::printf("ALL FDIR TESTS PASSED\n");
    return 0;
}
