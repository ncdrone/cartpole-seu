/** @file Runtime.cpp  @brief See Runtime.hpp. */
#include "fsw/Runtime.hpp"
#include "fsw/Math.hpp"
#include <cmath>
#include <cstring>

namespace fsw {

/* FDIR state codes: Hamming distance 16 apart; anything else reads as SAFE. */
static const U32 C_NOMINAL = 0x5A5A5A5AU, C_RECOVERING = 0xA5A5A5A5U, C_DEGRADED = 0x3C3C3C3CU, C_SAFE = 0xC3C3C3C3U;

FdirState runtime_fdir(const Runtime& rt) {
    if ((rt.fdir_state ^ rt.fdir_state_inv) != 0xFFFFFFFFU) return FDIR_SAFE;   // the state word itself is corrupt
    if (rt.fdir_state == C_NOMINAL) return FDIR_NOMINAL;
    if (rt.fdir_state == C_RECOVERING) return FDIR_RECOVERING;
    if (rt.fdir_state == C_DEGRADED) return FDIR_DEGRADED;
    return FDIR_SAFE;
}
static void fdir_set(Runtime& rt, FdirState s) {
    U32 c = C_SAFE;
    if (s == FDIR_NOMINAL) c = C_NOMINAL;
    else if (s == FDIR_RECOVERING) c = C_RECOVERING;
    else if (s == FDIR_DEGRADED) c = C_DEGRADED;
    rt.fdir_state = c; rt.fdir_state_inv = ~c;
}

static U32 params_crc(const Params& p) {
    U8 buf[sizeof(Params)]; std::memcpy(buf, &p, sizeof buf);
    return crc32c(buf, static_cast<U32>(sizeof buf));
}

#if FSW_PROTECT
static void encode_params(Runtime& rt, const Params& p) {
    U32 w[PARAM_WORDS]; std::memcpy(w, &p, sizeof w);
    store_encode(w, PARAM_WORDS, rt.pstore.words, rt.pstore.check, &rt.pstore.crc, &rt.pstore.crc_inv);
}
static void encode_state(Runtime& rt, const State& s) {
    U32 w[STATE_WORDS]; std::memcpy(w, &s, sizeof w);
    store_encode(w, STATE_WORDS, rt.sstore.words, rt.sstore.check, &rt.sstore.crc, &rt.sstore.crc_inv);
}
#endif

bool runtime_init(Runtime& rt, const PlantParams& model) {
    std::memset(&rt, 0, sizeof rt);
    rt.model = model;
    Params p; if (!params_design(p, model)) return false;
    const U32 c = params_crc(p); rt.golden_crc = c; rt.golden_crc_inv = ~c;
    State s; state_reset(s);
#if FSW_PROTECT
    encode_params(rt, p); encode_state(rt, s);
#else
    rt.p = p; rt.s = s;
#endif
    fdir_set(rt, FDIR_NOMINAL);
    rt.det_count = 0U; rt.det_count_inv = ~0U;
    rt.window_start = 0U; rt.window_start_inv = ~0U;
    rt.clean_ticks = 0U; rt.clean_ticks_inv = ~0U;
    return true;
}

#if FSW_PROTECT
/* Rebuild the golden from the model and accept it only if it reproduces the pinned CRC. */
static bool reload_golden(Runtime& rt, Params& p, State& s) {
    Params g; if (!params_design(g, rt.model)) return false;
    const U32 c = params_crc(g);
    if (c != rt.golden_crc || c != static_cast<U32>(~rt.golden_crc_inv)) return false;
    p = g; state_reseed(s);
    encode_params(rt, p); encode_state(rt, s);
    rt.reload_total++;
    return true;
}

/* (v, ~v) read: a mismatch means the counter is corrupt, which is SAFE. */
static bool mirror_ok_else_safe(Runtime& rt, U32 v, U32 inv) {
    if ((v ^ inv) != 0xFFFFFFFFU) { fdir_set(rt, FDIR_SAFE); return false; }
    return true;
}
static void set_window_start(Runtime& rt, U32 v) { rt.window_start = v; rt.window_start_inv = ~v; }
static void set_clean_ticks(Runtime& rt, U32 v) { rt.clean_ticks = v; rt.clean_ticks_inv = ~v; }

static void on_detection(Runtime& rt, Detection d, Params& p, State& s, TickTlm& t, bool reload = true) {
    if (t.det == DET_NONE || t.det == DET_SEC) t.det = static_cast<U8>(d);
    rt.det_total++;
    if (!mirror_ok_else_safe(rt, rt.window_start, rt.window_start_inv)) return;                            // window anchor corrupt
    if (rt.tick - rt.window_start > cfg::FDIR_WINDOW_TICKS) { set_window_start(rt, rt.tick); rt.det_count = 0U; rt.det_count_inv = ~0U; }   // window expired: re-anchor, reset both words
    if (!mirror_ok_else_safe(rt, rt.det_count, rt.det_count_inv)) return;                                  // counter corrupt
    if (rt.det_count == 0U) set_window_start(rt, rt.tick);                                          // window anchors at the first detection
    rt.det_count++; rt.det_count_inv = ~rt.det_count;
    if (rt.det_count >= cfg::FDIR_MAX_DETECTIONS) { fdir_set(rt, FDIR_SAFE); return; }           // persistence
    const FdirState prev = runtime_fdir(rt);
    if (prev == FDIR_DEGRADED) { fdir_set(rt, FDIR_SAFE); return; }                              // any detection while degraded
    if (reload && !reload_golden(rt, p, s)) { t.det = DET_GOLDEN; fdir_set(rt, FDIR_SAFE); return; }
    // sample history is untrusted after a store/compute reload; NOT after the input guard's own reload (a persistent input fault must keep escalating)
    if (reload && d != DET_INPUT) { rt.have_last = 0U; rt.input_rejects = 0U; }
    // a second detection inside the window while still recovering escalates to DEGRADED
    fdir_set(rt, (prev == FDIR_RECOVERING && rt.det_count >= 2U) ? FDIR_DEGRADED : FDIR_RECOVERING);
    set_clean_ticks(rt, 0U);
}

#endif  /* FSW_PROTECT: reload + on_detection */

/* SAFE: hand the caller defined defaults, never uninitialised or corrupt copies. */
static bool decode_safe(Params& p, State& s, TickTlm& t) {
    params_default(p); state_reset(s);
    t.fdir = FDIR_SAFE;
    return false;
}

bool runtime_decode(Runtime& rt, Params& p, State& s, TickTlm& t) {
    std::memset(&t, 0, sizeof t);
    rt.tick++;
    if (runtime_fdir(rt) == FDIR_SAFE) return decode_safe(p, s, t);
#if FSW_PROTECT
    StoreReport rp, rs; U32 pw[PARAM_WORDS], sw[STATE_WORDS];
    const bool okp = store_decode(rt.pstore.words, rt.pstore.check, &rt.pstore.crc, &rt.pstore.crc_inv, PARAM_WORDS, pw, rp);
    const bool oks = store_decode(rt.sstore.words, rt.sstore.check, &rt.sstore.crc, &rt.sstore.crc_inv, STATE_WORDS, sw, rs);
    std::memcpy(&p, pw, sizeof p); std::memcpy(&s, sw, sizeof s);
    rt.sec_total += rp.sec + rs.sec;
    if (rp.sec + rs.sec > 0U) t.det = DET_SEC;
    if (!okp || !oks) { /* first detector in the order names the event */ on_detection(rt, (rp.ded || rs.ded) ? DET_DED : DET_CRC, p, s, t); return runtime_fdir(rt) == FDIR_SAFE ? decode_safe(p, s, t) : true; }
    Params ref; params_default(ref);                    // range reference: compile-time literals, no RAM copy
    if (!params_in_range(p, ref)) { on_detection(rt, DET_RANGE, p, s, t); return runtime_fdir(rt) == FDIR_SAFE ? decode_safe(p, s, t) : true; }
    if (rt.stall_pending) { rt.stall_pending = 0U; rt.have_last = 0U; rt.input_rejects = 0U; on_detection(rt, DET_STALL, p, s, t); return runtime_fdir(rt) == FDIR_SAFE ? decode_safe(p, s, t) : true; }
#else
    p = rt.p; s = rt.s;
    rt.stall_pending = 0U;                              // baseline: a stall is just skipped ticks
#endif
    return true;
}

#if FSW_PROTECT
/* Plausibility of the sensor sample against the last accepted one. Held samples widen the comparison interval. */
static bool input_plausible(const Runtime& rt, const Input& in) {
    if (!std::isfinite(in.x) || !std::isfinite(in.xdot) || !std::isfinite(in.theta) || !std::isfinite(in.thetadot)) return false;
    if (std::fabs(in.x) > cfg::TRACK_LIMIT_M + 0.5f) return false;
    if (std::fabs(in.xdot) > cfg::PLAUS_MAX_XDOT) return false;
    if (std::fabs(in.thetadot) > cfg::PLAUS_MAX_RATE) return false;
    if (rt.have_last) {
        const F32 dt = in.dt * static_cast<F32>(rt.input_rejects + 1U);
        const F32 pred = 0.5f * (in.thetadot + rt.last_thetadot) * dt;
        if (std::fabs(wrap_pi(in.theta - rt.last_theta) - pred) > cfg::PLAUS_THETA_TOL) return false;
    }
    return true;
}

/* Input guard: plausibility check on a local copy `inp` (the caller's sample is never modified). A reject holds the last
 * accepted sample and raises DET_INPUT. Returns true when step() must be skipped (nothing safe to feed the law).
 * `persistent` is set when this is a repeat reject: both lanes then run on the (possibly reloaded) golden `pr`. */
static bool guard_input(Runtime& rt, Input& inp, Params& pr, State& s, TickTlm& t, bool& persistent) {
    bool skip = false;
    persistent = false;
    if (!input_plausible(rt, inp)) {
        if (rt.have_last) { inp.x = rt.last_x; inp.xdot = rt.last_xdot; inp.theta = rt.last_theta; inp.thetadot = rt.last_thetadot; }   // hold the last accepted sample
        else {  // nothing to hold (first sample, or history reset by a stall/reload). A non-finite field cannot be repaired without
                // inventing a sample, so step() is skipped and the safe command is issued; otherwise the rejected sample is used
                // with x, xdot and thetadot clamped to their plausible limits
            if (!std::isfinite(inp.x) || !std::isfinite(inp.xdot) || !std::isfinite(inp.theta) || !std::isfinite(inp.thetadot)) skip = true;
            else {
                inp.x = clampf(inp.x, cfg::TRACK_LIMIT_M + 0.5f);
                inp.xdot = clampf(inp.xdot, cfg::PLAUS_MAX_XDOT);
                inp.thetadot = clampf(inp.thetadot, cfg::PLAUS_MAX_RATE);
            }
        }
        const bool repeat = rt.input_rejects > 0U;
        rt.input_rejects++;
        on_detection(rt, DET_INPUT, pr, s, t, repeat);  // first reject: no reload (the store is fine); a persistent one escalates via the reload path
        persistent = repeat;
    } else {
        rt.input_rejects = 0U;
        rt.last_x = inp.x; rt.last_xdot = inp.xdot; rt.last_theta = inp.theta; rt.last_thetadot = inp.thetadot; rt.have_last = 1U;
    }
    return skip;
}

/* Dual execution: step() on both lanes; a difference is a compute-window flip. Leaves the accepted command in o2 and
 * the next state in s. */
static void dual_step(Runtime& rt, const Params& q1, const Params& q2, Params& pr, State& s, const Input& inp, TickTlm& t, Output& o2) {
    State s1 = s, s2 = s; Output o1;
    step(q1, inp, s1, o1); step(q2, inp, s2, o2);
    if (o1.force != o2.force || o1.fault != o2.fault || s1.mode != s2.mode) {   // compute-window flip in one lane
        on_detection(rt, DET_MISMATCH, pr, s, t);
        if (runtime_fdir(rt) != FDIR_SAFE) { State sr = s; step(pr, inp, sr, o2); s = sr; }   // one more step on the reloaded golden
    } else { s = s1; }
}

/* Output guard from constants (step already clamps with cfg::FORCE_LIMIT_N; this is the independent check). */
static void guard_output(Runtime& rt, const Output& o2, Params& pr, State& s, TickTlm& t, Output& out) {
    if (runtime_fdir(rt) == FDIR_SAFE) { out.force = cfg::SAFE_FORCE_N; out.fault = 1U; return; }
    out = o2;
    if (o2.fault) { on_detection(rt, DET_NONFINITE, pr, s, t); if (runtime_fdir(rt) == FDIR_SAFE) out.force = cfg::SAFE_FORCE_N; }
    if (!std::isfinite(out.force) || out.force > cfg::FORCE_LIMIT_N || out.force < -cfg::FORCE_LIMIT_N) {
        out.force = cfg::SAFE_FORCE_N; out.fault = 1U; on_detection(rt, DET_NONFINITE, pr, s, t);
    }
}

/* DEGRADED law: swing-up inhibited, output clamp halved (applied after step(), not inside it). */
static void apply_degraded_law(const Runtime& rt, const State& s, Output& out) {
    if (runtime_fdir(rt) != FDIR_DEGRADED) return;
    if (s.mode == MODE_SWING) { out.force = cfg::SAFE_FORCE_N; out.fault = 1U; out.saturated = 0U; }
    const F32 lim = cfg::FORCE_LIMIT_N * cfg::DEGRADED_CLAMP_FRAC;
    if (out.force > lim) { out.force = lim; out.saturated = 1U; }
    else if (out.force < -lim) { out.force = -lim; out.saturated = 1U; }
}

/* A clean tick counts toward leaving RECOVERING / DEGRADED. A corrupt counter is SAFE (and this tick's command with it). */
static void count_clean_tick(Runtime& rt, const TickTlm& t, Output& out) {
    if (t.det != DET_NONE && t.det != DET_SEC) return;
    const FdirState fs = runtime_fdir(rt);
    if (fs != FDIR_RECOVERING && fs != FDIR_DEGRADED) return;
    if (!mirror_ok_else_safe(rt, rt.clean_ticks, rt.clean_ticks_inv)) { out.force = cfg::SAFE_FORCE_N; out.fault = 1U; out.saturated = 0U; return; }
    const U32 n = rt.clean_ticks + 1U;
    set_clean_ticks(rt, n);
    if (n >= (fs == FDIR_RECOVERING ? cfg::FDIR_CLEAN_TICKS : cfg::FDIR_DEGRADED_CLEAN_TICKS)) fdir_set(rt, FDIR_NOMINAL);
}
#endif  /* FSW_PROTECT: control helpers */

void runtime_control(Runtime& rt, const Params& p1, const Params& p2, State& s, const Input& in, Output& out, TickTlm& t) {
    out.fault = 0U; out.saturated = 0U; out.pad[0] = 0U; out.pad[1] = 0U;
    if (runtime_fdir(rt) == FDIR_SAFE) {
        out.force = cfg::SAFE_FORCE_N; out.fault = 1U; t.fdir = FDIR_SAFE;
        t.mode = 0U;   // SAFE ticks report mode 0 (fail-safe telemetry)
        t.sec_total = rt.sec_total; t.reload_total = rt.reload_total; t.det_total = rt.det_total;
        return;
    }
#if FSW_PROTECT
    Params pr = p2;
    Input inp = in;                                     // local copy: the caller's sample is never modified
    bool persistent = false;
    if (guard_input(rt, inp, pr, s, t, persistent)) {   // no history and a non-finite sample: nothing safe to feed the law
        out.force = cfg::SAFE_FORCE_N; out.fault = 1U;
        if (runtime_fdir(rt) != FDIR_SAFE) encode_state(rt, s);
    } else {
        Output o2;
        if (persistent) dual_step(rt, pr, pr, pr, s, inp, t, o2);
        else            dual_step(rt, p1, p2, pr, s, inp, t, o2);
        guard_output(rt, o2, pr, s, t, out);
        apply_degraded_law(rt, s, out);
        encode_state(rt, s);
        count_clean_tick(rt, t, out);
    }
#else
    (void)p2;
    step(p1, in, s, out);
    rt.s = s;
#endif
    t.fdir = static_cast<U8>(runtime_fdir(rt)); t.mode = s.mode;
    t.sec_total = rt.sec_total; t.reload_total = rt.reload_total; t.det_total = rt.det_total;
}

void runtime_tick(Runtime& rt, const Input& in, Output& out, TickTlm& t) {
    Params p; State s;
    runtime_decode(rt, p, s, t);
    runtime_control(rt, p, p, s, in, out, t);
}

void runtime_notify_stall(Runtime& rt) { rt.stall_pending = 1U; }

} // namespace fsw
