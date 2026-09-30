/** @file main.cpp  @brief Sim-side harness: plant + actuator monitor + injector around the flight runtime. CSV to stdout.
 *  Built twice from this file: cartpole_demo (FSW_PROTECT=1) and cartpole_baseline (FSW_PROTECT=0).
 *
 *  Injection (SPEC-02 §3, sim side only; the flight core has no knowledge of it):
 *    --flip <field>:<bit>[+<bit>...]   XOR bits of one Params word in the store (protected) / in RAM (baseline) at --flip-tick
 *    --flip-state <field>:<bit>        same for State: integ thdf tick flags
 *    --flip-check <field>:<bit>        a SECDED check bit of that Params word (protected build only)
 *    --flip-crc <bit>                  the Params CRC word (protected build only)
 *    --flip-local <field>:<bit>        corrupt lane-1's decoded copy for one tick (compute window; protected build)
 *    --flip-input <theta|x|xdot|thetadot>:<bit>   one sensor sample at --flip-tick
 *    --stall <k>                       controller skipped for k ticks starting at --flip-tick; actuator holds the last
 *                                      command for at most STALE_MAX_TICKS, then applies the fallback (0 N)
 *  Params fields: k0 k1 k2 k3 ki ilim alpha swke swamax swamin swkx swkv sweref swenter swrate swexit mM mm ml mg
 */
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "fsw/Runtime.hpp"
#include "fsw/Plant.hpp"

using namespace fsw;

static const char* PFIELDS[20] = { "k0", "k1", "k2", "k3", "ki", "ilim", "alpha", "swke", "swamax", "swamin", "swkx", "swkv",
                                   "sweref", "swenter", "swrate", "swexit", "mM", "mm", "ml", "mg" };
static const char* SFIELDS[4] = { "integ", "thdf", "tick", "flags" };
static const char* IFIELDS[4] = { "x", "xdot", "theta", "thetadot" };

/* parse "field:b1+b2" against a name table; returns field index or -1; fills mask */
static int parse_target(const char* arg, const char* const* names, int n, U32& mask) {
    const char* colon = std::strchr(arg, ':'); if (!colon) return -1;
    int f = -1;
    for (int i = 0; i < n; ++i)
        if (std::strlen(names[i]) == static_cast<size_t>(colon - arg) && std::strncmp(arg, names[i], std::strlen(names[i])) == 0) f = i;
    mask = 0U; const char* s = colon + 1;
    while (*s) { const int b = std::atoi(s); if (b < 0 || b > 31) return -1; mask |= (1U << b); const char* plus = std::strchr(s, '+'); if (!plus) break; s = plus + 1; }
    return (mask == 0U) ? -1 : f;
}

static F32 xor_f32(F32 v, U32 mask) { U32 w; std::memcpy(&w, &v, 4); w ^= mask; std::memcpy(&v, &w, 4); return v; }

int main(int argc, char** argv) {
    double x0 = 0.0, th0 = 0.0, secs = 5.0, dt = cfg::DT_S; int substeps = 1;
    U32 flip_tick = 0; int pf = -1, sf = -1, cf = -1, lf = -1, inf = -1; U32 pmask = 0, smask = 0, cmask = 0, crcmask = 0, lmask = 0, imask = 0;
    int stall = 0;
    PlantParams pp; plant_params_default(pp);           /* sim plant; the flags below mistune it, the controller keeps its nominal model */
    for (int i = 1; i + 1 < argc; i += 2) {
        const char* a = argv[i]; const char* v = argv[i + 1];
        if (!std::strcmp(a, "--x0")) x0 = std::atof(v);
        else if (!std::strcmp(a, "--theta0")) th0 = std::atof(v);
        else if (!std::strcmp(a, "--seconds")) secs = std::atof(v);
        else if (!std::strcmp(a, "--cart-mass")) pp.cart_mass = std::atof(v);
        else if (!std::strcmp(a, "--pole-mass")) pp.pole_mass = std::atof(v);
        else if (!std::strcmp(a, "--half-len")) pp.half_len = std::atof(v);
        else if (!std::strcmp(a, "--dt")) dt = std::atof(v);
        else if (!std::strcmp(a, "--substeps")) substeps = std::atoi(v);
        else if (!std::strcmp(a, "--flip-tick")) flip_tick = static_cast<U32>(std::atoi(v));
        else if (!std::strcmp(a, "--flip")) { pf = parse_target(v, PFIELDS, 20, pmask); if (pf < 0) { std::fprintf(stderr, "bad --flip\n"); return 2; } }
        else if (!std::strcmp(a, "--flip-state")) { sf = parse_target(v, SFIELDS, 4, smask); if (sf < 0) { std::fprintf(stderr, "bad --flip-state\n"); return 2; } }
        else if (!std::strcmp(a, "--flip-check")) { cf = parse_target(v, PFIELDS, 20, cmask); if (cf < 0 || cmask > 0x7FU) { std::fprintf(stderr, "bad --flip-check (bits 0-6)\n"); return 2; } }
        else if (!std::strcmp(a, "--flip-crc")) { const int b = std::atoi(v); if (b < 0 || b > 31) return 2; crcmask = 1U << b; }
        else if (!std::strcmp(a, "--flip-local")) { lf = parse_target(v, PFIELDS, 20, lmask); if (lf < 0) { std::fprintf(stderr, "bad --flip-local\n"); return 2; } }
        else if (!std::strcmp(a, "--flip-input")) { inf = parse_target(v, IFIELDS, 4, imask); if (inf < 0) { std::fprintf(stderr, "bad --flip-input\n"); return 2; } }
        else if (!std::strcmp(a, "--stall")) stall = std::atoi(v);
        else if (!std::strcmp(a, "--gains")) { /* accepted for compatibility; gains are always designed on board now */ }
        else { std::fprintf(stderr, "usage: see the header of fsw/src/main.cpp\n"); return 2; }
    }
    if (!(dt > 0.0) || dt > 1.0 || substeps < 1 || substeps > 1000) { std::fprintf(stderr, "bad --dt / --substeps\n"); return 2; }

    PlantParams model; plant_params_default(model);
    Runtime rt; if (!runtime_init(rt, model)) { std::fprintf(stderr, "runtime_init failed (Riccati)\n"); return 3; }
    PlantState ps = { x0, 0.0, th0, 0.0 };
    const U32 n = static_cast<U32>(secs / dt);
    U32 stall_left = 0; F32 last_force = 0.0f; U32 stale_ticks = 0;

    std::printf("tick,t,x,xdot,theta,thetadot,u,fault,u_x,u_xd,u_th,u_thd,u_i,integ,sat,mode,det,fdir,sec,reload\n");
    for (U32 k = 0; k < n; ++k) {
        /* ---- injection at rest: the stored words / check bits / CRC (protected) or RAM (baseline) ---- */
        if (k == flip_tick) {
#if FSW_PROTECT
            if (pf >= 0) { const U32 b = rt.pstore.words[pf]; rt.pstore.words[pf] = b ^ pmask; std::fprintf(stderr, "flip tick=%u params[%s] 0x%08X->0x%08X\n", k, PFIELDS[pf], b, b ^ pmask); }
            if (sf >= 0) { const U32 b = rt.sstore.words[sf]; rt.sstore.words[sf] = b ^ smask; std::fprintf(stderr, "flip tick=%u state[%s] 0x%08X->0x%08X\n", k, SFIELDS[sf], b, b ^ smask); }
            if (cf >= 0) { const U8 b = rt.pstore.check[cf]; rt.pstore.check[cf] = static_cast<U8>(b ^ cmask); std::fprintf(stderr, "flip tick=%u check[%s] 0x%02X->0x%02X\n", k, PFIELDS[cf], b, b ^ cmask); }
            if (crcmask) { const U32 b = rt.pstore.crc; rt.pstore.crc = b ^ crcmask; std::fprintf(stderr, "flip tick=%u params-crc 0x%08X->0x%08X\n", k, b, b ^ crcmask); }
#else
            if (pf >= 0) { U32 w; std::memcpy(&w, reinterpret_cast<U8*>(&rt.p) + 4 * pf, 4); const U32 b = w; w ^= pmask; std::memcpy(reinterpret_cast<U8*>(&rt.p) + 4 * pf, &w, 4); std::fprintf(stderr, "flip tick=%u params[%s] 0x%08X->0x%08X\n", k, PFIELDS[pf], b, w); }
            if (sf >= 0) { U32 w; std::memcpy(&w, reinterpret_cast<U8*>(&rt.s) + 4 * sf, 4); w ^= smask; std::memcpy(reinterpret_cast<U8*>(&rt.s) + 4 * sf, &w, 4); std::fprintf(stderr, "flip tick=%u state[%s]\n", k, SFIELDS[sf]); }
            if (cf >= 0 || crcmask) std::fprintf(stderr, "note: baseline has no check bits / CRC; --flip-check/--flip-crc ignored\n");
#endif
            if (stall > 0) { stall_left = static_cast<U32>(stall); std::fprintf(stderr, "stall tick=%u for %d ticks\n", k, stall); }
        }
        Input in = { static_cast<F32>(ps.x), static_cast<F32>(ps.xdot), static_cast<F32>(ps.theta), static_cast<F32>(ps.thetadot), static_cast<F32>(dt) };
        if (inf >= 0 && k == flip_tick) {                 /* sensor sample corruption */
            F32* f = (inf == 0) ? &in.x : (inf == 1) ? &in.xdot : (inf == 2) ? &in.theta : &in.thetadot;
            const F32 b = *f; *f = xor_f32(*f, imask); std::fprintf(stderr, "flip tick=%u input[%s] %g -> %g\n", k, IFIELDS[inf], static_cast<double>(b), static_cast<double>(*f));
        }

        Output out; TickTlm t; std::memset(&t, 0, sizeof t);
        if (stall_left > 0) {                             /* controller does not run; actuator-side freshness monitor */
            stall_left--; stale_ticks++;
            out.force = (stale_ticks <= cfg::STALE_MAX_TICKS) ? last_force : cfg::SAFE_FORCE_N;  /* ZOH, then fallback */
            out.fault = (stale_ticks > cfg::STALE_MAX_TICKS) ? 1U : 0U; out.saturated = 0U;
            t.fdir = static_cast<U8>(runtime_fdir(rt)); t.det = DET_STALL;
            if (stall_left == 0) runtime_notify_stall(rt);
        } else {
            stale_ticks = 0;
            Params p; State s;
            runtime_decode(rt, p, s, t);
            Params p1 = p;
            if (lf >= 0 && k == flip_tick) {              /* compute-window: corrupt lane 1's decoded copy only */
                F32* f = reinterpret_cast<F32*>(&p1) + lf; *f = xor_f32(*f, lmask);
                std::fprintf(stderr, "flip tick=%u local[%s] (lane 1 only)\n", k, PFIELDS[lf]);
            }
            runtime_control(rt, p1, p, s, in, out, t);
            last_force = out.force;
        }

        /* per-term telemetry for the viewer (from the same law the controller used; zero during a stall) */
        Params pv; params_default(pv); State sv; std::memset(&sv, 0, sizeof sv);
        const double u_x = -static_cast<double>(pv.k[0]) * in.x, u_xd = -static_cast<double>(pv.k[1]) * in.xdot;
        const double u_th = -static_cast<double>(pv.k[2]) * in.theta, u_thd = -static_cast<double>(pv.k[3]) * in.thetadot;
        std::printf("%u,%.3f,%.5f,%.5f,%.5f,%.5f,%.4f,%u,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%u,%u,%u,%u,%u,%u\n",
                    k, static_cast<double>(k) * dt, ps.x, ps.xdot, ps.theta, ps.thetadot, static_cast<double>(out.force), out.fault,
                    u_x, u_xd, u_th, u_thd, 0.0, 0.0, out.saturated, t.mode, t.det, t.fdir, t.sec_total, t.reload_total);
        for (int j = 0; j < substeps; ++j) plant_step(pp, ps, static_cast<F64>(out.force), dt / substeps);   /* ZOH on u */
    }
    return 0;
}
