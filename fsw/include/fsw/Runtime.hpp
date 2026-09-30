/** @file Runtime.hpp  @brief The tick wrapper around the pure control law: encoded store -> decode (SECDED, then CRC)
 *  -> range table -> dual execution of step() -> output guard -> FDIR -> encode. One source, two builds:
 *    FSW_PROTECT=1  protected (the ladder above)
 *    FSW_PROTECT=0  baseline  (plain Params/State in RAM, step() once, no detection, no FDIR)
 *  The API is split in two so a sim-side harness can corrupt the decoded local copy between decode and control
 *  (compute-window injection) without any hook inside the flight core.
 */
#ifndef FSW_RUNTIME_HPP
#define FSW_RUNTIME_HPP
#include "fsw/Controller.hpp"
#include "fsw/Protect.hpp"

#ifndef FSW_PROTECT
#define FSW_PROTECT 1
#endif

namespace fsw {

enum Detection { DET_NONE = 0, DET_SEC = 1, DET_DED = 2, DET_CRC = 3, DET_RANGE = 4, DET_MISMATCH = 5,
                 DET_NONFINITE = 6, DET_STALL = 7, DET_GOLDEN = 8 };
enum FdirState { FDIR_NOMINAL = 0, FDIR_RECOVERING = 1, FDIR_DEGRADED = 2, FDIR_SAFE = 3 };

static const U32 PARAM_WORDS = static_cast<U32>(sizeof(Params) / 4U);
static const U32 STATE_WORDS = static_cast<U32>(sizeof(State) / 4U);

/** Per-tick telemetry. */
struct TickTlm {
    U8  det;          /**< Detection this tick (first detector wins) */
    U8  fdir;         /**< FdirState after this tick */
    U8  mode;         /**< controller mode (SWING / BALANCE) */
    U8  pad;
    U32 sec_total;    /**< single-bit corrections since init */
    U32 reload_total; /**< verified golden reloads since init */
    U32 det_total;    /**< detections since init (excluding SEC) */
};

struct Runtime {
    PlantParams model;              /**< nominal model: the golden source (K is re-derived from it on reload) */
    U32 golden_crc, golden_crc_inv; /**< pinned at init over the designed Params; a reload must reproduce it */
#if FSW_PROTECT
    EncodedStore<PARAM_WORDS> pstore;
    EncodedStore<STATE_WORDS> sstore;
#else
    Params p;
    State  s;
#endif
    U32 fdir_state, fdir_state_inv; /**< (v, ~v): a flipped state word reads as SAFE, never as "more permissive" */
    U32 det_count, det_count_inv;   /**< detections in the current persistence window, (v, ~v) */
    U32 window_start;               /**< tick the window opened */
    U32 clean_ticks;                /**< consecutive clean ticks while RECOVERING or DEGRADED */
    U32 stall_pending;              /**< harness reported skipped ticks; handled on the next decode */
    U32 sec_total, reload_total, det_total, tick;
};

/** @brief Design the golden Params from the model, pin its CRC, encode the stores, FDIR = NOMINAL. false if the
 *  Riccati solve fails. */
bool runtime_init(Runtime& rt, const PlantParams& model);

/**
 * @brief Decode the stores into local copies (SECDED -> CRC -> range table). Any detection reloads the verified
 *        golden into `p`, reseeds `s`, and moves FDIR to RECOVERING (or SAFE on persistence / untrusted golden).
 * @return false when FDIR is SAFE (the caller still calls runtime_control, which outputs the safe command).
 */
bool runtime_decode(Runtime& rt, Params& p, State& s, TickTlm& t);

/**
 * @brief Dual-execute step() on (p1, s) and (p2, s); a mismatch is a detection (compute window). Then guard the
 *        output, run FDIR bookkeeping and re-encode the state store. Flight code passes the same Params twice; a
 *        harness may pass a corrupted copy as p1 to test the compare.
 */
void runtime_control(Runtime& rt, const Params& p1, const Params& p2, State& s, const Input& in, Output& out, TickTlm& t);

/** @brief The flight-side one-call tick: decode then control with the same copy in both lanes. */
void runtime_tick(Runtime& rt, const Input& in, Output& out, TickTlm& t);

/** @brief Harness: the controller was not run for some ticks (stall). Treated as a detection on the next decode. */
void runtime_notify_stall(Runtime& rt);

FdirState runtime_fdir(const Runtime& rt);

} // namespace fsw
#endif
