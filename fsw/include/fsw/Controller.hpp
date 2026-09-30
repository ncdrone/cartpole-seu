/** @file Controller.hpp  @brief Pure re-entrant full-state-feedback cart-pole controller. */
#ifndef FSW_CONTROLLER_HPP
#define FSW_CONTROLLER_HPP
#include "fsw/FswTypes.hpp"
#include "fsw/Config.hpp"
#include "fsw/Plant.hpp"

namespace fsw {

/** Configuration blob (the protected parameter store). POD, no pointers. */
struct Params {
    F32 k[4];        /**< Gains on [x, xdot, theta, thetadot]; u = -k.s */
    F32 ki_x;        /**< Integral gain on cart position */
    F32 integ_limit; /**< Integrator anti-windup clamp */
    F32 dfilt_alpha; /**< thetadot filter coefficient */
    /* The output clamp and the fault-time command are NOT parameters: they are compile-time constants
       (cfg::FORCE_LIMIT_N, cfg::SAFE_FORCE_N) so the last line of defence cannot be corrupted along with the data. */
    /* swing-up */
    F32 sw_k_energy; F32 sw_a_max; F32 sw_a_min; F32 sw_k_x; F32 sw_k_v; F32 sw_e_ref;
    F32 sw_enter_rad; F32 sw_enter_rate; F32 sw_exit_rad;
    /* the controller's plant model (for the energy computation and the accel->force map) */
    F32 m_cart; F32 m_pole; F32 half_len; F32 gravity;
};

enum { MODE_SWING = 0, MODE_BALANCE = 1 };
static_assert(sizeof(Params) == 80, "Params must be 20 x F32 (encoded store word count)");

/** Mutable controller memory (candidate for reset on recovery). */
struct State {
    F32 integ;       /**< Integral of cart position */
    F32 thdot_filt;  /**< Filtered thetadot (derivative memory) */
    U32 tick;        /**< Step counter */
    U8  filt_init;   /**< Nonzero once filter is seeded (also: mode chosen) */
    U8  mode;        /**< MODE_SWING or MODE_BALANCE (hysteresis) */
    U8  pad[2];      /**< Explicit padding for byte-wise checksums */
};
static_assert(sizeof(State) == 16, "State must be 4 x U32 (encoded store word count)");

/** Sensor snapshot and timing for one step. */
struct Input {
    F32 x;        /**< Cart position [m] */
    F32 xdot;     /**< Cart velocity [m/s] */
    F32 theta;    /**< Pole angle from upright [rad] */
    F32 thetadot; /**< Pole rate [rad/s] */
    F32 dt;       /**< Step length [s] (simulated time) */
};

/** Controller command. */
struct Output {
    F32 force;    /**< Cart force [N], clamped */
    U8  fault;    /**< Nonzero on non-finite input/intermediate */
    U8  saturated;/**< Nonzero if clamp was active */
    U8  pad[2];
};

/** @brief Fill Params with the golden table from Config.hpp (offline-solved gains). */
void params_default(Params& p);
/**
 * @brief Fill Params like params_default(), then replace the gains with the on-board Riccati solve for
 *        the given plant model. The table is the offline cross-check (test_controller f); the on-board solve is the
 *        only runtime source and init refuses to start if it does not converge (returns false).
 */
bool params_design(Params& p, const PlantParams& model);
/** @brief Zero State. */
void state_reset(State& st);
/** @brief Range table: every field the right sign and within a band of the compile-time reference. Independent of
 *  the codes; this is what catches the finite-but-wrong sign/exponent flips that isfinite() misses. */
bool params_in_range(const Params& p, const Params& ref);
/** @brief Reseed dynamic state: integrator to 0, θ̇ filter re-seeded from the next measurement (bumpless).
 *  Called after a verified golden reload; never used as a recovery by itself. */
void state_reseed(State& st);

/**
 * @brief One control step. Pure function of (params, in, st); no statics.
 * @note Integrity is verified by the caller (Runtime: decode -> CRC -> range table) before step().
 */
void step(const Params& p, const Input& in, State& st, Output& out);

} // namespace fsw
#endif
