/** @file Config.hpp  @brief Named constants: plant, limits, timing, gains. No magic numbers elsewhere. */
#ifndef FSW_CONFIG_HPP
#define FSW_CONFIG_HPP
#include "fsw/FswTypes.hpp"

namespace fsw {
namespace cfg {

/* Timing (simulated; never wall-clock) */
static const F32 DT_S = 0.01f;            /**< Control/plant step [s] */

/* Standard plant */
static const F32 CART_MASS_KG   = 1.0f;
static const F32 POLE_MASS_KG   = 0.1f;
static const F32 POLE_HALF_LEN_M = 0.5f;  /**< Half-length l */
static const F32 GRAVITY_MS2    = 9.81f;

/* Command limits / safe values */
static const F32 FORCE_LIMIT_N  = 20.0f;  /**< |u| clamp */
static const F32 SAFE_FORCE_N   = 0.0f;   /**< Output on fault */

/* LQR cost weights: the design parameters. K is derived from these on board (Lqr.hpp) */
static const F64 LQR_Q_X     = 1.0;
static const F64 LQR_Q_XDOT  = 1.0;
static const F64 LQR_Q_THETA = 10.0;
static const F64 LQR_Q_THDOT = 1.0;
static const F64 LQR_R       = 0.1;
static const U32 LQR_MAX_ITER = 2000U;   /**< Riccati iteration cap (bounded loop) */
static const F64 LQR_TOL      = 1e-12;   /**< Riccati convergence on max |dP| */

/**
 * Golden table gains, u = -K [x, xdot, theta, thetadot]: the same design solved offline by
 * tools/compute_lqr_gains.py (scipy DARE, ZOH at DT). Used as the cross-check for the on-board
 * solve and as the fallback if the solve does not converge.
 */
static const F32 GAIN_X      =  -2.962850f;
static const F32 GAIN_XDOT   =  -5.556633f;
static const F32 GAIN_THETA  = -47.373731f;
static const F32 GAIN_THDOT  = -12.275332f;

/* Optional integrator on cart position (0 = disabled) */
static const F32 KI_X            = 0.0f;
static const F32 INTEG_LIMIT     = 5.0f;  /**< Anti-windup clamp on integrator state [m*s] */

/** thetadot low-pass coefficient in (0,1]; 1 = pass-through. */
static const F32 DFILT_ALPHA     = 1.0f;

/* Swing-up (energy shaping, Astrom-Furuta) and the handoff to LQR. All protectable parameters. */
static const F32 SW_K_ENERGY   = 12.0f;    /**< cart accel per joule of energy error [m/s^2/J] */
static const F32 SW_A_MAX      = 16.0f;   /**< swing accel saturation [m/s^2] ((M+m)*a stays under the force clamp) */
static const F32 SW_K_X        = 3.0f;    /**< cart position regulation during swing [1/s^2] */
static const F32 SW_K_V        = 3.0f;    /**< cart velocity damping during swing [1/s] */
static const F32 SW_E_REF      = 0.06f;   /**< target energy above upright [J]: arrive with a little margin */
static const F32 SW_A_MIN      = 4.0f;    /**< minimum pump accel while not near upright [m/s^2]: a model error in the
                                                energy estimate can otherwise stall the swing below the top (seen at -10% pole length) */
static const F32 SW_ENTER_RAD  = 0.55f;   /**< |theta| below this (31.5 deg, inside the measured LQR basin at every x0) AND slow -> BALANCE */
static const F32 SW_ENTER_RATE = 4.0f;    /**< |thetadot| must be below this to hand off [rad/s]; 2.5 stalled at +10% pole length, 4.0 catches every mistune corner */
static const F32 SW_EXIT_RAD   = 0.75f;   /**< |theta| above this (43 deg, the basin edge) -> back to SWING (hysteresis) */
static const F32 SW_KICK_EPS   = 1e-3f;   /**< |thetadot*cos| below this: deterministic kick instead of sign(0) */

/* FDIR persistence (ticks) and the actuator-side freshness limit */
static const U32 FDIR_WINDOW_TICKS   = 200U;  /**< persistence window (2 s at 10 ms) */
static const U32 FDIR_MAX_DETECTIONS = 3U;    /**< detections within the window -> SAFE (latched) */
static const U32 FDIR_CLEAN_TICKS    = 50U;   /**< clean ticks in RECOVERING -> NOMINAL */
static const U32 STALE_MAX_TICKS     = 10U;   /**< actuator holds a stale command this long (100 ms), then fallback */

/* Demo / test */
static const F32 TRACK_LIMIT_M   = 2.4f;  /**< Plant rail half-length (hard stop is not modelled) */

} // namespace cfg
} // namespace fsw
#endif
