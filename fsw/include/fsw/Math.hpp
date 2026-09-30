/** @file Math.hpp  @brief Two tiny shared helpers (angle wrap, symmetric clamp). Header-only, no state. */
#ifndef FSW_MATH_HPP
#define FSW_MATH_HPP
#include <cmath>
#include "fsw/FswTypes.hpp"

namespace fsw {

/** @brief Wrap to (-pi, pi]. Computed in double and rounded once to F32, so every caller gets the same bits. */
inline F32 wrap_pi(F32 a) {
    return static_cast<F32>(std::remainder(static_cast<F64>(a), 2.0 * 3.14159265358979323846));
}

/** @brief Clamp v to [-lim, lim]. */
inline F32 clampf(F32 v, F32 lim) {
    return (v > lim) ? lim : ((v < -lim) ? -lim : v);
}

} // namespace fsw
#endif
