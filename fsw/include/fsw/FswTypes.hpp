/** @file FswTypes.hpp  @brief Fixed-width typedefs mirroring F Prime's Fw basic types. */
#ifndef FSW_TYPES_HPP
#define FSW_TYPES_HPP
#include <cstdint>
#include <cstddef>
#include <limits>

#if defined(__FAST_MATH__) || defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "fast-math is forbidden: it lets the compiler delete the isfinite guards (research/02 finding 6)"
#endif

typedef float    F32;
typedef double   F64;
typedef uint32_t U32;
typedef int32_t  I32;
typedef uint8_t  U8;

static_assert(std::numeric_limits<F32>::is_iec559, "F32 must be IEEE-754 binary32");
static_assert(std::numeric_limits<F64>::is_iec559, "F64 must be IEEE-754 binary64");
static_assert(sizeof(F32) == 4 && sizeof(F64) == 8, "unexpected float sizes");

#endif
