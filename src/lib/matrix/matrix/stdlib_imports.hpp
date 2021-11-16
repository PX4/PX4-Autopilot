/**
 * @file stdlib_imports.hpp
 *
 * This file is needed to shadow the C standard library math functions with ones provided by the C++ standard library.
 * This way we can guarantee that unwanted functions from the C library will never creep back in unexpectedly.
 *
 * @author Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <cmath>
#include <cstdlib>
#include <inttypes.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI (M_PI * 2.0)
#endif

namespace matrix {

#if !defined(FLT_EPSILON)
#define FLT_EPSILON     __FLT_EPSILON__
#endif

#if defined(__PX4_NUTTX)
/*
 * NuttX has no usable C++ math library, so we need to provide the needed definitions here manually.
 */
#define MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(name)                                   \
	inline float       name(float x)       { return ::name##f(x); }          \
	inline double      name(double x)      { return ::name(x); }             \
	inline long double name(long double x) { return ::name##l(x); }

#define MATRIX_NUTTX_WRAP_MATH_FUN_BINARY(name)                                                    \
	inline float       name(float x, float y)             { return ::name##f(x, y); }          \
	inline double      name(double x, double y)           { return ::name(x, y); }             \
	inline long double name(long double x, long double y) { return ::name##l(x, y); }

MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(fabs)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(log)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(log10)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(exp)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(sqrt)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(sin)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(cos)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(tan)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(asin)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(acos)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(atan)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(sinh)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(cosh)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(tanh)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(ceil)
MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(floor)

MATRIX_NUTTX_WRAP_MATH_FUN_BINARY(pow)
MATRIX_NUTTX_WRAP_MATH_FUN_BINARY(atan2)

#else       // Not NuttX, using the C++ standard library

using std::abs;
using std::div;
using std::fabs;
using std::fmod;
using std::exp;
using std::log;
using std::log10;
using std::pow;
using std::sqrt;
using std::sin;
using std::cos;
using std::tan;
using std::asin;
using std::acos;
using std::atan;
using std::atan2;
using std::sinh;
using std::cosh;
using std::tanh;
using std::ceil;
using std::floor;
using std::frexp;
using std::ldexp;
using std::modf;

# if (__cplusplus >= 201103L)

using std::remainder;
using std::remquo;
using std::fma;
using std::fmax;
using std::fmin;
using std::fdim;
using std::nan;
using std::nanf;
using std::nanl;
using std::exp2;
using std::expm1;
using std::log2;
using std::log1p;
using std::cbrt;
using std::hypot;
using std::asinh;
using std::acosh;
using std::atanh;
using std::erf;
using std::erfc;
using std::tgamma;
using std::lgamma;
using std::trunc;
using std::round;
using std::nearbyint;
using std::rint;
using std::scalbn;
using std::ilogb;
using std::logb;
using std::nextafter;
using std::copysign;
using std::fpclassify;
using std::isfinite;
using std::isinf;
using std::isnan;
using std::isnormal;
using std::signbit;
using std::isgreater;
using std::isgreaterequal;
using std::isless;
using std::islessequal;
using std::islessgreater;
using std::isunordered;

# endif
#endif

}
