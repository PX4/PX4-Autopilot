/**
 * @file stdlib_imports.hpp
 *
 * This file is needed to shadow the C standard library math functions with ones provided by the C++ standard library.
 * This way we can guarantee that unwanted functions from the C library will never creep back in unexpectedly.
 *
 * @author Pavel Kirienko <pavel.kirienko@zubax.com>
 */

#pragma once

#include <inttypes.h>

#include <cmath>
#include <cstdlib>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#ifndef M_TWOPI
#define M_TWOPI (M_PI * 2.0)
#endif

namespace matrix {

#if !defined(FLT_EPSILON)
#define FLT_EPSILON __FLT_EPSILON__
#endif

#if defined(__PX4_NUTTX)
/*
 * NuttX has no usable C++ math library, so we need to provide the needed definitions here manually.
 */
#define MATRIX_NUTTX_WRAP_MATH_FUN_UNARY(name)              \
	inline float name(float x) { return ::name##f(x); } \
	inline double name(double x) { return ::name(x); }  \
	inline long double name(long double x) { return ::name##l(x); }

#define MATRIX_NUTTX_WRAP_MATH_FUN_BINARY(name)                         \
	inline float name(float x, float y) { return ::name##f(x, y); } \
	inline double name(double x, double y) { return ::name(x, y); } \
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

#else  // Not NuttX, using the C++ standard library

using std::abs;
using std::acos;
using std::asin;
using std::atan;
using std::atan2;
using std::ceil;
using std::cos;
using std::cosh;
using std::div;
using std::exp;
using std::fabs;
using std::floor;
using std::fmod;
using std::frexp;
using std::ldexp;
using std::log;
using std::log10;
using std::modf;
using std::pow;
using std::sin;
using std::sinh;
using std::sqrt;
using std::tan;
using std::tanh;

#if (__cplusplus >= 201103L)

using std::acosh;
using std::asinh;
using std::atanh;
using std::cbrt;
using std::copysign;
using std::erf;
using std::erfc;
using std::exp2;
using std::expm1;
using std::fdim;
using std::fma;
using std::fmax;
using std::fmin;
using std::fpclassify;
using std::hypot;
using std::ilogb;
using std::isfinite;
using std::isgreater;
using std::isgreaterequal;
using std::isinf;
using std::isless;
using std::islessequal;
using std::islessgreater;
using std::isnan;
using std::isnormal;
using std::isunordered;
using std::lgamma;
using std::log1p;
using std::log2;
using std::logb;
using std::nan;
using std::nanf;
using std::nanl;
using std::nearbyint;
using std::nextafter;
using std::remainder;
using std::remquo;
using std::rint;
using std::round;
using std::scalbn;
using std::signbit;
using std::tgamma;
using std::trunc;

#endif
#endif

}  // namespace matrix
