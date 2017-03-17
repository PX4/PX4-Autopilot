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
#include <cinttypes>

namespace matrix {

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

#if (__cplusplus >= 201103L) && !defined(__PX4_NUTTX)

using std::imaxabs;
using std::imaxdiv;
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

#endif

}
