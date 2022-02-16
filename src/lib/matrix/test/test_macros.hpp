/**
 * @file test_marcos.hpp
 *
 * Helps with cmake testing.
 *
 * @author James Goppert <james.goppert@gmail.com>
 *         Pavel Kirienko <pavel.kirienko@zubax.com>
 */
#pragma once

#include <cstdio>
#include <cmath>    // cmath has to be introduced BEFORE we poison the C library identifiers

#define TEST(X) if(!(X)) { fprintf(stderr, "test failed on %s:%d\n", __FILE__, __LINE__); return -1;}

/**
 * This construct is needed to catch any unintended use of the C standard library.
 * Feel free to extend the list of poisoned identifiers with as many C functions as possible.
 * The current list was constructed by means of automated parsing of http://en.cppreference.com/w/c/numeric/math
 */
#ifdef __GNUC__

// float functions
# pragma GCC poison fabsf fmodf
# pragma GCC poison remainderf remquof fmaf fmaxf fminf fdimf fnanf expf
# pragma GCC poison exp2f expm1f logf log10f log2f log1pf powf sqrtf cbrtf
# pragma GCC poison hypotf sinf cosf tanf asinf acosf atanf atan2f sinhf
# pragma GCC poison coshf tanhf asinhf acoshf atanhf erff erfcf tgammaf
# pragma GCC poison lgammaf ceilf floorf truncf roundf nearbyintf rintf
# pragma GCC poison frexpf ldexpf modff scalbnf ilogbf logbf nextafterf
# pragma GCC poison copysignf

// the list of double functions is missing because otherwise most functions from std:: would be also poisoned,
// which we don't want

// long double functions
# pragma GCC poison fabsl fabsl fabsl fmodl
# pragma GCC poison remainderl remquol fmal fmaxl fminl fdiml fnanl expl
# pragma GCC poison exp2l expm1l logl log10l log2l log1pl powl sqrtl cbrtl
# pragma GCC poison hypotl sinl cosl tanl asinl acosl atanl atan2l sinhl
# pragma GCC poison coshl tanhl asinhl acoshl atanhl erfl erfcl tgammal
# pragma GCC poison lgammal ceill floorl truncl roundl nearbyintl rintl
# pragma GCC poison frexpl ldexpl modfl scalbnl ilogbl logbl nextafterl
# pragma GCC poison copysignl

#endif
