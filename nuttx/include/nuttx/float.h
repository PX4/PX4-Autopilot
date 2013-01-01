/****************************************************************************
 * include/nuttx/float.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: http://pubs.opengroup.org/onlinepubs/009695399/basedefs/float.h.html
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_FLOAT_H
#define __INCLUDE_NUTTX_FLOAT_H

/* TODO:  These values could vary with architectures toolchains.  This
 * logic should be move at least to the include/arch directory.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Radix of exponent representation, b. */

#define FLT_RADIX 2

/* Number of base-FLT_RADIX digits in the floating-point significand, p. */

#define FLT_MANT_DIG 24

#if CONFIG_HAVE_DOUBLE
#  define DBL_MANT_DIG 53
#else
#  define DBL_MANT_DIG FLT_MANT_DIG
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MANT_DIG DBL_MANT_DIG /* FIX ME */
#else
#  define LDBL_MANT_DIG DBL_MANT_DIG
#endif

/* Number of decimal digits, n, such that any floating-point number in the
 * widest supported floating type with pmax radix b digits can be rounded
 * to a floating-point number with n decimal digits and back again without
 * change to the value.
 */

#define DECIMAL_DIG 10

/* Number of decimal digits, q, such that any floating-point number with q
 * decimal digits can be rounded into a floating-point number with p radix
 * b digits and back again without change to the q decimal digits.
 */

#define FLT_DIG 6

#if CONFIG_HAVE_DOUBLE
#  define DBL_DIG 15  /* 10 */
#else
#  define DBL_DIG FLT_DIG
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_DIG DBL_DIG  /* FIX ME */
#else
#  define LDBL_DIG DBL_DIG
#endif

/* Minimum negative integer such that FLT_RADIX raised to that power minus
 * 1 is a normalized floating-point number, emin.
 */

#define FLT_MIN_EXP (-125)

#if CONFIG_HAVE_DOUBLE
#  define DBL_MIN_EXP (-1021)
#else
#  define DBL_MIN_EXP FLT_MIN_EXP
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MIN_EXP DBL_MIN_EXP /* FIX ME */
#else
#  define LDBL_MIN_EXP DBL_MIN_EXP
#endif

/* inimum negative integer such that 10 raised to that power is in the range
 * of normalized floating-point numbers.
 */

#define FLT_MIN_10_EXP (-37)

#if CONFIG_HAVE_DOUBLE
#  define DBL_MIN_10_EXP (-307)  /* -37 */
#else
#  define DBL_MIN_10_EXP FLT_MIN_10_EXP
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MIN_10_EXP DBL_MIN_10_EXP  /* FIX ME */
#else
#  define LDBL_MIN_10_EXP DBL_MIN_10_EXP
#endif

/* Maximum integer such that FLT_RADIX raised to that power minus 1 is a
 * representable finite floating-point number, emax.
 */

#define FLT_MAX_EXP 128

#if CONFIG_HAVE_DOUBLE
#  define DBL_MAX_EXP 1024
#else
#  define DBL_MAX_EXP FLT_MAX_EXP
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MAX_EXP DBL_MAX_EXP /* FIX ME */
#else
#  define LDBL_MAX_EXP DBL_MAX_EXP
#endif

/* Maximum integer such that 10 raised to that power is in the range of
 * representable finite floating-point numbers.
 */

#define FLT_MAX_10_EXP 38  /* 37 */

#if CONFIG_HAVE_DOUBLE
#  define DBL_MAX_10_EXP 308  /* 37 */
#else
#  define DBL_MAX_10_EXP FLT_MAX_10_EXP
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MAX_10_EXP DBL_MAX_10_EXP  /* FIX ME */
#else
#  define LDBL_MAX_10_EXP DBL_MAX_10_EXP
#endif

/* Maximum representable finite floating-point number. */

#define FLT_MAX 3.40282347e+38F  /* 1E+37 */

#if CONFIG_HAVE_DOUBLE
#  define DBL_MAX 1.7976931348623157e+308  /* 1E+37 */
#else
#  define DBL_MAX FLT_MAX
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MAX DBL_MAX  /* FIX ME */
#else
#  define LDBL_MAX DBL_MAX
#endif

/* The difference between 1 and the least value greater than 1 that is
 * representable in the given floating-point type, b1-p.
 */

#define FLT_EPSILON 1.1920929e-07F  /* 1E-5 */

#if CONFIG_HAVE_DOUBLE
#  define DBL_EPSILON 2.2204460492503131e-16  /* 1E-9 */
#else
#  define DBL_EPSILON FLT_EPSILON
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_EPSILON DBL_EPSILON /* FIX ME */
#else
#  define LDBL_EPSILON DBL_EPSILON
#endif

/* Minimum normalized positive floating-point number, bemin -1. */

#define FLT_MIN 1.17549435e-38F  /* 1E-37 */

#if CONFIG_HAVE_DOUBLE
#define DBL_MIN 2.2250738585072014e-308  /* 1E-37 */
#else
#  define DBL_MIN FLT_MIN
#endif

#ifdef CONFIG_HAVE_LONG_DOUBLE
#  define LDBL_MIN DBL_MIN /* FIX ME */
#else
#  define LDBL_MIN DBL_MIN
#endif

#endif /* __INCLUDE_NUTTX_FLOAT_H */
