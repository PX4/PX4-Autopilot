/****************************************************************************
 * arch/rgmp/include/math.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_RGMP_INCLUDE_MATH_H
#define __ARCH_RGMP_INCLUDE_MATH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#include <rgmp/math.h>

// following functions are not implemented by RGMP math library
// don't use them
// declared here for cmath

/* General Functions ********************************************************/

float       ceilf (float x);
#if CONFIG_HAVE_DOUBLE
//double      ceil  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double ceill (long double x);
#endif

float       floorf(float x);
#if CONFIG_HAVE_DOUBLE
//double      floor (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double floorl(long double x);
#endif

float       fabsf (float x);
#if CONFIG_HAVE_DOUBLE
//double      fabs  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fabsl (long double x);
#endif

float       modff (float x, float *iptr);
#if CONFIG_HAVE_DOUBLE
//double      modf  (double x, double *iptr);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double modfl (long double x, long double *iptr);
#endif

float       fmodf (float x, float div);
#if CONFIG_HAVE_DOUBLE
//double      fmod  (double x, double div);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double fmodl (long double x, long double div);
#endif

/* Exponential and Logarithmic Functions ************************************/

float       powf  (float b, float e);
#if CONFIG_HAVE_DOUBLE
//double      pow   (double b, double e);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double powl  (long double b, long double e);
#endif

float       expf  (float x);
#if CONFIG_HAVE_DOUBLE
//double      exp   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double expl  (long double x);
#endif

float       logf  (float x);
#if CONFIG_HAVE_DOUBLE
//double      log   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double logl  (long double x);
#endif

float       log10f(float x);
#if CONFIG_HAVE_DOUBLE
//double      log10 (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double log10l(long double x);
#endif

float       log2f (float x);
#if CONFIG_HAVE_DOUBLE
//double      log2  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double log2l (long double x);
#endif

float       sqrtf (float x);
#if CONFIG_HAVE_DOUBLE
//double      sqrt  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double sqrtl (long double x);
#endif

float       ldexpf(float x, int n);
#if CONFIG_HAVE_DOUBLE
double      ldexp (double x, int n);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double ldexpl(long double x, int n);
#endif

float       frexpf(float x, int *exp);
#if CONFIG_HAVE_DOUBLE
double      frexp (double x, int *exp);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double frexpl(long double x, int *exp);
#endif

/* Trigonometric Functions **************************************************/

float       sinf  (float x);
#if CONFIG_HAVE_DOUBLE
//double      sin   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double sinl  (long double x);
#endif

float       cosf  (float x);
#if CONFIG_HAVE_DOUBLE
//double      cos   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double cosl  (long double x);
#endif

float       tanf  (float x);
#if CONFIG_HAVE_DOUBLE
//double      tan   (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double tanl  (long double x);
#endif

float       asinf (float x);
#if CONFIG_HAVE_DOUBLE
//double      asin  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double asinl (long double x);
#endif

float       acosf (float x);
#if CONFIG_HAVE_DOUBLE
//double      acos  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double acosl (long double x);
#endif

float       atanf (float x);
#if CONFIG_HAVE_DOUBLE
//double      atan  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double atanl (long double x);
#endif

float       atan2f(float y, float x);
#if CONFIG_HAVE_DOUBLE
//double      atan2 (double y, double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double atan2l(long double y, long double x);
#endif

float       sinhf (float x);
#if CONFIG_HAVE_DOUBLE
//double      sinh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double sinhl (long double x);
#endif

float       coshf (float x);
#if CONFIG_HAVE_DOUBLE
//double      cosh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double coshl (long double x);
#endif

float       tanhf (float x);
#if CONFIG_HAVE_DOUBLE
//double      tanh  (double x);
#endif
#ifdef CONFIG_HAVE_LONG_DOUBLE
long double tanhl (long double x);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_RGMP_INCLUDE_MATH_H */
