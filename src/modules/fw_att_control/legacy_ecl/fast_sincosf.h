/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file fast_sincosf.h
 * A truly-fused sincosf: one Cody-Waite argument reduction shared for both
 * sin and cos, instead of two (the reduction, not the polynomial, is the
 * expensive part on a no-hardware-trig target). newlib's libm sincosf on
 * Cortex-M7 calls sinf() then cosf() internally and reduces the argument
 * twice, so it does not give this benefit; this is a from-scratch fused
 * implementation for that target.
 *
 * Valid for |x| <= ~1e4 (control angles here are within +-pi). Single-
 * precision minimax polynomials; accuracy <= 3.6e-6 vs libm over that range.
 */

#pragma once

#include <cmath>

static inline void fast_sincosf(float x, float *s, float *c)
{
	// reduce to quadrant: k = round(x * 2/pi); r = x - k*(pi/2)
	const float two_over_pi = 0.63661977236758134f;
	float kf = __builtin_rintf(x * two_over_pi);
	int k = (int)kf;

	// Cody-Waite: pi/2 split in 3 parts for accuracy
	float r = x;
	r -= kf * 1.5707963109016418f;     // pi/2 hi
	r -= kf * 1.5893254712295857e-8f;  // pi/2 mid
	r -= kf * 6.0770617817619014e-16f; // pi/2 lo (negligible in float, kept for range)

	float r2 = r * r;
	// sin(r) ~ r + r^3*(s1 + r^2*(s2 + r^2*s3)), cos(r) ~ 1 + r^2*(c1 + r^2*(c2 + r^2*c3))
	float sp = r * (1.0f + r2 * (-0.16666667f + r2 * (0.0083333310f + r2 * (-0.00019840874f))));
	float cp = 1.0f + r2 * (-0.5f + r2 * (0.041666638f + r2 * (-0.0013888397f)));

	// select by quadrant k mod 4
	switch (k & 3) {
	case 0: *s =  sp; *c =  cp; break;

	case 1: *s =  cp; *c = -sp; break;

	case 2: *s = -sp; *c = -cp; break;

	default:*s = -cp; *c =  sp; break;
	}
}
