/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * @file terrain_sim.cpp
 *
 * Procedural terrain and a downward raycast for SIH. fBm value noise with
 * analytic derivatives and a slope damped divisor, after Quilez,
 * iquilezles.org/articles/morenoise (2008).
 *
 * No PX4 headers and no heap. The integer hash is bit exact across platforms,
 * the float field is repeatable for a given build.
 */

#include "terrain_sim.h"

#include <math.h>
#include <stdint.h>

namespace terrain_sim
{

/* set by set_params(), read by every evaluation, the only mutable state */
static struct {
	float    amp         = 0.f;
	float    wavelength  = 200.f;
	int      oct         = 6;
	float    hurst       = 0.7f;
	float    erosion     = 1.f;
	int      seed        = 0;
	float    amp_decay   = 0.6155722f;  /* 2^(-hurst), amplitude factor per octave */
	float    home_offset = 0.f;         /* raw_eval() at home, subtracted in height() */
} s_params;

static constexpr float kFlatRadiusSqM = kFlatRadiusM * kFlatRadiusM;

/* murmur3 style finalizers, uint32 multiplies and xor shifts only */
static inline uint32_t hash2d(int32_t ix, int32_t iy, int32_t seed)
{
	uint32_t h = (uint32_t)ix * 0x27d4eb2du;
	h ^= (uint32_t)iy * 0x9e3779b1u;
	h ^= (uint32_t)seed * 0x85ebca6bu;
	h ^= h >> 16;
	h *= 0x85ebca6bu;
	h ^= h >> 13;
	h *= 0xc2b2ae35u;
	h ^= h >> 16;
	return h;
}

/* upper 24 bits of the hash to [-1, 1) */
static inline float hash_to_float(uint32_t h)
{
	int32_t s = (int32_t)((h >> 8) & 0x00FFFFFFu) - 0x00800000;
	return (float)s * (1.f / (float)0x00800000);
}

/* one octave of value noise and its derivatives, quintic smoothstep so the
 * gradient is continuous across cell edges */
static void value_noise_d(float x, float y, int32_t seed,
			  float *out_n, float *out_dx, float *out_dy)
{
	float xi_f = floorf(x);
	float yi_f = floorf(y);
	float xf   = x - xi_f;
	float yf   = y - yi_f;

	int32_t ix = (int32_t)xi_f;
	int32_t iy = (int32_t)yi_f;

	float u  = xf * xf * xf * (xf * (xf * 6.f - 15.f) + 10.f);
	float du = 30.f * xf * xf * (xf * (xf - 2.f) + 1.f);
	float v  = yf * yf * yf * (yf * (yf * 6.f - 15.f) + 10.f);
	float dv = 30.f * yf * yf * (yf * (yf - 2.f) + 1.f);

	float a = hash_to_float(hash2d(ix,     iy,     seed));
	float b = hash_to_float(hash2d(ix + 1, iy,     seed));
	float c = hash_to_float(hash2d(ix,     iy + 1, seed));
	float d = hash_to_float(hash2d(ix + 1, iy + 1, seed));

	/* n(u, v) = a + (b - a) u + (c - a) v + (a - b - c + d) u v */
	float k0 = a;
	float k1 = b - a;
	float k2 = c - a;
	float k3 = a - b - c + d;

	*out_n  = k0 + k1 * u + k2 * v + k3 * u * v;
	*out_dx = (k1 + k3 * v) * du;
	*out_dy = (k2 + k3 * u) * dv;
}

/* fBm over s_params.oct octaves. Each octave rotates the input by
 * R = [[1.6, 1.2], [-1.2, 1.6]] and is damped by 1 / (1 + erosion * |grad|^2)
 * of the gradient accumulated so far. The gradient is carried through the
 * rotation by the chain rule, and the divisor is treated as a constant when
 * differentiating, as in the Quilez reference. */
static float raw_eval(float n, float e)
{
	if (fabsf(s_params.amp) < 1e-6f) {
		return 0.f;
	}

	const float k = 1.f / s_params.wavelength;
	float x = n * k;
	float y = e * k;

	float c00 = 1.f, c01 = 0.f;
	float c10 = 0.f, c11 = 1.f;

	const float R00 =  1.6f, R01 =  1.2f;
	const float R10 = -1.2f, R11 =  1.6f;

	float h  = 0.f;
	float dn = 0.f;
	float de = 0.f;
	float amp = s_params.amp;

	for (int i = 0; i < s_params.oct; ++i) {
		float nv, ndx, ndy;
		value_noise_d(x, y, s_params.seed + i, &nv, &ndx, &ndy);

		/* gradient with respect to (n, e) */
		float gn = (c00 * ndx + c10 * ndy) * k;
		float ge = (c01 * ndx + c11 * ndy) * k;

		float denom = 1.f + s_params.erosion * (dn * dn + de * de);
		float inv_denom = 1.f / denom;

		h  += amp * nv  * inv_denom;
		dn += amp * gn  * inv_denom;
		de += amp * ge  * inv_denom;

		/* rotate the input and accumulate C = R * C */
		float xr = R00 * x + R01 * y;
		float yr = R10 * x + R11 * y;
		x = xr;
		y = yr;

		float nc00 = R00 * c00 + R01 * c10;
		float nc01 = R00 * c01 + R01 * c11;
		float nc10 = R10 * c00 + R11 * c10;
		float nc11 = R10 * c01 + R11 * c11;
		c00 = nc00;
		c01 = nc01;
		c10 = nc10;
		c11 = nc11;

		amp *= s_params.amp_decay;
	}

	return h;
}

/* zero inside the flat pad, the home offset subtracted outside it */
float height(float north_m, float east_m)
{
	/* flat pad, a hard step at the radius, squared compare so no sqrtf */
	if (north_m * north_m + east_m * east_m < kFlatRadiusSqM) {
		return 0.f;
	}

	return raw_eval(north_m, east_m) - s_params.home_offset;
}

/* beams more than about 78 deg off vertical miss */
static constexpr float kRaycastMinCos = 0.2f;
/* fixed point passes, enough at the tilts a multirotor flies */
static constexpr int kRaycastPasses = 3;

float raycast(float origin_n, float origin_e, float origin_alt,
	      float dir_n, float dir_e, float dir_alt, float max_t)
{
	if (max_t <= 0.f) {
		return 0.f;
	}

	/* dir_alt is positive up, so the cosine against the down axis is -dir_alt */
	const float cos_tilt = -dir_alt;

	if (cos_tilt < kRaycastMinCos) {
		return max_t;
	}

	/* fixed point iteration on the hit, t = (alt - height(hit)) / cos_tilt,
	 * converges while slope * tan(tilt) < 1 */
	float t = (origin_alt - height(origin_n, origin_e)) / cos_tilt;

	for (int i = 0; i < kRaycastPasses; i++) {
		if (t <= 0.f) {
			return 0.f; /* at or below the surface */
		}

		if (t > max_t) {
			return max_t;
		}

		t = (origin_alt - height(origin_n + t * dir_n, origin_e + t * dir_e)) / cos_tilt;
	}

	if (t <= 0.f) {
		return 0.f;
	}

	if (t > max_t) {
		return max_t;
	}

	return t;
}

/* hurst and erosion stay fixed in the initializer above, the octave count has a setter */
void set_params(float amp, float wavelength, int seed)
{
	s_params.amp        = amp;
	s_params.wavelength = (wavelength > 1e-3f) ? wavelength : 1e-3f;
	s_params.seed       = seed;

	/* raw_eval() rather than height(), which would subtract the stale offset */
	s_params.home_offset = 0.f;
	s_params.home_offset = raw_eval(0.f, 0.f);
}

void set_octaves(int octaves)
{
	s_params.oct = (octaves < 1) ? 1 : (octaves > 9) ? 9 : octaves;

	/* the home offset depends on the octave count */
	s_params.home_offset = 0.f;
	s_params.home_offset = raw_eval(0.f, 0.f);
}

} // namespace terrain_sim
