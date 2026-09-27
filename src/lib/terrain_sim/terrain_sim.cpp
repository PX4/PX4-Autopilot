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

/* set by set_formula() from a row, or by set_params() for a test's own world, read by every evaluation,
 * the only mutable state */
static struct {
	const Rules *rules   = &kRules[0];
	float    amp         = 0.f;
	float    wavelength  = kRules[0].wavelength_m;
	int      oct         = kRules[0].octaves;
	float    erosion     = kRules[0].erosion;
	int      seed        = 0;
	float    amp_decay   = kRules[0].amp_decay;
	float    flat_r2     = kRules[0].flat_radius_m * kRules[0].flat_radius_m;
	float    home_offset = 0.f;         /* raw_eval() at home, subtracted in height() */
} s_params;

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

/* fBm over the first `octaves` octaves, surface_band() stops early for the slope
 * the erosion divisor is treated as a constant when differentiating, as in the reference above */
static float field_eval(float n, float e, int octaves, float *grad_n, float *grad_e)
{
	if (fabsf(s_params.amp) < 1e-6f) {
		if (grad_n) { *grad_n = 0.f; }

		if (grad_e) { *grad_e = 0.f; }

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

	for (int i = 0; i < octaves; ++i) {
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

	if (grad_n) { *grad_n = dn; }

	if (grad_e) { *grad_e = de; }

	return h;
}

static float raw_eval(float n, float e)
{
	return field_eval(n, e, s_params.oct, nullptr, nullptr);
}

/* zero inside the flat pad, the home offset subtracted outside it */
float height(float north_m, float east_m)
{
	/* flat pad, a hard step at the radius, squared compare so no sqrtf */
	if (north_m * north_m + east_m * east_m < s_params.flat_r2) {
		return 0.f;
	}

	return raw_eval(north_m, east_m) - s_params.home_offset;
}

/* beams more than about 78 deg off vertical miss */
static constexpr float kRaycastMinCos = 0.2f;
/* march step as a share of the finest octave's wavelength, ground features narrower than a step can be skipped */
static constexpr float kRaycastStepShare = 0.25f;
/* halvings of the last step, enough for sub millimeter resolution on a step of a few meters */
static constexpr int kRaycastBisections = 12;

/* highest ground the field can reach, each octave's noise stays within [-1, 1) and erosion only shrinks it */
static float height_ceiling()
{
	float sum = 0.f;
	float amp = s_params.amp;

	for (int i = 0; i < s_params.oct; ++i) {
		sum += amp;
		amp *= s_params.amp_decay;
	}

	return sum - s_params.home_offset;
}

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

	if (origin_alt <= height(origin_n, origin_e)) {
		return 0.f; /* at or below the surface */
	}

	/* march from where the beam drops under the highest ground the field can reach, so the first
	 * crossing is found even where the beam passes over a crest or a step before the ground it hits */
	float lo = fmaxf((origin_alt - height_ceiling()) / cos_tilt, 0.f);

	if (lo >= max_t) {
		return max_t;
	}

	const float step = kRaycastStepShare * s_params.wavelength / (float)(1 << ((s_params.oct > 1) ? (s_params.oct - 1) : 0));
	float hi = lo;

	/* the last step stops at the range limit and is tested there, a step longer than the beam is not a miss */
	while (origin_alt + hi * dir_alt > height(origin_n + hi * dir_n, origin_e + hi * dir_e)) {
		if (hi >= max_t) {
			return max_t;
		}

		lo = hi;
		hi = fminf(hi + step, max_t);
	}

	/* the crossing is inside the last step, halve it */
	for (int i = 0; i < kRaycastBisections && hi > lo; i++) {
		const float mid = 0.5f * (lo + hi);

		if (origin_alt + mid * dir_alt <= height(origin_n + mid * dir_n, origin_e + mid * dir_e)) {
			hi = mid;

		} else {
			lo = mid;
		}
	}

	return 0.5f * (lo + hi);
}

static float smoothstep_band(float lo, float hi, float x)
{
	const float t = fminf(fmaxf((x - lo) / (hi - lo), 0.f), 1.f);
	return t * t * (3.f - 2.f * t);
}

/* the band rule of generator 1, every number from the row in use */
int surface_band(float north_m, float east_m)
{
	if (fabsf(s_params.amp) < 1e-6f) {
		return 0;
	}

	const Rules &r = *s_params.rules;

	float gn, ge;
	const int octaves = s_params.oct < r.band_octaves ? s_params.oct : r.band_octaves;
	const float h = field_eval(north_m, east_m, octaves, &gn, &ge);
	const float slope = sqrtf(gn * gn + ge * ge);

	/* a road follows the zero crossing of a two octave noise layer of its own, the terrain's own octave turn,
	 * 36.87 deg, and a second octave at twice the frequency turned again, divided by the layer's slope so the
	 * width stays constant */
	const float kr = 1.f / s_params.wavelength;
	const float x1 = (0.8f * north_m + 0.6f * east_m) * kr;
	const float y1 = (-0.6f * north_m + 0.8f * east_m) * kr;
	float v1, d1x, d1y, v2, d2x, d2y;
	value_noise_d(x1, y1, s_params.seed ^ r.road_salt, &v1, &d1x, &d1y);
	value_noise_d(2.f * (0.8f * x1 + 0.6f * y1), 2.f * (-0.6f * x1 + 0.8f * y1), (s_params.seed ^ r.road_salt) + 1,
		      &v2, &d2x, &d2y);
	const float rv = v1 + r.road_bend_amp * v2;
	const float gx = d1x + 2.f * r.road_bend_amp * (0.8f * d2x - 0.6f * d2y);
	const float gy = d1y + 2.f * r.road_bend_amp * (0.6f * d2x + 0.8f * d2y);
	const float rgrad = sqrtf(gx * gx + gy * gy) * kr;

	/* distance to the zero line in meters is the noise value over its slope per meter */
	if (rgrad > 1e-6f && fabsf(rv) < r.road_half_width_m * rgrad) {
		return 4;
	}

	/* outcrops where the road layer peaks */
	if (rv > r.outcrop_level) {
		return 2;
	}

	const float from_height = smoothstep_band(r.band_height_low, r.band_height_high, h / s_params.amp);
	const float from_slope = smoothstep_band(r.band_slope_low, r.band_slope_high, slope);

	/* breakup noise keeps band edges off the contours */
	const float k = 1.f / (r.breakup_wavelength_share * s_params.wavelength);
	float nv, ndx, ndy;
	value_noise_d(north_m * k, east_m * k, s_params.seed ^ r.breakup_salt, &nv, &ndx, &ndy);

	const float breakup = r.breakup_amp * nv;

	/* high ground that is not steep, then steep or high ground by how strongly, as in a slope and height material rule */
	if (from_height * (1.f - from_slope) + breakup >= r.snow_level) {
		return 3;
	}

	const float rockiness = fmaxf(from_height, from_slope) + breakup;

	return (rockiness < r.sand_level) ? 0 : (rockiness < r.rock_level) ? 1 : 2;
}

const Rules *rules_for(uint16_t formula)
{
	for (const Rules &r : kRules) {
		if (r.formula == formula) {
			return &r;
		}
	}

	return nullptr;
}

uint16_t formula()
{
	return s_params.rules->formula;
}

bool set_formula(uint16_t formula, int seed, bool enabled)
{
	const Rules *r = rules_for(formula);

	if (r == nullptr) {
		return false;
	}

	s_params.rules     = r;
	s_params.oct       = r->octaves;
	s_params.erosion   = r->erosion;
	s_params.amp_decay = r->amp_decay;
	s_params.flat_r2   = r->flat_radius_m * r->flat_radius_m;
	set_params(enabled ? r->amplitude_m : 0.f, r->wavelength_m, seed);
	return true;
}

/* a test's own world on top of the row in use, the decay, erosion and bands stay the row's */
void set_params(float amp, float wavelength, int seed)
{
	s_params.amp        = amp;
	s_params.wavelength = (wavelength > 1e-3f) ? wavelength : 1e-3f;
	s_params.seed       = seed;

	/* raw_eval(), since height() would subtract the stale offset */
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

float rangefinder_reading(float true_range_m, int band, float gauss, float noise_floor_m, float noise_at_25m,
			  float max_range_m, RangefinderFaults &faults)
{
	/* sqrt of grass reflectance over the surface's at 850 nm, USGS Spectral Library version 7, by band */
	static constexpr float kSurfaceRangeScale[5] = {1.f, 1.37f, 1.82f, 0.95f, 2.12f};
	const float stretched = true_range_m * kSurfaceRangeScale[band];

	/* the floor is the spread at zero range, the square law in the stretched range adds the rest */
	static constexpr float kNoiseRefRangeM = 25.f;
	const float ratio = stretched / kNoiseRefRangeM;
	const float sigma = noise_floor_m + noise_at_25m * ratio * ratio;
	float reading = fmaxf(true_range_m + gauss * sigma, 0.f);

	/* a darker surface stops returning sooner, where the stretched range passes the limit */
	if (stretched > max_range_m) {
		reading = max_range_m + 1.f;
	}

	/* rare faults at the share of readings measured over 500 Flight Review flights, by stretched range band,
	 * a short return at 40 % of the range or no return, single or a burst that continues with a fixed chance */
	static constexpr float kFaultBand[5] = {2.f, 5.f, 10.f, 20.f, 40.f};
	static constexpr float kShortRate[6] = {0.f, 6.4e-5f, 6.0e-5f, 1.69e-4f, 1.2e-4f, 1.261e-3f};
	static constexpr float kInvalidRate[6] = {1.103e-3f, 7.92e-4f, 9.35e-4f, 3.45e-3f, 8.2e-5f, 2.337e-3f};
	static constexpr float kSingleShare[2] = {0.63f, 0.41f};
	static constexpr float kBurstStay[2] = {0.796f, 0.837f};
	/* the measured shares count every reading of a burst, so a fault starts at the share over the mean
	 * readings per fault, which the single share and the burst chance above give as 2.81 and 4.62 */
	static constexpr float kReadingsPerFault[2] = {2.81f, 4.62f};
	const float fault_u = rand_uniform(faults.rng);

	/* a burst always takes its second reading, then continues with the burst chance */
	if (faults.kind != 0 && !faults.second && !(faults.burst && fault_u < kBurstStay[faults.kind - 1])) {
		faults.kind = 0;
	}

	faults.second = false;

	if (faults.kind == 0) {
		int band_i = 0;

		while (band_i < 5 && stretched > kFaultBand[band_i]) { band_i++; }

		const float p_short = kShortRate[band_i] / kReadingsPerFault[0];
		const float p_invalid = kInvalidRate[band_i] / kReadingsPerFault[1];
		faults.kind = (fault_u < p_short) ? 1 : (fault_u < p_short + p_invalid) ? 2 : 0;
		faults.burst = faults.kind != 0 && rand_uniform(faults.rng) >= kSingleShare[faults.kind - 1];
		faults.second = faults.burst;
	}

	if (faults.kind == 1) {
		reading *= 0.4f;

	} else if (faults.kind == 2) {
		reading = max_range_m + 1.f;
	}

	return reading;
}

} // namespace terrain_sim
