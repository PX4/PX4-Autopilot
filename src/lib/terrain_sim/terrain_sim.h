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
 * @file terrain_sim.h
 *
 * Procedural terrain for SIH. A deterministic fBm heightfield evaluated on
 * demand from a seed, no storage and no file I/O.
 *
 * Coordinates are north and east in meters from home, height is positive up
 * and relative to the home altitude. height(0, 0) is 0 by construction.
 *
 * A pure function library with no PX4 headers and no heap, so it builds and
 * tests on the host. The integer hash is bit exact across platforms, the float
 * field is repeatable for a given build.
 */

#pragma once

#include <cstdint>

namespace terrain_sim
{

/**
 * Every constant that shapes the world, as one row the world formula names.
 *
 * The formula is the generator in the high byte and its version in the low
 * byte, the value SIH publishes in sih_terrain beside the seed. A visualizer
 * that has the row for a formula rebuilds the exact world from the seed. A
 * row is never edited. A change to any value here is the next row with the
 * next formula, so a log keeps its exact lookup forever. The seed, which
 * places the terrain, is the parameter SIH_TERR_SEED.
 */
struct Rules {
	uint16_t formula;              /**< generator in the high byte, version in the low byte */

	float amplitude_m;             /**< peak height of the terrain [m], a world a multirotor can terrain follow over */
	float wavelength_m;            /**< wavelength of the largest features [m], each octave halves it */
	int   octaves;                 /**< noise layers summed, more is finer detail at more cost */
	float amp_decay;               /**< amplitude factor per octave */
	float erosion;                 /**< divisor on each octave by the slope so far, smooths the steep faces */
	float flat_radius_m;           /**< radius of the flat pad around home [m], the field is exactly 0 inside it */

	int   band_octaves;            /**< the bands read the landform of this many coarse octaves */
	float band_height_low;         /**< rock from height, none below this share of the amplitude */
	float band_height_high;        /**< rock from height, all above this share */
	float band_slope_low;          /**< rock from slope, none below this tangent */
	float band_slope_high;         /**< rock from slope, all above this tangent */
	float snow_level;              /**< high ground that is not steep becomes snow above this */
	float sand_level;              /**< rockiness below this is grass, above it sand */
	float rock_level;              /**< rockiness above this is rock */
	int32_t breakup_salt;          /**< seeds the breakup noise apart from the terrain */
	float breakup_amp;             /**< breakup added to the rockiness */
	float breakup_wavelength_share;/**< breakup wavelength as a share of the terrain's */

	int32_t road_salt;             /**< seeds the road layer apart from the terrain */
	float road_bend_amp;           /**< the road layer's second octave as a share of the first */
	float road_half_width_m;       /**< half the road's width [m] */
	float outcrop_level;           /**< rock where the road layer rises above this */
};

/** the rows, generator 1 is the fBm terrain, version 1 is the rules as first shipped */
static constexpr Rules kRules[] = {
	{0x0101, 5.f, 200.f, 6, 0.6155722f, 1.f, 1.5f, 3, 0.30f, 0.70f, 0.268f, 0.532f, 0.5f, 0.35f, 0.65f, 0x5ca7, 0.12f, 0.125f, 0x70ad, 0.5f, 3.f, 0.8f},
};

/** the formula SIH runs, the newest row */
static constexpr uint16_t kFormula = 0x0101;

/** the shipped shape, the newest row's values, for callers and tests that name them */
static constexpr float kAmplitudeM  = kRules[0].amplitude_m;
static constexpr float kWavelengthM = kRules[0].wavelength_m;
static constexpr int   kOctaves     = kRules[0].octaves;
static constexpr float kFlatRadiusM = kRules[0].flat_radius_m;

/** the row a formula names, nullptr for a formula this header does not know */
const Rules *rules_for(uint16_t formula);

/**
 * Configure the field from the row a formula names, the call SIH and a
 * visualizer make. With enabled false the amplitude is 0 and the field is
 * flat everywhere, the formula still reported.
 *
 * @return false and the field unchanged if the formula is unknown
 */
bool set_formula(uint16_t formula, int seed, bool enabled = true);

/** the formula of the row in use */
uint16_t formula();

/** terrain height at (north, east) [m], positive up, 0 at home */
float height(float north_m, float east_m);

/**
 * Distance along a downward beam to the terrain.
 *
 * dir_alt is positive up, so a straight down beam has dir_alt = -1. The
 * direction is expected to be unit length. The march starts where the beam
 * can first reach the ground and steps a quarter of the finest octave's
 * wavelength. It bisects the step that crosses, so the reading is the first
 * crossing at any tilt, over steep slopes and over steps in the field.
 * Ground narrower than one step can be missed.
 *
 * @return distance to the hit, max_t if nothing is hit within range or the
 *         beam points more than about 78 degrees off vertical, 0 if the
 *         origin is at or below the surface
 */
float raycast(float origin_n, float origin_e, float origin_alt,
	      float dir_n, float dir_e, float dir_alt, float max_t);

/**
 * Surface band at (north, east), read from the landform of the first three
 * octaves. Rockiness rises with the landform's height, as a share of the
 * amplitude, and with its slope, and a little breakup noise keeps band edges
 * off the contours.
 *
 * Roads run where two turned octaves of a noise layer of their own cross
 * zero, a constant 6 m wide, over any ground. Rock outcrops sit where the
 * same layer peaks, about 5 % of the map on any ground.
 *
 * @return 0 low and gentle ground, 1 the transition, 2 steep ground,
 *         3 high ground that is not steep, 4 road, 0 everywhere when the
 *         amplitude is 0
 */
int surface_band(float north_m, float east_m);

/**
 * Configure the field. SIH passes the constants above and its seed
 * parameter, or amplitude 0 with the terrain off, and the tests pass worlds
 * of their own.
 *
 * @param amp         peak amplitude [m], 0 gives a flat field everywhere
 * @param wavelength  wavelength of the largest octave [m]
 * @param seed        hash seed, the same seed gives the same field
 */
void set_params(float amp, float wavelength, int seed);

/** number of noise octaves, 1 to 9, 6 by default, more is finer detail at more cost per call */
void set_octaves(int octaves);

/** uniform in [0, 1) from a xorshift32 state, the state must start nonzero */
inline float rand_uniform(uint32_t &state)
{
	state ^= state << 13; state ^= state >> 17; state ^= state << 5;
	return (float)(state >> 8) * (1.f / 16777216.f);
}

/** the state of the rare fault model, one per sensor, the generator seeded nonzero */
struct RangefinderFaults {
	uint8_t kind{0};             /**< 0 none, 1 a short return, 2 no return */
	bool burst{false};           /**< the fault continues with the burst chance */
	bool second{false};          /**< a burst always takes its second reading */
	uint32_t rng{0x9e3779b9u};   /**< xorshift32 state of the fault draws */
};

/**
 * Turns the true range along the beam into the simulated rangefinder's reading.
 *
 * The surface under the beam's hit point stretches the range the law is
 * read at, the square root of grass reflectance over the surface's at
 * 850 nm from the USGS Spectral Library version 7, by band grass, dry sand,
 * rock, melting snow and old asphalt. The return off a diffuse surface
 * weakens with the square of range, and the weaker the return the less
 * precisely the sensor can time it, so the spread grows with the square of
 * the stretched range. noise_floor_m is the standard deviation at zero
 * range. noise_at_25m is what the square law adds at 25 m over grass.
 * gauss is one standard normal draw the caller makes, so a test can hand
 * in any value.
 * A darker surface stops returning sooner, where the stretched range passes
 * the limit.
 *
 * Rare faults come on top, at the share of readings measured over 500
 * Flight Review flights by stretched range band, a short return at 40 % of
 * the range or no return, single or a burst that continues with a fixed
 * chance. They draw from the generator in the fault state, so the fault
 * sequence does not depend on the noise draws.
 *
 * @return the reading [m], never negative, max_range_m + 1 past the limit
 *         or on a no return fault
 */
float rangefinder_reading(float true_range_m, int band, float gauss, float noise_floor_m, float noise_at_25m,
			  float max_range_m, RangefinderFaults &faults);

} // namespace terrain_sim
