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
};

/** the rows, generator 1 is the fBm terrain, version 1 is the rules as first shipped */
static constexpr Rules kRules[] = {
	{0x0101, 5.f, 200.f, 6, 0.6155722f, 1.f, 1.5f},
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

} // namespace terrain_sim
