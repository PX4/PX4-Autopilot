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

namespace terrain_sim
{

/** radius of the flat pad around home [m], the field is exactly 0 inside it */
static constexpr float kFlatRadiusM = 1.5f;

/** terrain height at (north, east) [m], positive up, 0 at home */
float height(float north_m, float east_m);

/**
 * Distance along a downward beam to the terrain.
 *
 * dir_alt is positive up, so a straight down beam has dir_alt = -1. The
 * direction is expected to be unit length. Solved by fixed point iteration
 * on the hit, which converges while slope * tan(tilt) < 1.
 *
 * @return distance to the hit, max_t if nothing is hit within range or the
 *         beam points more than about 78 degrees off vertical, 0 if the
 *         origin is at or below the surface
 */
float raycast(float origin_n, float origin_e, float origin_alt,
	      float dir_n, float dir_e, float dir_alt, float max_t);

/**
 * Configure the field. Called by SIH on every parameter update.
 *
 * @param amp         peak amplitude [m], 0 gives a flat field everywhere
 * @param wavelength  wavelength of the largest octave [m], SIH_TERR_WAVELEN
 * @param seed        hash seed, the same seed gives the same field
 */
void set_params(float amp, float wavelength, int seed);

/** number of noise octaves, 1 to 9, 6 by default, more is finer detail at more cost per call */
void set_octaves(int octaves);

} // namespace terrain_sim
