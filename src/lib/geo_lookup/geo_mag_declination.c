/****************************************************************************
 *
 *   Copyright (c) 2014 MAV GEO Library (MAVGEO). All rights reserved.
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
 * 3. Neither the name MAVGEO nor the names of its contributors may be
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
* @file geo_mag_declination.c
*
* Calculation / lookup table for Earth's magnetic field declination.
*
* Lookup table from Scott Ferguson <scottfromscott@gmail.com> and
* Stephan Brown <stephan.brown.07@gmail.com>
*
* XXX Lookup table currently too coarse in resolution (only full degrees)
* and lat/lon res - needs extension medium term.
*
*/

#include <stdint.h>
#include "geo_mag_declination.h"

/** set this always to the sampling in degrees for the table below */
#define SAMPLING_RES		10.0f
#define SAMPLING_MIN_LAT	-60.0f
#define SAMPLING_MAX_LAT	60.0f
#define SAMPLING_MIN_LON	-180.0f
#define SAMPLING_MAX_LON	180.0f

#define constrain(val, min, max) (val < min) ? min : ((val > max) ? max : val)

static const int8_t declination_table[13][37] = \
{
	{ 47, 45, 44, 43, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -41, -48, -55, -61, -67, -71, -74, -75, -72, -61, -23, 23, 41, 46, 47, 47 },
	{ 30, 30, 30, 30, 30, 29, 29, 29, 27, 23, 18, 11, 3, -3, -9, -12, -15, -17, -21, -26, -32, -39, -46, -51, -55, -57, -56, -52, -44, -31, -14, 1, 13, 21, 26, 29, 30 },
	{ 22, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, -40, -44, -45, -44, -39, -31, -21, -11, -3, 3, 9, 14, 18, 20, 22 },
	{ 16, 17, 17, 17, 17, 16, 16, 16, 15, 13, 8, 0, -9, -17, -22, -24, -25, -24, -22, -20, -21, -24, -29, -31, -31, -28, -23, -16, -9, -3, 0, 4, 7, 10, 13, 15, 16 },
	{ 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -13, -19, -23, -24, -24, -21, -17, -12, -9, -10, -14, -17, -18, -16, -12, -8, -3, 0, 1, 3, 5, 8, 10, 12, 12 },
	{ 10, 10, 10, 10, 10, 10, 10, 9, 8, 5, 0, -7, -15, -20, -22, -22, -19, -15, -10, -5, -2, -1, -4, -7, -8, -8, -6, -3, 0, 0, 1, 2, 4, 6, 8, 10, 10 },
	{ 9, 9, 9, 9, 9, 9, 8, 8, 7, 3, -2, -9, -15, -19, -20, -17, -13, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 2, 5, 7, 8, 9 },
	{ 8, 8, 8, 9, 9, 9, 8, 7, 5, 1, -3, -10, -15, -17, -17, -14, -10, -5, -2, 0, 1, 2, 2, 0, -1, -1, -1, -1, 0, 0, 0, 0, 0, 3, 5, 7, 8 },
	{ 8, 8, 9, 9, 10, 10, 9, 8, 5, 0, -5, -11, -15, -16, -15, -11, -7, -3, -1, 0, 2, 3, 3, 1, 0, 0, 0, 0, 0, -1, -2, -3, -2, 0, 3, 6, 8 },
	{ 6, 8, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -6, -3, 0, 1, 3, 4, 4, 3, 2, 1, 1, 0, -1, -3, -5, -6, -5, -3, 0, 3, 6 },
	{ 5, 8, 11, 13, 14, 15, 13, 10, 5, -1, -9, -14, -16, -16, -13, -10, -6, -3, 0, 2, 3, 5, 5, 5, 5, 4, 3, 1, -1, -4, -7, -9, -8, -6, -2, 1, 5 },
	{ 3, 8, 12, 15, 17, 17, 16, 12, 5, -3, -12, -18, -20, -19, -16, -12, -8, -4, 0, 2, 4, 7, 8, 9, 10, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, 0, 3 },
	{ 3, 8, 13, 16, 19, 20, 19, 14, 4, -7, -18, -24, -26, -24, -21, -16, -11, -6, -2, 2, 6, 10, 13, 15, 17, 16, 13, 7, 0, -7, -13, -15, -14, -11, -6, -1, 3 },
};

static float get_lookup_table_val(unsigned lat, unsigned lon);
static unsigned get_lookup_table_index(float *val, float min, float max);

unsigned get_lookup_table_index(float *val, float min, float max)
{
	/* for the rare case of hitting the bounds exactly
	 * the rounding logic wouldn't fit, so enforce it.
	 */
	/* limit to table bounds - required for maxima even when table spans full globe range */
	if (*val < min) {
		*val = min;
	}

	/* limit to (table bounds - 1) because bilinear interpolation requires checking (index + 1) */
	if (*val > max) {
		*val = max - SAMPLING_RES;
	}

	return (-(min) + *val)  / SAMPLING_RES;
}

__EXPORT float get_mag_declination(float lat, float lon)
{
	/*
	 * If the values exceed valid ranges, return zero as default
	 * as we have no way of knowing what the closest real value
	 * would be.
	 */
	if (lat < -90.0f || lat > 90.0f ||
	    lon < -180.0f || lon > 180.0f) {
		return 0.0f;
	}

	/* round down to nearest sampling resolution */
	float min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES;
	float min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES;

	/* find index of nearest low sampling point */
	unsigned min_lat_index = get_lookup_table_index(&min_lat, SAMPLING_MIN_LAT, SAMPLING_MAX_LAT);
	unsigned min_lon_index = get_lookup_table_index(&min_lon, SAMPLING_MIN_LON, SAMPLING_MAX_LON);

	float declination_sw = get_lookup_table_val(min_lat_index, min_lon_index);
	float declination_se = get_lookup_table_val(min_lat_index, min_lon_index + 1);
	float declination_ne = get_lookup_table_val(min_lat_index + 1, min_lon_index + 1);
	float declination_nw = get_lookup_table_val(min_lat_index + 1, min_lon_index);

	/* perform bilinear interpolation on the four grid corners */
	float lat_scale = constrain((lat - min_lat) / SAMPLING_RES, 0.0f, 1.0f);
	float lon_scale = constrain((lon - min_lon) / SAMPLING_RES, 0.0f, 1.0f);

	float declination_min = lon_scale * (declination_se - declination_sw) + declination_sw;
	float declination_max = lon_scale * (declination_ne - declination_nw) + declination_nw;

	return lat_scale * (declination_max - declination_min) + declination_min;
}

float get_lookup_table_val(unsigned lat_index, unsigned lon_index)
{
	return declination_table[lat_index][lon_index];
}
