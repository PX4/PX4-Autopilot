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
* Calculation / lookup table for earth magnetic field declination.
*
* Lookup table from Scott Ferguson <scottfromscott@gmail.com>
*
* XXX Lookup table currently too coarse in resolution (only full degrees)
* and lat/lon res - needs extension medium term.
*
*/

#include <geo/geo.h>

/** set this always to the sampling in degrees for the table below */
#define SAMPLING_RES		10.0f
#define SAMPLING_MIN_LAT	-60.0f
#define SAMPLING_MAX_LAT	60.0f
#define SAMPLING_MIN_LON	-180.0f
#define SAMPLING_MAX_LON	180.0f

static const int8_t declination_table[13][37] = \
{
	{ 46, 45, 44, 42, 41, 40, 38, 36, 33, 28, 23, 16, 10, 4, -1, -5, -9, -14, -19, -26, -33, -40, -48, -55, -61, -66, -71, -74, -75, -72, -61, -25, 22, 40, 45, 47, 46 },
	{ 30, 30, 30, 30, 29, 29, 29, 29, 27, 24, 18, 11, 3, -3, -9, -12, -15, -17, -21, -26, -32, -39, -45, -51, -55, -57, -56, -53, -44, -31, -14, 0, 13, 21, 26, 29, 30 },
	{ 21, 22, 22, 22, 22, 22, 22, 22, 21, 18, 13, 5, -3, -11, -17, -20, -21, -22, -23, -25, -29, -35, -40, -44, -45, -44, -40, -32, -22, -12, -3, 3, 9, 14, 18, 20, 21 },
	{ 16, 17, 17, 17, 17, 17, 16, 16, 16, 13, 8, 0, -9, -16, -21, -24, -25, -25, -23, -20, -21, -24, -28, -31, -31, -29, -24, -17, -9, -3, 0, 4, 7, 10, 13, 15, 16 },
	{ 12, 13, 13, 13, 13, 13, 12, 12, 11, 9, 3, -4, -12, -19, -23, -24, -24, -22, -17, -12, -9, -10, -13, -17, -18, -16, -13, -8, -3, 0, 1, 3, 6, 8, 10, 12, 12 },
	{ 10, 10, 10, 10, 10, 10, 10, 9, 9, 6, 0, -6, -14, -20, -22, -22, -19, -15, -10, -6, -2, -2, -4, -7, -8, -8, -7, -4, 0, 1, 1, 2, 4, 6, 8, 10, 10 },
	{ 9, 9, 9, 9, 9, 9, 8, 8, 7, 4, -1, -8, -15, -19, -20, -18, -14, -9, -5, -2, 0, 1, 0, -2, -3, -4, -3, -2, 0, 0, 0, 1, 3, 5, 7, 8, 9 },
	{ 8, 8, 8, 9, 9, 9, 8, 8, 6, 2, -3, -9, -15, -18, -17, -14, -10, -6, -2, 0, 1, 2, 2, 0, -1, -1, -2, -1, 0, 0, 0, 0, 1, 3, 5, 7, 8 },
	{ 8, 9, 9, 10, 10, 10, 10, 8, 5, 0, -5, -11, -15, -16, -15, -12, -8, -4, -1, 0, 2, 3, 2, 1, 0, 0, 0, 0, 0, -1, -2, -2, -1, 0, 3, 6, 8 },
	{ 6, 9, 10, 11, 12, 12, 11, 9, 5, 0, -7, -12, -15, -15, -13, -10, -7, -3, 0, 1, 2, 3, 3, 3, 2, 1, 0, 0, -1, -3, -4, -5, -5, -2, 0, 3, 6 },
	{ 5, 8, 11, 13, 15, 15, 14, 11, 5, -1, -9, -14, -17, -16, -14, -11, -7, -3, 0, 1, 3, 4, 5, 5, 5, 4, 3, 1, -1, -4, -7, -8, -8, -6, -2, 1, 5 },
	{ 4, 8, 12, 15, 17, 18, 16, 12, 5, -3, -12, -18, -20, -19, -16, -13, -8, -4, -1, 1, 4, 6, 8, 9, 9, 9, 7, 3, -1, -6, -10, -12, -11, -9, -5, 0, 4 },
	{ 3, 9, 14, 17, 20, 21, 19, 14, 4, -8, -19, -25, -26, -25, -21, -17, -12, -7, -2, 1, 5, 9, 13, 15, 16, 16, 13, 7, 0, -7, -12, -15, -14, -11, -6, -1, 3 },
};

static float get_lookup_table_val(unsigned lat, unsigned lon);

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
	int min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES;
	int min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES;

	/* for the rare case of hitting the bounds exactly
	 * the rounding logic wouldn't fit, so enforce it.
	 */

	/* limit to table bounds - required for maxima even when table spans full globe range */
	if (lat <= SAMPLING_MIN_LAT) {
		min_lat = SAMPLING_MIN_LAT;
	}

	if (lat >= SAMPLING_MAX_LAT) {
		min_lat = (int)(lat / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
	}

	if (lon <= SAMPLING_MIN_LON) {
		min_lon = SAMPLING_MIN_LON;
	}

	if (lon >= SAMPLING_MAX_LON) {
		min_lon = (int)(lon / SAMPLING_RES) * SAMPLING_RES - SAMPLING_RES;
	}

	/* find index of nearest low sampling point */
	unsigned min_lat_index = (-(SAMPLING_MIN_LAT) + min_lat)  / SAMPLING_RES;
	unsigned min_lon_index = (-(SAMPLING_MIN_LON) + min_lon) / SAMPLING_RES;

	float declination_sw = get_lookup_table_val(min_lat_index, min_lon_index);
	float declination_se = get_lookup_table_val(min_lat_index, min_lon_index + 1);
	float declination_ne = get_lookup_table_val(min_lat_index + 1, min_lon_index + 1);
	float declination_nw = get_lookup_table_val(min_lat_index + 1, min_lon_index);

	/* perform bilinear interpolation on the four grid corners */

	float declination_min = ((lon - min_lon) / SAMPLING_RES) * (declination_se - declination_sw) + declination_sw;
	float declination_max = ((lon - min_lon) / SAMPLING_RES) * (declination_ne - declination_nw) + declination_nw;

	return ((lat - min_lat) / SAMPLING_RES) * (declination_max - declination_min) + declination_min;
}

float get_lookup_table_val(unsigned lat_index, unsigned lon_index)
{
	return declination_table[lat_index][lon_index];
}