/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "util.h"

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>

using namespace terrain;

namespace terrain
{
namespace util
{

void getFileName(unsigned tag_lat, unsigned tag_lon, char *buf, int len)
{
	int n = snprintf(buf, len, "%s/%05i_%05i.dat", TERRAIN_STORAGE_DIR, tag_lat, tag_lon);

	if (n >= len) {
		PX4_ERR("terrain path too long (%i %i)", n, len);
	}

	buf[len - 1] = '\0';
}

int maxNumItemsPerFile(int grid_spacing_m)
{
	// Given a delta angle (delta_lat_lon_deg), this computes the maximum area in local space
	// over all possible lat/lon reference points. See map_projection_project().
	// The maximum is at latitude=0, the longitude does not matter.
	constexpr double r = CONSTANTS_RADIUS_OF_EARTH;
	const double cos_d = cos(math::radians(delta_lat_lon_deg));
	const double cos_d_sq = cos_d * cos_d;
	const double sin_d = sin(math::radians(delta_lat_lon_deg));

	double c = acos(cos_d_sq);
	double x = c / sin(c) * r * sin_d;
	double y = x * cos_d;
	// account for 1 item overlap in both directions
	return (int)((x + grid_spacing_m) / grid_spacing_m + 1) * (int)((y + grid_spacing_m) / grid_spacing_m + 1);
}
} // namespace util
} // namespace terrain
