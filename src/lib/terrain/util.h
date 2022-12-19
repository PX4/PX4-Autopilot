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

#pragma once

#include <matrix/math.hpp>
#include <px4_platform_common/defines.h>

namespace terrain
{

#define TERRAIN_STORAGE_DIR PX4_STORAGEDIR "/terrain_data"

namespace util
{

/// This defines the delta latitude/longitude angle stored in a single file.
/// The larger this is the less files are generated, but the larger the files and thus required
/// RAM buffer, as always whole files are loaded to RAM (see maxNumItemsPerFile()).
/// Note: if this is changed, make sure to also increase the FileHeader::version
static constexpr double delta_lat_lon_deg = 0.02;

// 90/delta_lat_lon_deg must be int, the alignment in TerrainUploader::updateArea() (and other places) depend on that.
static_assert(90. / delta_lat_lon_deg - (int)(90. / delta_lat_lon_deg) < 0.000001, "invalid delta_lat_lon_deg");

// The following are given by the mavlink messages
static constexpr int mavlink_num_blocks_x = 7; ///< 7 blocks in north direction
static constexpr int mavlink_num_blocks_y = 8; ///< 8 blocks in east direction
static constexpr int mavlink_terrain_data_size = 4; ///< 4x4 data points in TERRAIN_DATA.data

/**
 * Maximum number of items (x*y) in a file
 */
int maxNumItemsPerFile(int grid_spacing_m);

/**
 * Get a tag from a latitude, used to identify cached items or files on disk (this is used as part of the filename)
 */
static inline unsigned cacheTagFromLat(double lat)
{
	lat = matrix::wrap(lat, -90., 90.);
	return (lat + 90.) / delta_lat_lon_deg;
}

/**
 * Get a tag from a longitude, used to identify cached items or files on disk (this is used as part of the filename)
 */
static inline unsigned cacheTagFromLon(double lon)
{
	lon = matrix::wrap(lon, -180., 180.);
	return (lon + 180.) / delta_lat_lon_deg;
}

void getFileName(unsigned tag_lat, unsigned tag_lon, char *buf, int len);

static inline void getFileName(double lat, double lon, char *buf, int len)
{
	unsigned tag_lat = cacheTagFromLat(lat);
	unsigned tag_lon = cacheTagFromLon(lon);
	getFileName(tag_lat, tag_lon, buf, len);
}


static inline bool getTagsFromFileName(const char *filename, unsigned &tag_lat, unsigned &tag_lon)
{
	return sscanf(filename, "%d_%d.dat", &tag_lat, &tag_lon) == 2;
}

}

#pragma pack(push, 1)
struct FileHeader {
	uint16_t magic{0xccbb};
	uint16_t version{0};
	uint16_t num_points_x;
	uint16_t num_points_y;
	int grid_spacing_m;
	double lat_sw; ///< latitude of south-west corner
	double lon_sw; ///< longitude of south-west corner
};
#pragma pack(pop)


}
