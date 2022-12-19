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

#include <stdint.h>
#include <math.h>

#include "mavlink_bridge_header.h"

#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/terrain/util.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <matrix/math.hpp>

using namespace time_literals;

namespace terrain
{

// The following are given by the mavlink messages
static constexpr int mavlink_num_blocks_x = 7; ///< 7 blocks in north direction
static constexpr int mavlink_num_blocks_y = 8; ///< 8 blocks in east direction
static constexpr int mavlink_terrain_data_size = 4; ///< 4x4 data points in TERRAIN_DATA.data


/**
 * @class TerrainUploader
 * Handles terrain upload and disk storage
 */
class TerrainUploader
{
public:
	static constexpr double maximum_latitude = 80.; ///< maximum supported latitude (this also holds for negative values).
	///< Beyond that the distortion becomes too high (though we could go up to ~85 deg if needed)

	static constexpr hrt_abstime request_timeout = 200_ms;

	/// User-facing errors
	enum class Error {
		None = 0,
		LatitudeExceeded,
		DirectoryCreationFailure,
		StorageFailure,
	};

	TerrainUploader(int grid_spacing_m);
	~TerrainUploader() = default;

	/**
	 * Configure an area in which terrain data should be requested and stored to disk.
	 * If the same area is requested as previously, no new requests are done.
	 * Note: the latitude must be within [-maximum_latitude, maximum_latitude]. The longitude can be outside of [-180, 180].
	 * Note: the caller is responsible to add a margin around the area.
	 * @param now
	 * @param lat_sw SW latitude corner
	 * @param lon_sw SW longitude corner
	 * @param lat_ne NE latitude corner
	 * @param lon_ne NE longitude corner
	 * @return 0 on success, <0 error otherwise
	 */
	int updateArea(const hrt_abstime &now, double lat_sw, double lon_sw, double lat_ne, double lon_ne);

	void update(const hrt_abstime &now);

	int num4x4BlocksPending() const { return _num_4x4_blocks_pending; }
	int num4x4BlocksLoaded() const { return _num_4x4_blocks_loaded; }

	/**
	 * Get the latest request ID & data pointer
	 * @param request output request (set to nullptr if no active request)
	 * @return request ID. If this differs from a previous ID, a new request should be sent
	 */
	int getLatestRequest(const mavlink_terrain_request_t **request) const;

	void handleTerrainData(const hrt_abstime &now, const mavlink_terrain_data_t &data);

	void printStatus();

	int gridSpacing() const { return _latest_request.grid_spacing; }

	Error getLatestErrorAndClear() { Error e = _latest_error; _latest_error = Error::None; return e; }
private:

	class File
	{
	public:
		~File();

		/**
		 * Initialize a new file
		 * @param lat latitude, expected to be aligned to a multiple of util::delta_lat_lon_deg
		 * @param lon longitude, expected to be aligned to a multiple of util::delta_lat_lon_deg
		 * @param grid_spacing_m
		 * @return 0 on success, 1 if file already exists, <0 error otherwise
		 */
		int init(double lat, double lon, int grid_spacing_m);

		int writeBlock(const int16_t data[util::mavlink_terrain_data_size * util::mavlink_terrain_data_size], int block_x,
			       int block_y);

		bool nextMavlinkRequest(mavlink_terrain_request_t &request);

		int finalize();

		bool fileExists(double lat, double lon);
	private:
		static constexpr char tmp_filename[] = TERRAIN_STORAGE_DIR "/next.dat";
		void getCurrentFileName(char *filename, int len);

		FileHeader _header;
		MapProjection _ref;
		int _fd{-1};
		int _x{-1};
		int _y{-1};
	};

	bool latestRequestValid() const { return _latest_request.mask != 0; }

	void requestNextFile(const hrt_abstime &now, bool start_with_current = false);
	void allRequestsDone();

	void pruneStorage(double lat_sw, double lon_sw, double lat_ne, double lon_ne);

	int _current_request_id{0};
	mavlink_terrain_request_t _latest_request{};
	hrt_abstime _next_timeout{0};
	int _num_4x4_blocks_pending{0};
	int _num_4x4_blocks_loaded{0};

	File _file;

	double _lat_start;
	double _lon_start;
	double _lat_end;
	double _lon_end;
	double _lat_cur;
	double _lon_cur;

	Error _latest_error{Error::None};
};

}
