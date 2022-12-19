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

#include "terrain.h"

#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

using namespace terrain;

constexpr char terrain::TerrainUploader::File::tmp_filename[];


TerrainUploader::TerrainUploader(int grid_spacing_m)
{
	_latest_request.grid_spacing = grid_spacing_m;
	PX4_DEBUG("terrain: max num items per file: %i (grid spacing=%i m)", util::maxNumItemsPerFile(grid_spacing_m),
		  grid_spacing_m);
}

int TerrainUploader::updateArea(const hrt_abstime &now, double lat_sw, double lon_sw, double lat_ne, double lon_ne)
{
	// align to file boundary
	lat_sw = floor(lat_sw / util::delta_lat_lon_deg) * util::delta_lat_lon_deg;
	lon_sw = floor(lon_sw / util::delta_lat_lon_deg) * util::delta_lat_lon_deg;

	double min_diff = 0.00001;

	if (latestRequestValid() && fabs(lat_sw - _lat_start) < min_diff  &&
	    fabs(lon_sw - _lon_start) < min_diff  &&
	    fabs(lat_ne - _lat_end) < min_diff  &&
	    fabs(lon_ne - _lon_end) < min_diff) {
		PX4_DEBUG("Request already in progress");
		return 0;
	}

	pruneStorage(lat_sw, lon_sw, lat_ne, lon_ne);

	// create storage dir if not exists
	int mkdir_ret = mkdir(TERRAIN_STORAGE_DIR, S_IRWXU | S_IRWXG | S_IRWXO);

	if (mkdir_ret != OK && errno != EEXIST) {
		PX4_ERR("failed creating dir: %s (%i)", TERRAIN_STORAGE_DIR, errno);
		_latest_error = Error::DirectoryCreationFailure;
		return errno;
	}

	if (lat_ne < lat_sw || lon_ne < lon_sw) {
		PX4_ERR("invalid lat/lon area");
		return -EINVAL;
	}

	if (lat_sw < -maximum_latitude || lat_ne > maximum_latitude) {
		PX4_ERR("unsupported lat/lon area");
		_latest_error = Error::LatitudeExceeded;
		return -EINVAL;
	}

	_lat_cur = _lat_start = lat_sw;
	_lon_cur = _lon_start = lon_sw;
	_lat_end = lat_ne;
	_lon_end = lon_ne;
	_num_4x4_blocks_loaded = 0;

	// calculate the number of pending blocks
	_num_4x4_blocks_pending = 0;

	for (double lat = _lat_start; lat <= _lat_end; lat += util::delta_lat_lon_deg) {
		for (double lon = _lon_start; lon <= _lon_end; lon += util::delta_lat_lon_deg) {
			if (_file.fileExists(lat, lon)) {
				continue;
			}

			MapProjection ref(lat, lon);
			float x, y;
			ref.project(lat + util::delta_lat_lon_deg, lon + util::delta_lat_lon_deg, x, y);
			int num_items_x = (x + _latest_request.grid_spacing) / _latest_request.grid_spacing +
					  1; // see TerrainUploader::File::init()
			int num_items_y = (y + _latest_request.grid_spacing) / _latest_request.grid_spacing + 1;
			_num_4x4_blocks_pending += ((num_items_x + util::mavlink_terrain_data_size - 1) / util::mavlink_terrain_data_size) *
						   ((num_items_y + util::mavlink_terrain_data_size - 1) / util::mavlink_terrain_data_size);
			PX4_DEBUG("num items x=%i, y=%i, pending blocks=%i", num_items_x, num_items_y, _num_4x4_blocks_pending);
		}
	}

	if (_num_4x4_blocks_pending > 0) {
		PX4_INFO("Terrain: area update: SW: %.3f %.3f, NE: %.3f %.3f",
			 lat_sw, lon_sw, lat_ne, lon_ne);
	}

	requestNextFile(now, true);

	return 0;
}

void TerrainUploader::update(const hrt_abstime &now)
{
	if (!latestRequestValid()) {
		return;
	}

	if (_next_timeout < now) {
		// resend current request on a timeout
		++_current_request_id;
		_next_timeout = now + request_timeout;
		PX4_DEBUG("timed out, re-requesting");
	}
}

int TerrainUploader::getLatestRequest(const mavlink_terrain_request_t **request) const
{
	if (latestRequestValid()) {
		*request = &_latest_request;

	} else {
		*request = nullptr;
	}

	return _current_request_id;
}

void TerrainUploader::handleTerrainData(const hrt_abstime &now, const mavlink_terrain_data_t &data)
{
	if (data.grid_spacing != _latest_request.grid_spacing) {
		PX4_WARN("unexpected grid_spacing: %i %i", data.grid_spacing, _latest_request.grid_spacing);
		return;
	}

	if (data.lat != _latest_request.lat || data.lon != _latest_request.lon) {
		PX4_DEBUG("unexpected lat/lon: %i == %i, %i == %i", data.lat, _latest_request.lat, data.lon, _latest_request.lon);
		return;
	}

	uint64_t bit = (uint64_t)1 << data.gridbit;

	if ((_latest_request.mask & bit) == 0) {
		// already done / not requested
		return;
	}

	static_assert(sizeof(data.data) / sizeof(data.data[0]) == util::mavlink_terrain_data_size *
		      util::mavlink_terrain_data_size, "mavlink data size mismatch");
	int block_x = (data.gridbit / util::mavlink_num_blocks_y) * util::mavlink_terrain_data_size;
	int block_y = (data.gridbit % util::mavlink_num_blocks_y) * util::mavlink_terrain_data_size;
	PX4_DEBUG("Got terrain data: %.3f %.3f, bit=%i, bx=%i, by=%i, data[0]=%i", data.lat * 1e-7, data.lon * 1e-7,
		  data.gridbit, block_x, block_y, data.data[0]);

	int ret = _file.writeBlock(data.data, block_x, block_y);

	if (ret != 0) {
		PX4_ERR("block writing failed (%i)", ret);
		return;
	}

	_latest_request.mask &= ~bit; // mark as done
	--_num_4x4_blocks_pending;
	++_num_4x4_blocks_loaded;
	_next_timeout = now + request_timeout;

	if (_latest_request.mask == 0) {
		if (_file.nextMavlinkRequest(_latest_request)) {
			++_current_request_id;

		} else {
			ret = _file.finalize();

			if (ret != 0) {
				PX4_ERR("finalize failed (%i)", ret);
				_latest_error = Error::StorageFailure;
			}

			requestNextFile(now);
		}
	}
}

void TerrainUploader::requestNextFile(const hrt_abstime &now, bool start_with_current)
{
	_next_timeout = now + request_timeout;
	bool need_next_file = false;
	double delta_lat = start_with_current ? 0. : util::delta_lat_lon_deg;

	do {
		need_next_file = false;
		_lat_cur += delta_lat;

		if (_lat_cur > _lat_end) {
			_lon_cur += util::delta_lat_lon_deg;

			if (_lon_cur > _lon_end) {
				allRequestsDone();
				return;
			}

			_lat_cur = _lat_start;
		}

		int ret = _file.init(_lat_cur, _lon_cur, _latest_request.grid_spacing);

		if (ret == 1) {
			need_next_file = true; // already done

		} else if (ret < 0) {
			PX4_ERR("Failed to init file (%i)", ret);
			_latest_error = Error::StorageFailure;
			need_next_file = true; // skip this

		} else {
			if (_file.nextMavlinkRequest(_latest_request)) {
				++_current_request_id;
			}
		}

		delta_lat = util::delta_lat_lon_deg;
	} while (need_next_file);
}

void TerrainUploader::allRequestsDone()
{
	if (_num_4x4_blocks_loaded > 0) {
		PX4_INFO("Terrain: finished all requests");
	}

	if (_num_4x4_blocks_pending != 0) {
		PX4_DEBUG("pending blocks not zero! %i", _num_4x4_blocks_pending);
		_num_4x4_blocks_pending = 0;
	}

	_next_timeout = 0;
	_latest_request.mask = 0;
}

void TerrainUploader::pruneStorage(double lat_sw, double lon_sw, double lat_ne, double lon_ne)
{
	DIR *dp = opendir(TERRAIN_STORAGE_DIR);

	if (dp == nullptr) {
		return; // ignore if we cannot access the directory
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {

		if (result->d_type != DIRENT_REGULAR_FILE) {
			continue;
		}

		char filename[64];

		if (snprintf(filename, sizeof(filename), "%s/%s", TERRAIN_STORAGE_DIR, result->d_name) >= (int)sizeof(filename)) {
			PX4_ERR("file name too long: %s", result->d_name);
			continue;
		}

		unsigned tag_lat, tag_lon;
		bool delete_file = false;

		if (util::getTagsFromFileName(result->d_name, tag_lat, tag_lon)) {

			int fd = open(filename, O_RDONLY, PX4_O_MODE_666);

			if (fd == -1) {
				PX4_ERR("failed to open file %s", filename);
				continue;
			}

			// validate the file matches our expectations and is not too far away from the requested area
			FileHeader header;
			uint16_t expected_magic = header.magic;
			uint16_t expected_version = header.version;

			if (read(fd, &header, sizeof(header)) == sizeof(header)) {
				if (header.magic != expected_magic || header.version != expected_version ||
				    header.grid_spacing_m != _latest_request.grid_spacing) {
					delete_file = true;

				} else {

					// check if too far away. This could be extended, e.g. using a maximum used storage size parameter
					const double max_lat_lon_diff = 0.1;
					double lat = header.lat_sw;
					double lon = header.lon_sw;

					// handle wrap-arounds
					while (lat < lat_sw - max_lat_lon_diff) { lat += 180.; }

					while (lat > lat_ne + max_lat_lon_diff) { lat -= 180.; }

					while (lon < lon_sw - max_lat_lon_diff) { lon += 360.; }

					while (lon > lon_ne + max_lat_lon_diff) { lon -= 360.; }

					delete_file =
						lat < lat_sw - max_lat_lon_diff ||
						lat > lat_ne + max_lat_lon_diff ||
						lon < lon_sw - max_lat_lon_diff ||
						lon > lon_ne + max_lat_lon_diff;
				}

			} else {
				delete_file = true;
			}

			close(fd);

		} else {
			delete_file = true;
		}

		if (delete_file) {
			PX4_DEBUG("Removing terrain file %s", result->d_name);

			if (unlink(filename) != 0) {
				PX4_ERR("unlink failed (%i)", errno);
			}
		}
	}

	closedir(dp);
}

TerrainUploader::File::~File()
{
	if (_fd >= 0) {
		close(_fd);
	}
}

int TerrainUploader::File::init(double lat, double lon, int grid_spacing_m)
{
	if (_fd >= 0) {
		close(_fd);
	}

	_header.grid_spacing_m = grid_spacing_m;

	// check if file already exists, so that we don't need to request it
	if (fileExists(lat, lon)) {
		// as the file prunning already removed files with different grid spacing etc., we don't need to check that here
		PX4_DEBUG("skipping existing file: %.3f, %.3f", _header.lat_sw, _header.lon_sw);
		return 1;
	}

	_ref.initReference(_header.lat_sw, _header.lon_sw);
	float x, y;
	_ref.project(_header.lat_sw + util::delta_lat_lon_deg, _header.lon_sw + util::delta_lat_lon_deg, x, y);
	_header.num_points_x = (x + grid_spacing_m) / grid_spacing_m +
			       1; // add grid_spacing to ensure overlap and one more as x/spacing is the max index
	_header.num_points_y = (y + grid_spacing_m) / grid_spacing_m + 1;

	_fd = open(tmp_filename, O_CREAT | O_WRONLY, PX4_O_MODE_666);

	if (_fd < 0) {
		PX4_ERR("Can't open file %s, errno: %d", tmp_filename, errno);
		return -1;
	}

	if (write(_fd, &_header, sizeof(_header)) < (int)sizeof(_header)) {
		return -errno;
	}

	// fill the file with 0 data
	uint8_t buf[128];
	int num_write_bytes = sizeof(int16_t) * _header.num_points_x * _header.num_points_y;

	while (num_write_bytes > 0) {
		int num_write_bytes_current = math::min((int)sizeof(buf), num_write_bytes);

		if (write(_fd, &buf, num_write_bytes_current) < num_write_bytes_current) {
			return -errno;
		}

		num_write_bytes -= num_write_bytes_current;
	}

	PX4_DEBUG("init file: %.3f, %.3f, num x=%i num y=%i", _header.lat_sw, _header.lon_sw, _header.num_points_x,
		  _header.num_points_y);

	_x = _y = -1;
	return 0;
}

bool TerrainUploader::File::fileExists(double lat, double lon)
{
	_header.lat_sw = round(matrix::wrap(lat, -90., 90.) / util::delta_lat_lon_deg) * util::delta_lat_lon_deg;
	_header.lon_sw = round(matrix::wrap(lon, -180., 180.) / util::delta_lat_lon_deg) * util::delta_lat_lon_deg;

	char filename[64];
	getCurrentFileName(filename, sizeof(filename));
	struct stat buffer;
	return stat(filename, &buffer) == 0;
}

int TerrainUploader::File::writeBlock(const int16_t data[util::mavlink_terrain_data_size *
								      util::mavlink_terrain_data_size], int block_x, int block_y)
{
	// get x and y indexes
	const int x0 = _x + block_x;
	const int y0 = _y + block_y;
	PX4_DEBUG("writing indexes: x=%i y=%i", x0, y0);

	// bound check
	if (x0 < 0 || x0 >= _header.num_points_x) {
		PX4_ERR("x idx out of bounds: %i (%i)", x0, _header.num_points_x);
		return -EINVAL;
	}

	if (y0 < 0 || y0 >= _header.num_points_y) {
		PX4_ERR("y idx out of bounds: %i (%i)", y0, _header.num_points_y);
		return -EINVAL;
	}

	// TODO: might be worth storing everything (or 4 rows) in a buffer and write all at once
	for (int x1 = 0; x1 < util::mavlink_terrain_data_size; ++x1) {
		if (x0 + x1 >= _header.num_points_x) {
			break;
		}

		ssize_t offset = sizeof(_header) + (y0 + (x0 + x1) * _header.num_points_y) * sizeof(int16_t);

		if (lseek(_fd, offset, SEEK_SET) < 0) {
			return -errno;
		}

		int num_y = math::min(util::mavlink_terrain_data_size, _header.num_points_y - y0);

		if (write(_fd, &data[x1 * util::mavlink_terrain_data_size], sizeof(int16_t)*num_y) < (int)sizeof(int16_t)*num_y) {
			return -errno;
		}
	}

	return 0;
}

bool TerrainUploader::File::nextMavlinkRequest(mavlink_terrain_request_t &request)
{
	if (_fd < 0) {
		return false;
	}

	if (_x == -1) {
		_x = _y = 0;

	} else {
		_x += util::mavlink_num_blocks_x * util::mavlink_terrain_data_size;

		if (_x >= _header.num_points_x) {
			_y += util::mavlink_num_blocks_y * util::mavlink_terrain_data_size;

			if (_y >= _header.num_points_y) {
				// we're all done
				return false;
			}

			_x = 0;
		}
	}

	double lat, lon;
	_ref.reproject(_x * _header.grid_spacing_m, _y * _header.grid_spacing_m, lat, lon);
	request.lat = lat * 1e7;
	request.lon = lon * 1e7;
	// fill in the block bits we want
	int x_requests = math::min(util::mavlink_num_blocks_x,
				   (_header.num_points_x - _x + util::mavlink_terrain_data_size - 1) / util::mavlink_terrain_data_size);
	int y_requests = math::min(util::mavlink_num_blocks_y,
				   (_header.num_points_y - _y + util::mavlink_terrain_data_size - 1) / util::mavlink_terrain_data_size);
	request.mask = 0;

	for (int x = 0; x < x_requests; ++x) {
		for (int y = 0; y < y_requests; ++y) {
			request.mask |= (uint64_t)1 << (y + x * util::mavlink_num_blocks_y);
		}
	}

	PX4_DEBUG("sending new request: %.3f %.3f", lat, lon);
	return true;
}

int TerrainUploader::File::finalize()
{
	if (_fd < 0) {
		return -1;
	}

	close(_fd);
	_fd = -1;
	char filename[64];
	getCurrentFileName(filename, sizeof(filename));
	PX4_DEBUG("renaming terrain block file: %s -> %s", tmp_filename, filename);

	if (rename(tmp_filename, filename) == -1) {
		return -errno;
	}

	return 0;
}

void TerrainUploader::File::getCurrentFileName(char *filename, int len)
{
	util::getFileName(_header.lat_sw + util::delta_lat_lon_deg / 2., _header.lon_sw + util::delta_lat_lon_deg / 2.,
			  filename, len);
}

void TerrainUploader::printStatus()
{
	PX4_INFO("Num pending 4x4 blocks: %i", _num_4x4_blocks_pending);
	PX4_INFO("Num downloaded 4x4 blocks: %i", _num_4x4_blocks_loaded);
	PX4_INFO("Storage dir: %s", TERRAIN_STORAGE_DIR);

	DIR *dp = opendir(TERRAIN_STORAGE_DIR);

	if (dp == nullptr) {
		return;
	}

	PX4_INFO("Storage files:");

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {

		if (result->d_type != DIRENT_REGULAR_FILE) {
			continue;
		}

		char filename[64];

		if (snprintf(filename, sizeof(filename), "%s/%s", TERRAIN_STORAGE_DIR, result->d_name) >= (int)sizeof(filename)) {
			PX4_ERR("file name too long: %s", result->d_name);
			continue;
		}

		unsigned tag_lat, tag_lon;

		if (util::getTagsFromFileName(result->d_name, tag_lat, tag_lon)) {

			int fd = open(filename, O_RDONLY, PX4_O_MODE_666);

			if (fd == -1) {
				PX4_ERR("failed to open file %s", filename);
				continue;
			}

			FileHeader header;
			uint16_t expected_magic = header.magic;

			if (read(fd, &header, sizeof(header)) == sizeof(header)) {
				if (header.magic == expected_magic) {
					PX4_INFO("%s: spacing: %im, lat: %.3f, lon: %.3f, #x: %i, #y: %i",
						 result->d_name, header.grid_spacing_m, header.lat_sw, header.lon_sw,
						 header.num_points_x, header.num_points_y);
				}
			}

			close(fd);

		}
	}

	closedir(dp);
}
