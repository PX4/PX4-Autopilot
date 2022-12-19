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

#include "mavlink_terrain.h"
#include <lib/terrain/terrain_provider.h>

#include <dirent.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include <gtest/gtest.h>
#include <array>

using namespace terrain;
// to run: make tests TESTFILTER=mavlink_terrain

class TestData
{
public:
	TestData(int grid_spacing, double lat_sw_conf, double lon_sw_conf, double lat_ne_conf, double lon_ne_conf);
	~TestData();

	int16_t lookup(double lat, double lon) const;

	// configured area (w/o margin)
	const double lat_sw;
	const double lon_sw;
	const double lat_ne;
	const double lon_ne;
private:
	// add some margin to the test area, as the uploader might request some data outside the area
	static constexpr double margin = util::delta_lat_lon_deg * 4.;

	double _lat_sw_data;
	double _lon_sw_data;
	double _lat_ne_data;
	double _lon_ne_data;
	int16_t *_data{nullptr};
	int _x_count;
	int _y_count;
};

TestData::TestData(int grid_spacing, double lat_sw_conf, double lon_sw_conf, double lat_ne_conf, double lon_ne_conf)
	: lat_sw(lat_sw_conf), lon_sw(lon_sw_conf), lat_ne(lat_ne_conf), lon_ne(lon_ne_conf)
{
	_lat_sw_data = lat_sw - margin;
	_lon_sw_data = lon_sw - margin;
	_lat_ne_data = lat_ne + margin;
	_lon_ne_data = lon_ne + margin;

	MapProjection ref(_lat_sw_data, _lon_sw_data);
	double lat, lon;
	// get approximate delta lat/lon for the grid spacing
	ref.reproject(grid_spacing, grid_spacing, lat, lon);

	double dlat = lat - _lat_sw_data;
	double dlon = lon - _lon_sw_data;
	_x_count = (_lat_ne_data - _lat_sw_data) / dlat * 4; // 4x oversampling
	_y_count = (_lon_ne_data - _lon_sw_data) / dlon * 4;
	printf("x count: %i, y count: %i\n", _x_count, _y_count);

	_data = new int16_t[_x_count * _y_count];
	// setup synthetic dataset, no value must be zero
	int16_t lowest_value = 500;

	for (int y = 0; y < _y_count; ++y) {
		for (int x = 0; x < _x_count; ++x) {
			_data[x + y * _x_count] = lowest_value + x + y;
		}
	}
}

TestData::~TestData()
{
	delete[] _data;
}

int16_t TestData::lookup(double lat, double lon) const
{
	while (lat < _lat_sw_data) { lat += 180.; }

	while (lat > _lat_ne_data) { lat -= 180.; }

	while (lon < _lon_sw_data) { lon += 360.; }

	while (lon > _lon_ne_data) { lon -= 360.; }

	int x = (lat - _lat_sw_data) / (_lat_ne_data - _lat_sw_data) * _x_count;
	int y = (lon - _lon_sw_data) / (_lon_ne_data - _lon_sw_data) * _y_count;

	if (x < 0 || x >= _x_count || y < 0 || y >= _y_count) {
		printf("x=%i x count=%i y=%i y count=%i\n", x, _x_count, y, _y_count);
		EXPECT_FALSE(x < 0 || x >= _x_count || y < 0
			     || y >= _y_count) << "lookup out of range: lat: " << lat << ", lon: " << lon;
		return 1;
	}

	return _data[x + y * _x_count];
}


class MavlinkTerrainTest : public ::testing::Test
{
public:
	static void clearStorageDirectory();
	int validateTerrainDataFiles();
	void validateFile(const char *filename);

	void SetUp() override
	{
		clearStorageDirectory();
	}

	void uploadData(TerrainUploader &uploader, TestData &dataset, hrt_abstime &time, int grid_spacing,
			bool enable_timeouts);

	void lookupWaitAndCheck(TerrainProvider &provider, const TestData &dataset, double lat, double lon);
};

void MavlinkTerrainTest::clearStorageDirectory()
{
	DIR *dp = opendir(TERRAIN_STORAGE_DIR);

	if (dp == nullptr) {
		return; // ignore if we cannot access the directory
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {

		if (!strcmp(result->d_name, ".") || !strcmp(result->d_name, "..")) {
			continue;
		}

		char filename[64];

		if (snprintf(filename, sizeof(filename), "%s/%s", TERRAIN_STORAGE_DIR, result->d_name) >= (int)sizeof(filename)) {
			FAIL() << "file name too long: " << result->d_name;
		}

		EXPECT_EQ(unlink(filename), 0);
	}

	closedir(dp);
}

void MavlinkTerrainTest::uploadData(TerrainUploader &uploader, TestData &dataset, hrt_abstime &time, int grid_spacing,
				    bool enable_timeouts)
{
	uploader.updateArea(time, dataset.lat_sw, dataset.lon_sw, dataset.lat_ne, dataset.lon_ne);
	uploader.update(time);

	// should be downloading now
	ASSERT_GT(uploader.num4x4BlocksPending(), 0);

	int latest_request_id = 0;
	int timeout_counter = 0;

	for (int i = 0; i < 10000; ++i) { // limit the amount of loops (arbitrary)
		if (uploader.num4x4BlocksPending() == 0) {
			break;
		}

		time += 10_ms;
		uploader.update(time);
		const mavlink_terrain_request_t *request{nullptr};
		int request_id = uploader.getLatestRequest(&request);

		if (request && latest_request_id != request_id) {
			latest_request_id = request_id;

			mavlink_terrain_request_t latest_request;
			memcpy(&latest_request, request, sizeof(latest_request));

			const double request_lat = latest_request.lat * 1e-7;
			const double request_lon = latest_request.lon * 1e-7;
			ASSERT_GE(request_lat, -90.);
			ASSERT_LE(request_lat, 90.);
			ASSERT_GE(request_lon, -180.);
			ASSERT_LE(request_lon, 180.);

			// feed in requested data
			MapProjection ref(request_lat, request_lon);
			bool had_previous_request = false;
			bool should_abort_request = false;

			for (int x = 0; x < util::mavlink_num_blocks_x && !should_abort_request; ++x) {
				for (int y = 0; y < util::mavlink_num_blocks_y && !should_abort_request; ++y) {
					mavlink_terrain_data_t data{};
					data.grid_spacing = grid_spacing;
					data.gridbit = y + x * util::mavlink_num_blocks_y;

					if ((latest_request.mask & ((uint64_t)1 << data.gridbit)) == 0) {
						continue;
					}

					data.lat = latest_request.lat;
					data.lon = latest_request.lon;
					static constexpr int block_size = util::mavlink_terrain_data_size;
					float x_offset = x * block_size * grid_spacing;
					float y_offset = y * block_size * grid_spacing;

					for (int x_block = 0; x_block < block_size; ++x_block) {
						for (int y_block = 0; y_block < block_size; ++y_block) {
							double lat_cur, lon_cur;
							ref.reproject(x_offset + x_block * grid_spacing, y_offset + y_block * grid_spacing, lat_cur, lon_cur);
							data.data[y_block + block_size * x_block] = dataset.lookup(lat_cur, lon_cur);
						}
					}

					// if timeouts are enabled, pass only every 3rd request
					bool should_timeout = enable_timeouts && (++timeout_counter % 3 != 2);

					if (should_timeout) {
						time += TerrainUploader::request_timeout + 10_ms;
						should_abort_request = true;

					} else {
						time += 10_ms;
						uploader.handleTerrainData(time, data);
					}

					uploader.update(time);


					// except for the last iteration there should be no new request
					EXPECT_FALSE(had_previous_request);

					request_id = uploader.getLatestRequest(&request);

					if (request_id != latest_request_id) {
						had_previous_request = true;
					}
				}
			}
		}
	}

	ASSERT_EQ(uploader.num4x4BlocksPending(), 0);
}

void MavlinkTerrainTest::lookupWaitAndCheck(TerrainProvider &provider, const TestData &dataset, double lat, double lon)
{
	int16_t expected = dataset.lookup(lat, lon);

	lat = matrix::wrap(lat, -90., 90.);
	lon = matrix::wrap(lon, -180., 180.);

	// we expect the lookup to succeed, so we use a relatively large timeout to avoid false CI failures
	for (int loops = 0; loops < 2000; ++loops) {
		float alt_msl_m;

		if (provider.lookup(lat, lon, alt_msl_m)) {
			ASSERT_NEAR((float)expected, alt_msl_m, 2.0f) << "lat: " << lat << " lon: " << lon;
			return;
		}

		usleep(1000);
	}

	FAIL() << "terrain lookup failed for lat=" << lat << ", lon=" << lon;
}

int MavlinkTerrainTest::validateTerrainDataFiles()
{
	DIR *dp = opendir(TERRAIN_STORAGE_DIR);

	EXPECT_TRUE(dp);

	if (!dp) {
		return 0;
	}

	struct dirent *result = nullptr;

	int num_files = 0;

	while ((result = readdir(dp))) {

		char filename[64];

		if (snprintf(filename, sizeof(filename), "%s/%s", TERRAIN_STORAGE_DIR, result->d_name) >= (int)sizeof(filename)) {
			EXPECT_TRUE(false) << "file name too long: " << result->d_name;
			return 0;
		}

		unsigned tag_lat, tag_lon;

		if (util::getTagsFromFileName(result->d_name, tag_lat, tag_lon)) {

			validateFile(filename);

			++num_files;
		}
	}

	closedir(dp);
	return num_files;
}

void MavlinkTerrainTest::validateFile(const char *filename)
{
	int fd = open(filename, O_RDONLY, PX4_O_MODE_666);

	if (fd == -1) {
		FAIL() << "failed to open file: " << filename;
	}

	printf("checking file %s\n", filename);

	FileHeader header;
	uint16_t expected_magic = header.magic;
	uint16_t expected_version = header.version;

	ASSERT_EQ(read(fd, &header, sizeof(header)), sizeof(header));
	ASSERT_EQ(header.magic, expected_magic);
	ASSERT_EQ(header.version, expected_version);
	ASSERT_GT(header.num_points_x, 0);
	ASSERT_GT(header.num_points_y, 0);

	ASSERT_LE(header.num_points_x * header.num_points_y, util::maxNumItemsPerFile(header.grid_spacing_m));

	// read all data
	int16_t *buffer = new int16_t[header.num_points_x * header.num_points_y];
	int read_size = sizeof(int16_t) * header.num_points_x * header.num_points_y;
	ASSERT_EQ(read(fd, buffer, read_size), read_size);

	// check that all altitude values are non-zero
	for (int i = 0; i < header.num_points_x * header.num_points_y; ++i) {
		ASSERT_GT(buffer[i], 0) << "i: " << i;
	}

	delete[] buffer;

	close(fd);
}

TEST_F(MavlinkTerrainTest, upload)
{
	int grid_spacing = 50;
	TestData dataset{grid_spacing, 47.357022, 8.468667, 47.371207, 8.493230};
	TerrainUploader uploader{grid_spacing};
	hrt_abstime time = 432532423;

	ASSERT_NO_FATAL_FAILURE(uploadData(uploader, dataset, time, grid_spacing, false));

	// validate all files to contain only non-zero altitude data
	ASSERT_GT(validateTerrainDataFiles(), 0);
}

TEST_F(MavlinkTerrainTest, reupload_with_timeouts)
{
	int grid_spacing = 50;
	TestData dataset{grid_spacing, 47.357022, 8.468667, 47.371207, 8.493230};
	TerrainUploader uploader{grid_spacing};
	hrt_abstime time = 87654321;

	ASSERT_NO_FATAL_FAILURE(uploadData(uploader, dataset, time, grid_spacing, true));

	int num_terrain_files;
	ASSERT_NO_FATAL_FAILURE(num_terrain_files = validateTerrainDataFiles(););
	ASSERT_GT(num_terrain_files, 0);

	const mavlink_terrain_request_t *request{nullptr};
	int request_id = uploader.getLatestRequest(&request);

	// request the same area again
	uploader.updateArea(time, dataset.lat_sw, dataset.lon_sw, dataset.lat_ne, dataset.lon_ne);

	time += 10_ms;

	// should not be downloading as we specify the same area
	ASSERT_EQ(uploader.num4x4BlocksPending(), 0);
	uploader.update(time);
	ASSERT_EQ(uploader.num4x4BlocksPending(), 0);
	ASSERT_EQ(request_id, uploader.getLatestRequest(&request));


	// request different area, far away: this should prune all data
	uploader.updateArea(time, dataset.lat_sw, dataset.lon_sw + 1., dataset.lat_ne, dataset.lon_ne + 1.);
	ASSERT_EQ(validateTerrainDataFiles(), 0);
}

TEST_F(MavlinkTerrainTest, upload_and_lookup)
{
	struct Area {
		double lat_sw;
		double lon_sw;
		double lat_ne;
		double lon_ne;
	};
	std::array<int, 4> grid_spacings{10, 50, 100, 201};
	std::array<Area, 6> areas{
		Area{47.357022, 8.468667, 47.371207, 8.493230},
		Area{0, 0, 0.001, 0.001},
		Area{-5, -8, -5 + 0.01, -8 + 0.03},
		Area{-0.0143, -0.02423, 0.014, 0.01},
		Area{-70.022, -190.0104, -69.992, -189.9633},
		Area{-79.999, -190.0104, -79.932, -189.9633},
	};

	for (int grid_spacing : grid_spacings) {
		for (unsigned area_index = 0; area_index < areas.size(); ++area_index) {
			printf("Testing with: grid_spacing: %i, area: %i\n", grid_spacing, area_index);

			clearStorageDirectory();

			const Area &area = areas[area_index];
			TestData dataset{grid_spacing, area.lat_sw, area.lon_sw, area.lat_ne, area.lon_ne};
			TerrainUploader uploader{grid_spacing};
			hrt_abstime time = 432532423;

			ASSERT_NO_FATAL_FAILURE(uploadData(uploader, dataset, time, grid_spacing, false));

			// validate all files to contain only non-zero altitude data
			int num_terrain_files;
			ASSERT_NO_FATAL_FAILURE(num_terrain_files = validateTerrainDataFiles(););
			ASSERT_GT(num_terrain_files, 0);

			TerrainProvider provider{grid_spacing};

			// validate lookups in a grid over the configured area
			const double delta = util::delta_lat_lon_deg / 100.;

			for (double lat = dataset.lat_sw; lat <= dataset.lat_ne; lat += delta) {
				for (double lon = dataset.lon_sw; lon <= dataset.lon_ne; lon += delta) {

					ASSERT_NO_FATAL_FAILURE(
						lookupWaitAndCheck(provider, dataset, lat, lon);
					);
				}
			}

			// test at the border of a file
			double lat_border = floor((dataset.lat_sw + util::delta_lat_lon_deg) / util::delta_lat_lon_deg) *
					    util::delta_lat_lon_deg;
			double lon_border = floor((dataset.lon_sw + util::delta_lat_lon_deg) / util::delta_lat_lon_deg) *
					    util::delta_lat_lon_deg;
			const double increment = 0.00001;
			const int count = 30;

			for (double lat = lat_border - increment * count; lat <= lat_border + increment * count; lat += increment) {
				for (double lon = lon_border - increment * count; lon <= lon_border + increment * count; lon += increment) {
					if (lat > dataset.lat_sw && lat < dataset.lat_ne && lon > dataset.lon_sw && lon < dataset.lon_ne) {
						ASSERT_NO_FATAL_FAILURE(
							lookupWaitAndCheck(provider, dataset, lat, lon);
						);
					}
				}
			}
		}
	}
}
