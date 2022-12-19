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

#include "terrain_provider.h"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <lib/mathlib/mathlib.h>

#include <limits.h>

using namespace terrain;


TerrainProvider::TerrainProvider(int grid_spacing_m)
	: _grid_spacing_m(grid_spacing_m)
{
	_cache_data_size = util::maxNumItemsPerFile(_grid_spacing_m);

	// allocate on startup so we know if we have the memory available or not
	for (int i = 0; i < cache_size; ++i) {
		_cache[i].data = new int16_t[_cache_data_size];

		if (!_cache[i].data) {
			PX4_ERR("terrain alloc failed");
		}
	}

	pthread_mutex_init(&_mtx, nullptr);
	pthread_cond_init(&_cv, nullptr);
	int ret = threadStart();

	if (ret != 0) {
		PX4_ERR("Failed to start terrain IO thread (%i)", ret);
	}

}

TerrainProvider::~TerrainProvider()
{
	threadStop();
	pthread_mutex_destroy(&_mtx);
	pthread_cond_destroy(&_cv);
}

int TerrainProvider::threadStart()
{
	pthread_attr_t thr_attr;
	pthread_attr_init(&thr_attr);

	sched_param param;
	/* low priority, as this is disk I/O */
	param.sched_priority = SCHED_PRIORITY_DEFAULT - 41;
	(void)pthread_attr_setschedparam(&thr_attr, &param);

	pthread_attr_setstacksize(&thr_attr, PX4_STACK_ADJUSTED(1170));

	int ret = pthread_create(&_thread, &thr_attr, &ioThreadEntry, this);
	pthread_attr_destroy(&thr_attr);

	return ret;
}

void TerrainProvider::threadStop()
{
	_should_exit.store(true);
	lock();
	wakeup();
	unlock();

	// wait for thread to complete
	int ret = pthread_join(_thread, nullptr);

	if (ret) {
		PX4_ERR("join failed: %d", ret);
	}
}

void *TerrainProvider::ioThreadEntry(void *context)
{
	px4_prctl(PR_SET_NAME, "terrain_io", px4_getpid());
	static_cast<TerrainProvider *>(context)->runThread();
	return nullptr;
}

void TerrainProvider::runThread()
{
	while (true) {
		pthread_mutex_lock(&_mtx);
		// check for requests
		bool had_request = false;

		do {
			had_request = false;

			for (int i = 0; i < cache_size; ++i) {
				if (_cache[i].state == CacheState::Requested) {
					pthread_mutex_unlock(&_mtx);
					had_request = true;
					bool success = loadCache(_cache[i]);
					pthread_mutex_lock(&_mtx);

					if (success) {
						_cache[i].state = CacheState::Used;

					} else {
						_cache[i].reset();
					}
				}
			}
		} while (had_request);

		if (_should_exit.load()) {
			break;
		}

		pthread_cond_wait(&_cv, &_mtx);
		pthread_mutex_unlock(&_mtx);
	}

	pthread_mutex_unlock(&_mtx);
}

bool TerrainProvider::loadCache(CacheItem &item)
{
	if (!item.data) {
		return false;
	}

	char filename[64];
	util::getFileName(item.tag_lat, item.tag_lon, filename, sizeof(filename));

	int fd = open(filename, O_RDONLY, PX4_O_MODE_666);

	if (fd == -1) {
		PX4_DEBUG("failed to open file %s", filename);
		return false;
	}

	FileHeader header;
	uint16_t expected_magic = header.magic;
	uint16_t expected_version = header.version;

	bool success = false;

	if (read(fd, &header, sizeof(header)) == sizeof(header)) {
		if (header.magic == expected_magic && header.version == expected_version &&
		    header.grid_spacing_m == _grid_spacing_m) {

			item.x_count = header.num_points_x;
			item.y_count = header.num_points_y;
			item.ref.initReference(header.lat_sw, header.lon_sw);

			if (item.x_count * item.y_count > _cache_data_size) {
				PX4_ERR("File contains too many items (%i*%i > %i)", item.x_count, item.y_count, _cache_data_size);

			} else {
				int read_size = header.num_points_x * header.num_points_y * sizeof(int16_t);

				if (read(fd, item.data, read_size) == read_size) {
					success = true;
				}
			}
		}
	}

	close(fd);

	PX4_DEBUG("loaded terrain data to cache: lat: %.3f lon: %.3f", header.lat_sw, header.lon_sw);

	item.used_tick = _current_tick++;
	return success;
}

bool TerrainProvider::lookup(double lat, double lon, float &elevation_m)
{
	bool ret = false;
	unsigned tag_lat = util::cacheTagFromLat(lat);
	unsigned tag_lon = util::cacheTagFromLon(lon);

	lock();
	unsigned lowest_tick = UINT_MAX;
	int lowest_tick_idx = -1;
	int found_tag_idx = -1;

	for (int i = 0; i < cache_size; ++i) {
		if (_cache[i].state != CacheState::Unused) {
			if (_cache[i].tag_lat == tag_lat && _cache[i].tag_lon == tag_lon) {
				found_tag_idx = i;
			}
		}

		if (_cache[i].state != CacheState::Requested) {
			// keep track of the lowest used tick: this will be an unused item or the oldest one (LRU)
			if (_cache[i].used_tick < lowest_tick) {
				lowest_tick = _cache[i].used_tick;
				lowest_tick_idx = i;
			}
		}
	}

	if (found_tag_idx == -1) {
		if (lowest_tick_idx != -1) {
			// request item for caching
			CacheItem &item = _cache[lowest_tick_idx];
			item.tag_lat = tag_lat;
			item.tag_lon = tag_lon;
			item.state = CacheState::Requested;
			wakeup();
		} // else: all items currently being requested for caching

	} else {
		if (_cache[found_tag_idx].state == CacheState::Used) {
			ret = getElevationFromCache(_cache[found_tag_idx], lat, lon, elevation_m);
		} // else: currently being requested for caching
	}

	unlock();
	return ret;
}

bool TerrainProvider::getElevationFromCache(CacheItem &item, double lat, double lon, float &elevation_m)
{
	if (!item.ref.isInitialized()) {
		return false;
	}

	float x, y;
	item.ref.project(lat, lon, x, y);

	x /= _grid_spacing_m;
	y /= _grid_spacing_m;
	int ix = x;
	int iy = y;

	if (ix < 0 || ix >= item.x_count - 1) {
		// getting here is a bug, but we don't want to spam the console
		double ref_lat = item.ref.getProjectionReferenceLat();
		double ref_lon = item.ref.getProjectionReferenceLon();
		PX4_DEBUG("x index out of range: %i, %i, x=%.3f, lat=%.3f, ref lat=%.3f, max lat=%.3f, lat/delta=%.3f",
			  ix, item.x_count, (double)x, lat, math::degrees(ref_lat), math::degrees(ref_lat) + util::delta_lat_lon_deg,
			  lat / util::delta_lat_lon_deg);
		(void)ref_lat;
		(void)ref_lon;
		return false;
	}

	if (iy < 0 || iy >= item.y_count - 1) {
		double ref_lat = item.ref.getProjectionReferenceLat();
		double ref_lon = item.ref.getProjectionReferenceLon();
		PX4_DEBUG("y index out of range: %i, %i, y=%.3f, lon=%.3f, ref lon=%.3f, max lon=%.3f, lon/delta=%.3f",
			  iy, item.y_count, (double)y, lon, math::degrees(ref_lon), math::degrees(ref_lon) + util::delta_lat_lon_deg,
			  lon / util::delta_lat_lon_deg);
		(void)ref_lat;
		(void)ref_lon;
		return false;
	}

	// bilinear interpolation
	float data_sw = item.data[iy + ix * item.y_count];
	float data_se = item.data[(iy + 1) + ix * item.y_count];
	float data_ne = item.data[(iy + 1) + (ix + 1) * item.y_count];
	float data_nw = item.data[iy + (ix + 1) * item.y_count];

	float x_scale = x - ix;
	float y_scale = y - iy;

	float data_min = y_scale * (data_se - data_sw) + data_sw;
	float data_max = y_scale * (data_ne - data_nw) + data_nw;
	elevation_m = x_scale * (data_max - data_min) + data_min;

	item.used_tick = _current_tick++;

	return true;
}
