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
#include <pthread.h>

#include "util.h"

#include <lib/geo/geo.h>
#include <px4_platform_common/atomic.h>

namespace terrain
{

/**
 * @class TerrainProvider
 * This provides terrain lookup from disk, with an LRU cache in RAM.
 * If not in cache, data is loaded async with an IO thread.
 * Make sure to only keep one instance as it is expensive on memory usage.
 */
class TerrainProvider
{
public:
	TerrainProvider(int grid_spacing_m);
	virtual ~TerrainProvider();

	/**
	 * lookup altitude data. This method returns immediately, triggering a caching request if data is not
	 * cached already.
	 * Note: this can be called concurrently from different threads.
	 * @param lat lookup latitude in [-90, 90]
	 * @param lon lookup longitude in [-180, 180]
	 * @param alt_msl_m output altitude
	 * @return true on success, false if not in cache / outside of data range
	 */
	virtual bool lookup(double lat, double lon, float &alt_msl_m);
private:
	static constexpr int cache_size =
		3; ///< number of cached items. Item size depends on grid_spacing, generally several KB

	enum class CacheState {
		Unused = 0,
		Requested,
		Used
	};

	struct CacheItem {
		~CacheItem() { delete[] data; }

		void reset()
		{
			state = CacheState::Unused;
			used_tick = 0;
		}

		CacheState state{CacheState::Unused};
		unsigned tag_lat{};
		unsigned tag_lon{};
		unsigned used_tick{};
		int x_count{};
		int y_count{};
		MapProjection ref{};
		int16_t *data{nullptr};
	};

	int threadStart();
	void threadStop();

	static void *ioThreadEntry(void *);
	void runThread();

	void lock()
	{
		pthread_mutex_lock(&_mtx);
	}

	void unlock()
	{
		pthread_mutex_unlock(&_mtx);
	}

	/**
	 * Wakeup the IO thread. Must be called in locked state to avoid lost wakeups.
	 */
	void wakeup()
	{
		pthread_cond_broadcast(&_cv);
	}

	/**
	 * Load data from disk into cache.
	 * Does not change the item's state.
	 * Called from the IO thread, in unlocked state
	 * @return true on success
	 */
	bool loadCache(CacheItem &item);

	bool getElevationFromCache(CacheItem &item, double lat, double lon, float &elevation_m);

	const int _grid_spacing_m;
	int _cache_data_size;

	CacheItem _cache[cache_size];
	unsigned _current_tick{};

	pthread_mutex_t _mtx;
	pthread_cond_t _cv;
	pthread_t _thread{0};

	px4::atomic_bool _should_exit{false};
};

}
