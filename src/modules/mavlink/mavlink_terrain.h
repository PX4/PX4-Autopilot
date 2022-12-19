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

#include <pthread.h>

#include "mavlink_bridge_header.h"
#include "terrain.h"

#include <drivers/drv_hrt.h>
#include <systemlib/mavlink_log.h>

namespace terrain
{

/**
 * @class MavlinkTerrainUploader
 * Wrapper around a static TerrainUploader instance, so it can be used concurrently.
 */
class MavlinkTerrainUploader
{
public:
	MavlinkTerrainUploader() = default;
	~MavlinkTerrainUploader() = default;

	bool init(int grid_spacing_m);
	void deinit();

	bool valid() const { return _instance != nullptr; }

	int updateArea(double lat_sw, double lon_sw, double lat_ne, double lon_ne);

	void update(const mavlink_channel_t &channel, orb_advert_t &mavlink_log_pub);
	void handleTerrainData(mavlink_message_t *msg);

	static void printStatus();

private:
	static void lock()
	{
		pthread_mutex_lock(&_mtx);
	}

	static void unlock()
	{
		pthread_mutex_unlock(&_mtx);
	}

	static TerrainUploader *_instance;
	static pthread_mutex_t _mtx;

	hrt_abstime _next_status_send{0};
	int _current_request_id{0};
};

}
