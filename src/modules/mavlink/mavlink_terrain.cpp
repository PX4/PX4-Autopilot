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

#include <errno.h>

using namespace terrain;

TerrainUploader *MavlinkTerrainUploader::_instance = nullptr;
pthread_mutex_t MavlinkTerrainUploader::_mtx = PTHREAD_MUTEX_INITIALIZER;

bool MavlinkTerrainUploader::init(int grid_spacing_m)
{
	if (_instance) {
		return true;
	}

	_instance = new TerrainUploader(grid_spacing_m);
	return _instance != nullptr;
}

void MavlinkTerrainUploader::deinit()
{
	if (_instance) {
		delete _instance;
		_instance = nullptr;
	}
}

int MavlinkTerrainUploader::updateArea(double lat_sw, double lon_sw, double lat_ne, double lon_ne)
{
	if (!_instance) {
		return -EINVAL;
	}

	lock();
	int ret = _instance->updateArea(hrt_absolute_time(), lat_sw, lon_sw, lat_ne, lon_ne);
	unlock();
	return ret;
}

void MavlinkTerrainUploader::update(const mavlink_channel_t &channel, orb_advert_t &mavlink_log_pub)
{
	if (!_instance) {
		return;
	}

	hrt_abstime now = hrt_absolute_time();

	lock();

	if (now > _next_status_send) {
		mavlink_terrain_report_t report{};
		report.loaded = _instance->num4x4BlocksLoaded();
		report.pending = _instance->num4x4BlocksPending();
		report.spacing = _instance->gridSpacing();
		mavlink_msg_terrain_report_send_struct(channel, &report);
		_next_status_send = now + (report.pending > 0 ? 200_ms : 1_s);
	}

	_instance->update(now);

	const mavlink_terrain_request_t *request;
	int request_id = _instance->getLatestRequest(&request);

	if (request && _current_request_id != request_id) {
		_current_request_id = request_id;
		mavlink_msg_terrain_request_send_struct(channel, request);
		PX4_DEBUG("Sending request: lat=%.3f, lon=%.3f, grid_spacing=%i, ", request->lat * 1e-7, request->lon * 1e-7,
			  request->grid_spacing);
	}

	TerrainUploader::Error latest_error = _instance->getLatestErrorAndClear();

	unlock();

	switch (latest_error) {
	case TerrainUploader::Error::DirectoryCreationFailure:
		mavlink_log_warning(&mavlink_log_pub, "terrain upload: directory creation failed");
		break;

	case TerrainUploader::Error::LatitudeExceeded:
		mavlink_log_warning(&mavlink_log_pub, "terrain upload: latitude exceeded");
		break;

	case TerrainUploader::Error::StorageFailure:
		mavlink_log_warning(&mavlink_log_pub, "terrain upload: storage failure (SD card full?)");
		break;

	case TerrainUploader::Error::None:
		break;
	}
}

void MavlinkTerrainUploader::handleTerrainData(mavlink_message_t *msg)
{
	if (!_instance) {
		return;
	}

	mavlink_terrain_data_t terrain_data;
	mavlink_msg_terrain_data_decode(msg, &terrain_data);
	lock();
	_instance->handleTerrainData(hrt_absolute_time(), terrain_data);
	unlock();
}

void MavlinkTerrainUploader::printStatus()
{
	if (!_instance) {
		return;
	}

	lock();
	_instance->printStatus();
	unlock();
}
