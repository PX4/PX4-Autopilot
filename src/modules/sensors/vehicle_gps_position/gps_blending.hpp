/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file gps_blending.hpp
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <uORB/topics/sensor_gnss.h>

using matrix::Vector3f;

using namespace time_literals;

class GpsBlending
{
public:
	// Set the GPS timeout to 2s, after which a receiver will be ignored
	static constexpr hrt_abstime GPS_TIMEOUT_US = 2_s;

	GpsBlending() = default;
	~GpsBlending() = default;

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS_BLEND = 2;

	void setGnssData(const sensor_gnss_s &gnss_data, uint8_t instance)
	{
		if (instance < GPS_MAX_RECEIVERS_BLEND) {
			_gnss_state[instance] = gnss_data;
			_gps_updated[instance] = true;
		}
	}
	void setPrimaryInstance(int primary) { _primary_instance = primary; }
	void setAntennaOffset(const Vector3f &offset, uint8_t instance)
	{
		if (instance < GPS_MAX_RECEIVERS_BLEND) { _antenna_offset[instance] = offset; }
	}
	const Vector3f &getOutputAntennaOffset() const { return _output_antenna_offset; }

	void update(uint64_t hrt_now_us);

	bool isNewOutputDataAvailable() const { return _is_new_output_data_available; }
	const sensor_gnss_s &getOutputGnssData() const { return _gnss_state[_selected_gps]; }
	int getSelectedGps() const { return _selected_gps; }

private:
	// Drop the stored fix of a receiver that stopped publishing, and track whether the primary one is publishing
	void updateReceiverTimeouts(uint64_t hrt_now_us);

	sensor_gnss_s _gnss_state[GPS_MAX_RECEIVERS_BLEND] {}; ///< internal state data for the physical GPS
	bool _gps_updated[GPS_MAX_RECEIVERS_BLEND] {};
	int _selected_gps{0};
	int _primary_instance{0}; ///< if -1, there is no primary isntance and the best receiver is used // TODO: use device_id
	bool _primary_instance_available{false};

	bool _is_new_output_data_available{false};

	uint64_t _time_prev_us[GPS_MAX_RECEIVERS_BLEND] {};	///< the previous value of time_us for that GPS instance - used to detect new data.

	Vector3f _antenna_offset[GPS_MAX_RECEIVERS_BLEND] {};
	Vector3f _output_antenna_offset {};
};
