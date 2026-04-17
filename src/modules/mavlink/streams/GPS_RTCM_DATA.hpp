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

#ifndef GPS_RTCM_DATA_HPP
#define GPS_RTCM_DATA_HPP

#include <uORB/topics/gps_inject_data.h>

class MavlinkStreamGPSRTCMData : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGPSRTCMData(mavlink); }

	static constexpr const char *get_name_static() { return "GPS_RTCM_DATA"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GPS_RTCM_DATA; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _gps_inject_data_sub.advertised() ? (MAVLINK_MSG_ID_GPS_RTCM_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGPSRTCMData(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _gps_inject_data_sub{ORB_ID(gps_inject_data), 0};

	bool send() override
	{
		gps_inject_data_s gps_inject_data;
		bool sent = false;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _gps_inject_data_sub.update(&gps_inject_data)) {
			mavlink_gps_rtcm_data_t msg{};

			msg.len = gps_inject_data.len;
			msg.flags = gps_inject_data.flags;
			memcpy(msg.data, gps_inject_data.data, sizeof(msg.data));

			mavlink_msg_gps_rtcm_data_send_struct(_mavlink->get_channel(), &msg);

			sent = true;
		}

		return sent;
	}
};

#endif // GPS_RTCM_DATA_HPP
