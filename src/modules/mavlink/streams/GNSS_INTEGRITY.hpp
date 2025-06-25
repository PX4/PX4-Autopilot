/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#ifndef GNSS_INTEGRITY_HPP
#define GNSS_INTEGRITY_HPP

#include <uORB/topics/sensor_gps.h>

using namespace time_literals;

class MavlinkStreamGNSSIntegrity : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamGNSSIntegrity(mavlink); }

	static constexpr const char *get_name_static() { return "GNSS_INTEGRITY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_GNSS_INTEGRITY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _sensor_gps_sub.advertised() ? (MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	explicit MavlinkStreamGNSSIntegrity(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps), 0};
	hrt_abstime _last_send_ts {};
	static constexpr hrt_abstime kNoGpsSendInterval {1_s};

	bool send() override
	{
		sensor_gps_s gps;
		mavlink_gnss_integrity_t msg{};
		hrt_abstime now{};

		if (_sensor_gps_sub.update(&gps)) {
			msg.id = gps.device_id;
			msg.system_errors = gps.system_error;
			msg.authentication_state = gps.authentication_state;
			msg.jamming_state = gps.jamming_state;
			msg.spoofing_state = gps.spoofing_state;

			msg.raim_state = 0;
			msg.raim_hfom = UINT16_MAX;
			msg.raim_vfom = UINT16_MAX;

			mavlink_msg_gnss_integrity_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = gps.timestamp;

			return true;

		} else if (_last_send_ts != 0 && (now = hrt_absolute_time()) > _last_send_ts + kNoGpsSendInterval) {
			msg.raim_hfom = UINT16_MAX;
			msg.raim_vfom = UINT16_MAX;

			mavlink_msg_gnss_integrity_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = now;

			return true;
		}

		return false;
	}
};

#endif // GNSS_INTEGRITY_HPP
