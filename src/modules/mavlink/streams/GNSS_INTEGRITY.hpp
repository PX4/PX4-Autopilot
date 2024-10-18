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

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_ANTENNA) {
				msg.system_errors |= 8;
			}

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_SOFTWARE) {
				msg.system_errors |= 4;
			}

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_CPU_OVERLOAD) {
				msg.system_errors |= 32;
			}

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_CONFIGURATION) {
				msg.system_errors |= 2;
			}

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_EVENT_CONGESTION) {
				msg.system_errors |= 16;
			}

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_OUTPUT_CONGESTION) {
				msg.system_errors |= 64;
			}

			if (gps.system_error & sensor_gps_s::SYSTEM_ERROR_INCOMING_CORRECTIONS) {
				msg.system_errors |= 1;
			}

			switch (gps.authentication_state) {
			case sensor_gps_s::AUTHENTICATION_STATE_UNKNOWN:
				msg.authentication_state = 0;
				break;

			case sensor_gps_s::AUTHENTICATION_STATE_DISABLED:
				msg.authentication_state = 4;
				break;

			case sensor_gps_s::AUTHENTICATION_STATE_INITIALIZING:
				msg.authentication_state = 1;
				break;

			case sensor_gps_s::AUTHENTICATION_STATE_FAILED:
				msg.authentication_state = 2;
				break;

			case sensor_gps_s::AUTHENTICATION_STATE_OK:
				msg.authentication_state = 3;
				break;
			}

			switch (gps.jamming_state) {
			case sensor_gps_s::JAMMING_STATE_UNKNOWN:
				msg.jamming_state = 0;
				break;

			case sensor_gps_s::JAMMING_STATE_OK:
				msg.jamming_state = 1;
				break;

			case sensor_gps_s::JAMMING_STATE_MITIGATED:
				msg.jamming_state = 2;
				break;

			case sensor_gps_s::JAMMING_STATE_WARNING:
			case sensor_gps_s::JAMMING_STATE_CRITICAL:
				msg.jamming_state = 3;
				break;
			}

			switch (gps.spoofing_state) {
			case sensor_gps_s::SPOOFING_STATE_UNKNOWN:
				msg.spoofing_state = 0;
				break;

			case sensor_gps_s::SPOOFING_STATE_NONE:
				msg.spoofing_state = 1;
				break;

			case sensor_gps_s::SPOOFING_STATE_MITIGATED:
				msg.spoofing_state = 2;
				break;

			case sensor_gps_s::SPOOFING_STATE_INDICATED:
			case sensor_gps_s::SPOOFING_STATE_MULTIPLE:
				msg.spoofing_state = 3;
				break;
			}

			msg.raim_state = 0;
			msg.raim_hfom = UINT16_MAX;
			msg.raim_vfom = UINT16_MAX;
			msg.corrections_quality = gps.quality_corrections;
			msg.system_status_summary = gps.quality_receiver;
			msg.gnss_signal_quality = gps.quality_gnss_signals;
			msg.post_processing_quality = gps.quality_post_processing;

			mavlink_msg_gnss_integrity_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = gps.timestamp;

			return true;

		} else if (_last_send_ts != 0 && (now = hrt_absolute_time()) > _last_send_ts + kNoGpsSendInterval) {
			msg.raim_hfom = UINT16_MAX;
			msg.raim_vfom = UINT16_MAX;
			msg.corrections_quality = UINT8_MAX;
			msg.system_status_summary = UINT8_MAX;
			msg.gnss_signal_quality = UINT8_MAX;
			msg.post_processing_quality = UINT8_MAX;

			mavlink_msg_gnss_integrity_send_struct(_mavlink->get_channel(), &msg);
			_last_send_ts = now;

			return true;
		}

		return false;
	}
};

#endif // GNSS_INTEGRITY_HPP
