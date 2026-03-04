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
#include <uORB/topics/sensor_gnss_status.h>
#include <uORB/PublicationMulti.hpp>

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
		return _vehicle_gps_position_sub.advertised() ? (MAVLINK_MSG_ID_GNSS_INTEGRITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	static constexpr int GPS_MAX_RECEIVERS = 2;

	explicit MavlinkStreamGNSSIntegrity(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::SubscriptionMultiArray<sensor_gnss_status_s, GPS_MAX_RECEIVERS> _sensor_gnss_status_sub{ORB_ID::sensor_gnss_status};

	bool send() override
	{
		sensor_gps_s vehicle_gps_position{};

		if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {
			mavlink_gnss_integrity_t msg{};

			msg.id = vehicle_gps_position.device_id;
			msg.system_errors = vehicle_gps_position.system_error;
			msg.authentication_state = vehicle_gps_position.authentication_state;
			msg.jamming_state = vehicle_gps_position.jamming_state;
			msg.spoofing_state = vehicle_gps_position.spoofing_state;

			msg.corrections_quality = UINT8_MAX;
			msg.system_status_summary = UINT8_MAX;
			msg.gnss_signal_quality = UINT8_MAX;
			msg.post_processing_quality = UINT8_MAX;

			for (int i = 0; i < GPS_MAX_RECEIVERS; i++) {
				sensor_gnss_status_s sensor_gnss_status{};

				if (_sensor_gnss_status_sub[i].copy(&sensor_gnss_status)) {
					if ((hrt_elapsed_time(&sensor_gnss_status.timestamp) < 3_s)
					    && (sensor_gnss_status.device_id == vehicle_gps_position.device_id)
					    && (sensor_gnss_status.quality_available)) {
						msg.corrections_quality = sensor_gnss_status.quality_corrections;
						msg.system_status_summary = sensor_gnss_status.quality_receiver;
						msg.gnss_signal_quality = sensor_gnss_status.quality_gnss_signals;
						msg.post_processing_quality = sensor_gnss_status.quality_post_processing;
						break;
					}
				}
			}

			msg.raim_hfom = UINT16_MAX;
			msg.raim_vfom = UINT16_MAX;

			mavlink_msg_gnss_integrity_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}

};

#endif // GNSS_INTEGRITY_HPP
