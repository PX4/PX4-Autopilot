/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#ifndef OPEN_DRONE_ID_SYSTEM_HPP
#define OPEN_DRONE_ID_SYSTEM_HPP

#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>

class MavlinkStreamOpenDroneIdSystem : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpenDroneIdSystem(mavlink); }

	static constexpr const char *get_name_static() { return "OPEN_DRONE_ID_SYSTEM"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_gps_position_sub.advertised() && _home_position_sub.advertised()) {
			return MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamOpenDroneIdSystem(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};

	bool send() override
	{
		sensor_gps_s vehicle_gps_position;
		home_position_s home_position;

		if (_vehicle_gps_position_sub.update(&vehicle_gps_position) && _home_position_sub.copy(&home_position)) {
			if (vehicle_gps_position.fix_type >= 3
			    && home_position.valid_alt && home_position.valid_hpos) {

				mavlink_open_drone_id_system_t msg{};
				msg.target_component = 0; // 0 for broadcast
				msg.target_system = 0; // 0 for broadcast
				// msg.id_or_mac // Only used for drone ID data received from other UAs.
				msg.operator_location_type = MAV_ODID_OPERATOR_LOCATION_TYPE_TAKEOFF;
				msg.classification_type = MAV_ODID_CLASSIFICATION_TYPE_UNDECLARED;
				msg.operator_latitude = home_position.lat * 1e7;
				msg.operator_longitude = home_position.lon * 1e7;
				msg.area_count = 1;
				msg.area_radius = 0;
				msg.area_ceiling = -1000;
				msg.area_floor = -1000;
				msg.category_eu = MAV_ODID_CATEGORY_EU_UNDECLARED;
				msg.class_eu = MAV_ODID_CLASS_EU_UNDECLARED;
				float wgs84_amsl_offset = vehicle_gps_position.altitude_ellipsoid_m - vehicle_gps_position.altitude_msl_m;
				msg.operator_altitude_geo = home_position.alt + wgs84_amsl_offset;

				// timestamp: 32 bit Unix Timestamp in seconds since 00:00:00 01/01/2019.
				static uint64_t utc_offset_s = 1'546'300'800; // UTC seconds since 00:00:00 01/01/2019
				msg.timestamp = vehicle_gps_position.time_utc_usec / 1e6 - utc_offset_s;

				mavlink_msg_open_drone_id_system_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_SYSTEM_HPP
