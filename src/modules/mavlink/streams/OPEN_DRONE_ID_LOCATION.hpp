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

#ifndef OPEN_DRONE_ID_LOCATION
#define OPEN_DRONE_ID_LOCATION

#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>

class MavlinkStreamOpenDroneIdLocation : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpenDroneIdLocation(mavlink); }

	static constexpr const char *get_name_static() { return "OPEN_DRONE_ID_LOCATION"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	bool _airborne = false;
	bool _in_failsafe = false;
	mavlink_open_drone_id_location_t _message = {0};

	explicit MavlinkStreamOpenDroneIdLocation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		// Set the message contents to the "UNKNOWN" init values
		_message.latitude = 0;
		_message.longitude = 0;
		_message.altitude_barometric = -1000;
		_message.altitude_geodetic = -1000;
		_message.speed_horizontal = 25500;
		_message.speed_vertical = 6300;
		_message.direction = 36100;
		_message.height = -1000;
	}

	bool send() override
	{
		bool updated = false;

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);
			_in_failsafe = vehicle_status.failsafe || vehicle_status.failsafe_but_user_took_over;
			updated = true;
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;
			_vehicle_land_detected_sub.copy(&vehicle_land_detected);
			_airborne = !vehicle_land_detected.landed;
			updated = true;
		}

		_message.target_component = 0;
		_message.target_system = 0;
		_message.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
		_message.horizontal_accuracy = MAV_ODID_HOR_ACC_UNKNOWN;
		_message.vertical_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
		_message.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
		_message.speed_accuracy = MAV_ODID_SPEED_ACC_UNKNOWN;

		_message.status = MAV_ODID_STATUS_GROUND;

		if (_airborne) {
			_message.status = MAV_ODID_STATUS_AIRBORNE;
		}

		if (_in_failsafe) {
			_message.status = MAV_ODID_STATUS_EMERGENCY;
		}

		if (_vehicle_global_position_sub.updated()) {
			vehicle_global_position_s vgp;
			_vehicle_global_position_sub.copy(&vgp);
			_message.latitude = static_cast<int32_t>(vgp.lat * 1e7);
			_message.longitude = static_cast<int32_t>(vgp.lon * 1e7);
			_message.altitude_barometric = vgp.alt;
			_message.altitude_geodetic = vgp.alt_ellipsoid;
			updated = true;
		}


		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vlp;
			_vehicle_local_position_sub.copy(&vlp);
			float speed_xy = sqrtf(vlp.vx * vlp.vx + vlp.vy * vlp.vy);
			float direction_deg = (atan2f(vlp.vy, vlp.vx) + M_PI_F) * 180.f / M_PI_F;
			_message.speed_horizontal = vlp.v_xy_valid ?
						    static_cast<uint16_t>(math::min(25425.f, speed_xy * 100.f)) : 25500;
			_message.speed_vertical = vlp.v_z_valid ?
						  static_cast<int16_t>(math::min(6200.f, math::max(-6200.f, -vlp.vz * 100.f))) : 6300;
			_message.direction = vlp.v_xy_valid ? static_cast<uint16_t>(direction_deg * 100.f) : 36100;
			_message.height = vlp.z_valid ? -vlp.z : -1000;
			updated = true;
		}

		struct timespec ts = {};

		px4_clock_gettime(CLOCK_REALTIME, &ts);

		int64_t utc_time_msec = static_cast<int64_t>(ts.tv_sec) * 1000L + (static_cast<int64_t>(ts.tv_nsec) / 1e6L);

		_message.timestamp = static_cast<float>(utc_time_msec % 3600000L) / 1000.f;

		_message.timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;

		if (updated) {
			mavlink_msg_open_drone_id_location_send_struct(_mavlink->get_channel(), &_message);
			return true;
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_LOCATION
