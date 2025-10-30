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

#ifndef OPEN_DRONE_ID_LOCATION_HPP
#define OPEN_DRONE_ID_LOCATION_HPP

#include <uORB/topics/home_position.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>

static constexpr unsigned MAX_GPS_SENS = 4;

using namespace time_literals;

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
	explicit MavlinkStreamOpenDroneIdLocation(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};

	bool send() override
	{
		mavlink_open_drone_id_location_t msg{};
		msg.target_component = 0; // 0 for broadcast
		msg.target_system = 0; // 0 for broadcast
		// msg.id_or_mac // Only used for drone ID data received from other UAs.

		// initialize all fields to unknown
		msg.status = MAV_ODID_STATUS_UNDECLARED;
		msg.direction = 36100; // If unknown: 36100 centi-degrees
		msg.speed_horizontal = 25500; // If unknown: 25500 cm/s
		msg.speed_vertical = 6300; // If unknown: 6300 cm/s
		msg.latitude = 0; // If unknown: 0
		msg.longitude = 0; // If unknown: 0
		msg.altitude_geodetic = -1000; // If unknown: -1000 m
		msg.altitude_geodetic = -1000; // If unknown: -1000 m
		msg.height = -1000; // If unknown: -1000 m
		msg.horizontal_accuracy = MAV_ODID_HOR_ACC_UNKNOWN;
		msg.vertical_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
		msg.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN;
		msg.speed_accuracy = MAV_ODID_SPEED_ACC_UNKNOWN;
		msg.timestamp = 0xFFFF; // If unknown: 0xFFFF
		msg.timestamp_accuracy = MAV_ODID_TIME_ACC_UNKNOWN;

		bool updated = false;
		const uint8_t num_gps_instances = orb_group_count(ORB_ID(sensor_gps));
		const bool vehicle_has_gps = num_gps_instances > 0;
		bool gps_timeout = true;

		if (num_gps_instances > MAX_GPS_SENS) {
			PX4_WARN("Number of GPS instances exceeds maximum (%d)", MAX_GPS_SENS);
			msg.status = MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
			updated = true;

		} else {

			for (uint8_t gps_index = 0; gps_index < num_gps_instances; gps_index++) { // Check all GPS instances if any present
				uORB::SubscriptionMultiArray<sensor_gps_s, MAX_GPS_SENS> sensor_gps_sub{ORB_ID::sensor_gps};

				sensor_gps_s sensor_gps_data;
				sensor_gps_sub[gps_index].update(&sensor_gps_data);

				if (hrt_elapsed_time(&sensor_gps_data.timestamp) < 3_s) {
					gps_timeout = false;
					break;
				}
			}

			if (!vehicle_has_gps || gps_timeout) {
				msg.status = MAV_ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
				updated = true;
			}
		}

		// status: MAV_ODID_STATUS_GROUND/MAV_ODID_STATUS_AIRBORNE
		if (_vehicle_land_detected_sub.advertised()) {
			vehicle_land_detected_s vehicle_land_detected{};

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)
			    && (hrt_elapsed_time(&vehicle_land_detected.timestamp) < 10_s)) {
				if (vehicle_land_detected.landed) {
					msg.status = MAV_ODID_STATUS_GROUND;

				} else {
					msg.status = MAV_ODID_STATUS_AIRBORNE;
				}

				updated = true;
			}
		}

		if (_actuator_armed_sub.updated()) {
			actuator_armed_s actuator_armed{};
			_actuator_armed_sub.copy(&actuator_armed);

			if (actuator_armed.force_failsafe || actuator_armed.lockdown || actuator_armed.manual_lockdown) {
				msg.status = MAV_ODID_STATUS_EMERGENCY;
				updated = true;
			}
		}

		// status: MAV_ODID_STATUS_EMERGENCY
		if (_vehicle_status_sub.advertised()) {
			vehicle_status_s vehicle_status{};

			if (_vehicle_status_sub.copy(&vehicle_status) && hrt_elapsed_time(&vehicle_status.timestamp) < 10_s) {
				if (vehicle_status.failsafe && (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
					msg.status = MAV_ODID_STATUS_EMERGENCY;
					updated = true;

				} else if (vehicle_status.failsafe_and_user_took_over
					   || vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION) {
					msg.status = MAV_ODID_STATUS_EMERGENCY;
					updated = true;
				}
			}
		}

		if (_vehicle_gps_position_sub.advertised()) {
			sensor_gps_s vehicle_gps_position{};

			if (_vehicle_gps_position_sub.copy(&vehicle_gps_position)
			    && (hrt_elapsed_time(&vehicle_gps_position.timestamp) < 10_s)) {

				if (vehicle_gps_position.vel_ned_valid) {
					const matrix::Vector3f vel_ned{vehicle_gps_position.vel_n_m_s, vehicle_gps_position.vel_e_m_s, vehicle_gps_position.vel_d_m_s};

					// direction: calculate GPS course over ground angle
					const float course = atan2f(vel_ned(1), vel_ned(0));
					const int course_deg = roundf(math::degrees(matrix::wrap_2pi(course)));
					msg.direction = math::constrain(100 * course_deg, 0, 35999); // 0 - 35999 centi-degrees

					// speed_horizontal: If speed is larger than 25425 cm/s, use 25425 cm/s.
					const int speed_horizontal_cm_s = matrix::Vector2f(vel_ned).length() * 100.f;
					msg.speed_horizontal = math::constrain(speed_horizontal_cm_s, 0, 25425);

					// speed_vertical: Up is positive, If speed is larger than 6200 cm/s, use 6200 cm/s. If lower than -6200 cm/s, use -6200 cm/s.
					const int speed_vertical_cm_s = roundf(-vel_ned(2) * 100.f);
					msg.speed_vertical = math::constrain(speed_vertical_cm_s, -6200, 6200);

					msg.speed_accuracy = open_drone_id_translations::odidSpeedAccForVariance(vehicle_gps_position.s_variance_m_s);

					updated = true;
				}

				if (vehicle_gps_position.fix_type >= 2) {
					msg.latitude = static_cast<int32_t>(round(vehicle_gps_position.latitude_deg * 1e7));
					msg.longitude = static_cast<int32_t>(round(vehicle_gps_position.longitude_deg * 1e7));

					// altitude_geodetic
					if (vehicle_gps_position.fix_type >= 3) {
						msg.altitude_geodetic = static_cast<float>(round(vehicle_gps_position.altitude_msl_m)); // [m]
					}

					msg.horizontal_accuracy = open_drone_id_translations::odidHorAccForEph(vehicle_gps_position.eph);

					msg.vertical_accuracy = open_drone_id_translations::odidVerAccForEpv(vehicle_gps_position.epv);

					updated = true;
				}

				if (vehicle_gps_position.time_utc_usec != 0) {
					// timestamp: UTC then convert for this field using ((float) (time_week_ms % (60*60*1000))) / 1000
					uint64_t utc_time_msec = vehicle_gps_position.time_utc_usec / 1000;
					msg.timestamp = ((float)(utc_time_msec % (60 * 60 * 1000))) / 1000;

					msg.timestamp_accuracy = open_drone_id_translations::odidTimeForElapsed(hrt_elapsed_time(
									 &vehicle_gps_position.timestamp));

					updated = true;
				}
			}
		}

		// altitude_barometric: The altitude calculated from the barometric pressue
		if (_vehicle_air_data_sub.advertised()) {
			vehicle_air_data_s vehicle_air_data{};

			if (_vehicle_air_data_sub.copy(&vehicle_air_data) && (hrt_elapsed_time(&vehicle_air_data.timestamp) < 10_s)) {
				msg.altitude_barometric = vehicle_air_data.baro_alt_meter;
				msg.barometer_accuracy = MAV_ODID_VER_ACC_UNKNOWN; // We just don't without calibration.
				updated = true;
			}
		}

		// height: The current height of the unmanned aircraft above the take-off location or the ground as indicated by height_reference
		if (_home_position_sub.advertised() && _vehicle_local_position_sub.updated()) {
			home_position_s home_position{};
			vehicle_local_position_s vehicle_local_position{};

			if (_home_position_sub.copy(&home_position)
			    && _vehicle_local_position_sub.copy(&vehicle_local_position)
			    && (hrt_elapsed_time(&vehicle_local_position.timestamp) < 1_s)
			   ) {

				if (home_position.valid_alt && vehicle_local_position.z_valid && vehicle_local_position.z_global) {
					float altitude = (-vehicle_local_position.z + vehicle_local_position.ref_alt);

					msg.height = altitude - home_position.alt;
					msg.height_reference = MAV_ODID_HEIGHT_REF_OVER_TAKEOFF;
					updated = true;
				}
			}
		}

		if (updated) {
			mavlink_msg_open_drone_id_location_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_LOCATION_HPP
