/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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

#include "remoteid.hpp"
#include <modules/mavlink/open_drone_id_translations.hpp>
#include <drivers/drv_hrt.h>

using namespace time_literals;


UavcanRemoteIDController::UavcanRemoteIDController(uavcan::INode &node) :
	ModuleParams(nullptr),
	_timer(node),
	_node(node),
	_uavcan_pub_remoteid_basicid(node),
	_uavcan_pub_remoteid_location(node),
	_uavcan_pub_remoteid_self_id(node),
	_uavcan_pub_remoteid_system(node),
	_uavcan_pub_remoteid_operator_id(node),
	_uavcan_sub_arm_status(node)
{
}

int UavcanRemoteIDController::init()
{
	// Setup timer and call back function for periodic updates
	_timer.setCallback(TimerCbBinder(this, &UavcanRemoteIDController::periodic_update));
	_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));

	int res = _uavcan_sub_arm_status.start(ArmStatusBinder(this, &UavcanRemoteIDController::arm_status_sub_cb));

	if (res < 0) {
		PX4_WARN("ArmStatus sub failed %i", res);
		return res;
	}

	return 0;
}

void UavcanRemoteIDController::periodic_update(const uavcan::TimerEvent &)
{
	_vehicle_status.update();

	send_basic_id();
	send_location();
	send_self_id();
	send_system();
	send_operator_id();
}

void UavcanRemoteIDController::send_basic_id()
{
	dronecan::remoteid::BasicID basic_id {};
	// basic_id.id_or_mac // supposedly only used for drone ID data from other UAs
	basic_id.id_type = dronecan::remoteid::BasicID::ODID_ID_TYPE_SERIAL_NUMBER;
	basic_id.ua_type = static_cast<uint8_t>(open_drone_id_translations::odidTypeForMavType(
			_vehicle_status.get().system_type));

	// uas_id: UAS (Unmanned Aircraft System) ID following the format specified by id_type
	// TODO: MAV_ODID_ID_TYPE_SERIAL_NUMBER needs to be ANSI/CTA-2063 format

	char uas_id[20] = {};
	board_get_px4_guid_formated((char *)(uas_id), sizeof(uas_id));
	basic_id.uas_id = uas_id;

	_uavcan_pub_remoteid_basicid.broadcast(basic_id);
}

void UavcanRemoteIDController::send_location()
{
	dronecan::remoteid::Location msg {};

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

	if (hrt_elapsed_time(&_vehicle_status.get().timestamp) < 10_s) {
		if (_vehicle_status.get().failsafe && (_vehicle_status.get().arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
			msg.status = MAV_ODID_STATUS_EMERGENCY;
			updated = true;
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
		_uavcan_pub_remoteid_location.broadcast(msg);
	}
}

void UavcanRemoteIDController::send_system()
{
	open_drone_id_system_s system;

	if (_open_drone_id_system.advertised() && _open_drone_id_system.copy(&system)) {

		// Use what ground station sends us.

		dronecan::remoteid::System msg {};
		msg.timestamp = system.timestamp;

		for (unsigned i = 0; i < sizeof(system.id_or_mac); ++i) {
			msg.id_or_mac.push_back(system.id_or_mac[i]);
		}

		msg.operator_location_type = system.operator_location_type;
		msg.classification_type = system.classification_type;
		msg.operator_latitude = system.operator_latitude;
		msg.operator_longitude = system.operator_longitude;
		msg.area_count = system.area_count;
		msg.area_radius = system.area_radius;
		msg.area_ceiling = system.area_ceiling;
		msg.area_floor = system.area_floor;
		msg.category_eu = system.category_eu;
		msg.class_eu = system.class_eu;
		msg.operator_altitude_geo = system.operator_altitude_geo;

		_uavcan_pub_remoteid_system.broadcast(msg);

	} else {
		// And otherwise, send our home/takeoff location.

		sensor_gps_s vehicle_gps_position;
		home_position_s home_position;

		if (_vehicle_gps_position_sub.copy(&vehicle_gps_position) && _home_position_sub.copy(&home_position)) {
			if (vehicle_gps_position.fix_type >= 3
			    && home_position.valid_alt && home_position.valid_hpos) {

				dronecan::remoteid::System msg {};

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

				_uavcan_pub_remoteid_system.broadcast(msg);
			}
		}
	}
}

void UavcanRemoteIDController::send_self_id()
{
	open_drone_id_self_id_s self_id;

	if (_open_drone_id_self_id.copy(&self_id)) {

		dronecan::remoteid::SelfID msg {};

		for (unsigned i = 0; i < sizeof(self_id.id_or_mac); ++i) {
			msg.id_or_mac.push_back(self_id.id_or_mac[i]);
		}

		msg.description_type = self_id.description_type;

		for (unsigned i = 0; i < sizeof(self_id.description); ++i) {
			msg.description.push_back(self_id.description[i]);
		}

		_uavcan_pub_remoteid_self_id.broadcast(msg);
	}
}

void UavcanRemoteIDController::send_operator_id()
{
	open_drone_id_operator_id_s operator_id;

	if (_open_drone_id_operator_id.copy(&operator_id)) {

		dronecan::remoteid::OperatorID msg {};

		for (unsigned i = 0; i < sizeof(operator_id.id_or_mac); ++i) {
			msg.id_or_mac.push_back(operator_id.id_or_mac[i]);
		}

		msg.operator_id_type = operator_id.operator_id_type;

		for (unsigned i = 0; i < sizeof(operator_id.operator_id); ++i) {
			msg.operator_id.push_back(operator_id.operator_id[i]);
		}

		_uavcan_pub_remoteid_operator_id.broadcast(msg);
	}
}

void
UavcanRemoteIDController::arm_status_sub_cb(const uavcan::ReceivedDataStructure<dronecan::remoteid::ArmStatus> &msg)
{
	open_drone_id_arm_status_s arm_status{};

	arm_status.timestamp = hrt_absolute_time();
	arm_status.status = msg.status;
	memcpy(arm_status.error, msg.error.c_str(), sizeof(arm_status.error));

	_open_drone_id_arm_status_pub.publish(arm_status);
}
