/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"
#include "mavlink_simple_analyzer.h"
#include "mavlink_high_latency2.h"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_pwm_output.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_time.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/orbit_status.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_mag.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/uORB.h>

using matrix::wrap_2pi;

static uint16_t cm_uint16_from_m_float(float m);

static void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
				   uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

uint16_t
cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_navigation_mode(const struct vehicle_status_s *const status, uint8_t *mavlink_base_mode,
				 union px4_custom_mode *custom_mode)
{
	custom_mode->data = 0;
	*mavlink_base_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTGS;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}
}

void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
			    uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	union px4_custom_mode custom_mode;
	get_mavlink_navigation_mode(status, mavlink_base_mode, &custom_mode);
	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_REBOOT) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}


class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &) = delete;
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &) = delete;

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	bool send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};

		/* always send the heartbeat, independent of the update status of the topics */
		if (!_status_sub->update(&status)) {
			/* if topic update failed fill it with defaults */
			memset(&status, 0, sizeof(status));
		}

		uint8_t base_mode = 0;
		uint32_t custom_mode = 0;
		uint8_t system_status = 0;
		get_mavlink_mode_state(&status, &system_status, &base_mode, &custom_mode);

		mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
					   base_mode, custom_mode, system_status);

		return true;
	}
};

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamStatustext::get_name_static();
	}

	static const char *get_name_static()
	{
		return "STATUSTEXT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATUSTEXT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStatustext(mavlink);
	}

	unsigned get_size()
	{
		return _mavlink->get_logbuffer()->empty() ? 0 : (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamStatustext(MavlinkStreamStatustext &) = delete;
	MavlinkStreamStatustext &operator = (const MavlinkStreamStatustext &) = delete;

protected:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t)
	{
		if (!_mavlink->get_logbuffer()->empty() && _mavlink->is_connected()) {

			struct mavlink_log_s mavlink_log = {};

			if (_mavlink->get_logbuffer()->get(&mavlink_log)) {

				mavlink_statustext_t msg;
				msg.severity = mavlink_log.severity;
				strncpy(msg.text, (const char *)mavlink_log.text, sizeof(msg.text));
				msg.text[sizeof(msg.text) - 1] = '\0';

				mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCommandLong : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCommandLong::get_name_static();
	}

	static const char *get_name_static()
	{
		return "COMMAND_LONG";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCommandLong(mavlink);
	}

	unsigned get_size()
	{
		return 0;	// commands stream is not regular and not predictable
	}

private:
	MavlinkOrbSubscription *_cmd_sub;

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &) = delete;
	MavlinkStreamCommandLong &operator = (const MavlinkStreamCommandLong &) = delete;

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink),
		_cmd_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_command), 0, true))
	{}

	bool send(const hrt_abstime t)
	{
		struct vehicle_command_s cmd;
		bool sent = false;

		if (_cmd_sub->update_if_changed(&cmd)) {

			if (!cmd.from_external) {
				PX4_DEBUG("sending command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);

				MavlinkCommandSender::instance().handle_vehicle_command(cmd, _mavlink->get_channel());
				sent = true;

			} else {
				PX4_DEBUG("not forwarding command %d to %d/%d", cmd.command, cmd.target_system, cmd.target_component);
			}
		}

		MavlinkCommandSender::instance().check_timeout(_mavlink->get_channel());

		return sent;
	}
};

class MavlinkStreamSysStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSysStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYS_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSysStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYS_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	MavlinkOrbSubscription *_cpuload_sub;
	MavlinkOrbSubscription *_battery_status_sub;

	uint64_t _status_timestamp{0};
	uint64_t _cpuload_timestamp{0};
	uint64_t _battery_status_timestamp{0};

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &) = delete;
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &) = delete;

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_cpuload_sub(_mavlink->add_orb_subscription(ORB_ID(cpuload))),
		_battery_status_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_status_s status = {};
		cpuload_s cpuload = {};
		battery_status_s battery_status = {};

		const bool updated_status = _status_sub->update(&_status_timestamp, &status);
		const bool updated_cpuload = _cpuload_sub->update(&_cpuload_timestamp, &cpuload);
		const bool updated_battery = _battery_status_sub->update(&_battery_status_timestamp, &battery_status);

		if (updated_status || updated_battery || updated_cpuload) {
			mavlink_sys_status_t msg = {};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;
			msg.load = cpuload.load * 1000.0f;
			msg.voltage_battery = (battery_status.connected) ? battery_status.voltage_filtered_v * 1000.0f : UINT16_MAX;
			msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100.0f : -1;
			msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;
			// TODO: fill in something useful in the fields below
			msg.drop_rate_comm = 0;
			msg.errors_comm = 0;
			msg.errors_count1 = 0;
			msg.errors_count2 = 0;
			msg.errors_count3 = 0;
			msg.errors_count4 = 0;

			mavlink_msg_sys_status_send_struct(_mavlink->get_channel(), &msg);

			/* battery status message with higher resolution */
			mavlink_battery_status_t bat_msg = {};
			bat_msg.id = 0;
			bat_msg.battery_function = MAV_BATTERY_FUNCTION_ALL;
			bat_msg.type = MAV_BATTERY_TYPE_LIPO;
			bat_msg.current_consumed = (battery_status.connected) ? battery_status.discharged_mah : -1;
			bat_msg.energy_consumed = -1;
			bat_msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100 : -1;
			bat_msg.battery_remaining = (battery_status.connected) ? ceilf(battery_status.remaining * 100.0f) : -1;
			bat_msg.temperature = (battery_status.connected) ? (int16_t)battery_status.temperature : INT16_MAX;
			//bat_msg.average_current_battery = (battery_status.connected) ? battery_status.average_current_a * 100.0f : -1;
			//bat_msg.serial_number = (battery_status.connected) ? battery_status.serial_number : 0;
			//bat_msg.capacity = (battery_status.connected) ? battery_status.capacity : 0;
			//bat_msg.cycle_count = (battery_status.connected) ? battery_status.cycle_count : UINT16_MAX;
			//bat_msg.run_time_to_empty = (battery_status.connected) ? battery_status.run_time_to_empty * 60 : 0;
			//bat_msg.average_time_to_empty = (battery_status.connected) ? battery_status.average_time_to_empty * 60 : 0;

			for (unsigned int i = 0; i < (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0])); i++) {
				if ((int)i < battery_status.cell_count && battery_status.connected) {
					bat_msg.voltages[i] = (battery_status.voltage_v / battery_status.cell_count) * 1000.0f;

				} else {
					bat_msg.voltages[i] = UINT16_MAX;
				}
			}

			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighresIMU(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_sensor_sub;
	uint64_t _sensor_time;

	MavlinkOrbSubscription *_bias_sub;
	MavlinkOrbSubscription *_differential_pressure_sub;
	MavlinkOrbSubscription *_magnetometer_sub;
	MavlinkOrbSubscription *_air_data_sub;

	uint64_t _accel_timestamp;
	uint64_t _gyro_timestamp;
	uint64_t _mag_timestamp;
	uint64_t _baro_timestamp;
	uint64_t _dpres_timestamp;

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &) = delete;
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &) = delete;

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0),
		_bias_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_bias))),
		_differential_pressure_sub(_mavlink->add_orb_subscription(ORB_ID(differential_pressure))),
		_magnetometer_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_magnetometer))),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data))),
		_accel_timestamp(0),
		_gyro_timestamp(0),
		_mag_timestamp(0),
		_baro_timestamp(0),
		_dpres_timestamp(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_combined_s sensor;

		if (_sensor_sub->update(&_sensor_time, &sensor)) {
			uint16_t fields_updated = 0;

			if (_accel_timestamp != sensor.timestamp + sensor.accelerometer_timestamp_relative) {
				/* mark first three dimensions as changed */
				fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
				_accel_timestamp = sensor.timestamp + sensor.accelerometer_timestamp_relative;
			}

			if (_gyro_timestamp != sensor.timestamp) {
				/* mark second group dimensions as changed */
				fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
				_gyro_timestamp = sensor.timestamp;
			}

			vehicle_magnetometer_s magnetometer = {};
			_magnetometer_sub->update(&magnetometer);

			if (_mag_timestamp != magnetometer.timestamp) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
				_mag_timestamp = magnetometer.timestamp;
			}

			vehicle_air_data_s air_data = {};
			_air_data_sub->update(&air_data);

			if (_baro_timestamp != air_data.timestamp) {
				/* mark fourth group (baro fields) dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
				_baro_timestamp = air_data.timestamp;
			}

			sensor_bias_s bias = {};
			_bias_sub->update(&bias);

			differential_pressure_s differential_pressure = {};
			_differential_pressure_sub->update(&differential_pressure);

			if (_dpres_timestamp != differential_pressure.timestamp) {
				/* mark fourth group (dpres field) dimensions as changed */
				fields_updated |= (1 << 10);
				_dpres_timestamp = differential_pressure.timestamp;
			}

			mavlink_highres_imu_t msg = {};

			msg.time_usec = sensor.timestamp;
			msg.xacc = sensor.accelerometer_m_s2[0] - bias.accel_x_bias;
			msg.yacc = sensor.accelerometer_m_s2[1] - bias.accel_y_bias;
			msg.zacc = sensor.accelerometer_m_s2[2] - bias.accel_z_bias;
			msg.xgyro = sensor.gyro_rad[0] - bias.gyro_x_bias;
			msg.ygyro = sensor.gyro_rad[1] - bias.gyro_y_bias;
			msg.zgyro = sensor.gyro_rad[2] - bias.gyro_z_bias;
			msg.xmag = magnetometer.magnetometer_ga[0] - bias.mag_x_bias;
			msg.ymag = magnetometer.magnetometer_ga[1] - bias.mag_y_bias;
			msg.zmag = magnetometer.magnetometer_ga[2] - bias.mag_z_bias;
			msg.abs_pressure = air_data.baro_pressure_pa;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = air_data.baro_alt_meter;
			msg.temperature = air_data.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamScaledIMU : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU(mavlink);
	}

	unsigned get_size()
	{
		return _raw_accel_sub->is_published() ? (MAVLINK_MSG_ID_SCALED_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_raw_accel_sub;
	MavlinkOrbSubscription *_raw_gyro_sub;
	MavlinkOrbSubscription *_raw_mag_sub;

	uint64_t _raw_accel_time;
	uint64_t _raw_gyro_time;
	uint64_t _raw_mag_time;

	// do not allow top copy this class
	MavlinkStreamScaledIMU(MavlinkStreamScaledIMU &) = delete;
	MavlinkStreamScaledIMU &operator = (const MavlinkStreamScaledIMU &) = delete;

protected:
	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_accel_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_accel), 0)),
		_raw_gyro_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_gyro), 0)),
		_raw_mag_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_mag), 0)),
		_raw_accel_time(0),
		_raw_gyro_time(0),
		_raw_mag_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_accel_s sensor_accel = {};
		sensor_gyro_s sensor_gyro = {};
		sensor_mag_s sensor_mag = {};

		bool updated = false;
		updated |= _raw_accel_sub->update(&_raw_accel_time, &sensor_accel);
		updated |= _raw_gyro_sub->update(&_raw_gyro_time, &sensor_gyro);
		updated |= _raw_mag_sub->update(&_raw_mag_time, &sensor_mag);

		if (updated) {

			mavlink_scaled_imu_t msg = {};

			msg.time_boot_ms = sensor_accel.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_accel.x_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.yacc = (int16_t)(sensor_accel.y_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.zacc = (int16_t)(sensor_accel.z_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.xgyro = sensor_gyro.x_raw;					// [milli rad/s]
			msg.ygyro = sensor_gyro.y_raw;					// [milli rad/s]
			msg.zgyro = sensor_gyro.z_raw;					// [milli rad/s]
			msg.xmag = sensor_mag.x_raw;					// [milli tesla]
			msg.ymag = sensor_mag.y_raw;					// [milli tesla]
			msg.zmag = sensor_mag.z_raw;					// [milli tesla]

			mavlink_msg_scaled_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamScaledIMU2 : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU2::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU2";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU2;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU2(mavlink);
	}

	unsigned get_size()
	{
		return _raw_accel_sub->is_published() ? (MAVLINK_MSG_ID_SCALED_IMU2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_raw_accel_sub;
	MavlinkOrbSubscription *_raw_gyro_sub;
	MavlinkOrbSubscription *_raw_mag_sub;

	uint64_t _raw_accel_time;
	uint64_t _raw_gyro_time;
	uint64_t _raw_mag_time;

	// do not allow top copy this class
	MavlinkStreamScaledIMU2(MavlinkStreamScaledIMU2 &) = delete;
	MavlinkStreamScaledIMU2 &operator = (const MavlinkStreamScaledIMU2 &) = delete;

protected:
	explicit MavlinkStreamScaledIMU2(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_accel_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_accel), 1)),
		_raw_gyro_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_gyro), 1)),
		_raw_mag_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_mag), 1)),
		_raw_accel_time(0),
		_raw_gyro_time(0),
		_raw_mag_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_accel_s sensor_accel = {};
		sensor_gyro_s sensor_gyro = {};
		sensor_mag_s sensor_mag = {};

		bool updated = false;
		updated |= _raw_accel_sub->update(&_raw_accel_time, &sensor_accel);
		updated |= _raw_gyro_sub->update(&_raw_gyro_time, &sensor_gyro);
		updated |= _raw_mag_sub->update(&_raw_mag_time, &sensor_mag);

		if (updated) {

			mavlink_scaled_imu2_t msg = {};

			msg.time_boot_ms = sensor_accel.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_accel.x_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.yacc = (int16_t)(sensor_accel.y_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.zacc = (int16_t)(sensor_accel.z_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.xgyro = sensor_gyro.x_raw;					// [milli rad/s]
			msg.ygyro = sensor_gyro.y_raw;					// [milli rad/s]
			msg.zgyro = sensor_gyro.z_raw;					// [milli rad/s]
			msg.xmag = sensor_mag.x_raw;					// [milli tesla]
			msg.ymag = sensor_mag.y_raw;					// [milli tesla]
			msg.zmag = sensor_mag.z_raw;					// [milli tesla]

			mavlink_msg_scaled_imu2_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamScaledIMU3 : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamScaledIMU3::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SCALED_IMU3";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SCALED_IMU3;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamScaledIMU3(mavlink);
	}

	unsigned get_size()
	{
		return _raw_accel_sub->is_published() ? (MAVLINK_MSG_ID_SCALED_IMU3_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_raw_accel_sub;
	MavlinkOrbSubscription *_raw_gyro_sub;
	MavlinkOrbSubscription *_raw_mag_sub;

	uint64_t _raw_accel_time;
	uint64_t _raw_gyro_time;
	uint64_t _raw_mag_time;

	// do not allow top copy this class
	MavlinkStreamScaledIMU3(MavlinkStreamScaledIMU3 &) = delete;
	MavlinkStreamScaledIMU3 &operator = (const MavlinkStreamScaledIMU3 &) = delete;

protected:
	explicit MavlinkStreamScaledIMU3(Mavlink *mavlink) : MavlinkStream(mavlink),
		_raw_accel_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_accel), 2)),
		_raw_gyro_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_gyro), 2)),
		_raw_mag_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_mag), 2)),
		_raw_accel_time(0),
		_raw_gyro_time(0),
		_raw_mag_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		sensor_accel_s sensor_accel = {};
		sensor_gyro_s sensor_gyro = {};
		sensor_mag_s sensor_mag = {};

		bool updated = false;
		updated |= _raw_accel_sub->update(&_raw_accel_time, &sensor_accel);
		updated |= _raw_gyro_sub->update(&_raw_gyro_time, &sensor_gyro);
		updated |= _raw_mag_sub->update(&_raw_mag_time, &sensor_mag);

		if (updated) {

			mavlink_scaled_imu3_t msg = {};

			msg.time_boot_ms = sensor_accel.timestamp / 1000;
			msg.xacc = (int16_t)(sensor_accel.x_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.yacc = (int16_t)(sensor_accel.y_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.zacc = (int16_t)(sensor_accel.z_raw / CONSTANTS_ONE_G); 	// [milli g]
			msg.xgyro = sensor_gyro.x_raw;					// [milli rad/s]
			msg.ygyro = sensor_gyro.y_raw;					// [milli rad/s]
			msg.zgyro = sensor_gyro.z_raw;					// [milli rad/s]
			msg.xmag = sensor_mag.x_raw;					// [milli tesla]
			msg.ymag = sensor_mag.y_raw;					// [milli tesla]
			msg.zmag = sensor_mag.z_raw;					// [milli tesla]

			mavlink_msg_scaled_imu3_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitude : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitude::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitude(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_att_sub;
	uint64_t _att_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitude(MavlinkStreamAttitude &) = delete;
	MavlinkStreamAttitude &operator = (const MavlinkStreamAttitude &) = delete;


protected:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_s att;

		if (_att_sub->update(&_att_time, &att)) {
			mavlink_attitude_t msg = {};
			matrix::Eulerf euler = matrix::Quatf(att.q);
			msg.time_boot_ms = att.timestamp / 1000;
			msg.roll = euler.phi();
			msg.pitch = euler.theta();
			msg.yaw = euler.psi();
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeQuaternion::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_QUATERNION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeQuaternion(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_att_sub;
	uint64_t _att_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitudeQuaternion(MavlinkStreamAttitudeQuaternion &) = delete;
	MavlinkStreamAttitudeQuaternion &operator = (const MavlinkStreamAttitudeQuaternion &) = delete;

protected:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_s att;

		if (_att_sub->update(&_att_time, &att)) {
			mavlink_attitude_quaternion_t msg = {};

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = att.q[0];
			msg.q2 = att.q[1];
			msg.q3 = att.q[2];
			msg.q4 = att.q[3];
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamVFRHUD : public MavlinkStream
{
public:

	const char *get_name() const
	{
		return MavlinkStreamVFRHUD::get_name_static();
	}

	static const char *get_name_static()
	{
		return "VFR_HUD";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VFR_HUD;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVFRHUD(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_VFR_HUD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	MavlinkOrbSubscription *_armed_sub;
	uint64_t _armed_time;

	MavlinkOrbSubscription *_act0_sub;
	MavlinkOrbSubscription *_act1_sub;

	MavlinkOrbSubscription *_airspeed_sub;
	uint64_t _airspeed_time;

	MavlinkOrbSubscription *_air_data_sub;

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &) = delete;
	MavlinkStreamVFRHUD &operator = (const MavlinkStreamVFRHUD &) = delete;

protected:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_pos_time(0),
		_armed_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_armed))),
		_armed_time(0),
		_act0_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_0))),
		_act1_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_1))),
		_airspeed_sub(_mavlink->add_orb_subscription(ORB_ID(airspeed))),
		_airspeed_time(0),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_local_position_s pos = {};
		actuator_armed_s armed = {};
		airspeed_s airspeed = {};

		bool updated = false;
		updated |= _pos_sub->update(&_pos_time, &pos);
		updated |= _armed_sub->update(&_armed_time, &armed);
		updated |= _airspeed_sub->update(&_airspeed_time, &airspeed);

		if (updated) {
			mavlink_vfr_hud_t msg = {};
			msg.airspeed = airspeed.indicated_airspeed_m_s;
			msg.groundspeed = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
			msg.heading = math::degrees(wrap_2pi(pos.yaw));

			if (armed.armed) {
				actuator_controls_s act0 = {};
				actuator_controls_s act1 = {};
				_act0_sub->update(&act0);
				_act1_sub->update(&act1);

				// VFR_HUD throttle should only be used for operator feedback.
				// VTOLs switch between actuator_controls_0 and actuator_controls_1. During transition there isn't a
				// a single throttle value, but this should still be a useful heuristic for operator awareness.
				//
				// Use ACTUATOR_CONTROL_TARGET if accurate states are needed.
				msg.throttle = 100 * math::max(
						       act0.control[actuator_controls_s::INDEX_THROTTLE],
						       act1.control[actuator_controls_s::INDEX_THROTTLE]);

			} else {
				msg.throttle = 0.0f;
			}

			if (pos.z_valid && pos.z_global) {
				/* use local position estimate */
				msg.alt = -pos.z + pos.ref_alt;

			} else {
				vehicle_air_data_s air_data = {};
				_air_data_sub->update(&air_data);

				/* fall back to baro altitude */
				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter;
				}
			}

			if (pos.v_z_valid) {
				msg.climb = -pos.vz;
			}

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamGPSRawInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSRawInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_RAW_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSRawInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_gps_sub;
	uint64_t _gps_time;

	/* do not allow top copying this class */
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &) = delete;
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &) = delete;

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
		_gps_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps_raw_int_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.alt_ellipsoid = gps.alt_ellipsoid;
			msg.eph = gps.hdop * 100;
			msg.epv = gps.vdop * 100;
			msg.h_acc = gps.eph * 1e3f;
			msg.v_acc = gps.epv * 1e3f;
			msg.vel_acc = gps.s_variance_m_s * 1e3f;
			msg.hdg_acc = gps.c_variance_rad * 1e5f / M_DEG_TO_RAD_F;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGPS2Raw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPS2Raw::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS2_RAW";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GPS2_RAW;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPS2Raw(mavlink);
	}

	unsigned get_size()
	{
		return (_gps_time > 0) ? (MAVLINK_MSG_ID_GPS2_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_gps_sub;
	uint64_t _gps_time;

	/* do not allow top copying this class */
	MavlinkStreamGPS2Raw(MavlinkStreamGPS2Raw &) = delete;
	MavlinkStreamGPS2Raw &operator = (const MavlinkStreamGPS2Raw &) = delete;

protected:
	explicit MavlinkStreamGPS2Raw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position), 1)),
		_gps_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps2_raw_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.eph * 1e3f;
			msg.epv = gps.epv * 1e3f;
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s);
			msg.cog = math::degrees(wrap_2pi(gps.cog_rad)) * 1e2f;
			msg.satellites_visible = gps.satellites_used;
			//msg.dgps_numch = // Number of DGPS satellites
			//msg.dgps_age = // Age of DGPS info

			mavlink_msg_gps2_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamSystemTime : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamSystemTime::get_name_static();
	}

	static const char *get_name_static()
	{
		return "SYSTEM_TIME";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSystemTime(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &) = delete;
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &) = delete;

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_system_time_t msg = {};
		timespec tv;

		px4_clock_gettime(CLOCK_REALTIME, &tv);

		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		// and we don't need to bother sending it because it's definitely wrong.
		if (msg.time_unix_usec > 978307200000000) {
			mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

class MavlinkStreamTimesync : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamTimesync::get_name_static();
	}

	static const char *get_name_static()
	{
		return "TIMESYNC";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TIMESYNC;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTimesync(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_TIMESYNC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamTimesync(MavlinkStreamTimesync &) = delete;
	MavlinkStreamTimesync &operator = (const MavlinkStreamTimesync &) = delete;

protected:
	explicit MavlinkStreamTimesync(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_timesync_t msg = {};

		msg.tc1 = 0;
		msg.ts1 = hrt_absolute_time() * 1000; // boot time in nanoseconds

		mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamADSBVehicle : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamADSBVehicle::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ADSB_VEHICLE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ADSB_VEHICLE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamADSBVehicle(mavlink);
	}

	bool const_rate()
	{
		return true;
	}

	unsigned get_size()
	{
		return (_pos_time > 0) ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	/* do not allow top copying this class */
	MavlinkStreamADSBVehicle(MavlinkStreamADSBVehicle &) = delete;
	MavlinkStreamADSBVehicle &operator = (const MavlinkStreamADSBVehicle &) = delete;

protected:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(transponder_report))),
		_pos_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct transponder_report_s pos;
		bool sent = false;

		while (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_adsb_vehicle_t msg = {};

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) { continue; }

			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamUTMGlobalPosition : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamUTMGlobalPosition::get_name_static();
	}

	static const char *get_name_static()
	{
		return "UTM_GLOBAL_POSITION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_UTM_GLOBAL_POSITION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamUTMGlobalPosition(mavlink);
	}

	bool const_rate()
	{
		return true;
	}

	unsigned get_size()
	{
		return _local_pos_time > 0 ? MAVLINK_MSG_ID_UTM_GLOBAL_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_local_pos_sub;
	uint64_t _local_pos_time = 0;
	vehicle_local_position_s _local_position   = {};

	MavlinkOrbSubscription *_global_pos_sub;
	uint64_t _global_pos_time = 0;
	vehicle_global_position_s _global_position = {};

	MavlinkOrbSubscription *_position_setpoint_triplet_sub;
	uint64_t _setpoint_triplet_time = 0;
	position_setpoint_triplet_s _setpoint_triplet = {};

	MavlinkOrbSubscription *_vehicle_status_sub;
	uint64_t _vehicle_status_time = 0;
	vehicle_status_s _vehicle_status = {};

	MavlinkOrbSubscription *_land_detected_sub;
	uint64_t _land_detected_time = 0;
	vehicle_land_detected_s _land_detected = {};

	/* do not allow top copying this class */
	MavlinkStreamUTMGlobalPosition(MavlinkStreamUTMGlobalPosition &) = delete;
	MavlinkStreamUTMGlobalPosition &operator = (const MavlinkStreamUTMGlobalPosition &) = delete;

protected:
	explicit MavlinkStreamUTMGlobalPosition(Mavlink *mavlink) : MavlinkStream(mavlink),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_position_setpoint_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet))),
		_vehicle_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_land_detected_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_land_detected)))
	{}

	bool send(const hrt_abstime t)
	{

		// Check if new uORB messages are available otherwise use the last received
		_local_pos_sub->update(&_local_pos_time, &_local_position);
		_global_pos_sub->update(&_global_pos_time, &_global_position);
		_position_setpoint_triplet_sub->update(&_setpoint_triplet_time, &_setpoint_triplet);
		_vehicle_status_sub->update(&_vehicle_status_time, &_vehicle_status);
		_land_detected_sub->update(&_land_detected_time, &_land_detected);

		mavlink_utm_global_position_t msg = {};

		// Compute Unix epoch and set time field
		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);
		uint64_t unix_epoch = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		if (unix_epoch > 978307200000000) {
			msg.time = unix_epoch;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_TIME_VALID;
		}

#ifndef BOARD_HAS_NO_UUID
		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);
		static_assert(sizeof(px4_guid_t) == sizeof(msg.uas_id), "GUID byte length mismatch");
		memcpy(&msg.uas_id, &px4_guid, sizeof(msg.uas_id));
		msg.flags |= UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE;
#else
		// TODO Fill ID with something reasonable
		memset(&msg.uas_id[0], 0, sizeof(msg.uas_id));
#endif /* BOARD_HAS_NO_UUID */

		// Handle global position
		if (_global_pos_time > 0) {
			msg.lat = _global_position.lat * 1e7;
			msg.lon = _global_position.lon * 1e7;
			msg.alt = _global_position.alt_ellipsoid * 1000.0f;

			msg.h_acc = _global_position.eph * 1000.0f;
			msg.v_acc = _global_position.epv * 1000.0f;

			msg.flags |= UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE;
		}

		// Handle local position
		if (_local_pos_time > 0) {
			float evh = 0.0f;
			float evv = 0.0f;

			if (_local_position.v_xy_valid) {
				msg.vx = _local_position.vx * 100.0f;
				msg.vy = _local_position.vy * 100.0f;
				evh = _local_position.evh;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE;
			}

			if (_local_position.v_z_valid) {
				msg.vz = _local_position.vz * 100.0f;
				evv = _local_position.evv;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE;
			}

			msg.vel_acc = sqrtf(evh * evh + evv * evv) * 100.0f;

			if (_local_position.dist_bottom_valid) {
				msg.relative_alt = _local_position.dist_bottom * 1000.0f;
				msg.flags |= UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE;
			}
		}

		bool vehicle_in_auto_mode = _vehicle_status_time > 0
					    && (_vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
						|| _vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER);

		// Handle next waypoint if it is valid
		if (vehicle_in_auto_mode && _setpoint_triplet_time > 0 && _setpoint_triplet.current.valid) {
			msg.next_lat = _setpoint_triplet.current.lat * 1e7;
			msg.next_lon = _setpoint_triplet.current.lon * 1e7;
			// HACK We assume that the offset between AMSL and WGS84 is constant between the current
			// vehicle position and the the target waypoint.
			msg.next_alt = (_setpoint_triplet.current.alt + (_global_position.alt_ellipsoid - _global_position.alt)) * 1000.0f;
			msg.flags |= UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE;
		}

		// Handle flight state
		if (_vehicle_status_time > 0 && _land_detected_time > 0
		    && _vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
			if (_land_detected.landed) {
				msg.flight_state |= UTM_FLIGHT_STATE_GROUND;

			} else {
				msg.flight_state |= UTM_FLIGHT_STATE_AIRBORNE;
			}

		} else {
			msg.flight_state |= UTM_FLIGHT_STATE_UNKNOWN;
		}

		msg.update_rate = 0; // Data driven mode

		mavlink_msg_utm_global_position_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamCollision : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCollision::get_name_static();
	}

	static const char *get_name_static()
	{
		return "COLLISION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COLLISION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCollision(mavlink);
	}

	unsigned get_size()
	{
		return (_collision_time > 0) ? MAVLINK_MSG_ID_COLLISION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_collision_sub;
	uint64_t _collision_time;

	/* do not allow top copying this class */
	MavlinkStreamCollision(MavlinkStreamCollision &) = delete;
	MavlinkStreamCollision &operator = (const MavlinkStreamCollision &) = delete;

protected:
	explicit MavlinkStreamCollision(Mavlink *mavlink) : MavlinkStream(mavlink),
		_collision_sub(_mavlink->add_orb_subscription(ORB_ID(collision_report))),
		_collision_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct collision_report_s report;
		bool sent = false;

		while (_collision_sub->update(&_collision_time, &report)) {
			mavlink_collision_t msg = {};

			msg.src = report.src;
			msg.id = report.id;
			msg.action = report.action;
			msg.threat_level = report.threat_level;
			msg.time_to_minimum_delta = report.time_to_minimum_delta;
			msg.altitude_minimum_delta = report.altitude_minimum_delta;
			msg.horizontal_minimum_delta = report.horizontal_minimum_delta;

			mavlink_msg_collision_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

class MavlinkStreamCameraTrigger : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraTrigger::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_TRIGGER";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_TRIGGER;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraTrigger(mavlink);
	}

	bool const_rate()
	{
		return true;
	}

	unsigned get_size()
	{
		return (_trigger_time > 0) ? MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_trigger_sub;
	uint64_t _trigger_time;

	/* do not allow top copying this class */
	MavlinkStreamCameraTrigger(MavlinkStreamCameraTrigger &) = delete;
	MavlinkStreamCameraTrigger &operator = (const MavlinkStreamCameraTrigger &) = delete;

protected:
	explicit MavlinkStreamCameraTrigger(Mavlink *mavlink) : MavlinkStream(mavlink),
		_trigger_sub(_mavlink->add_orb_subscription(ORB_ID(camera_trigger))),
		_trigger_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct camera_trigger_s trigger;

		if (_trigger_sub->update(&_trigger_time, &trigger)) {
			mavlink_camera_trigger_t msg = {};

			msg.time_usec = trigger.timestamp;
			msg.seq = trigger.seq;

			/* ensure that only active trigger events are sent */
			if (trigger.timestamp > 0) {

				mavlink_msg_camera_trigger_send_struct(_mavlink->get_channel(), &msg);

				vehicle_command_s vcmd = {};
				vcmd.timestamp = hrt_absolute_time();
				vcmd.param1 = 0.0f; // all cameras
				vcmd.param2 = 0.0f; // duration 0 because only taking one picture
				vcmd.param3 = 1.0f; // only take one
				vcmd.param4 = NAN;
				vcmd.param5 = (double)NAN;
				vcmd.param6 = (double)NAN;
				vcmd.param7 = NAN;
				vcmd.command = MAV_CMD_IMAGE_START_CAPTURE;
				vcmd.target_system = mavlink_system.sysid;
				vcmd.target_component = MAV_COMP_ID_CAMERA;

				MavlinkCommandSender::instance().handle_vehicle_command(vcmd, _mavlink->get_channel());

				// TODO: move this camera_trigger and publish as a vehicle_command
				/* send MAV_CMD_DO_DIGICAM_CONTROL*/
				mavlink_command_long_t digicam_ctrl_cmd = {};

				digicam_ctrl_cmd.target_system = 0; // 0 for broadcast
				digicam_ctrl_cmd.target_component = MAV_COMP_ID_CAMERA;
				digicam_ctrl_cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
				digicam_ctrl_cmd.confirmation = 0;
				digicam_ctrl_cmd.param1 = NAN;
				digicam_ctrl_cmd.param2 = NAN;
				digicam_ctrl_cmd.param3 = NAN;
				digicam_ctrl_cmd.param4 = NAN;
				digicam_ctrl_cmd.param5 = 1;   // take 1 picture
				digicam_ctrl_cmd.param6 = NAN;
				digicam_ctrl_cmd.param7 = NAN;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &digicam_ctrl_cmd);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamCameraImageCaptured : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraImageCaptured::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_IMAGE_CAPTURED";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	bool const_rate()
	{
		return true;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraImageCaptured(mavlink);
	}

	unsigned get_size()
	{
		return (_capture_time > 0) ? MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_capture_sub;
	uint64_t _capture_time;

	/* do not allow top copying this class */
	MavlinkStreamCameraImageCaptured(MavlinkStreamCameraImageCaptured &) = delete;
	MavlinkStreamCameraImageCaptured &operator = (const MavlinkStreamCameraImageCaptured &) = delete;

protected:
	explicit MavlinkStreamCameraImageCaptured(Mavlink *mavlink) : MavlinkStream(mavlink),
		_capture_sub(_mavlink->add_orb_subscription(ORB_ID(camera_capture))),
		_capture_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct camera_capture_s capture;

		if (_capture_sub->update(&_capture_time, &capture)) {

			mavlink_camera_image_captured_t msg;

			msg.time_boot_ms = capture.timestamp / 1000;
			msg.time_utc = capture.timestamp_utc;
			msg.camera_id = 1;	// FIXME : get this from uORB
			msg.lat = capture.lat * 1e7;
			msg.lon = capture.lon * 1e7;
			msg.alt = capture.alt * 1e3f;
			msg.relative_alt = capture.ground_distance * 1e3f;
			msg.q[0] = capture.q[0];
			msg.q[1] = capture.q[1];
			msg.q[2] = capture.q[2];
			msg.q[3] = capture.q[3];
			msg.image_index = capture.seq;
			msg.capture_result = capture.result;
			msg.file_url[0] = '\0';

			mavlink_msg_camera_image_captured_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGlobalPositionInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGlobalPositionInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GLOBAL_POSITION_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGlobalPositionInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_gpos_sub;
	uint64_t _gpos_time;

	MavlinkOrbSubscription *_lpos_sub;
	uint64_t _lpos_time;

	MavlinkOrbSubscription *_home_sub;
	MavlinkOrbSubscription *_air_data_sub;

	/* do not allow top copying this class */
	MavlinkStreamGlobalPositionInt(MavlinkStreamGlobalPositionInt &) = delete;
	MavlinkStreamGlobalPositionInt &operator = (const MavlinkStreamGlobalPositionInt &) = delete;

protected:
	explicit MavlinkStreamGlobalPositionInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_gpos_time(0),
		_lpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_lpos_time(0),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_global_position_s gpos = {};
		vehicle_local_position_s lpos = {};

		bool gpos_updated = _gpos_sub->update(&_gpos_time, &gpos);
		bool lpos_updated = _lpos_sub->update(&_lpos_time, &lpos);

		if (gpos_updated && lpos_updated) {
			mavlink_global_position_int_t msg = {};

			if (lpos.z_valid && lpos.z_global) {
				msg.alt = (-lpos.z + lpos.ref_alt) * 1000.0f;

			} else {
				// fall back to baro altitude
				vehicle_air_data_s air_data = {};
				_air_data_sub->update(&air_data);

				if (air_data.timestamp > 0) {
					msg.alt = air_data.baro_alt_meter * 1000.0f;
				}
			}

			home_position_s home = {};
			_home_sub->update(&home);

			if ((home.timestamp > 0) && home.valid_alt) {
				if (lpos.z_valid) {
					msg.relative_alt = -(lpos.z - home.z) * 1000.0f;

				} else {
					msg.relative_alt = msg.alt - (home.alt * 1000.0f);
				}

			} else {
				if (lpos.z_valid) {
					msg.relative_alt = -lpos.z * 1000.0f;
				}
			}

			msg.time_boot_ms = gpos.timestamp / 1000;
			msg.lat = gpos.lat * 1e7;
			msg.lon = gpos.lon * 1e7;

			msg.vx = lpos.vx * 100.0f;
			msg.vy = lpos.vy * 100.0f;
			msg.vz = lpos.vz * 100.0f;

			msg.hdg = math::degrees(wrap_2pi(lpos.yaw)) * 100.0f;

			mavlink_msg_global_position_int_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

//TODO: remove this -> add ODOMETRY loopback only
class MavlinkStreamVisionPositionEstimate : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamVisionPositionEstimate::get_name_static();
	}

	static const char *get_name_static()
	{
		return "VISION_POSITION_ESTIMATE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVisionPositionEstimate(mavlink);
	}

	unsigned get_size()
	{
		return (_odom_time > 0) ? MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}
private:

	MavlinkOrbSubscription *_odom_sub;
	uint64_t _odom_time;

	/* do not allow top copying this class */
	MavlinkStreamVisionPositionEstimate(MavlinkStreamVisionPositionEstimate &) = delete;
	MavlinkStreamVisionPositionEstimate &operator = (const MavlinkStreamVisionPositionEstimate &) = delete;

protected:
	explicit MavlinkStreamVisionPositionEstimate(Mavlink *mavlink) : MavlinkStream(mavlink),
		_odom_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_visual_odometry))),
		_odom_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_odometry_s vodom;

		if (_odom_sub->update(&_odom_time, &vodom)) {
			mavlink_vision_position_estimate_t vmsg = {};
			vmsg.usec = vodom.timestamp;
			vmsg.x = vodom.x;
			vmsg.y = vodom.y;
			vmsg.z = vodom.z;

			matrix::Eulerf euler = matrix::Quatf(vodom.q);
			vmsg.roll = euler.phi();
			vmsg.pitch = euler.theta();
			vmsg.yaw = euler.psi();

			mavlink_msg_vision_position_estimate_send_struct(_mavlink->get_channel(), &vmsg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionNED::get_name_static();
	}

	static const char *get_name_static()
	{
		return "LOCAL_POSITION_NED";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionNED(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionNED(MavlinkStreamLocalPositionNED &) = delete;
	MavlinkStreamLocalPositionNED &operator = (const MavlinkStreamLocalPositionNED &) = delete;

protected:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_pos_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_local_position_s pos;

		if (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_local_position_ned_t msg = {};

			msg.time_boot_ms = pos.timestamp / 1000;
			msg.x = pos.x;
			msg.y = pos.y;
			msg.z = pos.z;
			msg.vx = pos.vx;
			msg.vy = pos.vy;
			msg.vz = pos.vz;

			mavlink_msg_local_position_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamLocalPositionNEDCOV : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionNEDCOV::get_name_static();
	}

	static const char *get_name_static()
	{
		return "LOCAL_POSITION_NED_COV";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionNEDCOV(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_est_sub;
	uint64_t _est_time;

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionNEDCOV(MavlinkStreamLocalPositionNEDCOV &) = delete;
	MavlinkStreamLocalPositionNEDCOV &operator = (const MavlinkStreamLocalPositionNEDCOV &) = delete;

protected:
	explicit MavlinkStreamLocalPositionNEDCOV(Mavlink *mavlink) : MavlinkStream(mavlink),
		_est_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
		_est_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct estimator_status_s est = {};

		if (_est_sub->update(&_est_time, &est)) {
			mavlink_local_position_ned_cov_t msg = {};

			msg.time_usec = est.timestamp;
			msg.x = est.states[0];
			msg.y = est.states[1];
			msg.z = est.states[2];
			msg.vx = est.states[3];
			msg.vy = est.states[4];
			msg.vz = est.states[5];
			msg.ax = est.states[6];
			msg.ay = est.states[7];
			msg.az = est.states[8];

			for (int i = 0; i < 9; i++) {
				msg.covariance[i] = est.covariances[i];
			}

			msg.covariance[10] = est.health_flags;
			msg.covariance[11] = est.timeout_flags;

			mavlink_msg_local_position_ned_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamEstimatorStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamEstimatorStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ESTIMATOR_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VIBRATION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamEstimatorStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_VIBRATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_est_sub;
	uint64_t _est_time;

	/* do not allow top copying this class */
	MavlinkStreamEstimatorStatus(MavlinkStreamEstimatorStatus &) = delete;
	MavlinkStreamEstimatorStatus &operator = (const MavlinkStreamEstimatorStatus &) = delete;

protected:
	explicit MavlinkStreamEstimatorStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_est_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
		_est_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		estimator_status_s est;

		if (_est_sub->update(&_est_time, &est)) {
			// ESTIMATOR_STATUS
			mavlink_estimator_status_t est_msg = {};
			est_msg.time_usec = est.timestamp;
			est_msg.vel_ratio = est.vel_test_ratio;
			est_msg.pos_horiz_ratio = est.pos_test_ratio;
			est_msg.pos_vert_ratio = est.hgt_test_ratio;
			est_msg.mag_ratio = est.mag_test_ratio;
			est_msg.hagl_ratio = est.hagl_test_ratio;
			est_msg.tas_ratio = est.tas_test_ratio;
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;
			est_msg.flags = est.solution_status_flags;
			mavlink_msg_estimator_status_send_struct(_mavlink->get_channel(), &est_msg);

			// VIBRATION
			mavlink_vibration_t msg = {};
			msg.time_usec = est.timestamp;
			msg.vibration_x = est.vibe[0];
			msg.vibration_y = est.vibe[1];
			msg.vibration_z = est.vibe[2];
			mavlink_msg_vibration_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamAttPosMocap : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttPosMocap::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATT_POS_MOCAP";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATT_POS_MOCAP;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttPosMocap(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_mocap_sub;
	uint64_t _mocap_time;

	/* do not allow top copying this class */
	MavlinkStreamAttPosMocap(MavlinkStreamAttPosMocap &) = delete;
	MavlinkStreamAttPosMocap &operator = (const MavlinkStreamAttPosMocap &) = delete;

protected:
	explicit MavlinkStreamAttPosMocap(Mavlink *mavlink) : MavlinkStream(mavlink),
		_mocap_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_mocap_odometry))),
		_mocap_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_odometry_s mocap;

		if (_mocap_sub->update(&_mocap_time, &mocap)) {
			mavlink_att_pos_mocap_t msg = {};

			msg.time_usec = mocap.timestamp;
			msg.q[0] = mocap.q[0];
			msg.q[1] = mocap.q[1];
			msg.q[2] = mocap.q[2];
			msg.q[3] = mocap.q[3];
			msg.x = mocap.x;
			msg.y = mocap.y;
			msg.z = mocap.z;

			mavlink_msg_att_pos_mocap_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamHomePosition : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHomePosition::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HOME_POSITION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HOME_POSITION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHomePosition(mavlink);
	}

	unsigned get_size()
	{
		return _home_sub->is_published() ? (MAVLINK_MSG_ID_HOME_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_home_sub;

	/* do not allow top copying this class */
	MavlinkStreamHomePosition(MavlinkStreamHomePosition &) = delete;
	MavlinkStreamHomePosition &operator = (const MavlinkStreamHomePosition &) = delete;

protected:
	explicit MavlinkStreamHomePosition(Mavlink *mavlink) : MavlinkStream(mavlink),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position)))
	{}

	bool send(const hrt_abstime t)
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		if (_home_sub->is_published()) {
			home_position_s home;

			if (_home_sub->update(&home)) {
				if (home.valid_hpos) {
					mavlink_home_position_t msg;

					msg.latitude = home.lat * 1e7;
					msg.longitude = home.lon * 1e7;
					msg.altitude = home.alt * 1e3f;

					msg.x = home.x;
					msg.y = home.y;
					msg.z = home.z;

					matrix::Quatf q(matrix::Eulerf(0.0f, 0.0f, home.yaw));
					msg.q[0] = q(0);
					msg.q[1] = q(1);
					msg.q[2] = q(2);
					msg.q[3] = q(3);

					msg.approach_x = 0.0f;
					msg.approach_y = 0.0f;
					msg.approach_z = 0.0f;

					mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);

					return true;
				}
			}
		}

		return false;
	}
};


template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamServoOutputRaw<N>::get_name_static();
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";

		case 2:
			return "SERVO_OUTPUT_RAW_2";

		case 3:
			return "SERVO_OUTPUT_RAW_3";
		}
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamServoOutputRaw<N>(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamServoOutputRaw(MavlinkStreamServoOutputRaw &) = delete;
	MavlinkStreamServoOutputRaw &operator = (const MavlinkStreamServoOutputRaw &) = delete;

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs), N)),
		_act_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			mavlink_servo_output_raw_t msg = {};

			static_assert(sizeof(act.output) / sizeof(act.output[0]) >= 16, "mavlink message requires at least 16 outputs");

			msg.time_usec = act.timestamp;
			msg.port = N;
			msg.servo1_raw = act.output[0];
			msg.servo2_raw = act.output[1];
			msg.servo3_raw = act.output[2];
			msg.servo4_raw = act.output[3];
			msg.servo5_raw = act.output[4];
			msg.servo6_raw = act.output[5];
			msg.servo7_raw = act.output[6];
			msg.servo8_raw = act.output[7];
			msg.servo9_raw = act.output[8];
			msg.servo10_raw = act.output[9];
			msg.servo11_raw = act.output[10];
			msg.servo12_raw = act.output[11];
			msg.servo13_raw = act.output[12];
			msg.servo14_raw = act.output[13];
			msg.servo15_raw = act.output[14];
			msg.servo16_raw = act.output[15];

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

template <int N>
class MavlinkStreamActuatorControlTarget : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamActuatorControlTarget<N>::get_name_static();
	}

	static const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "ACTUATOR_CONTROL_TARGET0";

		case 1:
			return "ACTUATOR_CONTROL_TARGET1";

		case 2:
			return "ACTUATOR_CONTROL_TARGET2";

		case 3:
			return "ACTUATOR_CONTROL_TARGET3";
		}
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamActuatorControlTarget<N>(mavlink);
	}

	unsigned get_size()
	{
		return _att_ctrl_sub->is_published() ? (MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_att_ctrl_sub;
	uint64_t _att_ctrl_time;

	/* do not allow top copying this class */
	MavlinkStreamActuatorControlTarget(MavlinkStreamActuatorControlTarget &) = delete;
	MavlinkStreamActuatorControlTarget &operator = (const MavlinkStreamActuatorControlTarget &) = delete;

protected:
	explicit MavlinkStreamActuatorControlTarget(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_ctrl_sub(nullptr),
		_att_ctrl_time(0)
	{
		// XXX this can be removed once the multiplatform system remaps topics
		switch (N) {
		case 0:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_0));
			break;

		case 1:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_1));
			break;

		case 2:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_2));
			break;

		case 3:
			_att_ctrl_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_controls_3));
			break;
		}
	}

	bool send(const hrt_abstime t)
	{
		actuator_controls_s att_ctrl;

		if (_att_ctrl_sub->update(&_att_ctrl_time, &att_ctrl)) {
			mavlink_actuator_control_target_t msg = {};

			msg.time_usec = att_ctrl.timestamp;
			msg.group_mlx = N;

			for (unsigned i = 0; i < sizeof(msg.controls) / sizeof(msg.controls[0]); i++) {
				msg.controls[i] = att_ctrl.control[i];
			}

			mavlink_msg_actuator_control_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamHILActuatorControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHILActuatorControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIL_ACTUATOR_CONTROLS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHILActuatorControls(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamHILActuatorControls(MavlinkStreamHILActuatorControls &) = delete;
	MavlinkStreamHILActuatorControls &operator = (const MavlinkStreamHILActuatorControls &) = delete;

protected:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs))),
		_act_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			vehicle_status_s status = {};
			_status_sub->update(&status);

			if ((status.timestamp > 0) && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
				/* translate the current system state to mavlink state and mode */
				uint8_t mavlink_state;
				uint8_t mavlink_base_mode;
				uint32_t mavlink_custom_mode;
				mavlink_hil_actuator_controls_t msg = {};

				get_mavlink_mode_state(&status, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

				const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

				unsigned system_type = _mavlink->get_system_type();

				/* scale outputs depending on system type */
				if (system_type == MAV_TYPE_QUADROTOR ||
				    system_type == MAV_TYPE_HEXAROTOR ||
				    system_type == MAV_TYPE_OCTOROTOR ||
				    system_type == MAV_TYPE_VTOL_DUOROTOR ||
				    system_type == MAV_TYPE_VTOL_QUADROTOR ||
				    system_type == MAV_TYPE_VTOL_RESERVED2) {

					/* multirotors: set number of rotor outputs depending on type */

					unsigned n;

					switch (system_type) {
					case MAV_TYPE_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_HEXAROTOR:
						n = 6;
						break;

					case MAV_TYPE_VTOL_DUOROTOR:
						n = 2;
						break;

					case MAV_TYPE_VTOL_QUADROTOR:
						n = 4;
						break;

					case MAV_TYPE_VTOL_RESERVED2:
						n = 8;
						break;

					default:
						n = 8;
						break;
					}

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i < n) {
								/* scale PWM out 900..2100 us to 0..1 for rotors */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

							} else {
								/* scale PWM out 900..2100 us to -1..1 for other channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
							}

						} else {
							/* send 0 when disarmed and for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}

				} else {
					/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

					for (unsigned i = 0; i < 16; i++) {
						if (act.output[i] > PWM_DEFAULT_MIN / 2) {
							if (i != 3) {
								/* scale PWM out 900..2100 us to -1..1 for normal channels */
								msg.controls[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

							} else {
								/* scale PWM out 900..2100 us to 0..1 for throttle */
								msg.controls[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
							}

						} else {
							/* set 0 for disabled channels */
							msg.controls[i] = 0.0f;
						}
					}
				}

				msg.time_usec = hrt_absolute_time();
				msg.mode = mavlink_base_mode;
				msg.flags = 0;

				mavlink_msg_hil_actuator_controls_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

class MavlinkStreamPositionTargetGlobalInt : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamPositionTargetGlobalInt::get_name_static();
	}

	static const char *get_name_static()
	{
		return "POSITION_TARGET_GLOBAL_INT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPositionTargetGlobalInt(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_control_mode_sub;
	MavlinkOrbSubscription *_lpos_sp_sub;
	MavlinkOrbSubscription *_pos_sp_triplet_sub;

	/* do not allow top copying this class */
	MavlinkStreamPositionTargetGlobalInt(MavlinkStreamPositionTargetGlobalInt &) = delete;
	MavlinkStreamPositionTargetGlobalInt &operator = (const MavlinkStreamPositionTargetGlobalInt &) = delete;

protected:
	explicit MavlinkStreamPositionTargetGlobalInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_control_mode_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_control_mode))),
		_lpos_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint))),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_control_mode_s control_mode = {};
		_control_mode_sub->update(&control_mode);

		if (control_mode.flag_control_position_enabled) {

			position_setpoint_triplet_s pos_sp_triplet;
			_pos_sp_triplet_sub->update(&pos_sp_triplet);

			if (pos_sp_triplet.timestamp > 0 && pos_sp_triplet.current.valid
			    && PX4_ISFINITE(pos_sp_triplet.current.lat) && PX4_ISFINITE(pos_sp_triplet.current.lon)) {

				mavlink_position_target_global_int_t msg = {};

				msg.time_boot_ms = hrt_absolute_time() / 1000;
				msg.coordinate_frame = MAV_FRAME_GLOBAL_INT;
				msg.lat_int = pos_sp_triplet.current.lat * 1e7;
				msg.lon_int = pos_sp_triplet.current.lon * 1e7;
				msg.alt = pos_sp_triplet.current.alt;

				vehicle_local_position_setpoint_s lpos_sp;

				if (_lpos_sp_sub->update(&lpos_sp)) {
					// velocity
					msg.vx = lpos_sp.vx;
					msg.vy = lpos_sp.vy;
					msg.vz = lpos_sp.vz;

					// acceleration
					msg.afx = lpos_sp.acc_x;
					msg.afy = lpos_sp.acc_y;
					msg.afz = lpos_sp.acc_z;

					// yaw
					msg.yaw = lpos_sp.yaw;
					msg.yaw_rate = lpos_sp.yawspeed;
				}

				mavlink_msg_position_target_global_int_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};


class MavlinkStreamLocalPositionSetpoint : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamLocalPositionSetpoint::get_name_static();
	}

	static const char *get_name_static()
	{
		return "POSITION_TARGET_LOCAL_NED";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionSetpoint(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sp_sub;
	uint64_t _pos_sp_time;

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionSetpoint(MavlinkStreamLocalPositionSetpoint &) = delete;
	MavlinkStreamLocalPositionSetpoint &operator = (const MavlinkStreamLocalPositionSetpoint &) = delete;

protected:
	explicit MavlinkStreamLocalPositionSetpoint(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint))),
		_pos_sp_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub->update(&_pos_sp_time, &pos_sp)) {
			mavlink_position_target_local_ned_t msg = {};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;
			msg.yaw = pos_sp.yaw;
			msg.yaw_rate = pos_sp.yawspeed;
			msg.vx = pos_sp.vx;
			msg.vy = pos_sp.vy;
			msg.vz = pos_sp.vz;
			msg.afx = pos_sp.acc_x;
			msg.afy = pos_sp.acc_y;
			msg.afz = pos_sp.acc_z;

			mavlink_msg_position_target_local_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamAttitudeTarget : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeTarget::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_TARGET";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeTarget(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_att_sp_sub;
	MavlinkOrbSubscription *_att_rates_sp_sub;

	uint64_t _att_sp_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitudeTarget(MavlinkStreamAttitudeTarget &) = delete;
	MavlinkStreamAttitudeTarget &operator = (const MavlinkStreamAttitudeTarget &) = delete;

protected:
	explicit MavlinkStreamAttitudeTarget(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint))),
		_att_rates_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_rates_setpoint))),
		_att_sp_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_attitude_setpoint_s att_sp;

		if (_att_sp_sub->update(&_att_sp_time, &att_sp)) {

			vehicle_rates_setpoint_s att_rates_sp = {};
			_att_rates_sp_sub->update(&att_rates_sp);

			mavlink_attitude_target_t msg = {};

			msg.time_boot_ms = att_sp.timestamp / 1000;

			if (att_sp.q_d_valid) {
				memcpy(&msg.q[0], &att_sp.q_d[0], sizeof(msg.q));

			} else {
				matrix::Quatf q = matrix::Eulerf(att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body);
				memcpy(&msg.q[0], q.data(), sizeof(msg.q));
			}

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = att_sp.thrust_body[0];

			mavlink_msg_attitude_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};


class MavlinkStreamRCChannels : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRCChannels::get_name_static();
	}

	static const char *get_name_static()
	{
		return "RC_CHANNELS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_RC_CHANNELS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRCChannels(mavlink);
	}

	unsigned get_size()
	{
		return _rc_sub->is_published() ? (MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_rc_sub;
	uint64_t _rc_time;

	/* do not allow top copying this class */
	MavlinkStreamRCChannels(MavlinkStreamRCChannels &) = delete;
	MavlinkStreamRCChannels &operator = (const MavlinkStreamRCChannels &) = delete;

protected:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink),
		_rc_sub(_mavlink->add_orb_subscription(ORB_ID(input_rc))),
		_rc_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		input_rc_s rc;

		if (_rc_sub->update(&_rc_time, &rc)) {

			/* send RC channel data and RSSI */
			mavlink_rc_channels_t msg = {};

			msg.time_boot_ms = rc.timestamp / 1000;
			msg.chancount = rc.channel_count;
			msg.chan1_raw = (rc.channel_count > 0) ? rc.values[0] : UINT16_MAX;
			msg.chan2_raw = (rc.channel_count > 1) ? rc.values[1] : UINT16_MAX;
			msg.chan3_raw = (rc.channel_count > 2) ? rc.values[2] : UINT16_MAX;
			msg.chan4_raw = (rc.channel_count > 3) ? rc.values[3] : UINT16_MAX;
			msg.chan5_raw = (rc.channel_count > 4) ? rc.values[4] : UINT16_MAX;
			msg.chan6_raw = (rc.channel_count > 5) ? rc.values[5] : UINT16_MAX;
			msg.chan7_raw = (rc.channel_count > 6) ? rc.values[6] : UINT16_MAX;
			msg.chan8_raw = (rc.channel_count > 7) ? rc.values[7] : UINT16_MAX;
			msg.chan9_raw = (rc.channel_count > 8) ? rc.values[8] : UINT16_MAX;
			msg.chan10_raw = (rc.channel_count > 9) ? rc.values[9] : UINT16_MAX;
			msg.chan11_raw = (rc.channel_count > 10) ? rc.values[10] : UINT16_MAX;
			msg.chan12_raw = (rc.channel_count > 11) ? rc.values[11] : UINT16_MAX;
			msg.chan13_raw = (rc.channel_count > 12) ? rc.values[12] : UINT16_MAX;
			msg.chan14_raw = (rc.channel_count > 13) ? rc.values[13] : UINT16_MAX;
			msg.chan15_raw = (rc.channel_count > 14) ? rc.values[14] : UINT16_MAX;
			msg.chan16_raw = (rc.channel_count > 15) ? rc.values[15] : UINT16_MAX;
			msg.chan17_raw = (rc.channel_count > 16) ? rc.values[16] : UINT16_MAX;
			msg.chan18_raw = (rc.channel_count > 17) ? rc.values[17] : UINT16_MAX;

			msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;

			mavlink_msg_rc_channels_send_struct(_mavlink->get_channel(), &msg);

			/* send override message - harmless if connected to GCS, allows to connect a board to a Linux system */
			/* http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE */
			mavlink_rc_channels_override_t over;
			over.target_system = mavlink_system.sysid;
			over.target_component = 0;
			over.chan1_raw = msg.chan1_raw;
			over.chan2_raw = msg.chan2_raw;
			over.chan3_raw = msg.chan3_raw;
			over.chan4_raw = msg.chan4_raw;
			over.chan5_raw = msg.chan5_raw;
			over.chan6_raw = msg.chan6_raw;
			over.chan7_raw = msg.chan7_raw;
			over.chan8_raw = msg.chan8_raw;

			mavlink_msg_rc_channels_override_send_struct(_mavlink->get_channel(), &over);

			return true;
		}

		return false;
	}
};


class MavlinkStreamManualControl : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamManualControl::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MANUAL_CONTROL";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MANUAL_CONTROL;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamManualControl(mavlink);
	}

	unsigned get_size()
	{
		return _manual_sub->is_published() ? (MAVLINK_MSG_ID_MANUAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_manual_sub;
	uint64_t _manual_time;

	/* do not allow top copying this class */
	MavlinkStreamManualControl(MavlinkStreamManualControl &) = delete;
	MavlinkStreamManualControl &operator = (const MavlinkStreamManualControl &) = delete;

protected:
	explicit MavlinkStreamManualControl(Mavlink *mavlink) : MavlinkStream(mavlink),
		_manual_sub(_mavlink->add_orb_subscription(ORB_ID(manual_control_setpoint))),
		_manual_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		manual_control_setpoint_s manual;

		if (_manual_sub->update(&_manual_time, &manual)) {
			mavlink_manual_control_t msg = {};

			msg.target = mavlink_system.sysid;
			msg.x = manual.x * 1000;
			msg.y = manual.y * 1000;
			msg.z = manual.z * 1000;
			msg.r = manual.r * 1000;
			unsigned shift = 2;
			msg.buttons = 0;
			msg.buttons |= (manual.mode_switch << (shift * 0));
			msg.buttons |= (manual.return_switch << (shift * 1));
			msg.buttons |= (manual.posctl_switch << (shift * 2));
			msg.buttons |= (manual.loiter_switch << (shift * 3));
			msg.buttons |= (manual.acro_switch << (shift * 4));
			msg.buttons |= (manual.offboard_switch << (shift * 5));

			mavlink_msg_manual_control_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamTrajectoryRepresentationWaypoints: public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamTrajectoryRepresentationWaypoints::get_name_static();
	}

	static const char *get_name_static()
	{
		return "TRAJECTORY_REPRESENTATION_WAYPOINTS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTrajectoryRepresentationWaypoints(mavlink);
	}

	unsigned get_size()
	{
		return _traj_wp_avoidance_sub->is_published() ? (MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS_LEN +
				MAVLINK_NUM_NON_PAYLOAD_BYTES)
		       : 0;
	}

private:
	MavlinkOrbSubscription *_traj_wp_avoidance_sub;
	uint64_t _traj_wp_avoidance_time;

	/* do not allow top copying this class */
	MavlinkStreamTrajectoryRepresentationWaypoints(MavlinkStreamTrajectoryRepresentationWaypoints &);
	MavlinkStreamTrajectoryRepresentationWaypoints &operator = (const MavlinkStreamTrajectoryRepresentationWaypoints &);

protected:
	explicit MavlinkStreamTrajectoryRepresentationWaypoints(Mavlink *mavlink) : MavlinkStream(mavlink),
		_traj_wp_avoidance_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_trajectory_waypoint_desired))),
		_traj_wp_avoidance_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct vehicle_trajectory_waypoint_s traj_wp_avoidance_desired;

		if (_traj_wp_avoidance_sub->update(&_traj_wp_avoidance_time, &traj_wp_avoidance_desired)) {
			mavlink_trajectory_representation_waypoints_t msg = {};

			msg.time_usec = traj_wp_avoidance_desired.timestamp;
			int number_valid_points = 0;

			for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
				msg.pos_x[i] = traj_wp_avoidance_desired.waypoints[i].position[0];
				msg.pos_y[i] = traj_wp_avoidance_desired.waypoints[i].position[1];
				msg.pos_z[i] = traj_wp_avoidance_desired.waypoints[i].position[2];

				msg.vel_x[i] = traj_wp_avoidance_desired.waypoints[i].velocity[0];
				msg.vel_y[i] = traj_wp_avoidance_desired.waypoints[i].velocity[1];
				msg.vel_z[i] = traj_wp_avoidance_desired.waypoints[i].velocity[2];

				msg.acc_x[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[0];
				msg.acc_y[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[1];
				msg.acc_z[i] = traj_wp_avoidance_desired.waypoints[i].acceleration[2];

				msg.pos_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw;
				msg.vel_yaw[i] = traj_wp_avoidance_desired.waypoints[i].yaw_speed;

				if (traj_wp_avoidance_desired.waypoints[i].point_valid) {
					number_valid_points++;
				}

			}

			msg.valid_points = number_valid_points;

			mavlink_msg_trajectory_representation_waypoints_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamOpticalFlowRad : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOpticalFlowRad::get_name_static();
	}

	static const char *get_name_static()
	{
		return "OPTICAL_FLOW_RAD";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOpticalFlowRad(mavlink);
	}

	unsigned get_size()
	{
		return _flow_sub->is_published() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_flow_sub;
	uint64_t _flow_time;

	/* do not allow top copying this class */
	MavlinkStreamOpticalFlowRad(MavlinkStreamOpticalFlowRad &) = delete;
	MavlinkStreamOpticalFlowRad &operator = (const MavlinkStreamOpticalFlowRad &) = delete;

protected:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink),
		_flow_sub(_mavlink->add_orb_subscription(ORB_ID(optical_flow))),
		_flow_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		optical_flow_s flow;

		if (_flow_sub->update(&_flow_time, &flow)) {
			mavlink_optical_flow_rad_t msg = {};

			msg.time_usec = flow.timestamp;
			msg.sensor_id = flow.sensor_id;
			msg.integrated_x = flow.pixel_flow_x_integral;
			msg.integrated_y = flow.pixel_flow_y_integral;
			msg.integrated_xgyro = flow.gyro_x_rate_integral;
			msg.integrated_ygyro = flow.gyro_y_rate_integral;
			msg.integrated_zgyro = flow.gyro_z_rate_integral;
			msg.distance = flow.ground_distance_m;
			msg.quality = flow.quality;
			msg.integration_time_us = flow.integration_timespan;
			msg.sensor_id = flow.sensor_id;
			msg.time_delta_distance_us = flow.time_since_last_sonar_update;
			msg.temperature = flow.gyro_temperature;

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamNamedValueFloat::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NAMED_VALUE_FLOAT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNamedValueFloat(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &) = delete;
	MavlinkStreamNamedValueFloat &operator = (const MavlinkStreamNamedValueFloat &) = delete;

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_key_value))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_key_value_s debug;

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_named_value_float_t msg = {};

			msg.time_boot_ms = debug.timestamp / 1000ULL;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebug : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDebug::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DEBUG";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebug(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_DEBUG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamDebug(MavlinkStreamDebug &) = delete;
	MavlinkStreamDebug &operator = (const MavlinkStreamDebug &) = delete;

protected:
	explicit MavlinkStreamDebug(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_value))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_value_s debug = {};

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_debug_t msg = {};

			msg.time_boot_ms = debug.timestamp / 1000ULL;
			msg.ind = debug.ind;
			msg.value = debug.value;

			mavlink_msg_debug_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebugVect : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDebugVect::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DEBUG_VECT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_VECT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugVect(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_DEBUG_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamDebugVect(MavlinkStreamDebugVect &) = delete;
	MavlinkStreamDebugVect &operator = (const MavlinkStreamDebugVect &) = delete;

protected:
	explicit MavlinkStreamDebugVect(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_vect))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_vect_s debug = {};

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_debug_vect_t msg = {};

			msg.time_usec = debug.timestamp;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.x = debug.x;
			msg.y = debug.y;
			msg.z = debug.z;

			mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDebugFloatArray::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DEBUG_FLOAT_ARRAY";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugFloatArray(mavlink);
	}

	unsigned get_size()
	{
		return (_debug_time > 0) ? MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_debug_array_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamDebugFloatArray(MavlinkStreamDebugFloatArray &);
	MavlinkStreamDebugFloatArray &operator = (const MavlinkStreamDebugFloatArray &);

protected:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_array_sub(_mavlink->add_orb_subscription(ORB_ID(debug_array))),
		_debug_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct debug_array_s debug = {};

		if (_debug_array_sub->update(&_debug_time, &debug)) {
			mavlink_debug_float_array_t msg = {};

			msg.time_usec = debug.timestamp;
			msg.array_id = debug.id;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';

			for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
				msg.data[i] = debug.data[i];
			}

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamNavControllerOutput : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamNavControllerOutput::get_name_static();
	}

	static const char *get_name_static()
	{
		return "NAV_CONTROLLER_OUTPUT";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNavControllerOutput(mavlink);
	}

	unsigned get_size()
	{
		return (_pos_ctrl_status_sub->is_published()) ?
		       MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_pos_ctrl_status_sub;
	MavlinkOrbSubscription *_tecs_status_sub;

	uint64_t _pos_ctrl_status_timestamp{0};
	uint64_t _tecs_status_timestamp{0};

	/* do not allow top copying this class */
	MavlinkStreamNavControllerOutput(MavlinkStreamNavControllerOutput &) = delete;
	MavlinkStreamNavControllerOutput &operator = (const MavlinkStreamNavControllerOutput &) = delete;

protected:
	explicit MavlinkStreamNavControllerOutput(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_ctrl_status_sub(_mavlink->add_orb_subscription(ORB_ID(position_controller_status))),
		_tecs_status_sub(_mavlink->add_orb_subscription(ORB_ID(tecs_status)))
	{}

	bool send(const hrt_abstime t)
	{
		position_controller_status_s pos_ctrl_status = {};
		tecs_status_s tecs_status = {};

		bool updated = false;
		updated |= _pos_ctrl_status_sub->update(&_pos_ctrl_status_timestamp, &pos_ctrl_status);
		updated |= _tecs_status_sub->update(&_tecs_status_timestamp, &tecs_status);

		if (updated) {
			mavlink_nav_controller_output_t msg = {};

			msg.nav_roll = math::degrees(pos_ctrl_status.nav_roll);
			msg.nav_pitch = math::degrees(pos_ctrl_status.nav_pitch);
			msg.nav_bearing = (int16_t)math::degrees(pos_ctrl_status.nav_bearing);
			msg.target_bearing = (int16_t)math::degrees(pos_ctrl_status.target_bearing);
			msg.wp_dist = (uint16_t)pos_ctrl_status.wp_dist;
			msg.xtrack_error = pos_ctrl_status.xtrack_error;
			msg.alt_error = tecs_status.altitude_filtered - tecs_status.altitude_sp;
			msg.aspd_error = tecs_status.airspeed_filtered - tecs_status.airspeed_sp;

			mavlink_msg_nav_controller_output_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamCameraCapture : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamCameraCapture::get_name_static();
	}

	static const char *get_name_static()
	{
		return "CAMERA_CAPTURE";
	}

	static uint16_t get_id_static()
	{
		return 0;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraCapture(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamCameraCapture(MavlinkStreamCameraCapture &) = delete;
	MavlinkStreamCameraCapture &operator = (const MavlinkStreamCameraCapture &) = delete;

protected:
	explicit MavlinkStreamCameraCapture(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	bool send(const hrt_abstime t)
	{
		vehicle_status_s status;

		if (_status_sub->update(&status)) {

			mavlink_command_long_t msg = {};

			msg.target_system = 0;
			msg.target_component = MAV_COMP_ID_ALL;
			msg.command = MAV_CMD_DO_CONTROL_VIDEO;
			msg.confirmation = 0;
			msg.param1 = 0;
			msg.param2 = 0;
			msg.param3 = 0;
			/* set camera capture ON/OFF depending on arming state */
			msg.param4 = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 1 : 0;
			msg.param5 = 0;
			msg.param6 = 0;
			msg.param7 = 0;

			mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
		}

		return true;
	}
};

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamDistanceSensor::get_name_static();
	}

	static const char *get_name_static()
	{
		return "DISTANCE_SENSOR";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DISTANCE_SENSOR;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDistanceSensor(mavlink);
	}

	unsigned get_size()
	{
		return _distance_sensor_sub->is_published() ? (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_distance_sensor_sub;
	uint64_t _dist_sensor_time;

	/* do not allow top copying this class */
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &) = delete;
	MavlinkStreamDistanceSensor &operator = (const MavlinkStreamDistanceSensor &) = delete;

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink),
		_distance_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(distance_sensor))),
		_dist_sensor_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		distance_sensor_s dist_sensor;

		if (_distance_sensor_sub->update(&_dist_sensor_time, &dist_sensor)) {

			mavlink_distance_sensor_t msg = {};

			msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

			/* TODO: use correct ID here */
			msg.id = 0;

			switch (dist_sensor.type) {
			case MAV_DISTANCE_SENSOR_ULTRASOUND:
				msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
				break;

			case MAV_DISTANCE_SENSOR_LASER:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;

			case MAV_DISTANCE_SENSOR_INFRARED:
				msg.type = MAV_DISTANCE_SENSOR_INFRARED;
				break;

			default:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			msg.orientation = dist_sensor.orientation;
			msg.min_distance = dist_sensor.min_distance * 100.0f; /* m to cm */
			msg.max_distance = dist_sensor.max_distance * 100.0f; /* m to cm */
			msg.current_distance = dist_sensor.current_distance * 100.0f; /* m to cm */
			msg.covariance = dist_sensor.covariance;

			mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamExtendedSysState : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamExtendedSysState::get_name_static();
	}

	static const char *get_name_static()
	{
		return "EXTENDED_SYS_STATE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_EXTENDED_SYS_STATE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamExtendedSysState(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_EXTENDED_SYS_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	MavlinkOrbSubscription *_landed_sub;
	MavlinkOrbSubscription *_pos_sp_triplet_sub;
	MavlinkOrbSubscription *_control_mode_sub;
	mavlink_extended_sys_state_t _msg;

	/* do not allow top copying this class */
	MavlinkStreamExtendedSysState(MavlinkStreamExtendedSysState &) = delete;
	MavlinkStreamExtendedSysState &operator = (const MavlinkStreamExtendedSysState &) = delete;

protected:
	explicit MavlinkStreamExtendedSysState(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_landed_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_land_detected))),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet))),
		_control_mode_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_control_mode))),
		_msg()
	{
		_msg.vtol_state = MAV_VTOL_STATE_UNDEFINED;
		_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;
	}

	bool send(const hrt_abstime t)
	{
		bool updated = false;

		vehicle_status_s status;

		if (_status_sub->update(&status)) {
			updated = true;

			if (status.is_vtol) {
				if (!status.in_transition_mode && status.is_rotary_wing) {
					_msg.vtol_state = MAV_VTOL_STATE_MC;

				} else if (!status.in_transition_mode) {
					_msg.vtol_state = MAV_VTOL_STATE_FW;

				} else if (status.in_transition_mode && status.in_transition_to_fw) {
					_msg.vtol_state = MAV_VTOL_STATE_TRANSITION_TO_FW;

				} else if (status.in_transition_mode) {
					_msg.vtol_state = MAV_VTOL_STATE_TRANSITION_TO_MC;
				}
			}
		}

		vehicle_land_detected_s land_detected;

		if (_landed_sub->update(&land_detected)) {
			updated = true;

			if (land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;

			} else if (!land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_IN_AIR;

				vehicle_control_mode_s control_mode;
				position_setpoint_triplet_s pos_sp_triplet;

				if (_control_mode_sub->update(&control_mode) && _pos_sp_triplet_sub->update(&pos_sp_triplet)) {
					if (control_mode.flag_control_auto_enabled && pos_sp_triplet.current.valid) {
						if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
							_msg.landed_state = MAV_LANDED_STATE_TAKEOFF;

						} else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
							_msg.landed_state = MAV_LANDED_STATE_LANDING;
						}
					}
				}
			}
		}

		if (updated) {
			mavlink_msg_extended_sys_state_send_struct(_mavlink->get_channel(), &_msg);
		}

		return updated;
	}
};

class MavlinkStreamAltitude : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAltitude::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ALTITUDE";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ALTITUDE;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAltitude(mavlink);
	}

	unsigned get_size()
	{
		return (_local_pos_time > 0) ? MAVLINK_MSG_ID_ALTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_local_pos_sub;
	MavlinkOrbSubscription *_home_sub;
	MavlinkOrbSubscription *_air_data_sub;

	uint64_t _local_pos_time{0};

	/* do not allow top copying this class */
	MavlinkStreamAltitude(MavlinkStreamAltitude &) = delete;
	MavlinkStreamAltitude &operator = (const MavlinkStreamAltitude &) = delete;

protected:
	explicit MavlinkStreamAltitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_air_data_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_air_data)))
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_altitude_t msg = {};

		msg.altitude_monotonic = NAN;
		msg.altitude_amsl = NAN;
		msg.altitude_local = NAN;
		msg.altitude_relative = NAN;
		msg.altitude_terrain = NAN;
		msg.bottom_clearance = NAN;

		// always update monotonic altitude
		bool air_data_updated = false;
		vehicle_air_data_s air_data = {};
		_air_data_sub->update(&air_data);

		if (air_data.timestamp > 0) {
			msg.altitude_monotonic = air_data.baro_alt_meter;

			air_data_updated = true;
		}

		bool lpos_updated = false;

		vehicle_local_position_s local_pos;

		if (_local_pos_sub->update(&_local_pos_time, &local_pos)) {

			if (local_pos.z_valid) {
				if (local_pos.z_global) {
					msg.altitude_amsl = -local_pos.z + local_pos.ref_alt;

				} else {
					msg.altitude_amsl = msg.altitude_monotonic;
				}

				msg.altitude_local = -local_pos.z;

				home_position_s home = {};
				_home_sub->update(&home);

				if (home.valid_alt) {
					msg.altitude_relative = -(local_pos.z - home.z);

				} else {
					msg.altitude_relative = -local_pos.z;
				}

				if (local_pos.dist_bottom_valid) {
					msg.altitude_terrain = -local_pos.z - local_pos.dist_bottom;
					msg.bottom_clearance = local_pos.dist_bottom;
				}
			}

			lpos_updated = true;
		}

		// local position timeout after 10 ms
		// avoid publishing only baro altitude_monotonic if possible
		bool lpos_timeout = (hrt_elapsed_time(&_local_pos_time) > 10000);

		if (lpos_updated || (air_data_updated && lpos_timeout)) {
			msg.time_usec = hrt_absolute_time();
			mavlink_msg_altitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamWind : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamWind::get_name_static();
	}

	static const char *get_name_static()
	{
		return "WIND_COV";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_WIND_COV;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamWind(mavlink);
	}

	unsigned get_size()
	{
		return (_wind_estimate_time > 0) ? MAVLINK_MSG_ID_WIND_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_wind_estimate_sub;
	uint64_t _wind_estimate_time;

	MavlinkOrbSubscription *_local_pos_sub;

	/* do not allow top copying this class */
	MavlinkStreamWind(MavlinkStreamWind &) = delete;
	MavlinkStreamWind &operator = (const MavlinkStreamWind &) = delete;

protected:
	explicit MavlinkStreamWind(Mavlink *mavlink) : MavlinkStream(mavlink),
		_wind_estimate_sub(_mavlink->add_orb_subscription(ORB_ID(wind_estimate))),
		_wind_estimate_time(0),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position)))
	{}

	bool send(const hrt_abstime t)
	{
		wind_estimate_s wind_estimate;

		if (_wind_estimate_sub->update(&_wind_estimate_time, &wind_estimate)) {

			mavlink_wind_cov_t msg = {};

			msg.time_usec = wind_estimate.timestamp;

			msg.wind_x = wind_estimate.windspeed_north;
			msg.wind_y = wind_estimate.windspeed_east;
			msg.wind_z = 0.0f;

			msg.var_horiz = wind_estimate.variance_north + wind_estimate.variance_east;
			msg.var_vert = 0.0f;

			vehicle_local_position_s lpos = {};
			_local_pos_sub->update(&lpos);
			msg.wind_alt = (lpos.z_valid && lpos.z_global) ? (-lpos.z + lpos.ref_alt) : NAN;

			msg.horiz_accuracy = 0.0f;
			msg.vert_accuracy = 0.0f;

			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamMountOrientation : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamMountOrientation::get_name_static();
	}

	static const char *get_name_static()
	{
		return "MOUNT_ORIENTATION";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MOUNT_ORIENTATION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamMountOrientation(mavlink);
	}

	unsigned get_size()
	{
		return (_mount_orientation_time > 0) ? MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_mount_orientation_sub;
	uint64_t _mount_orientation_time;

	/* do not allow top copying this class */
	MavlinkStreamMountOrientation(MavlinkStreamMountOrientation &) = delete;
	MavlinkStreamMountOrientation &operator = (const MavlinkStreamMountOrientation &) = delete;

protected:
	explicit MavlinkStreamMountOrientation(Mavlink *mavlink) : MavlinkStream(mavlink),
		_mount_orientation_sub(_mavlink->add_orb_subscription(ORB_ID(mount_orientation))),
		_mount_orientation_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct mount_orientation_s mount_orientation;

		if (_mount_orientation_sub->update(&_mount_orientation_time, &mount_orientation)) {

			mavlink_mount_orientation_t msg = {};

			msg.roll = math::degrees(mount_orientation.attitude_euler_angle[0]);
			msg.pitch = math::degrees(mount_orientation.attitude_euler_angle[1]);
			msg.yaw = math::degrees(mount_orientation.attitude_euler_angle[2]);

			mavlink_msg_mount_orientation_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamGroundTruth : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGroundTruth::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GROUND_TRUTH";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_STATE_QUATERNION;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGroundTruth(mavlink);
	}

	unsigned get_size()
	{
		return (_att_time > 0 || _gpos_time > 0) ? MAVLINK_MSG_ID_HIL_STATE_QUATERNION_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_att_sub;
	MavlinkOrbSubscription *_gpos_sub;
	uint64_t _att_time;
	uint64_t _gpos_time;

	/* do not allow top copying this class */
	MavlinkStreamGroundTruth(MavlinkStreamGroundTruth &) = delete;
	MavlinkStreamGroundTruth &operator = (const MavlinkStreamGroundTruth &) = delete;

protected:
	explicit MavlinkStreamGroundTruth(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_groundtruth))),
		_gpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position_groundtruth))),
		_att_time(0),
		_gpos_time(0)
	{}

	bool send(const hrt_abstime t)
	{

		vehicle_attitude_s att = {};
		vehicle_global_position_s gpos = {};
		bool att_updated = _att_sub->update(&_att_time, &att);
		bool gpos_updated = _gpos_sub->update(&_gpos_time, &gpos);

		if (att_updated || gpos_updated) {

			mavlink_hil_state_quaternion_t msg = {};

			// vehicle_attitude -> hil_state_quaternion
			msg.attitude_quaternion[0] = att.q[0];
			msg.attitude_quaternion[1] = att.q[1];
			msg.attitude_quaternion[2] = att.q[2];
			msg.attitude_quaternion[3] = att.q[3];
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			// vehicle_global_position -> hil_state_quaternion
			msg.lat = gpos.lat;
			msg.lon = gpos.lon;
			msg.alt = gpos.alt;
			msg.vx = gpos.vel_n;
			msg.vy = gpos.vel_e;
			msg.vz = gpos.vel_d;
			msg.ind_airspeed = 0;
			msg.true_airspeed = 0;
			msg.xacc = 0;
			msg.yacc = 0;
			msg.zacc = 0;

			mavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

class MavlinkStreamPing : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamPing::get_name_static();
	}

	static const char *get_name_static()
	{
		return "PING";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_PING;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPing(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_PING_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

private:
	uint32_t _sequence;

	/* do not allow top copying this class */
	MavlinkStreamPing(MavlinkStreamPing &) = delete;
	MavlinkStreamPing &operator = (const MavlinkStreamPing &) = delete;

protected:
	explicit MavlinkStreamPing(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sequence(0)
	{}

	bool send(const hrt_abstime t)
	{
		mavlink_ping_t msg = {};

		msg.time_usec = hrt_absolute_time();
		msg.seq = _sequence++;
		msg.target_system = 0; // All systems
		msg.target_component = 0; // All components

		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

class MavlinkStreamOrbitStatus : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOrbitStatus::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ORBIT_EXECUTION_STATUS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOrbitStatus(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_sub;
	uint64_t _orbit_status_time;

	/* do not allow top copying this class */
	MavlinkStreamOrbitStatus(MavlinkStreamOrbitStatus &);
	MavlinkStreamOrbitStatus &operator = (const MavlinkStreamOrbitStatus &);

protected:
	explicit MavlinkStreamOrbitStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sub(_mavlink->add_orb_subscription(ORB_ID(orbit_status))),
		_orbit_status_time(0)
	{}

	bool send(const hrt_abstime t)
	{
		struct orbit_status_s _orbit_status;

		if (_sub->update(&_orbit_status_time, &_orbit_status)) {
			mavlink_orbit_execution_status_t _msg_orbit_execution_status;

			_msg_orbit_execution_status.time_usec = _orbit_status.timestamp;
			_msg_orbit_execution_status.radius = _orbit_status.radius;
			_msg_orbit_execution_status.frame  = _orbit_status.frame;
			_msg_orbit_execution_status.x = _orbit_status.x * 1e7;
			_msg_orbit_execution_status.y = _orbit_status.y * 1e7;
			_msg_orbit_execution_status.z = _orbit_status.z;

			mavlink_msg_orbit_execution_status_send_struct(_mavlink->get_channel(), &_msg_orbit_execution_status);
		}

		return true;
	}
};

static const StreamListItem streams_list[] = {
	StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static, &MavlinkStreamHeartbeat::get_id_static),
	StreamListItem(&MavlinkStreamStatustext::new_instance, &MavlinkStreamStatustext::get_name_static, &MavlinkStreamStatustext::get_id_static),
	StreamListItem(&MavlinkStreamCommandLong::new_instance, &MavlinkStreamCommandLong::get_name_static, &MavlinkStreamCommandLong::get_id_static),
	StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static, &MavlinkStreamSysStatus::get_id_static),
	StreamListItem(&MavlinkStreamHighresIMU::new_instance, &MavlinkStreamHighresIMU::get_name_static, &MavlinkStreamHighresIMU::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU::new_instance, &MavlinkStreamScaledIMU::get_name_static, &MavlinkStreamScaledIMU::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU2::new_instance, &MavlinkStreamScaledIMU2::get_name_static, &MavlinkStreamScaledIMU2::get_id_static),
	StreamListItem(&MavlinkStreamScaledIMU3::new_instance, &MavlinkStreamScaledIMU3::get_name_static, &MavlinkStreamScaledIMU3::get_id_static),
	StreamListItem(&MavlinkStreamAttitude::new_instance, &MavlinkStreamAttitude::get_name_static, &MavlinkStreamAttitude::get_id_static),
	StreamListItem(&MavlinkStreamAttitudeQuaternion::new_instance, &MavlinkStreamAttitudeQuaternion::get_name_static, &MavlinkStreamAttitudeQuaternion::get_id_static),
	StreamListItem(&MavlinkStreamVFRHUD::new_instance, &MavlinkStreamVFRHUD::get_name_static, &MavlinkStreamVFRHUD::get_id_static),
	StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static, &MavlinkStreamGPSRawInt::get_id_static),
	StreamListItem(&MavlinkStreamGPS2Raw::new_instance, &MavlinkStreamGPS2Raw::get_name_static, &MavlinkStreamGPS2Raw::get_id_static),
	StreamListItem(&MavlinkStreamSystemTime::new_instance, &MavlinkStreamSystemTime::get_name_static, &MavlinkStreamSystemTime::get_id_static),
	StreamListItem(&MavlinkStreamTimesync::new_instance, &MavlinkStreamTimesync::get_name_static, &MavlinkStreamTimesync::get_id_static),
	StreamListItem(&MavlinkStreamGlobalPositionInt::new_instance, &MavlinkStreamGlobalPositionInt::get_name_static, &MavlinkStreamGlobalPositionInt::get_id_static),
	StreamListItem(&MavlinkStreamLocalPositionNED::new_instance, &MavlinkStreamLocalPositionNED::get_name_static, &MavlinkStreamLocalPositionNED::get_id_static),
	StreamListItem(&MavlinkStreamVisionPositionEstimate::new_instance, &MavlinkStreamVisionPositionEstimate::get_name_static, &MavlinkStreamVisionPositionEstimate::get_id_static),
	StreamListItem(&MavlinkStreamLocalPositionNEDCOV::new_instance, &MavlinkStreamLocalPositionNEDCOV::get_name_static, &MavlinkStreamLocalPositionNEDCOV::get_id_static),
	StreamListItem(&MavlinkStreamEstimatorStatus::new_instance, &MavlinkStreamEstimatorStatus::get_name_static, &MavlinkStreamEstimatorStatus::get_id_static),
	StreamListItem(&MavlinkStreamAttPosMocap::new_instance, &MavlinkStreamAttPosMocap::get_name_static, &MavlinkStreamAttPosMocap::get_id_static),
	StreamListItem(&MavlinkStreamHomePosition::new_instance, &MavlinkStreamHomePosition::get_name_static, &MavlinkStreamHomePosition::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<0>::new_instance, &MavlinkStreamServoOutputRaw<0>::get_name_static, &MavlinkStreamServoOutputRaw<0>::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<1>::new_instance, &MavlinkStreamServoOutputRaw<1>::get_name_static, &MavlinkStreamServoOutputRaw<1>::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<2>::new_instance, &MavlinkStreamServoOutputRaw<2>::get_name_static, &MavlinkStreamServoOutputRaw<2>::get_id_static),
	StreamListItem(&MavlinkStreamServoOutputRaw<3>::new_instance, &MavlinkStreamServoOutputRaw<3>::get_name_static, &MavlinkStreamServoOutputRaw<3>::get_id_static),
	StreamListItem(&MavlinkStreamHILActuatorControls::new_instance, &MavlinkStreamHILActuatorControls::get_name_static, &MavlinkStreamHILActuatorControls::get_id_static),
	StreamListItem(&MavlinkStreamPositionTargetGlobalInt::new_instance, &MavlinkStreamPositionTargetGlobalInt::get_name_static, &MavlinkStreamPositionTargetGlobalInt::get_id_static),
	StreamListItem(&MavlinkStreamLocalPositionSetpoint::new_instance, &MavlinkStreamLocalPositionSetpoint::get_name_static, &MavlinkStreamLocalPositionSetpoint::get_id_static),
	StreamListItem(&MavlinkStreamAttitudeTarget::new_instance, &MavlinkStreamAttitudeTarget::get_name_static, &MavlinkStreamAttitudeTarget::get_id_static),
	StreamListItem(&MavlinkStreamRCChannels::new_instance, &MavlinkStreamRCChannels::get_name_static, &MavlinkStreamRCChannels::get_id_static),
	StreamListItem(&MavlinkStreamManualControl::new_instance, &MavlinkStreamManualControl::get_name_static, &MavlinkStreamManualControl::get_id_static),
	StreamListItem(&MavlinkStreamTrajectoryRepresentationWaypoints::new_instance, &MavlinkStreamTrajectoryRepresentationWaypoints::get_name_static, &MavlinkStreamTrajectoryRepresentationWaypoints::get_id_static),
	StreamListItem(&MavlinkStreamOpticalFlowRad::new_instance, &MavlinkStreamOpticalFlowRad::get_name_static, &MavlinkStreamOpticalFlowRad::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<0>::new_instance, &MavlinkStreamActuatorControlTarget<0>::get_name_static, &MavlinkStreamActuatorControlTarget<0>::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<1>::new_instance, &MavlinkStreamActuatorControlTarget<1>::get_name_static, &MavlinkStreamActuatorControlTarget<1>::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<2>::new_instance, &MavlinkStreamActuatorControlTarget<2>::get_name_static, &MavlinkStreamActuatorControlTarget<2>::get_id_static),
	StreamListItem(&MavlinkStreamActuatorControlTarget<3>::new_instance, &MavlinkStreamActuatorControlTarget<3>::get_name_static, &MavlinkStreamActuatorControlTarget<3>::get_id_static),
	StreamListItem(&MavlinkStreamNamedValueFloat::new_instance, &MavlinkStreamNamedValueFloat::get_name_static, &MavlinkStreamNamedValueFloat::get_id_static),
	StreamListItem(&MavlinkStreamDebug::new_instance, &MavlinkStreamDebug::get_name_static, &MavlinkStreamDebug::get_id_static),
	StreamListItem(&MavlinkStreamDebugVect::new_instance, &MavlinkStreamDebugVect::get_name_static, &MavlinkStreamDebugVect::get_id_static),
	StreamListItem(&MavlinkStreamDebugFloatArray::new_instance, &MavlinkStreamDebugFloatArray::get_name_static, &MavlinkStreamDebugFloatArray::get_id_static),
	StreamListItem(&MavlinkStreamNavControllerOutput::new_instance, &MavlinkStreamNavControllerOutput::get_name_static, &MavlinkStreamNavControllerOutput::get_id_static),
	StreamListItem(&MavlinkStreamCameraCapture::new_instance, &MavlinkStreamCameraCapture::get_name_static, &MavlinkStreamCameraCapture::get_id_static),
	StreamListItem(&MavlinkStreamCameraTrigger::new_instance, &MavlinkStreamCameraTrigger::get_name_static, &MavlinkStreamCameraTrigger::get_id_static),
	StreamListItem(&MavlinkStreamCameraImageCaptured::new_instance, &MavlinkStreamCameraImageCaptured::get_name_static, &MavlinkStreamCameraImageCaptured::get_id_static),
	StreamListItem(&MavlinkStreamDistanceSensor::new_instance, &MavlinkStreamDistanceSensor::get_name_static, &MavlinkStreamDistanceSensor::get_id_static),
	StreamListItem(&MavlinkStreamExtendedSysState::new_instance, &MavlinkStreamExtendedSysState::get_name_static, &MavlinkStreamExtendedSysState::get_id_static),
	StreamListItem(&MavlinkStreamAltitude::new_instance, &MavlinkStreamAltitude::get_name_static, &MavlinkStreamAltitude::get_id_static),
	StreamListItem(&MavlinkStreamADSBVehicle::new_instance, &MavlinkStreamADSBVehicle::get_name_static, &MavlinkStreamADSBVehicle::get_id_static),
	StreamListItem(&MavlinkStreamUTMGlobalPosition::new_instance, &MavlinkStreamUTMGlobalPosition::get_name_static, &MavlinkStreamUTMGlobalPosition::get_id_static),
	StreamListItem(&MavlinkStreamCollision::new_instance, &MavlinkStreamCollision::get_name_static, &MavlinkStreamCollision::get_id_static),
	StreamListItem(&MavlinkStreamWind::new_instance, &MavlinkStreamWind::get_name_static, &MavlinkStreamWind::get_id_static),
	StreamListItem(&MavlinkStreamMountOrientation::new_instance, &MavlinkStreamMountOrientation::get_name_static, &MavlinkStreamMountOrientation::get_id_static),
	StreamListItem(&MavlinkStreamHighLatency2::new_instance, &MavlinkStreamHighLatency2::get_name_static, &MavlinkStreamHighLatency2::get_id_static),
	StreamListItem(&MavlinkStreamGroundTruth::new_instance, &MavlinkStreamGroundTruth::get_name_static, &MavlinkStreamGroundTruth::get_id_static),
	StreamListItem(&MavlinkStreamPing::new_instance, &MavlinkStreamPing::get_name_static, &MavlinkStreamPing::get_id_static),
	StreamListItem(&MavlinkStreamOrbitStatus::new_instance, &MavlinkStreamOrbitStatus::get_name_static, &MavlinkStreamOrbitStatus::get_id_static)
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}
