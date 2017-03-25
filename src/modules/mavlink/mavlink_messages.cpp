/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * MAVLink 1.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>
#include <errno.h>

#include "mavlink_main.h"
#include "mavlink_messages.h"

#include <commander/px4_custom_mode.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_time.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/cpuload.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/collision_report.h>
#include <uORB/uORB.h>


static uint16_t cm_uint16_from_m_float(float m);
static void get_mavlink_mode_state(struct vehicle_status_s *status, uint8_t *mavlink_state,
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

void get_mavlink_mode_state(struct vehicle_status_s *status, uint8_t *mavlink_state,
			    uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED
	    || status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	union px4_custom_mode custom_mode;
	custom_mode.data = 0;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RCRECOVER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTGS:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTGS;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}

	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) {
		*mavlink_state = MAV_STATE_CRITICAL;

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
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &);
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &);

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	void send(const hrt_abstime t)
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
	MavlinkStreamStatustext(MavlinkStreamStatustext &);
	MavlinkStreamStatustext &operator = (const MavlinkStreamStatustext &);

	unsigned _write_err_count = 0;
	static const unsigned write_err_threshold = 5;
#if !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)
	FILE *_fp = nullptr;
#endif

protected:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	~MavlinkStreamStatustext()
	{
#if !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)

		if (_fp != nullptr) {
			fclose(_fp);
		}

#endif
	}


	void send(const hrt_abstime t)
	{
		if (!_mavlink->get_logbuffer()->empty()) {

			struct mavlink_log_s mavlink_log = {};

			if (_mavlink->get_logbuffer()->get(&mavlink_log)) {

				mavlink_statustext_t msg = {};
				msg.severity = mavlink_log.severity;
				strncpy(msg.text, (const char *)mavlink_log.text, sizeof(msg.text));
				msg.text[sizeof(msg.text) - 1] = '\0';

				mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

// TODO: the logging doesn't work on Snapdragon yet because of file paths.
#if !defined(__PX4_POSIX_EAGLE) && !defined(__PX4_POSIX_EXCELSIOR)
				/* write log messages in first instance to disk
				 * timestamp each message with gps time
				 */
				timespec ts;
				px4_clock_gettime(CLOCK_REALTIME, &ts);
				time_t gps_time_sec = ts.tv_sec + (ts.tv_nsec / 1e9);
				struct tm tt = {};
				gmtime_r(&gps_time_sec, &tt);
				char tstamp[22];
				strftime(tstamp, sizeof(tstamp) - 1, "%Y_%m_%d_%H_%M_%S", &tt);

				if (_mavlink->get_instance_id() == 0/* && _mavlink->get_logging_enabled()*/) {
					if (_fp != nullptr) {
						fputs(tstamp, _fp);
						fputs(": ", _fp);

						if (EOF == fputs(msg.text, _fp)) {
							_write_err_count++;

						} else {
							_write_err_count = 0;
						}

						if (_write_err_count >= write_err_threshold) {
							(void)fclose(_fp);
							_fp = nullptr;
							PX4_WARN("mavlink logging disabled");

						} else {
							(void)fputs("\n", _fp);
#ifdef __PX4_NUTTX
							fsync(fileno(_fp));
#endif
						}

					} else if (_write_err_count < write_err_threshold) {
						/* string to hold the path to the log */
						char log_file_path[128];

						/* use GPS time for log file naming, e.g. /fs/microsd/2014-01-19/19_37_52.bin */

						/* store the log file in the root directory */
						snprintf(log_file_path, sizeof(log_file_path) - 1, PX4_ROOTFSDIR"/fs/microsd/msgs_%s.txt", tstamp);
						_fp = fopen(log_file_path, "ab");

						if (_fp != nullptr) {
							/* write first message */
							fputs(tstamp, _fp);
							fputs(": ", _fp);
							fputs(msg.text, _fp);
							fputs("\n", _fp);
#ifdef __PX4_NUTTX
							fsync(fileno(_fp));
#endif

						} else {
							PX4_WARN("Failed to open MAVLink log: %s", log_file_path);
							_write_err_count = write_err_threshold; //only try to open the file once
						}
					}
				}

#endif
			}
		}
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
	uint64_t _cmd_time;

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &);
	MavlinkStreamCommandLong &operator = (const MavlinkStreamCommandLong &);

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink),
		_cmd_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_command))),
		_cmd_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_command_s cmd = {};

		if (_cmd_sub->update(&_cmd_time, &cmd)) {
			/* only send commands for other systems/components */
			if (cmd.target_system != mavlink_system.sysid || cmd.target_component != mavlink_system.compid) {
				mavlink_command_long_t msg = {};

				msg.target_system = cmd.target_system;
				msg.target_component = cmd.target_component;
				msg.command = cmd.command;
				msg.confirmation = cmd.confirmation;
				msg.param1 = cmd.param1;
				msg.param2 = cmd.param2;
				msg.param3 = cmd.param3;
				msg.param4 = cmd.param4;
				msg.param5 = cmd.param5;
				msg.param6 = cmd.param6;
				msg.param7 = cmd.param7;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
			}
		}
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

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &);
	MavlinkStreamSysStatus &operator = (const MavlinkStreamSysStatus &);

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_cpuload_sub(_mavlink->add_orb_subscription(ORB_ID(cpuload))),
		_battery_status_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};
		struct cpuload_s cpuload = {};
		struct battery_status_s battery_status = {};

		const bool updated_status = _status_sub->update(&status);
		const bool updated_cpuload = _cpuload_sub->update(&cpuload);
		const bool updated_battery = _battery_status_sub->update(&battery_status);

		if (updated_status) {
			if (status.arming_state >= vehicle_status_s::ARMING_STATE_ARMED) {
				_mavlink->set_logging_enabled(true);

			} else {
				_mavlink->set_logging_enabled(false);
			}
		}

		if (updated_status || updated_battery || updated_cpuload) {
			mavlink_sys_status_t msg = {};

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;
			msg.load = cpuload.load * 1000.0f;
			msg.voltage_battery = (battery_status.connected) ? battery_status.voltage_filtered_v * 1000.0f : UINT16_MAX;
			msg.current_battery = (battery_status.connected) ? battery_status.current_filtered_a * 100.0f : -1;
			msg.battery_remaining = (battery_status.connected) ? battery_status.remaining * 100.0f : -1;
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
			bat_msg.battery_remaining = (battery_status.connected) ? battery_status.remaining * 100.0f : -1;
			bat_msg.temperature = INT16_MAX;

			for (unsigned int i = 0; i < (sizeof(bat_msg.voltages) / sizeof(bat_msg.voltages[0])); i++) {
				if ((int)i < battery_status.cell_count && battery_status.connected) {
					bat_msg.voltages[i] = (battery_status.voltage_v / battery_status.cell_count) * 1000.0f;

				} else {
					bat_msg.voltages[i] = UINT16_MAX;
				}
			}

			mavlink_msg_battery_status_send_struct(_mavlink->get_channel(), &bat_msg);
		}
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

	MavlinkOrbSubscription *_differential_pressure_sub;
	uint64_t _differential_pressure_time;

	uint64_t _accel_timestamp;
	uint64_t _gyro_timestamp;
	uint64_t _mag_timestamp;
	uint64_t _baro_timestamp;

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &);
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &);

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0),
		_differential_pressure_sub(_mavlink->add_orb_subscription(ORB_ID(differential_pressure))),
		_differential_pressure_time(0),
		_accel_timestamp(0),
		_gyro_timestamp(0),
		_mag_timestamp(0),
		_baro_timestamp(0)
	{}

	void send(const hrt_abstime t)
	{
		struct sensor_combined_s sensor = {};
		struct differential_pressure_s differential_pressure = {};

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

			if (_mag_timestamp != sensor.timestamp + sensor.magnetometer_timestamp_relative) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
				_mag_timestamp = sensor.timestamp + sensor.magnetometer_timestamp_relative;
			}

			if (_baro_timestamp != sensor.timestamp + sensor.baro_timestamp_relative) {
				/* mark last group dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
				_baro_timestamp = sensor.timestamp + sensor.baro_timestamp_relative;
			}

			_differential_pressure_sub->update(&_differential_pressure_time, &differential_pressure);

			mavlink_highres_imu_t msg = {};

			msg.time_usec = sensor.timestamp;
			msg.xacc = sensor.accelerometer_m_s2[0];
			msg.yacc = sensor.accelerometer_m_s2[1];
			msg.zacc = sensor.accelerometer_m_s2[2];
			msg.xgyro = sensor.gyro_rad[0];
			msg.ygyro = sensor.gyro_rad[1];
			msg.zgyro = sensor.gyro_rad[2];
			msg.xmag = sensor.magnetometer_ga[0];
			msg.ymag = sensor.magnetometer_ga[1];
			msg.zmag = sensor.magnetometer_ga[2];
			msg.abs_pressure = 0;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = sensor.baro_alt_meter;
			msg.temperature = sensor.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamAttitude(MavlinkStreamAttitude &);
	MavlinkStreamAttitude &operator = (const MavlinkStreamAttitude &);


protected:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;

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
		}
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
	MavlinkStreamAttitudeQuaternion(MavlinkStreamAttitudeQuaternion &);
	MavlinkStreamAttitudeQuaternion &operator = (const MavlinkStreamAttitudeQuaternion &);

protected:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;

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
		}
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
	MavlinkOrbSubscription *_att_sub;
	uint64_t _att_time;

	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	MavlinkOrbSubscription *_armed_sub;
	uint64_t _armed_time;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	MavlinkOrbSubscription *_airspeed_sub;
	uint64_t _airspeed_time;

	MavlinkOrbSubscription *_sensor_sub;
	uint64_t _sensor_time;

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &);
	MavlinkStreamVFRHUD &operator = (const MavlinkStreamVFRHUD &);

protected:
	explicit MavlinkStreamVFRHUD(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_pos_time(0),
		_armed_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_armed))),
		_armed_time(0),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_0))),
		_act_time(0),
		_airspeed_sub(_mavlink->add_orb_subscription(ORB_ID(airspeed))),
		_airspeed_time(0),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att = {};
		struct vehicle_global_position_s pos = {};
		struct actuator_armed_s armed = {};
		struct actuator_controls_s act = {};
		struct airspeed_s airspeed = {};

		bool updated = _att_sub->update(&_att_time, &att);
		updated |= _pos_sub->update(&_pos_time, &pos);
		updated |= _armed_sub->update(&_armed_time, &armed);
		updated |= _act_sub->update(&_act_time, &act);
		updated |= _airspeed_sub->update(&_airspeed_time, &airspeed);

		if (updated) {
			mavlink_vfr_hud_t msg = {};
			matrix::Eulerf euler = matrix::Quatf(att.q);
			msg.airspeed = airspeed.indicated_airspeed_m_s;
			msg.groundspeed = sqrtf(pos.vel_n * pos.vel_n + pos.vel_e * pos.vel_e);
			msg.heading = _wrap_2pi(euler.psi()) * M_RAD_TO_DEG_F;
			msg.throttle = armed.armed ? act.control[3] * 100.0f : 0.0f;

			if (_pos_time > 0) {
				/* use global estimate */
				msg.alt = pos.alt;

			} else {
				/* fall back to baro altitude */
				sensor_combined_s sensor;
				(void)_sensor_sub->update(&_sensor_time, &sensor);
				msg.alt = sensor.baro_alt_meter;
			}

			msg.climb = -pos.vel_d;

			mavlink_msg_vfr_hud_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamGPSRawInt(MavlinkStreamGPSRawInt &);
	MavlinkStreamGPSRawInt &operator = (const MavlinkStreamGPSRawInt &);

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
		_gps_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps_raw_int_t msg = {};

			msg.time_usec = gps.timestamp;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = gps.hdop * 100; //cm_uint16_from_m_float(gps.eph);
			msg.epv = gps.vdop * 100; //cm_uint16_from_m_float(gps.epv);
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s),
			    msg.cog = _wrap_2pi(gps.cog_rad) * M_RAD_TO_DEG_F * 1e2f,
				msg.satellites_visible = gps.satellites_used;

			mavlink_msg_gps_raw_int_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &);
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &);

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	void send(const hrt_abstime t)
	{
		mavlink_system_time_t msg = {};
		timespec tv;

		px4_clock_gettime(CLOCK_REALTIME, &tv);

		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
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
	MavlinkStreamTimesync(MavlinkStreamTimesync &);
	MavlinkStreamTimesync &operator = (const MavlinkStreamTimesync &);

protected:
	explicit MavlinkStreamTimesync(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	void send(const hrt_abstime t)
	{
		mavlink_timesync_t msg = {};

		msg.tc1 = 0;
		msg.ts1 = hrt_absolute_time() * 1000; // boot time in nanoseconds

		mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &msg);
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

	unsigned get_size()
	{
		return (_pos_time > 0) ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	/* do not allow top copying this class */
	MavlinkStreamADSBVehicle(MavlinkStreamADSBVehicle &);
	MavlinkStreamADSBVehicle &operator = (const MavlinkStreamADSBVehicle &);

protected:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(transponder_report))),
		_pos_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct transponder_report_s pos;

		if (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_adsb_vehicle_t msg = {};

			msg.ICAO_address = pos.ICAO_address;
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
			msg.flags = pos.flags;
			msg.squawk = pos.squawk;

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamCollision(MavlinkStreamCollision &);
	MavlinkStreamCollision &operator = (const MavlinkStreamCollision &);

protected:
	explicit MavlinkStreamCollision(Mavlink *mavlink) : MavlinkStream(mavlink),
		_collision_sub(_mavlink->add_orb_subscription(ORB_ID(collision_report))),
		_collision_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct collision_report_s report;

		if (_collision_sub->update(&_collision_time, &report)) {
			mavlink_collision_t msg = {};

			msg.src = report.src;
			msg.id = report.id;
			msg.action = report.action;
			msg.threat_level = report.threat_level;
			msg.time_to_minimum_delta = report.time_to_minimum_delta;
			msg.altitude_minimum_delta = report.altitude_minimum_delta;
			msg.horizontal_minimum_delta = report.horizontal_minimum_delta;

			mavlink_msg_collision_send_struct(_mavlink->get_channel(), &msg);
		}
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

	unsigned get_size()
	{
		return (_trigger_time > 0) ? MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_trigger_sub;
	uint64_t _trigger_time;

	/* do not allow top copying this class */
	MavlinkStreamCameraTrigger(MavlinkStreamCameraTrigger &);
	MavlinkStreamCameraTrigger &operator = (const MavlinkStreamCameraTrigger &);

protected:
	explicit MavlinkStreamCameraTrigger(Mavlink *mavlink) : MavlinkStream(mavlink),
		_trigger_sub(_mavlink->add_orb_subscription(ORB_ID(camera_trigger))),
		_trigger_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct camera_trigger_s trigger;

		if (_trigger_sub->update(&_trigger_time, &trigger)) {
			mavlink_camera_trigger_t msg = {};

			msg.time_usec = trigger.timestamp;
			msg.seq = trigger.seq;

			/* ensure that only active trigger events are sent */
			if (trigger.timestamp > 0) {
				mavlink_msg_camera_trigger_send_struct(_mavlink->get_channel(), &msg);

				/* send MAV_CMD_IMAGE_START_CAPTURE */
				mavlink_command_long_t msg_cmd;

				msg_cmd.target_system = mavlink_system.sysid;
				msg_cmd.target_component = MAV_COMP_ID_CAMERA;
				msg_cmd.command = MAV_CMD_IMAGE_START_CAPTURE;
				msg_cmd.confirmation = 0;
				msg_cmd.param1 = 0; // duration between 2 consecutive images (seconds)
				msg_cmd.param2 = 1; // take 1 picture
				msg_cmd.param3 = -1; // resolution (use the highest possible)
				msg_cmd.param4 = NAN;
				msg_cmd.param5 = NAN;
				msg_cmd.param6 = NAN;
				msg_cmd.param7 = NAN;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg_cmd);

				/* send MAV_CMD_DO_DIGICAM_CONTROL*/
				mavlink_command_long_t digicam_ctrl_cmd;

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
			}
		}
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
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	MavlinkOrbSubscription *_home_sub;
	uint64_t _home_time;

	/* do not allow top copying this class */
	MavlinkStreamGlobalPositionInt(MavlinkStreamGlobalPositionInt &);
	MavlinkStreamGlobalPositionInt &operator = (const MavlinkStreamGlobalPositionInt &);

protected:
	explicit MavlinkStreamGlobalPositionInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_pos_time(0),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_home_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_global_position_s pos = {};
		struct home_position_s home = {};

		bool updated = _pos_sub->update(&_pos_time, &pos);
		updated |= _home_sub->update(&_home_time, &home);

		if (updated) {
			mavlink_global_position_int_t msg = {};

			msg.time_boot_ms = pos.timestamp / 1000;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.alt = pos.alt * 1000.0f;
			msg.relative_alt = (pos.alt - home.alt) * 1000.0f;
			msg.vx = pos.vel_n * 100.0f;
			msg.vy = pos.vel_e * 100.0f;
			msg.vz = pos.vel_d * 100.0f;
			msg.hdg = _wrap_2pi(pos.yaw) * M_RAD_TO_DEG_F * 100.0f;

			mavlink_msg_global_position_int_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

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
		return (_pos_time > 0) ? MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}
private:

	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	MavlinkOrbSubscription *_att_sub;
	uint64_t _att_time;

	/* do not allow top copying this class */
	MavlinkStreamVisionPositionEstimate(MavlinkStreamVisionPositionEstimate &);
	MavlinkStreamVisionPositionEstimate &operator = (const MavlinkStreamVisionPositionEstimate &);

protected:
	explicit MavlinkStreamVisionPositionEstimate(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_vision_position))),
		_pos_time(0),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_vision_attitude))),
		_att_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_s vpos = {};
		struct vehicle_attitude_s vatt = {};

		bool att_updated = _att_sub->update(&_att_time, &vatt);
		bool pos_updated = _pos_sub->update(&_pos_time, &vpos);

		if (pos_updated || att_updated) {
			mavlink_vision_position_estimate_t vmsg = {};
			vmsg.usec = vpos.timestamp;
			vmsg.x = vpos.x;
			vmsg.y = vpos.y;
			vmsg.z = vpos.z;
			math::Quaternion q(vatt.q);
			math::Vector<3> rpy = q.to_euler();
			vmsg.roll = rpy(0);
			vmsg.pitch = rpy(1);
			vmsg.yaw = rpy(2);

			mavlink_msg_vision_position_estimate_send_struct(_mavlink->get_channel(), &vmsg);
		}
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
	MavlinkStreamLocalPositionNED(MavlinkStreamLocalPositionNED &);
	MavlinkStreamLocalPositionNED &operator = (const MavlinkStreamLocalPositionNED &);

protected:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_pos_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_s pos;

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
		}
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
	MavlinkStreamLocalPositionNEDCOV(MavlinkStreamLocalPositionNEDCOV &);
	MavlinkStreamLocalPositionNEDCOV &operator = (const MavlinkStreamLocalPositionNEDCOV &);

protected:
	explicit MavlinkStreamLocalPositionNEDCOV(Mavlink *mavlink) : MavlinkStream(mavlink),
		_est_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
		_est_time(0)
	{}

	void send(const hrt_abstime t)
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

			msg.covariance[9] = est.nan_flags;
			msg.covariance[10] = est.health_flags;
			msg.covariance[11] = est.timeout_flags;

			mavlink_msg_local_position_ned_cov_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamEstimatorStatus(MavlinkStreamEstimatorStatus &);
	MavlinkStreamEstimatorStatus &operator = (const MavlinkStreamEstimatorStatus &);

protected:
	explicit MavlinkStreamEstimatorStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_est_sub(_mavlink->add_orb_subscription(ORB_ID(estimator_status))),
		_est_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct estimator_status_s est;

		if (_est_sub->update(&_est_time, &est)) {

			mavlink_estimator_status_t est_msg = {};

			est_msg.time_usec = est.timestamp;
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;
			est_msg.mag_ratio = est.mag_test_ratio;
			est_msg.vel_ratio = est.vel_test_ratio;
			est_msg.pos_horiz_ratio = est.pos_test_ratio;
			est_msg.pos_vert_ratio = est.hgt_test_ratio;
			est_msg.hagl_ratio = est.hagl_test_ratio;
			est_msg.tas_ratio = est.tas_test_ratio;
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;
			est_msg.flags = est.solution_status_flags;

			mavlink_msg_estimator_status_send_struct(_mavlink->get_channel(), &est_msg);

			mavlink_vibration_t msg = {};

			msg.vibration_x = est.vibe[0];
			msg.vibration_y = est.vibe[1];
			msg.vibration_z = est.vibe[2];

			mavlink_msg_vibration_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamAttPosMocap(MavlinkStreamAttPosMocap &);
	MavlinkStreamAttPosMocap &operator = (const MavlinkStreamAttPosMocap &);

protected:
	explicit MavlinkStreamAttPosMocap(Mavlink *mavlink) : MavlinkStream(mavlink),
		_mocap_sub(_mavlink->add_orb_subscription(ORB_ID(att_pos_mocap))),
		_mocap_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct att_pos_mocap_s mocap;

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
		}
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
	MavlinkStreamHomePosition(MavlinkStreamHomePosition &);
	MavlinkStreamHomePosition &operator = (const MavlinkStreamHomePosition &);

protected:
	explicit MavlinkStreamHomePosition(Mavlink *mavlink) : MavlinkStream(mavlink),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position)))
	{}

	void send(const hrt_abstime t)
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		if (_home_sub->is_published()) {
			struct home_position_s home;

			if (_home_sub->update(&home)) {
				mavlink_home_position_t msg = {};

				msg.latitude = home.lat * 1e7;
				msg.longitude = home.lon * 1e7;
				msg.altitude = home.alt * 1e3f;

				msg.x = home.x;
				msg.y = home.y;
				msg.z = home.z;

				matrix::Eulerf euler(0.0f, 0.0f, home.yaw);
				matrix::Quatf q(euler);

				msg.q[0] = q(0);
				msg.q[1] = q(1);
				msg.q[2] = q(2);
				msg.q[3] = q(3);

				msg.approach_x = 0.0f;
				msg.approach_y = 0.0f;
				msg.approach_z = 0.0f;

				mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);
			}
		}
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
	MavlinkStreamServoOutputRaw(MavlinkStreamServoOutputRaw &);
	MavlinkStreamServoOutputRaw &operator = (const MavlinkStreamServoOutputRaw &);

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_act_sub(nullptr),
		_act_time(0)
	{
		_act_sub = _mavlink->add_orb_subscription(ORB_ID(actuator_outputs), N);
	}

	void send(const hrt_abstime t)
	{
		struct actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			mavlink_servo_output_raw_t msg = {};

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

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamActuatorControlTarget(MavlinkStreamActuatorControlTarget &);
	MavlinkStreamActuatorControlTarget &operator = (const MavlinkStreamActuatorControlTarget &);

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

	void send(const hrt_abstime t)
	{
		struct actuator_controls_s att_ctrl;

		if (_att_ctrl_sub->update(&_att_ctrl_time, &att_ctrl)) {
			mavlink_actuator_control_target_t msg = {};

			msg.time_usec = att_ctrl.timestamp;
			msg.group_mlx = N;

			for (unsigned i = 0; i < sizeof(msg.controls) / sizeof(msg.controls[0]); i++) {
				msg.controls[i] = att_ctrl.control[i];
			}

			mavlink_msg_actuator_control_target_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};


//TODO: this is deprecated (09.2016). Remove it some time in the future...
class MavlinkStreamHILControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHILControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIL_CONTROLS";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIL_CONTROLS;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHILControls(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIL_CONTROLS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	uint64_t _status_time;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamHILControls(MavlinkStreamHILControls &);
	MavlinkStreamHILControls &operator = (const MavlinkStreamHILControls &);

protected:
	explicit MavlinkStreamHILControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_status_time(0),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs))),
		_act_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct actuator_outputs_s act;

		bool updated = _act_sub->update(&_act_time, &act);
		updated |= _status_sub->update(&_status_time, &status);

		if (updated && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state;
			uint8_t mavlink_base_mode;
			uint32_t mavlink_custom_mode;
			get_mavlink_mode_state(&status, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			float out[8];

			const float pwm_center = (PWM_DEFAULT_MAX + PWM_DEFAULT_MIN) / 2;

			unsigned system_type = _mavlink->get_system_type();

			/* scale outputs depending on system type */
			if (system_type == MAV_TYPE_QUADROTOR ||
			    system_type == MAV_TYPE_HEXAROTOR ||
			    system_type == MAV_TYPE_OCTOROTOR ||
			    system_type == MAV_TYPE_VTOL_DUOROTOR ||
			    system_type == MAV_TYPE_VTOL_QUADROTOR) {

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

				default:
					n = 8;
					break;
				}

				for (unsigned i = 0; i < 8; i++) {
					if (act.output[i] > PWM_DEFAULT_MIN / 2) {
						if (i < n) {
							/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for rotors */
							out[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);

						} else {
							/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for other channels */
							out[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);
						}

					} else {
						/* send 0 when disarmed and for disabled channels */
						out[i] = 0.0f;
					}
				}

			} else {
				/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

				for (unsigned i = 0; i < 8; i++) {
					if (act.output[i] > PWM_DEFAULT_MIN / 2) {
						if (i != 3) {
							/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to -1..1 for normal channels */
							out[i] = (act.output[i] - pwm_center) / ((PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) / 2);

						} else {
							/* scale PWM out PWM_DEFAULT_MIN..PWM_DEFAULT_MAX us to 0..1 for throttle */
							out[i] = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
						}

					} else {
						/* set 0 for disabled channels */
						out[i] = 0.0f;
					}
				}
			}

			mavlink_hil_controls_t msg = {};

			msg.time_usec = hrt_absolute_time();
			msg.roll_ailerons = out[0];
			msg.pitch_elevator = out[1];
			msg.yaw_rudder = out[2];
			msg.throttle = out[3];
			msg.aux1 = out[4];
			msg.aux2 = out[5];
			msg.aux3 = out[6];
			msg.aux4 = out[7];
			msg.mode = mavlink_base_mode;
			msg.nav_mode = 0;

			mavlink_msg_hil_controls_send_struct(_mavlink->get_channel(), &msg);
		}
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
	uint64_t _status_time;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamHILActuatorControls(MavlinkStreamHILActuatorControls &);
	MavlinkStreamHILActuatorControls &operator = (const MavlinkStreamHILActuatorControls &);

protected:
	explicit MavlinkStreamHILActuatorControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_status_time(0),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs))),
		_act_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct actuator_outputs_s act;

		bool updated = _act_sub->update(&_act_time, &act);
		updated |= _status_sub->update(&_status_time, &status);

		if (updated && (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
			/* translate the current syste state to mavlink state and mode */
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
		}
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
	MavlinkOrbSubscription *_pos_sp_triplet_sub;

	/* do not allow top copying this class */
	MavlinkStreamPositionTargetGlobalInt(MavlinkStreamPositionTargetGlobalInt &);
	MavlinkStreamPositionTargetGlobalInt &operator = (const MavlinkStreamPositionTargetGlobalInt &);

protected:
	explicit MavlinkStreamPositionTargetGlobalInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet)))
	{}

	void send(const hrt_abstime t)
	{
		struct position_setpoint_triplet_s pos_sp_triplet;

		if (_pos_sp_triplet_sub->update(&pos_sp_triplet)) {
			mavlink_position_target_global_int_t msg = {};

			msg.time_boot_ms = hrt_absolute_time() / 1000;
			msg.coordinate_frame = MAV_FRAME_GLOBAL;
			msg.lat_int = pos_sp_triplet.current.lat * 1e7;
			msg.lon_int = pos_sp_triplet.current.lon * 1e7;
			msg.alt = pos_sp_triplet.current.alt;

			mavlink_msg_position_target_global_int_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamLocalPositionSetpoint(MavlinkStreamLocalPositionSetpoint &);
	MavlinkStreamLocalPositionSetpoint &operator = (const MavlinkStreamLocalPositionSetpoint &);

protected:
	explicit MavlinkStreamLocalPositionSetpoint(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint))),
		_pos_sp_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub->update(&_pos_sp_time, &pos_sp)) {
			mavlink_position_target_local_ned_t msg = {};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;
			msg.yaw = pos_sp.yaw;
			msg.vx = pos_sp.vx;
			msg.vy = pos_sp.vy;
			msg.vz = pos_sp.vz;
			msg.afx = pos_sp.acc_x;
			msg.afy = pos_sp.acc_y;
			msg.afz = pos_sp.acc_z;

			mavlink_msg_position_target_local_ned_send_struct(_mavlink->get_channel(), &msg);
		}
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
	uint64_t _att_rates_sp_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitudeTarget(MavlinkStreamAttitudeTarget &);
	MavlinkStreamAttitudeTarget &operator = (const MavlinkStreamAttitudeTarget &);

protected:
	explicit MavlinkStreamAttitudeTarget(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint))),
		_att_rates_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_rates_setpoint))),
		_att_sp_time(0),
		_att_rates_sp_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_setpoint_s att_sp = {};

		if (_att_sp_sub->update(&_att_sp_time, &att_sp)) {

			struct vehicle_rates_setpoint_s att_rates_sp = {};
			(void)_att_rates_sp_sub->update(&_att_rates_sp_time, &att_rates_sp);

			mavlink_attitude_target_t msg = {};

			msg.time_boot_ms = att_sp.timestamp / 1000;

			if (att_sp.q_d_valid) {
				memcpy(&msg.q[0], &att_sp.q_d[0], sizeof(msg.q));

			} else {
				mavlink_euler_to_quaternion(att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body, msg.q);
			}

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = att_sp.thrust;

			mavlink_msg_attitude_target_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamRCChannels(MavlinkStreamRCChannels &);
	MavlinkStreamRCChannels &operator = (const MavlinkStreamRCChannels &);

protected:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink),
		_rc_sub(_mavlink->add_orb_subscription(ORB_ID(input_rc))),
		_rc_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct rc_input_values rc = {};

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
		}
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
	MavlinkStreamManualControl(MavlinkStreamManualControl &);
	MavlinkStreamManualControl &operator = (const MavlinkStreamManualControl &);

protected:
	explicit MavlinkStreamManualControl(Mavlink *mavlink) : MavlinkStream(mavlink),
		_manual_sub(_mavlink->add_orb_subscription(ORB_ID(manual_control_setpoint))),
		_manual_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct manual_control_setpoint_s manual = {};

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
		}
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
	MavlinkStreamOpticalFlowRad(MavlinkStreamOpticalFlowRad &);
	MavlinkStreamOpticalFlowRad &operator = (const MavlinkStreamOpticalFlowRad &);

protected:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink),
		_flow_sub(_mavlink->add_orb_subscription(ORB_ID(optical_flow))),
		_flow_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct optical_flow_s flow = {};

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
		}
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
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &);
	MavlinkStreamNamedValueFloat &operator = (const MavlinkStreamNamedValueFloat &);

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_key_value))),
		_debug_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct debug_key_value_s debug = {};

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_named_value_float_t msg = {};

			msg.time_boot_ms = debug.timestamp_ms;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);
		}
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
		return (_fw_pos_ctrl_status_sub->is_published()) ?
		       MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	MavlinkOrbSubscription *_fw_pos_ctrl_status_sub;
	MavlinkOrbSubscription *_tecs_status_sub;

	/* do not allow top copying this class */
	MavlinkStreamNavControllerOutput(MavlinkStreamNavControllerOutput &);
	MavlinkStreamNavControllerOutput &operator = (const MavlinkStreamNavControllerOutput &);

protected:
	explicit MavlinkStreamNavControllerOutput(Mavlink *mavlink) : MavlinkStream(mavlink),
		_fw_pos_ctrl_status_sub(_mavlink->add_orb_subscription(ORB_ID(fw_pos_ctrl_status))),
		_tecs_status_sub(_mavlink->add_orb_subscription(ORB_ID(tecs_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct fw_pos_ctrl_status_s _fw_pos_ctrl_status = {};
		struct tecs_status_s _tecs_status = {};

		const bool updated_fw_pos_ctrl_status = _fw_pos_ctrl_status_sub->update(&_fw_pos_ctrl_status);
		const bool updated_tecs = _tecs_status_sub->update(&_tecs_status);

		if (updated_fw_pos_ctrl_status || updated_tecs) {
			mavlink_nav_controller_output_t msg = {};

			msg.nav_roll = math::degrees(_fw_pos_ctrl_status.nav_roll);
			msg.nav_pitch = math::degrees(_fw_pos_ctrl_status.nav_pitch);
			msg.nav_bearing = (int16_t)math::degrees(_fw_pos_ctrl_status.nav_bearing);
			msg.target_bearing = (int16_t)math::degrees(_fw_pos_ctrl_status.target_bearing);
			msg.wp_dist = (uint16_t)_fw_pos_ctrl_status.wp_dist;
			msg.xtrack_error = _fw_pos_ctrl_status.xtrack_error;
			msg.alt_error = _tecs_status.altitude_filtered - _tecs_status.altitudeSp;
			msg.aspd_error = _tecs_status.airspeed_filtered - _tecs_status.airspeedSp;

			mavlink_msg_nav_controller_output_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamCameraCapture(MavlinkStreamCameraCapture &);
	MavlinkStreamCameraCapture &operator = (const MavlinkStreamCameraCapture &);

protected:
	explicit MavlinkStreamCameraCapture(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};
		(void)_status_sub->update(&status);

		mavlink_command_long_t msg = {};

		msg.target_system = 0;
		msg.target_component = MAV_COMP_ID_ALL;
		msg.command = MAV_CMD_DO_CONTROL_VIDEO;
		msg.confirmation = 0;
		msg.param1 = 0;
		msg.param2 = 0;
		msg.param3 = 0;
		/* set camera capture ON/OFF depending on arming state */
		msg.param4 = (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED
			      || status.arming_state == vehicle_status_s::ARMING_STATE_ARMED_ERROR) ? 1 : 0;
		msg.param5 = 0;
		msg.param6 = 0;
		msg.param7 = 0;

		mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &msg);
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
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &);
	MavlinkStreamDistanceSensor &operator = (const MavlinkStreamDistanceSensor &);

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink),
		_distance_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(distance_sensor))),
		_dist_sensor_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct distance_sensor_s dist_sensor = {};

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
		}
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
	mavlink_extended_sys_state_t _msg;

	/* do not allow top copying this class */
	MavlinkStreamExtendedSysState(MavlinkStreamExtendedSysState &);
	MavlinkStreamExtendedSysState &operator = (const MavlinkStreamExtendedSysState &);

protected:
	explicit MavlinkStreamExtendedSysState(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_landed_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_land_detected))),
		_msg()
	{

		_msg.vtol_state = MAV_VTOL_STATE_UNDEFINED;
		_msg.landed_state = MAV_LANDED_STATE_UNDEFINED;
	}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status = {};
		struct vehicle_land_detected_s land_detected = {};
		bool updated = false;

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

		if (_landed_sub->update(&land_detected)) {
			updated = true;

			if (land_detected.landed) {
				_msg.landed_state = MAV_LANDED_STATE_ON_GROUND;

			} else {
				_msg.landed_state = MAV_LANDED_STATE_IN_AIR;
			}
		}

		if (updated) {
			mavlink_msg_extended_sys_state_send_struct(_mavlink->get_channel(), &_msg);
		}
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
	MavlinkOrbSubscription *_global_pos_sub;
	uint64_t _global_pos_time;

	MavlinkOrbSubscription *_local_pos_sub;
	uint64_t _local_pos_time;

	MavlinkOrbSubscription *_home_sub;
	uint64_t _home_time;

	MavlinkOrbSubscription *_sensor_sub;
	uint64_t _sensor_time;

	/* do not allow top copying this class */
	MavlinkStreamAltitude(MavlinkStreamAltitude &);
	MavlinkStreamAltitude &operator = (const MavlinkStreamAltitude &);

protected:
	explicit MavlinkStreamAltitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_global_pos_time(0),
		_local_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_local_pos_time(0),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_home_time(0),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0)
	{}

	void send(const hrt_abstime t)
	{
		mavlink_altitude_t msg = {};
		bool updated = false;
		float global_alt = 0.0f;

		{
			struct vehicle_global_position_s global_pos;
			updated |= _global_pos_sub->update(&_global_pos_time, &global_pos);

			if (_global_pos_time != 0) {
				msg.altitude_amsl = global_pos.alt;
				global_alt = global_pos.alt;

			} else {
				msg.altitude_amsl = NAN;
			}

			if (_global_pos_time != 0 && global_pos.terrain_alt_valid) {
				msg.altitude_terrain = global_pos.terrain_alt;
				msg.bottom_clearance = global_pos.alt - global_pos.terrain_alt;

			} else {
				msg.altitude_terrain = NAN;
				msg.bottom_clearance = NAN;
			}
		}

		{
			struct vehicle_local_position_s local_pos;
			updated |= _local_pos_sub->update(&_local_pos_time, &local_pos);
			msg.altitude_local = (_local_pos_time > 0) ? -local_pos.z : NAN;

			// publish this data if global isn't publishing
			if (_global_pos_time == 0) {
				if (local_pos.dist_bottom_valid) {
					msg.bottom_clearance = local_pos.dist_bottom;
					msg.altitude_terrain = msg.altitude_local - msg.bottom_clearance;

				} else {
					msg.bottom_clearance = NAN;
					msg.altitude_terrain = NAN;
				}
			}
		}

		{
			struct home_position_s home = {};
			updated |= _home_sub->update(&_home_time, &home);

			if (_global_pos_time > 0 && _home_time > 0) {
				msg.altitude_relative = global_alt - home.alt;

			} else if (_local_pos_time > 0 && _home_time > 0) {
				msg.altitude_relative = msg.altitude_local;

			} else {
				msg.altitude_relative = NAN;
			}
		}

		if (updated) {

			msg.time_usec = hrt_absolute_time();

			{
				struct sensor_combined_s sensor = {};
				(void)_sensor_sub->update(&_sensor_time, &sensor);
				msg.altitude_monotonic = (_sensor_time > 0) ? sensor.baro_alt_meter : NAN;
			}

			mavlink_msg_altitude_send_struct(_mavlink->get_channel(), &msg);
		}
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

	MavlinkOrbSubscription *_global_pos_sub;
	uint64_t _global_pos_time;

	/* do not allow top copying this class */
	MavlinkStreamWind(MavlinkStreamWind &);
	MavlinkStreamWind &operator = (const MavlinkStreamWind &);

protected:
	explicit MavlinkStreamWind(Mavlink *mavlink) : MavlinkStream(mavlink),
		_wind_estimate_sub(_mavlink->add_orb_subscription(ORB_ID(wind_estimate))),
		_wind_estimate_time(0),
		_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_global_pos_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct wind_estimate_s wind_estimate = {};

		bool updated = _wind_estimate_sub->update(&_wind_estimate_time, &wind_estimate);

		if (updated) {

			mavlink_wind_cov_t msg = {};

			msg.time_usec = wind_estimate.timestamp;

			msg.wind_x = wind_estimate.windspeed_north;
			msg.wind_y = wind_estimate.windspeed_east;
			msg.wind_z = 0.0f;

			msg.var_horiz = wind_estimate.covariance_north + wind_estimate.covariance_east;
			msg.var_vert = 0.0f;

			struct vehicle_global_position_s global_pos = {};
			_global_pos_sub->update(&_global_pos_time, &global_pos);
			msg.wind_alt = (_global_pos_time > 0) ? global_pos.alt : NAN;


			msg.horiz_accuracy = 0.0f;
			msg.vert_accuracy = 0.0f;

			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);
		}
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
	MavlinkStreamMountOrientation(MavlinkStreamMountOrientation &);
	MavlinkStreamMountOrientation &operator = (const MavlinkStreamMountOrientation &);

protected:
	explicit MavlinkStreamMountOrientation(Mavlink *mavlink) : MavlinkStream(mavlink),
		_mount_orientation_sub(_mavlink->add_orb_subscription(ORB_ID(mount_orientation))),
		_mount_orientation_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct mount_orientation_s mount_orientation = {};

		bool updated = _mount_orientation_sub->update(&_mount_orientation_time, &mount_orientation);

		if (updated) {

			mavlink_mount_orientation_t msg = {};

			msg.roll = 180.0f / M_PI_F * mount_orientation.attitude_euler_angle[0];
			msg.pitch = 180.0f / M_PI_F * mount_orientation.attitude_euler_angle[1];
			msg.yaw = 180.0f / M_PI_F * mount_orientation.attitude_euler_angle[2];

			mavlink_msg_mount_orientation_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

class MavlinkStreamHighLatency : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamHighLatency::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGH_LATENCY";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighLatency(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_actuator_sub;
	uint64_t _actuator_time;

	MavlinkOrbSubscription *_airspeed_sub;
	uint64_t _airspeed_time;

	MavlinkOrbSubscription *_attitude_sp_sub;
	uint64_t _attitude_sp_time;

	MavlinkOrbSubscription *_attitude_sub;
	uint64_t _attitude_time;

	MavlinkOrbSubscription *_battery_sub;
	uint64_t _battery_time;

	MavlinkOrbSubscription *_fw_pos_ctrl_status_sub;
	uint64_t _fw_pos_ctrl_status_time;

	MavlinkOrbSubscription *_global_pos_sub;
	uint64_t _global_pos_time;

	MavlinkOrbSubscription *_gps_sub;
	uint64_t _gps_time;

	MavlinkOrbSubscription *_home_sub;
	uint64_t _home_time;

	MavlinkOrbSubscription *_landed_sub;
	uint64_t _landed_time;

	MavlinkOrbSubscription *_mission_result_sub;
	uint64_t _mission_result_time;

	MavlinkOrbSubscription *_sensor_sub;
	uint64_t _sensor_time;

	MavlinkOrbSubscription *_status_sub;
	uint64_t _status_time;

	MavlinkOrbSubscription *_tecs_status_sub;
	uint64_t _tecs_time;

	MavlinkOrbSubscription *_wind_sub;
	uint64_t _wind_time;

	/* do not allow top copying this class */
	MavlinkStreamHighLatency(MavlinkStreamHighLatency &);
	MavlinkStreamHighLatency &operator = (const MavlinkStreamHighLatency &);

protected:
	explicit MavlinkStreamHighLatency(Mavlink *mavlink) : MavlinkStream(mavlink),
		_actuator_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_controls_0))),
		_actuator_time(0),
		_airspeed_sub(_mavlink->add_orb_subscription(ORB_ID(airspeed))),
		_airspeed_time(0),
		_attitude_sp_sub(_mavlink->add_orb_subscription(ORB_ID(fw_pos_ctrl_status))),
		_attitude_sp_time(0),
		_attitude_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_attitude_time(0),
		_battery_sub(_mavlink->add_orb_subscription(ORB_ID(battery_status))),
		_battery_time(0),
		_fw_pos_ctrl_status_sub(_mavlink->add_orb_subscription(ORB_ID(fw_pos_ctrl_status))),
		_fw_pos_ctrl_status_time(0),
		_global_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_global_pos_time(0),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
		_gps_time(0),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_home_time(0),
		_landed_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_land_detected))),
		_landed_time(0),
		_mission_result_sub(_mavlink->add_orb_subscription(ORB_ID(mission_result))),
		_mission_result_time(0),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_status_time(0),
		_tecs_status_sub(_mavlink->add_orb_subscription(ORB_ID(tecs_status))),
		_tecs_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct actuator_controls_s actuator = {};
		struct airspeed_s airspeed = {};
		struct battery_status_s battery = {};
		struct fw_pos_ctrl_status_s fw_pos_ctrl_status = {};
		struct home_position_s home = {};
		struct mission_result_s mission_result = {};
		struct sensor_combined_s sensor = {};
		struct tecs_status_s tecs_status = {};
		struct vehicle_attitude_s attitude = {};
		struct vehicle_attitude_setpoint_s attitude_sp = {};
		struct vehicle_global_position_s global_pos = {};
		struct vehicle_gps_position_s gps = {};
		struct vehicle_land_detected_s land_detected = {};
		struct vehicle_status_s status = {};

		bool updated = _status_sub->update(&_status_time, &status);
		updated |= _actuator_sub->update(&_actuator_time, &actuator);
		updated |= _airspeed_sub->update(&_airspeed_time, &airspeed);
		updated |= _attitude_sp_sub->update(&_attitude_sp_time, &attitude_sp);
		updated |= _attitude_sub->update(&_attitude_time, &attitude);
		updated |= _battery_sub->update(&_battery_time, &battery);
		updated |= _fw_pos_ctrl_status_sub->update(&_fw_pos_ctrl_status_time, &fw_pos_ctrl_status);
		updated |= _global_pos_sub->update(&_global_pos_time, &global_pos);
		updated |= _gps_sub->update(&_gps_time, &gps);
		updated |= _home_sub->update(&_home_time, &home);
		updated |= _landed_sub->update(&_landed_time, &land_detected);
		updated |= _mission_result_sub->update(&_mission_result_time, &mission_result);
		updated |= _sensor_sub->update(&_sensor_time, &sensor);
		updated |= _tecs_status_sub->update(&_tecs_time, &tecs_status);

		if (updated) {
			mavlink_high_latency_t msg = {};

			//timespec tv;
			//px4_clock_gettime(CLOCK_REALTIME, &tv);
			//msg.time_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

			msg.base_mode = 0;
			msg.custom_mode = 0;
			uint8_t sys_status;
			get_mavlink_mode_state(&status, &sys_status, &msg.base_mode, &msg.custom_mode);

			matrix::Eulerf euler = matrix::Quatf(attitude.q);
			msg.roll = math::degrees(euler.phi()) * 100;
			msg.pitch = math::degrees(euler.theta()) * 100;
			msg.heading = math::degrees(_wrap_2pi(euler.psi())) * 100;

			//msg.roll_sp = math::degrees(attitude_sp.roll_body) * 100;
			//msg.pitch_sp = math::degrees(attitude_sp.pitch_body) * 100;
			msg.heading_sp = math::degrees(_wrap_2pi(attitude_sp.yaw_body)) * 100;

			if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				msg.throttle = actuator.control[actuator_controls_s::INDEX_THROTTLE] * 100;

			} else {
				msg.throttle = 0;
			}

			msg.latitude = global_pos.lat * 1e7;
			msg.longitude = global_pos.lon * 1e7;

			//msg.altitude_home = (_home_time > 0) ? (global_pos.alt - home.alt) : NAN;
			msg.altitude_amsl = (_global_pos_time > 0) ? global_pos.alt : NAN;

			msg.altitude_sp = (_tecs_time > 0) ? (tecs_status.altitudeSp - home.alt) : NAN;

			msg.airspeed = airspeed.indicated_airspeed_m_s * 100.0f;
			msg.groundspeed = sqrtf(global_pos.vel_n * global_pos.vel_n + global_pos.vel_e * global_pos.vel_e) * 100.0f;
			msg.climb_rate = -global_pos.vel_d;

			msg.gps_nsat = gps.satellites_used;
			msg.gps_fix_type = gps.fix_type;

			msg.landed_state = land_detected.landed ? MAV_LANDED_STATE_ON_GROUND : MAV_LANDED_STATE_IN_AIR;

			msg.battery_remaining = (battery.connected) ? battery.remaining * 100.0f : -1;

			msg.temperature = sensor.baro_temp_celcius;
			msg.temperature_air = airspeed.air_temperature_celsius;

			msg.wp_num = mission_result.seq_current;
			msg.wp_distance = fw_pos_ctrl_status.wp_dist;

			mavlink_msg_high_latency_send_struct(_mavlink->get_channel(), &msg);
		}
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
	struct vehicle_attitude_s _att;
	struct vehicle_global_position_s _gpos;

	/* do not allow top copying this class */
	MavlinkStreamGroundTruth(MavlinkStreamGroundTruth &);
	MavlinkStreamGroundTruth &operator = (const MavlinkStreamGroundTruth &);

protected:
	explicit MavlinkStreamGroundTruth(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_groundtruth))),
		_gpos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position_groundtruth))),
		_att_time(0),
		_gpos_time(0),
		_att(),
		_gpos()
	{}

	void send(const hrt_abstime t)
	{
		bool att_updated = _att_sub->update(&_att_time, &_att);
		bool gpos_updated = _gpos_sub->update(&_gpos_time, &_gpos);

		if (att_updated || gpos_updated) {

			mavlink_hil_state_quaternion_t msg = {};

			if (att_updated) {
				msg.attitude_quaternion[0] = _att.q[0];
				msg.attitude_quaternion[1] = _att.q[1];
				msg.attitude_quaternion[2] = _att.q[2];
				msg.attitude_quaternion[3] = _att.q[3];
				msg.rollspeed = _att.rollspeed;
				msg.pitchspeed = _att.pitchspeed;
				msg.yawspeed = _att.yawspeed;
			}

			if (gpos_updated) {
				msg.lat = _gpos.lat;
				msg.lon = _gpos.lon;
				msg.alt = _gpos.alt;
				msg.vx = _gpos.vel_n;
				msg.vy = _gpos.vel_e;
				msg.vz = _gpos.vel_d;
				msg.ind_airspeed = 0;
				msg.true_airspeed = 0;
				msg.xacc = 0;
				msg.yacc = 0;
				msg.zacc = 0;
			}

			mavlink_msg_hil_state_quaternion_send_struct(_mavlink->get_channel(), &msg);
		}
	}
};

const StreamListItem *streams_list[] = {
	new StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static, &MavlinkStreamHeartbeat::get_id_static),
	new StreamListItem(&MavlinkStreamStatustext::new_instance, &MavlinkStreamStatustext::get_name_static, &MavlinkStreamStatustext::get_id_static),
	new StreamListItem(&MavlinkStreamCommandLong::new_instance, &MavlinkStreamCommandLong::get_name_static, &MavlinkStreamCommandLong::get_id_static),
	new StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static, &MavlinkStreamSysStatus::get_id_static),
	new StreamListItem(&MavlinkStreamHighresIMU::new_instance, &MavlinkStreamHighresIMU::get_name_static, &MavlinkStreamHighresIMU::get_id_static),
	new StreamListItem(&MavlinkStreamAttitude::new_instance, &MavlinkStreamAttitude::get_name_static, &MavlinkStreamAttitude::get_id_static),
	new StreamListItem(&MavlinkStreamAttitudeQuaternion::new_instance, &MavlinkStreamAttitudeQuaternion::get_name_static, &MavlinkStreamAttitudeQuaternion::get_id_static),
	new StreamListItem(&MavlinkStreamVFRHUD::new_instance, &MavlinkStreamVFRHUD::get_name_static, &MavlinkStreamVFRHUD::get_id_static),
	new StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static, &MavlinkStreamGPSRawInt::get_id_static),
	new StreamListItem(&MavlinkStreamSystemTime::new_instance, &MavlinkStreamSystemTime::get_name_static, &MavlinkStreamSystemTime::get_id_static),
	new StreamListItem(&MavlinkStreamTimesync::new_instance, &MavlinkStreamTimesync::get_name_static, &MavlinkStreamTimesync::get_id_static),
	new StreamListItem(&MavlinkStreamGlobalPositionInt::new_instance, &MavlinkStreamGlobalPositionInt::get_name_static, &MavlinkStreamGlobalPositionInt::get_id_static),
	new StreamListItem(&MavlinkStreamLocalPositionNED::new_instance, &MavlinkStreamLocalPositionNED::get_name_static, &MavlinkStreamLocalPositionNED::get_id_static),
	new StreamListItem(&MavlinkStreamVisionPositionEstimate::new_instance, &MavlinkStreamVisionPositionEstimate::get_name_static, &MavlinkStreamVisionPositionEstimate::get_id_static),
	new StreamListItem(&MavlinkStreamLocalPositionNEDCOV::new_instance, &MavlinkStreamLocalPositionNEDCOV::get_name_static, &MavlinkStreamLocalPositionNEDCOV::get_id_static),
	new StreamListItem(&MavlinkStreamEstimatorStatus::new_instance, &MavlinkStreamEstimatorStatus::get_name_static, &MavlinkStreamEstimatorStatus::get_id_static),
	new StreamListItem(&MavlinkStreamAttPosMocap::new_instance, &MavlinkStreamAttPosMocap::get_name_static, &MavlinkStreamAttPosMocap::get_id_static),
	new StreamListItem(&MavlinkStreamHomePosition::new_instance, &MavlinkStreamHomePosition::get_name_static, &MavlinkStreamHomePosition::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<0>::new_instance, &MavlinkStreamServoOutputRaw<0>::get_name_static, &MavlinkStreamServoOutputRaw<0>::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<1>::new_instance, &MavlinkStreamServoOutputRaw<1>::get_name_static, &MavlinkStreamServoOutputRaw<1>::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<2>::new_instance, &MavlinkStreamServoOutputRaw<2>::get_name_static, &MavlinkStreamServoOutputRaw<2>::get_id_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<3>::new_instance, &MavlinkStreamServoOutputRaw<3>::get_name_static, &MavlinkStreamServoOutputRaw<3>::get_id_static),
	new StreamListItem(&MavlinkStreamHILControls::new_instance, &MavlinkStreamHILControls::get_name_static, &MavlinkStreamHILControls::get_id_static),
	new StreamListItem(&MavlinkStreamHILActuatorControls::new_instance, &MavlinkStreamHILActuatorControls::get_name_static, &MavlinkStreamHILActuatorControls::get_id_static),
	new StreamListItem(&MavlinkStreamPositionTargetGlobalInt::new_instance, &MavlinkStreamPositionTargetGlobalInt::get_name_static, &MavlinkStreamPositionTargetGlobalInt::get_id_static),
	new StreamListItem(&MavlinkStreamLocalPositionSetpoint::new_instance, &MavlinkStreamLocalPositionSetpoint::get_name_static, &MavlinkStreamLocalPositionSetpoint::get_id_static),
	new StreamListItem(&MavlinkStreamAttitudeTarget::new_instance, &MavlinkStreamAttitudeTarget::get_name_static, &MavlinkStreamAttitudeTarget::get_id_static),
	new StreamListItem(&MavlinkStreamRCChannels::new_instance, &MavlinkStreamRCChannels::get_name_static, &MavlinkStreamRCChannels::get_id_static),
	new StreamListItem(&MavlinkStreamManualControl::new_instance, &MavlinkStreamManualControl::get_name_static, &MavlinkStreamManualControl::get_id_static),
	new StreamListItem(&MavlinkStreamOpticalFlowRad::new_instance, &MavlinkStreamOpticalFlowRad::get_name_static, &MavlinkStreamOpticalFlowRad::get_id_static),
	new StreamListItem(&MavlinkStreamActuatorControlTarget<0>::new_instance, &MavlinkStreamActuatorControlTarget<0>::get_name_static, &MavlinkStreamActuatorControlTarget<0>::get_id_static),
	new StreamListItem(&MavlinkStreamActuatorControlTarget<1>::new_instance, &MavlinkStreamActuatorControlTarget<1>::get_name_static, &MavlinkStreamActuatorControlTarget<1>::get_id_static),
	new StreamListItem(&MavlinkStreamActuatorControlTarget<2>::new_instance, &MavlinkStreamActuatorControlTarget<2>::get_name_static, &MavlinkStreamActuatorControlTarget<2>::get_id_static),
	new StreamListItem(&MavlinkStreamActuatorControlTarget<3>::new_instance, &MavlinkStreamActuatorControlTarget<3>::get_name_static, &MavlinkStreamActuatorControlTarget<3>::get_id_static),
	new StreamListItem(&MavlinkStreamNamedValueFloat::new_instance, &MavlinkStreamNamedValueFloat::get_name_static, &MavlinkStreamNamedValueFloat::get_id_static),
	new StreamListItem(&MavlinkStreamNavControllerOutput::new_instance, &MavlinkStreamNavControllerOutput::get_name_static, &MavlinkStreamNavControllerOutput::get_id_static),
	new StreamListItem(&MavlinkStreamCameraCapture::new_instance, &MavlinkStreamCameraCapture::get_name_static, &MavlinkStreamCameraCapture::get_id_static),
	new StreamListItem(&MavlinkStreamCameraTrigger::new_instance, &MavlinkStreamCameraTrigger::get_name_static, &MavlinkStreamCameraTrigger::get_id_static),
	new StreamListItem(&MavlinkStreamDistanceSensor::new_instance, &MavlinkStreamDistanceSensor::get_name_static, &MavlinkStreamDistanceSensor::get_id_static),
	new StreamListItem(&MavlinkStreamExtendedSysState::new_instance, &MavlinkStreamExtendedSysState::get_name_static, &MavlinkStreamExtendedSysState::get_id_static),
	new StreamListItem(&MavlinkStreamAltitude::new_instance, &MavlinkStreamAltitude::get_name_static, &MavlinkStreamAltitude::get_id_static),
	new StreamListItem(&MavlinkStreamADSBVehicle::new_instance, &MavlinkStreamADSBVehicle::get_name_static, &MavlinkStreamADSBVehicle::get_id_static),
	new StreamListItem(&MavlinkStreamCollision::new_instance, &MavlinkStreamCollision::get_name_static, &MavlinkStreamCollision::get_id_static),
	new StreamListItem(&MavlinkStreamWind::new_instance, &MavlinkStreamWind::get_name_static, &MavlinkStreamWind::get_id_static),
	new StreamListItem(&MavlinkStreamMountOrientation::new_instance, &MavlinkStreamMountOrientation::get_name_static, &MavlinkStreamMountOrientation::get_id_static),
	new StreamListItem(&MavlinkStreamHighLatency::new_instance, &MavlinkStreamHighLatency::get_name_static, &MavlinkStreamWind::get_id_static),
	new StreamListItem(&MavlinkStreamGroundTruth::new_instance, &MavlinkStreamGroundTruth::get_name_static, &MavlinkStreamGroundTruth::get_id_static),
	nullptr
};
