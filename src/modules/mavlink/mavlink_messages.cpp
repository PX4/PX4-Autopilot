/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>

#include <commander/px4_custom_mode.h>
#include <lib/geo/geo.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/navigation_capabilities.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_range_finder.h>
#include <systemlib/err.h>
#include <mavlink/mavlink_log.h>

#include "mavlink_messages.h"
#include "mavlink_main.h"

static uint16_t cm_uint16_from_m_float(float m);
static void get_mavlink_mode_state(struct vehicle_status_s *status, struct position_setpoint_triplet_s *pos_sp_triplet,
				   uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode);

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

void get_mavlink_mode_state(struct vehicle_status_s *status, struct position_setpoint_triplet_s *pos_sp_triplet,
			    uint8_t *mavlink_state, uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	/* HIL */
	if (status->hil_state == HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == ARMING_STATE_ARMED
	    || status->arming_state == ARMING_STATE_ARMED_ERROR) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	union px4_custom_mode custom_mode;
	custom_mode.data = 0;

	switch (status->nav_state) {

		case NAVIGATION_STATE_MANUAL:
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
			                      | (status->is_rotary_wing ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
			break;

		case NAVIGATION_STATE_ACRO:
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
			break;

		case NAVIGATION_STATE_ALTCTL:
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
			break;

		case NAVIGATION_STATE_POSCTL:
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
			break;

		case NAVIGATION_STATE_AUTO_MISSION:
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
			break;

		case NAVIGATION_STATE_AUTO_LOITER:
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
			break;

		case NAVIGATION_STATE_AUTO_RTL:
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
			break;

		case NAVIGATION_STATE_LAND:
			/* fallthrough */
		case NAVIGATION_STATE_DESCEND:
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
			break;

		case NAVIGATION_STATE_AUTO_RTGS:
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTGS;
			break;

		case NAVIGATION_STATE_TERMINATION:
			*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
			break;

		case NAVIGATION_STATE_OFFBOARD:
			*mavlink_base_mode |= MAV_MODE_FLAG_AUTO_ENABLED
			                      | MAV_MODE_FLAG_STABILIZE_ENABLED
					      | MAV_MODE_FLAG_GUIDED_ENABLED;
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
			break;

		case NAVIGATION_STATE_MAX:
			/* this is an unused case, ignore */
			break;

	}

	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == ARMING_STATE_INIT
	    || status->arming_state == ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == ARMING_STATE_ARMED_ERROR) {
		*mavlink_state = MAV_STATE_CRITICAL;

	} else if (status->arming_state == ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == ARMING_STATE_REBOOT) {
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() {
		return true;
	}

private:
	MavlinkOrbSubscription *_status_sub;
	MavlinkOrbSubscription *_pos_sp_triplet_sub;

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &);
	MavlinkStreamHeartbeat& operator = (const MavlinkStreamHeartbeat &);

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct position_setpoint_triplet_s pos_sp_triplet;

		/* always send the heartbeat, independent of the update status of the topics */
		if (!_status_sub->update(&status)) {
			/* if topic update failed fill it with defaults */
			memset(&status, 0, sizeof(status));
		}

		if (!_pos_sp_triplet_sub->update(&pos_sp_triplet)) {
			/* if topic update failed fill it with defaults */
			memset(&pos_sp_triplet, 0, sizeof(pos_sp_triplet));
		}

		mavlink_heartbeat_t msg;

		msg.base_mode = 0;
		msg.custom_mode = 0;
		get_mavlink_mode_state(&status, &pos_sp_triplet, &msg.system_status, &msg.base_mode, &msg.custom_mode);
		msg.type = mavlink_system.type;
		msg.autopilot = MAV_AUTOPILOT_PX4;
		msg.mavlink_version = 3;

		_mavlink->send_message(MAVLINK_MSG_ID_HEARTBEAT, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_STATUSTEXT;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStatustext(mavlink);
	}

	unsigned get_size() {
		return mavlink_logbuffer_is_empty(_mavlink->get_logbuffer()) ? 0 : (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamStatustext(MavlinkStreamStatustext &);
	MavlinkStreamStatustext& operator = (const MavlinkStreamStatustext &);

protected:
	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	void send(const hrt_abstime t)
	{
		if (!mavlink_logbuffer_is_empty(_mavlink->get_logbuffer())) {
			struct mavlink_logmessage logmsg;
			int lb_ret = mavlink_logbuffer_read(_mavlink->get_logbuffer(), &logmsg);

			if (lb_ret == OK) {
				mavlink_statustext_t msg;

				msg.severity = logmsg.severity;
				strncpy(msg.text, logmsg.text, sizeof(msg.text));

				_mavlink->send_message(MAVLINK_MSG_ID_STATUSTEXT, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_COMMAND_LONG;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCommandLong(mavlink);
	}

	unsigned get_size() {
		return 0;	// commands stream is not regular and not predictable
	}

private:
	MavlinkOrbSubscription *_cmd_sub;
	uint64_t _cmd_time;

	/* do not allow top copying this class */
	MavlinkStreamCommandLong(MavlinkStreamCommandLong &);
	MavlinkStreamCommandLong& operator = (const MavlinkStreamCommandLong &);

protected:
	explicit MavlinkStreamCommandLong(Mavlink *mavlink) : MavlinkStream(mavlink),
		_cmd_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_command))),
		_cmd_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_command_s cmd;

		if (_cmd_sub->update(&_cmd_time, &cmd)) {
			/* only send commands for other systems/components */
			if (cmd.target_system != mavlink_system.sysid || cmd.target_component != mavlink_system.compid) {
				mavlink_command_long_t msg;

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

				_mavlink->send_message(MAVLINK_MSG_ID_COMMAND_LONG, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_SYS_STATUS;
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

	/* do not allow top copying this class */
	MavlinkStreamSysStatus(MavlinkStreamSysStatus &);
	MavlinkStreamSysStatus& operator = (const MavlinkStreamSysStatus &);

protected:
	explicit MavlinkStreamSysStatus(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;

		if (_status_sub->update(&status)) {
			mavlink_sys_status_t msg;

			msg.onboard_control_sensors_present = status.onboard_control_sensors_present;
			msg.onboard_control_sensors_enabled = status.onboard_control_sensors_enabled;
			msg.onboard_control_sensors_health = status.onboard_control_sensors_health;
			msg.load = status.load * 1000.0f;
			msg.voltage_battery = status.battery_voltage * 1000.0f;
			msg.current_battery = status.battery_current * 100.0f;
			msg.drop_rate_comm = status.drop_rate_comm;
			msg.errors_comm = status.errors_comm;
			msg.errors_count1 = status.errors_count1;
			msg.errors_count2 = status.errors_count2;
			msg.errors_count3 = status.errors_count3;
			msg.errors_count4 = status.errors_count4;
			msg.battery_remaining = status.battery_remaining * 100.0f;

			_mavlink->send_message(MAVLINK_MSG_ID_SYS_STATUS, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
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

	uint64_t _accel_timestamp;
	uint64_t _gyro_timestamp;
	uint64_t _mag_timestamp;
	uint64_t _baro_timestamp;

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &);
	MavlinkStreamHighresIMU& operator = (const MavlinkStreamHighresIMU &);

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sensor_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_combined))),
		_sensor_time(0),
		_accel_timestamp(0),
		_gyro_timestamp(0),
		_mag_timestamp(0),
		_baro_timestamp(0)
	{}

	void send(const hrt_abstime t)
	{
		struct sensor_combined_s sensor;

		if (_sensor_sub->update(&_sensor_time, &sensor)) {
			uint16_t fields_updated = 0;

			if (_accel_timestamp != sensor.accelerometer_timestamp) {
				/* mark first three dimensions as changed */
				fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
				_accel_timestamp = sensor.accelerometer_timestamp;
			}

			if (_gyro_timestamp != sensor.timestamp) {
				/* mark second group dimensions as changed */
				fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
				_gyro_timestamp = sensor.timestamp;
			}

			if (_mag_timestamp != sensor.magnetometer_timestamp) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
				_mag_timestamp = sensor.magnetometer_timestamp;
			}

			if (_baro_timestamp != sensor.baro_timestamp) {
				/* mark last group dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
				_baro_timestamp = sensor.baro_timestamp;
			}

			mavlink_highres_imu_t msg;

			msg.time_usec = sensor.timestamp;
			msg.xacc = sensor.accelerometer_m_s2[0];
			msg.yacc = sensor.accelerometer_m_s2[1];
			msg.zacc = sensor.accelerometer_m_s2[2];
			msg.xgyro = sensor.gyro_rad_s[0];
			msg.ygyro = sensor.gyro_rad_s[1];
			msg.zgyro = sensor.gyro_rad_s[2];
			msg.xmag = sensor.magnetometer_ga[0];
			msg.ymag = sensor.magnetometer_ga[1];
			msg.zmag = sensor.magnetometer_ga[2];
			msg.abs_pressure = sensor.baro_pres_mbar;
			msg.diff_pressure = sensor.differential_pressure_pa;
			msg.pressure_alt = sensor.baro_alt_meter;
			msg.temperature = sensor.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			_mavlink->send_message(MAVLINK_MSG_ID_HIGHRES_IMU, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_ATTITUDE;
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
	MavlinkStreamAttitude& operator = (const MavlinkStreamAttitude &);


protected:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;

		if (_att_sub->update(&_att_time, &att)) {
			mavlink_attitude_t msg;

			msg.time_boot_ms = att.timestamp / 1000;
			msg.roll = att.roll;
			msg.pitch = att.pitch;
			msg.yaw = att.yaw;
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			_mavlink->send_message(MAVLINK_MSG_ID_ATTITUDE, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
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
	MavlinkStreamAttitudeQuaternion& operator = (const MavlinkStreamAttitudeQuaternion &);

protected:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude))),
		_att_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;

		if (_att_sub->update(&_att_time, &att)) {
			mavlink_attitude_quaternion_t msg;

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = att.q[0];
			msg.q2 = att.q[1];
			msg.q3 = att.q[2];
			msg.q4 = att.q[3];
			msg.rollspeed = att.rollspeed;
			msg.pitchspeed = att.pitchspeed;
			msg.yawspeed = att.yawspeed;

			_mavlink->send_message(MAVLINK_MSG_ID_ATTITUDE_QUATERNION, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_VFR_HUD;
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

	/* do not allow top copying this class */
	MavlinkStreamVFRHUD(MavlinkStreamVFRHUD &);
	MavlinkStreamVFRHUD& operator = (const MavlinkStreamVFRHUD &);

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
		_airspeed_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_s att;
		struct vehicle_global_position_s pos;
		struct actuator_armed_s armed;
		struct actuator_controls_s act;
		struct airspeed_s airspeed;

		bool updated = _att_sub->update(&_att_time, &att);
		updated |= _pos_sub->update(&_pos_time, &pos);
		updated |= _armed_sub->update(&_armed_time, &armed);
		updated |= _act_sub->update(&_act_time, &act);
		updated |= _airspeed_sub->update(&_airspeed_time, &airspeed);

		if (updated) {
			mavlink_vfr_hud_t msg;

			msg.airspeed = airspeed.true_airspeed_m_s;
			msg.groundspeed = sqrtf(pos.vel_n * pos.vel_n + pos.vel_e * pos.vel_e);
			msg.heading = _wrap_2pi(att.yaw) * M_RAD_TO_DEG_F;
			msg.throttle = armed.armed ? act.control[3] * 100.0f : 0.0f;
			msg.alt = pos.alt;
			msg.climb = -pos.vel_d;

			_mavlink->send_message(MAVLINK_MSG_ID_VFR_HUD, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_GPS_RAW_INT;
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
	MavlinkStreamGPSRawInt& operator = (const MavlinkStreamGPSRawInt &);

protected:
	explicit MavlinkStreamGPSRawInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_gps_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_gps_position))),
		_gps_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_gps_position_s gps;

		if (_gps_sub->update(&_gps_time, &gps)) {
			mavlink_gps_raw_int_t msg;

			msg.time_usec = gps.timestamp_position;
			msg.fix_type = gps.fix_type;
			msg.lat = gps.lat;
			msg.lon = gps.lon;
			msg.alt = gps.alt;
			msg.eph = cm_uint16_from_m_float(gps.eph);
			msg.epv = cm_uint16_from_m_float(gps.epv);
			msg.vel = cm_uint16_from_m_float(gps.vel_m_s),
		    msg.cog = _wrap_2pi(gps.cog_rad) * M_RAD_TO_DEG_F * 1e2f,
		    msg.satellites_visible = gps.satellites_used;

			_mavlink->send_message(MAVLINK_MSG_ID_GPS_RAW_INT, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
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
	MavlinkStreamGlobalPositionInt& operator = (const MavlinkStreamGlobalPositionInt &);

protected:
	explicit MavlinkStreamGlobalPositionInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_global_position))),
		_pos_time(0),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position))),
		_home_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_global_position_s pos;
		struct home_position_s home;

		bool updated = _pos_sub->update(&_pos_time, &pos);
		updated |= _home_sub->update(&_home_time, &home);

		if (updated) {
			mavlink_global_position_int_t msg;

			msg.time_boot_ms = pos.timestamp / 1000;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.alt = pos.alt * 1000.0f;
		    msg.relative_alt = (pos.alt - home.alt) * 1000.0f;
		    msg.vx = pos.vel_n * 100.0f;
		    msg.vy = pos.vel_e * 100.0f;
		    msg.vz = pos.vel_d * 100.0f;
		    msg.hdg = _wrap_2pi(pos.yaw) * M_RAD_TO_DEG_F * 100.0f;

			_mavlink->send_message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED;
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
	MavlinkStreamLocalPositionNED& operator = (const MavlinkStreamLocalPositionNED &);

protected:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position))),
		_pos_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_s pos;

		if (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_local_position_ned_t msg;

			msg.time_boot_ms = pos.timestamp / 1000;
			msg.x = pos.x;
			msg.y = pos.y;
			msg.z = pos.z;
		    msg.vx = pos.vx;
		    msg.vy = pos.vy;
		    msg.vz = pos.vz;

			_mavlink->send_message(MAVLINK_MSG_ID_LOCAL_POSITION_NED, &msg);
		}
	}
};


class MavlinkStreamViconPositionEstimate : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamViconPositionEstimate::get_name_static();
	}

	static const char *get_name_static()
	{
		return "VICON_POSITION_ESTIMATE";
	}

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamViconPositionEstimate(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_pos_sub;
	uint64_t _pos_time;

	/* do not allow top copying this class */
	MavlinkStreamViconPositionEstimate(MavlinkStreamViconPositionEstimate &);
	MavlinkStreamViconPositionEstimate& operator = (const MavlinkStreamViconPositionEstimate &);

protected:
	explicit MavlinkStreamViconPositionEstimate(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_vicon_position))),
		_pos_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_vicon_position_s pos;

		if (_pos_sub->update(&_pos_time, &pos)) {
			mavlink_vicon_position_estimate_t msg;

			msg.usec = pos.timestamp;
			msg.x = pos.x;
			msg.y = pos.y;
			msg.z = pos.z;
		    msg.roll = pos.roll;
		    msg.pitch = pos.pitch;
		    msg.yaw = pos.yaw;

			_mavlink->send_message(MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE, &msg);
		}
	}
};


class MavlinkStreamGPSGlobalOrigin : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamGPSGlobalOrigin::get_name_static();
	}

	static const char *get_name_static()
	{
		return "GPS_GLOBAL_ORIGIN";
	}

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamGPSGlobalOrigin(mavlink);
	}

	unsigned get_size()
	{
		return _home_sub->is_published() ? (MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_home_sub;

	/* do not allow top copying this class */
	MavlinkStreamGPSGlobalOrigin(MavlinkStreamGPSGlobalOrigin &);
	MavlinkStreamGPSGlobalOrigin& operator = (const MavlinkStreamGPSGlobalOrigin &);

protected:
	explicit MavlinkStreamGPSGlobalOrigin(Mavlink *mavlink) : MavlinkStream(mavlink),
		_home_sub(_mavlink->add_orb_subscription(ORB_ID(home_position)))
	{}

	void send(const hrt_abstime t)
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		if (_home_sub->is_published()) {
			struct home_position_s home;

			if (_home_sub->update(&home)) {
				mavlink_gps_global_origin_t msg;

				msg.latitude = home.lat * 1e7;
				msg.longitude = home.lon * 1e7;
				msg.altitude = home.alt * 1e3f;

				_mavlink->send_message(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
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
	MavlinkStreamServoOutputRaw& operator = (const MavlinkStreamServoOutputRaw &);

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_act_sub(nullptr),
		_act_time(0)
	{
		orb_id_t act_topics[] = {
			ORB_ID(actuator_outputs_0),
			ORB_ID(actuator_outputs_1),
			ORB_ID(actuator_outputs_2),
			ORB_ID(actuator_outputs_3)
		};

		_act_sub = _mavlink->add_orb_subscription(act_topics[N]);
	}

	void send(const hrt_abstime t)
	{
		struct actuator_outputs_s act;

		if (_act_sub->update(&_act_time, &act)) {
			mavlink_servo_output_raw_t msg;

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

			_mavlink->send_message(MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, &msg);
		}
	}
};


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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_HIL_CONTROLS;
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

	MavlinkOrbSubscription *_pos_sp_triplet_sub;
	uint64_t _pos_sp_triplet_time;

	MavlinkOrbSubscription *_act_sub;
	uint64_t _act_time;

	/* do not allow top copying this class */
	MavlinkStreamHILControls(MavlinkStreamHILControls &);
	MavlinkStreamHILControls& operator = (const MavlinkStreamHILControls &);

protected:
	explicit MavlinkStreamHILControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status))),
		_status_time(0),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet))),
		_pos_sp_triplet_time(0),
		_act_sub(_mavlink->add_orb_subscription(ORB_ID(actuator_outputs_0))),
		_act_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		struct position_setpoint_triplet_s pos_sp_triplet;
		struct actuator_outputs_s act;

		bool updated = _act_sub->update(&_act_time, &act);
		updated |= _pos_sp_triplet_sub->update(&_pos_sp_triplet_time, &pos_sp_triplet);
		updated |= _status_sub->update(&_status_time, &status);

		if (updated && (status.arming_state == ARMING_STATE_ARMED)) {
			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state;
			uint8_t mavlink_base_mode;
			uint32_t mavlink_custom_mode;
			get_mavlink_mode_state(&status, &pos_sp_triplet, &mavlink_state, &mavlink_base_mode, &mavlink_custom_mode);

			float out[8];

			const float pwm_center = (PWM_HIGHEST_MAX + PWM_LOWEST_MIN) / 2;

			/* scale outputs depending on system type */
			if (mavlink_system.type == MAV_TYPE_QUADROTOR ||
				mavlink_system.type == MAV_TYPE_HEXAROTOR ||
				mavlink_system.type == MAV_TYPE_OCTOROTOR) {
				/* multirotors: set number of rotor outputs depending on type */

				unsigned n;

				switch (mavlink_system.type) {
				case MAV_TYPE_QUADROTOR:
					n = 4;
					break;

				case MAV_TYPE_HEXAROTOR:
					n = 6;
					break;

				default:
					n = 8;
					break;
				}

				for (unsigned i = 0; i < 8; i++) {
					if (mavlink_base_mode & MAV_MODE_FLAG_SAFETY_ARMED) {
						if (i < n) {
							/* scale PWM out 900..2100 us to 0..1 for rotors */
							out[i] = (act.output[i] - PWM_LOWEST_MIN) / (PWM_HIGHEST_MAX - PWM_LOWEST_MIN);

						} else {
							/* scale PWM out 900..2100 us to -1..1 for other channels */
							out[i] = (act.output[i] - pwm_center) / ((PWM_HIGHEST_MAX - PWM_LOWEST_MIN) / 2);
						}

					} else {
						/* send 0 when disarmed */
						out[i] = 0.0f;
					}
				}

			} else {
				/* fixed wing: scale throttle to 0..1 and other channels to -1..1 */

				for (unsigned i = 0; i < 8; i++) {
					if (i != 3) {
						/* scale PWM out 900..2100 us to -1..1 for normal channels */
						out[i] = (act.output[i] - pwm_center) / ((PWM_HIGHEST_MAX - PWM_LOWEST_MIN) / 2);

					} else {
						/* scale PWM out 900..2100 us to 0..1 for throttle */
						out[i] = (act.output[i] - PWM_LOWEST_MIN) / (PWM_HIGHEST_MAX - PWM_LOWEST_MIN);
					}

				}
			}

			mavlink_hil_controls_t msg;

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

			_mavlink->send_message(MAVLINK_MSG_ID_HIL_CONTROLS, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;
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
	MavlinkStreamPositionTargetGlobalInt& operator = (const MavlinkStreamPositionTargetGlobalInt &);

protected:
	explicit MavlinkStreamPositionTargetGlobalInt(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sp_triplet_sub(_mavlink->add_orb_subscription(ORB_ID(position_setpoint_triplet)))
	{}

	void send(const hrt_abstime t)
	{
		struct position_setpoint_triplet_s pos_sp_triplet;

		if (_pos_sp_triplet_sub->update(&pos_sp_triplet)) {
			mavlink_position_target_global_int_t msg{};

			msg.coordinate_frame = MAV_FRAME_GLOBAL;
			msg.lat_int = pos_sp_triplet.current.lat * 1e7;
			msg.lon_int = pos_sp_triplet.current.lon * 1e7;
			msg.alt = pos_sp_triplet.current.alt;

			_mavlink->send_message(MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;
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
	MavlinkStreamLocalPositionSetpoint& operator = (const MavlinkStreamLocalPositionSetpoint &);

protected:
	explicit MavlinkStreamLocalPositionSetpoint(Mavlink *mavlink) : MavlinkStream(mavlink),
		_pos_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_local_position_setpoint))),
		_pos_sp_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub->update(&_pos_sp_time, &pos_sp)) {
			mavlink_position_target_local_ned_t msg{};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;

			_mavlink->send_message(MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET;
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
	MavlinkStreamAttitudeTarget& operator = (const MavlinkStreamAttitudeTarget &);

protected:
	explicit MavlinkStreamAttitudeTarget(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_attitude_setpoint))),
		_att_rates_sp_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_rates_setpoint))),
		_att_sp_time(0),
		_att_rates_sp_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_attitude_setpoint_s att_sp;

		if (_att_sp_sub->update(&_att_sp_time, &att_sp)) {

			struct vehicle_rates_setpoint_s att_rates_sp;
			(void)_att_rates_sp_sub->update(&_att_rates_sp_time, &att_rates_sp);

			mavlink_attitude_target_t msg{};

			msg.time_boot_ms = att_sp.timestamp / 1000;
			mavlink_euler_to_quaternion(att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body, msg.q);

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = att_sp.thrust;

			_mavlink->send_message(MAVLINK_MSG_ID_ATTITUDE_TARGET, &msg);
		}
	}
};


class MavlinkStreamRCChannelsRaw : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamRCChannelsRaw::get_name_static();
	}

	static const char *get_name_static()
	{
		return "RC_CHANNELS_RAW";
	}

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_RC_CHANNELS_RAW;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRCChannelsRaw(mavlink);
	}

	unsigned get_size()
	{
		return _rc_sub->is_published() ? ((RC_INPUT_MAX_CHANNELS / 8) * (MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) +
				MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_rc_sub;
	uint64_t _rc_time;

	/* do not allow top copying this class */
	MavlinkStreamRCChannelsRaw(MavlinkStreamRCChannelsRaw &);
	MavlinkStreamRCChannelsRaw& operator = (const MavlinkStreamRCChannelsRaw &);

protected:
	explicit MavlinkStreamRCChannelsRaw(Mavlink *mavlink) : MavlinkStream(mavlink),
		_rc_sub(_mavlink->add_orb_subscription(ORB_ID(input_rc))),
		_rc_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct rc_input_values rc;

		if (_rc_sub->update(&_rc_time, &rc)) {
			const unsigned port_width = 8;

			/* deprecated message (but still needed for compatibility!) */
			for (unsigned i = 0; (i * port_width) < rc.channel_count; i++) {
				/* channels are sent in MAVLink main loop at a fixed interval */
				mavlink_rc_channels_raw_t msg;

				msg.time_boot_ms = rc.timestamp_publication / 1000;
				msg.port = i;
				msg.chan1_raw = (rc.channel_count > (i * port_width) + 0) ? rc.values[(i * port_width) + 0] : UINT16_MAX;
				msg.chan2_raw = (rc.channel_count > (i * port_width) + 1) ? rc.values[(i * port_width) + 1] : UINT16_MAX;
				msg.chan3_raw = (rc.channel_count > (i * port_width) + 2) ? rc.values[(i * port_width) + 2] : UINT16_MAX;
				msg.chan4_raw = (rc.channel_count > (i * port_width) + 3) ? rc.values[(i * port_width) + 3] : UINT16_MAX;
				msg.chan4_raw = (rc.channel_count > (i * port_width) + 4) ? rc.values[(i * port_width) + 4] : UINT16_MAX;
				msg.chan4_raw = (rc.channel_count > (i * port_width) + 5) ? rc.values[(i * port_width) + 5] : UINT16_MAX;
				msg.chan4_raw = (rc.channel_count > (i * port_width) + 6) ? rc.values[(i * port_width) + 6] : UINT16_MAX;
				msg.chan4_raw = (rc.channel_count > (i * port_width) + 7) ? rc.values[(i * port_width) + 7] : UINT16_MAX;
				msg.rssi = rc.rssi;

				_mavlink->send_message(MAVLINK_MSG_ID_RC_CHANNELS_RAW, &msg);
			}

			/* new message */
			mavlink_rc_channels_t msg;

			msg.time_boot_ms = rc.timestamp_publication / 1000;
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
			msg.rssi = rc.rssi;

			_mavlink->send_message(MAVLINK_MSG_ID_RC_CHANNELS, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_MANUAL_CONTROL;
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
	MavlinkStreamManualControl& operator = (const MavlinkStreamManualControl &);

protected:
	explicit MavlinkStreamManualControl(Mavlink *mavlink) : MavlinkStream(mavlink),
		_manual_sub(_mavlink->add_orb_subscription(ORB_ID(manual_control_setpoint))),
		_manual_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct manual_control_setpoint_s manual;

		if (_manual_sub->update(&_manual_time, &manual)) {
			mavlink_manual_control_t msg;

			msg.target = mavlink_system.sysid;
			msg.x = manual.x * 1000;
			msg.y = manual.y * 1000;
			msg.z = manual.z * 1000;
			msg.r = manual.r * 1000;
			msg.buttons = 0;

			_mavlink->send_message(MAVLINK_MSG_ID_MANUAL_CONTROL, &msg);
		}
	}
};


class MavlinkStreamOpticalFlow : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamOpticalFlow::get_name_static();
	}

	static const char *get_name_static()
	{
		return "OPTICAL_FLOW";
	}

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_OPTICAL_FLOW;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOpticalFlow(mavlink);
	}

	unsigned get_size()
	{
		return _flow_sub->is_published() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_flow_sub;
	uint64_t _flow_time;

	/* do not allow top copying this class */
	MavlinkStreamOpticalFlow(MavlinkStreamOpticalFlow &);
	MavlinkStreamOpticalFlow& operator = (const MavlinkStreamOpticalFlow &);

protected:
	explicit MavlinkStreamOpticalFlow(Mavlink *mavlink) : MavlinkStream(mavlink),
		_flow_sub(_mavlink->add_orb_subscription(ORB_ID(optical_flow))),
		_flow_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct optical_flow_s flow;

		if (_flow_sub->update(&_flow_time, &flow)) {
			mavlink_optical_flow_t msg;

			msg.time_usec = flow.timestamp;
			msg.sensor_id = flow.sensor_id;
			msg.flow_x = flow.flow_raw_x;
			msg.flow_y = flow.flow_raw_y;
			msg.flow_comp_m_x = flow.flow_comp_x_m;
			msg.flow_comp_m_y = flow.flow_comp_y_m;
			msg.quality = flow.quality;
			msg.ground_distance = flow.ground_distance_m;

			_mavlink->send_message(MAVLINK_MSG_ID_OPTICAL_FLOW, &msg);
		}
	}
};

class MavlinkStreamAttitudeControls : public MavlinkStream
{
public:
	const char *get_name() const
	{
		return MavlinkStreamAttitudeControls::get_name_static();
	}

	static const char *get_name_static()
	{
		return "ATTITUDE_CONTROLS";
	}

	uint8_t get_id()
	{
		return 0;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeControls(mavlink);
	}

	unsigned get_size()
	{
		return 4 * (MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	MavlinkOrbSubscription *_att_ctrl_sub;
	uint64_t _att_ctrl_time;

	/* do not allow top copying this class */
	MavlinkStreamAttitudeControls(MavlinkStreamAttitudeControls &);
	MavlinkStreamAttitudeControls& operator = (const MavlinkStreamAttitudeControls &);

protected:
	explicit MavlinkStreamAttitudeControls(Mavlink *mavlink) : MavlinkStream(mavlink),
		_att_ctrl_sub(_mavlink->add_orb_subscription(ORB_ID_VEHICLE_ATTITUDE_CONTROLS)),
		_att_ctrl_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct actuator_controls_s att_ctrl;

		if (_att_ctrl_sub->update(&_att_ctrl_time, &att_ctrl)) {
			/* send, add spaces so that string buffer is at least 10 chars long */
			mavlink_named_value_float_t msg;

			msg.time_boot_ms = att_ctrl.timestamp / 1000;

			snprintf(msg.name, sizeof(msg.name), "rll ctrl");
			msg.value = att_ctrl.control[0];

			_mavlink->send_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, &msg);

			snprintf(msg.name, sizeof(msg.name), "ptch ctrl");
			msg.value = att_ctrl.control[1];

			_mavlink->send_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, &msg);

			snprintf(msg.name, sizeof(msg.name), "yaw ctrl");
			msg.value = att_ctrl.control[2];

			_mavlink->send_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, &msg);

			snprintf(msg.name, sizeof(msg.name), "thr ctrl");
			msg.value = att_ctrl.control[3];

			_mavlink->send_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNamedValueFloat(mavlink);
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	MavlinkOrbSubscription *_debug_sub;
	uint64_t _debug_time;

	/* do not allow top copying this class */
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &);
	MavlinkStreamNamedValueFloat& operator = (const MavlinkStreamNamedValueFloat &);

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink),
		_debug_sub(_mavlink->add_orb_subscription(ORB_ID(debug_key_value))),
		_debug_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct debug_key_value_s debug;

		if (_debug_sub->update(&_debug_time, &debug)) {
			mavlink_named_value_float_t msg;

			msg.time_boot_ms = debug.timestamp_ms;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			_mavlink->send_message(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, &msg);
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

	uint8_t get_id()
	{
		return 0;
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
	MavlinkStreamCameraCapture& operator = (const MavlinkStreamCameraCapture &);

protected:
	explicit MavlinkStreamCameraCapture(Mavlink *mavlink) : MavlinkStream(mavlink),
		_status_sub(_mavlink->add_orb_subscription(ORB_ID(vehicle_status)))
	{}

	void send(const hrt_abstime t)
	{
		struct vehicle_status_s status;
		(void)_status_sub->update(&status);

		mavlink_command_long_t msg;

		msg.target_system = mavlink_system.sysid;
		msg.target_component = MAV_COMP_ID_ALL;
		msg.command = MAV_CMD_DO_CONTROL_VIDEO;
		msg.confirmation = 0;
		msg.param1 = 0;
		msg.param2 = 0;
		msg.param3 = 0;
		/* set camera capture ON/OFF depending on arming state */
		msg.param4 = (status.arming_state == ARMING_STATE_ARMED || status.arming_state == ARMING_STATE_ARMED_ERROR) ? 1 : 0;
		msg.param5 = 0;
		msg.param6 = 0;
		msg.param7 = 0;

		_mavlink->send_message(MAVLINK_MSG_ID_COMMAND_LONG, &msg);
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

	uint8_t get_id()
	{
		return MAVLINK_MSG_ID_DISTANCE_SENSOR;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDistanceSensor(mavlink);
	}

	unsigned get_size()
	{
		return _range_sub->is_published() ? (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	MavlinkOrbSubscription *_range_sub;
	uint64_t _range_time;

	/* do not allow top copying this class */
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &);
	MavlinkStreamDistanceSensor& operator = (const MavlinkStreamDistanceSensor &);

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink),
		_range_sub(_mavlink->add_orb_subscription(ORB_ID(sensor_range_finder))),
		_range_time(0)
	{}

	void send(const hrt_abstime t)
	{
		struct range_finder_report range;

		if (_range_sub->update(&_range_time, &range)) {

			mavlink_distance_sensor_t msg;

			msg.time_boot_ms = range.timestamp / 1000;

			switch (range.type) {
			case RANGE_FINDER_TYPE_LASER:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;

			default:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			msg.id = 0;
			msg.orientation = 0;
			msg.min_distance = range.minimum_distance * 100;
			msg.max_distance = range.minimum_distance * 100;
			msg.current_distance = range.distance * 100;
			msg.covariance = 20;

			_mavlink->send_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, &msg);
		}
	}
};


StreamListItem *streams_list[] = {
	new StreamListItem(&MavlinkStreamHeartbeat::new_instance, &MavlinkStreamHeartbeat::get_name_static),
	new StreamListItem(&MavlinkStreamStatustext::new_instance, &MavlinkStreamStatustext::get_name_static),
	new StreamListItem(&MavlinkStreamCommandLong::new_instance, &MavlinkStreamCommandLong::get_name_static),
	new StreamListItem(&MavlinkStreamSysStatus::new_instance, &MavlinkStreamSysStatus::get_name_static),
	new StreamListItem(&MavlinkStreamHighresIMU::new_instance, &MavlinkStreamHighresIMU::get_name_static),
	new StreamListItem(&MavlinkStreamAttitude::new_instance, &MavlinkStreamAttitude::get_name_static),
	new StreamListItem(&MavlinkStreamAttitudeQuaternion::new_instance, &MavlinkStreamAttitudeQuaternion::get_name_static),
	new StreamListItem(&MavlinkStreamVFRHUD::new_instance, &MavlinkStreamVFRHUD::get_name_static),
	new StreamListItem(&MavlinkStreamGPSRawInt::new_instance, &MavlinkStreamGPSRawInt::get_name_static),
	new StreamListItem(&MavlinkStreamGlobalPositionInt::new_instance, &MavlinkStreamGlobalPositionInt::get_name_static),
	new StreamListItem(&MavlinkStreamLocalPositionNED::new_instance, &MavlinkStreamLocalPositionNED::get_name_static),
	new StreamListItem(&MavlinkStreamViconPositionEstimate::new_instance, &MavlinkStreamViconPositionEstimate::get_name_static),
	new StreamListItem(&MavlinkStreamGPSGlobalOrigin::new_instance, &MavlinkStreamGPSGlobalOrigin::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<0>::new_instance, &MavlinkStreamServoOutputRaw<0>::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<1>::new_instance, &MavlinkStreamServoOutputRaw<1>::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<2>::new_instance, &MavlinkStreamServoOutputRaw<2>::get_name_static),
	new StreamListItem(&MavlinkStreamServoOutputRaw<3>::new_instance, &MavlinkStreamServoOutputRaw<3>::get_name_static),
	new StreamListItem(&MavlinkStreamHILControls::new_instance, &MavlinkStreamHILControls::get_name_static),
	new StreamListItem(&MavlinkStreamPositionTargetGlobalInt::new_instance, &MavlinkStreamPositionTargetGlobalInt::get_name_static),
	new StreamListItem(&MavlinkStreamLocalPositionSetpoint::new_instance, &MavlinkStreamLocalPositionSetpoint::get_name_static),
	new StreamListItem(&MavlinkStreamAttitudeTarget::new_instance, &MavlinkStreamAttitudeTarget::get_name_static),
	new StreamListItem(&MavlinkStreamRCChannelsRaw::new_instance, &MavlinkStreamRCChannelsRaw::get_name_static),
	new StreamListItem(&MavlinkStreamManualControl::new_instance, &MavlinkStreamManualControl::get_name_static),
	new StreamListItem(&MavlinkStreamOpticalFlow::new_instance, &MavlinkStreamOpticalFlow::get_name_static),
	new StreamListItem(&MavlinkStreamAttitudeControls::new_instance, &MavlinkStreamAttitudeControls::get_name_static),
	new StreamListItem(&MavlinkStreamNamedValueFloat::new_instance, &MavlinkStreamNamedValueFloat::get_name_static),
	new StreamListItem(&MavlinkStreamCameraCapture::new_instance, &MavlinkStreamCameraCapture::get_name_static),
	new StreamListItem(&MavlinkStreamDistanceSensor::new_instance, &MavlinkStreamDistanceSensor::get_name_static),
	nullptr
};
