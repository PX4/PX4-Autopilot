/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.cpp
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/* XXX trim includes */
#include <px4_config.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include <px4_defines.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_rc_input.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#ifndef __PX4_POSIX
#include <termios.h>
#endif
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <conversion/rotation.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
#include <commander/px4_custom_mode.h>
#include <geo/geo.h>

__BEGIN_DECLS

#include "mavlink_bridge_header.h"
#include "mavlink_receiver.h"
#include "mavlink_main.h"

__END_DECLS

static const float mg2ms2 = CONSTANTS_ONE_G / 1000.0f;

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	_mavlink(parent),
	status{},
	hil_local_pos{},
	hil_land_detector{},
	_control_mode{},
	_global_pos_pub(nullptr),
	_local_pos_pub(nullptr),
	_attitude_pub(nullptr),
	_gps_pub(nullptr),
	_sensors_pub(nullptr),
	_gyro_pub(nullptr),
	_accel_pub(nullptr),
	_mag_pub(nullptr),
	_baro_pub(nullptr),
	_airspeed_pub(nullptr),
	_battery_pub(nullptr),
	_cmd_pub(nullptr),
	_flow_pub(nullptr),
	_hil_distance_sensor_pub(nullptr),
	_flow_distance_sensor_pub(nullptr),
	_distance_sensor_pub(nullptr),
	_offboard_control_mode_pub(nullptr),
	_actuator_controls_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_att_sp_pub(nullptr),
	_rates_sp_pub(nullptr),
	_force_sp_pub(nullptr),
	_pos_sp_triplet_pub(nullptr),
	_att_pos_mocap_pub(nullptr),
	_vision_position_pub(nullptr),
	_telemetry_status_pub(nullptr),
	_rc_pub(nullptr),
	_manual_pub(nullptr),
	_land_detector_pub(nullptr),
	_time_offset_pub(nullptr),
	_follow_target_pub(nullptr),
	_control_mode_sub(orb_subscribe(ORB_ID(vehicle_control_mode))),
	_hil_frames(0),
	_old_timestamp(0),
	_hil_last_frame(0),
	_hil_local_proj_inited(0),
	_hil_local_alt0(0.0f),
	_hil_prev_gyro{},
	_hil_prev_accel{},
	_hil_local_proj_ref{},
	_offboard_control_mode{},
	_att_sp{},
	_rates_sp{},
	_time_offset_avg_alpha(0.6),
	_time_offset(0),
	_orb_class_instance(-1),
	_mom_switch_pos{},
	_mom_switch_state(0)
{

}

MavlinkReceiver::~MavlinkReceiver()
{
}

void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_message_command_long(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_INT:
		handle_message_command_int(msg);
		break;

	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		handle_message_optical_flow_rad(msg);
		break;

	case MAVLINK_MSG_ID_PING:
		handle_message_ping(msg);
		break;

	case MAVLINK_MSG_ID_SET_MODE:
		handle_message_set_mode(msg);
		break;

	case MAVLINK_MSG_ID_ATT_POS_MOCAP:
		handle_message_att_pos_mocap(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		handle_message_set_position_target_local_ned(msg);
		break;

	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		handle_message_set_attitude_target(msg);
		break;

	case MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET:
		handle_message_set_actuator_control_target(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status(msg);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		handle_message_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		handle_message_rc_channels_override(msg);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		handle_message_request_data_stream(msg);
		break;

	case MAVLINK_MSG_ID_SYSTEM_TIME:
		handle_message_system_time(msg);
		break;

	case MAVLINK_MSG_ID_TIMESYNC:
		handle_message_timesync(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;
	case MAVLINK_MSG_ID_FOLLOW_TARGET:
		handle_message_follow_target(msg);
		break;
	default:
		break;
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 *
	 * Accept HIL GPS messages if use_hil_gps flag is true.
	 * This allows to provide fake gps measurements to the system.
	 */
	if (_mavlink->get_hil_enabled()) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			handle_message_hil_sensor(msg);
			break;

		case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
			handle_message_hil_state_quaternion(msg);
			break;

		case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
			handle_message_hil_optical_flow(msg);
			break;

		default:
			break;
		}
	}


   	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

    /* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

bool
MavlinkReceiver::evaluate_target_ok(int command, int target_system, int target_component)
{
	/* evaluate if this system should accept this command */
	bool target_ok = false;
	switch (command) {

		case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
			/* broadcast and ignore component */
			target_ok = (target_system == 0) || (target_system == mavlink_system.sysid);
			break;

		default:
			target_ok = (target_system == mavlink_system.sysid) && ((target_component == mavlink_system.compid)
			|| (target_component == MAV_COMP_ID_ALL));
			break;
	}

	return target_ok;
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);

	if (target_ok) {
		//check for MAVLINK terminate command
		if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
			/* This is the link shutdown command, terminate mavlink */
			warnx("terminated by remote");
			fflush(stdout);
			usleep(50000);

			/* terminate other threads and this thread */
			_mavlink->_task_should_exit = true;
		} else if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
			/* send autopilot version message */
			_mavlink->send_autopilot_capabilites();

		} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
			_mavlink->configure_stream_threadsafe("HOME_POSITION", 0.5f);

		} else {

			if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
				warnx("ignoring CMD with same SYS/COMP (%d/%d) ID",
				      mavlink_system.sysid, mavlink_system.compid);
				return;
			}

			struct vehicle_command_s vcmd;
			memset(&vcmd, 0, sizeof(vcmd));

			/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
			vcmd.param1 = cmd_mavlink.param1;
			vcmd.param2 = cmd_mavlink.param2;
			vcmd.param3 = cmd_mavlink.param3;
			vcmd.param4 = cmd_mavlink.param4;
			vcmd.param5 = cmd_mavlink.param5;
			vcmd.param6 = cmd_mavlink.param6;
			vcmd.param7 = cmd_mavlink.param7;
			// XXX do proper translation
			vcmd.command = cmd_mavlink.command;
			vcmd.target_system = cmd_mavlink.target_system;
			vcmd.target_component = cmd_mavlink.target_component;
			vcmd.source_system = msg->sysid;
			vcmd.source_component = msg->compid;
			vcmd.confirmation =  cmd_mavlink.confirmation;

			if (_cmd_pub == nullptr) {
				_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

			} else {
				orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_command_int(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_int_t cmd_mavlink;
	mavlink_msg_command_int_decode(msg, &cmd_mavlink);

	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);

	if (target_ok) {
		//check for MAVLINK terminate command
		if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
			/* This is the link shutdown command, terminate mavlink */
			warnx("terminated by remote");
			fflush(stdout);
			usleep(50000);

			/* terminate other threads and this thread */
			_mavlink->_task_should_exit = true;

		} else if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
			/* send autopilot version message */
			_mavlink->send_autopilot_capabilites();

		} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
			_mavlink->configure_stream_threadsafe("HOME_POSITION", 0.5f);

		} else {

			if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
				warnx("ignoring CMD with same SYS/COMP (%d/%d) ID",
				      mavlink_system.sysid, mavlink_system.compid);
				return;
			}

			struct vehicle_command_s vcmd;
			memset(&vcmd, 0, sizeof(vcmd));

			/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
			vcmd.param1 = cmd_mavlink.param1;
			vcmd.param2 = cmd_mavlink.param2;
			vcmd.param3 = cmd_mavlink.param3;
			vcmd.param4 = cmd_mavlink.param4;
			/* these are coordinates as 1e7 scaled integers to work around the 32 bit floating point limits */
			vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
			vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
			vcmd.param7 = cmd_mavlink.z;
			// XXX do proper translation
			vcmd.command = cmd_mavlink.command;
			vcmd.target_system = cmd_mavlink.target_system;
			vcmd.target_component = cmd_mavlink.target_component;
			vcmd.source_system = msg->sysid;
			vcmd.source_component = msg->compid;

			if (_cmd_pub == nullptr) {
				_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

			} else {
				orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	enum Rotation flow_rot;
	param_get(param_find("SENS_FLOW_ROT"),&flow_rot);

	struct optical_flow_s f;
	memset(&f, 0, sizeof(f));

	f.timestamp = flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	/* rotate measurements according to parameter */
	float zeroval = 0.0f;
	rotate_3f(flow_rot, f.pixel_flow_x_integral, f.pixel_flow_y_integral, zeroval);
	rotate_3f(flow_rot, f.gyro_x_rate_integral, f.gyro_y_rate_integral, f.gyro_z_rate_integral);

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = flow.integration_time_us * 1000; /* ms to us */
	d.min_distance = 0.3f;
	d.max_distance = 5.0f;
	d.current_distance = flow.distance; /* both are in m */
	d.type = 1;
	d.id = MAV_DISTANCE_SENSOR_ULTRASOUND;
	d.orientation = 8;
	d.covariance = 0.0;

	if (_flow_distance_sensor_pub == nullptr) {
		_flow_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
								     &_orb_class_instance, ORB_PRIO_HIGH);
	} else {
		orb_publish(ORB_ID(distance_sensor), _flow_distance_sensor_pub, &d);
	}
}

void
MavlinkReceiver::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	struct optical_flow_s f;
	memset(&f, 0, sizeof(f));

	f.timestamp = hrt_absolute_time(); // XXX we rely on the system time for now and not flow.time_usec;
	f.integration_timespan = flow.integration_time_us;
	f.pixel_flow_x_integral = flow.integrated_x;
	f.pixel_flow_y_integral = flow.integrated_y;
	f.gyro_x_rate_integral = flow.integrated_xgyro;
	f.gyro_y_rate_integral = flow.integrated_ygyro;
	f.gyro_z_rate_integral = flow.integrated_zgyro;
	f.time_since_last_sonar_update = flow.time_delta_distance_us;
	f.ground_distance_m = flow.distance;
	f.quality = flow.quality;
	f.sensor_id = flow.sensor_id;
	f.gyro_temperature = flow.temperature;

	if (_flow_pub == nullptr) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for distance sensor topic */
	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = hrt_absolute_time();
	d.min_distance = 0.3f;
	d.max_distance = 5.0f;
	d.current_distance = flow.distance; /* both are in m */
	d.type = 1;
	d.id = 0;
	d.orientation = 8;
	d.covariance = 0.0;

	if (_hil_distance_sensor_pub == nullptr) {
		_hil_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
								     &_orb_class_instance, ORB_PRIO_HIGH);
	} else {
		orb_publish(ORB_ID(distance_sensor), _hil_distance_sensor_pub, &d);
	}
}

void
MavlinkReceiver::handle_message_set_mode(mavlink_message_t *msg)
{
	mavlink_set_mode_t new_mode;
	mavlink_msg_set_mode_decode(msg, &new_mode);

	struct vehicle_command_s vcmd;
	memset(&vcmd, 0, sizeof(vcmd));

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;
	/* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = new_mode.base_mode;
	vcmd.param2 = custom_mode.main_mode;
	vcmd.param3 = custom_mode.sub_mode;
	vcmd.param4 = 0;
	vcmd.param5 = 0;
	vcmd.param6 = 0;
	vcmd.param7 = 0;
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = 1;

	if (_cmd_pub == nullptr) {
		_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

	} else {
		orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
	}
}

void
MavlinkReceiver::handle_message_distance_sensor(mavlink_message_t *msg)
{
	/* distance sensor */
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	struct distance_sensor_s d;
	memset(&d, 0, sizeof(d));

	d.timestamp = dist_sensor.time_boot_ms * 1000; /* ms to us */
	d.min_distance = float(dist_sensor.min_distance) * 1e-2f; /* cm to m */
	d.max_distance = float(dist_sensor.max_distance) * 1e-2f; /* cm to m */
	d.current_distance = float(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	d.type = dist_sensor.type;
	d.id = 	MAV_DISTANCE_SENSOR_LASER;
	d.orientation = dist_sensor.orientation;
	d.covariance = dist_sensor.covariance;

	/// TODO Add sensor rotation according to MAV_SENSOR_ORIENTATION enum

	if (_distance_sensor_pub == nullptr) {
		_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
								     &_orb_class_instance, ORB_PRIO_HIGH);

	} else {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub, &d);
	}
}

void
MavlinkReceiver::handle_message_att_pos_mocap(mavlink_message_t *msg)
{
	mavlink_att_pos_mocap_t mocap;
	mavlink_msg_att_pos_mocap_decode(msg, &mocap);

	struct att_pos_mocap_s att_pos_mocap;
	memset(&att_pos_mocap, 0, sizeof(att_pos_mocap));

	// Use the component ID to identify the mocap system
	att_pos_mocap.id = msg->compid;

	att_pos_mocap.timestamp_boot = hrt_absolute_time(); // Monotonic time
	att_pos_mocap.timestamp_computer = sync_stamp(mocap.time_usec); // Synced time

	att_pos_mocap.q[0] = mocap.q[0];
	att_pos_mocap.q[1] = mocap.q[1];
	att_pos_mocap.q[2] = mocap.q[2];
	att_pos_mocap.q[3] = mocap.q[3];

	att_pos_mocap.x = mocap.x;
	att_pos_mocap.y = mocap.y;
	att_pos_mocap.z = mocap.z;

	if (_att_pos_mocap_pub == nullptr) {
		_att_pos_mocap_pub = orb_advertise(ORB_ID(att_pos_mocap), &att_pos_mocap);

	} else {
		orb_publish(ORB_ID(att_pos_mocap), _att_pos_mocap_pub, &att_pos_mocap);
	}
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t set_position_target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

	struct offboard_control_mode_s offboard_control_mode;
	memset(&offboard_control_mode, 0, sizeof(offboard_control_mode));//XXX breaks compatibility with multiple setpoints

	bool values_finite =
		PX4_ISFINITE(set_position_target_local_ned.x) &&
		PX4_ISFINITE(set_position_target_local_ned.y) &&
		PX4_ISFINITE(set_position_target_local_ned.z) &&
		PX4_ISFINITE(set_position_target_local_ned.vx) &&
		PX4_ISFINITE(set_position_target_local_ned.vy) &&
		PX4_ISFINITE(set_position_target_local_ned.vz) &&
		PX4_ISFINITE(set_position_target_local_ned.afx) &&
		PX4_ISFINITE(set_position_target_local_ned.afy) &&
		PX4_ISFINITE(set_position_target_local_ned.afz) &&
		PX4_ISFINITE(set_position_target_local_ned.yaw);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_position_target_local_ned.target_system ||
				set_position_target_local_ned.target_system == 0) &&
			(mavlink_system.compid == set_position_target_local_ned.target_component ||
				set_position_target_local_ned.target_component == 0) &&
			values_finite) {

		/* convert mavlink type (local, NED) to uORB offboard control struct */
		offboard_control_mode.ignore_position = (bool)(set_position_target_local_ned.type_mask & 0x7);
		offboard_control_mode.ignore_velocity = (bool)(set_position_target_local_ned.type_mask & 0x38);
		offboard_control_mode.ignore_acceleration_force = (bool)(set_position_target_local_ned.type_mask & 0x1C0);
		bool is_force_sp = (bool)(set_position_target_local_ned.type_mask & (1 << 9));
		/* yaw ignore flag mapps to ignore_attitude */
		offboard_control_mode.ignore_attitude = (bool)(set_position_target_local_ned.type_mask & 0x400);
		/* yawrate ignore flag mapps to ignore_bodyrate */
		offboard_control_mode.ignore_bodyrate = (bool)(set_position_target_local_ned.type_mask & 0x800);


		bool is_takeoff_sp = (bool)(set_position_target_local_ned.type_mask & 0x1000);
		bool is_land_sp = (bool)(set_position_target_local_ned.type_mask & 0x2000);
		bool is_loiter_sp = (bool)(set_position_target_local_ned.type_mask & 0x3000);
		bool is_idle_sp = (bool)(set_position_target_local_ned.type_mask & 0x4000);

		offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
		}

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {
			bool updated;
			orb_check(_control_mode_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
			}
			if (_control_mode.flag_control_offboard_enabled) {
				if (is_force_sp && offboard_control_mode.ignore_position &&
						offboard_control_mode.ignore_velocity) {
					/* The offboard setpoint is a force setpoint only, directly writing to the force
					 * setpoint topic and not publishing the setpoint triplet topic */
					struct vehicle_force_setpoint_s	force_sp;
					force_sp.x = set_position_target_local_ned.afx;
					force_sp.y = set_position_target_local_ned.afy;
					force_sp.z = set_position_target_local_ned.afz;
					//XXX: yaw
					if (_force_sp_pub == nullptr) {
						_force_sp_pub = orb_advertise(ORB_ID(vehicle_force_setpoint), &force_sp);
					} else {
						orb_publish(ORB_ID(vehicle_force_setpoint), _force_sp_pub, &force_sp);
					}
				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					struct position_setpoint_triplet_s pos_sp_triplet;
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

					} else if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_idle_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;

					} else {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
					}

					/* set the local pos values */
					if (!offboard_control_mode.ignore_position) {
						pos_sp_triplet.current.position_valid = true;
						pos_sp_triplet.current.x = set_position_target_local_ned.x;
						pos_sp_triplet.current.y = set_position_target_local_ned.y;
						pos_sp_triplet.current.z = set_position_target_local_ned.z;
					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					/* set the local vel values */
					if (!offboard_control_mode.ignore_velocity) {
						pos_sp_triplet.current.velocity_valid = true;
						pos_sp_triplet.current.vx = set_position_target_local_ned.vx;
						pos_sp_triplet.current.vy = set_position_target_local_ned.vy;
						pos_sp_triplet.current.vz = set_position_target_local_ned.vz;
					} else {
						pos_sp_triplet.current.velocity_valid = false;
					}

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */
					if (!offboard_control_mode.ignore_acceleration_force) {
						pos_sp_triplet.current.acceleration_valid = true;
						pos_sp_triplet.current.a_x = set_position_target_local_ned.afx;
						pos_sp_triplet.current.a_y = set_position_target_local_ned.afy;
						pos_sp_triplet.current.a_z = set_position_target_local_ned.afz;
						pos_sp_triplet.current.acceleration_is_force =
						is_force_sp;

					} else {
						pos_sp_triplet.current.acceleration_valid = false;
					}

					/* set the yaw sp value */
					if (!offboard_control_mode.ignore_attitude) {
						pos_sp_triplet.current.yaw_valid = true;
						pos_sp_triplet.current.yaw = set_position_target_local_ned.yaw;

					} else {
						pos_sp_triplet.current.yaw_valid = false;
					}

					/* set the yawrate sp value */
					if (!offboard_control_mode.ignore_bodyrate) {
						pos_sp_triplet.current.yawspeed_valid = true;
						pos_sp_triplet.current.yawspeed = set_position_target_local_ned.yaw_rate;

					} else {
						pos_sp_triplet.current.yawspeed_valid = false;
					}
					//XXX handle global pos setpoints (different MAV frames)

					if (_pos_sp_triplet_pub == nullptr) {
						_pos_sp_triplet_pub = orb_advertise(ORB_ID(position_setpoint_triplet),
								&pos_sp_triplet);
					} else {
						orb_publish(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_pub,
								&pos_sp_triplet);
					}

				}

			}

		}
	}
}

void
MavlinkReceiver::handle_message_set_actuator_control_target(mavlink_message_t *msg)
{
	mavlink_set_actuator_control_target_t set_actuator_control_target;
	mavlink_msg_set_actuator_control_target_decode(msg, &set_actuator_control_target);

	struct offboard_control_mode_s offboard_control_mode;
	memset(&offboard_control_mode, 0, sizeof(offboard_control_mode));//XXX breaks compatibility with multiple setpoints

	struct actuator_controls_s actuator_controls;
	memset(&actuator_controls, 0, sizeof(actuator_controls));//XXX breaks compatibility with multiple setpoints

	bool values_finite =
		PX4_ISFINITE(set_actuator_control_target.controls[0]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[1]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[2]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[3]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[4]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[5]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[6]) &&
		PX4_ISFINITE(set_actuator_control_target.controls[7]);

	if ((mavlink_system.sysid == set_actuator_control_target.target_system ||
		    set_actuator_control_target.target_system == 0) &&
		(mavlink_system.compid == set_actuator_control_target.target_component ||
		    set_actuator_control_target.target_component == 0) &&
		values_finite) {

		/* ignore all since we are setting raw actuators here */
		offboard_control_mode.ignore_thrust             = true;
		offboard_control_mode.ignore_attitude           = true;
		offboard_control_mode.ignore_bodyrate           = true;
		offboard_control_mode.ignore_position           = true;
		offboard_control_mode.ignore_velocity           = true;
		offboard_control_mode.ignore_acceleration_force = true;

		offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &offboard_control_mode);
		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &offboard_control_mode);
		}


		/* If we are in offboard control mode, publish the actuator controls */
		bool updated;
		orb_check(_control_mode_sub, &updated);
		if (updated) {
			orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
		}

		if (_control_mode.flag_control_offboard_enabled) {

			actuator_controls.timestamp = hrt_absolute_time();

			/* Set duty cycles for the servos in actuator_controls_0 */
			for(size_t i = 0; i < 8; i++) {
				actuator_controls.control[i] = set_actuator_control_target.controls[i];
			}

			if (_actuator_controls_pub == nullptr) {
				_actuator_controls_pub = orb_advertise(ORB_ID(actuator_controls_0), &actuator_controls);
			} else {
				orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pub, &actuator_controls);
			}
		}
	}

}

void
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t pos;
	mavlink_msg_vision_position_estimate_decode(msg, &pos);

	struct vision_position_estimate_s vision_position;
	memset(&vision_position, 0, sizeof(vision_position));

	// Use the component ID to identify the vision sensor
	vision_position.id = msg->compid;

	vision_position.timestamp_boot = hrt_absolute_time(); // Monotonic time
	vision_position.timestamp_computer = sync_stamp(pos.usec); // Synced time
	vision_position.x = pos.x;
	vision_position.y = pos.y;
	vision_position.z = pos.z;

	// XXX fix this
	vision_position.vx = 0.0f;
	vision_position.vy = 0.0f;
	vision_position.vz = 0.0f;

	math::Quaternion q;
	q.from_euler(pos.roll, pos.pitch, pos.yaw);

	vision_position.q[0] = q(0);
	vision_position.q[1] = q(1);
	vision_position.q[2] = q(2);
	vision_position.q[3] = q(3);

	if (_vision_position_pub == nullptr) {
		_vision_position_pub = orb_advertise(ORB_ID(vision_position_estimate), &vision_position);

	} else {
		orb_publish(ORB_ID(vision_position_estimate), _vision_position_pub, &vision_position);
	}
}

void
MavlinkReceiver::handle_message_set_attitude_target(mavlink_message_t *msg)
{
	mavlink_set_attitude_target_t set_attitude_target;
	mavlink_msg_set_attitude_target_decode(msg, &set_attitude_target);

	bool values_finite =
		PX4_ISFINITE(set_attitude_target.q[0]) &&
		PX4_ISFINITE(set_attitude_target.q[1]) &&
		PX4_ISFINITE(set_attitude_target.q[2]) &&
		PX4_ISFINITE(set_attitude_target.q[3]) &&
		PX4_ISFINITE(set_attitude_target.thrust) &&
		PX4_ISFINITE(set_attitude_target.body_roll_rate) &&
		PX4_ISFINITE(set_attitude_target.body_pitch_rate) &&
		PX4_ISFINITE(set_attitude_target.body_yaw_rate);

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_attitude_target.target_system ||
				set_attitude_target.target_system == 0) &&
			(mavlink_system.compid == set_attitude_target.target_component ||
				set_attitude_target.target_component == 0) &&
			values_finite) {

		/* set correct ignore flags for thrust field: copy from mavlink message */
		_offboard_control_mode.ignore_thrust = (bool)(set_attitude_target.type_mask & (1 << 6));

		/*
		 * The tricky part in parsing this message is that the offboard sender *can* set attitude and thrust
		 * using different messages. Eg.: First send set_attitude_target containing the attitude and ignore
		 * bits set for everything else and then send set_attitude_target containing the thrust and ignore bits
		 * set for everything else.
		 */

		/*
		 * if attitude or body rate have been used (not ignored) previously and this message only sends
		 * throttle and has the ignore bits set for attitude and rates don't change the flags for attitude and
		 * body rates to keep the controllers running
		 */
		bool ignore_bodyrate_msg = (bool)(set_attitude_target.type_mask & 0x7);
		bool ignore_attitude_msg = (bool)(set_attitude_target.type_mask & (1 << 7));

		if (ignore_bodyrate_msg && ignore_attitude_msg && !_offboard_control_mode.ignore_thrust) {
			/* Message want's us to ignore everything except thrust: only ignore if previously ignored */
			_offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg && _offboard_control_mode.ignore_bodyrate;
			_offboard_control_mode.ignore_attitude = ignore_attitude_msg && _offboard_control_mode.ignore_attitude;
		} else {
			_offboard_control_mode.ignore_bodyrate = ignore_bodyrate_msg;
			_offboard_control_mode.ignore_attitude = ignore_attitude_msg;
		}

		_offboard_control_mode.ignore_position = true;
		_offboard_control_mode.ignore_velocity = true;
		_offboard_control_mode.ignore_acceleration_force = true;

		_offboard_control_mode.timestamp = hrt_absolute_time();

		if (_offboard_control_mode_pub == nullptr) {
			_offboard_control_mode_pub = orb_advertise(ORB_ID(offboard_control_mode), &_offboard_control_mode);

		} else {
			orb_publish(ORB_ID(offboard_control_mode), _offboard_control_mode_pub, &_offboard_control_mode);
		}

		/* If we are in offboard control mode and offboard control loop through is enabled
		 * also publish the setpoint topic which is read by the controller */
		if (_mavlink->get_forward_externalsp()) {
			bool updated;
			orb_check(_control_mode_sub, &updated);
			if (updated) {
				orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
			}

			if (_control_mode.flag_control_offboard_enabled) {

				/* Publish attitude setpoint if attitude and thrust ignore bits are not set */
				if (!(_offboard_control_mode.ignore_attitude)) {
					_att_sp.timestamp = hrt_absolute_time();
					if (!ignore_attitude_msg) { // only copy att sp if message contained new data
						mavlink_quaternion_to_euler(set_attitude_target.q,
								&_att_sp.roll_body, &_att_sp.pitch_body, &_att_sp.yaw_body);
						mavlink_quaternion_to_dcm(set_attitude_target.q, (float(*)[3])_att_sp.R_body);
						_att_sp.R_valid = true;
						_att_sp.yaw_sp_move_rate = 0.0;
						memcpy(_att_sp.q_d, set_attitude_target.q, sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;
					}

					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						_att_sp.thrust = set_attitude_target.thrust;
					}

					if (_att_sp_pub == nullptr) {
						_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &_att_sp);
					} else {
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &_att_sp);
					}
				}

				/* Publish attitude rate setpoint if bodyrate and thrust ignore bits are not set */
				///XXX add support for ignoring individual axes
				if (!(_offboard_control_mode.ignore_bodyrate)) {
					_rates_sp.timestamp = hrt_absolute_time();
					if (!ignore_bodyrate_msg) { // only copy att rates sp if message contained new data
						_rates_sp.roll = set_attitude_target.body_roll_rate;
						_rates_sp.pitch = set_attitude_target.body_pitch_rate;
						_rates_sp.yaw = set_attitude_target.body_yaw_rate;
					}
					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						_rates_sp.thrust = set_attitude_target.thrust;
					}

					if (_rates_sp_pub == nullptr) {
						_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &_rates_sp);
					} else {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &_rates_sp);
					}
				}
			}

		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink->get_channel() < ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

		tstatus.timestamp = hrt_absolute_time();
		tstatus.telem_time = tstatus.timestamp;
		/* tstatus.heartbeat_time is set by system heartbeats */
		tstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO;
		tstatus.rssi = rstatus.rssi;
		tstatus.remote_rssi = rstatus.remrssi;
		tstatus.txbuf = rstatus.txbuf;
		tstatus.noise = rstatus.noise;
		tstatus.remote_noise = rstatus.remnoise;
		tstatus.rxerrors = rstatus.rxerrors;
		tstatus.fixed = rstatus.fixed;
		tstatus.system_id = msg->sysid;
		tstatus.component_id = msg->compid;

		if (_telemetry_status_pub == nullptr) {
			int multi_instance;
			_telemetry_status_pub = orb_advertise_multi(ORB_ID(telemetry_status), &tstatus, &multi_instance, ORB_PRIO_HIGH);

		} else {
			orb_publish(ORB_ID(telemetry_status), _telemetry_status_pub, &tstatus);
		}
	}
}

switch_pos_t
MavlinkReceiver::decode_switch_pos(uint16_t buttons, unsigned sw)
{
	// This 2-bit method should be used in the future: (buttons >> (sw * 2)) & 3;
	return (buttons & (1 << sw)) ? manual_control_setpoint_s::SWITCH_POS_ON : manual_control_setpoint_s::SWITCH_POS_OFF;
}

int
MavlinkReceiver::decode_switch_pos_n(uint16_t buttons, unsigned sw)
{

	bool on = (buttons & (1 << sw));

	if (sw < MOM_SWITCH_COUNT) {

		bool last_on = (_mom_switch_state & (1 << sw));

		/* first switch is 2-pos, rest is 2 pos */
		unsigned state_count = (sw == 0) ? 3 : 2;

		/* only transition on low state */
		if (!on && (on != last_on)) {

			_mom_switch_pos[sw]++;
			if (_mom_switch_pos[sw] == state_count) {
				_mom_switch_pos[sw] = 0;
			}
		}

		/* state_count - 1 is the number of intervals and 1000 is the range,
		 * with 2 states 0 becomes 0, 1 becomes 1000. With
		 * 3 states 0 becomes 0, 1 becomes 500, 2 becomes 1000,
		 * and so on for more states.
		 */
		return (_mom_switch_pos[sw] * 1000) / (state_count - 1) + 1000;

	} else {
		/* return the current state */
		return on * 1000 + 1000;
	}
}

void
MavlinkReceiver::handle_message_rc_channels_override(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	// Check target
	if (man.target_system != 0 && man.target_system != _mavlink->get_system_id()) {
		return;
	}

	struct rc_input_values rc = {};
	rc.timestamp_publication = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp_publication;

	rc.channel_count = 8;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
	rc.rssi = RC_INPUT_RSSI_MAX;

	/* channels */
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;

	if (_rc_pub == nullptr) {
		_rc_pub = orb_advertise(ORB_ID(input_rc), &rc);

	} else {
		orb_publish(ORB_ID(input_rc), _rc_pub, &rc);
	}
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t man;
	mavlink_msg_manual_control_decode(msg, &man);

	// Check target
	if (man.target != 0 && man.target != _mavlink->get_system_id()) {
		return;
	}

	if (_mavlink->get_manual_input_mode_generation()) {

		struct rc_input_values rc = {};
		rc.timestamp_publication = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp_publication;

		rc.channel_count = 8;
		rc.rc_failsafe = false;
		rc.rc_lost = false;
		rc.rc_lost_frame_count = 0;
		rc.rc_total_frame_count = 1;
		rc.rc_ppm_frame_length = 0;
		rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
		rc.rssi = RC_INPUT_RSSI_MAX;

		/* roll */
		rc.values[0] = man.x / 2 + 1500;
		/* pitch */
		rc.values[1] = man.y / 2 + 1500;
		/* yaw */
		rc.values[2] = man.r / 2 + 1500;
		/* throttle */
		rc.values[3] = fminf(fmaxf(man.z / 0.9f + 800, 1000.0f), 2000.0f);

		/* decode all switches which fit into the channel mask */
		unsigned max_switch = (sizeof(man.buttons) * 8);
		unsigned max_channels = (sizeof(rc.values) / sizeof(rc.values[0]));
		if (max_switch > (max_channels - 4)) {
			max_switch = (max_channels - 4);
		}

		/* fill all channels */
		for (unsigned i = 0; i < max_switch; i++) {
			rc.values[i + 4] = decode_switch_pos_n(man.buttons, i);
		}
		_mom_switch_state = man.buttons;

		if (_rc_pub == nullptr) {
			_rc_pub = orb_advertise(ORB_ID(input_rc), &rc);

		} else {
			orb_publish(ORB_ID(input_rc), _rc_pub, &rc);
		}

	} else {
		struct manual_control_setpoint_s manual = {};

		manual.timestamp = hrt_absolute_time();
		manual.x = man.x / 1000.0f;
		manual.y = man.y / 1000.0f;
		manual.r = man.r / 1000.0f;
		manual.z = man.z / 1000.0f;

		if (_manual_pub == nullptr) {
			_manual_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);

		} else {
			orb_publish(ORB_ID(manual_control_setpoint), _manual_pub, &manual);
		}
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		/* ignore own heartbeats, accept only heartbeats from GCS */
		if (msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) {

			struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

			/* set heartbeat time and topic time and publish -
			 * the telem status also gets updated on telemetry events
			 */
			tstatus.timestamp = hrt_absolute_time();
			tstatus.heartbeat_time = tstatus.timestamp;

			if (_telemetry_status_pub == nullptr) {
				int multi_instance;
				_telemetry_status_pub = orb_advertise_multi(ORB_ID(telemetry_status), &tstatus, &multi_instance, ORB_PRIO_HIGH);

			} else {
				orb_publish(ORB_ID(telemetry_status), _telemetry_status_pub, &tstatus);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_ping(mavlink_message_t *msg)
{
	mavlink_ping_t ping;
	mavlink_msg_ping_decode( msg, &ping);
	if ((mavlink_system.sysid == ping.target_system) &&
		(mavlink_system.compid == ping.target_component)) {
		mavlink_message_t msg_out;
		mavlink_msg_ping_encode(_mavlink->get_system_id(), _mavlink->get_component_id(), &msg_out, &ping);
		_mavlink->send_message(MAVLINK_MSG_ID_PING, &msg_out);
	}
}

void
MavlinkReceiver::handle_message_request_data_stream(mavlink_message_t *msg)
{
	mavlink_request_data_stream_t req;
	mavlink_msg_request_data_stream_decode(msg, &req);

	if (req.target_system == mavlink_system.sysid && req.target_component == mavlink_system.compid && req.req_message_rate != 0) {
		float rate = req.start_stop ? (1000.0f / req.req_message_rate) : 0.0f;

		MavlinkStream *stream;
		LL_FOREACH(_mavlink->get_streams(), stream) {
			if (req.req_stream_id == stream->get_id()) {
				_mavlink->configure_stream_threadsafe(stream->get_name(), rate);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_system_time(mavlink_message_t *msg)
{
	mavlink_system_time_t time;
	mavlink_msg_system_time_decode(msg, &time);

	timespec tv;
	px4_clock_gettime(CLOCK_REALTIME, &tv);

	// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
	bool onb_unix_valid = (unsigned long long)tv.tv_sec > PX4_EPOCH_SECS;
	bool ofb_unix_valid = time.time_unix_usec > PX4_EPOCH_SECS * 1000ULL;

	if (!onb_unix_valid && ofb_unix_valid) {
		tv.tv_sec = time.time_unix_usec / 1000000ULL;
		tv.tv_nsec = (time.time_unix_usec % 1000000ULL) * 1000ULL;
		if(px4_clock_settime(CLOCK_REALTIME, &tv)) {
			warn("failed setting clock");
		}
		else {
		warnx("[timesync] UTC time synced.");
		}
	}

}

void
MavlinkReceiver::handle_message_timesync(mavlink_message_t *msg)
{
	mavlink_timesync_t tsync;
	mavlink_msg_timesync_decode(msg, &tsync);

	struct time_offset_s tsync_offset;
	memset(&tsync_offset, 0, sizeof(tsync_offset));

	uint64_t now_ns = hrt_absolute_time() * 1000LL ;

	if (tsync.tc1 == 0) {

		mavlink_timesync_t rsync; // return timestamped sync message

		rsync.tc1 = now_ns;
		rsync.ts1 = tsync.ts1;

		_mavlink->send_message(MAVLINK_MSG_ID_TIMESYNC, &rsync);

		return;

	} else if (tsync.tc1 > 0) {

		int64_t offset_ns = (tsync.ts1 + now_ns - tsync.tc1*2)/2 ;
		int64_t dt = _time_offset - offset_ns;

		if (dt > 10000000LL || dt < -10000000LL) { // 10 millisecond skew
			_time_offset = offset_ns;
			warnx("[timesync] Hard setting offset.");
		} else {
			smooth_time_offset(offset_ns);
		}
	}

	tsync_offset.offset_ns = _time_offset ;

	if (_time_offset_pub == nullptr) {
		_time_offset_pub = orb_advertise(ORB_ID(time_offset), &tsync_offset);

	} else {
		orb_publish(ORB_ID(time_offset), _time_offset_pub, &tsync_offset);
	}

}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	uint64_t timestamp = hrt_absolute_time();

	float dt;

	if (_hil_last_frame == 0 ||
	   (timestamp <= _hil_last_frame) ||
	   (timestamp - _hil_last_frame) > (0.1f * 1e6f) ||
	   (_hil_last_frame >= timestamp)) {
		dt = 0.01f; /* default to 100 Hz */
		memset(&_hil_prev_gyro[0], 0, sizeof(_hil_prev_gyro));
		_hil_prev_accel[0] = 0.0f;
		_hil_prev_accel[1] = 0.0f;
		_hil_prev_accel[2] = 9.866f;
	} else {
		dt = (timestamp - _hil_last_frame) / 1e6f;
	}
	_hil_last_frame = timestamp;

	/* airspeed */
	{
		struct airspeed_s airspeed = {};

		float ias = calc_indicated_airspeed(imu.diff_pressure * 1e2f);
		float tas = calc_true_airspeed_from_indicated(ias, imu.abs_pressure * 100, imu.temperature);

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		if (_airspeed_pub == nullptr) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* gyro */
	{
		struct gyro_report gyro = {};

		gyro.timestamp = timestamp;
		gyro.x_raw = imu.xgyro * 1000.0f;
		gyro.y_raw = imu.ygyro * 1000.0f;
		gyro.z_raw = imu.zgyro * 1000.0f;
		gyro.x = imu.xgyro;
		gyro.y = imu.ygyro;
		gyro.z = imu.zgyro;
		gyro.temperature = imu.temperature;

		if (_gyro_pub == nullptr) {
			_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);

		} else {
			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &gyro);
		}
	}

	/* accelerometer */
	{
		struct accel_report accel = {};

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / mg2ms2;
		accel.y_raw = imu.yacc / mg2ms2;
		accel.z_raw = imu.zacc / mg2ms2;
		accel.x = imu.xacc;
		accel.y = imu.yacc;
		accel.z = imu.zacc;
		accel.temperature = imu.temperature;

		if (_accel_pub == nullptr) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* magnetometer */
	{
		struct mag_report mag = {};

		mag.timestamp = timestamp;
		mag.x_raw = imu.xmag * 1000.0f;
		mag.y_raw = imu.ymag * 1000.0f;
		mag.z_raw = imu.zmag * 1000.0f;
		mag.x = imu.xmag;
		mag.y = imu.ymag;
		mag.z = imu.zmag;

		if (_mag_pub == nullptr) {
			/* publish to the first mag topic */
			_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

		} else {
			orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
		}
	}

	/* baro */
	{
		struct baro_report baro = {};

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.altitude = imu.pressure_alt;
		baro.temperature = imu.temperature;

		if (_baro_pub == nullptr) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		}
	}

	/* sensor combined */
	{
		struct sensor_combined_s hil_sensors = {};

		hil_sensors.timestamp = timestamp;

		hil_sensors.gyro_raw[0] = imu.xgyro * 1000.0f;
		hil_sensors.gyro_raw[1] = imu.ygyro * 1000.0f;
		hil_sensors.gyro_raw[2] = imu.zgyro * 1000.0f;
		hil_sensors.gyro_rad_s[0] = imu.xgyro;
		hil_sensors.gyro_rad_s[1] = imu.ygyro;
		hil_sensors.gyro_rad_s[2] = imu.zgyro;
		hil_sensors.gyro_integral_rad[0] = 0.5f * (imu.xgyro + _hil_prev_gyro[0]) * dt;
		hil_sensors.gyro_integral_rad[1] = 0.5f * (imu.ygyro + _hil_prev_gyro[1]) * dt;
		hil_sensors.gyro_integral_rad[2] = 0.5f * (imu.zgyro + _hil_prev_gyro[2]) * dt;
		memcpy(&_hil_prev_gyro[0], &hil_sensors.gyro_rad_s[0], sizeof(_hil_prev_gyro));
		hil_sensors.gyro_integral_dt[0] = dt * 1e6f;
		hil_sensors.gyro_timestamp[0] = timestamp;
		hil_sensors.gyro_priority[0] = ORB_PRIO_HIGH;

		hil_sensors.accelerometer_raw[0] = imu.xacc / mg2ms2;
		hil_sensors.accelerometer_raw[1] = imu.yacc / mg2ms2;
		hil_sensors.accelerometer_raw[2] = imu.zacc / mg2ms2;
		hil_sensors.accelerometer_m_s2[0] = imu.xacc;
		hil_sensors.accelerometer_m_s2[1] = imu.yacc;
		hil_sensors.accelerometer_m_s2[2] = imu.zacc;
		hil_sensors.accelerometer_integral_m_s[0] = 0.5f * (imu.xacc + _hil_prev_accel[0]) * dt;
		hil_sensors.accelerometer_integral_m_s[1] = 0.5f * (imu.yacc + _hil_prev_accel[1]) * dt;
		hil_sensors.accelerometer_integral_m_s[2] = 0.5f * (imu.zacc + _hil_prev_accel[2]) * dt;
		memcpy(&_hil_prev_accel[0], &hil_sensors.accelerometer_m_s2[0], sizeof(_hil_prev_accel));
		hil_sensors.accelerometer_integral_dt[0] = dt * 1e6f;
		hil_sensors.accelerometer_mode[0] = 0; // TODO what is this?
		hil_sensors.accelerometer_range_m_s2[0] = 32.7f; // int16
		hil_sensors.accelerometer_timestamp[0] = timestamp;
		hil_sensors.accelerometer_priority[0] = ORB_PRIO_HIGH;

		hil_sensors.adc_voltage_v[0] = 0.0f;
		hil_sensors.adc_voltage_v[1] = 0.0f;
		hil_sensors.adc_voltage_v[2] = 0.0f;

		hil_sensors.magnetometer_raw[0] = imu.xmag * 1000.0f;
		hil_sensors.magnetometer_raw[1] = imu.ymag * 1000.0f;
		hil_sensors.magnetometer_raw[2] = imu.zmag * 1000.0f;
		hil_sensors.magnetometer_ga[0] = imu.xmag;
		hil_sensors.magnetometer_ga[1] = imu.ymag;
		hil_sensors.magnetometer_ga[2] = imu.zmag;
		hil_sensors.magnetometer_range_ga[0] = 32.7f; // int16
		hil_sensors.magnetometer_mode[0] = 0; // TODO what is this
		hil_sensors.magnetometer_cuttoff_freq_hz[0] = 50.0f;
		hil_sensors.magnetometer_timestamp[0] = timestamp;
		hil_sensors.magnetometer_priority[0] = ORB_PRIO_HIGH;

		hil_sensors.baro_pres_mbar[0] = imu.abs_pressure;
		hil_sensors.baro_alt_meter[0] = imu.pressure_alt;
		hil_sensors.baro_temp_celcius[0] = imu.temperature;
		hil_sensors.baro_timestamp[0] = timestamp;

		hil_sensors.differential_pressure_pa[0] = imu.diff_pressure * 1e2f; //from hPa to Pa
		hil_sensors.differential_pressure_filtered_pa[0] = hil_sensors.differential_pressure_pa[0];
		hil_sensors.differential_pressure_timestamp[0] = timestamp;

		/* publish combined sensor topic */
		if (_sensors_pub == nullptr) {
			_sensors_pub = orb_advertise(ORB_ID(sensor_combined), &hil_sensors);

		} else {
			orb_publish(ORB_ID(sensor_combined), _sensors_pub, &hil_sensors);
		}
	}

	/* battery status */
	{
		struct battery_status_s hil_battery_status = {};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.5f;
		hil_battery_status.voltage_filtered_v = 11.5f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub == nullptr) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}

	/* increment counters */
	_hil_frames++;

	/* print HIL sensors rate */
	if ((timestamp - _old_timestamp) > 10000000) {
		// printf("receiving HIL sensors at %d hz\n", _hil_frames / 10);
		_old_timestamp = timestamp;
		_hil_frames = 0;
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	uint64_t timestamp = hrt_absolute_time();

	struct vehicle_gps_position_s hil_gps;
	memset(&hil_gps, 0, sizeof(hil_gps));

	hil_gps.timestamp_time = timestamp;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp_position = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.timestamp_variance = timestamp;
	hil_gps.s_variance_m_s = 5.0f;

	hil_gps.timestamp_velocity = timestamp;
	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = _wrap_pi(gps.cog * M_DEG_TO_RAD_F * 1e-2f);

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	if (_gps_pub == nullptr) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
	}
}

void MavlinkReceiver::handle_message_follow_target(mavlink_message_t *msg)
{
	mavlink_follow_target_t follow_target_msg;
	follow_target_s follow_target_topic = { };

	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_topic.timestamp = hrt_absolute_time();

	follow_target_topic.lat = follow_target_msg.lat*1e-7;
	follow_target_topic.lon = follow_target_msg.lon*1e-7;
	follow_target_topic.alt = follow_target_msg.alt;

	if (_follow_target_pub == nullptr) {
		_follow_target_pub = orb_advertise(ORB_ID(follow_target), &follow_target_topic);
	} else {
		orb_publish(ORB_ID(follow_target), _follow_target_pub, &follow_target_topic);
	}
}

void
MavlinkReceiver::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed;
		memset(&airspeed, 0, sizeof(airspeed));

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;

		if (_airspeed_pub == nullptr) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* attitude */
	struct vehicle_attitude_s hil_attitude;
	{
		memset(&hil_attitude, 0, sizeof(hil_attitude));
		math::Quaternion q(hil_state.attitude_quaternion);
		math::Matrix<3, 3> C_nb = q.to_dcm();
		math::Vector<3> euler = C_nb.to_euler();

		hil_attitude.timestamp = timestamp;
		memcpy(hil_attitude.R, C_nb.data, sizeof(hil_attitude.R));
		hil_attitude.R_valid = true;

		hil_attitude.q[0] = q(0);
		hil_attitude.q[1] = q(1);
		hil_attitude.q[2] = q(2);
		hil_attitude.q[3] = q(3);
		hil_attitude.q_valid = true;

		hil_attitude.roll = euler(0);
		hil_attitude.pitch = euler(1);
		hil_attitude.yaw = euler(2);
		hil_attitude.rollspeed = hil_state.rollspeed;
		hil_attitude.pitchspeed = hil_state.pitchspeed;
		hil_attitude.yawspeed = hil_state.yawspeed;

		if (_attitude_pub == nullptr) {
			_attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &hil_attitude);

		} else {
			orb_publish(ORB_ID(vehicle_attitude), _attitude_pub, &hil_attitude);
		}
	}

	/* global position */
	{
		struct vehicle_global_position_s hil_global_pos;
		memset(&hil_global_pos, 0, sizeof(hil_global_pos));

		hil_global_pos.timestamp = timestamp;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.vel_n = hil_state.vx / 100.0f;
		hil_global_pos.vel_e = hil_state.vy / 100.0f;
		hil_global_pos.vel_d = hil_state.vz / 100.0f;
		hil_global_pos.yaw = hil_attitude.yaw;
		hil_global_pos.eph = 2.0f;
		hil_global_pos.epv = 4.0f;

		if (_global_pos_pub == nullptr) {
			_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &hil_global_pos);

		} else {
			orb_publish(ORB_ID(vehicle_global_position), _global_pos_pub, &hil_global_pos);
		}
	}

	/* local position */
	{
		double lat = hil_state.lat * 1e-7;
		double lon = hil_state.lon * 1e-7;

		if (!_hil_local_proj_inited) {
			_hil_local_proj_inited = true;
			_hil_local_alt0 = hil_state.alt / 1000.0f;
			map_projection_init(&_hil_local_proj_ref, lat, lon);
			hil_local_pos.ref_timestamp = timestamp;
			hil_local_pos.ref_lat = lat;
			hil_local_pos.ref_lon = lon;
			hil_local_pos.ref_alt = _hil_local_alt0;
		}

		float x;
		float y;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);
		hil_local_pos.timestamp = timestamp;
		hil_local_pos.xy_valid = true;
		hil_local_pos.z_valid = true;
		hil_local_pos.v_xy_valid = true;
		hil_local_pos.v_z_valid = true;
		hil_local_pos.x = x;
		hil_local_pos.y = y;
		hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
		hil_local_pos.vx = hil_state.vx / 100.0f;
		hil_local_pos.vy = hil_state.vy / 100.0f;
		hil_local_pos.vz = hil_state.vz / 100.0f;
		hil_local_pos.yaw = hil_attitude.yaw;
		hil_local_pos.xy_global = true;
		hil_local_pos.z_global = true;

		if (_local_pos_pub == nullptr) {
			_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &hil_local_pos);

		} else {
			orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &hil_local_pos);
		}
	}

	/* land detector */
	{
		bool landed = (float)(hil_state.alt) / 1000.0f < (_hil_local_alt0 + 0.1f); // XXX improve?
		if(hil_land_detector.landed != landed) {
			hil_land_detector.landed = landed;
			hil_land_detector.timestamp = hrt_absolute_time();

			if (_land_detector_pub == nullptr) {
				_land_detector_pub = orb_advertise(ORB_ID(vehicle_land_detected), &hil_land_detector);

			} else {
				orb_publish(ORB_ID(vehicle_land_detected), _land_detector_pub, &hil_land_detector);
			}
		}
	}

	/* accelerometer */
	{
		struct accel_report accel;
		memset(&accel, 0, sizeof(accel));

		accel.timestamp = timestamp;
		accel.x_raw = hil_state.xacc / CONSTANTS_ONE_G * 1e3f;
		accel.y_raw = hil_state.yacc / CONSTANTS_ONE_G * 1e3f;
		accel.z_raw = hil_state.zacc / CONSTANTS_ONE_G * 1e3f;
		accel.x = hil_state.xacc;
		accel.y = hil_state.yacc;
		accel.z = hil_state.zacc;
		accel.temperature = 25.0f;

		if (_accel_pub == nullptr) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* battery status */
	{
		struct battery_status_s	hil_battery_status;
		memset(&hil_battery_status, 0, sizeof(hil_battery_status));

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub == nullptr) {
			_battery_pub = orb_advertise(ORB_ID(battery_status), &hil_battery_status);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_pub, &hil_battery_status);
		}
	}
}

/**
 * Receive data from UART.
 */
void *
MavlinkReceiver::receive_thread(void *arg)
{

	/* set thread name */
	char thread_name[24];
	sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
	px4_prctl(PR_SET_NAME, thread_name, getpid());

	const int timeout = 500;
#ifdef __PX4_POSIX
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1600 * 5];
#else
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[64];
#endif
	mavlink_message_t msg;

	struct pollfd fds[1];

	int uart_fd = -1;

	if (_mavlink->get_protocol() == SERIAL) {
		uart_fd = _mavlink->get_uart_fd();

		fds[0].fd = uart_fd;
		fds[0].events = POLLIN;
	}
#ifdef __PX4_POSIX
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink->get_protocol() == UDP || _mavlink->get_protocol() == TCP) {
		// make sure mavlink app has booted before we start using the socket
		while (!_mavlink->boot_complete()) {
			usleep(100000);
		}

		fds[0].fd = _mavlink->get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif
	ssize_t nread = 0;

	while (!_mavlink->_task_should_exit) {
		if (poll(&fds[0], 1, timeout) > 0) {
			if (_mavlink->get_protocol() == SERIAL) {

				/*
				 * to avoid reading very small chunks wait for data before reading
				 * this is designed to target one message, so >20 bytes at a time
				 */
				const unsigned character_count = 20;

				/* non-blocking read. read may return negative values */
				if ((nread = ::read(uart_fd, buf, sizeof(buf))) < (ssize_t)character_count) {
					unsigned sleeptime = (1.0f / (_mavlink->get_baudrate() / 10)) * character_count * 1000000;
					usleep(sleeptime);
				}
			}
#ifdef __PX4_POSIX
			if (_mavlink->get_protocol() == UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink->get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}
			} else {
				// could be TCP or other protocol
			}

			struct sockaddr_in * srcaddr_last = _mavlink->get_client_source_address();
			int localhost = (127 << 24) + 1;
			if (!_mavlink->get_client_source_initialized()) {

				// set the address either if localhost or if 3 seconds have passed
				// this ensures that a GCS running on localhost can get a hold of
				// the system within the first N seconds
				hrt_abstime stime = _mavlink->get_start_time();
				if ((stime != 0 && (hrt_elapsed_time(&stime) > 3 * 1000 * 1000))
					|| (srcaddr_last->sin_addr.s_addr == htonl(localhost))) {
					srcaddr_last->sin_addr.s_addr = srcaddr.sin_addr.s_addr;
					srcaddr_last->sin_port = srcaddr.sin_port;
					_mavlink->set_client_source_initialized();
					warnx("changing partner IP to: %s", inet_ntoa(srcaddr.sin_addr));
				}
			}
#endif
			// only start accepting messages once we're sure who we talk to

			if (_mavlink->get_client_source_initialized()) {
				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {
						/* handle generic messages and commands */
						handle_message(&msg);

						/* handle packet with parent object */
						_mavlink->handle_message(&msg);
					}
				}

				/* count received bytes (nread will be -1 on read error) */
				if (nread > 0) {
					_mavlink->count_rxbytes(nread);
				}
			}
		}
	}

	return nullptr;
}

void MavlinkReceiver::print_status()
{

}

uint64_t MavlinkReceiver::sync_stamp(uint64_t usec)
{
	if(_time_offset > 0)
		return usec - (_time_offset / 1000) ;
	else
		return hrt_absolute_time();
}


void MavlinkReceiver::smooth_time_offset(uint64_t offset_ns)
{
	/* alpha = 0.6 fixed for now. The closer alpha is to 1.0,
         * the faster the moving average updates in response to
	 * new offset samples.
	 */

 	_time_offset = (_time_offset_avg_alpha * offset_ns) + (1.0 - _time_offset_avg_alpha) * _time_offset;
}


void *MavlinkReceiver::start_helper(void *context)
{

	MavlinkReceiver *rcv = new MavlinkReceiver((Mavlink *)context);

	void* ret = rcv->receive_thread(NULL);

	delete rcv;

	return ret;
}

void
MavlinkReceiver::receive_start(pthread_t *thread, Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

#ifndef __PX4_POSIX
	// set to non-blocking read
	int flags = fcntl(parent->get_uart_fd(), F_GETFL, 0);
	fcntl(parent->get_uart_fd(), F_SETFL, flags | O_NONBLOCK);
#endif

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2100);
	pthread_create(thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
}
