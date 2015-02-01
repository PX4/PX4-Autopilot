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
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

/* XXX trim includes */
#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_range_finder.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <mathlib/mathlib.h>

#include <conversion/rotation.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/airspeed.h>
#include <mavlink/mavlink_log.h>
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
	_global_pos_pub(-1),
	_local_pos_pub(-1),
	_attitude_pub(-1),
	_gps_pub(-1),
	_sensors_pub(-1),
	_gyro_pub(-1),
	_accel_pub(-1),
	_mag_pub(-1),
	_baro_pub(-1),
	_airspeed_pub(-1),
	_battery_pub(-1),
	_cmd_pub(-1),
	_flow_pub(-1),
	_range_pub(-1),
	_offboard_control_sp_pub(-1),
	_global_vel_sp_pub(-1),
	_att_sp_pub(-1),
	_rates_sp_pub(-1),
	_force_sp_pub(-1),
	_pos_sp_triplet_pub(-1),
	_vicon_position_pub(-1),
	_vision_position_pub(-1),
	_telemetry_status_pub(-1),
	_rc_pub(-1),
	_manual_pub(-1),
	_land_detector_pub(-1),
	_control_mode_sub(orb_subscribe(ORB_ID(vehicle_control_mode))),
	_hil_frames(0),
	_old_timestamp(0),
	_hil_local_proj_inited(0),
	_hil_local_alt0(0.0f),
	_hil_local_proj_ref{},
	_time_offset_avg_alpha(0.6),
	_time_offset(0)
{

	// make sure the FTP server is started
	(void)MavlinkFTP::get_server();
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

	case MAVLINK_MSG_ID_SET_MODE:
		handle_message_set_mode(msg);
		break;

	case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
		handle_message_vicon_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		handle_message_set_position_target_local_ned(msg);
		break;

	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		handle_message_set_attitude_target(msg);
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

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		handle_message_request_data_stream(msg);
		break;

	case MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL:
		MavlinkFTP::get_server()->handle_message(_mavlink, msg);
		break;

	case MAVLINK_MSG_ID_SYSTEM_TIME:
		handle_message_system_time(msg);
		break;

	case MAVLINK_MSG_ID_TIMESYNC:
		handle_message_timesync(msg);
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

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	if (cmd_mavlink.target_system == mavlink_system.sysid && ((cmd_mavlink.target_component == mavlink_system.compid)
			|| (cmd_mavlink.target_component == MAV_COMP_ID_ALL))) {
		//check for MAVLINK terminate command
		if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
			/* This is the link shutdown command, terminate mavlink */
			warnx("terminated by remote");
			fflush(stdout);
			usleep(50000);

			/* terminate other threads and this thread */
			_mavlink->_task_should_exit = true;

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
			vcmd.command = (enum VEHICLE_CMD)cmd_mavlink.command;
			vcmd.target_system = cmd_mavlink.target_system;
			vcmd.target_component = cmd_mavlink.target_component;
			vcmd.source_system = msg->sysid;
			vcmd.source_component = msg->compid;
			vcmd.confirmation =  cmd_mavlink.confirmation;

			if (_cmd_pub < 0) {
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

	if (cmd_mavlink.target_system == mavlink_system.sysid && ((cmd_mavlink.target_component == mavlink_system.compid)
			|| (cmd_mavlink.target_component == MAV_COMP_ID_ALL))) {
		//check for MAVLINK terminate command
		if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
			/* This is the link shutdown command, terminate mavlink */
			warnx("terminated by remote");
			fflush(stdout);
			usleep(50000);

			/* terminate other threads and this thread */
			_mavlink->_task_should_exit = true;

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
			vcmd.command = (enum VEHICLE_CMD)cmd_mavlink.command;
			vcmd.target_system = cmd_mavlink.target_system;
			vcmd.target_component = cmd_mavlink.target_component;
			vcmd.source_system = msg->sysid;
			vcmd.source_component = msg->compid;

			if (_cmd_pub < 0) {
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

	if (_flow_pub < 0) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
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

	if (_flow_pub < 0) {
		_flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

	} else {
		orb_publish(ORB_ID(optical_flow), _flow_pub, &f);
	}

	/* Use distance value for range finder report */
	struct range_finder_report r;
	memset(&r, 0, sizeof(r));

	r.timestamp = hrt_absolute_time();
	r.error_count = 0;
	r.type = RANGE_FINDER_TYPE_LASER;
	r.distance = flow.distance;
	r.minimum_distance = 0.0f;
	r.maximum_distance = 40.0f; // this is set to match the typical range of real sensors, could be made configurable
	r.valid = (r.distance > r.minimum_distance) && (r.distance < r.maximum_distance);

	if (_range_pub < 0) {
		_range_pub = orb_advertise(ORB_ID(sensor_range_finder), &r);
	} else {
		orb_publish(ORB_ID(sensor_range_finder), _range_pub, &r);
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
	vcmd.param3 = 0;
	vcmd.param4 = 0;
	vcmd.param5 = 0;
	vcmd.param6 = 0;
	vcmd.param7 = 0;
	vcmd.command = VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = 1;

	if (_cmd_pub < 0) {
		_cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

	} else {
		orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vcmd);
	}
}

void
MavlinkReceiver::handle_message_vicon_position_estimate(mavlink_message_t *msg)
{
	mavlink_vicon_position_estimate_t pos;
	mavlink_msg_vicon_position_estimate_decode(msg, &pos);

	struct vehicle_vicon_position_s vicon_position;
	memset(&vicon_position, 0, sizeof(vicon_position));

	vicon_position.timestamp = hrt_absolute_time();
	vicon_position.x = pos.x;
	vicon_position.y = pos.y;
	vicon_position.z = pos.z;
	vicon_position.roll = pos.roll;
	vicon_position.pitch = pos.pitch;
	vicon_position.yaw = pos.yaw;

	if (_vicon_position_pub < 0) {
		_vicon_position_pub = orb_advertise(ORB_ID(vehicle_vicon_position), &vicon_position);

	} else {
		orb_publish(ORB_ID(vehicle_vicon_position), _vicon_position_pub, &vicon_position);
	}
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t set_position_target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

	struct offboard_control_setpoint_s offboard_control_sp;
	memset(&offboard_control_sp, 0, sizeof(offboard_control_sp));//XXX breaks compatibility with multiple setpoints

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_position_target_local_ned.target_system ||
				set_position_target_local_ned.target_system == 0) &&
			(mavlink_system.compid == set_position_target_local_ned.target_component ||
			 set_position_target_local_ned.target_component == 0)) {

		/* convert mavlink type (local, NED) to uORB offboard control struct */
		switch (set_position_target_local_ned.coordinate_frame) {
			case MAV_FRAME_LOCAL_NED:
				offboard_control_sp.mode = OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED;
				break;
			case MAV_FRAME_LOCAL_OFFSET_NED:
				offboard_control_sp.mode = OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_OFFSET_NED;
				break;
			case MAV_FRAME_BODY_NED:
				offboard_control_sp.mode = OFFBOARD_CONTROL_MODE_DIRECT_BODY_NED;
				break;
			case MAV_FRAME_BODY_OFFSET_NED:
				offboard_control_sp.mode = OFFBOARD_CONTROL_MODE_DIRECT_BODY_OFFSET_NED;
				break;
			default:
				/* invalid setpoint, avoid publishing */
				return;
		}
		offboard_control_sp.position[0] = set_position_target_local_ned.x;
		offboard_control_sp.position[1] = set_position_target_local_ned.y;
		offboard_control_sp.position[2] = set_position_target_local_ned.z;
		offboard_control_sp.velocity[0] = set_position_target_local_ned.vx;
		offboard_control_sp.velocity[1] = set_position_target_local_ned.vy;
		offboard_control_sp.velocity[2] = set_position_target_local_ned.vz;
		offboard_control_sp.acceleration[0] = set_position_target_local_ned.afx;
		offboard_control_sp.acceleration[1] = set_position_target_local_ned.afy;
		offboard_control_sp.acceleration[2] = set_position_target_local_ned.afz;
		offboard_control_sp.yaw = set_position_target_local_ned.yaw;
		offboard_control_sp.yaw_rate = set_position_target_local_ned.yaw_rate;
		offboard_control_sp.isForceSetpoint = (bool)(set_position_target_local_ned.type_mask & (1 << 9));

		/* If we are in force control mode, for now set offboard mode to force control */
		if (offboard_control_sp.isForceSetpoint) {
			offboard_control_sp.mode = OFFBOARD_CONTROL_MODE_DIRECT_FORCE;
		}

		/* set ignore flags */
		for (int i = 0; i < 9; i++) {
			offboard_control_sp.ignore &=  ~(1 << i);
			offboard_control_sp.ignore |=  (set_position_target_local_ned.type_mask & (1 << i));
		}

		offboard_control_sp.ignore &=  ~(1 << OFB_IGN_BIT_YAW);
		if (set_position_target_local_ned.type_mask & (1 << 10)) {
			offboard_control_sp.ignore |=  (1 << OFB_IGN_BIT_YAW);
		}

		offboard_control_sp.ignore &=  ~(1 << OFB_IGN_BIT_YAWRATE);
		if (set_position_target_local_ned.type_mask & (1 << 11)) {
			offboard_control_sp.ignore |=  (1 << OFB_IGN_BIT_YAWRATE);
		}

		offboard_control_sp.timestamp = hrt_absolute_time();

		if (_offboard_control_sp_pub < 0) {
			_offboard_control_sp_pub = orb_advertise(ORB_ID(offboard_control_setpoint), &offboard_control_sp);

		} else {
			orb_publish(ORB_ID(offboard_control_setpoint), _offboard_control_sp_pub, &offboard_control_sp);
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
				if (offboard_control_sp.isForceSetpoint &&
						offboard_control_sp_ignore_position_all(offboard_control_sp) &&
						offboard_control_sp_ignore_velocity_all(offboard_control_sp)) {
					/* The offboard setpoint is a force setpoint only, directly writing to the force
					 * setpoint topic and not publishing the setpoint triplet topic */
					struct vehicle_force_setpoint_s	force_sp;
					force_sp.x = offboard_control_sp.acceleration[0];
					force_sp.y = offboard_control_sp.acceleration[1];
					force_sp.z = offboard_control_sp.acceleration[2];
					//XXX: yaw
					if (_force_sp_pub < 0) {
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
					pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_POSITION; //XXX support others

					/* set the local pos values if the setpoint type is 'local pos' and none
					 * of the local pos fields is set to 'ignore' */
					if (offboard_control_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED &&
							!offboard_control_sp_ignore_position_some(offboard_control_sp)) {
					pos_sp_triplet.current.position_valid = true;
					pos_sp_triplet.current.x = offboard_control_sp.position[0];
					pos_sp_triplet.current.y = offboard_control_sp.position[1];
					pos_sp_triplet.current.z = offboard_control_sp.position[2];
					} else {
						pos_sp_triplet.current.position_valid = false;
					}

					/* set the local vel values if the setpoint type is 'local pos' and none
					 * of the local vel fields is set to 'ignore' */
					if (offboard_control_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED &&
							!offboard_control_sp_ignore_velocity_some(offboard_control_sp)) {
						pos_sp_triplet.current.velocity_valid = true;
						pos_sp_triplet.current.vx = offboard_control_sp.velocity[0];
						pos_sp_triplet.current.vy = offboard_control_sp.velocity[1];
						pos_sp_triplet.current.vz = offboard_control_sp.velocity[2];
					} else {
						pos_sp_triplet.current.velocity_valid = false;
					}

					/* set the local acceleration values if the setpoint type is 'local pos' and none
					 * of the accelerations fields is set to 'ignore' */
					if (offboard_control_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED &&
							!offboard_control_sp_ignore_acceleration_some(offboard_control_sp)) {
						pos_sp_triplet.current.acceleration_valid = true;
						pos_sp_triplet.current.a_x = offboard_control_sp.acceleration[0];
						pos_sp_triplet.current.a_y = offboard_control_sp.acceleration[1];
						pos_sp_triplet.current.a_z = offboard_control_sp.acceleration[2];
						pos_sp_triplet.current.acceleration_is_force =
						offboard_control_sp.isForceSetpoint;

					} else {
						pos_sp_triplet.current.acceleration_valid = false;
					}

					/* set the yaw sp value if the setpoint type is 'local pos' and the yaw
					 * field is not set to 'ignore' */
					if (offboard_control_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED &&
							!offboard_control_sp_ignore_yaw(offboard_control_sp)) {
						pos_sp_triplet.current.yaw_valid = true;
						pos_sp_triplet.current.yaw = offboard_control_sp.yaw;

					} else {
						pos_sp_triplet.current.yaw_valid = false;
					}

					/* set the yawrate sp value if the setpoint type is 'local pos' and the yawrate
					 * field is not set to 'ignore' */
					if (offboard_control_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_LOCAL_NED &&
							!offboard_control_sp_ignore_yawrate(offboard_control_sp)) {
						pos_sp_triplet.current.yawspeed_valid = true;
						pos_sp_triplet.current.yawspeed = offboard_control_sp.yaw_rate;

					} else {
						pos_sp_triplet.current.yawspeed_valid = false;
					}
					//XXX handle global pos setpoints (different MAV frames)


					if (_pos_sp_triplet_pub < 0) {
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
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t pos;
	mavlink_msg_vision_position_estimate_decode(msg, &pos);

	struct vision_position_estimate vision_position;
	memset(&vision_position, 0, sizeof(vision_position));

	// Use the component ID to identify the vision sensor
	vision_position.id = msg->compid;

	vision_position.timestamp_boot = to_hrt(pos.usec); // Synced time
	vision_position.timestamp_computer = pos.usec;
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

	if (_vision_position_pub < 0) {
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

	struct offboard_control_setpoint_s offboard_control_sp;
	memset(&offboard_control_sp, 0, sizeof(offboard_control_sp)); //XXX breaks compatibility with multiple setpoints

	/* Only accept messages which are intended for this system */
	if ((mavlink_system.sysid == set_attitude_target.target_system ||
				set_attitude_target.target_system == 0) &&
			(mavlink_system.compid == set_attitude_target.target_component ||
			 set_attitude_target.target_component == 0)) {
		for (int i = 0; i < 4; i++) {
			offboard_control_sp.attitude[i] = set_attitude_target.q[i];
		}
		offboard_control_sp.attitude_rate[0] = set_attitude_target.body_roll_rate;
		offboard_control_sp.attitude_rate[1] = set_attitude_target.body_pitch_rate;
		offboard_control_sp.attitude_rate[2] = set_attitude_target.body_yaw_rate;

		/* set correct ignore flags for body rate fields: copy from mavlink message */
		for (int i = 0; i < 3; i++) {
			offboard_control_sp.ignore &=  ~(1 << (i + OFB_IGN_BIT_BODYRATE_X));
			offboard_control_sp.ignore |=  (set_attitude_target.type_mask & (1 << i)) << OFB_IGN_BIT_BODYRATE_X;
		}
		/* set correct ignore flags for thrust field: copy from mavlink message */
		offboard_control_sp.ignore &= ~(1 << OFB_IGN_BIT_THRUST);
		offboard_control_sp.ignore |= ((set_attitude_target.type_mask & (1 << 6)) << OFB_IGN_BIT_THRUST);
		/* set correct ignore flags for attitude field: copy from mavlink message */
		offboard_control_sp.ignore &= ~(1 << OFB_IGN_BIT_ATT);
		offboard_control_sp.ignore |= ((set_attitude_target.type_mask & (1 << 7)) << OFB_IGN_BIT_ATT);


		offboard_control_sp.timestamp = hrt_absolute_time();
		offboard_control_sp.mode =OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE; //XXX handle rate control mode

		if (_offboard_control_sp_pub < 0) {
			_offboard_control_sp_pub = orb_advertise(ORB_ID(offboard_control_setpoint), &offboard_control_sp);

		} else {
			orb_publish(ORB_ID(offboard_control_setpoint), _offboard_control_sp_pub, &offboard_control_sp);
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
				if (!(offboard_control_sp_ignore_attitude(offboard_control_sp) ||
							offboard_control_sp_ignore_thrust(offboard_control_sp))) {
					struct vehicle_attitude_setpoint_s att_sp;
					att_sp.timestamp = hrt_absolute_time();
					mavlink_quaternion_to_euler(set_attitude_target.q,
							&att_sp.roll_body, &att_sp.pitch_body, &att_sp.yaw_body);
					mavlink_quaternion_to_dcm(set_attitude_target.q, (float(*)[3])att_sp.R_body);
					att_sp.R_valid = true;
					att_sp.thrust = set_attitude_target.thrust;
					memcpy(att_sp.q_d, set_attitude_target.q, sizeof(att_sp.q_d));
					att_sp.q_d_valid = true;
					if (_att_sp_pub < 0) {
						_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
					} else {
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &att_sp);
					}
				}

				/* Publish attitude rate setpoint if bodyrate and thrust ignore bits are not set */
				///XXX add support for ignoring individual axes
				if (!(offboard_control_sp_ignore_bodyrates_some(offboard_control_sp) ||
							offboard_control_sp_ignore_thrust(offboard_control_sp))) {
					struct vehicle_rates_setpoint_s rates_sp;
					rates_sp.timestamp = hrt_absolute_time();
					rates_sp.roll = set_attitude_target.body_roll_rate;
					rates_sp.pitch = set_attitude_target.body_pitch_rate;
					rates_sp.yaw = set_attitude_target.body_yaw_rate;
					rates_sp.thrust = set_attitude_target.thrust;

					if (_att_sp_pub < 0) {
						_rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);
					} else {
						orb_publish(ORB_ID(vehicle_rates_setpoint), _rates_sp_pub, &rates_sp);
					}
				}
			}

		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < TELEMETRY_STATUS_ORB_ID_NUM) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		struct telemetry_status_s &tstatus = _mavlink->get_rx_status();

		tstatus.timestamp = hrt_absolute_time();
		tstatus.telem_time = tstatus.timestamp;
		/* tstatus.heartbeat_time is set by system heartbeats */
		tstatus.type = TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO;
		tstatus.rssi = rstatus.rssi;
		tstatus.remote_rssi = rstatus.remrssi;
		tstatus.txbuf = rstatus.txbuf;
		tstatus.noise = rstatus.noise;
		tstatus.remote_noise = rstatus.remnoise;
		tstatus.rxerrors = rstatus.rxerrors;
		tstatus.fixed = rstatus.fixed;
		tstatus.system_id = msg->sysid;
		tstatus.component_id = msg->compid;

		if (_telemetry_status_pub < 0) {
			_telemetry_status_pub = orb_advertise(telemetry_status_orb_id[_mavlink->get_channel()], &tstatus);

		} else {
			orb_publish(telemetry_status_orb_id[_mavlink->get_channel()], _telemetry_status_pub, &tstatus);
		}
	}
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t man;
	mavlink_msg_manual_control_decode(msg, &man);

	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));

	manual.timestamp = hrt_absolute_time();
	manual.x = man.x / 1000.0f;
	manual.y = man.y / 1000.0f;
	manual.r = man.r / 1000.0f;
	manual.z = man.z / 1000.0f;

	// warnx("pitch: %.2f, roll: %.2f, yaw: %.2f, throttle: %.2f", (double)manual.x, (double)manual.y, (double)manual.r, (double)manual.z);

	if (_manual_pub < 0) {
		_manual_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);

	} else {
		orb_publish(ORB_ID(manual_control_setpoint), _manual_pub, &manual);
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < TELEMETRY_STATUS_ORB_ID_NUM) {
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

			if (_telemetry_status_pub < 0) {
				_telemetry_status_pub = orb_advertise(telemetry_status_orb_id[_mavlink->get_channel()], &tstatus);

			} else {
				orb_publish(telemetry_status_orb_id[_mavlink->get_channel()], _telemetry_status_pub, &tstatus);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_request_data_stream(mavlink_message_t *msg)
{
	mavlink_request_data_stream_t req;
	mavlink_msg_request_data_stream_decode(msg, &req);

	if (req.target_system == mavlink_system.sysid && req.target_component == mavlink_system.compid) {
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
	clock_gettime(CLOCK_REALTIME, &tv);

	// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
	bool onb_unix_valid = tv.tv_sec > PX4_EPOCH_SECS;
	bool ofb_unix_valid = time.time_unix_usec > PX4_EPOCH_SECS * 1000ULL;

	if (!onb_unix_valid && ofb_unix_valid) {
		tv.tv_sec = time.time_unix_usec / 1000000ULL;
		tv.tv_nsec = (time.time_unix_usec % 1000000ULL) * 1000ULL;
		if(clock_settime(CLOCK_REALTIME, &tv)) {
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

}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	uint64_t timestamp = hrt_absolute_time();

	/* airspeed */
	{
		struct airspeed_s airspeed;
		memset(&airspeed, 0, sizeof(airspeed));

		float ias = calc_indicated_airspeed(imu.diff_pressure * 1e2f);
		// XXX need to fix this
		float tas = ias;

		airspeed.timestamp = timestamp;
		airspeed.indicated_airspeed_m_s = ias;
		airspeed.true_airspeed_m_s = tas;

		if (_airspeed_pub < 0) {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &airspeed);

		} else {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &airspeed);
		}
	}

	/* gyro */
	{
		struct gyro_report gyro;
		memset(&gyro, 0, sizeof(gyro));

		gyro.timestamp = timestamp;
		gyro.x_raw = imu.xgyro * 1000.0f;
		gyro.y_raw = imu.ygyro * 1000.0f;
		gyro.z_raw = imu.zgyro * 1000.0f;
		gyro.x = imu.xgyro;
		gyro.y = imu.ygyro;
		gyro.z = imu.zgyro;
		gyro.temperature = imu.temperature;

		if (_gyro_pub < 0) {
			_gyro_pub = orb_advertise(ORB_ID(sensor_gyro), &gyro);

		} else {
			orb_publish(ORB_ID(sensor_gyro), _gyro_pub, &gyro);
		}
	}

	/* accelerometer */
	{
		struct accel_report accel;
		memset(&accel, 0, sizeof(accel));

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / mg2ms2;
		accel.y_raw = imu.yacc / mg2ms2;
		accel.z_raw = imu.zacc / mg2ms2;
		accel.x = imu.xacc;
		accel.y = imu.yacc;
		accel.z = imu.zacc;
		accel.temperature = imu.temperature;

		if (_accel_pub < 0) {
			_accel_pub = orb_advertise(ORB_ID(sensor_accel), &accel);

		} else {
			orb_publish(ORB_ID(sensor_accel), _accel_pub, &accel);
		}
	}

	/* magnetometer */
	{
		struct mag_report mag;
		memset(&mag, 0, sizeof(mag));

		mag.timestamp = timestamp;
		mag.x_raw = imu.xmag * 1000.0f;
		mag.y_raw = imu.ymag * 1000.0f;
		mag.z_raw = imu.zmag * 1000.0f;
		mag.x = imu.xmag;
		mag.y = imu.ymag;
		mag.z = imu.zmag;

		if (_mag_pub < 0) {
			/* publish to the first mag topic */
			_mag_pub = orb_advertise(ORB_ID(sensor_mag), &mag);

		} else {
			orb_publish(ORB_ID(sensor_mag), _mag_pub, &mag);
		}
	}

	/* baro */
	{
		struct baro_report baro;
		memset(&baro, 0, sizeof(baro));

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.altitude = imu.pressure_alt;
		baro.temperature = imu.temperature;

		if (_baro_pub < 0) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
		}
	}

	/* sensor combined */
	{
		struct sensor_combined_s hil_sensors;
		memset(&hil_sensors, 0, sizeof(hil_sensors));

		hil_sensors.timestamp = timestamp;

		hil_sensors.gyro_raw[0] = imu.xgyro * 1000.0f;
		hil_sensors.gyro_raw[1] = imu.ygyro * 1000.0f;
		hil_sensors.gyro_raw[2] = imu.zgyro * 1000.0f;
		hil_sensors.gyro_rad_s[0] = imu.xgyro;
		hil_sensors.gyro_rad_s[1] = imu.ygyro;
		hil_sensors.gyro_rad_s[2] = imu.zgyro;

		hil_sensors.accelerometer_raw[0] = imu.xacc / mg2ms2;
		hil_sensors.accelerometer_raw[1] = imu.yacc / mg2ms2;
		hil_sensors.accelerometer_raw[2] = imu.zacc / mg2ms2;
		hil_sensors.accelerometer_m_s2[0] = imu.xacc;
		hil_sensors.accelerometer_m_s2[1] = imu.yacc;
		hil_sensors.accelerometer_m_s2[2] = imu.zacc;
		hil_sensors.accelerometer_mode = 0; // TODO what is this?
		hil_sensors.accelerometer_range_m_s2 = 32.7f; // int16
		hil_sensors.accelerometer_timestamp = timestamp;

		hil_sensors.adc_voltage_v[0] = 0.0f;
		hil_sensors.adc_voltage_v[1] = 0.0f;
		hil_sensors.adc_voltage_v[2] = 0.0f;

		hil_sensors.magnetometer_raw[0] = imu.xmag * 1000.0f;
		hil_sensors.magnetometer_raw[1] = imu.ymag * 1000.0f;
		hil_sensors.magnetometer_raw[2] = imu.zmag * 1000.0f;
		hil_sensors.magnetometer_ga[0] = imu.xmag;
		hil_sensors.magnetometer_ga[1] = imu.ymag;
		hil_sensors.magnetometer_ga[2] = imu.zmag;
		hil_sensors.magnetometer_range_ga = 32.7f; // int16
		hil_sensors.magnetometer_mode = 0; // TODO what is this
		hil_sensors.magnetometer_cuttoff_freq_hz = 50.0f;
		hil_sensors.magnetometer_timestamp = timestamp;

		hil_sensors.baro_pres_mbar = imu.abs_pressure;
		hil_sensors.baro_alt_meter = imu.pressure_alt;
		hil_sensors.baro_temp_celcius = imu.temperature;
		hil_sensors.baro_timestamp = timestamp;

		hil_sensors.differential_pressure_pa = imu.diff_pressure * 1e2f; //from hPa to Pa
		hil_sensors.differential_pressure_timestamp = timestamp;

		/* publish combined sensor topic */
		if (_sensors_pub < 0) {
			_sensors_pub = orb_advertise(ORB_ID(sensor_combined), &hil_sensors);

		} else {
			orb_publish(ORB_ID(sensor_combined), _sensors_pub, &hil_sensors);
		}
	}

	/* battery status */
	{
		struct battery_status_s hil_battery_status;
		memset(&hil_battery_status, 0, sizeof(hil_battery_status));

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;

		if (_battery_pub < 0) {
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

	if (_gps_pub < 0) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
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

		if (_airspeed_pub < 0) {
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

		if (_attitude_pub < 0) {
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
		hil_global_pos.lat = hil_state.lat;
		hil_global_pos.lon = hil_state.lon;
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.vel_n = hil_state.vx / 100.0f;
		hil_global_pos.vel_e = hil_state.vy / 100.0f;
		hil_global_pos.vel_d = hil_state.vz / 100.0f;
		hil_global_pos.yaw = hil_attitude.yaw;
		hil_global_pos.eph = 2.0f;
		hil_global_pos.epv = 4.0f;

		if (_global_pos_pub < 0) {
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
			map_projection_init(&_hil_local_proj_ref, hil_state.lat, hil_state.lon);
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

		if (_local_pos_pub < 0) {
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

			if (_land_detector_pub < 0) {
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

		if (_accel_pub < 0) {
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

		if (_battery_pub < 0) {
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
	int uart_fd = _mavlink->get_uart_fd();

	const int timeout = 500;
	uint8_t buf[32];

	mavlink_message_t msg;

	/* set thread name */
	char thread_name[24];
	sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
	prctl(PR_SET_NAME, thread_name, getpid());

	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	ssize_t nread = 0;

	while (!_mavlink->_task_should_exit) {
		if (poll(fds, 1, timeout) > 0) {

			/* non-blocking read. read may return negative values */
			if ((nread = read(uart_fd, buf, sizeof(buf))) < (ssize_t)sizeof(buf)) {
				/* to avoid reading very small chunks wait for data before reading */
				usleep(1000);
			}

			/* if read failed, this loop won't execute */
			for (ssize_t i = 0; i < nread; i++) {
				if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &status)) {
					/* handle generic messages and commands */
					handle_message(&msg);

					/* handle packet with parent object */
					_mavlink->handle_message(&msg);
				}
			}

			/* count received bytes */
			_mavlink->count_rxbytes(nread);
		}
	}

	return NULL;
}

void MavlinkReceiver::print_status()
{

}

uint64_t MavlinkReceiver::to_hrt(uint64_t usec)
{
	return usec - (_time_offset / 1000) ;
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

	rcv->receive_thread(NULL);

	delete rcv;

	return nullptr;
}

pthread_t
MavlinkReceiver::receive_start(Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	// set to non-blocking read
	int flags = fcntl(parent->get_uart_fd(), F_GETFL, 0);
	fcntl(parent->get_uart_fd(), F_SETFL, flags | O_NONBLOCK);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2900);
	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
	return thread;
}
