/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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

#include <airspeed/airspeed.h>
#include <commander/px4_custom_mode.h>
#include <conversion/rotation.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_range_finder.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_tone_alarm.h>
#include <ecl/geo/geo.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifdef __PX4_DARWIN
#include <sys/param.h>
#include <sys/mount.h>
#else
#include <sys/statfs.h>
#endif

#ifndef __PX4_POSIX
#include <termios.h>
#endif

#include "mavlink_command_sender.h"
#include "mavlink_main.h"
#include "mavlink_receiver.h"

#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

using matrix::wrap_2pi;

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	_mavlink(parent),
	_mavlink_ftp(parent),
	_mavlink_log_handler(parent),
	_mavlink_timesync(parent),
	_mission_manager(parent),
	_parameters_manager(parent),
	_p_bat_emergen_thr(param_find("BAT_EMERGEN_THR")),
	_p_bat_crit_thr(param_find("BAT_CRIT_THR")),
	_p_bat_low_thr(param_find("BAT_LOW_THR")),
	_p_flow_rot(param_find("SENS_FLOW_ROT")),
	_p_flow_maxr(param_find("SENS_FLOW_MAXR")),
	_p_flow_minhgt(param_find("SENS_FLOW_MINHGT")),
	_p_flow_maxhgt(param_find("SENS_FLOW_MAXHGT"))
{
	/* Make the attitude quaternion valid */
	_att.q[0] = 1.0f;
}

MavlinkReceiver::~MavlinkReceiver()
{
	orb_unsubscribe(_control_mode_sub);
	orb_unsubscribe(_actuator_armed_sub);
	orb_unsubscribe(_vehicle_attitude_sub);
}

void
MavlinkReceiver::acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result)
{
	vehicle_command_ack_s command_ack = {};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = command;
	command_ack.result = result;
	command_ack.target_system = sysid;
	command_ack.target_component = compid;

	if (_command_ack_pub == nullptr) {
		_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
						       vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);
	}
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

	case MAVLINK_MSG_ID_COMMAND_ACK:
		handle_message_command_ack(msg);
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

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:
		handle_message_gps_global_origin(msg);
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

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_FOLLOW_TARGET:
		handle_message_follow_target(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_ADSB_VEHICLE:
		handle_message_adsb_vehicle(msg);
		break;

	case MAVLINK_MSG_ID_COLLISION:
		handle_message_collision(msg);
		break;

	case MAVLINK_MSG_ID_GPS_RTCM_DATA:
		handle_message_gps_rtcm_data(msg);
		break;

	case MAVLINK_MSG_ID_BATTERY_STATUS:
		handle_message_battery_status(msg);
		break;

	case MAVLINK_MSG_ID_SERIAL_CONTROL:
		handle_message_serial_control(msg);
		break;

	case MAVLINK_MSG_ID_LOGGING_ACK:
		handle_message_logging_ack(msg);
		break;

	case MAVLINK_MSG_ID_PLAY_TUNE:
		handle_message_play_tune(msg);
		break;

	case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
		handle_message_obstacle_distance(msg);
		break;

	case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS:
		handle_message_trajectory_representation_waypoints(msg);
		break;

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		handle_message_named_value_float(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG:
		handle_message_debug(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_VECT:
		handle_message_debug_vect(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
		handle_message_debug_float_array(msg);
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
	case MAV_CMD_REQUEST_PROTOCOL_VERSION:
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
MavlinkReceiver::send_flight_information()
{
	mavlink_flight_information_t flight_info{};

	param_t param_flight_uuid = param_find("COM_FLIGHT_UUID");

	if (param_flight_uuid != PARAM_INVALID) {
		int32_t flight_uuid;
		param_get(param_flight_uuid, &flight_uuid);
		flight_info.flight_uuid = (uint64_t)flight_uuid;
	}

	actuator_armed_s actuator_armed;
	int ret = orb_copy(ORB_ID(actuator_armed), _actuator_armed_sub, &actuator_armed);

	if (ret == 0 && actuator_armed.timestamp != 0) {
		flight_info.arming_time_utc = flight_info.takeoff_time_utc = actuator_armed.armed_time_ms;
	}

	flight_info.time_boot_ms = hrt_absolute_time() / 1000;
	mavlink_msg_flight_information_send_struct(_mavlink->get_channel(), &flight_info);
}

void
MavlinkReceiver::send_storage_information(int storage_id)
{
	mavlink_storage_information_t storage_info{};
	const char *microsd_dir = PX4_STORAGEDIR;

	if (storage_id == 0 || storage_id == 1) { // request is for all or the first storage
		storage_info.storage_id = 1;

		struct statfs statfs_buf;
		uint64_t total_bytes = 0;
		uint64_t avail_bytes = 0;

		if (statfs(microsd_dir, &statfs_buf) == 0) {
			total_bytes = (uint64_t)statfs_buf.f_blocks * statfs_buf.f_bsize;
			avail_bytes = (uint64_t)statfs_buf.f_bavail * statfs_buf.f_bsize;
		}

		if (total_bytes == 0) { // on NuttX we get 0 total bytes if no SD card is inserted
			storage_info.storage_count = 0;
			storage_info.status = 0; // not available

		} else {
			storage_info.storage_count = 1;
			storage_info.status = 2; // available & formatted
			storage_info.total_capacity = total_bytes / 1024. / 1024.;
			storage_info.available_capacity = avail_bytes / 1024. / 1024.;
			storage_info.used_capacity = (total_bytes - avail_bytes) / 1024. / 1024.;
		}

	} else {
		// only one storage supported
		storage_info.storage_id = storage_id;
		storage_info.storage_count = 1;
	}

	storage_info.time_boot_ms = hrt_absolute_time() / 1000;
	mavlink_msg_storage_information_send_struct(_mavlink->get_channel(), &storage_info);
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = (double)cmd_mavlink.param5;
	vcmd.param6 = (double)cmd_mavlink.param6;
	vcmd.param7 = cmd_mavlink.param7;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = cmd_mavlink.confirmation;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

void
MavlinkReceiver::handle_message_command_int(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_int_t cmd_mavlink;
	mavlink_msg_command_int_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();

	/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
	vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
	vcmd.param7 = cmd_mavlink.z;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = false;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

template <class T>
void MavlinkReceiver::handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
		const vehicle_command_s &vehicle_command)
{
	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);

	bool send_ack = true;
	uint8_t result = vehicle_command_ack_s::VEHICLE_RESULT_ACCEPTED;

	if (!target_ok) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_RESULT_FAILED);
		return;
	}

	if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
		/* send autopilot version message */
		_mavlink->send_autopilot_capabilites();

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
		/* send protocol version message */
		_mavlink->send_protocol_version();

	} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
		_mavlink->configure_stream_threadsafe("HOME_POSITION", 0.5f);

	} else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
		if (set_message_interval((int)(cmd_mavlink.param1 + 0.5f), cmd_mavlink.param2, cmd_mavlink.param3)) {
			result = vehicle_command_ack_s::VEHICLE_RESULT_FAILED;
		}

	} else if (cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
		get_message_interval((int)cmd_mavlink.param1);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_FLIGHT_INFORMATION) {
		send_flight_information();

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
		if ((int)(cmd_mavlink.param2 + 0.5f) == 1) {
			send_storage_information(cmd_mavlink.param1 + 0.5f);
		}

	} else {

		send_ack = false;

		if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
			PX4_WARN("ignoring CMD with same SYS/COMP (%d/%d) ID", mavlink_system.sysid, mavlink_system.compid);
			return;
		}

		if (cmd_mavlink.command == MAV_CMD_LOGGING_START) {
			// check that we have enough bandwidth available: this is given by the configured logger topics
			// and rates. The 5000 is somewhat arbitrary, but makes sure that we cannot enable log streaming
			// on a radio link
			if (_mavlink->get_data_rate() < 5000) {
				send_ack = true;
				result = vehicle_command_ack_s::VEHICLE_RESULT_DENIED;
				_mavlink->send_statustext_critical("Not enough bandwidth to enable log streaming");

			} else {
				// we already instanciate the streaming object, because at this point we know on which
				// mavlink channel streaming was requested. But in fact it's possible that the logger is
				// not even running. The main mavlink thread takes care of this by waiting for an ack
				// from the logger.
				_mavlink->try_start_ulog_streaming(msg->sysid, msg->compid);
			}

		} else if (cmd_mavlink.command == MAV_CMD_LOGGING_STOP) {
			_mavlink->request_stop_ulog_streaming();
		}

		if (!send_ack) {
			if (_cmd_pub == nullptr) {
				_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vehicle_command, vehicle_command_s::ORB_QUEUE_LENGTH);

			} else {
				orb_publish(ORB_ID(vehicle_command), _cmd_pub, &vehicle_command);
			}
		}
	}

	if (send_ack) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, result);
	}
}

void
MavlinkReceiver::handle_message_command_ack(mavlink_message_t *msg)
{
	mavlink_command_ack_t ack;
	mavlink_msg_command_ack_decode(msg, &ack);

	MavlinkCommandSender::instance().handle_mavlink_command_ack(ack, msg->sysid, msg->compid);

	vehicle_command_ack_s command_ack = {};
	command_ack.timestamp = hrt_absolute_time();
	command_ack.result_param2 = ack.result_param2;
	command_ack.command = ack.command;
	command_ack.result = ack.result;
	command_ack.from_external = true;
	command_ack.result_param1 = ack.progress;
	command_ack.target_system = ack.target_system;
	command_ack.target_component = ack.target_component;

	if (_command_ack_pub == nullptr) {
		_command_ack_pub = orb_advertise_queue(ORB_ID(vehicle_command_ack), &command_ack,
						       vehicle_command_ack_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(vehicle_command_ack), _command_ack_pub, &command_ack);
	}

	// TODO: move it to the same place that sent the command
	if (ack.result != MAV_RESULT_ACCEPTED && ack.result != MAV_RESULT_IN_PROGRESS) {
		if (msg->compid == MAV_COMP_ID_CAMERA) {
			PX4_WARN("Got unsuccessful result %d from camera", ack.result);
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	/* optical flow */
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	/* read flow sensor parameters */
	int32_t flow_rot_int;
	param_get(_p_flow_rot, &flow_rot_int);
	const enum Rotation flow_rot = (Rotation)flow_rot_int;

	struct optical_flow_s f = {};

	f.timestamp = _mavlink_timesync.sync_stamp(flow.time_usec);
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
	param_get(_p_flow_maxr, &f.max_flow_rate);
	param_get(_p_flow_minhgt, &f.min_ground_distance);
	param_get(_p_flow_maxhgt, &f.max_ground_distance);

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
	struct distance_sensor_s d = {};

	if (flow.distance > 0.0f) { // negative values signal invalid data
		d.timestamp = f.timestamp;
		d.min_distance = 0.3f;
		d.max_distance = 5.0f;
		d.current_distance = flow.distance; /* both are in m */
		d.type = 1;
		d.id = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
		d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
		d.variance = 0.0;

		if (_flow_distance_sensor_pub == nullptr) {
			_flow_distance_sensor_pub = orb_advertise_multi(ORB_ID(distance_sensor), &d,
						    &_orb_class_instance, ORB_PRIO_HIGH);

		} else {
			orb_publish(ORB_ID(distance_sensor), _flow_distance_sensor_pub, &d);
		}
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
	d.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	d.id = 0;
	d.orientation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	d.variance = 0.0;

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

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;

	vehicle_command_s vcmd = {};
	vcmd.timestamp = hrt_absolute_time();

	/* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = (float)new_mode.base_mode;
	vcmd.param2 = (float)custom_mode.main_mode;
	vcmd.param3 = (float)custom_mode.sub_mode;

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = true;
	vcmd.from_external = true;

	if (_cmd_pub == nullptr) {
		_cmd_pub = orb_advertise_queue(ORB_ID(vehicle_command), &vcmd, vehicle_command_s::ORB_QUEUE_LENGTH);

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

	d.timestamp = hrt_absolute_time(); /* Use system time for now, don't trust sender to attach correct timestamp */
	d.min_distance = float(dist_sensor.min_distance) * 1e-2f; /* cm to m */
	d.max_distance = float(dist_sensor.max_distance) * 1e-2f; /* cm to m */
	d.current_distance = float(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	d.type = dist_sensor.type;
	d.id = 	MAV_DISTANCE_SENSOR_LASER;
	d.orientation = dist_sensor.orientation;
	d.variance = dist_sensor.covariance * 1e-4f; // cm^2 to m^2

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

	struct vehicle_odometry_s mocap_odom = {};

	mocap_odom.timestamp = _mavlink_timesync.sync_stamp(mocap.time_usec);
	mocap_odom.x = mocap.x;
	mocap_odom.y = mocap.y;
	mocap_odom.z = mocap.z;
	mocap_odom.q[0] = mocap.q[0];
	mocap_odom.q[1] = mocap.q[1];
	mocap_odom.q[2] = mocap.q[2];
	mocap_odom.q[3] = mocap.q[3];

	const size_t URT_SIZE = sizeof(mocap_odom.pose_covariance) / sizeof(mocap_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(mocap.covariance) / sizeof(mocap.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		mocap_odom.pose_covariance[i] = mocap.covariance[i];
	}

	mocap_odom.vx = NAN;
	mocap_odom.vy = NAN;
	mocap_odom.vz = NAN;
	mocap_odom.rollspeed = NAN;
	mocap_odom.pitchspeed = NAN;
	mocap_odom.yawspeed = NAN;
	mocap_odom.velocity_covariance[0] = NAN;

	int instance_id = 0;

	orb_publish_auto(ORB_ID(vehicle_mocap_odometry), &_mocap_odometry_pub, &mocap_odom, &instance_id, ORB_PRIO_HIGH);
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t set_position_target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &set_position_target_local_ned);

	struct offboard_control_mode_s offboard_control_mode = {};

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
		offboard_control_mode.ignore_alt_hold = (bool)(set_position_target_local_ned.type_mask & 0x4);
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

					PX4_WARN("force setpoint not supported");

				} else {
					/* It's not a pure force setpoint: publish to setpoint triplet  topic */
					struct position_setpoint_triplet_s pos_sp_triplet = {};
					pos_sp_triplet.timestamp = hrt_absolute_time();
					pos_sp_triplet.previous.valid = false;
					pos_sp_triplet.next.valid = false;
					pos_sp_triplet.current.valid = true;

					/* Order of statements matters. Takeoff can override loiter.
					 * See https://github.com/mavlink/mavlink/pull/670 for a broader conversation. */
					if (is_loiter_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LOITER;

					} else if (is_takeoff_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

					} else if (is_land_sp) {
						pos_sp_triplet.current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

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

						pos_sp_triplet.current.velocity_frame =
							set_position_target_local_ned.coordinate_frame;

					} else {
						pos_sp_triplet.current.velocity_valid = false;
					}

					if (!offboard_control_mode.ignore_alt_hold) {
						pos_sp_triplet.current.alt_valid = true;
						pos_sp_triplet.current.z = set_position_target_local_ned.z;

					} else {
						pos_sp_triplet.current.alt_valid = false;
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

	struct offboard_control_mode_s offboard_control_mode = {};

	struct actuator_controls_s actuator_controls = {};

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

			/* Set duty cycles for the servos in the actuator_controls message */
			for (size_t i = 0; i < 8; i++) {
				actuator_controls.control[i] = set_actuator_control_target.controls[i];
			}

			switch (set_actuator_control_target.group_mlx) {
			case 0:
				if (_actuator_controls_pubs[0] == nullptr) {
					_actuator_controls_pubs[0] = orb_advertise(ORB_ID(actuator_controls_0), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_0), _actuator_controls_pubs[0], &actuator_controls);
				}

				break;

			case 1:
				if (_actuator_controls_pubs[1] == nullptr) {
					_actuator_controls_pubs[1] = orb_advertise(ORB_ID(actuator_controls_1), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_1), _actuator_controls_pubs[1], &actuator_controls);
				}

				break;

			case 2:
				if (_actuator_controls_pubs[2] == nullptr) {
					_actuator_controls_pubs[2] = orb_advertise(ORB_ID(actuator_controls_2), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_2), _actuator_controls_pubs[2], &actuator_controls);
				}

				break;

			case 3:
				if (_actuator_controls_pubs[3] == nullptr) {
					_actuator_controls_pubs[3] = orb_advertise(ORB_ID(actuator_controls_3), &actuator_controls);

				} else {
					orb_publish(ORB_ID(actuator_controls_3), _actuator_controls_pubs[3], &actuator_controls);
				}

				break;

			default:
				break;
			}
		}
	}

}

void
MavlinkReceiver::handle_message_gps_global_origin(mavlink_message_t *msg)
{
	mavlink_gps_global_origin_t origin;
	mavlink_msg_gps_global_origin_decode(msg, &origin);

	if (!globallocalconverter_initialized()) {
		/* Set reference point conversion of local coordiantes <--> global coordinates */
		globallocalconverter_init((double)origin.latitude * 1.0e-7, (double)origin.longitude * 1.0e-7,
					  (float)origin.altitude * 1.0e-3f, hrt_absolute_time());
		_global_ref_timestamp = hrt_absolute_time();

	}
}

void
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t ev;
	mavlink_msg_vision_position_estimate_decode(msg, &ev);

	struct vehicle_odometry_s visual_odom = {};

	visual_odom.timestamp = _mavlink_timesync.sync_stamp(ev.usec);
	visual_odom.x = ev.x;
	visual_odom.y = ev.y;
	visual_odom.z = ev.z;
	matrix::Quatf q(matrix::Eulerf(ev.roll, ev.pitch, ev.yaw));
	q.copyTo(visual_odom.q);

	// TODO:
	// - add a MAV_FRAME_*_OTHER to the Mavlink MAV_FRAME enum IOT define
	// a frame of reference which is not aligned with NED or ENU
	// - add usage on the estimator side
	visual_odom.local_frame = visual_odom.LOCAL_FRAME_NED;

	const size_t URT_SIZE = sizeof(visual_odom.pose_covariance) / sizeof(visual_odom.pose_covariance[0]);
	static_assert(URT_SIZE == (sizeof(ev.covariance) / sizeof(ev.covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	for (size_t i = 0; i < URT_SIZE; i++) {
		visual_odom.pose_covariance[i] = ev.covariance[i];
	}

	visual_odom.vx = NAN;
	visual_odom.vy = NAN;
	visual_odom.vz = NAN;
	visual_odom.rollspeed = NAN;
	visual_odom.pitchspeed = NAN;
	visual_odom.yawspeed = NAN;
	visual_odom.velocity_covariance[0] = NAN;

	int instance_id = 0;
	orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_visual_odometry_pub, &visual_odom, &instance_id, ORB_PRIO_HIGH);
}

void
MavlinkReceiver::handle_message_odometry(mavlink_message_t *msg)
{
	mavlink_odometry_t odom;
	mavlink_msg_odometry_decode(msg, &odom);

	struct vehicle_odometry_s odometry = {};

	/* Dcm rotation matrix from body frame to local NED frame */
	matrix::Dcmf Rbl;

	odometry.timestamp = _mavlink_timesync.sync_stamp(odom.time_usec);
	/* The position is in the local NED frame */
	odometry.x = odom.x;
	odometry.y = odom.y;
	odometry.z = odom.z;
	/* The quaternion of the ODOMETRY msg represents a rotation from NED
	 * earth/local frame to XYZ body frame */
	matrix::Quatf q(odom.q);
	q.copyTo(odometry.q);

	// TODO:
	// - add a MAV_FRAME_*_OTHER to the Mavlink MAV_FRAME enum IOT define
	// a frame of reference which is not aligned with NED or ENU
	// - add usage on the estimator side
	odometry.local_frame = odometry.LOCAL_FRAME_NED;

	// pose_covariance
	static constexpr size_t POS_URT_SIZE = sizeof(odometry.pose_covariance) / sizeof(odometry.pose_covariance[0]);
	static_assert(POS_URT_SIZE == (sizeof(odom.pose_covariance) / sizeof(odom.pose_covariance[0])),
		      "Odometry Pose Covariance matrix URT array size mismatch");

	// velocity_covariance
	static constexpr size_t VEL_URT_SIZE = sizeof(odometry.velocity_covariance) / sizeof(odometry.velocity_covariance[0]);
	static_assert(VEL_URT_SIZE == (sizeof(odom.velocity_covariance) / sizeof(odom.velocity_covariance[0])),
		      "Odometry Velocity Covariance matrix URT array size mismatch");

	// create a method to simplify covariance copy
	for (size_t i = 0; i < POS_URT_SIZE; i++) {
		odometry.pose_covariance[i] = odom.pose_covariance[i];
	}

	bool updated;
	orb_check(_vehicle_attitude_sub, &updated);

	if (odom.child_frame_id == MAV_FRAME_BODY_FRD) { /* WRT to estimated vehicle body-fixed frame */
		/* get quaternion from the msg quaternion itself and build DCM matrix from it */
		Rbl = matrix::Dcmf(matrix::Quatf(odometry.q)).I();

		/* the linear velocities needs to be transformed to the local NED frame */\
		matrix::Vector3<float> linvel_local(Rbl * matrix::Vector3<float>(odom.vx, odom.vy, odom.vz));
		odometry.vx = linvel_local(0);
		odometry.vy = linvel_local(1);
		odometry.vz = linvel_local(2);

		odometry.rollspeed = odom.rollspeed;
		odometry.pitchspeed = odom.pitchspeed;
		odometry.yawspeed = odom.yawspeed;

		//TODO: Apply rotation matrix to transform from body-fixed NED to earth-fixed NED frame
		for (size_t i = 0; i < VEL_URT_SIZE; i++) {
			odometry.velocity_covariance[i] = odom.velocity_covariance[i];
		}

	} else if (odom.child_frame_id == MAV_FRAME_BODY_NED) { /* WRT to vehicle body-NED frame */
		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);

			/* get quaternion from vehicle_attitude quaternion and build DCM matrix from it */
			Rbl = matrix::Dcmf(matrix::Quatf(_att.q)).I();

			/* the linear velocities needs to be transformed to the local NED frame */
			matrix::Vector3<float> linvel_local(Rbl * matrix::Vector3<float>(odom.vx, odom.vy, odom.vz));
			odometry.vx = linvel_local(0);
			odometry.vy = linvel_local(1);
			odometry.vz = linvel_local(2);

			odometry.rollspeed = odom.rollspeed;
			odometry.pitchspeed = odom.pitchspeed;
			odometry.yawspeed = odom.yawspeed;

			//TODO: Apply rotation matrix to transform from body-fixed to earth-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				odometry.velocity_covariance[i] = odom.velocity_covariance[i];
			}

		}

	} else if (odom.child_frame_id == MAV_FRAME_VISION_NED || /* WRT to vehicle local NED frame */
		   odom.child_frame_id == MAV_FRAME_MOCAP_NED) {

		if (updated) {
			orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_att);

			/* get quaternion from vehicle_attitude quaternion and build DCM matrix from it */
			matrix::Dcmf Rlb = matrix::Quatf(_att.q);

			odometry.vx = odom.vx;
			odometry.vy = odom.vy;
			odometry.vz = odom.vz;

			/* the angular rates need to be transformed to the body frame */
			matrix::Vector3<float> angvel_local(Rlb * matrix::Vector3<float>(odom.rollspeed, odom.pitchspeed, odom.yawspeed));
			odometry.rollspeed = angvel_local(0);
			odometry.pitchspeed = angvel_local(1);
			odometry.yawspeed = angvel_local(2);

			//TODO: Apply rotation matrix to transform from earth-fixed to body-fixed NED frame
			for (size_t i = 0; i < VEL_URT_SIZE; i++) {
				odometry.velocity_covariance[i] = odom.velocity_covariance[i];
			}

		}

	} else {
		PX4_ERR("Body frame %u not supported. Unable to publish velocity", odom.child_frame_id);
	}

	int instance_id = 0;

	if (odom.frame_id == MAV_FRAME_VISION_NED) {
		orb_publish_auto(ORB_ID(vehicle_visual_odometry), &_visual_odometry_pub, &odometry, &instance_id, ORB_PRIO_HIGH);

	} else if (odom.frame_id == MAV_FRAME_MOCAP_NED) {
		orb_publish_auto(ORB_ID(vehicle_mocap_odometry), &_mocap_odometry_pub, &odometry, &instance_id, ORB_PRIO_HIGH);

	} else {
		PX4_ERR("Local frame %u not supported. Unable to publish pose and velocity", odom.frame_id);
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
					vehicle_attitude_setpoint_s att_sp = {};
					att_sp.timestamp = hrt_absolute_time();

					if (!ignore_attitude_msg) { // only copy att sp if message contained new data
						matrix::Quatf q(set_attitude_target.q);
						q.copyTo(att_sp.q_d);
						att_sp.q_d_valid = true;

						matrix::Eulerf euler{q};
						att_sp.roll_body = euler.phi();
						att_sp.pitch_body = euler.theta();
						att_sp.yaw_body = euler.psi();
						att_sp.yaw_sp_move_rate = 0.0f;
					}

					// TODO: We assume offboard is only used for multicopters which produce thrust along the
					// body z axis. If we want to support fixed wing as well we need to handle it differently here, e.g.
					// in that case we should assign att_sp.thrust_body[0]
					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						att_sp.thrust_body[2] = -set_attitude_target.thrust;
					}

					if (_att_sp_pub == nullptr) {
						_att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);

					} else {
						orb_publish(ORB_ID(vehicle_attitude_setpoint), _att_sp_pub, &att_sp);
					}
				}

				/* Publish attitude rate setpoint if bodyrate and thrust ignore bits are not set */
				///XXX add support for ignoring individual axes
				if (!(_offboard_control_mode.ignore_bodyrate)) {
					vehicle_rates_setpoint_s rates_sp = {};
					rates_sp.timestamp = hrt_absolute_time();

					if (!ignore_bodyrate_msg) { // only copy att rates sp if message contained new data
						rates_sp.roll = set_attitude_target.body_roll_rate;
						rates_sp.pitch = set_attitude_target.body_pitch_rate;
						rates_sp.yaw = set_attitude_target.body_yaw_rate;
					}

					if (!_offboard_control_mode.ignore_thrust) { // dont't overwrite thrust if it's invalid
						rates_sp.thrust_body[2] = -set_attitude_target.thrust;
					}

					if (_rates_sp_pub == nullptr) {
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
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		radio_status_s status = {};
		status.timestamp = hrt_absolute_time();
		status.rssi = rstatus.rssi;
		status.remote_rssi = rstatus.remrssi;
		status.txbuf = rstatus.txbuf;
		status.noise = rstatus.noise;
		status.remote_noise = rstatus.remnoise;
		status.rxerrors = rstatus.rxerrors;
		status.fix = rstatus.fixed;

		_mavlink->update_radio_status(status);

		int multi_instance;
		orb_publish_auto(ORB_ID(radio_status), &_radio_status_pub, &status, &multi_instance, ORB_PRIO_HIGH);
	}
}

void
MavlinkReceiver::handle_message_ping(mavlink_message_t *msg)
{
	mavlink_ping_t ping;
	mavlink_msg_ping_decode(msg, &ping);

	if ((ping.target_system == 0) &&
	    (ping.target_component == 0)) {	   // This is a ping request. Return it to the system which requested the ping.

		ping.target_system = msg->sysid;
		ping.target_component = msg->compid;
		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &ping);

	} else if ((ping.target_system == mavlink_system.sysid) &&
		   (ping.target_component ==
		    mavlink_system.compid)) {	// This is a returned ping message from this system. Calculate latency from it.

		const hrt_abstime now = hrt_absolute_time();

		// Calculate round trip time
		float rtt_ms = (now - ping.time_usec) / 1000.0f;

		// Update ping statistics
		struct Mavlink::ping_statistics_s &pstats = _mavlink->get_ping_statistics();

		pstats.last_ping_time = now;

		if (pstats.last_ping_seq == 0 && ping.seq > 0) {
			// This is the first reply we are receiving from an offboard system.
			// We may have been broadcasting pings for some time before it came online,
			// and these do not count as dropped packets.

			// Reset last_ping_seq counter for correct packet drop detection
			pstats.last_ping_seq = ping.seq - 1;
		}

		// We can only count dropped packets after the first message
		if (ping.seq > pstats.last_ping_seq) {
			pstats.dropped_packets += ping.seq - pstats.last_ping_seq - 1;
		}

		pstats.last_ping_seq = ping.seq;
		pstats.last_rtt = rtt_ms;
		pstats.mean_rtt = (rtt_ms + pstats.mean_rtt) / 2.0f;
		pstats.max_rtt = fmaxf(rtt_ms, pstats.max_rtt);
		pstats.min_rtt = pstats.min_rtt > 0.0f ? fminf(rtt_ms, pstats.min_rtt) : rtt_ms;

		/* Ping status is supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
		if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

			struct ping_s uorb_ping_msg = {};

			uorb_ping_msg.timestamp = now;
			uorb_ping_msg.ping_time = ping.time_usec;
			uorb_ping_msg.ping_sequence = ping.seq;
			uorb_ping_msg.dropped_packets = pstats.dropped_packets;
			uorb_ping_msg.rtt_ms = rtt_ms;
			uorb_ping_msg.system_id = msg->sysid;
			uorb_ping_msg.component_id = msg->compid;

			if (_ping_pub == nullptr) {
				int multi_instance;
				_ping_pub = orb_advertise_multi(ORB_ID(ping), &uorb_ping_msg, &multi_instance, ORB_PRIO_DEFAULT);

			} else {
				orb_publish(ORB_ID(ping), _ping_pub, &uorb_ping_msg);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_battery_status(mavlink_message_t *msg)
{
	if (msg->sysid != mavlink_system.sysid) {
		// ignore battery status of other system
		return;
	}

	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status = {};
	battery_status.timestamp = hrt_absolute_time();

	float voltage_sum = 0.0f;
	uint8_t cell_count = 0;

	while (battery_mavlink.voltages[cell_count] < UINT16_MAX && cell_count < 10) {
		voltage_sum += (float)(battery_mavlink.voltages[cell_count]) / 1000.0f;
		cell_count++;
	}

	battery_status.voltage_v = voltage_sum;
	battery_status.voltage_filtered_v  = voltage_sum;
	battery_status.current_a = battery_status.current_filtered_a = (float)(battery_mavlink.current_battery) / 100.0f;
	battery_status.current_filtered_a = battery_status.current_a;
	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.discharged_mah = (float)battery_mavlink.current_consumed;
	battery_status.cell_count = cell_count;
	battery_status.connected = true;

	// Get the battery level thresholds.
	float bat_emergen_thr;
	float bat_crit_thr;
	float bat_low_thr;
	param_get(_p_bat_emergen_thr, &bat_emergen_thr);
	param_get(_p_bat_crit_thr, &bat_crit_thr);
	param_get(_p_bat_low_thr, &bat_low_thr);

	// Set the battery warning based on remaining charge.
	//  Note: Smallest values must come first in evaluation.
	if (battery_status.remaining < bat_emergen_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (battery_status.remaining < bat_crit_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (battery_status.remaining < bat_low_thr) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	if (_battery_pub == nullptr) {
		_battery_pub = orb_advertise(ORB_ID(battery_status), &battery_status);

	} else {
		orb_publish(ORB_ID(battery_status), _battery_pub, &battery_status);
	}
}

void
MavlinkReceiver::handle_message_serial_control(mavlink_message_t *msg)
{
	mavlink_serial_control_t serial_control_mavlink;
	mavlink_msg_serial_control_decode(msg, &serial_control_mavlink);

	// we only support shell commands
	if (serial_control_mavlink.device != SERIAL_CONTROL_DEV_SHELL
	    || (serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_REPLY)) {
		return;
	}

	MavlinkShell *shell = _mavlink->get_shell();

	if (shell) {
		// we ignore the timeout, EXCLUSIVE & BLOCKING flags of the SERIAL_CONTROL message
		if (serial_control_mavlink.count > 0) {
			shell->write(serial_control_mavlink.data, serial_control_mavlink.count);
		}

		// if no response requested, assume the shell is no longer used
		if ((serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
			_mavlink->close_shell();
		}
	}
}

void
MavlinkReceiver::handle_message_logging_ack(mavlink_message_t *msg)
{
	mavlink_logging_ack_t logging_ack;
	mavlink_msg_logging_ack_decode(msg, &logging_ack);

	MavlinkULog *ulog_streaming = _mavlink->get_ulog_streaming();

	if (ulog_streaming) {
		ulog_streaming->handle_ack(logging_ack);
	}
}

void
MavlinkReceiver::handle_message_play_tune(mavlink_message_t *msg)
{
	mavlink_play_tune_t play_tune;
	mavlink_msg_play_tune_decode(msg, &play_tune);

	char *tune = play_tune.tune;

	if ((mavlink_system.sysid == play_tune.target_system ||
	     play_tune.target_system == 0) &&
	    (mavlink_system.compid == play_tune.target_component ||
	     play_tune.target_component == 0)) {

		if (*tune == 'M') {
			int fd = px4_open(TONE_ALARM0_DEVICE_PATH, PX4_F_WRONLY);

			if (fd >= 0) {
				px4_write(fd, tune, strlen(tune) + 1);
				px4_close(fd);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_obstacle_distance(mavlink_message_t *msg)
{
	mavlink_obstacle_distance_t mavlink_obstacle_distance;
	mavlink_msg_obstacle_distance_decode(msg, &mavlink_obstacle_distance);

	obstacle_distance_s obstacle_distance = {};
	obstacle_distance.timestamp = hrt_absolute_time();
	obstacle_distance.sensor_type = mavlink_obstacle_distance.sensor_type;
	memcpy(obstacle_distance.distances, mavlink_obstacle_distance.distances, sizeof(obstacle_distance.distances));
	obstacle_distance.increment = mavlink_obstacle_distance.increment;
	obstacle_distance.min_distance = mavlink_obstacle_distance.min_distance;
	obstacle_distance.max_distance = mavlink_obstacle_distance.max_distance;

	if (_obstacle_distance_pub == nullptr) {
		_obstacle_distance_pub = orb_advertise(ORB_ID(obstacle_distance), &obstacle_distance);

	} else {
		orb_publish(ORB_ID(obstacle_distance), _obstacle_distance_pub, &obstacle_distance);
	}
}

void
MavlinkReceiver::handle_message_trajectory_representation_waypoints(mavlink_message_t *msg)
{
	mavlink_trajectory_representation_waypoints_t trajectory;
	mavlink_msg_trajectory_representation_waypoints_decode(msg, &trajectory);


	struct vehicle_trajectory_waypoint_s trajectory_waypoint = {};

	trajectory_waypoint.timestamp = hrt_absolute_time();
	const int number_valid_points = trajectory.valid_points;

	for (int i = 0; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
		trajectory_waypoint.waypoints[i].position[0] = trajectory.pos_x[i];
		trajectory_waypoint.waypoints[i].position[1] = trajectory.pos_y[i];
		trajectory_waypoint.waypoints[i].position[2] = trajectory.pos_z[i];

		trajectory_waypoint.waypoints[i].velocity[0] = trajectory.vel_x[i];
		trajectory_waypoint.waypoints[i].velocity[1] = trajectory.vel_y[i];
		trajectory_waypoint.waypoints[i].velocity[2] = trajectory.vel_z[i];

		trajectory_waypoint.waypoints[i].acceleration[0] = trajectory.acc_x[i];
		trajectory_waypoint.waypoints[i].acceleration[1] = trajectory.acc_y[i];
		trajectory_waypoint.waypoints[i].acceleration[2] = trajectory.acc_z[i];

		trajectory_waypoint.waypoints[i].yaw = trajectory.pos_yaw[i];
		trajectory_waypoint.waypoints[i].yaw_speed = trajectory.vel_yaw[i];

	}

	for (int i = 0; i < number_valid_points; ++i) {
		trajectory_waypoint.waypoints[i].point_valid = true;
	}

	for (int i = number_valid_points; i < vehicle_trajectory_waypoint_s::NUMBER_POINTS; ++i) {
		trajectory_waypoint.waypoints[i].point_valid = false;
	}

	if (_trajectory_waypoint_pub == nullptr) {
		_trajectory_waypoint_pub = orb_advertise(ORB_ID(vehicle_trajectory_waypoint), &trajectory_waypoint);

	} else {
		orb_publish(ORB_ID(vehicle_trajectory_waypoint), _trajectory_waypoint_pub, &trajectory_waypoint);
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

	// fill uORB message
	struct input_rc_s rc = {};
	// metadata
	rc.timestamp = hrt_absolute_time();
	rc.timestamp_last_signal = rc.timestamp;
	rc.rssi = RC_INPUT_RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.rc_ppm_frame_length = 0;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;
	// channels
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;
	rc.values[8] = man.chan9_raw;
	rc.values[9] = man.chan10_raw;
	rc.values[10] = man.chan11_raw;
	rc.values[11] = man.chan12_raw;
	rc.values[12] = man.chan13_raw;
	rc.values[13] = man.chan14_raw;
	rc.values[14] = man.chan15_raw;
	rc.values[15] = man.chan16_raw;
	rc.values[16] = man.chan17_raw;
	rc.values[17] = man.chan18_raw;

	// check how many channels are valid
	for (int i = 17; i >= 0; i--) {
		const bool ignore_max = rc.values[i] == UINT16_MAX; // ignore any channel with value UINT16_MAX
		const bool ignore_zero = (i > 7) && (rc.values[i] == 0); // ignore channel 8-18 if value is 0

		if (ignore_max || ignore_zero) {
			// set all ignored values to zero
			rc.values[i] = 0;

		} else {
			// first channel to not ignore -> set count considering zero-based index
			rc.channel_count = i + 1;
			break;
		}
	}

	// publish uORB message
	int instance; // provides the instance ID or the publication
	ORB_PRIO priority = ORB_PRIO_HIGH; // since it is an override, set priority high
	orb_publish_auto(ORB_ID(input_rc), &_rc_pub, &rc, &instance, priority);
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

		struct input_rc_s rc = {};
		rc.timestamp = hrt_absolute_time();
		rc.timestamp_last_signal = rc.timestamp;

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
		manual.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0 + _mavlink->get_instance_id();

		int m_inst;
		orb_publish_auto(ORB_ID(manual_control_setpoint), &_manual_pub, &manual, &m_inst, ORB_PRIO_LOW);
	}
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		/* Accept only heartbeats from GCS or ONBOARD Controller, skip heartbeats from other vehicles */
		if ((msg->sysid != mavlink_system.sysid && hb.type == MAV_TYPE_GCS) || (msg->sysid == mavlink_system.sysid
				&& hb.type == MAV_TYPE_ONBOARD_CONTROLLER)) {

			telemetry_status_s &tstatus = _mavlink->get_telemetry_status();

			/* set heartbeat time and topic time and publish -
			 * the telem status also gets updated on telemetry events
			 */
			tstatus.heartbeat_time = hrt_absolute_time();
			tstatus.remote_system_id = msg->sysid;
			tstatus.remote_component_id = msg->compid;
			tstatus.remote_type = hb.type;
			tstatus.remote_system_status = hb.system_status;
		}

	}
}

int
MavlinkReceiver::set_message_interval(int msgId, float interval, int data_rate)
{
	if (msgId == MAVLINK_MSG_ID_HEARTBEAT) {
		return PX4_ERROR;
	}

	if (data_rate > 0) {
		_mavlink->set_data_rate(data_rate);
	}

	// configure_stream wants a rate (msgs/second), so convert here.
	float rate = 0.f;

	if (interval < -0.00001f) {
		rate = 0.f; // stop the stream

	} else if (interval > 0.00001f) {
		rate = 1000000.0f / interval;

	} else {
		rate = -2.f; // set default rate
	}

	bool found_id = false;


	// The interval between two messages is in microseconds.
	// Set to -1 to disable and 0 to request default rate
	if (msgId != 0) {
		const char *stream_name = get_stream_name(msgId);

		if (stream_name != nullptr) {
			_mavlink->configure_stream_threadsafe(stream_name, rate);
			found_id = true;
		}
	}

	return (found_id ? PX4_OK : PX4_ERROR);
}

void
MavlinkReceiver::get_message_interval(int msgId)
{
	unsigned interval = 0;

	for (const auto &stream : _mavlink->get_streams()) {
		if (stream->get_id() == msgId) {
			interval = stream->get_interval();
			break;
		}
	}

	// send back this value...
	mavlink_msg_message_interval_send(_mavlink->get_channel(), msgId, interval);
}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t imu;
	mavlink_msg_hil_sensor_decode(msg, &imu);

	uint64_t timestamp = hrt_absolute_time();

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
		sensor_gyro_s gyro = {};

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
		sensor_accel_s accel = {};

		accel.timestamp = timestamp;
		accel.x_raw = imu.xacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.y_raw = imu.yacc / (CONSTANTS_ONE_G / 1000.0f);
		accel.z_raw = imu.zacc / (CONSTANTS_ONE_G / 1000.0f);
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
		sensor_baro_s baro = {};

		baro.timestamp = timestamp;
		baro.pressure = imu.abs_pressure;
		baro.temperature = imu.temperature;

		/* fake device ID */
		baro.device_id = 1234567;

		if (_baro_pub == nullptr) {
			_baro_pub = orb_advertise(ORB_ID(sensor_baro), &baro);

		} else {
			orb_publish(ORB_ID(sensor_baro), _baro_pub, &baro);
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
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t gps;
	mavlink_msg_hil_gps_decode(msg, &gps);

	uint64_t timestamp = hrt_absolute_time();

	struct vehicle_gps_position_s hil_gps = {};

	hil_gps.timestamp_time_relative = 0;
	hil_gps.time_utc_usec = gps.time_usec;

	hil_gps.timestamp = timestamp;
	hil_gps.lat = gps.lat;
	hil_gps.lon = gps.lon;
	hil_gps.alt = gps.alt;
	hil_gps.eph = (float)gps.eph * 1e-2f; // from cm to m
	hil_gps.epv = (float)gps.epv * 1e-2f; // from cm to m

	hil_gps.s_variance_m_s = 0.1f;

	hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
	hil_gps.vel_n_m_s = gps.vn * 1e-2f; // from cm to m
	hil_gps.vel_e_m_s = gps.ve * 1e-2f; // from cm to m
	hil_gps.vel_d_m_s = gps.vd * 1e-2f; // from cm to m
	hil_gps.vel_ned_valid = true;
	hil_gps.cog_rad = ((gps.cog == 65535) ? NAN : wrap_2pi(math::radians(gps.cog * 1e-2f)));

	hil_gps.fix_type = gps.fix_type;
	hil_gps.satellites_used = gps.satellites_visible;  //TODO: rename mavlink_hil_gps_t sats visible to used?

	hil_gps.heading = NAN;
	hil_gps.heading_offset = NAN;

	if (_gps_pub == nullptr) {
		_gps_pub = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

	} else {
		orb_publish(ORB_ID(vehicle_gps_position), _gps_pub, &hil_gps);
	}
}

void
MavlinkReceiver::handle_message_follow_target(mavlink_message_t *msg)
{
	mavlink_follow_target_t follow_target_msg;
	follow_target_s follow_target_topic = {};

	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_topic.timestamp = hrt_absolute_time();

	follow_target_topic.lat = follow_target_msg.lat * 1e-7;
	follow_target_topic.lon = follow_target_msg.lon * 1e-7;
	follow_target_topic.alt = follow_target_msg.alt;

	if (_follow_target_pub == nullptr) {
		_follow_target_pub = orb_advertise(ORB_ID(follow_target), &follow_target_topic);

	} else {
		orb_publish(ORB_ID(follow_target), _follow_target_pub, &follow_target_topic);
	}
}

void
MavlinkReceiver::handle_message_landing_target(mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target;

	mavlink_msg_landing_target_decode(msg, &landing_target);

	if (landing_target.position_valid && landing_target.frame == MAV_FRAME_LOCAL_NED) {
		landing_target_pose_s landing_target_pose = {};
		landing_target_pose.timestamp = _mavlink_timesync.sync_stamp(landing_target.time_usec);
		landing_target_pose.abs_pos_valid = true;
		landing_target_pose.x_abs = landing_target.x;
		landing_target_pose.y_abs = landing_target.y;
		landing_target_pose.z_abs = landing_target.z;

		if (_landing_target_pose_pub == nullptr) {
			_landing_target_pose_pub = orb_advertise(ORB_ID(landing_target_pose), &landing_target_pose);

		} else {
			orb_publish(ORB_ID(landing_target_pose), _landing_target_pose_pub, &landing_target_pose);
		}
	}
}

void
MavlinkReceiver::handle_message_adsb_vehicle(mavlink_message_t *msg)
{
	mavlink_adsb_vehicle_t adsb;
	transponder_report_s t = {};

	mavlink_msg_adsb_vehicle_decode(msg, &adsb);

	t.timestamp = hrt_absolute_time();

	t.icao_address = adsb.ICAO_address;
	t.lat = adsb.lat * 1e-7;
	t.lon = adsb.lon * 1e-7;
	t.altitude_type = adsb.altitude_type;
	t.altitude = adsb.altitude / 1000.0f;
	t.heading = adsb.heading / 100.0f / 180.0f * M_PI_F - M_PI_F;
	t.hor_velocity = adsb.hor_velocity / 100.0f;
	t.ver_velocity = adsb.ver_velocity / 100.0f;
	memcpy(&t.callsign[0], &adsb.callsign[0], sizeof(t.callsign));
	t.emitter_type = adsb.emitter_type;
	t.tslc = adsb.tslc;
	t.squawk = adsb.squawk;

	t.flags = transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;  //Unset in receiver already broadcast its messages

	if (adsb.flags & ADSB_FLAGS_VALID_COORDS) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS; }

	if (adsb.flags & ADSB_FLAGS_VALID_ALTITUDE) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE; }

	if (adsb.flags & ADSB_FLAGS_VALID_HEADING) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING; }

	if (adsb.flags & ADSB_FLAGS_VALID_VELOCITY) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY; }

	if (adsb.flags & ADSB_FLAGS_VALID_CALLSIGN) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN; }

	if (adsb.flags & ADSB_FLAGS_VALID_SQUAWK) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK; }

	//PX4_INFO("code: %d callsign: %s, vel: %8.4f, tslc: %d", (int)t.ICAO_address, t.callsign, (double)t.hor_velocity, (int)t.tslc);

	if (_transponder_report_pub == nullptr) {
		_transponder_report_pub = orb_advertise_queue(ORB_ID(transponder_report), &t, transponder_report_s::ORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(transponder_report), _transponder_report_pub, &t);
	}
}

void
MavlinkReceiver::handle_message_collision(mavlink_message_t *msg)
{
	mavlink_collision_t collision;
	collision_report_s t = {};

	mavlink_msg_collision_decode(msg, &collision);

	t.timestamp = hrt_absolute_time();
	t.src = collision.src;
	t.id = collision.id;
	t.action = collision.action;
	t.threat_level = collision.threat_level;
	t.time_to_minimum_delta = collision.time_to_minimum_delta;
	t.altitude_minimum_delta = collision.altitude_minimum_delta;
	t.horizontal_minimum_delta = collision.horizontal_minimum_delta;

	if (_collision_report_pub == nullptr) {
		_collision_report_pub = orb_advertise(ORB_ID(collision_report), &t);

	} else {
		orb_publish(ORB_ID(collision_report), _collision_report_pub, &t);
	}
}

void
MavlinkReceiver::handle_message_gps_rtcm_data(mavlink_message_t *msg)
{
	mavlink_gps_rtcm_data_t gps_rtcm_data_msg = {};
	mavlink_msg_gps_rtcm_data_decode(msg, &gps_rtcm_data_msg);

	gps_inject_data_s gps_inject_data_topic = {};
	gps_inject_data_topic.len = math::min((int)sizeof(gps_rtcm_data_msg.data),
					      (int)sizeof(uint8_t) * gps_rtcm_data_msg.len);
	gps_inject_data_topic.flags = gps_rtcm_data_msg.flags;
	memcpy(gps_inject_data_topic.data, gps_rtcm_data_msg.data,
	       math::min((int)sizeof(gps_inject_data_topic.data), (int)sizeof(uint8_t) * gps_inject_data_topic.len));

	orb_advert_t &pub = _gps_inject_data_pub;

	if (pub == nullptr) {
		pub = orb_advertise_queue(ORB_ID(gps_inject_data), &gps_inject_data_topic,
					  _gps_inject_data_queue_size);

	} else {
		orb_publish(ORB_ID(gps_inject_data), pub, &gps_inject_data_topic);
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
		struct airspeed_s airspeed = {};

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
		hil_attitude = {};
		hil_attitude.timestamp = timestamp;

		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);

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
		struct vehicle_global_position_s hil_global_pos = {};

		hil_global_pos.timestamp = timestamp;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.vel_n = hil_state.vx / 100.0f;
		hil_global_pos.vel_e = hil_state.vy / 100.0f;
		hil_global_pos.vel_d = hil_state.vz / 100.0f;
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
			_hil_local_pos.ref_timestamp = timestamp;
			_hil_local_pos.ref_lat = lat;
			_hil_local_pos.ref_lon = lon;
			_hil_local_pos.ref_alt = _hil_local_alt0;
		}

		float x = 0.0f;
		float y = 0.0f;
		map_projection_project(&_hil_local_proj_ref, lat, lon, &x, &y);
		_hil_local_pos.timestamp = timestamp;
		_hil_local_pos.xy_valid = true;
		_hil_local_pos.z_valid = true;
		_hil_local_pos.v_xy_valid = true;
		_hil_local_pos.v_z_valid = true;
		_hil_local_pos.x = x;
		_hil_local_pos.y = y;
		_hil_local_pos.z = _hil_local_alt0 - hil_state.alt / 1000.0f;
		_hil_local_pos.vx = hil_state.vx / 100.0f;
		_hil_local_pos.vy = hil_state.vy / 100.0f;
		_hil_local_pos.vz = hil_state.vz / 100.0f;
		matrix::Eulerf euler = matrix::Quatf(hil_attitude.q);
		_hil_local_pos.yaw = euler.psi();
		_hil_local_pos.xy_global = true;
		_hil_local_pos.z_global = true;
		_hil_local_pos.vxy_max = INFINITY;
		_hil_local_pos.vz_max = INFINITY;
		_hil_local_pos.hagl_min = INFINITY;
		_hil_local_pos.hagl_max = INFINITY;

		if (_local_pos_pub == nullptr) {
			_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &_hil_local_pos);

		} else {
			orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &_hil_local_pos);
		}
	}

	/* land detector */
	{
		bool landed = (float)(hil_state.alt) / 1000.0f < (_hil_local_alt0 + 0.1f); // XXX improve?

		if (_hil_land_detector.landed != landed) {
			_hil_land_detector.landed = landed;
			_hil_land_detector.timestamp = hrt_absolute_time();

			if (_land_detector_pub == nullptr) {
				_land_detector_pub = orb_advertise(ORB_ID(vehicle_land_detected), &_hil_land_detector);

			} else {
				orb_publish(ORB_ID(vehicle_land_detected), _land_detector_pub, &_hil_land_detector);
			}
		}
	}

	/* accelerometer */
	{
		sensor_accel_s accel = {};

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
		struct battery_status_s	hil_battery_status = {};

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

void
MavlinkReceiver::handle_message_named_value_float(mavlink_message_t *msg)
{
	mavlink_named_value_float_t debug_msg;
	debug_key_value_s debug_topic;

	mavlink_msg_named_value_float_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.key, debug_msg.name, sizeof(debug_topic.key));
	debug_topic.key[sizeof(debug_topic.key) - 1] = '\0'; // enforce null termination
	debug_topic.value = debug_msg.value;

	if (_debug_key_value_pub == nullptr) {
		_debug_key_value_pub = orb_advertise(ORB_ID(debug_key_value), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_key_value), _debug_key_value_pub, &debug_topic);
	}
}

void
MavlinkReceiver::handle_message_debug(mavlink_message_t *msg)
{
	mavlink_debug_t debug_msg;
	debug_value_s debug_topic;

	mavlink_msg_debug_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.ind = debug_msg.ind;
	debug_topic.value = debug_msg.value;

	if (_debug_value_pub == nullptr) {
		_debug_value_pub = orb_advertise(ORB_ID(debug_value), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_value), _debug_value_pub, &debug_topic);
	}
}

void
MavlinkReceiver::handle_message_debug_vect(mavlink_message_t *msg)
{
	mavlink_debug_vect_t debug_msg;
	debug_vect_s debug_topic;

	mavlink_msg_debug_vect_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination
	debug_topic.x = debug_msg.x;
	debug_topic.y = debug_msg.y;
	debug_topic.z = debug_msg.z;

	if (_debug_vect_pub == nullptr) {
		_debug_vect_pub = orb_advertise(ORB_ID(debug_vect), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_vect), _debug_vect_pub, &debug_topic);
	}
}

void
MavlinkReceiver::handle_message_debug_float_array(mavlink_message_t *msg)
{
	mavlink_debug_float_array_t debug_msg;
	debug_array_s debug_topic = {};

	mavlink_msg_debug_float_array_decode(msg, &debug_msg);

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.id = debug_msg.array_id;
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

	for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		debug_topic.data[i] = debug_msg.data[i];
	}

	if (_debug_array_pub == nullptr) {
		_debug_array_pub = orb_advertise(ORB_ID(debug_array), &debug_topic);

	} else {
		orb_publish(ORB_ID(debug_array), _debug_array_pub, &debug_topic);
	}
}

/**
 * Receive data from UART/UDP
 */
void *
MavlinkReceiver::receive_thread(void *arg)
{

	/* set thread name */
	{
		char thread_name[24];
		sprintf(thread_name, "mavlink_rcv_if%d", _mavlink->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	// poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
	const int timeout = 10;

#if defined(__PX4_POSIX)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1600 * 5];
#elif defined(CONFIG_NET)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1000];
#else
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[64];
#endif
	mavlink_message_t msg;

	struct pollfd fds[1] = {};

	if (_mavlink->get_protocol() == SERIAL) {
		fds[0].fd = _mavlink->get_uart_fd();
		fds[0].events = POLLIN;
	}

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink->get_protocol() == UDP || _mavlink->get_protocol() == TCP) {
		// make sure mavlink app has booted before we start using the socket
		while (!_mavlink->boot_complete()) {
			px4_usleep(100000);
		}

		fds[0].fd = _mavlink->get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif
	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while (!_mavlink->_task_should_exit) {
		if (poll(&fds[0], 1, timeout) > 0) {
			if (_mavlink->get_protocol() == SERIAL) {

				/*
				 * to avoid reading very small chunks wait for data before reading
				 * this is designed to target one message, so >20 bytes at a time
				 */
				const unsigned character_count = 20;

				/* non-blocking read. read may return negative values */
				if ((nread = ::read(fds[0].fd, buf, sizeof(buf))) < (ssize_t)character_count) {
					const unsigned sleeptime = character_count * 1000000 / (_mavlink->get_baudrate() / 10);
					px4_usleep(sleeptime);
				}
			}

#if defined(CONFIG_NET) || defined(__PX4_POSIX)

			if (_mavlink->get_protocol() == UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink->get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}

			} else {
				// could be TCP or other protocol
			}

			struct sockaddr_in &srcaddr_last = _mavlink->get_client_source_address();

			int localhost = (127 << 24) + 1;

			if (!_mavlink->get_client_source_initialized()) {

				// set the address either if localhost or if 3 seconds have passed
				// this ensures that a GCS running on localhost can get a hold of
				// the system within the first N seconds
				hrt_abstime stime = _mavlink->get_start_time();

				if ((stime != 0 && (hrt_elapsed_time(&stime) > 3 * 1000 * 1000))
				    || (srcaddr_last.sin_addr.s_addr == htonl(localhost))) {
					srcaddr_last.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
					srcaddr_last.sin_port = srcaddr.sin_port;
					_mavlink->set_client_source_initialized();
					PX4_INFO("partner IP: %s", inet_ntoa(srcaddr.sin_addr));
				}
			}

#endif
			// only start accepting messages once we're sure who we talk to

			if (_mavlink->get_client_source_initialized()) {
				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &_status)) {

						/* check if we received version 2 and request a switch. */
						if (!(_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
							/* this will only switch to proto version 2 if allowed in settings */
							_mavlink->set_proto_version(2);
						}

						/* handle generic messages and commands */
						handle_message(&msg);

						/* handle packet with mission manager */
						_mission_manager.handle_message(&msg);


						/* handle packet with parameter component */
						_parameters_manager.handle_message(&msg);

						if (_mavlink->ftp_enabled()) {
							/* handle packet with ftp component */
							_mavlink_ftp.handle_message(&msg);
						}

						/* handle packet with log component */
						_mavlink_log_handler.handle_message(&msg);

						/* handle packet with timesync component */
						_mavlink_timesync.handle_message(&msg);

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

		hrt_abstime t = hrt_absolute_time();

		if (t - last_send_update > timeout * 1000) {
			_mission_manager.check_active_mission();
			_mission_manager.send(t);

			_parameters_manager.send(t);

			if (_mavlink->ftp_enabled()) {
				_mavlink_ftp.send(t);
			}

			_mavlink_log_handler.send(t);
			last_send_update = t;
		}

	}

	return nullptr;
}

void
MavlinkReceiver::print_status()
{

}

void *
MavlinkReceiver::start_helper(void *context)
{

	MavlinkReceiver *rcv = new MavlinkReceiver((Mavlink *)context);

	if (!rcv) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	void *ret = rcv->receive_thread(nullptr);

	delete rcv;

	return ret;
}

void
MavlinkReceiver::receive_start(pthread_t *thread, Mavlink *parent)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, PX4_STACK_ADJUSTED(2840 + MAVLINK_RECEIVER_NET_ADDED_STACK));
	pthread_create(thread, &receiveloop_attr, MavlinkReceiver::start_helper, (void *)parent);

	pthread_attr_destroy(&receiveloop_attr);
}
