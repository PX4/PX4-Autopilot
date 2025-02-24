/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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

#include <lib/airspeed/airspeed.h>
#include <lib/conversion/rotation.h>
#include <lib/systemlib/px4_macros.h>

#include <math.h>
#include <poll.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifndef __PX4_POSIX
#include <termios.h>
#endif

#include "mavlink_command_sender.h"
#include "mavlink_main.h"
#include "mavlink_receiver.h"

#include <lib/drivers/device/Device.hpp> // For DeviceId union

#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

MavlinkReceiver::~MavlinkReceiver()
{
	delete _tune_publisher;
	delete _px4_accel;
	delete _px4_gyro;
	delete _px4_mag;
#if !defined(CONSTRAINED_FLASH)
	delete[] _received_msg_stats;
#endif // !CONSTRAINED_FLASH

	_distance_sensor_pub.unadvertise();
	_gps_inject_data_pub.unadvertise();
	_rc_pub.unadvertise();
	_manual_control_input_pub.unadvertise();
	_ping_pub.unadvertise();
	_radio_status_pub.unadvertise();
	_sensor_baro_pub.unadvertise();
	_sensor_gps_pub.unadvertise();
	_sensor_optical_flow_pub.unadvertise();
}

static constexpr vehicle_odometry_s vehicle_odometry_empty {
	.timestamp = 0,
	.timestamp_sample = 0,
	.position = {NAN, NAN, NAN},
	.q = {NAN, NAN, NAN, NAN},
	.velocity = {NAN, NAN, NAN},
	.angular_velocity = {NAN, NAN, NAN},
	.position_variance = {NAN, NAN, NAN},
	.orientation_variance = {NAN, NAN, NAN},
	.velocity_variance = {NAN, NAN, NAN},
	.pose_frame = vehicle_odometry_s::POSE_FRAME_UNKNOWN,
	.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN,
	.reset_counter = 0,
	.quality = 0
};

MavlinkReceiver::MavlinkReceiver(Mavlink &parent) :
	ModuleParams(nullptr),
	_mavlink(parent),
	_mavlink_ftp(parent),
	_mavlink_log_handler(parent),
	_mission_manager(parent),
	_parameters_manager(parent),
	_mavlink_timesync(parent)
{
}

void
MavlinkReceiver::acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result, uint8_t progress)
{
	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = command;
	command_ack.result = result;
	command_ack.target_system = sysid;
	command_ack.target_component = compid;
	command_ack.result_param1 = progress;

	_cmd_ack_pub.publish(command_ack);
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

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
		handle_message_set_position_target_global_int(msg);
		break;

	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		handle_message_set_attitude_target(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
		handle_message_set_gps_global_origin(msg);
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

	case MAVLINK_MSG_ID_CELLULAR_STATUS:
		handle_message_cellular_status(msg);
		break;

	case MAVLINK_MSG_ID_ADSB_VEHICLE:
		handle_message_adsb_vehicle(msg);
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

	case MAVLINK_MSG_ID_PLAY_TUNE_V2:
		handle_message_play_tune_v2(msg);
		break;

	case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
		handle_message_obstacle_distance(msg);
		break;

	case MAVLINK_MSG_ID_TUNNEL:
		handle_message_tunnel(msg);
		break;

	case MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS:
		handle_message_onboard_computer_status(msg);
		break;

	case MAVLINK_MSG_ID_GENERATOR_STATUS:
		handle_message_generator_status(msg);
		break;

	case MAVLINK_MSG_ID_STATUSTEXT:
		handle_message_statustext(msg);
		break;

	case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID:
		handle_message_open_drone_id_operator_id(msg);
		break;

	case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID:
		handle_message_open_drone_id_self_id(msg);
		break;

	case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM:
		handle_message_open_drone_id_system(msg);
		break;

#if !defined(CONSTRAINED_FLASH)

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		handle_message_named_value_float(msg);
		break;

	case MAVLINK_MSG_ID_NAMED_VALUE_INT:
		handle_message_named_value_int(msg);
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
#endif // !CONSTRAINED_FLASH

	case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
		handle_message_gimbal_manager_set_attitude(msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL:
		handle_message_gimbal_manager_set_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
		handle_message_gimbal_device_information(msg);
		break;

	case MAVLINK_MSG_ID_REQUEST_EVENT:
		handle_message_request_event(msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
		handle_message_gimbal_device_attitude_status(msg);
		break;

#if defined(MAVLINK_MSG_ID_SET_VELOCITY_LIMITS) // For now only defined if development.xml is used

	case MAVLINK_MSG_ID_SET_VELOCITY_LIMITS:
		handle_message_set_velocity_limits(msg);
		break;
#endif

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
	if (_mavlink.get_hil_enabled()) {
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


	if (_mavlink.get_hil_enabled() || (_mavlink.get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

	/* handle packet with mission manager */
	_mission_manager.handle_message(msg);

	/* handle packet with parameter component */
	if (_mavlink.boot_complete()) {
		// make sure mavlink app has booted before we start processing parameter sync
		_parameters_manager.handle_message(msg);

	} else {
		if (hrt_elapsed_time(&_mavlink.get_first_start_time()) > 20_s) {
			PX4_ERR("system boot did not complete in 20 seconds");
			_mavlink.set_boot_complete();
		}
	}

	if (_mavlink.ftp_enabled()) {
		/* handle packet with ftp component */
		_mavlink_ftp.handle_message(msg);
	}

	/* handle packet with log component */
	_mavlink_log_handler.handle_message(msg);

	/* handle packet with timesync component */
	_mavlink_timesync.handle_message(msg);

	/* handle packet with parent object */
	_mavlink.handle_message(msg);
}

void MavlinkReceiver::handle_messages_in_gimbal_mode(mavlink_message_t &msg)
{
	switch (msg.msgid) {
	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(&msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
		handle_message_gimbal_manager_set_attitude(&msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL:
		handle_message_gimbal_manager_set_manual_control(&msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
		handle_message_gimbal_device_information(&msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
		handle_message_gimbal_device_attitude_status(&msg);
		break;
	}

	// Message forwarding
	_mavlink.handle_message(&msg);
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
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd{};

	vcmd.timestamp = hrt_absolute_time();

	const float before_int32_max = nextafterf((float)INT32_MAX, 0.0f);
	const float after_int32_max = nextafterf((float)INT32_MAX, (float)INFINITY);

	if (cmd_mavlink.param5 >= before_int32_max && cmd_mavlink.param5 <= after_int32_max &&
	    cmd_mavlink.param6 >= before_int32_max && cmd_mavlink.param6 <= after_int32_max) {
		// This looks suspicously like INT32_MAX was sent in a COMMAND_LONG instead of
		// a COMMAND_INT.
		PX4_ERR("param5/param6 invalid of command %" PRIu16, cmd_mavlink.command);
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		return;
	}

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

	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();

	if (cmd_mavlink.x == 0x7ff80000 && cmd_mavlink.y == 0x7ff80000) {
		// This looks like NAN was by accident sent as int.
		PX4_ERR("x/y invalid of command %" PRIu16, cmd_mavlink.command);
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		return;
	}

	/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;

	if (cmd_mavlink.x == INT32_MAX && cmd_mavlink.y == INT32_MAX) {
		// INT32_MAX for x and y means to ignore it.
		vcmd.param5 = (double)NAN;
		vcmd.param6 = (double)NAN;

	} else {
		vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
		vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
	}

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
	uint8_t result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	uint8_t progress = 0; // TODO: should be 255, 0 for backwards compatibility

	if (!target_ok) {
		if (!_mavlink.get_forwarding_on()
		    || !_mavlink.component_was_seen(cmd_mavlink.target_system, cmd_mavlink.target_component, _mavlink)) {
			PX4_INFO("Ignore command %d from %d/%d to %d/%d",
				 cmd_mavlink.command, msg->sysid, msg->compid, cmd_mavlink.target_system, cmd_mavlink.target_component);
		}

		return;
	}

	// First we handle legacy support requests which were used before we had
	// the generic MAV_CMD_REQUEST_MESSAGE.
	if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
		result = handle_request_message_command(MAVLINK_MSG_ID_AUTOPILOT_VERSION);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_PROTOCOL_VERSION);

	} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_HOME_POSITION);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_FLIGHT_INFORMATION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_FLIGHT_INFORMATION);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_STORAGE_INFORMATION);

	} else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
		if (set_message_interval(
			    (int)(cmd_mavlink.param1 + 0.5f), cmd_mavlink.param2, cmd_mavlink.param3, cmd_mavlink.param4, vehicle_command.param7)) {
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
		}

	} else if (cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
		get_message_interval((int)(cmd_mavlink.param1 + 0.5f));

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_MESSAGE) {

		uint16_t message_id = (uint16_t)roundf(vehicle_command.param1);
		result = handle_request_message_command(message_id,
							vehicle_command.param2, vehicle_command.param3, vehicle_command.param4,
							vehicle_command.param5, vehicle_command.param6, vehicle_command.param7);

	} else if (cmd_mavlink.command == MAV_CMD_INJECT_FAILURE) {
		if (_mavlink.failure_injection_enabled()) {
			_cmd_pub.publish(vehicle_command);
			send_ack = false;

		} else {
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			send_ack = true;
		}

	} else if (cmd_mavlink.command == MAV_CMD_DO_AUTOTUNE_ENABLE) {

		bool has_module = true;
		autotune_attitude_control_status_s status{};
		_autotune_attitude_control_status_sub.copy(&status);

		// if not busy enable via the parameter
		// do not check the return value of the uORB copy above because the module
		// starts publishing only when MC_AT_START is set
		if (status.state == autotune_attitude_control_status_s::STATE_IDLE) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (!vehicle_status.in_transition_mode) {
				param_t atune_start;

				switch (vehicle_status.vehicle_type) {
				case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
					atune_start = param_find("FW_AT_START");

					break;

				case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
					atune_start = param_find("MC_AT_START");

					break;

				default:
					atune_start = PARAM_INVALID;
					break;
				}

				if (atune_start == PARAM_INVALID) {
					has_module = false;

				} else {
					int32_t start = 1;
					param_set(atune_start, &start);
				}

			} else {
				has_module = false;
			}
		}

		if (has_module) {

			// most are in progress
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;

			switch (status.state) {
			case autotune_attitude_control_status_s::STATE_IDLE:
			case autotune_attitude_control_status_s::STATE_INIT:
				progress = 0;
				break;

			case autotune_attitude_control_status_s::STATE_ROLL:
			case autotune_attitude_control_status_s::STATE_ROLL_PAUSE:
				progress = 20;
				break;

			case autotune_attitude_control_status_s::STATE_PITCH:
			case autotune_attitude_control_status_s::STATE_PITCH_PAUSE:
				progress = 40;
				break;

			case autotune_attitude_control_status_s::STATE_YAW:
			case autotune_attitude_control_status_s::STATE_YAW_PAUSE:
				progress = 60;
				break;

			case autotune_attitude_control_status_s::STATE_VERIFICATION:
				progress = 80;
				break;

			case autotune_attitude_control_status_s::STATE_APPLY:
				progress = 85;
				break;

			case autotune_attitude_control_status_s::STATE_TEST:
				progress = 90;
				break;

			case autotune_attitude_control_status_s::STATE_WAIT_FOR_DISARM:
				progress = 95;
				break;

			case autotune_attitude_control_status_s::STATE_COMPLETE:
				progress = 100;
				// ack it properly with an ACCEPTED once we're done
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
				break;

			case autotune_attitude_control_status_s::STATE_FAIL:
				progress = 0;
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				break;
			}

		} else {
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		}

		send_ack = true;

	} else {
		send_ack = false;

		if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
			PX4_WARN("ignoring CMD with same SYS/COMP (%" PRIu8 "/%" PRIu8 ") ID", mavlink_system.sysid, mavlink_system.compid);
			return;
		}

		if (cmd_mavlink.command == MAV_CMD_LOGGING_START) {
			// check that we have enough bandwidth available: this is given by the configured logger topics
			// and rates. The 5000 is somewhat arbitrary, but makes sure that we cannot enable log streaming
			// on a radio link
			if (_mavlink.get_data_rate() < 5000) {
				send_ack = true;
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
				_mavlink.send_statustext_critical("Not enough bandwidth to enable log streaming\t");
				events::send<uint32_t>(events::ID("mavlink_log_not_enough_bw"), events::Log::Error,
						       "Not enough bandwidth to enable log streaming ({1} \\< 5000)", _mavlink.get_data_rate());

			} else {
				// we already instanciate the streaming object, because at this point we know on which
				// mavlink channel streaming was requested. But in fact it's possible that the logger is
				// not even running. The main mavlink thread takes care of this by waiting for an ack
				// from the logger.
				_mavlink.try_start_ulog_streaming(msg->sysid, msg->compid);
			}
		}

		if (!send_ack) {
			_cmd_pub.publish(vehicle_command);
		}
	}

	if (send_ack) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, result, progress);
	}
}

uint8_t MavlinkReceiver::handle_request_message_command(uint16_t message_id, float param2, float param3, float param4,
		float param5, float param6, float param7)
{
	bool stream_found = false;
	bool message_sent = false;

	for (const auto &stream : _mavlink.get_streams()) {
		if (stream->get_id() == message_id) {
			stream_found = true;
			message_sent = stream->request_message(param2, param3, param4, param5, param6, param7);
			break;
		}
	}

	if (!stream_found) {
		// If we don't find the stream, we can configure it with rate 0 and then trigger it once.
		const char *stream_name = get_stream_name(message_id);

		if (stream_name != nullptr) {
			_mavlink.configure_stream_threadsafe(stream_name, 0.0f);

			// Now we try again to send it.
			for (const auto &stream : _mavlink.get_streams()) {
				if (stream->get_id() == message_id) {
					message_sent = stream->request_message(param2, param3, param4, param5, param6, param7);
					break;
				}
			}
		}
	}

	return (message_sent ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
		vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
}


void
MavlinkReceiver::handle_message_command_ack(mavlink_message_t *msg)
{
	mavlink_command_ack_t ack;
	mavlink_msg_command_ack_decode(msg, &ack);

	// We should not clog the command_ack queue with acks that are not for us.
	// Therefore, we drop them early and move on.
	bool target_ok = evaluate_target_ok(0, ack.target_system, ack.target_component);

	if (!target_ok) {
		PX4_DEBUG("Drop ack %d for %d from %d/%d to %d/%d\n",
			  ack.result, ack.command, msg->sysid, msg->compid, ack.target_system, ack.target_component);
		return;
	}

	MavlinkCommandSender::instance().handle_mavlink_command_ack(ack, msg->sysid, msg->compid, _mavlink.get_channel());

	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.result_param2 = ack.result_param2;
	command_ack.command = ack.command;
	command_ack.result = ack.result;
	command_ack.from_external = true;
	command_ack.result_param1 = ack.progress;
	command_ack.target_system = ack.target_system;
	command_ack.target_component = ack.target_component;

	_cmd_ack_pub.publish(command_ack);

	// TODO: move it to the same place that sent the command
	if (ack.result == MAV_RESULT_FAILED) {
		if (msg->compid == MAV_COMP_ID_CAMERA) {
			PX4_WARN("Got unsuccessful result %" PRIu8 " from camera", ack.result);
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink.get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_MAVLINK;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	const matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}

void
MavlinkReceiver::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink.get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	const matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}

void
MavlinkReceiver::handle_message_set_mode(mavlink_message_t *msg)
{
	mavlink_set_mode_t new_mode;
	mavlink_msg_set_mode_decode(msg, &new_mode);

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;

	vehicle_command_s vcmd{};

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

	_cmd_pub.publish(vcmd);
}

void
MavlinkReceiver::handle_message_distance_sensor(mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	distance_sensor_s ds{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink.get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_MAVLINK;

	ds.timestamp        = hrt_absolute_time(); /* Use system time for now, don't trust sender to attach correct timestamp */
	ds.min_distance     = static_cast<float>(dist_sensor.min_distance) * 1e-2f;     /* cm to m */
	ds.max_distance     = static_cast<float>(dist_sensor.max_distance) * 1e-2f;     /* cm to m */
	ds.current_distance = static_cast<float>(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	ds.variance         = dist_sensor.covariance * 1e-4f;                           /* cm^2 to m^2 */
	ds.h_fov            = dist_sensor.horizontal_fov;
	ds.v_fov            = dist_sensor.vertical_fov;
	ds.q[0]             = dist_sensor.quaternion[0];
	ds.q[1]             = dist_sensor.quaternion[1];
	ds.q[2]             = dist_sensor.quaternion[2];
	ds.q[3]             = dist_sensor.quaternion[3];
	ds.type             = dist_sensor.type;
	ds.device_id        = device_id.devid;
	ds.orientation      = dist_sensor.orientation;

	// MAVLink DISTANCE_SENSOR signal_quality value of 0 means unset/unknown
	// quality value. Also it comes normalised between 1 and 100 while the uORB
	// signal quality is normalised between 0 and 100.
	ds.signal_quality = dist_sensor.signal_quality == 0 ? -1 : 100 * (dist_sensor.signal_quality - 1) / 99;

	_distance_sensor_pub.publish(ds);
}

void
MavlinkReceiver::handle_message_att_pos_mocap(mavlink_message_t *msg)
{
	mavlink_att_pos_mocap_t att_pos_mocap;
	mavlink_msg_att_pos_mocap_decode(msg, &att_pos_mocap);

	// fill vehicle_odometry from Mavlink ATT_POS_MOCAP
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = _mavlink_timesync.sync_stamp(att_pos_mocap.time_usec);

	odom.q[0] = att_pos_mocap.q[0];
	odom.q[1] = att_pos_mocap.q[1];
	odom.q[2] = att_pos_mocap.q[2];
	odom.q[3] = att_pos_mocap.q[3];

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = att_pos_mocap.x;
	odom.position[1] = att_pos_mocap.y;
	odom.position[2] = att_pos_mocap.z;

	// ATT_POS_MOCAP covariance
	//  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle
	//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = att_pos_mocap.covariance[0];  // X  row 0, col 0
	odom.position_variance[1] = att_pos_mocap.covariance[6];  // Y  row 1, col 1
	odom.position_variance[2] = att_pos_mocap.covariance[11]; // Z  row 2, col 2

	odom.orientation_variance[0] = att_pos_mocap.covariance[15]; // R  row 3, col 3
	odom.orientation_variance[1] = att_pos_mocap.covariance[18]; // P  row 4, col 4
	odom.orientation_variance[2] = att_pos_mocap.covariance[20]; // Y  row 5, col 5

	odom.timestamp = hrt_absolute_time();

	_mocap_odometry_pub.publish(odom);
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &target_local_ned);

	/* Only accept messages which are intended for this system */
	if (_mavlink.get_forward_externalsp() &&
	    (mavlink_system.sysid == target_local_ned.target_system || target_local_ned.target_system == 0) &&
	    (mavlink_system.compid == target_local_ned.target_component || target_local_ned.target_component == 0)) {

		trajectory_setpoint_s setpoint{};

		const uint16_t type_mask = target_local_ned.type_mask;

		if (target_local_ned.coordinate_frame == MAV_FRAME_LOCAL_NED) {
			setpoint.position[0] = (type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) ? (float)NAN : target_local_ned.x;
			setpoint.position[1] = (type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) ? (float)NAN : target_local_ned.y;
			setpoint.position[2] = (type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) ? (float)NAN : target_local_ned.z;

			setpoint.velocity[0] = (type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? (float)NAN : target_local_ned.vx;
			setpoint.velocity[1] = (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? (float)NAN : target_local_ned.vy;
			setpoint.velocity[2] = (type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? (float)NAN : target_local_ned.vz;

			setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_local_ned.afx;
			setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_local_ned.afy;
			setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_local_ned.afz;

		} else if (target_local_ned.coordinate_frame == MAV_FRAME_BODY_NED) {

			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);
			const matrix::Dcmf R{matrix::Quatf{vehicle_attitude.q}};

			const bool ignore_velocity = type_mask & (POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
						     POSITION_TARGET_TYPEMASK_VZ_IGNORE);

			if (!ignore_velocity) {
				const matrix::Vector3f velocity_body_sp{
					(type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? 0.f : target_local_ned.vx,
					(type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? 0.f : target_local_ned.vy,
					(type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? 0.f : target_local_ned.vz
				};


				const float yaw = matrix::Eulerf{R}(2);

				setpoint.velocity[0] = cosf(yaw) * velocity_body_sp(0) - sinf(yaw) * velocity_body_sp(1);
				setpoint.velocity[1] = sinf(yaw) * velocity_body_sp(0) + cosf(yaw) * velocity_body_sp(1);
				setpoint.velocity[2] = velocity_body_sp(2);

			} else {
				matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.velocity);
			}

			const bool ignore_acceleration = type_mask & (POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |
							 POSITION_TARGET_TYPEMASK_AZ_IGNORE);

			if (!ignore_acceleration) {
				const matrix::Vector3f acceleration_body_sp{
					(type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? 0.f : target_local_ned.afx,
					(type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? 0.f : target_local_ned.afy,
					(type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? 0.f : target_local_ned.afz
				};

				const matrix::Vector3f acceleration_setpoint{R * acceleration_body_sp};
				acceleration_setpoint.copyTo(setpoint.acceleration);

			} else {
				matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.acceleration);
			}

			matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.position);

		} else {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_LOCAL_NED coordinate frame %" PRIu8 " unsupported\t",
					     target_local_ned.coordinate_frame);
			events::send<uint8_t>(events::ID("mavlink_rcv_sp_target_unsup_coord"), events::Log::Error,
					      "SET_POSITION_TARGET_LOCAL_NED: coordinate frame {1} unsupported", target_local_ned.coordinate_frame);
			return;
		}

		setpoint.yaw      = (type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)      ? (float)NAN : target_local_ned.yaw;
		setpoint.yawspeed = (type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) ? (float)NAN : target_local_ned.yaw_rate;


		offboard_control_mode_s ocm{};
		ocm.position = !matrix::Vector3f(setpoint.position).isAllNan();
		ocm.velocity = !matrix::Vector3f(setpoint.velocity).isAllNan();
		ocm.acceleration = !matrix::Vector3f(setpoint.acceleration).isAllNan();

		if (ocm.acceleration && (type_mask & POSITION_TARGET_TYPEMASK_FORCE_SET)) {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_LOCAL_NED force not supported\t");
			events::send(events::ID("mavlink_rcv_sp_target_unsup_force"), events::Log::Error,
				     "SET_POSITION_TARGET_LOCAL_NED: FORCE is not supported");
			return;
		}

		if (ocm.position || ocm.velocity || ocm.acceleration) {
			// publish offboard_control_mode
			ocm.timestamp = hrt_absolute_time();
			_offboard_control_mode_pub.publish(ocm);

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				// only publish setpoint once in OFFBOARD
				setpoint.timestamp = hrt_absolute_time();
				_trajectory_setpoint_pub.publish(setpoint);
			}

		} else {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_LOCAL_NED invalid\t");
			events::send(events::ID("mavlink_rcv_sp_target_invalid"), events::Log::Error,
				     "SET_POSITION_TARGET_LOCAL_NED: invalid, missing position, velocity or acceleration");
		}
	}
}

void
MavlinkReceiver::handle_message_set_position_target_global_int(mavlink_message_t *msg)
{
	mavlink_set_position_target_global_int_t target_global_int;
	mavlink_msg_set_position_target_global_int_decode(msg, &target_global_int);

	/* Only accept messages which are intended for this system */
	if (_mavlink.get_forward_externalsp() &&
	    (mavlink_system.sysid == target_global_int.target_system || target_global_int.target_system == 0) &&
	    (mavlink_system.compid == target_global_int.target_component || target_global_int.target_component == 0)) {

		trajectory_setpoint_s setpoint{};

		const uint16_t type_mask = target_global_int.type_mask;

		// position
		if (!(type_mask & (POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
				   POSITION_TARGET_TYPEMASK_Z_IGNORE))) {

			vehicle_local_position_s local_pos{};
			_vehicle_local_position_sub.copy(&local_pos);

			if (!local_pos.xy_global || !local_pos.z_global) {
				return;
			}

			MapProjection global_local_proj_ref{local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp};

			// global -> local
			const double lat = target_global_int.lat_int / 1e7;
			const double lon = target_global_int.lon_int / 1e7;
			global_local_proj_ref.project(lat, lon, setpoint.position[0], setpoint.position[1]);

			if (target_global_int.coordinate_frame == MAV_FRAME_GLOBAL_INT) {
				setpoint.position[2] = local_pos.ref_alt - target_global_int.alt;

			} else if (target_global_int.coordinate_frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
				home_position_s home_position{};
				_home_position_sub.copy(&home_position);

				if (home_position.valid_alt) {
					const float alt = home_position.alt - target_global_int.alt;
					setpoint.position[2] = alt - local_pos.ref_alt;

				} else {
					// home altitude required
					return;
				}

			} else if (target_global_int.coordinate_frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
				vehicle_global_position_s vehicle_global_position{};
				_vehicle_global_position_sub.copy(&vehicle_global_position);

				if (vehicle_global_position.terrain_alt_valid) {
					const float alt = target_global_int.alt + vehicle_global_position.terrain_alt;
					setpoint.position[2] = local_pos.ref_alt - alt;

				} else {
					// valid terrain alt required
					return;
				}

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_GLOBAL_INT invalid coordinate frame %" PRIu8 "\t",
						     target_global_int.coordinate_frame);
				events::send<uint8_t>(events::ID("mavlink_rcv_sp_target_gl_invalid_coord"), events::Log::Error,
						      "SET_POSITION_TARGET_GLOBAL_INT invalid coordinate frame {1}", target_global_int.coordinate_frame);
				return;
			}

		} else {
			matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.position);
		}

		// velocity
		setpoint.velocity[0] = (type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? (float)NAN : target_global_int.vx;
		setpoint.velocity[1] = (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? (float)NAN : target_global_int.vy;
		setpoint.velocity[2] = (type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? (float)NAN : target_global_int.vz;

		// acceleration
		setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_global_int.afx;
		setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_global_int.afy;
		setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_global_int.afz;

		setpoint.yaw      = (type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)      ? (float)NAN : target_global_int.yaw;
		setpoint.yawspeed = (type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) ? (float)NAN : target_global_int.yaw_rate;


		offboard_control_mode_s ocm{};
		ocm.position = !matrix::Vector3f(setpoint.position).isAllNan();
		ocm.velocity = !matrix::Vector3f(setpoint.velocity).isAllNan();
		ocm.acceleration = !matrix::Vector3f(setpoint.acceleration).isAllNan();

		if (ocm.acceleration && (type_mask & POSITION_TARGET_TYPEMASK_FORCE_SET)) {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_GLOBAL_INT force not supported\t");
			events::send(events::ID("mavlink_rcv_sp_target_gl_unsup_force"), events::Log::Error,
				     "SET_POSITION_TARGET_GLOBAL_INT: FORCE is not supported");
			return;
		}

		if (ocm.position || ocm.velocity || ocm.acceleration) {
			// publish offboard_control_mode
			ocm.timestamp = hrt_absolute_time();
			_offboard_control_mode_pub.publish(ocm);

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				// only publish setpoint once in OFFBOARD
				setpoint.timestamp = hrt_absolute_time();
				_trajectory_setpoint_pub.publish(setpoint);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_set_gps_global_origin(mavlink_message_t *msg)
{
	mavlink_set_gps_global_origin_t gps_global_origin;
	mavlink_msg_set_gps_global_origin_decode(msg, &gps_global_origin);

	if (gps_global_origin.target_system == _mavlink.get_system_id()) {
		vehicle_command_s vcmd{};
		vcmd.param5 = (double)gps_global_origin.latitude * 1.e-7;
		vcmd.param6 = (double)gps_global_origin.longitude * 1.e-7;
		vcmd.param7 = (float)gps_global_origin.altitude * 1.e-3f;
		vcmd.command = vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
		vcmd.target_system = _mavlink.get_system_id();
		vcmd.target_component = MAV_COMP_ID_ALL;
		vcmd.source_system = msg->sysid;
		vcmd.source_component = msg->compid;
		vcmd.confirmation = false;
		vcmd.from_external = true;
		vcmd.timestamp = hrt_absolute_time();
		_cmd_pub.publish(vcmd);
	}

	handle_request_message_command(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN);
}

#if defined(MAVLINK_MSG_ID_SET_VELOCITY_LIMITS) // For now only defined if development.xml is used
void MavlinkReceiver::handle_message_set_velocity_limits(mavlink_message_t *msg)
{
	mavlink_set_velocity_limits_t mavlink_set_velocity_limits;
	mavlink_msg_set_velocity_limits_decode(msg, &mavlink_set_velocity_limits);

	velocity_limits_s velocity_limits{};
	velocity_limits.horizontal_velocity = mavlink_set_velocity_limits.horizontal_speed_limit;
	velocity_limits.vertical_velocity = mavlink_set_velocity_limits.vertical_speed_limit;
	velocity_limits.yaw_rate = mavlink_set_velocity_limits.yaw_rate_limit;
	velocity_limits.timestamp = hrt_absolute_time();
	_velocity_limits_pub.publish(velocity_limits);
}
#endif // MAVLINK_MSG_ID_SET_VELOCITY_LIMITS

void
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t vpe;
	mavlink_msg_vision_position_estimate_decode(msg, &vpe);

	// fill vehicle_odometry from Mavlink VISION_POSITION_ESTIMATE
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = _mavlink_timesync.sync_stamp(vpe.usec);

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = vpe.x;
	odom.position[1] = vpe.y;
	odom.position[2] = vpe.z;

	const matrix::Quatf q(matrix::Eulerf(vpe.roll, vpe.pitch, vpe.yaw));
	q.copyTo(odom.q);

	// VISION_POSITION_ESTIMATE covariance
	//  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle
	//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = vpe.covariance[0];  // X  row 0, col 0
	odom.position_variance[1] = vpe.covariance[6];  // Y  row 1, col 1
	odom.position_variance[2] = vpe.covariance[11]; // Z  row 2, col 2

	odom.orientation_variance[0] = vpe.covariance[15]; // R  row 3, col 3
	odom.orientation_variance[1] = vpe.covariance[18]; // P  row 4, col 4
	odom.orientation_variance[2] = vpe.covariance[20]; // Y  row 5, col 5

	odom.reset_counter = vpe.reset_counter;

	odom.timestamp = hrt_absolute_time();

	_visual_odometry_pub.publish(odom);
}

void
MavlinkReceiver::handle_message_odometry(mavlink_message_t *msg)
{
	mavlink_odometry_t odom_in;
	mavlink_msg_odometry_decode(msg, &odom_in);

	// fill vehicle_odometry from Mavlink ODOMETRY
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = _mavlink_timesync.sync_stamp(odom_in.time_usec);

	const matrix::Vector3f odom_in_p(odom_in.x, odom_in.y, odom_in.z);

	// position x/y/z (m)
	if (odom_in_p.isAllFinite()) {
		// frame_id: Coordinate frame of reference for the pose data.
		switch (odom_in.frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom.position[0] =  odom_in.y; // y: North
			odom.position[1] =  odom_in.x; // x: East
			odom.position[2] = -odom_in.z; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom.position[0] =  odom_in.x; // x: Forward
			odom.position[1] = -odom_in.y; // y: Left
			odom.position[2] = -odom_in.z; // z: Up
			break;

		default:
			break;
		}

		// pose_covariance
		//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw)
		//  first six entries are the first ROW, next five entries are the second ROW, etc.
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
				// position variances copied directly
				odom.position_variance[0] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[1] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.position_variance[0] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[1] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			default:
				break;
			}
		}
	}

	// q: the quaternion of the ODOMETRY msg represents a rotation from body frame to a local frame
	if (matrix::Quatf(odom_in.q).isAllFinite()) {

		odom.q[0] = odom_in.q[0];
		odom.q[1] = odom_in.q[1];
		odom.q[2] = odom_in.q[2];
		odom.q[3] = odom_in.q[3];

		// pose_covariance (roll, pitch, yaw)
		//  states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix pose_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			odom.orientation_variance[0] = odom_in.pose_covariance[15]; // R  row 3, col 3
			odom.orientation_variance[1] = odom_in.pose_covariance[18]; // P  row 4, col 4
			odom.orientation_variance[2] = odom_in.pose_covariance[20]; // Y  row 5, col 5
		}
	}

	const matrix::Vector3f odom_in_v(odom_in.vx, odom_in.vy, odom_in.vz);

	// velocity vx/vy/vz (m/s)
	if (odom_in_v.isAllFinite()) {
		// child_frame_id: Coordinate frame of reference for the velocity in free space (twist) data.
		switch (odom_in.child_frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom.velocity[0] =  odom_in.vy; // y: East
			odom.velocity[1] =  odom_in.vx; // x: North
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom.velocity[0] =  odom_in.vx; // x: Forward
			odom.velocity[1] = -odom_in.vy; // y: Left
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
			odom.velocity[0] = odom_in.vx;
			odom.velocity[1] = odom_in.vy;
			odom.velocity[2] = odom_in.vz;
			break;

		default:
			// unsupported child_frame_id
			break;
		}

		// velocity_covariance (vx, vy, vz)
		//  states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix velocity_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.child_frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
			case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_FRD:
				// velocity covariances copied directly
				odom.velocity_variance[0] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[1] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.velocity_variance[0] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[1] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			default:
				// unsupported child_frame_id
				break;
			}
		}
	}

	// Roll/Pitch/Yaw angular speed (rad/s)
	if (PX4_ISFINITE(odom_in.rollspeed)
	    && PX4_ISFINITE(odom_in.pitchspeed)
	    && PX4_ISFINITE(odom_in.yawspeed)) {

		odom.angular_velocity[0] = odom_in.rollspeed;
		odom.angular_velocity[1] = odom_in.pitchspeed;
		odom.angular_velocity[2] = odom_in.yawspeed;
	}

	odom.reset_counter = odom_in.reset_counter;
	odom.quality = odom_in.quality;

	switch (odom_in.estimator_type) {
	case MAV_ESTIMATOR_TYPE_UNKNOWN: // accept MAV_ESTIMATOR_TYPE_UNKNOWN for legacy support
	case MAV_ESTIMATOR_TYPE_NAIVE:
	case MAV_ESTIMATOR_TYPE_VISION:
	case MAV_ESTIMATOR_TYPE_VIO:
		odom.timestamp = hrt_absolute_time();
		_visual_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_MOCAP:
		odom.timestamp = hrt_absolute_time();
		_mocap_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_GPS:
	case MAV_ESTIMATOR_TYPE_GPS_INS:
	case MAV_ESTIMATOR_TYPE_LIDAR:
	case MAV_ESTIMATOR_TYPE_AUTOPILOT:
	default:
		mavlink_log_critical(&_mavlink_log_pub, "ODOMETRY: estimator_type %" PRIu8 " unsupported\t",
				     odom_in.estimator_type);
		events::send<uint8_t>(events::ID("mavlink_rcv_odom_unsup_estimator_type"), events::Log::Error,
				      "ODOMETRY: unsupported estimator_type {1}", odom_in.estimator_type);
		return;
	}
}

void MavlinkReceiver::fill_thrust(float *thrust_body_array, uint8_t vehicle_type, float thrust)
{
	// Fill correct field by checking frametype
	// TODO: add as needed
	switch (_mavlink.get_system_type()) {
	case MAV_TYPE_GENERIC:
		break;

	case MAV_TYPE_FIXED_WING:
	case MAV_TYPE_GROUND_ROVER:
		thrust_body_array[0] = thrust;
		break;

	case MAV_TYPE_QUADROTOR:
	case MAV_TYPE_HEXAROTOR:
	case MAV_TYPE_OCTOROTOR:
	case MAV_TYPE_TRICOPTER:
	case MAV_TYPE_HELICOPTER:
	case MAV_TYPE_COAXIAL:
		thrust_body_array[2] = -thrust;
		break;

	case MAV_TYPE_SUBMARINE:
		thrust_body_array[0] = thrust;
		break;

	case MAV_TYPE_VTOL_TAILSITTER_DUOROTOR:
	case MAV_TYPE_VTOL_TAILSITTER_QUADROTOR:
	case MAV_TYPE_VTOL_TILTROTOR:
	case MAV_TYPE_VTOL_FIXEDROTOR:
	case MAV_TYPE_VTOL_TAILSITTER:
	case MAV_TYPE_VTOL_TILTWING:
	case MAV_TYPE_VTOL_RESERVED5:
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			thrust_body_array[0] = thrust;

			break;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			thrust_body_array[2] = -thrust;

			break;

		default:
			// This should never happen
			break;
		}

		break;
	}
}

void
MavlinkReceiver::handle_message_set_attitude_target(mavlink_message_t *msg)
{
	mavlink_set_attitude_target_t attitude_target;
	mavlink_msg_set_attitude_target_decode(msg, &attitude_target);

	/* Only accept messages which are intended for this system */
	if (_mavlink.get_forward_externalsp() &&
	    (mavlink_system.sysid == attitude_target.target_system || attitude_target.target_system == 0) &&
	    (mavlink_system.compid == attitude_target.target_component || attitude_target.target_component == 0)) {

		const uint8_t type_mask = attitude_target.type_mask;

		const bool attitude = !(type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE);
		const bool body_rates = !(type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)
					&& !(type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE);
		const bool thrust_body = (type_mask & ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET);

		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (attitude || body_rates) {
			offboard_control_mode_s ocm{};
			ocm.attitude = attitude;
			ocm.body_rate = body_rates;
			ocm.timestamp = hrt_absolute_time();
			_offboard_control_mode_pub.publish(ocm);
		}

		if (attitude) {
			vehicle_attitude_setpoint_s attitude_setpoint{};

			const matrix::Quatf q{attitude_target.q};
			q.copyTo(attitude_setpoint.q_d);

			// TODO: review use case
			attitude_setpoint.yaw_sp_move_rate = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE) ?
							     (float)NAN : attitude_target.body_yaw_rate;

			if (!thrust_body && !(attitude_target.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE)) {
				fill_thrust(attitude_setpoint.thrust_body, vehicle_status.vehicle_type, attitude_target.thrust);

			} else if (thrust_body) {
				attitude_setpoint.thrust_body[0] = attitude_target.thrust_body[0];
				attitude_setpoint.thrust_body[1] = attitude_target.thrust_body[1];
				attitude_setpoint.thrust_body[2] = attitude_target.thrust_body[2];
			}

			// Publish attitude setpoint only once in OFFBOARD
			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				attitude_setpoint.timestamp = hrt_absolute_time();

				if (vehicle_status.is_vtol && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)) {
					_mc_virtual_att_sp_pub.publish(attitude_setpoint);

				} else if (vehicle_status.is_vtol && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
					_fw_virtual_att_sp_pub.publish(attitude_setpoint);

				} else {
					_att_sp_pub.publish(attitude_setpoint);
				}
			}

		}

		if (body_rates) {
			vehicle_rates_setpoint_s setpoint{};
			setpoint.roll  = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)  ? (float)NAN :
					 attitude_target.body_roll_rate;
			setpoint.pitch = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE) ? (float)NAN :
					 attitude_target.body_pitch_rate;
			setpoint.yaw   = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)   ? (float)NAN :
					 attitude_target.body_yaw_rate;

			if (!(attitude_target.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE)) {
				fill_thrust(setpoint.thrust_body, vehicle_status.vehicle_type, attitude_target.thrust);
			}

			// Publish rate setpoint only once in OFFBOARD
			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				setpoint.timestamp = hrt_absolute_time();
				_rates_sp_pub.publish(setpoint);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink.get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		radio_status_s status{};

		status.timestamp = hrt_absolute_time();
		status.rssi = rstatus.rssi;
		status.remote_rssi = rstatus.remrssi;
		status.txbuf = rstatus.txbuf;
		status.noise = rstatus.noise;
		status.remote_noise = rstatus.remnoise;
		status.rxerrors = rstatus.rxerrors;
		status.fix = rstatus.fixed;

		_mavlink.update_radio_status(status);

		_radio_status_pub.publish(status);
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
		mavlink_msg_ping_send_struct(_mavlink.get_channel(), &ping);

	} else if ((ping.target_system == mavlink_system.sysid) &&
		   (ping.target_component ==
		    mavlink_system.compid)) { // This is a returned ping message from this system. Calculate latency from it.

		const hrt_abstime now = hrt_absolute_time();

		// Calculate round trip time
		float rtt_ms = (now - ping.time_usec) / 1000.0f;

		// Update ping statistics
		struct Mavlink::ping_statistics_s &pstats = _mavlink.get_ping_statistics();

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
		if (_mavlink.get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

			ping_s uorb_ping_msg{};

			uorb_ping_msg.timestamp = now;
			uorb_ping_msg.ping_time = ping.time_usec;
			uorb_ping_msg.ping_sequence = ping.seq;
			uorb_ping_msg.dropped_packets = pstats.dropped_packets;
			uorb_ping_msg.rtt_ms = rtt_ms;
			uorb_ping_msg.system_id = msg->sysid;
			uorb_ping_msg.component_id = msg->compid;

			_ping_pub.publish(uorb_ping_msg);
		}
	}
}

void
MavlinkReceiver::handle_message_battery_status(mavlink_message_t *msg)
{
	if ((msg->sysid != mavlink_system.sysid) || (msg->compid == mavlink_system.compid)) {
		// ignore battery status coming from other systems or from the autopilot itself
		return;
	}

	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status{};
	battery_status.timestamp = hrt_absolute_time();

	float voltage_sum = 0.0f;
	uint8_t cell_count = 0;

	while ((cell_count < 10) && (battery_mavlink.voltages[cell_count] < UINT16_MAX)) {
		battery_status.voltage_cell_v[cell_count] = (float)(battery_mavlink.voltages[cell_count]) / 1000.0f;
		voltage_sum += battery_status.voltage_cell_v[cell_count];
		cell_count++;
	}

	battery_status.voltage_v = voltage_sum;
	battery_status.current_a = (float)(battery_mavlink.current_battery) / 100.0f;
	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.discharged_mah = (float)battery_mavlink.current_consumed;
	battery_status.cell_count = cell_count;
	battery_status.temperature = (float)battery_mavlink.temperature;
	battery_status.connected = true;

	// Set the battery warning based on remaining charge.
	//  Note: Smallest values must come first in evaluation.
	if (battery_status.remaining < _param_bat_emergen_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (battery_status.remaining < _param_bat_crit_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (battery_status.remaining < _param_bat_low_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	_battery_pub.publish(battery_status);
}

void
MavlinkReceiver::handle_message_serial_control(mavlink_message_t *msg)
{
	mavlink_serial_control_t serial_control_mavlink;
	mavlink_msg_serial_control_decode(msg, &serial_control_mavlink);

	// Check if the message is targeted at us.
	if ((serial_control_mavlink.target_system != 0 &&
	     mavlink_system.sysid != serial_control_mavlink.target_system) ||
	    (serial_control_mavlink.target_component != 0 &&
	     mavlink_system.compid != serial_control_mavlink.target_component)) {
		return;
	}

	// we only support shell commands
	if (serial_control_mavlink.device != SERIAL_CONTROL_DEV_SHELL
	    || (serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_REPLY)) {
		return;
	}

	MavlinkShell *shell = _mavlink.get_shell();

	if (shell) {
		// we ignore the timeout, EXCLUSIVE & BLOCKING flags of the SERIAL_CONTROL message
		if (serial_control_mavlink.count > 0 && serial_control_mavlink.count <= sizeof(serial_control_mavlink.data)) {
			shell->setTargetID(msg->sysid, msg->compid);
			shell->write(serial_control_mavlink.data, serial_control_mavlink.count);
		}

		// if no response requested, assume the shell is no longer used
		if ((serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
			_mavlink.close_shell();
		}
	}
}

void
MavlinkReceiver::handle_message_logging_ack(mavlink_message_t *msg)
{
	mavlink_logging_ack_t logging_ack;
	mavlink_msg_logging_ack_decode(msg, &logging_ack);

	MavlinkULog *ulog_streaming = _mavlink.get_ulog_streaming();

	if (ulog_streaming) {
		ulog_streaming->handle_ack(logging_ack);
	}
}

void
MavlinkReceiver::handle_message_play_tune(mavlink_message_t *msg)
{
	mavlink_play_tune_t play_tune;
	mavlink_msg_play_tune_decode(msg, &play_tune);

	if ((mavlink_system.sysid == play_tune.target_system || play_tune.target_system == 0) &&
	    (mavlink_system.compid == play_tune.target_component || play_tune.target_component == 0)) {

		// Let's make sure the input is 0 terminated, so we don't ever overrun it.
		play_tune.tune2[sizeof(play_tune.tune2) - 1] = '\0';

		schedule_tune(play_tune.tune);
	}
}

void
MavlinkReceiver::handle_message_play_tune_v2(mavlink_message_t *msg)
{
	mavlink_play_tune_v2_t play_tune_v2;
	mavlink_msg_play_tune_v2_decode(msg, &play_tune_v2);

	if ((mavlink_system.sysid == play_tune_v2.target_system || play_tune_v2.target_system == 0) &&
	    (mavlink_system.compid == play_tune_v2.target_component || play_tune_v2.target_component == 0)) {

		if (play_tune_v2.format != TUNE_FORMAT_QBASIC1_1) {
			PX4_ERR("Tune format %" PRIu32 " not supported", play_tune_v2.format);
			return;
		}

		// Let's make sure the input is 0 terminated, so we don't ever overrun it.
		play_tune_v2.tune[sizeof(play_tune_v2.tune) - 1] = '\0';

		schedule_tune(play_tune_v2.tune);
	}
}

void MavlinkReceiver::schedule_tune(const char *tune)
{
	// We only allocate the TunePublisher object if we ever use it but we
	// don't remove it to avoid fragmentation over time.
	if (_tune_publisher == nullptr) {
		_tune_publisher = new TunePublisher();

		if (_tune_publisher == nullptr) {
			PX4_ERR("Could not allocate tune publisher");
			return;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	_tune_publisher->set_tune_string(tune, now);
	// Send first one straightaway.
	_tune_publisher->publish_next_tune(now);
}


void
MavlinkReceiver::handle_message_obstacle_distance(mavlink_message_t *msg)
{
	mavlink_obstacle_distance_t mavlink_obstacle_distance;
	mavlink_msg_obstacle_distance_decode(msg, &mavlink_obstacle_distance);

	obstacle_distance_s obstacle_distance{};

	obstacle_distance.timestamp = hrt_absolute_time();
	obstacle_distance.sensor_type = mavlink_obstacle_distance.sensor_type;
	memcpy(obstacle_distance.distances, mavlink_obstacle_distance.distances, sizeof(obstacle_distance.distances));

	if (mavlink_obstacle_distance.increment_f > 0.f) {
		obstacle_distance.increment = mavlink_obstacle_distance.increment_f;

	} else {
		obstacle_distance.increment = (float)mavlink_obstacle_distance.increment;
	}

	obstacle_distance.min_distance = mavlink_obstacle_distance.min_distance;
	obstacle_distance.max_distance = mavlink_obstacle_distance.max_distance;
	obstacle_distance.angle_offset = mavlink_obstacle_distance.angle_offset;
	obstacle_distance.frame = mavlink_obstacle_distance.frame;

	_obstacle_distance_pub.publish(obstacle_distance);
}

void
MavlinkReceiver::handle_message_tunnel(mavlink_message_t *msg)
{
	mavlink_tunnel_t mavlink_tunnel;
	mavlink_msg_tunnel_decode(msg, &mavlink_tunnel);

	mavlink_tunnel_s tunnel{};

	tunnel.timestamp = hrt_absolute_time();
	tunnel.payload_type = mavlink_tunnel.payload_type;
	tunnel.target_system = mavlink_tunnel.target_system;
	tunnel.target_component = mavlink_tunnel.target_component;
	tunnel.payload_length = mavlink_tunnel.payload_length;
	memcpy(tunnel.payload, mavlink_tunnel.payload, sizeof(tunnel.payload));
	static_assert(sizeof(tunnel.payload) == sizeof(mavlink_tunnel.payload), "mavlink_tunnel.payload size mismatch");

	_mavlink_tunnel_pub.publish(tunnel);

}

void
MavlinkReceiver::handle_message_rc_channels_override(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	// Check target
	if (man.target_system != 0 && man.target_system != _mavlink.get_system_id()) {
		return;
	}

	// fill uORB message
	input_rc_s rc{};
	// metadata
	rc.timestamp = rc.timestamp_last_signal = hrt_absolute_time();
	rc.rssi = input_rc_s::RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
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

#if defined(ATL_MANTIS_RC_INPUT_HACKS)

	// Sanity checking if the RC controller is really sending.
	if (rc.values[5] == 2048 && rc.values[6] == 0 && (rc.values[8] & 0xff00) == 0x0C00) {
		rc.values[0] = 1000 + (uint16_t)((float)rc.values[0] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[1] = 1000 + (uint16_t)((float)rc.values[1] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[2] = 1000 + (uint16_t)((float)rc.values[2] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[3] = 1000 + (uint16_t)((float)rc.values[3] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[4] = 1000 + (uint16_t)((float)rc.values[4] / 4095.f * 1000.f); // 0..4095 -> 1000..2000 -> Aux 2
		rc.values[5] = 1000 + rc.values[7] * 500; // Switch toggles from 0 to 2, we map it from 1000 to 2000
		rc.values[6] = 1000 + (((rc.values[8] & 0x02) != 0) ? 1000 : 0); // Return button.
		rc.values[7] = 1000 + (((rc.values[8] & 0x01) != 0) ? 1000 : 0); // Video record button. -> Aux4
		rc.values[9] = 1000 + (((rc.values[8] & 0x10) != 0) ? 1000 : 0); // Photo record button. -> Aux3
		rc.values[8] = rc.values[8] & 0x00ff; // Remove magic identifier.
		// leave rc.values[10] as is 0..4095
		//
		// Note we are currently ignoring the stick buttons (sticks pressed down).
		// They are on rc.values[8] & 0x20 and rc.values[8] & 0x40.
	}

#endif

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

	rc.link_quality = -1;
	rc.rssi_dbm = NAN;

	// publish uORB message
	_rc_pub.publish(rc);
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t mavlink_manual_control;
	mavlink_msg_manual_control_decode(msg, &mavlink_manual_control);

	// Check target
	if (mavlink_manual_control.target != 0 && mavlink_manual_control.target != _mavlink.get_system_id()) {
		return;
	}

	manual_control_setpoint_s manual_control_setpoint{};

	if (math::isInRange((int)mavlink_manual_control.x, -1000, 1000)) { manual_control_setpoint.pitch = mavlink_manual_control.x / 1000.f; }

	if (math::isInRange((int)mavlink_manual_control.y, -1000, 1000)) { manual_control_setpoint.roll = mavlink_manual_control.y / 1000.f; }

	// For backwards compatibility at the moment interpret throttle in range [0,1000]
	if (math::isInRange((int)mavlink_manual_control.z, 0, 1000)) { manual_control_setpoint.throttle = ((mavlink_manual_control.z / 1000.f) * 2.f) - 1.f; }

	if (math::isInRange((int)mavlink_manual_control.r, -1000, 1000)) { manual_control_setpoint.yaw = mavlink_manual_control.r / 1000.f; }

	// Pass along the button states
	manual_control_setpoint.buttons = mavlink_manual_control.buttons;

	if (mavlink_manual_control.enabled_extensions & (1u << 2)
	    && math::isInRange((int)mavlink_manual_control.aux1, -1000, 1000)) { manual_control_setpoint.aux1 = mavlink_manual_control.aux1 / 1000.0f; }

	if (mavlink_manual_control.enabled_extensions & (1u << 3)
	    && math::isInRange((int)mavlink_manual_control.aux2, -1000, 1000)) { manual_control_setpoint.aux2 = mavlink_manual_control.aux2 / 1000.0f; }

	if (mavlink_manual_control.enabled_extensions & (1u << 4)
	    && math::isInRange((int)mavlink_manual_control.aux3, -1000, 1000)) { manual_control_setpoint.aux3 = mavlink_manual_control.aux3 / 1000.0f; }

	if (mavlink_manual_control.enabled_extensions & (1u << 5)
	    && math::isInRange((int)mavlink_manual_control.aux4, -1000, 1000)) { manual_control_setpoint.aux4 = mavlink_manual_control.aux4 / 1000.0f; }

	if (mavlink_manual_control.enabled_extensions & (1u << 6)
	    && math::isInRange((int)mavlink_manual_control.aux5, -1000, 1000)) { manual_control_setpoint.aux5 = mavlink_manual_control.aux5 / 1000.0f; }

	if (mavlink_manual_control.enabled_extensions & (1u << 7)
	    && math::isInRange((int)mavlink_manual_control.aux6, -1000, 1000)) { manual_control_setpoint.aux6 = mavlink_manual_control.aux6 / 1000.0f; }

	manual_control_setpoint.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0 + _mavlink.get_instance_id();
	manual_control_setpoint.timestamp = manual_control_setpoint.timestamp_sample = hrt_absolute_time();
	manual_control_setpoint.valid = true;
	_manual_control_input_pub.publish(manual_control_setpoint);
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink.get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

		const hrt_abstime now = hrt_absolute_time();

		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		const bool same_system = (msg->sysid == mavlink_system.sysid);

		if (same_system || hb.type == MAV_TYPE_GCS) {

			camera_status_s camera_status{};

			switch (hb.type) {
			case MAV_TYPE_ANTENNA_TRACKER:
				_heartbeat_type_antenna_tracker = now;
				break;

			case MAV_TYPE_GCS:
				_heartbeat_type_gcs = now;
				break;

			case MAV_TYPE_ONBOARD_CONTROLLER:
				_heartbeat_type_onboard_controller = now;
				break;

			case MAV_TYPE_GIMBAL:
				_heartbeat_type_gimbal = now;
				break;

			case MAV_TYPE_ADSB:
				_heartbeat_type_adsb = now;
				break;

			case MAV_TYPE_CAMERA:
				_heartbeat_type_camera = now;
				camera_status.timestamp = now;
				camera_status.active_comp_id = msg->compid;
				camera_status.active_sys_id = msg->sysid;
				_camera_status_pub.publish(camera_status);
				break;

			case MAV_TYPE_PARACHUTE:
				_heartbeat_type_parachute = now;
				_mavlink.telemetry_status().parachute_system_healthy =
					(hb.system_status == MAV_STATE_STANDBY) || (hb.system_status == MAV_STATE_ACTIVE);
				break;

			case MAV_TYPE_ODID:
				_heartbeat_type_open_drone_id = now;
				_mavlink.telemetry_status().open_drone_id_system_healthy =
					(hb.system_status == MAV_STATE_STANDBY) || (hb.system_status == MAV_STATE_ACTIVE);
				break;

			default:
				PX4_DEBUG("unhandled HEARTBEAT MAV_TYPE: %" PRIu8 " from SYSID: %" PRIu8 ", COMPID: %" PRIu8, hb.type, msg->sysid,
					  msg->compid);
			}


			switch (msg->compid) {
			case MAV_COMP_ID_TELEMETRY_RADIO:
				_heartbeat_component_telemetry_radio = now;
				break;

			case MAV_COMP_ID_LOG:
				_heartbeat_component_log = now;
				break;

			case MAV_COMP_ID_OSD:
				_heartbeat_component_osd = now;
				break;

			case MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY:
				_heartbeat_component_visual_inertial_odometry = now;
				break;

			case MAV_COMP_ID_PAIRING_MANAGER:
				_heartbeat_component_pairing_manager = now;
				break;

			case MAV_COMP_ID_UDP_BRIDGE:
				_heartbeat_component_udp_bridge = now;
				break;

			case MAV_COMP_ID_UART_BRIDGE:
				_heartbeat_component_uart_bridge = now;
				break;

			default:
				PX4_DEBUG("unhandled HEARTBEAT MAV_TYPE: %" PRIu8 " from SYSID: %" PRIu8 ", COMPID: %" PRIu8, hb.type, msg->sysid,
					  msg->compid);
			}

			CheckHeartbeats(now, true);
		}
	}
}

int
MavlinkReceiver::set_message_interval(int msgId, float interval, float param3, float param4, float param7)
{
	if (msgId == MAVLINK_MSG_ID_HEARTBEAT) {
		return PX4_ERROR;
	}

	if (PX4_ISFINITE(param3) && (int)(param3 + 0.5f) != 0) {
		PX4_ERR("SET_MESSAGE_INTERVAL requested param3 not supported.");
		return PX4_ERROR;
	}

	if (PX4_ISFINITE(param4) && (int)(param4 + 0.5f) != 0) {
		PX4_ERR("SET_MESSAGE_INTERVAL requested param4 not supported.");
		return PX4_ERROR;
	}

	if (PX4_ISFINITE(param7) && (int)(param7 + 0.5f) != 0) {
		PX4_ERR("SET_MESSAGE_INTERVAL response target not supported.");
		return PX4_ERROR;
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

	if (msgId != 0) {
		const char *stream_name = get_stream_name(msgId);

		if (stream_name != nullptr) {
			_mavlink.configure_stream_threadsafe(stream_name, rate);
			found_id = true;
		}
	}

	return (found_id ? PX4_OK : PX4_ERROR);
}

void
MavlinkReceiver::get_message_interval(int msgId)
{
	unsigned interval = 0;

	for (const auto &stream : _mavlink.get_streams()) {
		if (stream->get_id() == msgId) {
			interval = stream->get_interval();
			break;
		}
	}

	// send back this value...
	mavlink_msg_message_interval_send(_mavlink.get_channel(), msgId, interval);
}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t hil_sensor;
	mavlink_msg_hil_sensor_decode(msg, &hil_sensor);

	const uint64_t timestamp = hrt_absolute_time();

	// temperature only updated with baro
	float temperature = NAN;

	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		temperature = hil_sensor.temperature;
	}

	// gyro
	if ((hil_sensor.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);
		}

		if (_px4_gyro != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_gyro->set_temperature(temperature);
			}

			_px4_gyro->update(timestamp, hil_sensor.xgyro, hil_sensor.ygyro, hil_sensor.zgyro);
		}
	}

	// accelerometer
	if ((hil_sensor.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);
		}

		if (_px4_accel != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_accel->set_temperature(temperature);
			}

			_px4_accel->update(timestamp, hil_sensor.xacc, hil_sensor.yacc, hil_sensor.zacc);
		}
	}

	// magnetometer
	if ((hil_sensor.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
		if (_px4_mag == nullptr) {
			// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
			_px4_mag = new PX4Magnetometer(197388);
		}

		if (_px4_mag != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_mag->set_temperature(temperature);
			}

			_px4_mag->update(timestamp, hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag);
		}
	}

	// baro
	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		// publish
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = timestamp;
		sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
		sensor_baro.pressure = hil_sensor.abs_pressure * 100.0f; // hPa to Pa
		sensor_baro.temperature = hil_sensor.temperature;
		sensor_baro.error_count = 0;
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pub.publish(sensor_baro);
	}

	// differential pressure
	if ((hil_sensor.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS) {
		differential_pressure_s report{};
		report.timestamp_sample = timestamp;
		report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
		report.temperature = hil_sensor.temperature;
		report.differential_pressure_pa = hil_sensor.diff_pressure * 100.0f; // hPa to Pa
		report.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(report);
	}

	// battery status
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 16.0f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;
		hil_battery_status.connected = true;
		hil_battery_status.remaining = 0.70;
		hil_battery_status.time_remaining_s = NAN;

		_battery_pub.publish(hil_battery_status);
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	sensor_gps_s gps{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink.get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

	gps.device_id = device_id.devid;

	gps.latitude_deg = hil_gps.lat * 1e-7;
	gps.longitude_deg = hil_gps.lon * 1e-7;
	gps.altitude_msl_m = hil_gps.alt * 1e-3;
	gps.altitude_ellipsoid_m = hil_gps.alt * 1e-3;

	gps.s_variance_m_s = 0.25f;
	gps.c_variance_rad = 0.5f;
	gps.fix_type = hil_gps.fix_type;

	gps.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
	gps.epv = (float)hil_gps.epv * 1e-2f; // cm -> m

	gps.hdop = 0; // TODO
	gps.vdop = 0; // TODO

	gps.noise_per_ms = 0;
	gps.automatic_gain_control = 0;
	gps.jamming_indicator = 0;
	gps.jamming_state = 0;
	gps.spoofing_state = 0;

	gps.vel_m_s = (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s
	gps.vel_n_m_s = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
	gps.vel_e_m_s = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
	gps.vel_d_m_s = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
	gps.cog_rad = ((hil_gps.cog == 65535) ? (float)NAN : matrix::wrap_2pi(math::radians(
				hil_gps.cog * 1e-2f))); // cdeg -> rad
	gps.vel_ned_valid = true;

	gps.timestamp_time_relative = 0;
	gps.time_utc_usec = hil_gps.time_usec;

	gps.satellites_used = hil_gps.satellites_visible;

	gps.heading = NAN;
	gps.heading_offset = NAN;

	gps.timestamp = hrt_absolute_time();

	_sensor_gps_pub.publish(gps);
}

void
MavlinkReceiver::handle_message_follow_target(mavlink_message_t *msg)
{
	mavlink_follow_target_t follow_target_msg;
	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_s follow_target_topic{};

	follow_target_topic.timestamp = hrt_absolute_time();
	follow_target_topic.lat = follow_target_msg.lat * 1e-7;
	follow_target_topic.lon = follow_target_msg.lon * 1e-7;
	follow_target_topic.alt = follow_target_msg.alt;
	follow_target_topic.vx = follow_target_msg.vel[0];
	follow_target_topic.vy = follow_target_msg.vel[1];
	follow_target_topic.vz = follow_target_msg.vel[2];

	_follow_target_pub.publish(follow_target_topic);
}

void
MavlinkReceiver::handle_message_landing_target(mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target;
	mavlink_msg_landing_target_decode(msg, &landing_target);

	if (landing_target.position_valid && landing_target.frame == MAV_FRAME_LOCAL_NED) {
		landing_target_pose_s landing_target_pose{};

		landing_target_pose.timestamp = _mavlink_timesync.sync_stamp(landing_target.time_usec);
		landing_target_pose.abs_pos_valid = true;
		landing_target_pose.x_abs = landing_target.x;
		landing_target_pose.y_abs = landing_target.y;
		landing_target_pose.z_abs = landing_target.z;

		_landing_target_pose_pub.publish(landing_target_pose);

	} else if (landing_target.position_valid) {
		// We only support MAV_FRAME_LOCAL_NED. In this case, the frame was unsupported.
		mavlink_log_critical(&_mavlink_log_pub, "Landing target: coordinate frame %" PRIu8 " unsupported\t",
				     landing_target.frame);
		events::send<uint8_t>(events::ID("mavlink_rcv_lnd_target_unsup_coord"), events::Log::Error,
				      "Landing target: unsupported coordinate frame {1}", landing_target.frame);

	} else {
		irlock_report_s irlock_report{};

		irlock_report.timestamp = hrt_absolute_time();
		irlock_report.signature = landing_target.target_num;
		irlock_report.pos_x = landing_target.angle_x;
		irlock_report.pos_y = landing_target.angle_y;
		irlock_report.size_x = landing_target.size_x;
		irlock_report.size_y = landing_target.size_y;

		_irlock_report_pub.publish(irlock_report);
	}
}

void
MavlinkReceiver::handle_message_cellular_status(mavlink_message_t *msg)
{
	mavlink_cellular_status_t status;
	mavlink_msg_cellular_status_decode(msg, &status);

	cellular_status_s cellular_status{};

	cellular_status.timestamp = hrt_absolute_time();
	cellular_status.status = status.status;
	cellular_status.failure_reason = status.failure_reason;
	cellular_status.type = status.type;
	cellular_status.quality = status.quality;
	cellular_status.mcc = status.mcc;
	cellular_status.mnc = status.mnc;
	cellular_status.lac = status.lac;

	_cellular_status_pub.publish(cellular_status);
}

void
MavlinkReceiver::handle_message_adsb_vehicle(mavlink_message_t *msg)
{
	mavlink_adsb_vehicle_t adsb;
	mavlink_msg_adsb_vehicle_decode(msg, &adsb);

	transponder_report_s t{};

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

	_transponder_report_pub.publish(t);
}

void
MavlinkReceiver::handle_message_gps_rtcm_data(mavlink_message_t *msg)
{
	mavlink_gps_rtcm_data_t gps_rtcm_data_msg;
	mavlink_msg_gps_rtcm_data_decode(msg, &gps_rtcm_data_msg);

	gps_inject_data_s gps_inject_data_topic{};

	gps_inject_data_topic.timestamp = hrt_absolute_time();

	gps_inject_data_topic.len = math::min((int)sizeof(gps_rtcm_data_msg.data),
					      (int)sizeof(uint8_t) * gps_rtcm_data_msg.len);
	gps_inject_data_topic.flags = gps_rtcm_data_msg.flags;
	memcpy(gps_inject_data_topic.data, gps_rtcm_data_msg.data,
	       math::min((int)sizeof(gps_inject_data_topic.data), (int)sizeof(uint8_t) * gps_inject_data_topic.len));

	gps_inject_data_topic.timestamp = hrt_absolute_time();
	_gps_inject_data_pub.publish(gps_inject_data_topic);
}

void
MavlinkReceiver::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	const uint64_t timestamp_sample = hrt_absolute_time();

	/* airspeed */
	{
		airspeed_s airspeed{};
		airspeed.timestamp_sample = timestamp_sample;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;
		airspeed.timestamp = hrt_absolute_time();
		_airspeed_pub.publish(airspeed);
	}

	/* attitude */
	{
		vehicle_attitude_s hil_attitude{};
		hil_attitude.timestamp_sample = timestamp_sample;
		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);
		hil_attitude.timestamp = hrt_absolute_time();
		_attitude_pub.publish(hil_attitude);
	}

	/* global position */
	{
		vehicle_global_position_s hil_global_pos{};

		hil_global_pos.timestamp_sample = timestamp_sample;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.eph = 2.f;
		hil_global_pos.epv = 4.f;
		hil_global_pos.timestamp = hrt_absolute_time();
		_global_pos_pub.publish(hil_global_pos);
	}

	/* local position */
	{
		const double lat = hil_state.lat * 1e-7;
		const double lon = hil_state.lon * 1e-7;

		if (!_global_local_proj_ref.isInitialized() || !PX4_ISFINITE(_global_local_alt0)) {
			_global_local_proj_ref.initReference(lat, lon, hrt_absolute_time());
			_global_local_alt0 = hil_state.alt / 1000.f;
		}

		float x = 0.f;
		float y = 0.f;
		_global_local_proj_ref.project(lat, lon, x, y);

		vehicle_local_position_s hil_local_pos{};
		hil_local_pos.timestamp_sample = timestamp_sample;
		hil_local_pos.ref_timestamp = _global_local_proj_ref.getProjectionReferenceTimestamp();
		hil_local_pos.ref_lat = _global_local_proj_ref.getProjectionReferenceLat();
		hil_local_pos.ref_lon = _global_local_proj_ref.getProjectionReferenceLon();
		hil_local_pos.ref_alt = _global_local_alt0;
		hil_local_pos.xy_valid = true;
		hil_local_pos.z_valid = true;
		hil_local_pos.v_xy_valid = true;
		hil_local_pos.v_z_valid = true;
		hil_local_pos.x = x;
		hil_local_pos.y = y;
		hil_local_pos.z = _global_local_alt0 - hil_state.alt / 1000.f;
		hil_local_pos.vx = hil_state.vx / 100.f;
		hil_local_pos.vy = hil_state.vy / 100.f;
		hil_local_pos.vz = hil_state.vz / 100.f;

		matrix::Eulerf euler{matrix::Quatf(hil_state.attitude_quaternion)};
		hil_local_pos.heading = euler.psi();
		hil_local_pos.heading_good_for_control = PX4_ISFINITE(euler.psi());
		hil_local_pos.unaided_heading = NAN;
		hil_local_pos.xy_global = true;
		hil_local_pos.z_global = true;
		hil_local_pos.vxy_max = INFINITY;
		hil_local_pos.vz_max = INFINITY;
		hil_local_pos.hagl_min = INFINITY;
		hil_local_pos.hagl_max_z = INFINITY;
		hil_local_pos.hagl_max_xy = INFINITY;
		hil_local_pos.timestamp = hrt_absolute_time();
		_local_pos_pub.publish(hil_local_pos);
	}

	/* accelerometer */
	{
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);

			if (_px4_accel == nullptr) {
				PX4_ERR("PX4Accelerometer alloc failed");
			}
		}

		if (_px4_accel != nullptr) {
			// accel in mG
			_px4_accel->set_scale(CONSTANTS_ONE_G / 1000.0f);
			_px4_accel->update(timestamp_sample, hil_state.xacc, hil_state.yacc, hil_state.zacc);
		}
	}

	/* gyroscope */
	{
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);

			if (_px4_gyro == nullptr) {
				PX4_ERR("PX4Gyroscope alloc failed");
			}
		}

		if (_px4_gyro != nullptr) {
			_px4_gyro->update(timestamp_sample, hil_state.rollspeed, hil_state.pitchspeed, hil_state.yawspeed);
		}
	}

	/* battery status */
	{
		battery_status_s hil_battery_status{};
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;
		hil_battery_status.timestamp = hrt_absolute_time();
		hil_battery_status.time_remaining_s = NAN;
		_battery_pub.publish(hil_battery_status);
	}
}

#if !defined(CONSTRAINED_FLASH)
void
MavlinkReceiver::handle_message_named_value_float(mavlink_message_t *msg)
{
	mavlink_named_value_float_t debug_msg;
	mavlink_msg_named_value_float_decode(msg, &debug_msg);

	debug_key_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.key, debug_msg.name, sizeof(debug_topic.key));
	debug_topic.key[sizeof(debug_topic.key) - 1] = '\0'; // enforce null termination
	debug_topic.value = debug_msg.value;

	_debug_key_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_named_value_int(mavlink_message_t *msg)
{
	mavlink_named_value_int_t debug_msg;
	mavlink_msg_named_value_int_decode(msg, &debug_msg);

	debug_key_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.key, debug_msg.name, sizeof(debug_topic.key));
	debug_topic.key[sizeof(debug_topic.key) - 1] = '\0'; // enforce null termination
	debug_topic.value = debug_msg.value;

	_debug_key_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug(mavlink_message_t *msg)
{
	mavlink_debug_t debug_msg;
	mavlink_msg_debug_decode(msg, &debug_msg);

	debug_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.ind = debug_msg.ind;
	debug_topic.value = debug_msg.value;

	_debug_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug_vect(mavlink_message_t *msg)
{
	mavlink_debug_vect_t debug_msg;
	mavlink_msg_debug_vect_decode(msg, &debug_msg);

	debug_vect_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination
	debug_topic.x = debug_msg.x;
	debug_topic.y = debug_msg.y;
	debug_topic.z = debug_msg.z;

	_debug_vect_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug_float_array(mavlink_message_t *msg)
{
	mavlink_debug_float_array_t debug_msg;
	mavlink_msg_debug_float_array_decode(msg, &debug_msg);

	debug_array_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.id = debug_msg.array_id;
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

	for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		debug_topic.data[i] = debug_msg.data[i];
	}

	_debug_array_pub.publish(debug_topic);
}
#endif // !CONSTRAINED_FLASH

void
MavlinkReceiver::handle_message_onboard_computer_status(mavlink_message_t *msg)
{
	mavlink_onboard_computer_status_t status_msg;
	mavlink_msg_onboard_computer_status_decode(msg, &status_msg);

	onboard_computer_status_s onboard_computer_status_topic{};

	onboard_computer_status_topic.timestamp = hrt_absolute_time();
	onboard_computer_status_topic.uptime = status_msg.uptime;

	onboard_computer_status_topic.type = status_msg.type;

	memcpy(onboard_computer_status_topic.cpu_cores, status_msg.cpu_cores, sizeof(status_msg.cpu_cores));
	memcpy(onboard_computer_status_topic.cpu_combined, status_msg.cpu_combined, sizeof(status_msg.cpu_combined));
	memcpy(onboard_computer_status_topic.gpu_cores, status_msg.gpu_cores, sizeof(status_msg.gpu_cores));
	memcpy(onboard_computer_status_topic.gpu_combined, status_msg.gpu_combined, sizeof(status_msg.gpu_combined));
	onboard_computer_status_topic.temperature_board = status_msg.temperature_board;
	memcpy(onboard_computer_status_topic.temperature_core, status_msg.temperature_core,
	       sizeof(status_msg.temperature_core));
	memcpy(onboard_computer_status_topic.fan_speed, status_msg.fan_speed, sizeof(status_msg.fan_speed));
	onboard_computer_status_topic.ram_usage = status_msg.ram_usage;
	onboard_computer_status_topic.ram_total = status_msg.ram_total;
	memcpy(onboard_computer_status_topic.storage_type, status_msg.storage_type, sizeof(status_msg.storage_type));
	memcpy(onboard_computer_status_topic.storage_usage, status_msg.storage_usage, sizeof(status_msg.storage_usage));
	memcpy(onboard_computer_status_topic.storage_total, status_msg.storage_total, sizeof(status_msg.storage_total));
	memcpy(onboard_computer_status_topic.link_type, status_msg.link_type, sizeof(status_msg.link_type));
	memcpy(onboard_computer_status_topic.link_tx_rate, status_msg.link_tx_rate, sizeof(status_msg.link_tx_rate));
	memcpy(onboard_computer_status_topic.link_rx_rate, status_msg.link_rx_rate, sizeof(status_msg.link_rx_rate));
	memcpy(onboard_computer_status_topic.link_tx_max, status_msg.link_tx_max, sizeof(status_msg.link_tx_max));
	memcpy(onboard_computer_status_topic.link_rx_max, status_msg.link_rx_max, sizeof(status_msg.link_rx_max));

	_onboard_computer_status_pub.publish(onboard_computer_status_topic);
}

void MavlinkReceiver::handle_message_generator_status(mavlink_message_t *msg)
{
	mavlink_generator_status_t status_msg;
	mavlink_msg_generator_status_decode(msg, &status_msg);

	generator_status_s generator_status{};
	generator_status.timestamp = hrt_absolute_time();
	generator_status.status = status_msg.status;
	generator_status.battery_current = status_msg.battery_current;
	generator_status.load_current = status_msg.load_current;
	generator_status.power_generated = status_msg.power_generated;
	generator_status.bus_voltage = status_msg.bus_voltage;
	generator_status.bat_current_setpoint = status_msg.bat_current_setpoint;
	generator_status.runtime = status_msg.runtime;
	generator_status.time_until_maintenance = status_msg.time_until_maintenance;
	generator_status.generator_speed = status_msg.generator_speed;
	generator_status.rectifier_temperature = status_msg.rectifier_temperature;
	generator_status.generator_temperature = status_msg.generator_temperature;

	_generator_status_pub.publish(generator_status);
}

void MavlinkReceiver::handle_message_statustext(mavlink_message_t *msg)
{
	if (msg->sysid == mavlink_system.sysid) {
		// log message from the same system

		mavlink_statustext_t statustext;
		mavlink_msg_statustext_decode(msg, &statustext);

		if (_mavlink_statustext_handler.should_publish_previous(statustext)) {
			_log_message_pub.publish(_mavlink_statustext_handler.log_message());
		}

		if (_mavlink_statustext_handler.should_publish_current(statustext, hrt_absolute_time())) {
			_log_message_pub.publish(_mavlink_statustext_handler.log_message());
		}
	}
}

void MavlinkReceiver::CheckHeartbeats(const hrt_abstime &t, bool force)
{
	// check HEARTBEATs for timeout
	static constexpr uint64_t TIMEOUT = telemetry_status_s::HEARTBEAT_TIMEOUT_US;

	if (t <= TIMEOUT) {
		return;
	}

	if ((t >= _last_heartbeat_check + (TIMEOUT / 2)) || force) {
		telemetry_status_s &tstatus = _mavlink.telemetry_status();

		tstatus.heartbeat_type_antenna_tracker         = (t <= TIMEOUT + _heartbeat_type_antenna_tracker);
		tstatus.heartbeat_type_gcs                     = (t <= TIMEOUT + _heartbeat_type_gcs);
		tstatus.heartbeat_type_onboard_controller      = (t <= TIMEOUT + _heartbeat_type_onboard_controller);
		tstatus.heartbeat_type_gimbal                  = (t <= TIMEOUT + _heartbeat_type_gimbal);
		tstatus.heartbeat_type_adsb                    = (t <= TIMEOUT + _heartbeat_type_adsb);
		tstatus.heartbeat_type_camera                  = (t <= TIMEOUT + _heartbeat_type_camera);
		tstatus.heartbeat_type_parachute               = (t <= TIMEOUT + _heartbeat_type_parachute);
		tstatus.heartbeat_type_open_drone_id           = (t <= TIMEOUT + _heartbeat_type_open_drone_id);

		tstatus.heartbeat_component_telemetry_radio    = (t <= TIMEOUT + _heartbeat_component_telemetry_radio);
		tstatus.heartbeat_component_log                = (t <= TIMEOUT + _heartbeat_component_log);
		tstatus.heartbeat_component_osd                = (t <= TIMEOUT + _heartbeat_component_osd);
		tstatus.heartbeat_component_vio                = (t <= TIMEOUT + _heartbeat_component_visual_inertial_odometry);
		tstatus.heartbeat_component_pairing_manager    = (t <= TIMEOUT + _heartbeat_component_pairing_manager);
		tstatus.heartbeat_component_udp_bridge         = (t <= TIMEOUT + _heartbeat_component_udp_bridge);
		tstatus.heartbeat_component_uart_bridge        = (t <= TIMEOUT + _heartbeat_component_uart_bridge);

		_mavlink.telemetry_status_updated();
		_last_heartbeat_check = t;
	}
}

void MavlinkReceiver::handle_message_request_event(mavlink_message_t *msg)
{
	_mavlink.get_events_protocol().handle_request_event(*msg);
}

void
MavlinkReceiver::handle_message_gimbal_manager_set_manual_control(mavlink_message_t *msg)
{
	mavlink_gimbal_manager_set_manual_control_t set_manual_control_msg;
	mavlink_msg_gimbal_manager_set_manual_control_decode(msg, &set_manual_control_msg);

	gimbal_manager_set_manual_control_s set_manual_control{};
	set_manual_control.timestamp = hrt_absolute_time();
	set_manual_control.origin_sysid = msg->sysid;
	set_manual_control.origin_compid = msg->compid;
	set_manual_control.target_system = set_manual_control_msg.target_system;
	set_manual_control.target_component = set_manual_control_msg.target_component;
	set_manual_control.flags = set_manual_control_msg.flags;
	set_manual_control.gimbal_device_id = set_manual_control_msg.gimbal_device_id;

	set_manual_control.pitch = set_manual_control_msg.pitch;
	set_manual_control.yaw = set_manual_control_msg.yaw;
	set_manual_control.pitch_rate = set_manual_control_msg.pitch_rate;
	set_manual_control.yaw_rate = set_manual_control_msg.yaw_rate;

	_gimbal_manager_set_manual_control_pub.publish(set_manual_control);
}

void
MavlinkReceiver::handle_message_gimbal_manager_set_attitude(mavlink_message_t *msg)
{
	mavlink_gimbal_manager_set_attitude_t set_attitude_msg;
	mavlink_msg_gimbal_manager_set_attitude_decode(msg, &set_attitude_msg);

	gimbal_manager_set_attitude_s gimbal_attitude{};
	gimbal_attitude.timestamp = hrt_absolute_time();
	gimbal_attitude.origin_sysid = msg->sysid;
	gimbal_attitude.origin_compid = msg->compid;
	gimbal_attitude.target_system = set_attitude_msg.target_system;
	gimbal_attitude.target_component = set_attitude_msg.target_component;
	gimbal_attitude.flags = set_attitude_msg.flags;
	gimbal_attitude.gimbal_device_id = set_attitude_msg.gimbal_device_id;

	matrix::Quatf q(set_attitude_msg.q);
	q.copyTo(gimbal_attitude.q);

	gimbal_attitude.angular_velocity_x = set_attitude_msg.angular_velocity_x;
	gimbal_attitude.angular_velocity_y = set_attitude_msg.angular_velocity_y;
	gimbal_attitude.angular_velocity_z = set_attitude_msg.angular_velocity_z;

	_gimbal_manager_set_attitude_pub.publish(gimbal_attitude);
}

void
MavlinkReceiver::handle_message_gimbal_device_information(mavlink_message_t *msg)
{

	mavlink_gimbal_device_information_t gimbal_device_info_msg;
	mavlink_msg_gimbal_device_information_decode(msg, &gimbal_device_info_msg);

	gimbal_device_information_s gimbal_information{};
	gimbal_information.timestamp = hrt_absolute_time();

	static_assert(sizeof(gimbal_information.vendor_name) == sizeof(gimbal_device_info_msg.vendor_name),
		      "vendor_name length doesn't match");
	static_assert(sizeof(gimbal_information.model_name) == sizeof(gimbal_device_info_msg.model_name),
		      "model_name length doesn't match");
	static_assert(sizeof(gimbal_information.custom_name) == sizeof(gimbal_device_info_msg.custom_name),
		      "custom_name length doesn't match");
	memcpy(gimbal_information.vendor_name, gimbal_device_info_msg.vendor_name, sizeof(gimbal_information.vendor_name));
	memcpy(gimbal_information.model_name, gimbal_device_info_msg.model_name, sizeof(gimbal_information.model_name));
	memcpy(gimbal_information.custom_name, gimbal_device_info_msg.custom_name, sizeof(gimbal_information.custom_name));
	gimbal_device_info_msg.vendor_name[sizeof(gimbal_device_info_msg.vendor_name) - 1] = '\0';
	gimbal_device_info_msg.model_name[sizeof(gimbal_device_info_msg.model_name) - 1] = '\0';
	gimbal_device_info_msg.custom_name[sizeof(gimbal_device_info_msg.custom_name) - 1] = '\0';

	gimbal_information.firmware_version = gimbal_device_info_msg.firmware_version;
	gimbal_information.hardware_version = gimbal_device_info_msg.hardware_version;
	gimbal_information.cap_flags = gimbal_device_info_msg.cap_flags;
	gimbal_information.custom_cap_flags = gimbal_device_info_msg.custom_cap_flags;
	gimbal_information.uid = gimbal_device_info_msg.uid;

	gimbal_information.pitch_max = gimbal_device_info_msg.pitch_max;
	gimbal_information.pitch_min = gimbal_device_info_msg.pitch_min;

	gimbal_information.yaw_max = gimbal_device_info_msg.yaw_max;
	gimbal_information.yaw_min = gimbal_device_info_msg.yaw_min;

	gimbal_information.gimbal_device_id = msg->compid;

	_gimbal_device_information_pub.publish(gimbal_information);
}

void
MavlinkReceiver::handle_message_gimbal_device_attitude_status(mavlink_message_t *msg)
{
	mavlink_gimbal_device_attitude_status_t gimbal_device_attitude_status_msg;
	mavlink_msg_gimbal_device_attitude_status_decode(msg, &gimbal_device_attitude_status_msg);

	gimbal_device_attitude_status_s gimbal_attitude_status{};
	gimbal_attitude_status.timestamp = hrt_absolute_time();
	gimbal_attitude_status.target_system = gimbal_device_attitude_status_msg.target_system;
	gimbal_attitude_status.target_component = gimbal_device_attitude_status_msg.target_component;
	gimbal_attitude_status.device_flags = gimbal_device_attitude_status_msg.flags;

	for (unsigned i = 0; i < 4; ++i) {
		gimbal_attitude_status.q[i] = gimbal_device_attitude_status_msg.q[i];
	}

	gimbal_attitude_status.angular_velocity_x = gimbal_device_attitude_status_msg.angular_velocity_x;
	gimbal_attitude_status.angular_velocity_y = gimbal_device_attitude_status_msg.angular_velocity_y;
	gimbal_attitude_status.angular_velocity_z = gimbal_device_attitude_status_msg.angular_velocity_z;
	gimbal_attitude_status.failure_flags = gimbal_device_attitude_status_msg.failure_flags;

	gimbal_attitude_status.received_from_mavlink = true;
	gimbal_attitude_status.gimbal_device_id = gimbal_device_attitude_status_msg.gimbal_device_id;

	_gimbal_device_attitude_status_pub.publish(gimbal_attitude_status);
}

void MavlinkReceiver::handle_message_open_drone_id_operator_id(
	mavlink_message_t *msg)
{
	mavlink_open_drone_id_operator_id_t odid_module;
	mavlink_msg_open_drone_id_operator_id_decode(msg, &odid_module);

	open_drone_id_operator_id_s odid_operator_id{};
	memset(&odid_operator_id, 0, sizeof(odid_operator_id));

	odid_operator_id.timestamp = hrt_absolute_time();
	memcpy(odid_operator_id.id_or_mac, odid_module.id_or_mac, sizeof(odid_operator_id.id_or_mac));
	odid_operator_id.operator_id_type = odid_module.operator_id_type;
	memcpy(odid_operator_id.operator_id, odid_module.operator_id, sizeof(odid_operator_id.operator_id));

	_open_drone_id_operator_id_pub.publish(odid_operator_id);
}

void MavlinkReceiver::handle_message_open_drone_id_self_id(mavlink_message_t *msg)
{
	mavlink_open_drone_id_self_id_t odid_module;
	mavlink_msg_open_drone_id_self_id_decode(msg, &odid_module);

	open_drone_id_self_id_s odid_self_id{};
	memset(&odid_self_id, 0, sizeof(odid_self_id));

	odid_self_id.timestamp = hrt_absolute_time();
	memcpy(odid_self_id.id_or_mac, odid_module.id_or_mac, sizeof(odid_self_id.id_or_mac));
	odid_self_id.description_type = odid_module.description_type;
	memcpy(odid_self_id.description, odid_module.description, sizeof(odid_self_id.description));

	_open_drone_id_self_id_pub.publish(odid_self_id);
}

void MavlinkReceiver::handle_message_open_drone_id_system(
	mavlink_message_t *msg)
{
	mavlink_open_drone_id_system_t odid_module;
	mavlink_msg_open_drone_id_system_decode(msg, &odid_module);

	open_drone_id_system_s odid_system{};
	memset(&odid_system, 0, sizeof(odid_system));

	odid_system.timestamp = hrt_absolute_time();
	memcpy(odid_system.id_or_mac, odid_module.id_or_mac, sizeof(odid_system.id_or_mac));
	odid_system.operator_location_type = odid_module.operator_location_type;
	odid_system.classification_type = odid_module.classification_type;
	odid_system.operator_latitude = odid_module.operator_latitude;
	odid_system.operator_longitude = odid_module.operator_longitude;
	odid_system.area_count = odid_module.area_count;
	odid_system.area_radius = odid_module.area_radius;
	odid_system.area_ceiling = odid_module.area_ceiling;
	odid_system.area_floor = odid_module.area_floor;
	odid_system.category_eu = odid_module.category_eu;
	odid_system.class_eu = odid_module.class_eu;
	odid_system.operator_altitude_geo = odid_module.operator_altitude_geo;

	_open_drone_id_system_pub.publish(odid_system);
}
void
MavlinkReceiver::run()
{
	/* set thread name */
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "mavlink_rcv_if%d", _mavlink.get_instance_id());
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

	if (_mavlink.get_protocol() == Protocol::SERIAL) {
		fds[0].fd = _mavlink.get_uart_fd();
		fds[0].events = POLLIN;
	}

#if defined(MAVLINK_UDP)
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink.get_protocol() == Protocol::UDP) {
		fds[0].fd = _mavlink.get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif // MAVLINK_UDP

	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while (!_mavlink.should_exit()) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}

		int ret = poll(&fds[0], 1, timeout);

		if (ret > 0) {
			if (_mavlink.get_protocol() == Protocol::SERIAL) {
				/* non-blocking read. read may return negative values */
				nread = ::read(fds[0].fd, buf, sizeof(buf));

				if (nread == -1 && errno == ENOTCONN) { // Not connected (can happen for USB)
					usleep(100000);
				}
			}

#if defined(MAVLINK_UDP)

			else if (_mavlink.get_protocol() == Protocol::UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink.get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}

				struct sockaddr_in &srcaddr_last = _mavlink.get_client_source_address();

				int localhost = (127 << 24) + 1;

				if (!_mavlink.get_client_source_initialized()) {

					// set the address either if localhost or if 3 seconds have passed
					// this ensures that a GCS running on localhost can get a hold of
					// the system within the first N seconds
					hrt_abstime stime = _mavlink.get_start_time();

					if ((stime != 0 && (hrt_elapsed_time(&stime) > 3_s))
					    || (srcaddr_last.sin_addr.s_addr == htonl(localhost))) {

						srcaddr_last.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
						srcaddr_last.sin_port = srcaddr.sin_port;

						_mavlink.set_client_source_initialized();

						PX4_INFO("partner IP: %s", inet_ntoa(srcaddr.sin_addr));
					}
				}
			}

			// only start accepting messages on UDP once we're sure who we talk to
			if (_mavlink.get_protocol() != Protocol::UDP || _mavlink.get_client_source_initialized()) {
#endif // MAVLINK_UDP

				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink.get_channel(), buf[i], &msg, &_status)) {

						/* check if we received version 2 and request a switch. */
						if (!(_mavlink.get_status()->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
							/* this will only switch to proto version 2 if allowed in settings */
							_mavlink.set_proto_version(2);
						}

						switch (_mavlink.get_mode()) {
						case Mavlink::MAVLINK_MODE::MAVLINK_MODE_GIMBAL:
							handle_messages_in_gimbal_mode(msg);
							break;

						default:
							handle_message(&msg);
							break;
						}

						_mavlink.set_has_received_messages(true); // Received first message, unlock wait to transmit '-w' command-line flag
						update_rx_stats(msg);

						if (_message_statistics_enabled) {
							update_message_statistics(msg);
						}
					}
				}

				/* count received bytes (nread will be -1 on read error) */
				if (nread > 0) {
					_mavlink.count_rxbytes(nread);

					telemetry_status_s &tstatus = _mavlink.telemetry_status();
					tstatus.rx_message_count = _total_received_counter;
					tstatus.rx_message_lost_count = _total_lost_counter;
					tstatus.rx_message_lost_rate = static_cast<float>(_total_lost_counter) / static_cast<float>(_total_received_counter);

					if (_mavlink_status_last_buffer_overrun != _status.buffer_overrun) {
						tstatus.rx_buffer_overruns++;
						_mavlink_status_last_buffer_overrun = _status.buffer_overrun;
					}

					if (_mavlink_status_last_parse_error != _status.parse_error) {
						tstatus.rx_parse_errors++;
						_mavlink_status_last_parse_error = _status.parse_error;
					}

					if (_mavlink_status_last_packet_rx_drop_count != _status.packet_rx_drop_count) {
						tstatus.rx_packet_drop_count++;
						_mavlink_status_last_packet_rx_drop_count = _status.packet_rx_drop_count;
					}
				}

#if defined(MAVLINK_UDP)
			}

#endif // MAVLINK_UDP

		} else if (ret == -1) {
			usleep(10000);
		}

		const hrt_abstime t = hrt_absolute_time();

		CheckHeartbeats(t);

		if (t - last_send_update > timeout * 1000) {
			_mission_manager.check_active_mission();
			_mission_manager.send();

			if (_mavlink.get_mode() != Mavlink::MAVLINK_MODE::MAVLINK_MODE_IRIDIUM) {
				_parameters_manager.send();
			}

			if (_mavlink.ftp_enabled()) {
				_mavlink_ftp.send();
			}

			_mavlink_log_handler.send();
			last_send_update = t;
		}

		if (_tune_publisher != nullptr) {
			_tune_publisher->publish_next_tune(t);
		}
	}
}

bool MavlinkReceiver::component_was_seen(int system_id, int component_id)
{
	// For system broadcast messages return true if at least one component was seen before
	if (system_id == 0) {
		return _component_states_count > 0;
	}

	for (unsigned i = 0; i < _component_states_count; ++i) {
		if (_component_states[i].system_id == system_id
		    && (component_id == 0 || _component_states[i].component_id == component_id)) {
			return true;
		}
	}

	return false;
}

void MavlinkReceiver::update_rx_stats(const mavlink_message_t &message)
{
	const bool component_states_has_still_space = [this, &message]() {
		for (unsigned i = 0; i < MAX_REMOTE_COMPONENTS; ++i) {
			if (_component_states[i].system_id == message.sysid && _component_states[i].component_id == message.compid) {

				int lost_messages = 0;
				const uint8_t expected_seq = _component_states[i].last_sequence + 1;

				// Account for overflow during packet loss
				if (message.seq < expected_seq) {
					lost_messages = (message.seq + 255) - expected_seq;

				} else {
					lost_messages = message.seq - expected_seq;
				}

				_component_states[i].missed_messages += lost_messages;

				++_component_states[i].received_messages;
				_component_states[i].last_sequence = message.seq;

				// Also update overall stats
				++_total_received_counter;
				_total_lost_counter += lost_messages;

				return true;

			} else if (_component_states[i].system_id == 0 && _component_states[i].component_id == 0) {
				_component_states[i].system_id = message.sysid;
				_component_states[i].component_id = message.compid;

				++_component_states[i].received_messages;
				_component_states[i].last_sequence = message.seq;

				_component_states_count = i + 1;

				// Also update overall stats
				++_total_received_counter;

				return true;
			}
		}

		return false;
	}();

	if (!component_states_has_still_space && !_warned_component_states_full_once) {
		PX4_WARN("Max remote components of %u used up", MAX_REMOTE_COMPONENTS);
		_warned_component_states_full_once = true;
	}
}

void MavlinkReceiver::update_message_statistics(const mavlink_message_t &message)
{
#if !defined(CONSTRAINED_FLASH)

	if (_received_msg_stats == nullptr) {
		_received_msg_stats = new ReceivedMessageStats[MAX_MSG_STAT_SLOTS];
	}

	if (_received_msg_stats) {
		const hrt_abstime now_ms = hrt_absolute_time() / 1000;

		int msg_stats_slot = -1;
		bool reset_stats = false;

		// find matching msg id
		for (int stat_slot = 0; stat_slot < MAX_MSG_STAT_SLOTS; stat_slot++) {
			if ((_received_msg_stats[stat_slot].msg_id == message.msgid)
			    && (_received_msg_stats[stat_slot].system_id == message.sysid)
			    && (_received_msg_stats[stat_slot].component_id == message.compid)) {

				msg_stats_slot = stat_slot;
				break;
			}
		}

		// otherwise find oldest or empty slot
		if (msg_stats_slot < 0) {
			uint32_t oldest_slot_time_ms = 0;

			for (int stat_slot = 0; stat_slot < MAX_MSG_STAT_SLOTS; stat_slot++) {
				if (_received_msg_stats[stat_slot].last_time_received_ms <= oldest_slot_time_ms) {
					oldest_slot_time_ms = _received_msg_stats[stat_slot].last_time_received_ms;
					msg_stats_slot = stat_slot;
				}
			}

			reset_stats = true;
		}

		if (msg_stats_slot >= 0) {
			if (!reset_stats) {
				if ((_received_msg_stats[msg_stats_slot].last_time_received_ms != 0)
				    && (now_ms > _received_msg_stats[msg_stats_slot].last_time_received_ms)) {

					float rate = 1000.f / (now_ms - _received_msg_stats[msg_stats_slot].last_time_received_ms);

					if (PX4_ISFINITE(_received_msg_stats[msg_stats_slot].avg_rate_hz)) {
						_received_msg_stats[msg_stats_slot].avg_rate_hz = 0.9f * _received_msg_stats[msg_stats_slot].avg_rate_hz + 0.1f * rate;

					} else {
						_received_msg_stats[msg_stats_slot].avg_rate_hz = rate;
					}

				} else {
					_received_msg_stats[msg_stats_slot].avg_rate_hz = 0.f;
				}

			} else {
				_received_msg_stats[msg_stats_slot].avg_rate_hz = NAN;
			}

			_received_msg_stats[msg_stats_slot].last_time_received_ms = now_ms;
			_received_msg_stats[msg_stats_slot].msg_id = message.msgid;
			_received_msg_stats[msg_stats_slot].system_id = message.sysid;
			_received_msg_stats[msg_stats_slot].component_id = message.compid;
		}
	}

#endif // !CONSTRAINED_FLASH
}

void MavlinkReceiver::print_detailed_rx_stats() const
{
	// TODO: add mutex around shared data.
	if (_component_states_count > 0) {
		printf("\tReceived Messages:\n");

		for (const auto &comp_stat : _component_states) {
			if (comp_stat.received_messages > 0) {
				printf("\t  sysid:%3" PRIu8 ", compid:%3" PRIu8 ", Total: %" PRIu32 " (lost: %" PRIu32 ")\n",
				       comp_stat.system_id, comp_stat.component_id,
				       comp_stat.received_messages, comp_stat.missed_messages);

#if !defined(CONSTRAINED_FLASH)

				if (_message_statistics_enabled && _received_msg_stats) {
					for (int i = 0; i < MAX_MSG_STAT_SLOTS; i++) {
						const ReceivedMessageStats &msg_stat = _received_msg_stats[i];

						const uint32_t now_ms = hrt_absolute_time() / 1000;

						// valid messages received within the last 10 seconds
						if ((msg_stat.system_id == comp_stat.system_id)
						    && (msg_stat.component_id == comp_stat.component_id)
						    && (msg_stat.last_time_received_ms != 0)
						    && (now_ms - msg_stat.last_time_received_ms < 10'000)) {

							const float elapsed_s = (now_ms - msg_stat.last_time_received_ms) / 1000.f;

							printf("\t    msgid:%5" PRIu16 ", Rate:%5.1f Hz, last %.2fs ago\n",
							       msg_stat.msg_id, (double)msg_stat.avg_rate_hz, (double)elapsed_s);
						}
					}
				}

#endif // !CONSTRAINED_FLASH
			}
		}
	}
}

void MavlinkReceiver::start()
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr,
				  PX4_STACK_ADJUSTED(sizeof(MavlinkReceiver) + 2840 + MAVLINK_RECEIVER_NET_ADDED_STACK));

	pthread_create(&_thread, &receiveloop_attr, MavlinkReceiver::start_trampoline, (void *)this);

	pthread_attr_destroy(&receiveloop_attr);
}

void
MavlinkReceiver::updateParams()
{
	// update parameters from storage
	ModuleParams::updateParams();
}

void *MavlinkReceiver::start_trampoline(void *context)
{
	MavlinkReceiver *self = reinterpret_cast<MavlinkReceiver *>(context);
	self->run();
	return nullptr;
}

void MavlinkReceiver::stop()
{
	_should_exit.store(true);
	pthread_join(_thread, nullptr);
}
