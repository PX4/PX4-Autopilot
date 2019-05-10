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
 * @file mavlink_receiver.h
 * MAVLink receiver thread
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include <perf/perf_counter.h>
#include <uORB/uORB.h>

#include <uORB/topics/airspeed.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_value.h>
#include <uORB/topics/debug_vect.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/ping.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_trajectory_waypoint.h>

#include "mavlink_ftp.h"
#include "mavlink_log_handler.h"
#include "mavlink_mission.h"
#include "mavlink_parameters.h"
#include "mavlink_timesync.h"

class Mavlink;

class MavlinkReceiver
{
public:
	/**
	 * Constructor
	 */
	MavlinkReceiver(Mavlink *parent);

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~MavlinkReceiver();

	/**
	 * Display the mavlink status.
	 */
	void print_status();

	/**
	 * Start the receiver thread
	 */
	static void receive_start(pthread_t *thread, Mavlink *parent);

	static void *start_helper(void *context);

private:

	void acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result);

	/**
	 * Common method to handle both mavlink command types. T is one of mavlink_command_int_t or mavlink_command_long_t.
	 */
	template<class T>
	void handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
					 const vehicle_command_s &vehicle_command);

	void handle_message(mavlink_message_t *msg);
	void handle_message_adsb_vehicle(mavlink_message_t *msg);
	void handle_message_att_pos_mocap(mavlink_message_t *msg);
	void handle_message_battery_status(mavlink_message_t *msg);
	void handle_message_collision(mavlink_message_t *msg);
	void handle_message_command_ack(mavlink_message_t *msg);
	void handle_message_command_int(mavlink_message_t *msg);
	void handle_message_command_long(mavlink_message_t *msg);
	void handle_message_debug(mavlink_message_t *msg);
	void handle_message_debug_float_array(mavlink_message_t *msg);
	void handle_message_debug_vect(mavlink_message_t *msg);
	void handle_message_distance_sensor(mavlink_message_t *msg);
	void handle_message_follow_target(mavlink_message_t *msg);
	void handle_message_gps_global_origin(mavlink_message_t *msg);
	void handle_message_gps_rtcm_data(mavlink_message_t *msg);
	void handle_message_heartbeat(mavlink_message_t *msg);
	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(mavlink_message_t *msg);
	void handle_message_landing_target(mavlink_message_t *msg);
	void handle_message_logging_ack(mavlink_message_t *msg);
	void handle_message_manual_control(mavlink_message_t *msg);
	void handle_message_named_value_float(mavlink_message_t *msg);
	void handle_message_obstacle_distance(mavlink_message_t *msg);
	void handle_message_odometry(mavlink_message_t *msg);
	void handle_message_optical_flow_rad(mavlink_message_t *msg);
	void handle_message_ping(mavlink_message_t *msg);
	void handle_message_play_tune(mavlink_message_t *msg);
	void handle_message_radio_status(mavlink_message_t *msg);
	void handle_message_rc_channels_override(mavlink_message_t *msg);
	void handle_message_serial_control(mavlink_message_t *msg);
	void handle_message_set_actuator_control_target(mavlink_message_t *msg);
	void handle_message_set_attitude_target(mavlink_message_t *msg);
	void handle_message_set_mode(mavlink_message_t *msg);
	void handle_message_set_position_target_local_ned(mavlink_message_t *msg);
	void handle_message_trajectory_representation_waypoints(mavlink_message_t *msg);
	void handle_message_vision_position_estimate(mavlink_message_t *msg);

	void *receive_thread(void *arg);

	/**
	 * Set the interval at which the given message stream is published.
	 * The rate is the number of messages per second.
	 *
	 * @param msgId The ID of the message interval to be set.
	 * @param interval The interval in usec to send the message.
	 * @param data_rate The total link data rate in bytes per second.
	 *
	 * @return PX4_OK on success, PX4_ERROR on fail.
	 */
	int set_message_interval(int msgId, float interval, int data_rate = -1);
	void get_message_interval(int msgId);

	/**
	 * Decode a switch position from a bitfield.
	 */
	switch_pos_t decode_switch_pos(uint16_t buttons, unsigned sw);

	/**
	 * Decode a switch position from a bitfield and state.
	 */
	int decode_switch_pos_n(uint16_t buttons, unsigned sw);

	bool evaluate_target_ok(int command, int target_system, int target_component);

	void send_flight_information();

	void send_storage_information(int storage_id);

	Mavlink	*_mavlink;

	MavlinkFTP			_mavlink_ftp;
	MavlinkLogHandler		_mavlink_log_handler;
	MavlinkTimesync			_mavlink_timesync;
	MavlinkMissionManager		_mission_manager;
	MavlinkParametersManager	_parameters_manager;

	mavlink_status_t _status{}; ///< receiver status, used for mavlink_parse_char()

	map_projection_reference_s _hil_local_proj_ref {};
	offboard_control_mode_s _offboard_control_mode{};

	vehicle_attitude_s _att {};
	vehicle_local_position_s _hil_local_pos {};
	vehicle_land_detected_s _hil_land_detector {};
	vehicle_control_mode_s _control_mode {};

	orb_advert_t _accel_pub{nullptr};
	orb_advert_t _actuator_controls_pubs[4] {nullptr, nullptr, nullptr, nullptr};
	orb_advert_t _airspeed_pub{nullptr};
	orb_advert_t _att_sp_pub{nullptr};
	orb_advert_t _attitude_pub{nullptr};
	orb_advert_t _baro_pub{nullptr};
	orb_advert_t _battery_pub{nullptr};
	orb_advert_t _cmd_pub{nullptr};
	orb_advert_t _collision_report_pub{nullptr};
	orb_advert_t _command_ack_pub{nullptr};
	orb_advert_t _debug_array_pub{nullptr};
	orb_advert_t _debug_key_value_pub{nullptr};
	orb_advert_t _debug_value_pub{nullptr};
	orb_advert_t _debug_vect_pub{nullptr};
	orb_advert_t _distance_sensor_pub{nullptr};
	orb_advert_t _flow_distance_sensor_pub{nullptr};
	orb_advert_t _flow_pub{nullptr};
	orb_advert_t _follow_target_pub{nullptr};
	orb_advert_t _global_pos_pub{nullptr};
	orb_advert_t _gps_inject_data_pub{nullptr};
	orb_advert_t _gps_pub{nullptr};
	orb_advert_t _gyro_pub{nullptr};
	orb_advert_t _hil_distance_sensor_pub{nullptr};
	orb_advert_t _land_detector_pub{nullptr};
	orb_advert_t _landing_target_pose_pub{nullptr};
	orb_advert_t _local_pos_pub{nullptr};
	orb_advert_t _mag_pub{nullptr};
	orb_advert_t _manual_pub{nullptr};
	orb_advert_t _mocap_odometry_pub{nullptr};
	orb_advert_t _obstacle_distance_pub{nullptr};
	orb_advert_t _offboard_control_mode_pub{nullptr};
	orb_advert_t _ping_pub{nullptr};
	orb_advert_t _pos_sp_triplet_pub{nullptr};
	orb_advert_t _radio_status_pub{nullptr};
	orb_advert_t _rates_sp_pub{nullptr};
	orb_advert_t _rc_pub{nullptr};
	orb_advert_t _trajectory_waypoint_pub{nullptr};
	orb_advert_t _transponder_report_pub{nullptr};
	orb_advert_t _visual_odometry_pub{nullptr};

	static constexpr int _gps_inject_data_queue_size{6};

	int _actuator_armed_sub{orb_subscribe(ORB_ID(actuator_armed))};
	int _control_mode_sub{orb_subscribe(ORB_ID(vehicle_control_mode))};
	int _vehicle_attitude_sub{orb_subscribe(ORB_ID(vehicle_attitude))};

	int _orb_class_instance{-1};

	uint64_t _global_ref_timestamp{0};

	bool _hil_local_proj_inited{false};

	float _hil_local_alt0{0.0f};

	static constexpr unsigned MOM_SWITCH_COUNT{8};

	uint8_t _mom_switch_pos[MOM_SWITCH_COUNT] {};
	uint16_t _mom_switch_state{0};

	param_t _p_bat_emergen_thr{PARAM_INVALID};
	param_t _p_bat_crit_thr{PARAM_INVALID};
	param_t _p_bat_low_thr{PARAM_INVALID};
	param_t _p_flow_rot{PARAM_INVALID};
	param_t _p_flow_maxr{PARAM_INVALID};
	param_t _p_flow_minhgt{PARAM_INVALID};
	param_t _p_flow_maxhgt{PARAM_INVALID};

	// Disallow copy construction and move assignment.
	MavlinkReceiver(const MavlinkReceiver &) = delete;
	MavlinkReceiver operator=(const MavlinkReceiver &) = delete;
};
