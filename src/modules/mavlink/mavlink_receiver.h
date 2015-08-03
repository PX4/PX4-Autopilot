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
 * @file mavlink_orb_listener.h
 * MAVLink 1.0 uORB listener definition
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#pragma once

#include <systemlib/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/att_pos_mocap.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_force_setpoint.h>
#include <uORB/topics/time_offset.h>
#include <uORB/topics/distance_sensor.h>

#include "mavlink_ftp.h"

#define PX4_EPOCH_SECS 1234567890ULL

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
	* Start the mavlink task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display the mavlink status.
	 */
	void		print_status();

	static pthread_t receive_start(Mavlink *parent);

	static void *start_helper(void *context);

private:
	Mavlink	*_mavlink;

	void handle_message(mavlink_message_t *msg);
	void handle_message_command_long(mavlink_message_t *msg);
	void handle_message_command_int(mavlink_message_t *msg);
	void handle_message_optical_flow_rad(mavlink_message_t *msg);
	void handle_message_hil_optical_flow(mavlink_message_t *msg);
	void handle_message_set_mode(mavlink_message_t *msg);
	void handle_message_att_pos_mocap(mavlink_message_t *msg);
	void handle_message_vision_position_estimate(mavlink_message_t *msg);
	void handle_message_quad_swarm_roll_pitch_yaw_thrust(mavlink_message_t *msg);
	void handle_message_set_position_target_local_ned(mavlink_message_t *msg);
	void handle_message_set_actuator_control_target(mavlink_message_t *msg);
	void handle_message_set_attitude_target(mavlink_message_t *msg);
	void handle_message_radio_status(mavlink_message_t *msg);
	void handle_message_manual_control(mavlink_message_t *msg);
	void handle_message_rc_channels_override(mavlink_message_t *msg);
	void handle_message_heartbeat(mavlink_message_t *msg);
	void handle_message_ping(mavlink_message_t *msg);
	void handle_message_request_data_stream(mavlink_message_t *msg);
	void handle_message_system_time(mavlink_message_t *msg);
	void handle_message_timesync(mavlink_message_t *msg);
	void handle_message_hil_sensor(mavlink_message_t *msg);
	void handle_message_hil_gps(mavlink_message_t *msg);
	void handle_message_hil_state_quaternion(mavlink_message_t *msg);
	void handle_message_distance_sensor(mavlink_message_t *msg);

	void *receive_thread(void *arg);

	/**
	 * Convert remote timestamp to local hrt time (usec)
	 * Use timesync if available, monotonic boot time otherwise
	 */
	uint64_t sync_stamp(uint64_t usec);

	/**
	 * Exponential moving average filter to smooth time offset
	 */
	void smooth_time_offset(uint64_t offset_ns);

	/**
	 * Decode a switch position from a bitfield
	 */
	switch_pos_t decode_switch_pos(uint16_t buttons, unsigned sw);

	/**
	 * Decode a switch position from a bitfield and state
	 */
	int decode_switch_pos_n(uint16_t buttons, unsigned sw);

	mavlink_status_t status;
	struct vehicle_local_position_s hil_local_pos;
	struct vehicle_land_detected_s hil_land_detector;
	struct vehicle_control_mode_s _control_mode;
	orb_advert_t _global_pos_pub;
	orb_advert_t _local_pos_pub;
	orb_advert_t _attitude_pub;
	orb_advert_t _gps_pub;
	orb_advert_t _sensors_pub;
	orb_advert_t _gyro_pub;
	orb_advert_t _accel_pub;
	orb_advert_t _mag_pub;
	orb_advert_t _baro_pub;
	orb_advert_t _airspeed_pub;
	orb_advert_t _battery_pub;
	orb_advert_t _cmd_pub;
	orb_advert_t _flow_pub;
	orb_advert_t _distance_sensor_pub;
	orb_advert_t _offboard_control_mode_pub;
	orb_advert_t _actuator_controls_pub;
	orb_advert_t _global_vel_sp_pub;
	orb_advert_t _att_sp_pub;
	orb_advert_t _rates_sp_pub;
	orb_advert_t _force_sp_pub;
	orb_advert_t _pos_sp_triplet_pub;
	orb_advert_t _att_pos_mocap_pub;
	orb_advert_t _vision_position_pub;
	orb_advert_t _telemetry_status_pub;
	orb_advert_t _rc_pub;
	orb_advert_t _manual_pub;
	orb_advert_t _land_detector_pub;
	orb_advert_t _time_offset_pub;
	int _control_mode_sub;
	int _hil_frames;
	uint64_t _old_timestamp;
	bool _hil_local_proj_inited;
	float _hil_local_alt0;
	struct map_projection_reference_s _hil_local_proj_ref;
	struct offboard_control_mode_s _offboard_control_mode;
	struct vehicle_attitude_setpoint_s _att_sp;
	struct vehicle_rates_setpoint_s _rates_sp;
	double _time_offset_avg_alpha;
	uint64_t _time_offset;
	int	_orb_class_instance;

	static constexpr unsigned MOM_SWITCH_COUNT = 8;

	uint8_t _mom_switch_pos[MOM_SWITCH_COUNT];
	uint16_t _mom_switch_state;

	/* do not allow copying this class */
	MavlinkReceiver(const MavlinkReceiver &);
	MavlinkReceiver operator=(const MavlinkReceiver &);
};
