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
 * @file mavlink_orb_listener.h
 * MAVLink 1.0 uORB listener definition
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#pragma once

#include <systemlib/perf_counter.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
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

	static void * start_helper(void *context);

private:

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	Mavlink*	_mavlink;

	void handle_message(mavlink_message_t *msg);
	void *receive_thread(void *arg);

	mavlink_status_t status;
	struct vehicle_vicon_position_s vicon_position;
	struct vehicle_command_s vcmd;
	struct offboard_control_setpoint_s offboard_control_sp;
	struct vehicle_global_position_s hil_global_pos;
	struct vehicle_local_position_s hil_local_pos;
	struct vehicle_attitude_s hil_attitude;
	struct vehicle_gps_position_s hil_gps;
	struct sensor_combined_s hil_sensors;
	struct battery_status_s	hil_battery_status;
	struct position_setpoint_triplet_s pos_sp_triplet;
	orb_advert_t pub_hil_global_pos;
	orb_advert_t pub_hil_local_pos;
	orb_advert_t pub_hil_attitude;
	orb_advert_t pub_hil_gps;
	orb_advert_t pub_hil_sensors;
	orb_advert_t pub_hil_gyro;
	orb_advert_t pub_hil_accel;
	orb_advert_t pub_hil_mag;
	orb_advert_t pub_hil_baro;
	orb_advert_t pub_hil_airspeed;
	orb_advert_t pub_hil_battery;
	int hil_counter;
	int hil_frames;
	uint64_t old_timestamp;
	orb_advert_t cmd_pub;
	orb_advert_t flow_pub;
	orb_advert_t offboard_control_sp_pub;
	orb_advert_t vicon_position_pub;
	orb_advert_t telemetry_status_pub;
	int32_t lat0;
	int32_t lon0;
	float alt0;

};
