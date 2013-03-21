/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file orb_listener.c
 * Monitors ORB topics and sends update messages as appropriate.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

// XXX trim includes
#include <nuttx/config.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <v1.0/common/mavlink.h>
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <sys/prctl.h>
#include <stdlib.h>
#include <poll.h>

#include "waypoints.h"
#include "mavlink_log.h"
#include "orb_topics.h"
#include "missionlib.h"
#include "mavlink_hil.h"
#include "util.h"

extern bool gcs_link;

struct vehicle_global_position_s global_pos;
struct vehicle_local_position_s local_pos;
struct vehicle_status_s v_status;
struct rc_channels_s rc;
struct rc_input_values rc_raw;
struct actuator_armed_s armed;

struct mavlink_subscriptions mavlink_subs;

static int status_sub;
static int rc_sub;

static unsigned int sensors_raw_counter;
static unsigned int attitude_counter;
static unsigned int gps_counter;

/*
 * Last sensor loop time
 * some outputs are better timestamped
 * with this "global" reference.
 */
static uint64_t last_sensor_timestamp;

static void	*uorb_receive_thread(void *arg);

struct listener {
	void	(*callback)(const struct listener *l);
	int		*subp;
	uintptr_t	arg;
};

static void	l_sensor_combined(const struct listener *l);
static void	l_vehicle_attitude(const struct listener *l);
static void	l_vehicle_gps_position(const struct listener *l);
static void	l_vehicle_status(const struct listener *l);
static void	l_rc_channels(const struct listener *l);
static void	l_input_rc(const struct listener *l);
static void	l_global_position(const struct listener *l);
static void	l_local_position(const struct listener *l);
static void	l_global_position_setpoint(const struct listener *l);
static void	l_local_position_setpoint(const struct listener *l);
static void	l_attitude_setpoint(const struct listener *l);
static void	l_actuator_outputs(const struct listener *l);
static void	l_actuator_armed(const struct listener *l);
static void	l_manual_control_setpoint(const struct listener *l);
static void	l_vehicle_attitude_controls(const struct listener *l);
static void	l_debug_key_value(const struct listener *l);
static void	l_optical_flow(const struct listener *l);
static void	l_vehicle_rates_setpoint(const struct listener *l);
static void	l_home(const struct listener *l);

static const struct listener listeners[] = {
	{l_sensor_combined,		&mavlink_subs.sensor_sub,	0},
	{l_vehicle_attitude,		&mavlink_subs.att_sub,		0},
	{l_vehicle_gps_position,	&mavlink_subs.gps_sub,		0},
	{l_vehicle_status,		&status_sub,			0},
	{l_rc_channels,			&rc_sub,			0},
	{l_input_rc,			&mavlink_subs.input_rc_sub,	0},
	{l_global_position,		&mavlink_subs.global_pos_sub,	0},
	{l_local_position,		&mavlink_subs.local_pos_sub,	0},
	{l_global_position_setpoint,	&mavlink_subs.spg_sub,		0},
	{l_local_position_setpoint,	&mavlink_subs.spl_sub,		0},
	{l_attitude_setpoint,		&mavlink_subs.spa_sub,		0},
	{l_actuator_outputs,		&mavlink_subs.act_0_sub,	0},
	{l_actuator_outputs,		&mavlink_subs.act_1_sub,	1},
	{l_actuator_outputs,		&mavlink_subs.act_2_sub,	2},
	{l_actuator_outputs,		&mavlink_subs.act_3_sub,	3},
	{l_actuator_armed,		&mavlink_subs.armed_sub,	0},
	{l_manual_control_setpoint,	&mavlink_subs.man_control_sp_sub, 0},
	{l_vehicle_attitude_controls,	&mavlink_subs.actuators_sub,	0},
	{l_debug_key_value,		&mavlink_subs.debug_key_value,	0},
	{l_optical_flow,		&mavlink_subs.optical_flow,	0},
	{l_vehicle_rates_setpoint,	&mavlink_subs.rates_setpoint_sub,	0},
	{l_home,			&mavlink_subs.home_sub,		0},
};

static const unsigned n_listeners = sizeof(listeners) / sizeof(listeners[0]);

void
l_sensor_combined(const struct listener *l)
{
	struct sensor_combined_s raw;

	/* copy sensors raw data into local buffer */
	orb_copy(ORB_ID(sensor_combined), mavlink_subs.sensor_sub, &raw);

	last_sensor_timestamp = raw.timestamp;

	/* mark individual fields as changed */
	uint16_t fields_updated = 0;
	static unsigned accel_counter = 0;
	static unsigned gyro_counter = 0;
	static unsigned mag_counter = 0;
	static unsigned baro_counter = 0;

	if (accel_counter != raw.accelerometer_counter) {
		/* mark first three dimensions as changed */
		fields_updated |= (1 << 0) | (1 << 1) | (1 << 2);
		accel_counter = raw.accelerometer_counter;
	}

	if (gyro_counter != raw.gyro_counter) {
		/* mark second group dimensions as changed */
		fields_updated |= (1 << 3) | (1 << 4) | (1 << 5);
		gyro_counter = raw.gyro_counter;
	}

	if (mag_counter != raw.magnetometer_counter) {
		/* mark third group dimensions as changed */
		fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);
		mag_counter = raw.magnetometer_counter;
	}

	if (baro_counter != raw.baro_counter) {
		/* mark last group dimensions as changed */
		fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);
		baro_counter = raw.baro_counter;
	}

	if (gcs_link)
		mavlink_msg_highres_imu_send(MAVLINK_COMM_0, last_sensor_timestamp,
					     raw.accelerometer_m_s2[0], raw.accelerometer_m_s2[1],
					     raw.accelerometer_m_s2[2], raw.gyro_rad_s[0],
					     raw.gyro_rad_s[1], raw.gyro_rad_s[2],
					     raw.magnetometer_ga[0],
					     raw.magnetometer_ga[1], raw.magnetometer_ga[2],
					     raw.baro_pres_mbar, 0 /* no diff pressure yet */,
					     raw.baro_alt_meter, raw.baro_temp_celcius,
					     fields_updated);

	sensors_raw_counter++;
}

void
l_vehicle_attitude(const struct listener *l)
{
	struct vehicle_attitude_s att;


	/* copy attitude data into local buffer */
	orb_copy(ORB_ID(vehicle_attitude), mavlink_subs.att_sub, &att);

	if (gcs_link)
		/* send sensor values */
		mavlink_msg_attitude_send(MAVLINK_COMM_0,
					  last_sensor_timestamp / 1000,
					  att.roll,
					  att.pitch,
					  att.yaw,
					  att.rollspeed,
					  att.pitchspeed,
					  att.yawspeed);

	attitude_counter++;
}

void
l_vehicle_gps_position(const struct listener *l)
{
	struct vehicle_gps_position_s gps;

	/* copy gps data into local buffer */
	orb_copy(ORB_ID(vehicle_gps_position), mavlink_subs.gps_sub, &gps);

	/* GPS position */
	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0,
				     gps.timestamp_position,
				     gps.fix_type,
				     gps.lat,
				     gps.lon,
				     gps.alt,
				     (uint16_t)(gps.eph_m * 1e2f), // from m to cm
				     (uint16_t)(gps.epv_m * 1e2f), // from m to cm
				     (uint16_t)(gps.vel_m_s * 1e2f), // from m/s to cm/s
				     (uint16_t)(gps.cog_rad * M_RAD_TO_DEG_F * 1e2f), // from rad to deg * 100
				     gps.satellites_visible);

	if (gps.satellite_info_available && (gps_counter % 4 == 0)) {
		mavlink_msg_gps_status_send(MAVLINK_COMM_0,
					    gps.satellites_visible,
					    gps.satellite_prn,
					    gps.satellite_used,
					    gps.satellite_elevation,
					    gps.satellite_azimuth,
					    gps.satellite_snr);
	}

	gps_counter++;
}

void
l_vehicle_status(const struct listener *l)
{
	/* immediately communicate state changes back to user */
	orb_copy(ORB_ID(vehicle_status), status_sub, &v_status);
	orb_copy(ORB_ID(actuator_armed), mavlink_subs.armed_sub, &armed);

	/* enable or disable HIL */
	set_hil_on_off(v_status.flag_hil_enabled);

	/* translate the current syste state to mavlink state and mode */
	uint8_t mavlink_state = 0;
	uint8_t mavlink_mode = 0;
	get_mavlink_mode_and_state(&mavlink_state, &mavlink_mode);

	/* send heartbeat */
	mavlink_msg_heartbeat_send(chan,
				   mavlink_system.type,
				   MAV_AUTOPILOT_PX4,
				   mavlink_mode,
				   v_status.state_machine,
				   mavlink_state);
}

void
l_rc_channels(const struct listener *l)
{
	/* copy rc channels into local buffer */
	orb_copy(ORB_ID(rc_channels), rc_sub, &rc);
	// XXX Add RC channels scaled message here
}

void
l_input_rc(const struct listener *l)
{
	/* copy rc channels into local buffer */
	orb_copy(ORB_ID(input_rc), mavlink_subs.input_rc_sub, &rc_raw);

	if (gcs_link)
		/* Channels are sent in MAVLink main loop at a fixed interval */
		mavlink_msg_rc_channels_raw_send(chan,
						 rc_raw.timestamp / 1000,
						 0,
						 (rc_raw.channel_count > 0) ? rc_raw.values[0] : UINT16_MAX,
						 (rc_raw.channel_count > 1) ? rc_raw.values[1] : UINT16_MAX,
						 (rc_raw.channel_count > 2) ? rc_raw.values[2] : UINT16_MAX,
						 (rc_raw.channel_count > 3) ? rc_raw.values[3] : UINT16_MAX,
						 (rc_raw.channel_count > 4) ? rc_raw.values[4] : UINT16_MAX,
						 (rc_raw.channel_count > 5) ? rc_raw.values[5] : UINT16_MAX,
						 (rc_raw.channel_count > 6) ? rc_raw.values[6] : UINT16_MAX,
						 (rc_raw.channel_count > 7) ? rc_raw.values[7] : UINT16_MAX,
						 255);
}

void
l_global_position(const struct listener *l)
{
	/* copy global position data into local buffer */
	orb_copy(ORB_ID(vehicle_global_position), mavlink_subs.global_pos_sub, &global_pos);

	uint64_t timestamp = global_pos.timestamp;
	int32_t lat = global_pos.lat;
	int32_t lon = global_pos.lon;
	int32_t alt = (int32_t)(global_pos.alt * 1000);
	int32_t relative_alt = (int32_t)(global_pos.relative_alt * 1000.0f);
	int16_t vx = (int16_t)(global_pos.vx * 100.0f);
	int16_t vy = (int16_t)(global_pos.vy * 100.0f);
	int16_t vz = (int16_t)(global_pos.vz * 100.0f);

	/* heading in degrees * 10, from 0 to 36.000) */
	uint16_t hdg = (global_pos.hdg / M_PI_F) * (180.0f * 10.0f) + (180.0f * 10.0f);

	mavlink_msg_global_position_int_send(MAVLINK_COMM_0,
					     timestamp / 1000,
					     lat,
					     lon,
					     alt,
					     relative_alt,
					     vx,
					     vy,
					     vz,
					     hdg);
}

void
l_local_position(const struct listener *l)
{
	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_local_position), mavlink_subs.local_pos_sub, &local_pos);

	if (gcs_link)
		mavlink_msg_local_position_ned_send(MAVLINK_COMM_0,
						    local_pos.timestamp / 1000,
						    local_pos.x,
						    local_pos.y,
						    local_pos.z,
						    local_pos.vx,
						    local_pos.vy,
						    local_pos.vz);
}

void
l_global_position_setpoint(const struct listener *l)
{
	struct vehicle_global_position_setpoint_s global_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_global_position_setpoint), mavlink_subs.spg_sub, &global_sp);

	uint8_t coordinate_frame = MAV_FRAME_GLOBAL;

	if (global_sp.altitude_is_relative)
		coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;

	if (gcs_link)
		mavlink_msg_global_position_setpoint_int_send(MAVLINK_COMM_0,
				coordinate_frame,
				global_sp.lat,
				global_sp.lon,
				global_sp.altitude,
				global_sp.yaw);
}

void
l_local_position_setpoint(const struct listener *l)
{
	struct vehicle_local_position_setpoint_s local_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_local_position_setpoint), mavlink_subs.spl_sub, &local_sp);

	if (gcs_link)
		mavlink_msg_local_position_setpoint_send(MAVLINK_COMM_0,
				MAV_FRAME_LOCAL_NED,
				local_sp.x,
				local_sp.y,
				local_sp.z,
				local_sp.yaw);
}

void
l_attitude_setpoint(const struct listener *l)
{
	struct vehicle_attitude_setpoint_s att_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_attitude_setpoint), mavlink_subs.spa_sub, &att_sp);

	if (gcs_link)
		mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0,
				att_sp.timestamp / 1000,
				att_sp.roll_body,
				att_sp.pitch_body,
				att_sp.yaw_body,
				att_sp.thrust);
}

void
l_vehicle_rates_setpoint(const struct listener *l)
{
	struct vehicle_rates_setpoint_s rates_sp;

	/* copy local position data into local buffer */
	orb_copy(ORB_ID(vehicle_rates_setpoint), mavlink_subs.rates_setpoint_sub, &rates_sp);

	if (gcs_link)
		mavlink_msg_roll_pitch_yaw_rates_thrust_setpoint_send(MAVLINK_COMM_0,
				rates_sp.timestamp / 1000,
				rates_sp.roll,
				rates_sp.pitch,
				rates_sp.yaw,
				rates_sp.thrust);
}

void
l_actuator_outputs(const struct listener *l)
{
	struct actuator_outputs_s act_outputs;

	orb_id_t ids[] = {
		ORB_ID(actuator_outputs_0),
		ORB_ID(actuator_outputs_1),
		ORB_ID(actuator_outputs_2),
		ORB_ID(actuator_outputs_3)
	};

	/* copy actuator data into local buffer */
	orb_copy(ids[l->arg], *l->subp, &act_outputs);

	if (gcs_link) {
		mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, last_sensor_timestamp / 1000,
						  l->arg /* port number */,
						  act_outputs.output[0],
						  act_outputs.output[1],
						  act_outputs.output[2],
						  act_outputs.output[3],
						  act_outputs.output[4],
						  act_outputs.output[5],
						  act_outputs.output[6],
						  act_outputs.output[7]);

		/* only send in HIL mode */
		if (mavlink_hil_enabled && armed.armed) {

			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state = 0;
			uint8_t mavlink_mode = 0;
			get_mavlink_mode_and_state(&mavlink_state, &mavlink_mode);

			/* HIL message as per MAVLink spec */

			/* scale / assign outputs depending on system type */

			if (mavlink_system.type == MAV_TYPE_QUADROTOR) {
				mavlink_msg_hil_controls_send(chan,
							      hrt_absolute_time(),
							      ((act_outputs.output[0] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[1] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[2] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[3] - 900.0f) / 600.0f) / 2.0f,
							      -1,
							      -1,
							      -1,
							      -1,
							      mavlink_mode,
							      0);

			} else if (mavlink_system.type == MAV_TYPE_HEXAROTOR) {
				mavlink_msg_hil_controls_send(chan,
							      hrt_absolute_time(),
							      ((act_outputs.output[0] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[1] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[2] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[3] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[4] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[5] - 900.0f) / 600.0f) / 2.0f,
							      -1,
							      -1,
							      mavlink_mode,
							      0);

			} else if (mavlink_system.type == MAV_TYPE_OCTOROTOR) {
				mavlink_msg_hil_controls_send(chan,
							      hrt_absolute_time(),
							      ((act_outputs.output[0] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[1] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[2] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[3] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[4] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[5] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[6] - 900.0f) / 600.0f) / 2.0f,
							      ((act_outputs.output[7] - 900.0f) / 600.0f) / 2.0f,
							      mavlink_mode,
							      0);

			} else {
				mavlink_msg_hil_controls_send(chan,
							      hrt_absolute_time(),
							      (act_outputs.output[0] - 1500.0f) / 500.0f,
							      (act_outputs.output[1] - 1500.0f) / 500.0f,
							      (act_outputs.output[2] - 1500.0f) / 500.0f,
							      (act_outputs.output[3] - 1000.0f) / 1000.0f,
							      (act_outputs.output[4] - 1500.0f) / 500.0f,
							      (act_outputs.output[5] - 1500.0f) / 500.0f,
							      (act_outputs.output[6] - 1500.0f) / 500.0f,
							      (act_outputs.output[7] - 1500.0f) / 500.0f,
							      mavlink_mode,
							      0);
			}
		}
	}
}

void
l_actuator_armed(const struct listener *l)
{
	orb_copy(ORB_ID(actuator_armed), mavlink_subs.armed_sub, &armed);
}

void
l_manual_control_setpoint(const struct listener *l)
{
	struct manual_control_setpoint_s man_control;

	/* copy manual control data into local buffer */
	orb_copy(ORB_ID(manual_control_setpoint), mavlink_subs.man_control_sp_sub, &man_control);

	if (gcs_link)
		mavlink_msg_manual_control_send(MAVLINK_COMM_0,
						mavlink_system.sysid,
						man_control.roll * 1000,
						man_control.pitch * 1000,
						man_control.yaw * 1000,
						man_control.throttle * 1000,
						0);
}

void
l_vehicle_attitude_controls(const struct listener *l)
{
	struct actuator_controls_effective_s actuators;

	orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE, mavlink_subs.actuators_sub, &actuators);

	if (gcs_link) {
		/* send, add spaces so that string buffer is at least 10 chars long */
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,
						   last_sensor_timestamp / 1000,
						   "eff ctrl0    ",
						   actuators.control_effective[0]);
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,
						   last_sensor_timestamp / 1000,
						   "eff ctrl1    ",
						   actuators.control_effective[1]);
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,
						   last_sensor_timestamp / 1000,
						   "eff ctrl2     ",
						   actuators.control_effective[2]);
		mavlink_msg_named_value_float_send(MAVLINK_COMM_0,
						   last_sensor_timestamp / 1000,
						   "eff ctrl3     ",
						   actuators.control_effective[3]);
	}
}

void
l_debug_key_value(const struct listener *l)
{
	struct debug_key_value_s debug;

	orb_copy(ORB_ID(debug_key_value), mavlink_subs.debug_key_value, &debug);

	/* Enforce null termination */
	debug.key[sizeof(debug.key) - 1] = '\0';

	mavlink_msg_named_value_float_send(MAVLINK_COMM_0,
					   last_sensor_timestamp / 1000,
					   debug.key,
					   debug.value);
}

void
l_optical_flow(const struct listener *l)
{
	struct optical_flow_s flow;

	orb_copy(ORB_ID(optical_flow), mavlink_subs.optical_flow, &flow);

	mavlink_msg_optical_flow_send(MAVLINK_COMM_0, flow.timestamp, flow.sensor_id, flow.flow_raw_x, flow.flow_raw_y,
				      flow.flow_comp_x_m, flow.flow_comp_y_m, flow.quality, flow.ground_distance_m);
}

void
l_home(const struct listener *l)
{
	struct home_position_s home;

	orb_copy(ORB_ID(home_position), mavlink_subs.home_sub, &home);

	mavlink_msg_gps_global_origin_send(MAVLINK_COMM_0, home.lat, home.lon, home.alt);
}

static void *
uorb_receive_thread(void *arg)
{
	/* Set thread name */
	prctl(PR_SET_NAME, "mavlink orb rcv", getpid());

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms (1 second)
	 */
	const int timeout = 1000;

	/*
	 * Initialise listener array.
	 *
	 * Might want to invoke each listener once to set initial state.
	 */
	struct pollfd fds[n_listeners];

	for (unsigned i = 0; i < n_listeners; i++) {
		fds[i].fd = *listeners[i].subp;
		fds[i].events = POLLIN;

		/* Invoke callback to set initial state */
		//listeners[i].callback(&listener[i]);
	}

	while (!thread_should_exit) {

		int poll_ret = poll(fds, n_listeners, timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			mavlink_missionlib_send_gcs_string("[mavlink] No telemetry data for 1 s");

		} else if (poll_ret < 0) {
			mavlink_missionlib_send_gcs_string("[mavlink] ERROR reading uORB data");

		} else {

			for (unsigned i = 0; i < n_listeners; i++) {
				if (fds[i].revents & POLLIN)
					listeners[i].callback(&listeners[i]);
			}
		}
	}

	return NULL;
}

pthread_t
uorb_receive_start(void)
{
	/* --- SENSORS RAW VALUE --- */
	mavlink_subs.sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	/* rate limit set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink_subs.sensor_sub, 200);	/* 5Hz updates */

	/* --- ATTITUDE VALUE --- */
	mavlink_subs.att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	/* rate limit set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink_subs.att_sub, 200);	/* 5Hz updates */

	/* --- GPS VALUE --- */
	mavlink_subs.gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	orb_set_interval(mavlink_subs.gps_sub, 200);	/* 5Hz updates */

	/* --- HOME POSITION --- */
	mavlink_subs.home_sub = orb_subscribe(ORB_ID(home_position));
	orb_set_interval(mavlink_subs.home_sub, 1000);	/* 1Hz updates */

	/* --- SYSTEM STATE --- */
	status_sub = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(status_sub, 300);		/* max 3.33 Hz updates */

	/* --- RC CHANNELS VALUE --- */
	rc_sub = orb_subscribe(ORB_ID(rc_channels));
	orb_set_interval(rc_sub, 100);			/* 10Hz updates */

	/* --- RC RAW VALUE --- */
	mavlink_subs.input_rc_sub = orb_subscribe(ORB_ID(input_rc));
	orb_set_interval(mavlink_subs.input_rc_sub, 100);

	/* --- GLOBAL POS VALUE --- */
	mavlink_subs.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	orb_set_interval(mavlink_subs.global_pos_sub, 100);	/* 10 Hz active updates */

	/* --- LOCAL POS VALUE --- */
	mavlink_subs.local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_set_interval(mavlink_subs.local_pos_sub, 1000);	/* 1Hz active updates */

	/* --- GLOBAL SETPOINT VALUE --- */
	mavlink_subs.spg_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	orb_set_interval(mavlink_subs.spg_sub, 2000);	/* 0.5 Hz updates */

	/* --- LOCAL SETPOINT VALUE --- */
	mavlink_subs.spl_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	orb_set_interval(mavlink_subs.spl_sub, 2000);	/* 0.5 Hz updates */

	/* --- ATTITUDE SETPOINT VALUE --- */
	mavlink_subs.spa_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	orb_set_interval(mavlink_subs.spa_sub, 2000);	/* 0.5 Hz updates */

	/* --- RATES SETPOINT VALUE --- */
	mavlink_subs.rates_setpoint_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	orb_set_interval(mavlink_subs.rates_setpoint_sub, 2000);  /* 0.5 Hz updates */

	/* --- ACTUATOR OUTPUTS --- */
	mavlink_subs.act_0_sub = orb_subscribe(ORB_ID(actuator_outputs_0));
	mavlink_subs.act_1_sub = orb_subscribe(ORB_ID(actuator_outputs_1));
	mavlink_subs.act_2_sub = orb_subscribe(ORB_ID(actuator_outputs_2));
	mavlink_subs.act_3_sub = orb_subscribe(ORB_ID(actuator_outputs_3));
	/* rate limits set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink_subs.act_0_sub, 100);	/* 10Hz updates */
	orb_set_interval(mavlink_subs.act_1_sub, 100);	/* 10Hz updates */
	orb_set_interval(mavlink_subs.act_2_sub, 100);	/* 10Hz updates */
	orb_set_interval(mavlink_subs.act_3_sub, 100);	/* 10Hz updates */

	/* --- ACTUATOR ARMED VALUE --- */
	mavlink_subs.armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	orb_set_interval(mavlink_subs.armed_sub, 100);	/* 10Hz updates */

	/* --- MAPPED MANUAL CONTROL INPUTS --- */
	mavlink_subs.man_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	/* rate limits set externally based on interface speed, set a basic default here */
	orb_set_interval(mavlink_subs.man_control_sp_sub, 100);	/* 10Hz updates */

	/* --- ACTUATOR CONTROL VALUE --- */
	mavlink_subs.actuators_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS_EFFECTIVE);
	orb_set_interval(mavlink_subs.actuators_sub, 100);	/* 10Hz updates */

	/* --- DEBUG VALUE OUTPUT --- */
	mavlink_subs.debug_key_value = orb_subscribe(ORB_ID(debug_key_value));
	orb_set_interval(mavlink_subs.debug_key_value, 100);	/* 10Hz updates */

	/* --- FLOW SENSOR --- */
	mavlink_subs.optical_flow = orb_subscribe(ORB_ID(optical_flow));
	orb_set_interval(mavlink_subs.optical_flow, 200); 	/* 5Hz updates */

	/* start the listener loop */
	pthread_attr_t uorb_attr;
	pthread_attr_init(&uorb_attr);

	/* Set stack size, needs less than 2k */
	pthread_attr_setstacksize(&uorb_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &uorb_attr, uorb_receive_thread, NULL);
	return thread;
}
