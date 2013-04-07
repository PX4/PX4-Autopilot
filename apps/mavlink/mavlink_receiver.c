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
 * @file mavlink_receiver.c
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
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
#include "mavlink_bridge_header.h"
#include <v1.0/common/mavlink.h>
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#include "waypoints.h"
#include "mavlink_log.h"
#include "orb_topics.h"
#include "missionlib.h"
#include "mavlink_hil.h"
#include "mavlink_parameters.h"
#include "util.h"

/* XXX should be in a header somewhere */
pthread_t receive_start(int uart);

static void handle_message(mavlink_message_t *msg);
static void *receive_thread(void *arg);

static mavlink_status_t status;
static struct vehicle_vicon_position_s vicon_position;
static struct vehicle_command_s vcmd;
static struct offboard_control_setpoint_s offboard_control_sp;

struct vehicle_global_position_s hil_global_pos;
struct vehicle_attitude_s hil_attitude;
struct vehicle_gps_position_s hil_gps;
struct sensor_combined_s hil_sensors;
orb_advert_t pub_hil_global_pos = -1;
orb_advert_t pub_hil_attitude = -1;
orb_advert_t pub_hil_gps = -1;
orb_advert_t pub_hil_sensors = -1;

static orb_advert_t cmd_pub = -1;
static orb_advert_t flow_pub = -1;

static orb_advert_t offboard_control_sp_pub = -1;
static orb_advert_t vicon_position_pub = -1;

extern bool gcs_link;

static void
handle_message(mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_COMMAND_LONG) {

		mavlink_command_long_t cmd_mavlink;
		mavlink_msg_command_long_decode(msg, &cmd_mavlink);

		if (cmd_mavlink.target_system == mavlink_system.sysid && ((cmd_mavlink.target_component == mavlink_system.compid)
				|| (cmd_mavlink.target_component == MAV_COMP_ID_ALL))) {
			//check for MAVLINK terminate command
			if (cmd_mavlink.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN && ((int)cmd_mavlink.param1) == 3) {
				/* This is the link shutdown command, terminate mavlink */
				printf("[mavlink] Terminating .. \n");
				fflush(stdout);
				usleep(50000);

				/* terminate other threads and this thread */
				thread_should_exit = true;

			} else {

				/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
				vcmd.param1 = cmd_mavlink.param1;
				vcmd.param2 = cmd_mavlink.param2;
				vcmd.param3 = cmd_mavlink.param3;
				vcmd.param4 = cmd_mavlink.param4;
				vcmd.param5 = cmd_mavlink.param5;
				vcmd.param6 = cmd_mavlink.param6;
				vcmd.param7 = cmd_mavlink.param7;
				vcmd.command = cmd_mavlink.command;
				vcmd.target_system = cmd_mavlink.target_system;
				vcmd.target_component = cmd_mavlink.target_component;
				vcmd.source_system = msg->sysid;
				vcmd.source_component = msg->compid;
				vcmd.confirmation =  cmd_mavlink.confirmation;

				/* check if topic is advertised */
				if (cmd_pub <= 0) {
					cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);
				}

				/* publish */
				orb_publish(ORB_ID(vehicle_command), cmd_pub, &vcmd);
			}
		}
	}

	if (msg->msgid == MAVLINK_MSG_ID_OPTICAL_FLOW) {
		mavlink_optical_flow_t flow;
		mavlink_msg_optical_flow_decode(msg, &flow);

		struct optical_flow_s f;

		f.timestamp = flow.time_usec;
		f.flow_raw_x = flow.flow_x;
		f.flow_raw_y = flow.flow_y;
		f.flow_comp_x_m = flow.flow_comp_m_x;
		f.flow_comp_y_m = flow.flow_comp_m_y;
		f.ground_distance_m = flow.ground_distance;
		f.quality = flow.quality;
		f.sensor_id = flow.sensor_id;

		/* check if topic is advertised */
		if (flow_pub <= 0) {
			flow_pub = orb_advertise(ORB_ID(optical_flow), &f);

		} else {
			/* publish */
			orb_publish(ORB_ID(optical_flow), flow_pub, &f);
		}
	}

	if (msg->msgid == MAVLINK_MSG_ID_SET_MODE) {
		/* Set mode on request */
		mavlink_set_mode_t new_mode;
		mavlink_msg_set_mode_decode(msg, &new_mode);

		/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
		vcmd.param1 = new_mode.base_mode;
		vcmd.param2 = new_mode.custom_mode;
		vcmd.param3 = 0;
		vcmd.param4 = 0;
		vcmd.param5 = 0;
		vcmd.param6 = 0;
		vcmd.param7 = 0;
		vcmd.command = MAV_CMD_DO_SET_MODE;
		vcmd.target_system = new_mode.target_system;
		vcmd.target_component = MAV_COMP_ID_ALL;
		vcmd.source_system = msg->sysid;
		vcmd.source_component = msg->compid;
		vcmd.confirmation = 1;

		/* check if topic is advertised */
		if (cmd_pub <= 0) {
			cmd_pub = orb_advertise(ORB_ID(vehicle_command), &vcmd);

		} else {
			/* create command */
			orb_publish(ORB_ID(vehicle_command), cmd_pub, &vcmd);
		}
	}

	/* Handle Vicon position estimates */
	if (msg->msgid == MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE) {
		mavlink_vicon_position_estimate_t pos;
		mavlink_msg_vicon_position_estimate_decode(msg, &pos);

		vicon_position.timestamp = hrt_absolute_time();

		vicon_position.x = pos.x;
		vicon_position.y = pos.y;
		vicon_position.z = pos.z;

		vicon_position.roll = pos.roll;
		vicon_position.pitch = pos.pitch;
		vicon_position.yaw = pos.yaw;

		if (vicon_position_pub <= 0) {
			vicon_position_pub = orb_advertise(ORB_ID(vehicle_vicon_position), &vicon_position);

		} else {
			orb_publish(ORB_ID(vehicle_vicon_position), vicon_position_pub, &vicon_position);
		}
	}

	/* Handle quadrotor motor setpoints */

	if (msg->msgid == MAVLINK_MSG_ID_SET_QUAD_SWARM_ROLL_PITCH_YAW_THRUST) {
		mavlink_set_quad_swarm_roll_pitch_yaw_thrust_t quad_motors_setpoint;
		mavlink_msg_set_quad_swarm_roll_pitch_yaw_thrust_decode(msg, &quad_motors_setpoint);

		if (mavlink_system.sysid < 4) {

			/* switch to a receiving link mode */
			gcs_link = false;

			/*
			 * rate control mode - defined by MAVLink
			 */

			uint8_t ml_mode = 0;
			bool ml_armed = false;

			switch (quad_motors_setpoint.mode) {
			case 0:
				ml_armed = false;
				break;

			case 1:
				ml_mode = OFFBOARD_CONTROL_MODE_DIRECT_RATES;
				ml_armed = true;

				break;

			case 2:
				ml_mode = OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE;
				ml_armed = true;

				break;

			case 3:
				ml_mode = OFFBOARD_CONTROL_MODE_DIRECT_VELOCITY;
				break;

			case 4:
				ml_mode = OFFBOARD_CONTROL_MODE_DIRECT_POSITION;
				break;
			}

			offboard_control_sp.p1 = (float)quad_motors_setpoint.roll[mavlink_system.sysid - 1]   / (float)INT16_MAX;
			offboard_control_sp.p2 = (float)quad_motors_setpoint.pitch[mavlink_system.sysid - 1]  / (float)INT16_MAX;
			offboard_control_sp.p3 = (float)quad_motors_setpoint.yaw[mavlink_system.sysid - 1]    / (float)INT16_MAX;
			offboard_control_sp.p4 = (float)quad_motors_setpoint.thrust[mavlink_system.sysid - 1] / (float)UINT16_MAX;

			if (quad_motors_setpoint.thrust[mavlink_system.sysid - 1] == 0) {
				ml_armed = false;
			}

			offboard_control_sp.armed = ml_armed;
			offboard_control_sp.mode = ml_mode;

			offboard_control_sp.timestamp = hrt_absolute_time();

			/* check if topic has to be advertised */
			if (offboard_control_sp_pub <= 0) {
				offboard_control_sp_pub = orb_advertise(ORB_ID(offboard_control_setpoint), &offboard_control_sp);

			} else {
				/* Publish */
				orb_publish(ORB_ID(offboard_control_setpoint), offboard_control_sp_pub, &offboard_control_sp);
			}
		}
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 */

	if (mavlink_hil_enabled) {

		uint64_t timestamp = hrt_absolute_time();

		/* TODO, set ground_press/ temp during calib */
		static const float ground_press = 1013.25f; // mbar
		static const float ground_tempC = 21.0f;
		static const float ground_alt = 0.0f;
		static const float T0 = 273.15;
		static const float R = 287.05f;
		static const float g = 9.806f;

		if (msg->msgid == MAVLINK_MSG_ID_RAW_IMU) {

			mavlink_raw_imu_t imu;
			mavlink_msg_raw_imu_decode(msg, &imu);

			/* packet counter */
			static uint16_t hil_counter = 0;
			static uint16_t hil_frames = 0;
			static uint64_t old_timestamp = 0;

			/* sensors general */
			hil_sensors.timestamp = imu.time_usec;

			/* hil gyro */
			static const float mrad2rad = 1.0e-3f;
			hil_sensors.gyro_counter = hil_counter;
			hil_sensors.gyro_raw[0] = imu.xgyro;
			hil_sensors.gyro_raw[1] = imu.ygyro;
			hil_sensors.gyro_raw[2] = imu.zgyro;
			hil_sensors.gyro_rad_s[0] = imu.xgyro * mrad2rad;
			hil_sensors.gyro_rad_s[1] = imu.ygyro * mrad2rad;
			hil_sensors.gyro_rad_s[2] = imu.zgyro * mrad2rad;

			/* accelerometer */
			hil_sensors.accelerometer_counter = hil_counter;
			static const float mg2ms2 = 9.8f / 1000.0f;
			hil_sensors.accelerometer_raw[0] = imu.xacc;
			hil_sensors.accelerometer_raw[1] = imu.yacc;
			hil_sensors.accelerometer_raw[2] = imu.zacc;
			hil_sensors.accelerometer_m_s2[0] = mg2ms2 * imu.xacc;
			hil_sensors.accelerometer_m_s2[1] = mg2ms2 * imu.yacc;
			hil_sensors.accelerometer_m_s2[2] = mg2ms2 * imu.zacc;
			hil_sensors.accelerometer_mode = 0; // TODO what is this?
			hil_sensors.accelerometer_range_m_s2 = 32.7f; // int16

			/* adc */
			hil_sensors.adc_voltage_v[0] = 0;
			hil_sensors.adc_voltage_v[1] = 0;
			hil_sensors.adc_voltage_v[2] = 0;

			/* magnetometer */
			float mga2ga = 1.0e-3f;
			hil_sensors.magnetometer_counter = hil_counter;
			hil_sensors.magnetometer_raw[0] = imu.xmag;
			hil_sensors.magnetometer_raw[1] = imu.ymag;
			hil_sensors.magnetometer_raw[2] = imu.zmag;
			hil_sensors.magnetometer_ga[0] = imu.xmag * mga2ga;
			hil_sensors.magnetometer_ga[1] = imu.ymag * mga2ga;
			hil_sensors.magnetometer_ga[2] = imu.zmag * mga2ga;
			hil_sensors.magnetometer_range_ga = 32.7f; // int16
			hil_sensors.magnetometer_mode = 0; // TODO what is this
			hil_sensors.magnetometer_cuttoff_freq_hz = 50.0f;

			/* publish */
			orb_publish(ORB_ID(sensor_combined), pub_hil_sensors, &hil_sensors);

			// increment counters
			hil_counter += 1 ;
			hil_frames += 1 ;

			// output
			if ((timestamp - old_timestamp) > 10000000) {
				printf("receiving hil imu at %d hz\n", hil_frames/10);
				old_timestamp = timestamp;
				hil_frames = 0;
			}
		}

		if (msg->msgid == MAVLINK_MSG_ID_HIGHRES_IMU) {

			mavlink_highres_imu_t imu;
			mavlink_msg_highres_imu_decode(msg, &imu);

			/* packet counter */
			static uint16_t hil_counter = 0;
			static uint16_t hil_frames = 0;
			static uint64_t old_timestamp = 0;

			/* sensors general */
			hil_sensors.timestamp = imu.time_usec;

			/* hil gyro */
			static const float mrad2rad = 1.0e-3f;
			hil_sensors.gyro_counter = hil_counter;
			hil_sensors.gyro_raw[0] = imu.xgyro / mrad2rad;
			hil_sensors.gyro_raw[1] = imu.ygyro / mrad2rad;
			hil_sensors.gyro_raw[2] = imu.zgyro / mrad2rad;
			hil_sensors.gyro_rad_s[0] = imu.xgyro;
			hil_sensors.gyro_rad_s[1] = imu.ygyro;
			hil_sensors.gyro_rad_s[2] = imu.zgyro;

			/* accelerometer */
			hil_sensors.accelerometer_counter = hil_counter;
			static const float mg2ms2 = 9.8f / 1000.0f;
			hil_sensors.accelerometer_raw[0] = imu.xacc / mg2ms2;
			hil_sensors.accelerometer_raw[1] = imu.yacc / mg2ms2;
			hil_sensors.accelerometer_raw[2] = imu.zacc / mg2ms2;
			hil_sensors.accelerometer_m_s2[0] = imu.xacc;
			hil_sensors.accelerometer_m_s2[1] = imu.yacc;
			hil_sensors.accelerometer_m_s2[2] = imu.zacc;
			hil_sensors.accelerometer_mode = 0; // TODO what is this?
			hil_sensors.accelerometer_range_m_s2 = 32.7f; // int16

			/* adc */
			hil_sensors.adc_voltage_v[0] = 0;
			hil_sensors.adc_voltage_v[1] = 0;
			hil_sensors.adc_voltage_v[2] = 0;

			/* magnetometer */
			float mga2ga = 1.0e-3f;
			hil_sensors.magnetometer_counter = hil_counter;
			hil_sensors.magnetometer_raw[0] = imu.xmag / mga2ga;
			hil_sensors.magnetometer_raw[1] = imu.ymag / mga2ga;
			hil_sensors.magnetometer_raw[2] = imu.zmag / mga2ga;
			hil_sensors.magnetometer_ga[0] = imu.xmag;
			hil_sensors.magnetometer_ga[1] = imu.ymag;
			hil_sensors.magnetometer_ga[2] = imu.zmag;
			hil_sensors.magnetometer_range_ga = 32.7f; // int16
			hil_sensors.magnetometer_mode = 0; // TODO what is this
			hil_sensors.magnetometer_cuttoff_freq_hz = 50.0f;

			hil_sensors.baro_pres_mbar = imu.abs_pressure;

			float tempC =  imu.temperature;
			float tempAvgK = T0 + (tempC + ground_tempC) / 2.0f;
			float h =  ground_alt + (R / g) * tempAvgK * logf(ground_press / imu.abs_pressure);

			hil_sensors.baro_alt_meter = h;
			hil_sensors.baro_temp_celcius = imu.temperature;

			/* publish */
			orb_publish(ORB_ID(sensor_combined), pub_hil_sensors, &hil_sensors);

			// increment counters
			hil_counter += 1 ;
			hil_frames += 1 ;

			// output
			if ((timestamp - old_timestamp) > 10000000) {
				printf("receiving hil imu at %d hz\n", hil_frames/10);
				old_timestamp = timestamp;
				hil_frames = 0;
			}
		}

		if (msg->msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {

			mavlink_gps_raw_int_t gps;
			mavlink_msg_gps_raw_int_decode(msg, &gps);

			/* packet counter */
			static uint16_t hil_counter = 0;
			static uint16_t hil_frames = 0;
			static uint64_t old_timestamp = 0;

			/* gps */
			hil_gps.timestamp_position = gps.time_usec;
//			hil_gps.counter = hil_counter++;
			hil_gps.time_gps_usec = gps.time_usec;
			hil_gps.lat = gps.lat;
			hil_gps.lon = gps.lon;
			hil_gps.alt = gps.alt;
//			hil_gps.counter_pos_valid = hil_counter++;
			hil_gps.eph_m = (float)gps.eph * 1e-2f; // from cm to m
			hil_gps.epv_m = (float)gps.epv * 1e-2f; // from cm to m
			hil_gps.s_variance_m_s = 100; // XXX 100 m/s variance?
			hil_gps.p_variance_m = 100; // XXX 100 m variance?
			hil_gps.vel_m_s = (float)gps.vel * 1e-2f; // from cm/s to m/s
			hil_gps.vel_n_m_s = (float)gps.vel * 1e-2f * cosf(gps.cog * M_DEG_TO_RAD_F * 1e-2f);
			hil_gps.vel_e_m_s = (float)gps.vel * 1e-2f * sinf(gps.cog * M_DEG_TO_RAD_F * 1e-2f);
			hil_gps.vel_d_m_s = 0.0f;
			hil_gps.cog_rad = gps.cog * M_DEG_TO_RAD_F * 1e-2f; // from deg*100 to rad
			hil_gps.fix_type = gps.fix_type;
			hil_gps.satellites_visible = gps.satellites_visible;

			/* publish */
			orb_publish(ORB_ID(vehicle_gps_position), pub_hil_gps, &hil_gps);

			// increment counters
			hil_counter += 1 ;
			hil_frames += 1 ;

			// output
			if ((timestamp - old_timestamp) > 10000000) {
				printf("receiving hil gps at %d hz\n", hil_frames/10);
				old_timestamp = timestamp;
				hil_frames = 0;
			}
		}

		if (msg->msgid == MAVLINK_MSG_ID_RAW_PRESSURE) {

			mavlink_raw_pressure_t press;
			mavlink_msg_raw_pressure_decode(msg, &press);

			/* packet counter */
			static uint16_t hil_counter = 0;
			static uint16_t hil_frames = 0;
			static uint64_t old_timestamp = 0;

			/* sensors general */
			hil_sensors.timestamp = press.time_usec;

			/* baro */

			float tempC =  press.temperature / 100.0f;
			float tempAvgK = T0 + (tempC + ground_tempC) / 2.0f;
			float h =  ground_alt + (R / g) * tempAvgK * logf(ground_press / press.press_abs);
			hil_sensors.baro_counter = hil_counter;
			hil_sensors.baro_pres_mbar = press.press_abs;
			hil_sensors.baro_alt_meter = h;
			hil_sensors.baro_temp_celcius = tempC;

			/* publish */
			orb_publish(ORB_ID(sensor_combined), pub_hil_sensors, &hil_sensors);

			// increment counters
			hil_counter += 1 ;
			hil_frames += 1 ;

			// output
			if ((timestamp - old_timestamp) > 10000000) {
				printf("receiving hil pressure at %d hz\n", hil_frames/10);
				old_timestamp = timestamp;
				hil_frames = 0;
			}
		}

		if (msg->msgid == MAVLINK_MSG_ID_HIL_STATE) {

			mavlink_hil_state_t hil_state;
			mavlink_msg_hil_state_decode(msg, &hil_state);

			/* Calculate Rotation Matrix */
			//TODO: better clarification which app does this, atm we have a ekf for quadrotors which does this, but there is no such thing if fly in fixed wing mode

			if (mavlink_system.type == MAV_TYPE_FIXED_WING) {
				//TODO: assuming low pitch and roll values for now
				hil_attitude.R[0][0] = cosf(hil_state.yaw);
				hil_attitude.R[0][1] = sinf(hil_state.yaw);
				hil_attitude.R[0][2] = 0.0f;

				hil_attitude.R[1][0] = -sinf(hil_state.yaw);
				hil_attitude.R[1][1] = cosf(hil_state.yaw);
				hil_attitude.R[1][2] = 0.0f;

				hil_attitude.R[2][0] = 0.0f;
				hil_attitude.R[2][1] = 0.0f;
				hil_attitude.R[2][2] = 1.0f;

				hil_attitude.R_valid = true;
			}

			hil_global_pos.lat = hil_state.lat;
			hil_global_pos.lon = hil_state.lon;
			hil_global_pos.alt = hil_state.alt / 1000.0f;
			hil_global_pos.vx = hil_state.vx / 100.0f;
			hil_global_pos.vy = hil_state.vy / 100.0f;
			hil_global_pos.vz = hil_state.vz / 100.0f;


			/* set timestamp and notify processes (broadcast) */
			hil_global_pos.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(vehicle_global_position), pub_hil_global_pos, &hil_global_pos);

			hil_attitude.roll = hil_state.roll;
			hil_attitude.pitch = hil_state.pitch;
			hil_attitude.yaw = hil_state.yaw;
			hil_attitude.rollspeed = hil_state.rollspeed;
			hil_attitude.pitchspeed = hil_state.pitchspeed;
			hil_attitude.yawspeed = hil_state.yawspeed;

			/* set timestamp and notify processes (broadcast) */
			hil_attitude.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(vehicle_attitude), pub_hil_attitude, &hil_attitude);
		}

		if (msg->msgid == MAVLINK_MSG_ID_MANUAL_CONTROL) {
			mavlink_manual_control_t man;
			mavlink_msg_manual_control_decode(msg, &man);

			struct rc_channels_s rc_hil;
			memset(&rc_hil, 0, sizeof(rc_hil));
			static orb_advert_t rc_pub = 0;

			rc_hil.timestamp = hrt_absolute_time();
			rc_hil.chan_count = 4;

			rc_hil.chan[0].scaled = man.x / 1000.0f;
			rc_hil.chan[1].scaled = man.y / 1000.0f;
			rc_hil.chan[2].scaled = man.r / 1000.0f;
			rc_hil.chan[3].scaled = man.z / 1000.0f;

			struct manual_control_setpoint_s mc;
			static orb_advert_t mc_pub = 0;

			int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

			/* get a copy first, to prevent altering values that are not sent by the mavlink command */
			orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &mc);

			mc.timestamp = rc_hil.timestamp;
			mc.roll = man.x / 1000.0f;
			mc.pitch = man.y / 1000.0f;
			mc.yaw = man.r / 1000.0f;
			mc.throttle = man.z / 1000.0f;

			/* fake RC channels with manual control input from simulator */


			if (rc_pub == 0) {
				rc_pub = orb_advertise(ORB_ID(rc_channels), &rc_hil);

			} else {
				orb_publish(ORB_ID(rc_channels), rc_pub, &rc_hil);
			}

			if (mc_pub == 0) {
				mc_pub = orb_advertise(ORB_ID(manual_control_setpoint), &mc);

			} else {
				orb_publish(ORB_ID(manual_control_setpoint), mc_pub, &mc);
			}
		}
	}
}


/**
 * Receive data from UART.
 */
static void *
receive_thread(void *arg)
{
	int uart_fd = *((int *)arg);

	const int timeout = 1000;
	uint8_t buf[32];

	mavlink_message_t msg;

	prctl(PR_SET_NAME, "mavlink uart rcv", getpid());

	while (!thread_should_exit) {

		struct pollfd fds[] = { { .fd = uart_fd, .events = POLLIN } };

		if (poll(fds, 1, timeout) > 0) {
			/* non-blocking read. read may return negative values */
			ssize_t nread = read(uart_fd, buf, sizeof(buf));

			/* if read failed, this loop won't execute */
			for (ssize_t i = 0; i < nread; i++) {
				if (mavlink_parse_char(chan, buf[i], &msg, &status)) {
					/* handle generic messages and commands */
					handle_message(&msg);

					/* Handle packet with waypoint component */
					mavlink_wpm_message_handler(&msg, &global_pos, &local_pos);

					/* Handle packet with parameter component */
					mavlink_pm_message_handler(MAVLINK_COMM_0, &msg);
				}
			}
		}
	}

	return NULL;
}

pthread_t
receive_start(int uart)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	// set to non-blocking read
	int flags = fcntl(uart, F_GETFL, 0);
	fcntl(uart, F_SETFL, flags | O_NONBLOCK);

	struct sched_param param;
	param.sched_priority = SCHED_PRIORITY_MAX - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, receive_thread, &uart);
	return thread;
}
