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

#include "util.h"
#include "orb_topics.h"

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
orb_advert_t pub_hil_global_pos = -1;
orb_advert_t pub_hil_attitude = -1;

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
				warnx("terminating...");
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

		f.timestamp = hrt_absolute_time();
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

		vicon_position.x = pos.x;
		vicon_position.y = pos.y;
		vicon_position.z = pos.z;

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

	prctl(PR_SET_NAME, "mavlink_onboard_rcv", getpid());

	struct pollfd fds[] = { { .fd = uart_fd, .events = POLLIN } };

	ssize_t nread = 0;

	while (!thread_should_exit) {
		if (poll(fds, 1, timeout) > 0) {
			if (nread < sizeof(buf)) {
				/* to avoid reading very small chunks wait for data before reading */
				usleep(1000);
			}

			/* non-blocking read. read may return negative values */
			nread = read(uart_fd, buf, sizeof(buf));

			/* if read failed, this loop won't execute */
			for (ssize_t i = 0; i < nread; i++) {
				if (mavlink_parse_char(chan, buf[i], &msg, &status)) {
					/* handle generic messages and commands */
					handle_message(&msg);
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

	struct sched_param param;
	param.sched_priority = SCHED_PRIORITY_MAX - 40;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, receive_thread, &uart);
	return thread;
}
