/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file mavlink.c
 * MAVLink 1.0 protocol implementation.
 */

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
#include <arch/board/up_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/ardrone_control.h>
#include <uORB/topics/fixedwing_control.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/ardrone_motors_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_global_position_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <systemlib/param/param.h>

#include "waypoints.h"
#include "mavlink_log.h"

/* define MAVLink specific parameters */
PARAM_DEFINE_INT32(MAV_SYS_ID, 1);
PARAM_DEFINE_INT32(MAV_COMP_ID, 50);
PARAM_DEFINE_INT32(MAV_TYPE, MAV_TYPE_GENERIC);

__EXPORT int mavlink_main(int argc, char *argv[]);
int mavlink_thread_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int mavlink_task;

/* terminate MAVLink on user request - disabled by default */
static bool mavlink_link_termination_allowed = false;

mavlink_system_t mavlink_system = {100, 50, MAV_TYPE_FIXED_WING, 0, 0, 0}; // System ID, 1-255, Component/Subsystem ID, 1-255
static uint8_t chan = MAVLINK_COMM_0;
static mavlink_status_t status;

/* pthreads */
static pthread_t receive_thread;
static pthread_t uorb_receive_thread;

/* Allocate storage space for waypoints */
mavlink_wpm_storage wpm_s;

/** Global position */
static struct vehicle_global_position_s global_pos;

/** Local position */
static struct vehicle_local_position_s local_pos;

/** Vehicle status */
static struct vehicle_status_s v_status;

/** RC channels */
static struct rc_channels_s rc;

/* HIL publishers */
static orb_advert_t pub_hil_attitude = -1;

/** HIL attitude */
static struct vehicle_attitude_s hil_attitude;

static struct vehicle_global_position_s hil_global_pos;

static struct fixedwing_control_s fw_control;

static struct ardrone_motors_setpoint_s ardrone_motors;

static struct vehicle_command_s vcmd;

static orb_advert_t pub_hil_global_pos = -1;
static orb_advert_t ardrone_motors_pub = -1;
static orb_advert_t cmd_pub = -1;
static int local_pos_sub = -1;
static orb_advert_t flow_pub = -1;
static orb_advert_t global_position_setpoint_pub = -1;
static orb_advert_t local_position_setpoint_pub = -1;
static bool mavlink_hil_enabled = false;

static char mavlink_message_string[51] = {0};

/* interface mode */
static enum {
	MAVLINK_INTERFACE_MODE_OFFBOARD,
	MAVLINK_INTERFACE_MODE_ONBOARD
} mavlink_link_mode = MAVLINK_INTERFACE_MODE_OFFBOARD;

static struct mavlink_subscriptions {
	int sensor_sub;
	int att_sub;
	int global_pos_sub;
	int act_0_sub;
	int act_1_sub;
	int act_2_sub;
	int act_3_sub;
	int gps_sub;
	int man_control_sp_sub;
	bool initialized;
} mavlink_subs = {
	.sensor_sub = 0,
	.att_sub = 0,
	.global_pos_sub = 0,
	.act_0_sub = 0,
	.act_1_sub = 0,
	.act_2_sub = 0,
	.act_3_sub = 0,
	.gps_sub = 0,
	.man_control_sp_sub = 0,
	.initialized = false
};


/* 3: Define waypoint helper functions */
void mavlink_wpm_send_message(mavlink_message_t *msg);
void mavlink_wpm_send_gcs_string(const char *string);
uint64_t mavlink_wpm_get_system_timestamp(void);
int mavlink_missionlib_send_message(mavlink_message_t *msg);
int mavlink_missionlib_send_gcs_string(const char *string);
uint64_t mavlink_missionlib_get_system_timestamp(void);

void handleMessage(mavlink_message_t *msg);

/**
 * Enable / disable Hardware in the Loop simulation mode.
 */
int set_hil_on_off(bool hil_enabled);

/**
 * Translate the custom state into standard mavlink modes and state.
 */
void get_mavlink_mode_and_state(const struct vehicle_status_s *c_status, uint8_t *mavlink_state, uint8_t *mavlink_mode);

int mavlink_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb);

/* 4: Include waypoint protocol */
#include "waypoints.h"
mavlink_wpm_storage *wpm;


#include "mavlink_parameters.h"

/**
 * Print the usage
 */
static void usage(const char *reason);

static uint8_t missionlib_msg_buf[MAVLINK_MAX_PACKET_LEN];

int mavlink_missionlib_send_message(mavlink_message_t *msg)
{
	uint16_t len = mavlink_msg_to_send_buffer(missionlib_msg_buf, msg);
	int writelen = write(uart, missionlib_msg_buf, len);
	if (writelen != len) {
		return 1;
	} else {
		return 0;
	}
}

int mavlink_missionlib_send_gcs_string(const char *string)
{
	const int len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN;
	mavlink_statustext_t statustext;
	int i = 0;

	while (i < len - 1) {
		statustext.text[i] = string[i];

		if (string[i] == '\0')
			break;

		i++;
	}

	if (i > 1) {
		/* Enforce null termination */
		statustext.text[i] = '\0';
		mavlink_message_t msg;

		mavlink_msg_statustext_encode(mavlink_system.sysid, mavlink_system.compid, &msg, &statustext);
		return mavlink_missionlib_send_message(&msg);
	} else {
		return 1;
	}
}

/**
 * Get system time since boot in microseconds
 *
 * @return the system time since boot in microseconds
 */
uint64_t mavlink_missionlib_get_system_timestamp()
{
	return hrt_absolute_time();
}

/**
 * This callback is executed each time a waypoint changes.
 *
 * It publishes the vehicle_global_position_setpoint_s or the
 * vehicle_local_position_setpoint_s topic, depending on the type of waypoint
 */
extern void mavlink_missionlib_current_waypoint_changed(uint16_t index, float param1,
		float param2, float param3, float param4, float param5_lat_x,
		float param6_lon_y, float param7_alt_z, uint8_t frame, uint16_t command)
{
	char buf[50] = {0};

	/* Update controller setpoints */
	if (frame == (int)MAV_FRAME_GLOBAL) {
		/* global, absolute waypoint */
		struct vehicle_global_position_setpoint_s sp;
		sp.lat = param5_lat_x * 1e7f;
		sp.lon = param6_lon_y * 1e7f;
		sp.altitude = param7_alt_z;
		sp.altitude_is_relative = false;
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;
		/* Initialize publication if necessary */
		if (global_position_setpoint_pub < 0) {
			global_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_global_position_setpoint), &sp);
		} else {
			orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &sp);
		}
		sprintf(buf, "[mp] WP#%i lat: % 3.6f/lon % 3.6f/alt % 4.6f/hdg %3.4f\n", (int)index, (double)param5_lat_x, (double)param6_lon_y, (double)param7_alt_z, (double)param4);

	} else if (frame == (int)MAV_FRAME_GLOBAL_RELATIVE_ALT) {
		/* global, relative alt (in relation to HOME) waypoint */
		struct vehicle_global_position_setpoint_s sp;
		sp.lat = param5_lat_x * 1e7f;
		sp.lon = param6_lon_y * 1e7f;
		sp.altitude = param7_alt_z;
		sp.altitude_is_relative = true;
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;
		/* Initialize publication if necessary */
		if (global_position_setpoint_pub < 0) {
			global_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_global_position_setpoint), &sp);
		} else {
			orb_publish(ORB_ID(vehicle_global_position_setpoint), global_position_setpoint_pub, &sp);
		}
		sprintf(buf, "[mp] WP#%i (lat: %f/lon %f/rel alt %f/hdg %f\n", (int)index, (double)param5_lat_x, (double)param6_lon_y, (double)param7_alt_z, (double)param4);

	} else if (frame == (int)MAV_FRAME_LOCAL_ENU || frame == (int)MAV_FRAME_LOCAL_NED) {
		/* local, absolute waypoint */
		struct vehicle_local_position_setpoint_s sp;
		sp.x = param5_lat_x;
		sp.y = param6_lon_y;
		sp.z = param7_alt_z;
		sp.yaw = (param4 / 180.0f) * M_PI_F - M_PI_F;
		/* Initialize publication if necessary */
		if (local_position_setpoint_pub < 0) {
			local_position_setpoint_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &sp);
		} else {
			orb_publish(ORB_ID(vehicle_local_position_setpoint), local_position_setpoint_pub, &sp);
		}
		sprintf(buf, "[mp] WP#%i (x: %f/y %f/z %f/hdg %f\n", (int)index, (double)param5_lat_x, (double)param6_lon_y, (double)param7_alt_z, (double)param4);
	}
	
	mavlink_missionlib_send_gcs_string(buf);
	printf("%s\n", buf);
	//printf("[mavlink mp] new setpoint\n");//: frame: %d, lat: %d, lon: %d, alt: %d, yaw: %d\n", frame, param5_lat_x*1000, param6_lon_y*1000, param7_alt_z*1000, param4*1000);
}



int set_hil_on_off(bool hil_enabled)
{
	int ret = OK;

	/* Enable HIL */
	if (hil_enabled && !mavlink_hil_enabled) {

		//printf("\n HIL ON \n");

		(void)close(pub_hil_attitude);
		(void)close(pub_hil_global_pos);

		/* Advertise topics */
		pub_hil_attitude = orb_advertise(ORB_ID(vehicle_attitude), &hil_attitude);
		pub_hil_global_pos = orb_advertise(ORB_ID(vehicle_global_position), &hil_global_pos);

		printf("\n pub_hil_attitude :%i\n", pub_hil_attitude);
		printf("\n pub_hil_global_pos :%i\n", pub_hil_global_pos);

		if (pub_hil_attitude > 0 && pub_hil_global_pos > 0) {
			mavlink_hil_enabled = true;

		} else {
			ret = ERROR;
		}
	}

	if (!hil_enabled && mavlink_hil_enabled) {
		mavlink_hil_enabled = false;
		(void)close(pub_hil_attitude);
		(void)close(pub_hil_global_pos);

	} else {
		ret = ERROR;
	}

	return ret;
}

void get_mavlink_mode_and_state(const struct vehicle_status_s *c_status, uint8_t *mavlink_state, uint8_t *mavlink_mode)
{
	/* reset MAVLink mode bitfield */
	*mavlink_mode = 0;

	/* set mode flags independent of system state */
	if (c_status->flag_control_manual_enabled) {
		*mavlink_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	}

	/* set arming state */
	if (c_status->flag_system_armed) {
		*mavlink_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	} else {
		*mavlink_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	}

	switch (c_status->state_machine) {
	case SYSTEM_STATE_PREFLIGHT:
		if (c_status->flag_preflight_gyro_calibration ||
		    c_status->flag_preflight_mag_calibration ||
		    c_status->flag_preflight_accel_calibration) {
			*mavlink_state = MAV_STATE_CALIBRATING;
		} else {
			*mavlink_state = MAV_STATE_UNINIT;
		}
		break;

	case SYSTEM_STATE_STANDBY:
		*mavlink_state = MAV_STATE_STANDBY;
		break;

	case SYSTEM_STATE_GROUND_READY:
		*mavlink_state = MAV_STATE_ACTIVE;
		break;

	case SYSTEM_STATE_MANUAL:
		*mavlink_state = MAV_STATE_ACTIVE;
		*mavlink_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		break;

	case SYSTEM_STATE_STABILIZED:
		*mavlink_state = MAV_STATE_ACTIVE;
		*mavlink_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
		break;

	case SYSTEM_STATE_AUTO:
		*mavlink_state = MAV_STATE_ACTIVE;
		*mavlink_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
		break;

	case SYSTEM_STATE_MISSION_ABORT:
		*mavlink_state = MAV_STATE_EMERGENCY;
		break;

	case SYSTEM_STATE_EMCY_LANDING:
		*mavlink_state = MAV_STATE_EMERGENCY;
		break;

	case SYSTEM_STATE_EMCY_CUTOFF:
		*mavlink_state = MAV_STATE_EMERGENCY;
		break;

	case SYSTEM_STATE_GROUND_ERROR:
		*mavlink_state = MAV_STATE_EMERGENCY;
		break;

	case SYSTEM_STATE_REBOOT:
		*mavlink_state = MAV_STATE_POWEROFF;
		break;
	}

}

/**
 * Receive data from UART.
 */
static void *receiveloop(void *arg)
{
	int uart_fd = *((int*)arg);

	const int timeout = 1000;
	uint8_t ch;

	mavlink_message_t msg;

	prctl(PR_SET_NAME, "mavlink uart rcv", getpid());

	while (!thread_should_exit) {

		struct pollfd fds[] = { { .fd = uart_fd, .events = POLLIN } };

		if (poll(fds, 1, timeout) > 0) {
			/* non-blocking read until buffer is empty */
			int nread = 0;

			do {
				nread = read(uart_fd, &ch, 1);

				if (mavlink_parse_char(chan, ch, &msg, &status)) { //parse the char
					/* handle generic messages and commands */
					handleMessage(&msg);

					/* Handle packet with waypoint component */
					mavlink_wpm_message_handler(&msg, &global_pos, &local_pos);

					/* Handle packet with parameter component */
					mavlink_pm_message_handler(MAVLINK_COMM_0, &msg);
				}
			} while (nread > 0);
		}
	}

	return NULL;
}

static int set_mavlink_interval_limit(struct mavlink_subscriptions *subs, int mavlink_msg_id, int min_interval)
{
	int ret = OK;

	switch (mavlink_msg_id) {
		case MAVLINK_MSG_ID_SCALED_IMU:
			/* senser sub triggers scaled IMU */
			if (subs->sensor_sub) orb_set_interval(subs->sensor_sub, min_interval);
			break;
		case MAVLINK_MSG_ID_RAW_IMU:
			/* senser sub triggers RAW IMU */
			if (subs->sensor_sub) orb_set_interval(subs->sensor_sub, min_interval);
			break;
		case MAVLINK_MSG_ID_ATTITUDE:
			/* attitude sub triggers attitude */
			if (subs->att_sub) orb_set_interval(subs->att_sub, min_interval);
			break;
		case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
			/* actuator_outputs triggers this message */
			if (subs->act_0_sub) orb_set_interval(subs->act_0_sub, min_interval);
			if (subs->act_1_sub) orb_set_interval(subs->act_1_sub, min_interval);
			if (subs->act_2_sub) orb_set_interval(subs->act_2_sub, min_interval);
			if (subs->act_3_sub) orb_set_interval(subs->act_3_sub, min_interval);
			break;
		case MAVLINK_MSG_ID_MANUAL_CONTROL:
			/* manual_control_setpoint triggers this message */
			if (subs->man_control_sp_sub) orb_set_interval(subs->man_control_sp_sub, min_interval);
		default:
			/* not found */
			ret = ERROR;
			break;
	}

	return ret;
}

/**
 * Listen for uORB topics and send via MAVLink.
 *
 * This pthread performs a blocking wait on selected
 * uORB topics and sends them via MAVLink to other
 * vehicles or a ground control station.
 */
static void *uorb_receiveloop(void *arg)
{
	/* obtain reference to task's subscriptions */
	struct mavlink_subscriptions *subs = (struct mavlink_subscriptions *)arg;

	/* Set thread name */
	prctl(PR_SET_NAME, "mavlink orb rcv", getpid());


	/* --- IMPORTANT: DEFINE NUMBER OF ORB STRUCTS TO WAIT FOR HERE --- */
	/* number of messages */
	const ssize_t fdsc = 25;
	/* Sanity check variable and index */
	ssize_t fdsc_count = 0;
	/* file descriptors to wait for */
	struct pollfd fds[fdsc];


	union {
		struct sensor_combined_s raw;
		struct vehicle_attitude_s att;
		struct vehicle_gps_position_s gps;
		struct ardrone_control_s ar_control;
		struct vehicle_local_position_setpoint_s local_sp;
		struct vehicle_global_position_setpoint_s global_sp;
		struct vehicle_attitude_setpoint_s att_sp;
		struct actuator_outputs_s act_outputs;
		struct manual_control_setpoint_s man_control;
	} buf;

	/* --- SENSORS RAW VALUE --- */
	/* subscribe to ORB for sensors raw */
	subs->sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	fds[fdsc_count].fd = subs->sensor_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE VALUE --- */
	/* subscribe to ORB for attitude */
	subs->att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	orb_set_interval(subs->att_sub, 100);
	fds[fdsc_count].fd = subs->att_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GPS VALUE --- */
	/* subscribe to ORB for attitude */
	subs->gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	orb_set_interval(subs->gps_sub, 1000);	/* 1Hz updates */
	fds[fdsc_count].fd = subs->gps_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	// /* --- ARDRONE CONTROL --- */
	// /* subscribe to ORB for AR.Drone controller outputs */
	// int ar_sub = orb_subscribe(ORB_ID(ardrone_control));
	// orb_set_interval(ar_sub, 200);		/* 5Hz updates */
	// fds[fdsc_count].fd = ar_sub;
	// fds[fdsc_count].events = POLLIN;
	// fdsc_count++;

	/* --- SYSTEM STATE --- */
	/* struct already globally allocated */
	/* subscribe to topic */
	int status_sub = orb_subscribe(ORB_ID(vehicle_status));
	orb_set_interval(status_sub, 300);	/* max 3.33 Hz updates */
	fds[fdsc_count].fd = status_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- RC CHANNELS VALUE --- */
	/* struct already globally allocated */
	/* subscribe to ORB for global position */
	int rc_sub = orb_subscribe(ORB_ID(rc_channels));
	orb_set_interval(rc_sub, 100);		/* 10Hz updates */
	fds[fdsc_count].fd = rc_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- FIXED WING CONTROL VALUE --- */
	/* struct already globally allocated */
	/* subscribe to ORB for fixed wing control */
	int fw_sub = orb_subscribe(ORB_ID(fixedwing_control));
	orb_set_interval(fw_sub, 50);		/* 20 Hz updates */
	fds[fdsc_count].fd = fw_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GLOBAL POS VALUE --- */
	/* struct already globally allocated and topic already subscribed */
	fds[fdsc_count].fd = subs->global_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- LOCAL POS VALUE --- */
	/* struct and topic already globally subscribed */
	fds[fdsc_count].fd = local_pos_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- GLOBAL SETPOINT VALUE --- */
	/* subscribe to ORB for local setpoint */
	/* struct already allocated */
	int spg_sub = orb_subscribe(ORB_ID(vehicle_global_position_setpoint));
	orb_set_interval(spg_sub, 2000);	/* 0.5 Hz updates */
	fds[fdsc_count].fd = spg_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- LOCAL SETPOINT VALUE --- */
	/* subscribe to ORB for local setpoint */
	/* struct already allocated */
	int spl_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	orb_set_interval(spl_sub, 2000);	/* 0.5 Hz updates */
	fds[fdsc_count].fd = spl_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* --- ATTITUDE SETPOINT VALUE --- */
	/* subscribe to ORB for attitude setpoint */
	/* struct already allocated */
	int spa_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	orb_set_interval(spa_sub, 2000);	/* 0.5 Hz updates */
	fds[fdsc_count].fd = spa_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/** --- ACTUATOR OUTPUTS --- */
	subs->act_0_sub = orb_subscribe(ORB_ID(actuator_outputs_0));
	fds[fdsc_count].fd = subs->act_0_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;
	subs->act_1_sub = orb_subscribe(ORB_ID(actuator_outputs_1));
	fds[fdsc_count].fd = subs->act_1_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;
	subs->act_2_sub = orb_subscribe(ORB_ID(actuator_outputs_2));
	fds[fdsc_count].fd = subs->act_2_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;
	subs->act_3_sub = orb_subscribe(ORB_ID(actuator_outputs_3));
	fds[fdsc_count].fd = subs->act_3_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/** --- MAPPED MANUAL CONTROL INPUTS --- */
	subs->man_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	fds[fdsc_count].fd = subs->man_control_sp_sub;
	fds[fdsc_count].events = POLLIN;
	fdsc_count++;

	/* all subscriptions initialized, return success */
	subs->initialized = true;

	unsigned int sensors_raw_counter = 0;
	unsigned int attitude_counter = 0;
	unsigned int gps_counter = 0;

	/* WARNING: If you get the error message below,
	 * then the number of registered messages (fdsc)
	 * differs from the number of messages in the above list.
	 */
	if (fdsc_count > fdsc) {
		fprintf(stderr, "[mavlink] WARNING: Not enough space for poll fds allocated. Check %s:%d.\n", __FILE__, __LINE__);
		fdsc_count = fdsc;
	}

	/*
	 * set up poll to block for new data,
	 * wait for a maximum of 1000 ms (1 second)
	 */
	const int timeout = 1000;

	while (!thread_should_exit) {

		int poll_ret = poll(fds, fdsc_count, timeout);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* XXX this means none of our providers is giving us data - might be an error? */
		} else if (poll_ret < 0) {
			/* XXX this is seriously bad - should be an emergency */
		} else {

			int ifds = 0;

			/* --- SENSORS RAW VALUE --- */
			if (fds[ifds++].revents & POLLIN) {

				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(sensor_combined), subs->sensor_sub, &buf.raw);

				/* send raw imu data */
				mavlink_msg_raw_imu_send(MAVLINK_COMM_0, buf.raw.timestamp, buf.raw.accelerometer_raw[0], buf.raw.accelerometer_raw[1], buf.raw.accelerometer_raw[2], buf.raw.gyro_raw[0], buf.raw.gyro_raw[1], buf.raw.gyro_raw[2], buf.raw.magnetometer_raw[0], buf.raw.magnetometer_raw[1], buf.raw.magnetometer_raw[2]);
				/* send scaled imu data (m/s^2 accelerations scaled back to milli-g) */
				mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, buf.raw.timestamp, buf.raw.accelerometer_m_s2[0] * 101.936799f, buf.raw.accelerometer_m_s2[1] * 101.936799f, buf.raw.accelerometer_m_s2[2] * 101.936799f, buf.raw.gyro_rad_s[0] * 1000, buf.raw.gyro_rad_s[1] * 1000, buf.raw.gyro_rad_s[2] * 1000, buf.raw.magnetometer_ga[0] * 1000, buf.raw.magnetometer_ga[1] * 1000, buf.raw.magnetometer_ga[2] * 1000);
				/* send pressure */
				mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, buf.raw.timestamp / 1000, buf.raw.baro_pres_mbar, buf.raw.baro_alt_meter, buf.raw.baro_temp_celcius * 100);

				sensors_raw_counter++;
			}

			/* --- ATTITUDE VALUE --- */
			if (fds[ifds++].revents & POLLIN) {

				/* copy attitude data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude), subs->att_sub, &buf.att);

				/* send sensor values */
				mavlink_msg_attitude_send(MAVLINK_COMM_0, buf.att.timestamp / 1000, buf.att.roll, buf.att.pitch, buf.att.yaw, buf.att.rollspeed, buf.att.pitchspeed, buf.att.yawspeed);

				attitude_counter++;
			}

			/* --- GPS VALUE --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy gps data into local buffer */
				orb_copy(ORB_ID(vehicle_gps_position), subs->gps_sub, &buf.gps);
				/* GPS position */
				mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, buf.gps.timestamp, buf.gps.fix_type, buf.gps.lat, buf.gps.lon, buf.gps.alt, buf.gps.eph, buf.gps.epv, buf.gps.vel, buf.gps.cog, buf.gps.satellites_visible);

				if (buf.gps.satellite_info_available && (gps_counter % 4 == 0)) {
					mavlink_msg_gps_status_send(MAVLINK_COMM_0, buf.gps.satellites_visible, buf.gps.satellite_prn, buf.gps.satellite_used, buf.gps.satellite_elevation, buf.gps.satellite_azimuth, buf.gps.satellite_snr);
				}

				gps_counter++;
			}

			// /* --- ARDRONE CONTROL OUTPUTS --- */
			// if (fds[ifds++].revents & POLLIN) {
			// 	/* copy ardrone control data into local buffer */
			// 	orb_copy(ORB_ID(ardrone_control), ar_sub, &buf.ar_control);
			// 	uint64_t timestamp    = buf.ar_control.timestamp;
			// 	float setpoint_roll   = buf.ar_control.setpoint_attitude[0];
			// 	float setpoint_pitch  = buf.ar_control.setpoint_attitude[1];
			// 	float setpoint_yaw    = buf.ar_control.setpoint_attitude[2];
			// 	float setpoint_thrust = buf.ar_control.setpoint_thrust_cast;

			// 	float control_roll = buf.ar_control.attitude_control_output[0];
			// 	float control_pitch = buf.ar_control.attitude_control_output[1];
			// 	float control_yaw = buf.ar_control.attitude_control_output[2];

			// 	mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0, timestamp / 1000, setpoint_roll, setpoint_pitch, setpoint_yaw, setpoint_thrust);
			// 	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, timestamp / 1000, "cl.roll", control_roll);
			// 	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, timestamp / 1000, "cl.pitch", control_pitch);
			// 	mavlink_msg_named_value_float_send(MAVLINK_COMM_0, timestamp / 1000, "cl.yaw", control_yaw);
			// }

			/* --- SYSTEM STATUS --- */
			if (fds[ifds++].revents & POLLIN) {
				/* immediately communicate state changes back to user */
				orb_copy(ORB_ID(vehicle_status), status_sub, &v_status);
				/* enable or disable HIL */
				set_hil_on_off(v_status.flag_hil_enabled);

				/* translate the current syste state to mavlink state and mode */
				uint8_t mavlink_state = 0;
				uint8_t mavlink_mode = 0;
				get_mavlink_mode_and_state(&v_status, &mavlink_state, &mavlink_mode);

				/* send heartbeat */
				mavlink_msg_heartbeat_send(chan, mavlink_system.type, MAV_AUTOPILOT_PX4, mavlink_mode, v_status.state_machine, mavlink_state);
			}

			/* --- RC CHANNELS --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy rc channels into local buffer */
				orb_copy(ORB_ID(rc_channels), rc_sub, &rc);
				/* Channels are sent in MAVLink main loop at a fixed interval */
				// TODO decide where to send channels
			}

			/* --- FIXED WING CONTROL CHANNELS --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy fixed wing control into local buffer */
				orb_copy(ORB_ID(fixedwing_control), fw_sub, &fw_control);
				/* send control output via MAVLink */
				mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0, fw_control.timestamp / 1000, fw_control.attitude_control_output[0],
					fw_control.attitude_control_output[1], fw_control.attitude_control_output[2],
					fw_control.attitude_control_output[3]);

				/* Only send in HIL mode */
				if (v_status.flag_hil_enabled) {
					/* Send the desired attitude from RC or from the autonomous controller */
					// XXX it should not depend on a RC setting, but on a system_state value

					float roll_ail, pitch_elev, throttle, yaw_rudd;

					if (rc.chan[rc.function[OVERRIDE]].scale < 2000) {

						//orb_copy(ORB_ID(fixedwing_control), fixed_wing_control_sub, &fixed_wing_control);
						roll_ail = fw_control.attitude_control_output[ROLL];
						pitch_elev = fw_control.attitude_control_output[PITCH];
						throttle = fw_control.attitude_control_output[THROTTLE];
						yaw_rudd = fw_control.attitude_control_output[YAW];

					} else {

						roll_ail = rc.chan[rc.function[ROLL]].scale;
						pitch_elev = rc.chan[rc.function[PITCH]].scale;
						throttle = rc.chan[rc.function[THROTTLE]].scale;
						yaw_rudd = rc.chan[rc.function[YAW]].scale;
					}

					/* hacked HIL implementation in order for the APM Planner to work
					 * (correct cmd: mavlink_msg_hil_controls_send())
					 */

					mavlink_msg_rc_channels_scaled_send(chan,
									    hrt_absolute_time(),
									    0, // port 0
									    roll_ail,
									    pitch_elev,
									    throttle,
									    yaw_rudd,
									    0,
									    0,
									    0,
									    0,
									    1 /*rssi=1*/);

					/* correct command duplicate */
					mavlink_msg_hil_controls_send(chan,
										hrt_absolute_time(),
										roll_ail,
										pitch_elev,
										yaw_rudd,
										throttle,
										0,
										0,
										0,
										0,
										32,	/* HIL_MODE */
										0);
				}
			}

			/* --- VEHICLE GLOBAL POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy global position data into local buffer */
				orb_copy(ORB_ID(vehicle_global_position), subs->global_pos_sub, &global_pos);
				uint64_t timestamp = global_pos.timestamp;
				int32_t lat = global_pos.lat;
				int32_t lon = global_pos.lon;
				int32_t alt = (int32_t)(global_pos.alt*1000);
				int32_t relative_alt = (int32_t)(global_pos.relative_alt * 1000.0f);
				int16_t vx = (int16_t)(global_pos.vx * 100.0f);
				int16_t vy = (int16_t)(global_pos.vy * 100.0f);
				int16_t vz = (int16_t)(global_pos.vz * 100.0f);
				/* heading in degrees * 10, from 0 to 36.000) */
				uint16_t hdg = (global_pos.hdg / M_PI_F) * (180.0f * 10.0f) + (180.0f * 10.0f);

				mavlink_msg_global_position_int_send(MAVLINK_COMM_0, timestamp / 1000, lat, lon, alt, relative_alt, vx, vy, vz, hdg);
			}

			/* --- VEHICLE LOCAL POSITION --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);
				mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, local_pos.timestamp / 1000, local_pos.x, local_pos.y, local_pos.z, local_pos.vx, local_pos.vy, local_pos.vz);
			}

			/* --- VEHICLE GLOBAL SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(vehicle_global_position_setpoint), spg_sub, &buf.global_sp);
				uint8_t coordinate_frame = MAV_FRAME_GLOBAL;
				if (buf.global_sp.altitude_is_relative) coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
				mavlink_msg_global_position_setpoint_int_send(MAVLINK_COMM_0, coordinate_frame, buf.global_sp.lat, buf.global_sp.lon, buf.global_sp.altitude, buf.global_sp.yaw);
			}

			/* --- VEHICLE LOCAL SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(vehicle_local_position_setpoint), spl_sub, &buf.local_sp);
				mavlink_msg_local_position_setpoint_send(MAVLINK_COMM_0, MAV_FRAME_LOCAL_NED, buf.local_sp.x, buf.local_sp.y, buf.local_sp.z, buf.local_sp.yaw);
			}

			/* --- VEHICLE ATTITUDE SETPOINT --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), spa_sub, &buf.att_sp);
				mavlink_msg_roll_pitch_yaw_thrust_setpoint_send(MAVLINK_COMM_0, buf.att_sp.timestamp/1000, buf.att_sp.roll_body, buf.att_sp.pitch_body, buf.att_sp.yaw_body, buf.att_sp.thrust);
			}

			/* --- ACTUATOR OUTPUTS 0 --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy actuator data into local buffer */
				orb_copy(ORB_ID(actuator_outputs_0), subs->act_0_sub, &buf.act_outputs);
				mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					0 /* port 0 */,
					buf.act_outputs.output[0],
					buf.act_outputs.output[1],
					buf.act_outputs.output[2],
					buf.act_outputs.output[3],
					buf.act_outputs.output[4],
					buf.act_outputs.output[5],
					buf.act_outputs.output[6],
					buf.act_outputs.output[7]);
				// if (NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) {
				// 	mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
				// 	1 /* port 1 */,
				// 	buf.act_outputs.output[ 8],
				// 	buf.act_outputs.output[ 9],
				// 	buf.act_outputs.output[10],
				// 	buf.act_outputs.output[11],
				// 	buf.act_outputs.output[12],
				// 	buf.act_outputs.output[13],
				// 	buf.act_outputs.output[14],
				// 	buf.act_outputs.output[15]);
				// }
			}

			/* --- ACTUATOR OUTPUTS 1 --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy actuator data into local buffer */
				orb_copy(ORB_ID(actuator_outputs_1), subs->act_1_sub, &buf.act_outputs);
				mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					(NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) ? 2 : 1 /* port 2 or 1*/,
					buf.act_outputs.output[0],
					buf.act_outputs.output[1],
					buf.act_outputs.output[2],
					buf.act_outputs.output[3],
					buf.act_outputs.output[4],
					buf.act_outputs.output[5],
					buf.act_outputs.output[6],
					buf.act_outputs.output[7]);
				if (NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) {
					mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					3 /* port 3 */,
					buf.act_outputs.output[ 8],
					buf.act_outputs.output[ 9],
					buf.act_outputs.output[10],
					buf.act_outputs.output[11],
					buf.act_outputs.output[12],
					buf.act_outputs.output[13],
					buf.act_outputs.output[14],
					buf.act_outputs.output[15]);
				}
			}

			/* --- ACTUATOR OUTPUTS 2 --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy actuator data into local buffer */
				orb_copy(ORB_ID(actuator_outputs_2), subs->act_2_sub, &buf.act_outputs);
				mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					(NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) ? 4 : 2 /* port 4 or 2 */,
					buf.act_outputs.output[0],
					buf.act_outputs.output[1],
					buf.act_outputs.output[2],
					buf.act_outputs.output[3],
					buf.act_outputs.output[4],
					buf.act_outputs.output[5],
					buf.act_outputs.output[6],
					buf.act_outputs.output[7]);
				if (NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) {
					mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					5 /* port 5 */,
					buf.act_outputs.output[ 8],
					buf.act_outputs.output[ 9],
					buf.act_outputs.output[10],
					buf.act_outputs.output[11],
					buf.act_outputs.output[12],
					buf.act_outputs.output[13],
					buf.act_outputs.output[14],
					buf.act_outputs.output[15]);
				}
			}

			/* --- ACTUATOR OUTPUTS 3 --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy actuator data into local buffer */
				orb_copy(ORB_ID(actuator_outputs_3), subs->act_3_sub, &buf.act_outputs);
				mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					(NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) ? 6 : 3 /* port 6 or 3 */,
					buf.act_outputs.output[0],
					buf.act_outputs.output[1],
					buf.act_outputs.output[2],
					buf.act_outputs.output[3],
					buf.act_outputs.output[4],
					buf.act_outputs.output[5],
					buf.act_outputs.output[6],
					buf.act_outputs.output[7]);
				if (NUM_ACTUATOR_OUTPUTS > 8 && NUM_ACTUATOR_OUTPUTS <= 16) {
					mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, hrt_absolute_time(),
					7 /* port 7 */,
					buf.act_outputs.output[ 8],
					buf.act_outputs.output[ 9],
					buf.act_outputs.output[10],
					buf.act_outputs.output[11],
					buf.act_outputs.output[12],
					buf.act_outputs.output[13],
					buf.act_outputs.output[14],
					buf.act_outputs.output[15]);
				}
			}

			/* --- MAPPED MANUAL CONTROL INPUTS --- */
			if (fds[ifds++].revents & POLLIN) {
				/* copy local position data into local buffer */
				orb_copy(ORB_ID(manual_control_setpoint), subs->man_control_sp_sub, &buf.man_control);
				mavlink_msg_manual_control_send(MAVLINK_COMM_0, mavlink_system.sysid, buf.man_control.roll,
					buf.man_control.pitch, buf.man_control.yaw, buf.man_control.throttle, 1, 1, 1, 1);
			}
		}
	}

	return NULL;
}

/****************************************************************************
 * MAVLink text message logger
 ****************************************************************************/

static int	mavlink_dev_ioctl(struct file *filep, int cmd, unsigned long arg);

static const struct file_operations mavlink_fops = {
	.ioctl = mavlink_dev_ioctl
};

static int
mavlink_dev_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	static unsigned int total_counter = 0;

	switch (cmd) {
	case (int)MAVLINK_IOC_SEND_TEXT_INFO:
	case (int)MAVLINK_IOC_SEND_TEXT_CRITICAL:
	case (int)MAVLINK_IOC_SEND_TEXT_EMERGENCY: {
			const char *txt = (const char *)arg;
			strncpy(mavlink_message_string, txt, 51);
			total_counter++;
			return OK;
		}

	default:
		return ENOTTY;
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void handleMessage(mavlink_message_t *msg)
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
		}
		/* create command */
		orb_publish(ORB_ID(vehicle_command), cmd_pub, &vcmd);
	}

	/* Handle quadrotor motor setpoints */

	if (msg->msgid == MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT) {
		mavlink_set_quad_motors_setpoint_t quad_motors_setpoint;
		mavlink_msg_set_quad_motors_setpoint_decode(msg, &quad_motors_setpoint);
//		printf("got MAVLINK_MSG_ID_SET_QUAD_MOTORS_SETPOINT target_system=%u, sysid = %u\n", quad_motors_setpoint.target_system, mavlink_system.sysid);

		if (quad_motors_setpoint.target_system == mavlink_system.sysid) {
			ardrone_motors.motor_front_nw = quad_motors_setpoint.motor_front_nw;
			ardrone_motors.motor_right_ne = quad_motors_setpoint.motor_right_ne;
			ardrone_motors.motor_back_se = quad_motors_setpoint.motor_back_se;
			ardrone_motors.motor_left_sw = quad_motors_setpoint.motor_left_sw;

			ardrone_motors.counter++;
			ardrone_motors.timestamp = hrt_absolute_time();

			/* check if topic has to be advertised */
			if (ardrone_motors_pub <= 0) {
				ardrone_motors_pub = orb_advertise(ORB_ID(ardrone_motors_setpoint), &ardrone_motors);
			}
			/* Publish */
			orb_publish(ORB_ID(ardrone_motors_setpoint), ardrone_motors_pub, &ardrone_motors);
		}
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 */

	// printf("\n HIL ENABLED?: %s \n",(mavlink_hil_enabled)?"true":"false");

	if (mavlink_hil_enabled) {

		if (msg->msgid == MAVLINK_MSG_ID_HIL_STATE) {

			mavlink_hil_state_t hil_state;
			mavlink_msg_hil_state_decode(msg, &hil_state);

			//	printf("\n HILSTATE : \n LAT: %i \n LON: %i \n ALT: %i \n "
			//		"ROLL %i \n PITCH %i \n YAW %i \n"
			//		"ROLLSPEED: %i \n PITCHSPEED: %i \n, YAWSPEED: %i \n",
			//			hil_state.lat/1000000,	// 1e7
			//			hil_state.lon/1000000,	// 1e7
			//			hil_state.alt/1000,  // mm
			//			hil_state.roll, // float rad
			//			hil_state.pitch, // float rad
			//			hil_state.yaw, // float rad
			//			hil_state.rollspeed, // float rad/s
			//			hil_state.pitchspeed, // float rad/s
			//			hil_state.yawspeed); // float rad/s


			hil_global_pos.lat = hil_state.lat;
			hil_global_pos.lon = hil_state.lon;
			hil_global_pos.alt = hil_state.alt/1000;
			hil_global_pos.vx = hil_state.vx;
			hil_global_pos.vy = hil_state.vy;
			hil_global_pos.vz = hil_state.vz;

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
			hil_attitude.counter++;
			hil_attitude.timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(vehicle_attitude), pub_hil_attitude, &hil_attitude);
		}
	}
}

int mavlink_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
	/* process baud rate */
	int speed;

	switch (baudrate) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 921600: speed = B921600; break;

	default:
		fprintf(stderr, "[mavlink] ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600\n\n", baudrate);
		return -EINVAL;
	}

	/* open uart */
	printf("[mavlink] UART is %s, baudrate is %d\n", uart_name, baudrate);
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	if (strcmp(uart_name, "/dev/ttyACM0") != OK) {
		/* Back up the original uart configuration to restore it after exit */
		if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
			fprintf(stderr, "[mavlink] ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
			close(uart);
			return -1;
		}

		/* Fill the struct for the new configuration */
		tcgetattr(uart, &uart_config);

		/* Clear ONLCR flag (which appends a CR for every LF) */
		uart_config.c_oflag &= ~ONLCR;

		/* Set baud rate */
		if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
			fprintf(stderr, "[mavlink] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
			close(uart);
			return -1;
		}


		if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0) {
			fprintf(stderr, "[mavlink] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
			close(uart);
			return -1;
		}

	} else {
		*is_usb = true;
	}

	return uart;
}

/**
 * MAVLink Protocol main function.
 */
int mavlink_thread_main(int argc, char *argv[])
{
	wpm = &wpm_s;

	/* initialize global data structs */
	memset(&global_pos, 0, sizeof(global_pos));
	memset(&local_pos, 0, sizeof(local_pos));
	memset(&v_status, 0, sizeof(v_status));
	memset(&rc, 0, sizeof(rc));
	memset(&hil_attitude, 0, sizeof(hil_attitude));
	memset(&hil_global_pos, 0, sizeof(hil_global_pos));
	memset(&fw_control, 0, sizeof(fw_control));
	memset(&ardrone_motors, 0, sizeof(ardrone_motors));
	memset(&vcmd, 0, sizeof(vcmd));

	/* print welcome text */
	printf("[mavlink] MAVLink v1.0 serial interface starting..\n");

	/* reate the device node that's used for sending text log messages, etc. */
	register_driver(MAVLINK_LOG_DEVICE, &mavlink_fops, 0666, NULL);

	/* default values for arguments */
	char *uart_name = "/dev/ttyS1";
	int baudrate = 57600;

	/* read program arguments */
	int i;

	for (i = 1; i < argc; i++) { /* argv[0] is "mavlink" */

		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			usage("");
			return 0;
		} else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];
				i++;
			} else {
				usage("missing argument for device (-d)");
				return 1;
			}
		} else if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);
				i++;
			} else {
				usage("missing argument for baud rate (-b)");
				return 1;
			}
		} else if (strcmp(argv[i], "-e") == 0 || strcmp(argv[i], "--exit-allowed") == 0) {
			mavlink_link_termination_allowed = true;
		} else if (strcmp(argv[i], "-o") == 0 || strcmp(argv[i], "--onboard") == 0) {
			mavlink_link_mode = MAVLINK_INTERFACE_MODE_ONBOARD;
		} else {
			usage("out of order or invalid argument");
			return 1;
		}
	}

	struct termios uart_config_original;

	bool usb_uart;

	uart = mavlink_open_uart(baudrate, uart_name, &uart_config_original, &usb_uart);

	if (uart < 0) {
		printf("[mavlink] FAILED to open %s, terminating.\n", uart_name);
		goto exit_cleanup;
	}

	/* Flush UART */
	fflush(stdout);

	/* Initialize system properties */
	param_t param_system_id = param_find("MAV_SYS_ID");
	param_t param_component_id = param_find("MAV_COMP_ID");
	param_t param_system_type = param_find("MAV_TYPE");

	/* topics to subscribe globally */
	/* subscribe to ORB for global position */
	mavlink_subs.global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	orb_set_interval(mavlink_subs.global_pos_sub, 1000);	/* 1Hz active updates */
	/* subscribe to ORB for local position */
	local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_set_interval(local_pos_sub, 1000);	/* 1Hz active updates */


	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);
	pthread_attr_setstacksize(&receiveloop_attr, 2048);
	pthread_create(&receive_thread, &receiveloop_attr, receiveloop, &uart);

	pthread_attr_t uorb_attr;
	pthread_attr_init(&uorb_attr);
	/* Set stack size, needs more than 8000 bytes */
	pthread_attr_setstacksize(&uorb_attr, 8192);
	pthread_create(&uorb_receive_thread, &uorb_attr, uorb_receiveloop, &mavlink_subs);

	/* initialize waypoint manager */
	mavlink_wpm_init(wpm);

	uint16_t counter = 0;
	/* arm counter to go off immediately */
	int lowspeed_counter = 10;

	/* make sure all threads have registered their subscriptions */
	while (!mavlink_subs.initialized) {
		usleep(500);
	}

	/* all subscriptions are now active, set up initial guess about rate limits */
	if (baudrate >= 921600) {
		/* 500 Hz / 2 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SCALED_IMU, 2);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 2);
		/* 200 Hz / 5 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 5);
		/* 5 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 200);
	} else if (baudrate >= 460800) {
		/* 250 Hz / 4 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SCALED_IMU, 5);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 5);
		/* 50 Hz / 20 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 20);
		/* 2 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 500);
	} else if (baudrate >= 115200) {
		/* 50 Hz / 20 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SCALED_IMU, 50);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 50);
		/* 10 Hz / 100 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100);
		/* 1 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 1000);
	} else if (baudrate >= 57600) {
		/* 10 Hz / 100 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SCALED_IMU, 100);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 100);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 200);
		/* 0.2 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 5000);
	} else {
		/* very low baud rate, limit to 1 Hz / 1000 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SCALED_IMU, 1000);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 1000);
		/* 0.5 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2000);
		/* 0.1 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 10000);
	}

	thread_running = true;

	while (!thread_should_exit) {

		/* get local and global position */
		orb_copy(ORB_ID(vehicle_global_position), mavlink_subs.global_pos_sub, &global_pos);
		orb_copy(ORB_ID(vehicle_local_position), local_pos_sub, &local_pos);

		/* 1 Hz */
		if (lowspeed_counter == 10) {
			/* update system and component id */
			int32_t system_id;
			param_get(param_system_id, &system_id);
			if (system_id > 0 && system_id < 255) {
				mavlink_system.sysid = system_id;
			}

			int32_t component_id;
			param_get(param_component_id, &component_id);
			if (component_id > 0 && component_id < 255) {
				mavlink_system.compid = component_id;
			}
			
			int32_t system_type;
			param_get(param_system_type, &system_type);
			if (system_type >= 0 && system_type < MAV_AUTOPILOT_ENUM_END) {
				mavlink_system.type = system_type;
			}

			/* translate the current syste state to mavlink state and mode */
			uint8_t mavlink_state = 0;
			uint8_t mavlink_mode = 0;
			get_mavlink_mode_and_state(&v_status, &mavlink_state, &mavlink_mode);

			/* send heartbeat */
			mavlink_msg_heartbeat_send(chan, system_type, MAV_AUTOPILOT_PX4, mavlink_mode, v_status.state_machine, mavlink_state);

			/* send status (values already copied in the section above) */
			mavlink_msg_sys_status_send(chan, v_status.onboard_control_sensors_present, v_status.onboard_control_sensors_enabled,
						    v_status.onboard_control_sensors_health, v_status.load, v_status.voltage_battery * 1000.f, v_status.current_battery * 1000.f,
						    v_status.battery_remaining, v_status.drop_rate_comm, v_status.errors_comm,
						    v_status.errors_count1, v_status.errors_count2, v_status.errors_count3, v_status.errors_count4);

			/* send over MAVLink */
			if ((hrt_absolute_time() - rc.timestamp) < 200000) {
				mavlink_msg_rc_channels_raw_send(chan, rc.timestamp / 1000, 0, rc.chan[0].raw, rc.chan[1].raw, rc.chan[2].raw, rc.chan[3].raw,
							 rc.chan[4].raw, rc.chan[5].raw, rc.chan[6].raw, rc.chan[7].raw, rc.rssi);
			}

			lowspeed_counter = 0;
		}
		lowspeed_counter++;

		/* sleep quarter the time */
		usleep(25000);

		/* check if waypoint has been reached against the last positions */
		mavlink_waypoint_eventloop(mavlink_missionlib_get_system_timestamp(), &global_pos, &local_pos);

		/* sleep quarter the time */
		usleep(25000);

		/* send parameters at 20 Hz (if queued for sending) */
		mavlink_pm_queued_send();
		/* sleep quarter the time */
		usleep(25000);
		mavlink_pm_queued_send();

		/* sleep 10 ms */
		usleep(10000);

		/* send one string at 10 Hz */
		mavlink_missionlib_send_gcs_string(mavlink_message_string);
		mavlink_message_string[0] = '\0';
		counter++;

		/* sleep 15 ms */
		usleep(15000);
	}

	/* wait for threads to complete */
	pthread_join(receive_thread, NULL);
	pthread_join(uorb_receive_thread, NULL);

	/* Reset the UART flags to original state */
	if (!usb_uart) {
		int termios_state;

		if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config_original)) < 0) {
			fprintf(stderr, "[mavlink] ERROR setting baudrate / termios config for %s (tcsetattr)\r\n", uart_name);
		}

		printf("[mavlink] Restored original UART config, exiting..\n");
	}

exit_cleanup:

	/* close uart */
	close(uart);

	/* close subscriptions */
	close(mavlink_subs.global_pos_sub);
	close(local_pos_sub);

	fflush(stdout);
	fflush(stderr);

	thread_running = false;

	return 0;
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: mavlink {start|stop|status} [-d <devicename>] [-b <baudrate>] [-e/--exit-allowed]\n\n");
	exit(1);
}

int mavlink_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("mavlink already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		mavlink_task = task_create("mavlink", SCHED_PRIORITY_DEFAULT, 4400, mavlink_thread_main, (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tmavlink app is running\n");
		} else {
			printf("\tmavlink app not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

