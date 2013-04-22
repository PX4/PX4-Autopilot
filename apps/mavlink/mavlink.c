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
 * @file mavlink.c
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
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
#include <systemlib/err.h>

#include "waypoints.h"
#include "mavlink_log.h"
#include "orb_topics.h"
#include "missionlib.h"
#include "mavlink_hil.h"
#include "util.h"
#include "waypoints.h"
#include "mavlink_parameters.h"

/* define MAVLink specific parameters */
PARAM_DEFINE_INT32(MAV_SYS_ID, 1);
PARAM_DEFINE_INT32(MAV_COMP_ID, 50);
PARAM_DEFINE_INT32(MAV_TYPE, MAV_TYPE_FIXED_WING);

__EXPORT int mavlink_main(int argc, char *argv[]);

static int mavlink_thread_main(int argc, char *argv[]);

/* thread state */
volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int mavlink_task;

/* pthreads */
static pthread_t receive_thread;
static pthread_t uorb_receive_thread;

/* terminate MAVLink on user request - disabled by default */
static bool mavlink_link_termination_allowed = false;

mavlink_system_t mavlink_system = {
	100,
	50,
	MAV_TYPE_FIXED_WING,
	0,
	0,
	0
}; // System ID, 1-255, Component/Subsystem ID, 1-255

/* XXX not widely used */
uint8_t chan = MAVLINK_COMM_0;

/* XXX probably should be in a header... */
extern pthread_t receive_start(int uart);

/* Allocate storage space for waypoints */
static mavlink_wpm_storage wpm_s;
mavlink_wpm_storage *wpm = &wpm_s;

bool mavlink_hil_enabled = false;

/* protocol interface */
static int uart;
static int baudrate;
bool gcs_link = true;

/* interface mode */
static enum {
	MAVLINK_INTERFACE_MODE_OFFBOARD,
	MAVLINK_INTERFACE_MODE_ONBOARD
} mavlink_link_mode = MAVLINK_INTERFACE_MODE_OFFBOARD;

static struct mavlink_logbuffer lb;

static void mavlink_update_system(void);
static int mavlink_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb);
static void usage(void);
int set_mavlink_interval_limit(struct mavlink_subscriptions *subs, int mavlink_msg_id, int min_interval);



int
set_hil_on_off(bool hil_enabled)
{
	int ret = OK;

	/* Enable HIL */
	if (hil_enabled && !mavlink_hil_enabled) {

		/* Advertise topics */
		pub_hil_attitude = orb_advertise(ORB_ID(vehicle_attitude), &hil_attitude);
		pub_hil_global_pos = orb_advertise(ORB_ID(vehicle_global_position), &hil_global_pos);

		/* sensore level hil */
		pub_hil_sensors = orb_advertise(ORB_ID(sensor_combined), &hil_sensors);
		pub_hil_gps = orb_advertise(ORB_ID(vehicle_gps_position), &hil_gps);

		mavlink_hil_enabled = true;

		/* ramp up some HIL-related subscriptions */
		unsigned hil_rate_interval;

		if (baudrate < 19200) {
			/* 10 Hz */
			hil_rate_interval = 100;

		} else if (baudrate < 38400) {
			/* 10 Hz */
			hil_rate_interval = 100;

		} else if (baudrate < 115200) {
			/* 20 Hz */
			hil_rate_interval = 50;

		} else {
			/* 200 Hz */
			hil_rate_interval = 5;
		}

		orb_set_interval(mavlink_subs.spa_sub, hil_rate_interval);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, hil_rate_interval);
	}

	if (!hil_enabled && mavlink_hil_enabled) {
		mavlink_hil_enabled = false;
		orb_set_interval(mavlink_subs.spa_sub, 200);

	} else {
		ret = ERROR;
	}

	return ret;
}

void
get_mavlink_mode_and_state(uint8_t *mavlink_state, uint8_t *mavlink_mode)
{
	/* reset MAVLink mode bitfield */
	*mavlink_mode = 0;

	/* set mode flags independent of system state */

	/* HIL */
	if (v_status.flag_hil_enabled) {
		*mavlink_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* manual input */
	if (v_status.flag_control_manual_enabled) {
		*mavlink_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	}

	/* attitude or rate control */
	if (v_status.flag_control_attitude_enabled ||
	    v_status.flag_control_rates_enabled) {
		*mavlink_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
	}

	/* vector control */
	if (v_status.flag_control_velocity_enabled ||
	    v_status.flag_control_position_enabled) {
		*mavlink_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
	}

	/* autonomous mode */
	if (v_status.state_machine == SYSTEM_STATE_AUTO) {
		*mavlink_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
	}

	/* set arming state */
	if (armed.armed) {
		*mavlink_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

	} else {
		*mavlink_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	}

	switch (v_status.state_machine) {
	case SYSTEM_STATE_PREFLIGHT:
		if (v_status.flag_preflight_gyro_calibration ||
		    v_status.flag_preflight_mag_calibration ||
		    v_status.flag_preflight_accel_calibration) {
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
		break;

	case SYSTEM_STATE_STABILIZED:
		*mavlink_state = MAV_STATE_ACTIVE;
		break;

	case SYSTEM_STATE_AUTO:
		*mavlink_state = MAV_STATE_ACTIVE;
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


int set_mavlink_interval_limit(struct mavlink_subscriptions *subs, int mavlink_msg_id, int min_interval)
{
	int ret = OK;

	switch (mavlink_msg_id) {
	case MAVLINK_MSG_ID_SCALED_IMU:
		/* sensor sub triggers scaled IMU */
		orb_set_interval(subs->sensor_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_HIGHRES_IMU:
		/* sensor sub triggers highres IMU */
		orb_set_interval(subs->sensor_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_RAW_IMU:
		/* sensor sub triggers RAW IMU */
		orb_set_interval(subs->sensor_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_ATTITUDE:
		/* attitude sub triggers attitude */
		orb_set_interval(subs->att_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
		/* actuator_outputs triggers this message */
		orb_set_interval(subs->act_0_sub, min_interval);
		orb_set_interval(subs->act_1_sub, min_interval);
		orb_set_interval(subs->act_2_sub, min_interval);
		orb_set_interval(subs->act_3_sub, min_interval);
		orb_set_interval(subs->actuators_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		/* manual_control_setpoint triggers this message */
		orb_set_interval(subs->man_control_sp_sub, min_interval);
		break;

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		orb_set_interval(subs->debug_key_value, min_interval);
		break;

	default:
		/* not found */
		ret = ERROR;
		break;
	}

	return ret;
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
			struct mavlink_logmessage msg;
			strncpy(msg.text, txt, sizeof(msg.text));
			mavlink_logbuffer_write(&lb, &msg);
			total_counter++;
			return OK;
		}

	default:
		return ENOTTY;
	}
}

#define MAVLINK_OFFBOARD_CONTROL_FLAG_ARMED 0x10

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
	/* process baud rate */
	int speed;

	switch (baud) {
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
		fprintf(stderr, "[mavlink] ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600\n\n", baud);
		return -EINVAL;
	}

	/* open uart */
	printf("[mavlink] UART is %s, baudrate is %d\n", uart_name, baud);
	uart = open(uart_name, O_RDWR | O_NOCTTY);

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;
	*is_usb = false;

	/* make some wild guesses including that USB serial is indicated by either /dev/ttyACM0 or /dev/console */
	if (strcmp(uart_name, "/dev/ttyACM0") != OK && strcmp(uart_name, "/dev/console") != OK) {
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

void
mavlink_send_uart_bytes(mavlink_channel_t channel, uint8_t *ch, int length)
{
	write(uart, ch, (size_t)(sizeof(uint8_t) * length));
}

/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t *mavlink_get_channel_status(uint8_t channel)
{
	static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
	return &m_mavlink_status[channel];
}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel)
{
	static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
	return &m_mavlink_buffer[channel];
}

void mavlink_update_system(void)
{
	static bool initialized = false;
	static param_t param_system_id;
	static param_t param_component_id;
	static param_t param_system_type;

	if (!initialized) {
		param_system_id = param_find("MAV_SYS_ID");
		param_component_id = param_find("MAV_COMP_ID");
		param_system_type = param_find("MAV_TYPE");
		initialized = true;
	}

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

	if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END) {
		mavlink_system.type = system_type;
	}
}

/**
 * MAVLink Protocol main function.
 */
int mavlink_thread_main(int argc, char *argv[])
{
	/* initialize mavlink text message buffering */
	mavlink_logbuffer_init(&lb, 5);

	int ch;
	char *device_name = "/dev/ttyS1";
	baudrate = 57600;

	/* work around some stupidity in task_create's argv handling */
	argc -= 2;
	argv += 2;

	while ((ch = getopt(argc, argv, "b:d:eo")) != EOF) {
		switch (ch) {
		case 'b':
			baudrate = strtoul(optarg, NULL, 10);

			if (baudrate == 0)
				errx(1, "invalid baud rate '%s'", optarg);

			break;

		case 'd':
			device_name = optarg;
			break;

		case 'e':
			mavlink_link_termination_allowed = true;
			break;

		case 'o':
			mavlink_link_mode = MAVLINK_INTERFACE_MODE_ONBOARD;
			break;

		default:
			usage();
		}
	}

	struct termios uart_config_original;

	bool usb_uart;

	/* print welcome text */
	warnx("MAVLink v1.0 serial interface starting...");

	/* inform about mode */
	warnx((mavlink_link_mode == MAVLINK_INTERFACE_MODE_ONBOARD) ? "ONBOARD MODE" : "DOWNLINK MODE");

	/* Flush stdout in case MAVLink is about to take it over */
	fflush(stdout);

	/* default values for arguments */
	uart = mavlink_open_uart(baudrate, device_name, &uart_config_original, &usb_uart);

	if (uart < 0)
		err(1, "could not open %s", device_name);

	/* create the device node that's used for sending text log messages, etc. */
	register_driver(MAVLINK_LOG_DEVICE, &mavlink_fops, 0666, NULL);

	/* Initialize system properties */
	mavlink_update_system();

	/* start the MAVLink receiver */
	receive_thread = receive_start(uart);

	/* start the ORB receiver */
	uorb_receive_thread = uorb_receive_start();

	/* initialize waypoint manager */
	mavlink_wpm_init(wpm);

	/* all subscriptions are now active, set up initial guess about rate limits */
	if (baudrate >= 230400) {
		/* 200 Hz / 5 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_HIGHRES_IMU, 20);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_RAW_IMU, 20);
		/* 50 Hz / 20 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 30);
		/* 20 Hz / 50 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 10);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 50);
		/* 10 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_GPS_RAW_INT, 100);
		/* 10 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 100);

	} else if (baudrate >= 115200) {
		/* 20 Hz / 50 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_HIGHRES_IMU, 50);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_RAW_IMU, 50);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 50);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 50);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 200);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_GPS_RAW_INT, 200);
		/* 2 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 500);

	} else if (baudrate >= 57600) {
		/* 10 Hz / 100 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_RAW_IMU, 300);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_HIGHRES_IMU, 300);
		/* 10 Hz / 100 ms ATTITUDE */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 200);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 200);
		/* 5 Hz / 200 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 500);
		/* 2 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 500);
		/* 2 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_GPS_RAW_INT, 500);

	} else {
		/* very low baud rate, limit to 1 Hz / 1000 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_RAW_IMU, 1000);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_ATTITUDE, 1000);
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_HIGHRES_IMU, 1000);
		/* 1 Hz / 1000 ms */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 1000);
		/* 0.5 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 2000);
		/* 0.1 Hz */
		set_mavlink_interval_limit(&mavlink_subs, MAVLINK_MSG_ID_MANUAL_CONTROL, 10000);
	}

	thread_running = true;

	/* arm counter to go off immediately */
	unsigned lowspeed_counter = 10;

	while (!thread_should_exit) {

		/* 1 Hz */
		if (lowspeed_counter == 10) {
			mavlink_update_system();

			/* translate the current system state to mavlink state and mode */
			uint8_t mavlink_state = 0;
			uint8_t mavlink_mode = 0;
			get_mavlink_mode_and_state(&mavlink_state, &mavlink_mode);

			/* send heartbeat */
			mavlink_msg_heartbeat_send(chan, mavlink_system.type, MAV_AUTOPILOT_PX4, mavlink_mode, v_status.state_machine, mavlink_state);

			/* switch HIL mode if required */
			set_hil_on_off(v_status.flag_hil_enabled);

			/* send status (values already copied in the section above) */
			mavlink_msg_sys_status_send(chan,
						    v_status.onboard_control_sensors_present,
						    v_status.onboard_control_sensors_enabled,
						    v_status.onboard_control_sensors_health,
						    v_status.load,
						    v_status.voltage_battery * 1000.0f,
						    v_status.current_battery * 1000.0f,
						    v_status.battery_remaining,
						    v_status.drop_rate_comm,
						    v_status.errors_comm,
						    v_status.errors_count1,
						    v_status.errors_count2,
						    v_status.errors_count3,
						    v_status.errors_count4);
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

		if (baudrate > 57600) {
			mavlink_pm_queued_send();
		}

		/* sleep 10 ms */
		usleep(10000);

		/* send one string at 10 Hz */
		if (!mavlink_logbuffer_is_empty(&lb)) {
			struct mavlink_logmessage msg;
			int lb_ret = mavlink_logbuffer_read(&lb, &msg);

			if (lb_ret == OK) {
				mavlink_missionlib_send_gcs_string(msg.text);
			}
		}

		/* sleep 15 ms */
		usleep(15000);
	}

	/* wait for threads to complete */
	pthread_join(receive_thread, NULL);
	pthread_join(uorb_receive_thread, NULL);

	/* Reset the UART flags to original state */
	if (!usb_uart)
		tcsetattr(uart, TCSANOW, &uart_config_original);

	thread_running = false;

	exit(0);
}

static void
usage()
{
	fprintf(stderr, "usage: mavlink start [-d <devicename>] [-b <baud rate>]\n"
		"       mavlink stop\n"
		"       mavlink status\n");
	exit(1);
}

int mavlink_main(int argc, char *argv[])
{

	if (argc < 2) {
		warnx("missing command");
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* this is not an error */
		if (thread_running)
			errx(0, "mavlink already running\n");

		thread_should_exit = false;
		mavlink_task = task_spawn("mavlink",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2048,
					  mavlink_thread_main,
					  (const char **)argv);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;

		while (thread_running) {
			usleep(200000);
			printf(".");
		}

		warnx("terminated.");
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	usage();
	/* not getting here */
	return 0;
}

