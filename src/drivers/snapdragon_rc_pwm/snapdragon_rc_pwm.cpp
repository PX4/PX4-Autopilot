/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file snapdragon_rc_pwm.cpp
 * @author Roman Bapst <roman@px4.io>
 *
 * This driver sends rc commands to the Snapdragon via UART. On the same UART it receives pwm
 * motor commands from the Snapdragon.
 */


#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <string.h>
#include <termios.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <v1.0/mavlink_types.h>
#include <v1.0/common/mavlink.h>

#include "drivers/drv_pwm_output.h"
#include <drivers/drv_hrt.h>


#define MAX_LEN_DEV_PATH 32

namespace snapdragon_rc_pwm
{

static const uint8_t mavlink_message_lengths[256] = MAVLINK_MESSAGE_LENGTHS;
static const uint8_t mavlink_message_crcs[256] = MAVLINK_MESSAGE_CRCS;

volatile bool _task_should_exit = false; // flag indicating if uart_esc task should exit
static char _device[MAX_LEN_DEV_PATH];
static bool _is_running = false;         // flag indicating if uart_esc app is running
static px4_task_t _task_handle = -1;     // handle to the task main thread
static int _uart_fd = -1;
int _pwm_fd = -1;
static bool _flow_control_enabled = false;
int _rc_sub = -1;

hrt_abstime _last_actuator_controls_received = 0;

struct input_rc_s _rc = {};

// Print out the usage information
void usage();

void start();

/** uart_esc stop */
void stop();

int initialise_uart();

int deinitialize_uart();

int enable_flow_control(bool enabled);

void send_rc_mavlink();

void handle_message(mavlink_message_t *msg);

void set_pwm_output(mavlink_actuator_control_target_t *actuator_controls);

/** task main trampoline function */
void task_main_trampoline(int argc, char *argv[]);

/** uart_esc thread primary entry point */
void task_main(int argc, char *argv[]);

void task_main(int argc, char *argv[])
{
	char serial_buf[128];
	mavlink_status_t serial_status = {};

	_rc_sub = orb_subscribe(ORB_ID(input_rc));


	initialise_uart();

	_pwm_fd = open(PWM_OUTPUT0_DEVICE_PATH, 0);

	if (_pwm_fd < 0) {
		PX4_ERR("can't open %s", PWM_OUTPUT0_DEVICE_PATH);
		return;
	}

	// we wait for uart actuator controls messages from snapdragon
	px4_pollfd_struct_t fds[1];
	fds[0].fd = _uart_fd;
	fds[0].events = POLLIN;

	while (true) {

		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out
		if (pret == 0) {
			// let's run the loop anyway to send RC
		}

		if (pret < 0) {
			PX4_WARN("snapdragon_rc_pwm poll error");
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			int len = ::read(_uart_fd, serial_buf, sizeof(serial_buf));

			if (len > 0) {
				mavlink_message_t msg;

				for (int i = 0; i < len; ++i) {
					if (mavlink_parse_char(MAVLINK_COMM_1, serial_buf[i], &msg, &serial_status)) {
						// have a message, handle it
						handle_message(&msg);
					}
				}
			}
		}

		// check if we have new rc data, if yes send it to snapdragon
		bool rc_updated = false;
		orb_check(_rc_sub, &rc_updated);

		if (rc_updated) {
			orb_copy(ORB_ID(input_rc), _rc_sub, &_rc);
			// send mavlink message
			send_rc_mavlink();
		}

		// Turn motors off after timeout of 0.1s.
		if (hrt_elapsed_time(&_last_actuator_controls_received) > 100000) {
			set_pwm_output(nullptr);
		}
	}

	deinitialize_uart();
	close(_pwm_fd);

}

void handle_message(mavlink_message_t *msg)
{
	if (msg->msgid == MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET) {
		mavlink_actuator_control_target_t actuator_controls;
		mavlink_msg_actuator_control_target_decode(msg, &actuator_controls);

		//static unsigned counter = 0;
		//if (counter++ % 250 == 0) {
		//	PX4_INFO("got motor controls %.2f %.2f %.2f %.2f",
		//		 (double)actuator_controls.controls[0],
		//		 (double)actuator_controls.controls[1],
		//		 (double)actuator_controls.controls[2],
		//		 (double)actuator_controls.controls[3]);
		//}
		set_pwm_output(&actuator_controls);

		_last_actuator_controls_received = hrt_absolute_time();
	}
}

void set_pwm_output(mavlink_actuator_control_target_t *actuator_controls)
{
	if (actuator_controls == nullptr) {
		// Without valid argument, set all channels to 0
		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
			int ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), 0);

			if (ret != OK) {
				PX4_ERR("PWM_SERVO_SET(%d)", i);
			}
		}

	} else {
		for (unsigned i = 0; i < sizeof(actuator_controls->controls) / sizeof(actuator_controls->controls[0]); i++) {
			if (!isnan(actuator_controls->controls[i])) {
				long unsigned pwm = actuator_controls->controls[i];
				int ret = ::ioctl(_pwm_fd, PWM_SERVO_SET(i), pwm);

				if (ret != OK) {
					PX4_ERR("PWM_SERVO_SET(%d)", i);
				}
			}
		}
	}
}

void send_rc_mavlink()
{
	mavlink_rc_channels_t rc_message;
	rc_message.time_boot_ms = _rc.timestamp_publication / 1000;
	rc_message.chancount = _rc.channel_count;
	rc_message.chan1_raw = (_rc.channel_count > 0) ? _rc.values[0] : UINT16_MAX;
	rc_message.chan2_raw = (_rc.channel_count > 1) ? _rc.values[1] : UINT16_MAX;
	rc_message.chan3_raw = (_rc.channel_count > 2) ? _rc.values[2] : UINT16_MAX;
	rc_message.chan4_raw = (_rc.channel_count > 3) ? _rc.values[3] : UINT16_MAX;
	rc_message.chan5_raw = (_rc.channel_count > 4) ? _rc.values[4] : UINT16_MAX;
	rc_message.chan6_raw = (_rc.channel_count > 5) ? _rc.values[5] : UINT16_MAX;
	rc_message.chan7_raw = (_rc.channel_count > 6) ? _rc.values[6] : UINT16_MAX;
	rc_message.chan8_raw = (_rc.channel_count > 7) ? _rc.values[7] : UINT16_MAX;
	rc_message.chan9_raw = (_rc.channel_count > 8) ? _rc.values[8] : UINT16_MAX;
	rc_message.chan10_raw = (_rc.channel_count > 9) ? _rc.values[9] : UINT16_MAX;
	rc_message.chan11_raw = (_rc.channel_count > 10) ? _rc.values[10] : UINT16_MAX;
	rc_message.chan12_raw = (_rc.channel_count > 11) ? _rc.values[11] : UINT16_MAX;
	rc_message.chan13_raw = (_rc.channel_count > 12) ? _rc.values[12] : UINT16_MAX;
	rc_message.chan14_raw = (_rc.channel_count > 13) ? _rc.values[13] : UINT16_MAX;
	rc_message.chan15_raw = (_rc.channel_count > 14) ? _rc.values[14] : UINT16_MAX;
	rc_message.chan16_raw = (_rc.channel_count > 15) ? _rc.values[15] : UINT16_MAX;
	rc_message.chan17_raw = (_rc.channel_count > 16) ? _rc.values[16] : UINT16_MAX;
	rc_message.chan18_raw = (_rc.channel_count > 17) ? _rc.values[17] : UINT16_MAX;
	rc_message.rssi = _rc.rssi;

	const uint8_t msgid = MAVLINK_MSG_ID_RC_CHANNELS;
	uint8_t component_ID = 0;
	uint8_t payload_len = mavlink_message_lengths[msgid];
	unsigned packet_len = payload_len + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	uint8_t buf[MAVLINK_MAX_PACKET_LEN];

	/* header */
	buf[0] = MAVLINK_STX;
	buf[1] = payload_len;
	/* no idea which numbers should be here*/
	buf[2] = 100;
	buf[3] = 0;
	buf[4] = component_ID;
	buf[5] = msgid;

	/* payload */
	memcpy(&buf[MAVLINK_NUM_HEADER_BYTES], (const void *)&rc_message, payload_len);

	/* checksum */
	uint16_t checksum;
	crc_init(&checksum);
	crc_accumulate_buffer(&checksum, (const char *) &buf[1], MAVLINK_CORE_HEADER_LEN + payload_len);
	crc_accumulate(mavlink_message_crcs[msgid], &checksum);

	buf[MAVLINK_NUM_HEADER_BYTES + payload_len] = (uint8_t)(checksum & 0xFF);
	buf[MAVLINK_NUM_HEADER_BYTES + payload_len + 1] = (uint8_t)(checksum >> 8);

	int len = ::write(_uart_fd, &buf[0], packet_len);

	//static unsigned counter = 0;
	//if (counter++ % 100 == 0) {
	//	PX4_INFO("sent %d %d %d %d %d %d", len,
	//					   rc_message.chan1_raw,
	//					   rc_message.chan2_raw,
	//					   rc_message.chan3_raw,
	//					   rc_message.chan4_raw,
	//					   rc_message.chan5_raw);
	//}

	if (len < 1) {
		PX4_WARN("failed sending rc mavlink message");
	}
}

int initialise_uart()
{
	// open uart
	_uart_fd = px4_open(_device, O_RDWR | O_NOCTTY);
	int termios_state = -1;

	if (_uart_fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	int speed = B921600;
	struct termios uart_config;
	tcgetattr(_uart_fd, &uart_config);
	// clear ONLCR flag (which appends a CR for every LF)
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		warnx("ERR SET BAUD %s: %d\n", _device, termios_state);
		::close(_uart_fd);
		return -1;
	}

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", _device);
		px4_close(_uart_fd);
		return -1;
	}

	/* setup output flow control */
	if (enable_flow_control(false)) {
		PX4_WARN("hardware flow disable failed");
	}

	return _uart_fd;
}

int enable_flow_control(bool enabled)
{
	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (enabled) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;
	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_enabled = enabled;
	}

	return ret;
}

int deinitialize_uart()
{
	return close(_uart_fd);
}

// uart_esc main entrance
void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	/* start the task */
	_task_handle = px4_task_spawn_cmd("snapdragon_rc_pwm_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  2000,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

	_is_running = true;
}

void stop()
{
	// TODO - set thread exit signal to terminate the task main thread

	_is_running = false;
	_task_handle = -1;
}

void usage()
{
	PX4_WARN("missing command: try 'start', 'stop', 'status'");
	PX4_WARN("options:");
	PX4_WARN("    -d device");
}


}

extern "C" __EXPORT int snapdragon_rc_pwm_main(int argc, char *argv[]);

int snapdragon_rc_pwm_main(int argc, char *argv[])
{
	const char *device = NULL;
	int ch;
	int myoptind = 1;
	const char *myoptarg = NULL;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		default:
			snapdragon_rc_pwm::usage();
			return 1;
		}
	}

	// Check on required arguments
	if (device == NULL || strlen(device) == 0) {
		snapdragon_rc_pwm::usage();
		return 1;
	}

	memset(snapdragon_rc_pwm::_device, 0, MAX_LEN_DEV_PATH);
	strncpy(snapdragon_rc_pwm::_device, device, strlen(device));

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (snapdragon_rc_pwm::_is_running) {
			PX4_WARN("uart_esc already running");
			return 1;
		}

		snapdragon_rc_pwm::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (snapdragon_rc_pwm::_is_running) {
			PX4_WARN("snapdragon_rc_pwm is not running");
			return 1;
		}

		snapdragon_rc_pwm::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("snapdragon_rc_pwm is %s", snapdragon_rc_pwm::_is_running ? "running" : "stopped");
		return 0;

	} else {
		snapdragon_rc_pwm::usage();
		return 1;
	}

	return 0;
}
