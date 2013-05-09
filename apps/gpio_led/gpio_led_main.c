/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file gpio_led.c
 *
 * Drive status LED via GPIO.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <nuttx/sched.h>
#include <systemlib/systemlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>
#include <poll.h>
#include <drivers/drv_gpio.h>

static bool thread_should_exit = false;
static bool thread_running = false;

__EXPORT int gpio_led_main(int argc, char *argv[]);

static int gpio_led_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);
	fprintf(stderr, "usage: gpio_led {start|stop|status}\n\n");
	exit(1);
}

int gpio_led_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			printf("gpio_led already running\n");
			/* this is not an error */
			exit(0);
		}

		thread_should_exit = false;
		task_spawn("gpio_led",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MIN,
					 2048,
					 gpio_led_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			printf("\tgpio_led is running\n");
		} else {
			printf("\tgpio_led not started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

int gpio_led_thread_main(int argc, char *argv[])
{
	/* welcome user */
	printf("[gpio_led] started\n");

	int fd = open(GPIO_DEVICE_PATH, 0);
	if (fd < 0) {
		printf("[gpio_led] GPIO: open fail\n");
		return ERROR;
	}

	/* set GPIO EXT 1 as output */
	ioctl(fd, GPIO_SET_OUTPUT, GPIO_EXT_1);

	ioctl(fd, GPIO_CLEAR, GPIO_EXT_1);

	/* initialize values */
	bool led_state = false;
	int counter = 0;

	/* subscribe to vehicle status topic */
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));
	int vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	while (!thread_should_exit) {
		bool status_updated;
		orb_check((vehicle_status_sub), &status_updated);
		if (status_updated)
			orb_copy(ORB_ID(vehicle_status), vehicle_status_sub, &status);

		int pattern = 0;
		if (status.flag_system_armed) {
			if (status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
				pattern = 0x3f;	// ****** solid (armed)
			} else {
				pattern = 0x2A;	// *_*_*_ fast blink (armed, battery warning)
			}
		} else {
			if (status.state_machine == SYSTEM_STATE_PREFLIGHT) {
				pattern = 0x00;	// ______ off (disarmed, preflight check)
			} else if (status.state_machine == SYSTEM_STATE_STANDBY && status.battery_warning == VEHICLE_BATTERY_WARNING_NONE) {
				pattern = 0x38;	// ***___ slow blink (disarmed, ready)
			} else {
				pattern = 0x28;	// *_*___ slow double blink (disarmed, not good to arm)
			}
		}

		bool led_state_new = (pattern & (1 << counter)) != 0;
		if (led_state_new != led_state) {
			led_state = led_state_new;
			if (led_state) {
				ioctl(fd, GPIO_SET, GPIO_EXT_1);
			} else {
				ioctl(fd, GPIO_CLEAR, GPIO_EXT_1);
			}
		}

		counter++;
		if (counter > 5)
			counter = 0;

		usleep(333333);	// sleep ~1/3s
	}

	ioctl(fd, GPIO_CLEAR, GPIO_EXT_1);

	printf("[gpio_led] exiting\n");

	return 0;
}
