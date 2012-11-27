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
 * @file delay_test.c
 *
 * Simple but effective delay test using leds and a scope to measure
 * communication delays end-to-end accurately.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <string.h>

#include <systemlib/err.h>

#include <drivers/drv_led.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>

#include <uORB/topics/vehicle_vicon_position.h>
#include <uORB/topics/vehicle_command.h>

__EXPORT int delay_test_main(int argc, char *argv[]);
static int led_on(int leds, int led);
static int led_off(int leds, int led);

int delay_test_main(int argc, char *argv[])
{
	bool vicon_msg = false;
	bool command_msg = false;

	if (argc > 1 && !strcmp(argv[1], "--help")) {
		warnx("usage: delay_test [vicon] [command]\n");
		exit(1);
	}

	if (argc > 1 && !strcmp(argv[1], "vicon")) {
		vicon_msg = true;
	}

	if (argc > 1 && !strcmp(argv[1], "command")) {
		command_msg = true;
	}

	int buzzer = open("/dev/tone_alarm", O_WRONLY);
	int leds = open(LED_DEVICE_PATH, 0);

	/* prepare use of amber led */
	led_off(leds, LED_AMBER);

	int topic;

	/* subscribe to topic */
	if (vicon_msg) {
		topic = orb_subscribe(ORB_ID(vehicle_vicon_position));
	} else if (command_msg) {
		topic = orb_subscribe(ORB_ID(vehicle_command));
	} else {
		errx(1, "No topic selected for delay test, use --help for a list of topics.");
	}

	const int testcount = 20;

	warnx("running %d iterations..\n", testcount);

	struct pollfd fds[1];
	fds[0].fd = topic;
	fds[0].events = POLLIN;

	/* display and sound error */
	for (int i = 0; i < testcount; i++)
	{
		/* wait for topic */
		int ret = poll(&fds[0], 1, 2000);

		/* this would be bad... */
		if (ret < 0) {
			warnx("poll error %d", errno);
			usleep(1000000);
			continue;
		}

		/* do we have a topic update? */
		if (fds[0].revents & POLLIN) {

			/* copy object to disable poll ready state */
			if (vicon_msg) {
				struct vehicle_vicon_position_s vicon_position;
				orb_copy(ORB_ID(vehicle_vicon_position), topic, &vicon_position);
			} else if (command_msg) {
				struct vehicle_command_s vehicle_command;
				orb_copy(ORB_ID(vehicle_command), topic, &vehicle_command);
			}

			led_on(leds, LED_AMBER);
			ioctl(buzzer, TONE_SET_ALARM, 4);
			/* keep led on for 50 ms to make it barely visible */
			usleep(50000);
			led_off(leds, LED_AMBER);
		}
	}

	/* stop alarm */
	ioctl(buzzer, TONE_SET_ALARM, 0);

	/* switch on leds */
	led_on(leds, LED_BLUE);
	led_on(leds, LED_AMBER);

	exit(0);
}

static int led_off(int leds, int led)
{
	return ioctl(leds, LED_OFF, led);
}

static int led_on(int leds, int led)
{
	return ioctl(leds, LED_ON, led);
}