/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file led_control.cpp
 *
 * Command-line tool to control & test the (external) LED's.
 * To use it make sure there's a driver running, which handles the led_control uorb topic.
 */

#include <px4_getopt.h>
#include <px4_log.h>

#include <unistd.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>

static void	usage();

static orb_advert_t led_control_pub = nullptr;

extern "C" {
	__EXPORT int led_control_main(int argc, char *argv[]);
}

static void
usage()
{
	PX4_INFO(
		"External Led control for testing. Usage:\n"
		"led_control test\t\tRun test pattern\n"
		"led_control {on, off, reset, blink, breathe} [-c <color>] [-l <led>] [-n <num_blink>] [-s <speed>] [-p <prio>]\n"
		"\n"
		"\t-c <color>\t\tColor (red,blue,green,yellow,purple,amber,cyan,white) (default=white)\n"
		"\t-l <led>\t\tWhich led to control (0,1,...) (default=all)\n"
		"\t-n <num_blink>\t\tNumber of blinks (default=3)\n"
		"\t-s <speed>\t\tHow fast to blink (fast,normal,slow) (default=normal)\n"
		"\t-p <prio>\t\tControl priority (0,1,2) (default=maximum)\n"
	);
}

static void publish_led_control(led_control_s &led_control)
{
	led_control.timestamp = hrt_absolute_time();

	if (led_control_pub == nullptr) {
		led_control_pub = orb_advertise_queue(ORB_ID(led_control), &led_control, LED_UORB_QUEUE_LENGTH);

	} else {
		orb_publish(ORB_ID(led_control), led_control_pub, &led_control);
	}
}

static void run_led_test1()
{
	PX4_INFO("generating LED pattern...");

	led_control_s led_control = {};
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_OFF;
	led_control.priority = led_control_s::MAX_PRIORITY;
	publish_led_control(led_control);

	usleep(200 * 1000);

	// generate some pattern
	for (int round = led_control_s::COLOR_RED; round <= led_control_s::COLOR_WHITE; ++round) {
		for (int led = 0; led < BOARD_MAX_LEDS; ++led) {
			led_control.led_mask = 1 << led;
			led_control.mode = led_control_s::MODE_ON;
			led_control.color = round;
			publish_led_control(led_control);
			usleep(80 * 1000);
		}

		usleep(100 * 1000);
		led_control.led_mask = 0xff;

		for (int i = 0; i < 3; ++i) {
			led_control.mode = led_control_s::MODE_ON;
			publish_led_control(led_control);
			usleep(100 * 1000);
			led_control.mode = led_control_s::MODE_OFF;
			publish_led_control(led_control);
			usleep(100 * 1000);
		}

		usleep(200 * 1000);
	}

	usleep(500 * 1000);

	// reset
	led_control.led_mask = 0xff;
	led_control.mode = led_control_s::MODE_DISABLED;
	publish_led_control(led_control);

	PX4_INFO("Done");
}

int
led_control_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	uint8_t blink_speed = led_control_s::MODE_BLINK_NORMAL;
	led_control_s led_control = {};
	led_control.num_blinks = 3;
	led_control.priority = led_control_s::MAX_PRIORITY;
	led_control.mode = 0xff;
	led_control.led_mask = 0xff;
	led_control.color = led_control_s::COLOR_WHITE;

	while ((ch = px4_getopt(argc, argv, "c:l:n:s:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'c':
			if (!strcmp(myoptarg, "red")) {
				led_control.color = led_control_s::COLOR_RED;

			} else if (!strcmp(myoptarg, "blue")) {
				led_control.color = led_control_s::COLOR_BLUE;

			} else if (!strcmp(myoptarg, "green")) {
				led_control.color = led_control_s::COLOR_GREEN;

			} else if (!strcmp(myoptarg, "yellow")) {
				led_control.color = led_control_s::COLOR_YELLOW;

			} else if (!strcmp(myoptarg, "purple")) {
				led_control.color = led_control_s::COLOR_PURPLE;

			} else if (!strcmp(myoptarg, "amber")) {
				led_control.color = led_control_s::COLOR_AMBER;

			} else if (!strcmp(myoptarg, "cyan")) {
				led_control.color = led_control_s::COLOR_CYAN;

			} else if (!strcmp(myoptarg, "white")) {
				led_control.color = led_control_s::COLOR_WHITE;

			} else {
				usage();
				return 1;
			}

			break;

		case 'l':
			led_control.led_mask = 1 << strtol(myoptarg, nullptr, 0);
			break;

		case 'n':
			led_control.num_blinks = strtol(myoptarg, nullptr, 0);
			break;

		case 's':
			if (!strcmp(myoptarg, "fast")) {
				blink_speed = led_control_s::MODE_BLINK_FAST;

			} else if (!strcmp(myoptarg, "normal")) {
				blink_speed = led_control_s::MODE_BLINK_NORMAL;

			} else if (!strcmp(myoptarg, "slow")) {
				blink_speed = led_control_s::MODE_BLINK_SLOW;

			} else {
				usage();
				return 1;
			}

			break;

		case 'p':
			led_control.priority = strtol(myoptarg, nullptr, 0);
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	if (led_control.priority > led_control_s::MAX_PRIORITY) {
		led_control.priority = led_control_s::MAX_PRIORITY;
	}

	if (myoptind >= argc) {
		usage();
		return 1;
	}

	if (!strcmp(argv[myoptind], "test")) {
		run_led_test1();

	} else if (!strcmp(argv[myoptind], "on")) {
		led_control.mode = led_control_s::MODE_ON;

	} else if (!strcmp(argv[myoptind], "off")) {
		led_control.mode = led_control_s::MODE_OFF;

	} else if (!strcmp(argv[myoptind], "reset")) {
		led_control.mode = led_control_s::MODE_DISABLED;

	} else if (!strcmp(argv[myoptind], "blink")) {
		led_control.mode = blink_speed;

	} else if (!strcmp(argv[myoptind], "breathe")) {
		led_control.mode = led_control_s::MODE_BREATHE;

	} else {
		usage();
		return 1;
	}

	if (led_control.mode != 0xff) {
		publish_led_control(led_control);
	}

	return 0;
}
