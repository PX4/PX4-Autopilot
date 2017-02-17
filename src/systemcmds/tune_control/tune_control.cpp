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
 * @file tune_control.cpp
 *
 * Command-line tool to control & test the (external) tunes.
 * To use it make sure there's a driver running, which handles the tune_control uorb topic.
 */

#include <px4_getopt.h>
#include <px4_log.h>

#include <unistd.h>

#include <lib/tunes/tunes.h>
#include <uORB/topics/tune_control.h>

#include <drivers/drv_hrt.h>

#define MAX_NOTE_ITERATION 50

static void	usage(void);

static orb_advert_t tune_control_pub = nullptr;

extern "C" {
	__EXPORT int tune_control_main(int argc, char *argv[]);
}

static void
usage()
{
	PX4_INFO(
		"External tune control for testing. Usage:\n"
	);
}

static void publish_tune_control(tune_control_s &tune_control)
{
	tune_control.timestamp = hrt_absolute_time();

	if (tune_control_pub == nullptr) {
		tune_control_pub = orb_advertise_queue(ORB_ID(tune_control), &tune_control, 1);

	} else {
		orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
	}
}

// static void run_led_test1()
// {
// 	PX4_INFO("generating LED pattern...");
//
// 	led_control_s led_control = {};
// 	led_control.led_mask = 0xff;
// 	led_control.mode = led_control_s::MODE_OFF;
// 	led_control.priority = led_control_s::MAX_PRIORITY;
// 	publish_led_control(led_control);
//
// 	usleep(200 * 1000);
//
// 	// generate some pattern
// 	for (int round = led_control_s::COLOR_RED; round <= led_control_s::COLOR_WHITE; ++round) {
// 		for (int led = 0; led < BOARD_MAX_LEDS; ++led) {
// 			led_control.led_mask = 1 << led;
// 			led_control.mode = led_control_s::MODE_ON;
// 			led_control.color = round;
// 			publish_led_control(led_control);
// 			usleep(80 * 1000);
// 		}
//
// 		usleep(100 * 1000);
// 		led_control.led_mask = 0xff;
//
// 		for (int i = 0; i < 3; ++i) {
// 			led_control.mode = led_control_s::MODE_ON;
// 			publish_led_control(led_control);
// 			usleep(100 * 1000);
// 			led_control.mode = led_control_s::MODE_OFF;
// 			publish_led_control(led_control);
// 			usleep(100 * 1000);
// 		}
//
// 		usleep(200 * 1000);
// 	}
//
// 	usleep(500 * 1000);
//
// 	// reset
// 	led_control.led_mask = 0xff;
// 	led_control.mode = led_control_s::MODE_DISABLED;
// 	publish_led_control(led_control);
//
// 	PX4_INFO("Done");
// }

int
tune_control_main(int argc, char *argv[])
{
	output::Tunes tunes;
	bool string_input = false;
	const char *tune_string  = NULL;
	int myoptind = 1;
	int ch;
	const char *myoptarg = NULL;
	uint8_t value;
	tune_control_s tune_control = {};
	tune_control.tune_id = 0;
	tune_control.strength = 40;

	while ((ch = px4_getopt(argc, argv, "f:d:t:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f':
			if ((uint16_t)(strtol(myoptarg, NULL, 0)) > 0 && (uint16_t)(strtol(myoptarg, NULL, 0)) < 22000) {
				tune_control.frequency = (uint16_t)(strtol(myoptarg, NULL, 0));

			} else {
				usage();
				return 1;
			}

			break;

		case 'd':
			tune_control.duration = (uint16_t)(strtol(myoptarg, NULL, 0));
			break;

		case 't':
			value = (uint8_t)(strtol(myoptarg, NULL, 0));

			if (value >= 1 && value <= 15) {
				tune_control.tune_id = value;

			} else {
				usage();
				return 1;
			}

			break;

		case 's':
			string_input = true;

			// TODO: check if the string is a valid tune sequence
			if (1) {
				tune_string  = myoptarg;

			} else {
				usage();
				return 1;
			}

			break;

		case 'p':
			// led_control.priority = strtol(myoptarg, NULL, 0);
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	// if (led_control.priority > led_control_s::MAX_PRIORITY) {
	// 	led_control.priority = led_control_s::MAX_PRIORITY;
	// }

	if (myoptind >= argc) {
		usage();
		return 1;
	}

	unsigned frequency, duration, silence;
	int exit_counter = 0;

	if (!strcmp(argv[myoptind], "play")) {
		if (string_input) {
			PX4_INFO("Start playback...");

			while (tunes.parse_string(tune_string, frequency, duration, silence) > 0) {
				tune_control.tune_id = 0;
				tune_control.frequency = (uint16_t)frequency;
				tune_control.duration = (uint32_t)duration;
				publish_tune_control(tune_control);
				usleep(duration + silence);
				exit_counter++;

				// exit if the loop is doing more thatn 50 iteration
				if (exit_counter > MAX_NOTE_ITERATION) {
					break;
				}
			}

			PX4_INFO("Playback finished.");

		} else {
			if (tune_control.tune_id == 0) {
				tune_control.tune_id = 1;
			}

			PX4_INFO("Publishing standard tune %d", tune_control.tune_id);
			publish_tune_control(tune_control);
		}

	} else if (!strcmp(argv[myoptind], "libtest")) {
		while (tunes.parse_cmd(tune_control, frequency, duration, silence) > 0) {
			PX4_INFO("frequency: %d, duration %d, silence %d", frequency, duration, silence);
			usleep(500000);
			exit_counter++;

			// exit if the loop is doing more thatn 50 iteration
			if (exit_counter > MAX_NOTE_ITERATION) {
				break;
			}
		}

	} else {
		usage();
		return 1;
	}

	// if (led_control.mode != 0xff) {
	// 	publish_led_control(led_control);
	// }

	return 0;
}
