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

#include <string.h>
#include <unistd.h>

#include <lib/tunes/tunes.h>
#include <uORB/topics/tune_control.h>

#include <drivers/drv_hrt.h>

#define MAX_NOTE_ITERATION 50

static void	usage();

static orb_advert_t tune_control_pub = nullptr;

extern "C" {
	__EXPORT int tune_control_main(int argc, char *argv[]);
}

static void
usage()
{
	PX4_INFO(
		"External tune control for testing. Usage:\n"
		"tune_control play [-t <default tunes>] [-f <frequency>] [-d <duration>] [-s <strength>] [-m < melody>]\n"
		"\n"
		"\t-t <defualt tunes>\tPlay the default (1...15) (default=1)\n"
		"\t-f <frequency>\t\tFrequency from 0-20kHz\n"
		"\t-d <duration>\t\tDuration of the tone in us\n"
		"\t-s <strength>\t\tStrength of the tone between 0-100\n"
		"\t-m <melody>\t\tMelody in a string form ex: \"MFT200e8a8a\"\n"
		"\n"
		"tune_control stop \t\tStops playback, useful for repeated tunes\n"
	);
}

static void publish_tune_control(tune_control_s &tune_control)
{
	tune_control.timestamp = hrt_absolute_time();

	if (tune_control_pub == nullptr) {
		// We have a minimum of 3 so that tune, stop, tune will fit
		tune_control_pub = orb_advertise_queue(ORB_ID(tune_control), &tune_control, 3);

	} else {
		orb_publish(ORB_ID(tune_control), tune_control_pub, &tune_control);
	}
}

int
tune_control_main(int argc, char *argv[])
{
	Tunes tunes;
	bool string_input = false;
	const char *tune_string  = nullptr;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	unsigned int value;
	tune_control_s tune_control = {};
	tune_control.tune_id = 0;
	tune_control.strength = tune_control_s::STRENGTH_NORMAL;

	while ((ch = px4_getopt(argc, argv, "f:d:t:m:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'f':
			value = (uint16_t)(strtol(myoptarg, nullptr, 0));

			if (value > 0 && value < 22000) {
				tune_control.frequency = value;

			} else {
				usage();
				return 1;
			}

			break;

		case 'd':
			tune_control.duration = (uint32_t)(strtol(myoptarg, nullptr, 0));
			break;

		case 't':
			value = (uint8_t)(strtol(myoptarg, nullptr, 0));

			if (value > 0 && value < tunes.get_default_tunes_size()) {
				tune_control.tune_id = value;

			} else {
				usage();
				return 1;
			}

			break;

		case 'm':
			string_input = true;
			tune_string  = myoptarg;

			// check if string is a valid melody string
			if (tune_string[0] != 'M') {
				usage();
				return 1;
			}

			break;

		case 's':
			value = (uint8_t)(strtol(myoptarg, nullptr, 0));

			if (value > 0 && value < 100) {
				tune_control.strength = value;

			} else {
				tune_control.strength = tune_control_s::STRENGTH_NORMAL;
			}

			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	if (myoptind >= argc) {
		usage();
		return 1;
	}

	unsigned frequency, duration, silence;
	uint8_t strength;
	int exit_counter = 0;

	if (!strcmp(argv[myoptind], "play")) {
		if (string_input) {
			PX4_INFO("Start playback...");
			tunes.set_string(tune_string, tune_control.strength);

			while (tunes.get_next_tune(frequency, duration, silence, strength) > 0) {
				tune_control.tune_id = 0;
				tune_control.frequency = (uint16_t)frequency;
				tune_control.duration = (uint32_t)duration;
				tune_control.silence = (uint32_t)silence;
				tune_control.strength = (uint8_t)strength;
				publish_tune_control(tune_control);
				usleep(duration + silence);
				exit_counter++;

				// exit if the loop is doing too many iterations
				if (exit_counter > MAX_NOTE_ITERATION) {
					break;
				}
			}

			PX4_INFO("Playback finished.");

		} else {  // tune id instead of string has been provided
			if (tune_control.tune_id == 0) {
				tune_control.tune_id = 1;
			}

			PX4_INFO("Publishing standard tune %d", tune_control.tune_id);
			publish_tune_control(tune_control);
		}

	} else if (!strcmp(argv[myoptind], "libtest")) {
		int ret = tunes.set_control(tune_control);

		if (ret == -EINVAL) {
			PX4_WARN("Tune ID not recognized.");
		}

		while (tunes.get_next_tune(frequency, duration, silence, strength) > 0) {
			PX4_INFO("frequency: %d, duration %d, silence %d, strength%d",
				 frequency, duration, silence, strength);
			usleep(500000);
			exit_counter++;

			// exit if the loop is doing too many iterations
			if (exit_counter > MAX_NOTE_ITERATION) {
				break;
			}
		}

	} else if (!strcmp(argv[myoptind], "stop")) {
		PX4_INFO("Stopping playback...");
		tune_control.tune_id = 0;
		tune_control.frequency = 0;
		tune_control.duration = 0;
		tune_control.silence = 0;
		tune_control.tune_override = true;
		publish_tune_control(tune_control);
		// We wait the maximum update interval to ensure
		// The stop will not be overwritten
		usleep(tunes.get_maximum_update_interval());

	}	else {
		usage();
		return 1;
	}

	return 0;
}
