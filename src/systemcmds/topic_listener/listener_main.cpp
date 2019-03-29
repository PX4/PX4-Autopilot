/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file listener_main.cpp
 *
 * Tool for listening to topics.
 */

#include <px4_module.h>
#include <px4_getopt.h>

#include "topic_listener.hpp"
#include "topic_listener_generated.hpp"

extern "C" __EXPORT int listener_main(int argc, char *argv[]);

static void usage();

void listener(listener_print_topic_cb cb, const orb_id_t &id, unsigned num_msgs, unsigned topic_instance,
	      unsigned topic_interval)
{
	if (orb_exists(id, topic_instance) != 0) {
		PX4_INFO_RAW("never published\n");
		return;
	}

	int sub = orb_subscribe_multi(id, topic_instance);
	orb_set_interval(sub, topic_interval);

	bool updated = false;
	unsigned i = 0;
	hrt_abstime start_time = hrt_absolute_time();

	while (i < num_msgs) {
		orb_check(sub, &updated);

		if (i == 0) {
			updated = true;

		} else {
			px4_usleep(500);
		}

		if (updated) {
			start_time = hrt_absolute_time();
			i++;

			PX4_INFO_RAW("\nTOPIC: %s instance %d #%d\n", id->o_name, topic_instance, i);

			int ret = cb(id, sub);

			if (ret != PX4_OK) {
				PX4_ERR("listener callback failed (%i)", ret);
			}

		} else {
			if (hrt_elapsed_time(&start_time) > 2 * 1000 * 1000) {
				PX4_INFO_RAW("Waited for 2 seconds without a message. Giving up.\n");
				break;
			}
		}
	}

	orb_unsubscribe(sub);
}

int listener_main(int argc, char *argv[])
{
	if (argc <= 1) {
		usage();
		return 1;
	}

	char *topic_name = argv[1];

	unsigned topic_instance = 0;
	unsigned topic_rate = 0;
	unsigned num_msgs = 0;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:r:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'i':
			topic_instance = strtol(myoptarg, nullptr, 0);
			break;

		case 'r':
			topic_rate = strtol(myoptarg, nullptr, 0);
			break;

		case 'n':
			num_msgs = strtol(myoptarg, nullptr, 0);
			break;

		default:
			usage();
			return -1;
			break;
		}
	}

	if (num_msgs == 0) {
		if (topic_rate != 0) {
			num_msgs = 10 * topic_rate; // arbitrary limit (10 seconds at max rate)

		} else {
			num_msgs = 1;
		}
	}

	listener_generated(topic_name, topic_instance, topic_rate, num_msgs);

	return 0;
}


static void
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to listen on uORB topics and print the data to the console.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("listener", "command");
	PRINT_MODULE_USAGE_ARG("<topic_name>", "uORB topic name", false);

	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, ORB_MULTI_MAX_INSTANCES - 1, "Topic instance", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 0, 100, "Number of messages", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 1000, "Subscription rate (unlimited if 0)", true);
}
