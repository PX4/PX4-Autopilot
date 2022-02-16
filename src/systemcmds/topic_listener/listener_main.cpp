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

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>

#include <poll.h>

#include <uORB/topics/uORBTopics.hpp>
#include "topic_listener.hpp"

// Amount of time to wait when listening for a message, before giving up.
static constexpr float MESSAGE_TIMEOUT_S = 2.0f;

extern "C" __EXPORT int listener_main(int argc, char *argv[]);

static void usage();

void listener(const orb_id_t &id, unsigned num_msgs, int topic_instance,
	      unsigned topic_interval)
{

	if (topic_instance == -1 && num_msgs == 1) {
		// first count the number of instances
		int instances = 0;

		for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
			if (orb_exists(id, i) == PX4_OK) {
				instances++;
			}
		}

		if (instances == 1) {
			PX4_INFO_RAW("\nTOPIC: %s\n", id->o_name);
			int sub = orb_subscribe(id);
			listener_print_topic(id, sub);
			orb_unsubscribe(sub);

		} else if (instances > 1) {
			PX4_INFO_RAW("\nTOPIC: %s %d instances\n", id->o_name, instances);

			for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
				if (orb_exists(id, i) == PX4_OK) {
					PX4_INFO_RAW("\nInstance %d:\n", i);
					int sub = orb_subscribe_multi(id, i);
					listener_print_topic(id, sub);
					orb_unsubscribe(sub);
				}
			}
		}

		if (instances == 0) {
			PX4_INFO_RAW("never published\n");
		}

	} else {
		// default to the first instance if not specified
		if (topic_instance == -1) {
			topic_instance = 0;
		}

		if (orb_exists(id, topic_instance) != 0) {
			PX4_INFO_RAW("never published\n");
			return;
		}

		int sub = orb_subscribe_multi(id, topic_instance);
		orb_set_interval(sub, topic_interval);

		unsigned msgs_received = 0;

		struct pollfd fds[2] {};
		// Poll for user input (for q or escape)
		fds[0].fd = 0; /* stdin */
		fds[0].events = POLLIN;
		// Poll the UOrb subscription
		fds[1].fd = sub;
		fds[1].events = POLLIN;

		while (msgs_received < num_msgs) {

			if (poll(&fds[0], 2, int(MESSAGE_TIMEOUT_S * 1000)) > 0) {

				// Received character from stdin
				if (fds[0].revents & POLLIN) {
					char c = 0;
					int ret = read(0, &c, 1);

					if (ret) {
						return;
					}

					switch (c) {
					case 0x03: // ctrl-c
					case 0x1b: // esc
					case 'q':
						return;
						/* not reached */
					}
				}

				// Received message from subscription
				if (fds[1].revents & POLLIN) {
					msgs_received++;

					PX4_INFO_RAW("\nTOPIC: %s instance %d #%d\n", id->o_name, topic_instance, msgs_received);

					int ret = listener_print_topic(id, sub);

					if (ret != PX4_OK) {
						PX4_ERR("listener callback failed (%i)", ret);
					}
				}

			} else {
				PX4_INFO_RAW("Waited for %.1f seconds without a message. Giving up.\n", (double) MESSAGE_TIMEOUT_S);
				break;
			}
		}

		orb_unsubscribe(sub);
	}


}

int listener_main(int argc, char *argv[])
{
	if (argc <= 1) {
		usage();
		return 1;
	}

	char *topic_name = argv[1];

	int topic_instance = -1;
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
			num_msgs = 30 * topic_rate; // arbitrary limit (30 seconds at max rate)

		} else {
			num_msgs = 1;
		}
	}


	unsigned topic_interval = 0;

	if (topic_rate != 0) {
		topic_interval = 1000 / topic_rate;
	}

	const orb_metadata *const *topics = orb_get_topics();
	const orb_metadata *found_topic = nullptr;

	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (strcmp(topics[i]->o_name, topic_name) == 0) {
			found_topic = topics[i];
		}
	}

	if (found_topic) {
		listener(found_topic, num_msgs, topic_instance, topic_interval);

	} else {
		PX4_INFO_RAW("Topic %s did not match any known topics\n", topic_name);
		return -1;
	}

	return 0;
}


static void
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
Utility to listen on uORB topics and print the data to the console.

The listener can be exited any time by pressing Ctrl+C, Esc, or Q.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("listener", "command");
	PRINT_MODULE_USAGE_ARG("<topic_name>", "uORB topic name", false);

	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, ORB_MULTI_MAX_INSTANCES - 1, "Topic instance", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 1, 0, 100, "Number of messages", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, 1000, "Subscription rate (unlimited if 0)", true);
}
