/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "zenoh.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include <string.h>

#include <zenoh-pico.h>

// CycloneDDS CDR Deserializer
#include <dds/cdr/dds_cdrstream.h>

// Auto-generated header to all uORB <-> CDR conversions
#include <uorb_pubsub_factory.hpp>


#define Z_PUBLISH
#define Z_SUBSCRIBE

extern "C" __EXPORT int zenoh_main(int argc, char *argv[]);

ZENOH::ZENOH():
	ModuleParams(nullptr)
{

}

ZENOH::~ZENOH()
{

}

void ZENOH::run()
{
	char mode[NET_MODE_SIZE];
	char locator[NET_LOCATOR_SIZE];
	int8_t ret;
	int i;

	Zenoh_Config z_config;

	z_config.getNetworkConfig(mode, locator);

	z_owned_config_t config = z_config_default();
	zp_config_insert(z_config_loan(&config), Z_CONFIG_MODE_KEY, z_string_make(mode));

	if (locator[0] != 0) {
		zp_config_insert(z_config_loan(&config), Z_CONFIG_PEER_KEY, z_string_make(locator));

	} else if (strcmp(Z_CONFIG_MODE_PEER, mode) == 0) {
		zp_config_insert(z_config_loan(&config), Z_CONFIG_PEER_KEY, z_string_make(Z_CONFIG_MULTICAST_LOCATOR_DEFAULT));
	}

	PX4_INFO("Opening session...");
	z_owned_session_t s = z_open(z_config_move(&config));

	if (!z_session_check(&s)) {
		PX4_ERR("Unable to open session!");
		return;
	}

	// Start read and lease tasks for zenoh-pico
	if (zp_start_read_task(z_session_loan(&s), NULL) < 0 || zp_start_lease_task(z_session_loan(&s), NULL) < 0) {
		PX4_ERR("Unable to start read and lease tasks");
		return;
	}

#ifdef Z_SUBSCRIBE
	_sub_count =  z_config.getSubCount();
	_zenoh_subscribers = (Zenoh_Subscriber **)malloc(sizeof(Zenoh_Subscriber *)*_sub_count);
	{
		char topic[TOPIC_INFO_SIZE];
		char type[TOPIC_INFO_SIZE];

		for (i = 0; i < _sub_count; i++) {
			z_config.getSubscriberMapping(topic, type);
			_zenoh_subscribers[i] = genSubscriber(type);

			if (_zenoh_subscribers[i] != 0) {
				_zenoh_subscribers[i]->declare_subscriber(z_session_loan(&s), topic);
			}


		}

		if (z_config.getSubscriberMapping(topic, type) < 0) {
			PX4_WARN("Subscriber mapping parsing error");
		}
	}
#endif

#ifdef Z_PUBLISH

	_pub_count =  z_config.getPubCount();
	_zenoh_publishers = (uORB_Zenoh_Publisher **)malloc(_pub_count * sizeof(uORB_Zenoh_Publisher *));
	px4_pollfd_struct_t pfds[_pub_count];

	{
		char topic[TOPIC_INFO_SIZE];
		char type[TOPIC_INFO_SIZE];

		for (i = 0; i < _pub_count; i++) {
			z_config.getPublisherMapping(topic, type);
			_zenoh_publishers[i] = genPublisher(type);

			if (_zenoh_publishers[i] != 0) {
				_zenoh_publishers[i]->declare_publisher(z_session_loan(&s), topic);
				_zenoh_publishers[i]->setPollFD(&pfds[i]);
			}
		}

		if (z_config.getSubscriberMapping(topic, type) < 0) {
			PX4_WARN("Publisher mapping parsing error");
		}
	}

	if (_pub_count == 0) {
		// Nothing to publish but we don't want to stop this thread
		while (!should_exit()) {
			sleep(2);
		}
	}

	while (!should_exit()) {
		int pret = px4_poll(pfds, _pub_count, 100);

		if (pret == 0) {
			//PX4_INFO("Zenoh poll timeout\n");

		} else {
			for (i = 0; i < _pub_count; i++) {
				if (pfds[i].revents & POLLIN) {
					ret = _zenoh_publishers[i]->update();

					if (ret < 0) {
						PX4_WARN("Publisher error %i", ret);

					}
				}
			}
		}
	}

#endif

	// Exiting cleaning up publisher and subscribers
	for (i = 0; i < _sub_count; i++) {
		delete _zenoh_subscribers[i];
	}

	free(_zenoh_subscribers);

	for (i = 0; i < _pub_count; i++) {
		delete _zenoh_publishers[i];
	}

	free(_zenoh_publishers);

	// Stop read and lease tasks for zenoh-pico
	zp_stop_read_task(z_session_loan(&s));
	zp_stop_lease_task(z_session_loan(&s));

	z_close(z_session_move(&s));
	exit_and_cleanup();
}

int ZENOH::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp("config", argv[0]) == 0) {
		Zenoh_Config z_config;

		if (z_config.cli(argc, argv) == 0) {
			return 0;
		}
	}

	return print_usage("Unrecognized command.");
}

int ZENOH::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_USAGE_NAME("zenoh", "driver");
	PRINT_MODULE_DESCRIPTION(R"DESC_STR(
### Description

Zenoh demo bridge
	)DESC_STR");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("config");
	PX4_INFO_RAW("     addpublisher  <zenoh_topic> <uorb_topic>  Publish uORB topic to Zenoh\n");
	PX4_INFO_RAW("     addsubscriber <zenoh_topic> <uorb_topic>  Publish Zenoh topic to uORB\n");
	PX4_INFO_RAW("     net           <mode> <locator>            Zenoh network mode\n");
	PX4_INFO_RAW("          <mode>    values: client|peer   \n");
	PX4_INFO_RAW("          <locator> client: locator address for router\n");
	PX4_INFO_RAW("                    peer: multicast address e.g. udp/224.0.0.225:7447#iface=eth0\n");
	return 0;
}

int ZENOH::print_status()
{
	PX4_INFO("running");

	PX4_INFO("Publishers");

	for (int i = 0; i < _pub_count; i++) {
		_zenoh_publishers[i]->print();
	}

	PX4_INFO("Subscribers");

	for (int i = 0; i < _sub_count; i++) {
		_zenoh_subscribers[i]->print();
	}

	return 0;
}

int ZENOH::task_spawn(int argc, char *argv[])
{

	int task_id = px4_task_spawn_cmd(
			      "zenoh",
			      SCHED_DEFAULT,
			      SCHED_PRIORITY_DEFAULT,
			      4096,
			      &run_trampoline,
			      argv
		      );

	if (task_id < 0) {
		return -errno;

	} else {
		_task_id = task_id;
		return 0;
	}
}

ZENOH *ZENOH::instantiate(int argc, char *argv[])
{
	return new ZENOH();
}

int zenoh_main(int argc, char *argv[])
{
	return ZENOH::main(argc, argv);
}
