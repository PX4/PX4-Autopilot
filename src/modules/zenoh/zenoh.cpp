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
#include "zenoh-pico/api/macros.h"
#include "zenoh-pico/api/primitives.h"
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

void toCamelCase(char *input)
{
	bool capitalizeNext = true; // Capitalize the first letter
	int j = 0;

	for (int i = 0; input[i] != '\0'; i++) {
		if (input[i] == '_') {
			capitalizeNext = true; // Next letter should be capitalized

		} else {
			if (capitalizeNext && isalpha(input[i])) {
				input[j++] = toupper(input[i]);
				capitalizeNext = false;

			} else {
				input[j++] = input[i];
			}
		}
	}

	input[j] = '\0'; // Null-terminate the input string
}


ZENOH::ZENOH():
	ModuleParams(nullptr)
{

}

ZENOH::~ZENOH()
{

}

int ZENOH::generate_rmw_zenoh_node_liveliness_keyexpr(const z_id_t *id, char *keyexpr)
{
	return snprintf(keyexpr, KEYEXPR_SIZE,
			"@ros2_lv/0/%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/0/0/NN/%%/%%/"
			"px4_%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
			id->id[0], id->id[1],  id->id[2], id->id[3], id->id[4], id->id[5], id->id[6],
			id->id[7], id->id[8],  id->id[9], id->id[10], id->id[11], id->id[12], id->id[13],
			id->id[14], id->id[15],
			_px4_guid[0], _px4_guid[1], _px4_guid[2], _px4_guid[3],
			_px4_guid[4], _px4_guid[5], _px4_guid[6], _px4_guid[7],
			_px4_guid[8], _px4_guid[9], _px4_guid[10], _px4_guid[11],
			_px4_guid[12], _px4_guid[13], _px4_guid[14], _px4_guid[15]);
}

int ZENOH::generate_rmw_zenoh_topic_keyexpr(const char *topic, const uint8_t *rihs_hash, char *type, char *keyexpr)
{
	const char *type_name = getTypeName(type);

	if (type_name) {
		strncpy(type, type_name, TOPIC_INFO_SIZE);
		toCamelCase(type); // Convert uORB type to camel case
		return snprintf(keyexpr, KEYEXPR_SIZE, "%" PRId32 "%s/"
				KEYEXPR_MSG_NAME "%s_/RIHS01_"
				"%02x%02x%02x%02x%02x%02x%02x%02x"
				"%02x%02x%02x%02x%02x%02x%02x%02x"
				"%02x%02x%02x%02x%02x%02x%02x%02x"
				"%02x%02x%02x%02x%02x%02x%02x%02x",
				_zenoh_domain_id.get(), topic, type,
				rihs_hash[0], rihs_hash[1], rihs_hash[2], rihs_hash[3],
				rihs_hash[4], rihs_hash[5], rihs_hash[6], rihs_hash[7],
				rihs_hash[8], rihs_hash[9], rihs_hash[10], rihs_hash[11],
				rihs_hash[12], rihs_hash[13], rihs_hash[14], rihs_hash[15],
				rihs_hash[16], rihs_hash[17], rihs_hash[18], rihs_hash[19],
				rihs_hash[20], rihs_hash[21], rihs_hash[22], rihs_hash[23],
				rihs_hash[24], rihs_hash[25], rihs_hash[26], rihs_hash[27],
				rihs_hash[28], rihs_hash[29], rihs_hash[30], rihs_hash[31]
			       );
	}

	return -1;
}

int ZENOH::generate_rmw_zenoh_topic_liveliness_keyexpr(const z_id_t *id, const char *topic, const uint8_t *rihs_hash,
		char *type_camel_case, char *keyexpr, const char *entity_str)
{
	// NOT REALLY COMPLIANT WITH RMW_ZENOH_CPP but get's the job done
	// TODO build a correct keyexpr

	char topic_lv[TOPIC_INFO_SIZE];
	char *str = &topic_lv[0];

	strncpy(topic_lv, topic, sizeof(topic_lv));

	while (*str) {
		if (*str == '/') {
			*str = '%';
		}

		str++;
	}

	return snprintf(keyexpr, KEYEXPR_SIZE,
			"@ros2_lv/%" PRId32 "/"
			"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/"
			"0/11/%s/%%/%%/px4_%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/%s/"
			KEYEXPR_MSG_NAME "%s_/RIHS01_"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"%02x%02x%02x%02x%02x%02x%02x%02x"
			"/::,7:,:,:,,",
			_zenoh_domain_id.get(),
			id->id[0], id->id[1],  id->id[2], id->id[3], id->id[4], id->id[5], id->id[6],
			id->id[7], id->id[8],  id->id[9], id->id[10], id->id[11], id->id[12], id->id[13],
			id->id[14], id->id[15],
			entity_str,
			_px4_guid[0], _px4_guid[1], _px4_guid[2], _px4_guid[3],
			_px4_guid[4], _px4_guid[5], _px4_guid[6], _px4_guid[7],
			_px4_guid[8], _px4_guid[9], _px4_guid[10], _px4_guid[11],
			_px4_guid[12], _px4_guid[13], _px4_guid[14], _px4_guid[15],
			topic_lv, type_camel_case,
			rihs_hash[0], rihs_hash[1], rihs_hash[2], rihs_hash[3],
			rihs_hash[4], rihs_hash[5], rihs_hash[6], rihs_hash[7],
			rihs_hash[8], rihs_hash[9], rihs_hash[10], rihs_hash[11],
			rihs_hash[12], rihs_hash[13], rihs_hash[14], rihs_hash[15],
			rihs_hash[16], rihs_hash[17], rihs_hash[18], rihs_hash[19],
			rihs_hash[20], rihs_hash[21], rihs_hash[22], rihs_hash[23],
			rihs_hash[24], rihs_hash[25], rihs_hash[26], rihs_hash[27],
			rihs_hash[28], rihs_hash[29], rihs_hash[30], rihs_hash[31]
		       );
}

int ZENOH::setupSession()
{
	char mode[NET_MODE_SIZE];
	char locator[NET_LOCATOR_SIZE];
	z_owned_config_t config;
	int ret = 0;

	_config.getNetworkConfig(mode, locator);

	PX4_INFO("Opening session...");

	do {
		z_config_default(&config);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode);

		if (locator[0] != 0) {
			zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, locator);

		} else if (strcmp(Z_CONFIG_MODE_PEER, mode) == 0) {
			zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, Z_CONFIG_MULTICAST_LOCATOR_DEFAULT);
		}

		if (ret == _Z_ERR_TRANSPORT_OPEN_FAILED) {
			PX4_WARN("Unable to open session, make sure zenohd is running on %s", locator);

		} else if (ret == _Z_ERR_SCOUT_NO_RESULTS) {
			PX4_WARN("Unable to open session, scout no results");

		} else if (ret < 0) {
			PX4_WARN("Unable to open session, ret: %d", ret);
		}

		if (ret != 0) {
			sleep(5); // Wait 5 seconds when doing a retry
		}

	} while ((ret = z_open(&_s, z_move(config), NULL)) < 0);

	// Start read and lease tasks for zenoh-pico
	if (zp_start_read_task(z_loan_mut(_s), NULL) < 0 || zp_start_lease_task(z_loan_mut(_s), NULL) < 0) {
		PX4_ERR("Unable to start read and lease tasks");
		z_drop(z_move(_s));
		ret = -EINVAL;
	}

	return ret;
}

int ZENOH::setupTopics(px4_pollfd_struct_t *pfds)
{
	char keyexpr[KEYEXPR_SIZE];
	int i;
	int ret = 0;

#ifndef BOARD_HAS_NO_UUID
	board_get_px4_guid(_px4_guid);
#else
	// TODO Fill ID with something reasonable
	_px4_guid[0] = 0xAA;
	_px4_guid[1] = 0xBB;
	_px4_guid[2] = 0xCC;
#endif

#ifdef CONFIG_ZENOH_RMW_LIVELINESS
	z_id_t self_id = z_info_zid(z_loan(_s));

	if (generate_rmw_zenoh_node_liveliness_keyexpr(&self_id, keyexpr)) {
		z_view_keyexpr_t ke;

		if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
			PX4_ERR("%s is not a valid key expression\n", keyexpr);
			return -1;
		}

		z_owned_liveliness_token_t token;

		if (z_liveliness_declare_token(z_loan(_s), &token, z_loan(ke), NULL) < 0) {
			PX4_ERR("Unable to create liveliness token!\n");
			return -1;
		}
	}

#endif

#ifdef Z_SUBSCRIBE
	_zenoh_subscribers = (Zenoh_Subscriber **)malloc(sizeof(Zenoh_Subscriber *)*_sub_count);

	if (_zenoh_subscribers) {
		char topic[TOPIC_INFO_SIZE];
		char type[TOPIC_INFO_SIZE];
		int instance_no;

		for (i = 0; i < _sub_count; i++) {
			if (_config.getSubscriberMapping(topic, type, &instance_no)) {
				_zenoh_subscribers[i] = genSubscriber(type, instance_no);
				const uint8_t *rihs_hash = getRIHS01_Hash(type);

				if (rihs_hash != NULL && _zenoh_subscribers[i] != 0 &&
				    generate_rmw_zenoh_topic_keyexpr(topic, rihs_hash, type, keyexpr) > 0) {
					_zenoh_subscribers[i]->declare_subscriber(_s, keyexpr);
#ifdef CONFIG_ZENOH_RMW_LIVELINESS

					if (generate_rmw_zenoh_topic_liveliness_keyexpr(&self_id, topic, rihs_hash, type, keyexpr, "MS") > 0) {
						z_view_keyexpr_t ke;

						if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
							PX4_ERR("%s is not a valid key expression\n", keyexpr);
							return -1;
						}

						z_owned_liveliness_token_t token;

						if (z_liveliness_declare_token(z_loan(_s), &token, z_loan(ke), NULL) < 0) {
							PX4_ERR("Unable to create liveliness token!\n");
							return -1;
						}
					}

#endif

				} else {
					PX4_ERR("Could not create a subscriber for type %s", type);
				}

			} else {
				_zenoh_subscribers[i] = NULL;
				PX4_ERR("Error parsing publisher config at index %i", i);
			}
		}

		if (_config.getSubscriberMapping(topic, type, &instance_no) < 0) {
			PX4_WARN("Subscriber mapping parsing error");
		}

		_config.closePubSubMapping();
	}

#endif

#ifdef Z_PUBLISH
	_zenoh_publishers = (uORB_Zenoh_Publisher **)malloc(_pub_count * sizeof(uORB_Zenoh_Publisher *));

	if (_zenoh_publishers) {
		char topic[TOPIC_INFO_SIZE];
		char type[TOPIC_INFO_SIZE];
		int instance;

		for (i = 0; i < _pub_count; i++) {
			if (_config.getPublisherMapping(topic, type, &instance)) {
				_zenoh_publishers[i] = genPublisher(type, instance);
				const uint8_t *rihs_hash = getRIHS01_Hash(type);

				if (rihs_hash && _zenoh_publishers[i] != 0 &&
				    generate_rmw_zenoh_topic_keyexpr(topic, rihs_hash, type, keyexpr) > 0) {
					_zenoh_publishers[i]->declare_publisher(_s, keyexpr, (uint8_t *)&_px4_guid);
					_zenoh_publishers[i]->setPollFD(&pfds[i]);
#ifdef CONFIG_ZENOH_RMW_LIVELINESS

					if (generate_rmw_zenoh_topic_liveliness_keyexpr(&self_id, topic, rihs_hash, type, keyexpr, "MP") > 0) {
						z_view_keyexpr_t ke;

						if (z_view_keyexpr_from_str(&ke, keyexpr) < 0) {
							PX4_ERR("%s is not a valid key expression\n", keyexpr);
							return -1;
						}

						z_owned_liveliness_token_t token;

						if (z_liveliness_declare_token(z_loan(_s), &token, z_loan(ke), NULL) < 0) {
							PX4_ERR("Unable to create liveliness token!\n");
							return -1;
						}
					}

#endif

				} else {
					PX4_ERR("Could not create a publisher for type %s", type);
				}

			} else {
				_zenoh_publishers[i] = NULL;
				PX4_ERR("Error parsing publisher config at index %i", i);
			}
		}

		if (_config.getPublisherMapping(topic, type, &instance) < 0) {
			PX4_WARN("Publisher mapping parsing error");
		}

		_config.closePubSubMapping();
	}

#endif

	return ret;
}

void ZENOH::run()
{
	int8_t ret;
	int i;
	_pub_count =  _config.getPubCount();
	_sub_count =  _config.getSubCount();
	px4_pollfd_struct_t pfds[_pub_count];

	if (setupSession() < 0) {
		PX4_ERR("Failed to setup Zenoh session");
		return;
	}

	PX4_INFO("Starting reading/writing tasks...");

	if (setupTopics(pfds) < 0) {
		PX4_ERR("Failed to setup topics");
		return;
	}

	if (_pub_count == 0) {
		// Nothing to publish but we don't want to stop this thread
		while (!should_exit()) {
			usleep(1000);
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
						PX4_WARN("%s Publisher error %i", _zenoh_publishers[i]->getName(), ret);

					}
				}
			}
		}
	}

	// Exiting cleaning up publisher and subscribers
	for (i = 0; i < _sub_count; i++) {
		if (_zenoh_subscribers[i]) {
			delete _zenoh_subscribers[i];
		}
	}

	free(_zenoh_subscribers);

	for (i = 0; i < _pub_count; i++) {
		if (_zenoh_publishers[i]) {
			delete _zenoh_publishers[i];
		}
	}

	free(_zenoh_publishers);

	// Stop read and lease tasks for zenoh-pico
	zp_stop_read_task(z_session_loan_mut(&_s));
	zp_stop_lease_task(z_session_loan_mut(&_s));

	z_drop(z_session_move(&_s));
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
	PX4_INFO_RAW("     add publisher  <zenoh_topic> <uorb_topic> <optional uorb_instance>  Publish uORB topic to Zenoh\n");
	PX4_INFO_RAW("     add subscriber <zenoh_topic> <uorb_topic> <optional uorb_instance>  Publish Zenoh topic to uORB\n");
	PX4_INFO_RAW("     delete publisher  <zenoh_topic>\n");
	PX4_INFO_RAW("     delete subscriber <zenoh_topic>\n");
	PX4_INFO_RAW("     net           <mode> <locator>            Zenoh network mode\n");
	PX4_INFO_RAW("          <mode>    values: client|peer   \n");
	PX4_INFO_RAW("          <locator> client: locator address e.g. tcp/10.41.10.1:7447#iface=eth0\n");
	PX4_INFO_RAW("                    peer: multicast address e.g. udp/224.0.0.224:7446#iface=eth0\n");
	return 0;
}

int ZENOH::print_status()
{
	PX4_INFO("running");

	PX4_INFO("Publishers");

	if (_zenoh_publishers) {
		for (int i = 0; i < _pub_count; i++) {
			if (_zenoh_publishers[i]) {
				_zenoh_publishers[i]->print();
			}
		}
	}

	PX4_INFO("Subscribers");

	if (_zenoh_subscribers) {
		for (int i = 0; i < _sub_count; i++) {
			if (_zenoh_subscribers[i]) {
				_zenoh_subscribers[i]->print();
			}
		}
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
