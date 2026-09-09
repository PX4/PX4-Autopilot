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
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <ctype.h>
#include <string.h>

#include <zenoh-pico.h>

// CycloneDDS CDR Deserializer
#include <dds/cdr/dds_cdrstream.h>

// Auto-generated header to all uORB <-> CDR conversions
#include <uorb_pubsub_factory.hpp>

ModuleBase::Descriptor ZENOH::desc{task_spawn, custom_command, print_usage};

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
	z_internal_null(&_s);
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

#ifdef CONFIG_ZENOH_KEY_TYPE_HASH
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
#else
		return snprintf(keyexpr, KEYEXPR_SIZE, "%" PRId32 "%s/"
				KEYEXPR_MSG_NAME "%s_/TypeHashNotSupported",
				_zenoh_domain_id.get(), topic, type);
#endif
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

#ifdef CONFIG_ZENOH_KEY_TYPE_HASH
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
#else
	return snprintf(keyexpr, KEYEXPR_SIZE,
			"@ros2_lv/%" PRId32 "/"
			"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/"
			"0/11/%s/%%/%%/px4_%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x/%s/"
			KEYEXPR_MSG_NAME "%s_/TypeHashNotSupported"
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
			topic_lv, type_camel_case
		       );
#endif
}

bool ZENOH::parseTcpLocator(const char *locator, sockaddr_in &endpoint)
{
	static constexpr char kTcpPrefix[] = TCP_SCHEMA "/";

	if (strncmp(locator, kTcpPrefix, sizeof(kTcpPrefix) - 1) != 0) {
		return false;
	}

	// "<address>:<port>" ends at the metadata or config section of the locator
	static constexpr char kSectionSeparators[] = {LOCATOR_METADATA_SEPARATOR, ENDPOINT_CONFIG_SEPARATOR, '\0'};
	const char *address = locator + sizeof(kTcpPrefix) - 1;
	const size_t address_len = strcspn(address, kSectionSeparators);
	char address_str[kIpv4EndpointStringSize];

	if (address_len >= sizeof(address_str)) {
		return false;
	}

	memcpy(address_str, address, address_len);
	address_str[address_len] = '\0';
	char *port_str = strrchr(address_str, ':');

	if (port_str == nullptr) {
		return false;
	}

	*port_str++ = '\0';
	char *port_end = nullptr;
	const unsigned long port = strtoul(port_str, &port_end, 10);

	if (!isdigit(static_cast<unsigned char>(*port_str)) || *port_end != '\0' || port == 0 || port > UINT16_MAX) {
		return false;
	}

	memset(&endpoint, 0, sizeof(endpoint));
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(port);

	return inet_pton(AF_INET, address_str, &endpoint.sin_addr) == 1;
}

ZENOH::ProbeResult ZENOH::probeEndpoint(const sockaddr_in &endpoint)
{
	const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

	if (fd < 0) {
		PX4_WARN("Endpoint probe socket failed: %d", errno);
		return ProbeResult::Failed;
	}

	ProbeResult result = ProbeResult::Failed;
	const int flags = fcntl(fd, F_GETFL, 0);

	if (flags < 0 || fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
		PX4_WARN("Endpoint probe socket setup failed: %d", errno);

	} else if (connect(fd, reinterpret_cast<const sockaddr *>(&endpoint), sizeof(endpoint)) != 0 && errno != EINPROGRESS) {
		// some stacks report a refused connection or a dead link (NuttX with CONFIG_NET_ARP_SEND) right here
		if (errno == ECONNREFUSED || errno == ENETUNREACH || errno == EHOSTUNREACH || errno == ENETDOWN) {
			result = ProbeResult::Unreachable;

		} else {
			PX4_WARN("Endpoint probe connect failed: %d", errno);
		}

	} else {
		pollfd pfd {};
		pfd.fd = fd;
		pfd.events = POLLOUT;
		int ready = 0;

		// poll in slices so that a stop request is not delayed by the probe timeout
		for (hrt_abstime elapsed = 0; elapsed < kEndpointProbeTimeout && ready == 0 && !should_exit();
		     elapsed += kStopCheckInterval) {
			ready = poll(&pfd, 1, static_cast<int>(kStopCheckInterval / 1_ms));
		}

		if (ready < 0) {
			PX4_WARN("Endpoint probe poll failed: %d", errno);

		} else {
			int error = 0;
			socklen_t error_len = sizeof(error);
			const bool connected_ok = ready > 0 && (pfd.revents & POLLOUT) && !(pfd.revents & (POLLERR | POLLHUP))
						  && getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &error_len) == 0 && error == 0;
			result = connected_ok ? ProbeResult::Reachable : ProbeResult::Unreachable;
		}
	}

	close(fd);

	return result;
}

bool ZENOH::sleepInterruptible(hrt_abstime duration)
{
	for (hrt_abstime elapsed = 0; elapsed < duration && !should_exit(); elapsed += kStopCheckInterval) {
		px4_usleep(kStopCheckInterval);
	}

	return !should_exit();
}

bool ZENOH::waitForEndpoint(const char *locator)
{
	sockaddr_in endpoint;

	if (!parseTcpLocator(locator, endpoint)) {
		return true;
	}

	ProbeResult result = probeEndpoint(endpoint);

	while (result == ProbeResult::Unreachable) {
		if (!_waiting_for_endpoint) {
			PX4_WARN("%s does not accept connections yet, waiting...", locator);
			_waiting_for_endpoint = true;
		}

		if (!sleepInterruptible(kEndpointProbeInterval)) {
			_waiting_for_endpoint = false;
			return false;
		}

		result = probeEndpoint(endpoint);
	}

	// Reachable, or Failed: in both cases let z_open() go ahead and report
	_waiting_for_endpoint = false;

	return true;
}

int ZENOH::setupSession()
{
	char mode[NET_MODE_SIZE] {};
	char locator[NET_LOCATOR_SIZE] {};
	z_owned_config_t config;
	int ret = 0;

	_config.getNetworkConfig(mode, locator);
	// getNetworkConfig() fills the buffers with strncpy() and does not guarantee a terminator
	mode[sizeof(mode) - 1] = '\0';
	locator[sizeof(locator) - 1] = '\0';

	PX4_INFO("Opening session...");

	do {
		if (ret == _Z_ERR_TRANSPORT_OPEN_FAILED) {
			PX4_WARN("Unable to open session, make sure zenohd is running on %s", locator);

		} else if (ret == _Z_ERR_SCOUT_NO_RESULTS) {
			PX4_WARN("Unable to open session, scout no results");

		} else if (ret < 0) {
			PX4_WARN("Unable to open session, ret: %d", ret);
		}

		if (ret != 0 && !sleepInterruptible(kSessionRetryDelay)) {
			return -EINTR;
		}

		// z_open() connects with a blocking connect(). On NuttX that call only returns once the
		// SYN retransmissions are exhausted (tens of seconds) when the link is not up yet, which
		// is the case right after boot, and it cannot be interrupted by 'zenoh stop'. Probe the
		// endpoint with a bounded connect first so that z_open() only runs against a live link.
		if (!waitForEndpoint(locator)) {
			return -EINTR;
		}

		z_config_default(&config);
		zp_config_insert(z_loan_mut(config), Z_CONFIG_MODE_KEY, mode);

		if (locator[0] != 0) {
			zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, locator);

		} else if (strcmp(Z_CONFIG_MODE_PEER, mode) == 0) {
			zp_config_insert(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, Z_CONFIG_MULTICAST_LOCATOR_DEFAULT);
		}

	} while ((ret = z_open(&_s, z_move(config), NULL)) < 0);

	// Start read and lease tasks for zenoh-pico
	if (zp_start_read_task(z_loan_mut(_s), NULL) < 0 || zp_start_lease_task(z_loan_mut(_s), NULL) < 0) {
		PX4_ERR("Unable to start read and lease tasks");
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
	memset(_zenoh_subscribers, 0x0, sizeof(Zenoh_Subscriber *)*_sub_count);

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
					_zenoh_subscribers[i] = NULL;
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
	memset(_zenoh_publishers, 0x0, _pub_count * sizeof(uORB_Zenoh_Publisher *));

	if (_zenoh_publishers) {
		char topic[TOPIC_INFO_SIZE];
		char type[TOPIC_INFO_SIZE];
		int instance;
		z_publisher_options_t global_opts;
		z_publisher_options_default(&global_opts);
		global_opts.congestion_control = (z_congestion_control_t)_zenoh_pub_cc.get();
		global_opts.is_express = (bool)_zenoh_pub_expr.get();
		global_opts.priority = (z_priority_t)_zenoh_pub_prio.get();
#ifdef Z_FEATURE_UNSTABLE_API
		global_opts.reliability = (z_reliability_t)_zenoh_pub_rel.get();
#endif

		for (i = 0; i < _pub_count; i++) {

#ifdef CONFIG_ZENOH_PUB_OPTION_OVERRIDE
			z_publisher_options_t pub_opts = global_opts;
#endif

#ifdef CONFIG_ZENOH_PUB_OPTION_OVERRIDE

			if (_config.getPublisherMapping(topic, type, &instance, &pub_opts)) {
#else

			if (_config.getPublisherMapping(topic, type, &instance)) {
#endif
				_zenoh_publishers[i] = genPublisher(type, instance);
				const uint8_t *rihs_hash = getRIHS01_Hash(type);

				if (rihs_hash && _zenoh_publishers[i] != 0 &&
				    generate_rmw_zenoh_topic_keyexpr(topic, rihs_hash, type, keyexpr) > 0) {
#ifdef CONFIG_ZENOH_PUB_OPTION_OVERRIDE
					_zenoh_publishers[i]->declare_publisher(_s, keyexpr, (uint8_t *)&_px4_guid, &pub_opts);
#else
					_zenoh_publishers[i]->declare_publisher(_s, keyexpr, (uint8_t *)&_px4_guid, &global_opts);
#endif
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
					_zenoh_publishers[i] = NULL;
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

void ZENOH::cleanupSession()
{
	PX4_INFO("Cleaning up Zenoh session...");

	for (int i = 0; i < _sub_count; i++) {
		if (_zenoh_subscribers && _zenoh_subscribers[i]) {
			delete _zenoh_subscribers[i];
		}
	}

	if (_zenoh_subscribers) {
		free(_zenoh_subscribers);
		_zenoh_subscribers = nullptr;
	}

	for (int i = 0; i < _pub_count; i++) {
		if (_zenoh_publishers && _zenoh_publishers[i]) {
			delete _zenoh_publishers[i];
		}
	}

	if (_zenoh_publishers) {
		free(_zenoh_publishers);
		_zenoh_publishers = nullptr;
	}

	if (z_internal_check(_s)) {
		zp_stop_read_task(z_session_loan_mut(&_s));
		zp_stop_lease_task(z_session_loan_mut(&_s));

		z_drop(z_session_move(&_s));
	}

	connected = false;
}

void ZENOH::run()
{
	z_result_t ret;
	int i;
	_pub_count =  _config.getPubCount();
	_sub_count =  _config.getSubCount();
	px4_pollfd_struct_t pfds[_pub_count];

	const int setup_ret = setupSession();

	if (setup_ret < 0) {
		if (setup_ret != -EINTR) {
			PX4_ERR("Failed to setup Zenoh session");
		}

		cleanupSession();
		exit_and_cleanup(desc);
		return;
	}

	connected = true;

	PX4_INFO("Starting reading/writing tasks...");

	if (setupTopics(pfds) < 0) {
		PX4_ERR("Failed to setup topics");
		cleanupSession();
		exit_and_cleanup(desc);
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

	cleanupSession();
	exit_and_cleanup(desc);
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

#ifdef CONFIG_ZENOH_PUB_OPTION_OVERRIDE
	PX4_INFO_RAW("     add publisher  <zenoh_topic> <uorb_topic> [uorb_instance] [options]  Publish uORB topic to Zenoh\n");
	PX4_INFO_RAW("          [options]  key=value pairs: cc=drop|block, express=true|false,\n");
	PX4_INFO_RAW("                                      prio=real_time|interactive_high|interactive_low|data_high|data|data_low|background,\n");
	PX4_INFO_RAW("                                      rel=reliable|best_effort (e.g. \"cc=block,express=true\")\n");
#else
	PX4_INFO_RAW("     add publisher  <zenoh_topic> <uorb_topic> <optional uorb_instance>  Publish uORB topic to Zenoh\n");
#endif
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
	if (connected) {
		PX4_INFO("Connected");

	} else if (_waiting_for_endpoint) {
		PX4_INFO("Connecting, waiting for the endpoint to accept connections");

	} else {
		PX4_INFO("Connecting");
	}

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

int ZENOH::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return ZENOH::instantiate(ac, av);
	}, argc, argv);
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
		desc.task_id = task_id;
		return 0;
	}
}

ZENOH *ZENOH::instantiate(int argc, char *argv[])
{
	return new ZENOH();
}

int zenoh_main(int argc, char *argv[])
{
	return ModuleBase::main(ZENOH::desc, argc, argv);
}
