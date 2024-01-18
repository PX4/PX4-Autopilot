/****************************************************************************
 *
 *   Copyright (c) 2022-2023 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/posix.h>

#include "uxrce_dds_client.h"

// services
#include "vehicle_command_srv.h"

#include <uxr/client/client.h>
#include <uxr/client/util/ping.h>
#include <ucdr/microcdr.h>

#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
# define UXRCE_DDS_CLIENT_UDP 1
#endif

#define STREAM_HISTORY  4
#define BUFFER_SIZE (UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY) // MTU==512 by default

#define PARTICIPANT_XML_SIZE 512

using namespace time_literals;

void on_time(uxrSession *session, int64_t current_time, int64_t received_timestamp, int64_t transmit_timestamp,
	     int64_t originate_timestamp, void *args)
{
	// latest round trip time (RTT)
	int64_t rtt = current_time - originate_timestamp;

	// HRT to AGENT
	int64_t offset_1 = (received_timestamp - originate_timestamp) - (rtt / 2);
	int64_t offset_2 = (transmit_timestamp - current_time) - (rtt / 2);

	session->time_offset = (offset_1 + offset_2) / 2;

	if (args) {
		Timesync *timesync = static_cast<Timesync *>(args);
		timesync->update(current_time / 1000, transmit_timestamp, originate_timestamp);

		session->time_offset = -timesync->offset() * 1000; // us -> ns
	}
}

void on_time_no_sync(uxrSession *session, int64_t current_time, int64_t received_timestamp, int64_t transmit_timestamp,
		     int64_t originate_timestamp, void *args)
{
	session->time_offset = 0;
}


void on_request(
	uxrSession *session,
	uxrObjectId object_id,
	uint16_t request_id,
	SampleIdentity *sample_id,
	ucdrBuffer *ub,
	uint16_t length,
	void *args)
{
	(void) request_id;
	(void) length;
	(void) args;

	const int64_t time_offset_us = session->time_offset / 1000; // ns -> us

	UxrceddsClient *client = (UxrceddsClient *)args;

	client->process_requests(object_id, sample_id, ub, time_offset_us);
}

UxrceddsClient::UxrceddsClient(Transport transport, const char *device, int baudrate, const char *agent_ip,
			       const char *port, const char *client_namespace) :
	ModuleParams(nullptr),
	_client_namespace(client_namespace)
{
	if (transport == Transport::Serial) {

		int fd = -1;

		for (int attempt = 0; attempt < 3; attempt++) {
			fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

			if (fd < 0) {
				PX4_ERR("open %s failed (%i)", device, errno);
				// sleep before trying again
				px4_usleep(1_s);

			} else {
				break;
			}
		}

		_transport_serial = new uxrSerialTransport();

		if (fd >= 0 && setBaudrate(fd, baudrate) == 0 && _transport_serial) {
			// TODO:
			uint8_t remote_addr = 0; // Identifier of the Agent in the connection
			uint8_t local_addr = 1; // Identifier of the Client in the serial connection

			if (uxr_init_serial_transport(_transport_serial, fd, remote_addr, local_addr)) {
				_comm = &_transport_serial->comm;
				_fd = fd;

			} else {
				PX4_ERR("uxr_init_serial_transport failed");
			}
		}

	} else if (transport == Transport::Udp) {

#if defined(UXRCE_DDS_CLIENT_UDP)
		_transport_udp = new uxrUDPTransport();
		strncpy(_port, port, PORT_MAX_LENGTH - 1);
		strncpy(_agent_ip, agent_ip, AGENT_IP_MAX_LENGTH - 1);

		if (_transport_udp) {
			if (uxr_init_udp_transport(_transport_udp, UXR_IPv4, _agent_ip, _port)) {
				_comm = &_transport_udp->comm;
				_fd = _transport_udp->platform.poll_fd.fd;

			} else {
				PX4_ERR("uxr_init_udp_transport failed");
			}
		}


#else
		PX4_ERR("UDP not supported");
#endif
	}

	_participant_config = static_cast<ParticipantConfig>(_param_uxrce_dds_ptcfg.get());
	_synchronize_timestamps = _param_uxrce_dds_synct.get() > 0;
}

UxrceddsClient::~UxrceddsClient()
{
	delete _subs;
	delete _pubs;

	delete_repliers();

	if (_transport_serial) {
		uxr_close_serial_transport(_transport_serial);
		delete _transport_serial;
	}

	if (_transport_udp) {
		uxr_close_udp_transport(_transport_udp);
		delete _transport_udp;
	}
}

static void fillMessageFormatResponse(const message_format_request_s &message_format_request,
				      message_format_response_s &message_format_response)
{
	message_format_response.protocol_version = message_format_request_s::LATEST_PROTOCOL_VERSION;
	message_format_response.success = false;

	if (message_format_request.protocol_version == message_format_request_s::LATEST_PROTOCOL_VERSION) {
		static_assert(sizeof(message_format_request.topic_name) == sizeof(message_format_response.topic_name), "size mismatch");
		memcpy(message_format_response.topic_name, message_format_request.topic_name,
		       sizeof(message_format_response.topic_name));

		// Get the topic name by searching for the last '/'
		int idx_last_slash = -1;
		bool found_null = false;

		for (int i = 0; i < (int)sizeof(message_format_request.topic_name); ++i) {
			if (message_format_request.topic_name[i] == 0) {
				found_null = true;
				break;
			}

			if (message_format_request.topic_name[i] == '/') {
				idx_last_slash = i;
			}
		}

		if (found_null && idx_last_slash != -1) {
			const char *topic_name = message_format_request.topic_name + idx_last_slash + 1;
			// Find the format
			const orb_metadata *const *topics = orb_get_topics();
			const orb_metadata *topic_meta{nullptr};

			for (size_t i = 0; i < orb_topics_count(); i++) {
				if (strcmp(topic_name, topics[i]->o_name) == 0) {
					topic_meta = topics[i];
					break;
				}
			}

			if (topic_meta) {
				message_format_response.message_hash = topic_meta->message_hash;
				// The topic type is already checked by DDS
				message_format_response.success = true;
			}
		}
	}

	message_format_response.timestamp = hrt_absolute_time();
}

void UxrceddsClient::handleMessageFormatRequest()
{
	message_format_request_s message_format_request;

	if (_message_format_request_sub.update(&message_format_request)) {
		message_format_response_s message_format_response;
		fillMessageFormatResponse(message_format_request, message_format_response);
		_message_format_response_pub.publish(message_format_response);
	}
}

void UxrceddsClient::syncSystemClock(uxrSession *session)
{
	struct timespec ts = {};
	px4_clock_gettime(CLOCK_REALTIME, &ts);

	// UTC timestamps in microseconds
	int64_t system_utc = int64_t(ts.tv_sec) * 1000000LL + int64_t(ts.tv_nsec / 1000L);
	int64_t agent_utc = int64_t(hrt_absolute_time()) + (session->time_offset / 1000LL); // ns to us

	uint64_t delta = abs(system_utc - agent_utc);

	if (delta < 5_s) {
		// Only set the time if it's more than 5 seconds off (matches Mavlink and GPS logic)
		PX4_DEBUG("agents UTC time is %s by %-5" PRId64 "us, not setting clock", agent_utc > system_utc ? "ahead" : "behind",
			  llabs(system_utc - agent_utc));
		return;
	}

	ts.tv_sec = agent_utc / 1_s;
	ts.tv_nsec = (agent_utc % 1_s) * 1000;

	if (px4_clock_settime(CLOCK_REALTIME, &ts)) {
		PX4_ERR("failed setting system clock");

	} else {
		char buf[40];
		struct tm date_time;
		localtime_r(&ts.tv_sec, &date_time);
		strftime(buf, sizeof(buf), "%a %Y-%m-%d %H:%M:%S %Z", &date_time);
		PX4_INFO("successfully set system clock: %s", buf);
	}
}

void UxrceddsClient::run()
{
	if (!_comm) {
		PX4_ERR("init failed");
		return;
	}

	_subs = new SendTopicsSubs();
	_pubs = new RcvTopicsPubs();

	if (!_subs || !_pubs) {
		PX4_ERR("alloc failed");
		return;
	}

	while (!should_exit()) {
		bool got_response = false;

		while (!should_exit() && !got_response) {
			// Sending ping without initing a XRCE session
			got_response = uxr_ping_agent_attempts(_comm, 1000, 1);
		}

		if (!got_response) {
			break;
		}

		// Session
		// The key identifier of the Client. All Clients connected to an Agent must have a different key.
		const uint32_t key = (uint32_t)_param_uxrce_key.get();

		if (key == 0) {
			PX4_ERR("session key must be different from zero");
			return;
		}

		uxrSession session;
		uxr_init_session(&session, _comm, key);

		// void uxr_create_session_retries(uxrSession* session, size_t retries);
		if (!uxr_create_session(&session)) {
			PX4_ERR("uxr_create_session failed");
			return;
		}

		// TODO: uxr_set_status_callback

		// Streams
		// Reliable for setup, afterwards best-effort to send the data (important: need to create all 4 streams)
		uint8_t output_reliable_stream_buffer[BUFFER_SIZE] {};
		uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer,
					   sizeof(output_reliable_stream_buffer), STREAM_HISTORY);

		uint8_t output_data_stream_buffer[2048] {};
		uxrStreamId best_effort_out = uxr_create_output_best_effort_stream(&session, output_data_stream_buffer,
					      sizeof(output_data_stream_buffer));

		uint8_t input_reliable_stream_buffer[BUFFER_SIZE] {};
		uxrStreamId reliable_in = uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer,
					  sizeof(input_reliable_stream_buffer),
					  STREAM_HISTORY);

		uxrStreamId best_effort_in = uxr_create_input_best_effort_stream(&session);

		// Create entities
		uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);

		uint16_t domain_id = _param_uxrce_dds_dom_id.get();

		uint16_t participant_req{};

		if (_participant_config == ParticipantConfig::Custom) {
			// Create participant by reference (XML not required)
			participant_req = uxr_buffer_create_participant_ref(&session, reliable_out, participant_id, domain_id,
					  "px4_participant", UXR_REPLACE);

		} else {
			// Construct participant XML and create participant by XML
			char participant_xml[PARTICIPANT_XML_SIZE];
			int ret = snprintf(participant_xml, PARTICIPANT_XML_SIZE, "%s<name>%s/px4_micro_xrce_dds</name>%s",
					   (_participant_config == ParticipantConfig::LocalHostOnly) ?
					   "<dds>"
					   "<profiles>"
					   "<transport_descriptors>"
					   "<transport_descriptor>"
					   "<transport_id>udp_localhost</transport_id>"
					   "<type>UDPv4</type>"
					   "<interfaceWhiteList><address>127.0.0.1</address></interfaceWhiteList>"
					   "</transport_descriptor>"
					   "</transport_descriptors>"
					   "</profiles>"
					   "<participant>"
					   "<rtps>"
					   :
					   "<dds>"
					   "<participant>"
					   "<rtps>",
					   _client_namespace != nullptr ?
					   _client_namespace
					   :
					   "",
					   (_participant_config == ParticipantConfig::LocalHostOnly) ?
					   "<useBuiltinTransports>false</useBuiltinTransports>"
					   "<userTransports><transport_id>udp_localhost</transport_id></userTransports>"
					   "</rtps>"
					   "</participant>"
					   "</dds>"
					   :
					   "</rtps>"
					   "</participant>"
					   "</dds>"
					  );

			if (ret < 0 || ret >= PARTICIPANT_XML_SIZE) {
				PX4_ERR("create entities failed: namespace too long");
				return;
			}

			participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, domain_id,
					  participant_xml, UXR_REPLACE);
		}

		uint8_t request_status;

		if (!uxr_run_session_until_all_status(&session, 1000, &participant_req, &request_status, 1)) {
			PX4_ERR("create entities failed: participant: %i", request_status);
			return;
		}

		if (!_pubs->init(&session, reliable_out, reliable_in, best_effort_in, participant_id, _client_namespace)) {
			PX4_ERR("pubs init failed");
			return;
		}

		if (!_subs->init(&session, reliable_out, reliable_in, best_effort_in, participant_id, _client_namespace)) {
			PX4_ERR("subs init failed");
			return;
		}

		// create VehicleCommand replier
		if (num_of_repliers < MAX_NUM_REPLIERS) {
			if (add_replier(new VehicleCommandSrv(&session, reliable_out, reliable_in, participant_id, _client_namespace,
							      num_of_repliers))) {
				PX4_ERR("replier init failed");
				return;
			}
		}

		_connected = true;

		// Set time-callback.
		if (_synchronize_timestamps) {
			uxr_set_time_callback(&session, on_time, &_timesync);

		} else {
			uxr_set_time_callback(&session, on_time_no_sync, nullptr);
		}

		uxr_set_request_callback(&session, on_request, this);

		// Spin until sync with the Agent
		while (_synchronize_timestamps) {
			if (uxr_sync_session(&session, 1000) && _timesync.sync_converged()) {
				PX4_INFO("synchronized with time offset %-5" PRId64 "us", session.time_offset / 1000);

				if (_param_uxrce_dds_syncc.get() > 0) {
					syncSystemClock(&session);
				}

				break;
			}

			px4_usleep(10_ms);
		}

		hrt_abstime last_sync_session = 0;
		hrt_abstime last_status_update = hrt_absolute_time();
		hrt_abstime last_ping = hrt_absolute_time();
		int num_pings_missed = 0;
		bool had_ping_reply = false;
		uint32_t last_num_payload_sent{};
		uint32_t last_num_payload_received{};
		int poll_error_counter = 0;

		while (!should_exit() && _connected) {

			/* Wait for topic updates for max 1000 ms (1sec) */
			int poll = px4_poll(&_subs->fds[0], (sizeof(_subs->fds) / sizeof(_subs->fds[0])), 1000);

			/* Handle the poll results */
			if (poll == 0) {
				/* Timeout, no updates in selected uorbs */
				continue;

			} else if (poll < 0) {
				/* Error */
				if (poll_error_counter < 10 || poll_error_counter % 50 == 0) {
					/* Prevent flooding */
					PX4_ERR("ERROR while polling uorbs: %d", poll);
				}

				poll_error_counter++;
				continue;
			}

			_subs->update(&session, reliable_out, best_effort_out, participant_id, _client_namespace);

			// check if there are available replies
			process_replies();

			// Run the session until we receive no more data or up to a maximum number of iterations.
			// The maximum observed number of iterations was 6 (SITL). If we were to run only once, data starts to get
			// delayed, causing registered flight modes to time out.
			for (int i = 0; i < 10; ++i) {
				const uint32_t prev_num_payload_received = _pubs->num_payload_received;
				uxr_run_session_timeout(&session, 0);

				if (_pubs->num_payload_received == prev_num_payload_received) {
					break;
				}
			}

			// time sync session
			if (_synchronize_timestamps && hrt_elapsed_time(&last_sync_session) > 1_s) {
				if (uxr_sync_session(&session, 100) && _timesync.sync_converged()) {
					//PX4_INFO("synchronized with time offset %-5" PRId64 "ns", session.time_offset);
					last_sync_session = hrt_absolute_time();

					if (_param_uxrce_dds_syncc.get() > 0) {
						syncSystemClock(&session);
					}
				}
			}

			handleMessageFormatRequest();

			// Check for a ping response
			/* PONG_IN_SESSION_STATUS */
			if (session.on_pong_flag == 1) {
				had_ping_reply = true;
			}

			const hrt_abstime now = hrt_absolute_time();

			if (now - last_status_update > 1_s) {
				float dt = (now - last_status_update) / 1e6f;
				_last_payload_tx_rate = (_subs->num_payload_sent - last_num_payload_sent) / dt;
				_last_payload_rx_rate = (_pubs->num_payload_received - last_num_payload_received) / dt;
				last_num_payload_sent = _subs->num_payload_sent;
				last_num_payload_received = _pubs->num_payload_received;
				last_status_update = now;
			}

			// Handle ping
			if (now - last_ping > 500_ms) {
				last_ping = now;

				if (had_ping_reply) {
					num_pings_missed = 0;

				} else {
					++num_pings_missed;
				}

				uxr_ping_agent_session(&session, 0, 1);

				had_ping_reply = false;
			}

			if (num_pings_missed > 2) {
				PX4_INFO("No ping response, disconnecting");
				_connected = false;
			}

		}

		delete_repliers();

		uxr_delete_session_retries(&session, _connected ? 1 : 0);
		_last_payload_tx_rate = 0;
		_last_payload_tx_rate = 0;
		_subs->reset();
		_timesync.reset_filter();
	}
}

int UxrceddsClient::setBaudrate(int fd, unsigned baud)
{
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

#ifndef B460800
#define B460800 460800
#endif

	case 460800: speed = B460800; break;

#ifndef B921600
#define B921600 921600
#endif

	case 921600: speed = B921600; break;

#ifndef B1000000
#define B1000000 1000000
#endif

	case 1000000: speed = B1000000; break;

#ifndef B1500000
#define B1500000 1500000
#endif

	case 1500000: speed = B1500000; break;

#ifndef B2000000
#define B2000000 2000000
#endif

	case 2000000: speed = B2000000; break;

#ifndef B2500000
#define B2500000 2500000
#endif

	case 2500000: speed = B2500000; break;

#ifndef B3000000
#define B3000000 3000000
#endif

	case 3000000: speed = B3000000; break;

#ifndef B3500000
#define B3500000 3500000
#endif

	case 3500000: speed = B3500000; break;

#ifndef B4000000
#define B4000000 4000000
#endif

	case 4000000: speed = B4000000; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

	return 0;
}

bool UxrceddsClient::add_replier(SrvBase *replier)
{
	if (num_of_repliers < MAX_NUM_REPLIERS) {
		repliers_[num_of_repliers] = replier;

		num_of_repliers++;
	}

	return false;
}

void UxrceddsClient::process_requests(uxrObjectId object_id, SampleIdentity *sample_id, ucdrBuffer *ub,
				      const int64_t time_offset_us)
{
	for (uint8_t i = 0; i < num_of_repliers; i++) {
		if (object_id.id == repliers_[i]->replier_id_.id
		    && object_id.type == repliers_[i]->replier_id_.type) {
			repliers_[i]->process_request(ub, time_offset_us);
			memcpy(&(repliers_[i]->sample_id_), sample_id, sizeof(repliers_[i]->sample_id_));
			break;
		}
	}
}

void UxrceddsClient::process_replies()
{
	for (uint8_t i = 0; i < num_of_repliers; i++) {
		repliers_[i]->process_reply();
	}
}

void UxrceddsClient::delete_repliers()
{
	for (uint8_t i = 0; i < num_of_repliers; i++) {
		delete (repliers_[i]);
	}

	num_of_repliers = 0;
}

int UxrceddsClient::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UxrceddsClient::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uxrce_dds_client",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      PX4_STACK_ADJUSTED(10000),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int UxrceddsClient::print_status()
{
	PX4_INFO("Running, %s", _connected ? "connected" : "disconnected");
#if defined(UXRCE_DDS_CLIENT_UDP)

	if (_transport_udp != nullptr) {
		PX4_INFO("Using transport:     udp");
		PX4_INFO("Agent IP:            %s", _agent_ip);
		PX4_INFO("Agent port:          %s", _port);
		PX4_INFO("Custom participant:  %s", _participant_config == ParticipantConfig::Custom ? "yes" : "no");
		PX4_INFO("Localhost only:      %s", _participant_config == ParticipantConfig::LocalHostOnly ? "yes" : "no");
	}

#endif

	if (_transport_serial != nullptr) {
		PX4_INFO("Using transport:     serial");
	}

	if (_connected) {
		PX4_INFO("Payload tx:          %i B/s", _last_payload_tx_rate);
		PX4_INFO("Payload rx:          %i B/s", _last_payload_rx_rate);
	}

	return 0;
}

UxrceddsClient *UxrceddsClient::instantiate(int argc, char *argv[])
{
	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	char port[PORT_MAX_LENGTH] = {0};
	char agent_ip[AGENT_IP_MAX_LENGTH] = {0};

#if defined(UXRCE_DDS_CLIENT_UDP)
	Transport transport = Transport::Udp;
#else
	Transport transport = Transport::Serial;
#endif
	const char *device = nullptr;
	int baudrate = 921600;

	const char *client_namespace = nullptr;//"px4";

	while ((ch = px4_getopt(argc, argv, "t:d:b:h:p:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't':
			if (!strcmp(myoptarg, "serial")) {
				transport = Transport::Serial;

			} else if (!strcmp(myoptarg, "udp")) {
				transport = Transport::Udp;

			} else {
				PX4_ERR("unknown transport: %s", myoptarg);
				error_flag = true;
			}

			break;

		case 'd':
			device = myoptarg;
			break;

		case 'b':
			if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				error_flag = true;
			}

			break;

#if defined(UXRCE_DDS_CLIENT_UDP)

		case 'h':
			snprintf(agent_ip, AGENT_IP_MAX_LENGTH, "%s", myoptarg);
			break;

		case 'p':
			snprintf(port, PORT_MAX_LENGTH, "%s", myoptarg);
			break;
#endif // UXRCE_DDS_CLIENT_UDP

		case 'n':
			client_namespace = myoptarg;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

#if defined(UXRCE_DDS_CLIENT_UDP)

	if (port[0] == '\0') {
		// no port specified, use UXRCE_DDS_PRT
		int32_t port_i = 0;
		param_get(param_find("UXRCE_DDS_PRT"), &port_i);

		if (port_i < 0 || port_i > 65535) {
			PX4_ERR("port must be between 0 and 65535");
			return nullptr;
		}

		snprintf(port, PORT_MAX_LENGTH, "%u", (uint16_t)port_i);
	}

	if (agent_ip[0] == '\0') {
		// no agent ip specified, use UXRCE_DDS_AG_IP
		int32_t ip_i = 0;
		param_get(param_find("UXRCE_DDS_AG_IP"), &ip_i);
		snprintf(agent_ip, AGENT_IP_MAX_LENGTH, "%u.%u.%u.%u", static_cast<uint8_t>(((ip_i) >> 24) & 0xff),
			 static_cast<uint8_t>(((ip_i) >> 16) & 0xff),
			 static_cast<uint8_t>(((ip_i) >> 8) & 0xff),
			 static_cast<uint8_t>(ip_i & 0xff));
	}

#endif // UXRCE_DDS_CLIENT_UDP

	if (error_flag) {
		return nullptr;
	}

	if (transport == Transport::Serial) {
		if (!device) {
			PX4_ERR("Missing device");
			return nullptr;
		}
	}

	return new UxrceddsClient(transport, device, baudrate, agent_ip, port, client_namespace);
}

int UxrceddsClient::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
UXRCE-DDS Client used to communicate uORB topics with an Agent over serial or UDP.

### Examples
$ uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
$ uxrce_dds_client start -t udp -h 127.0.0.1 -p 15555
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uxrce_dds_client", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('t', "udp", "serial|udp", "Transport protocol", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('h', nullptr, "<IP>", "Agent IP. If not provided, defaults to UXRCE_DDS_AG_IP", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', -1, 0, 65535, "Agent listening port. If not provided, defaults to UXRCE_DDS_PRT", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Client DDS namespace", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uxrce_dds_client_main(int argc, char *argv[])
{
	return UxrceddsClient::main(argc, argv);
}
