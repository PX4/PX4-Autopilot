/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

		//fprintf(stderr, "time_offset: %ld, timesync: %ld, diff: %ld\n", session->time_offset/1000, timesync->offset(), session->time_offset/1000 + timesync->offset());

		session->time_offset = -timesync->offset() * 1000; // us -> ns
	}
}

UxrceddsClient::UxrceddsClient(Transport transport, const char *device, int baudrate, const char *agent_ip,
			       const char *port, bool localhost_only, bool custom_participant, const char *client_namespace) :
	ModuleParams(nullptr),
	_localhost_only(localhost_only), _custom_participant(custom_participant),
	_client_namespace(client_namespace)
{
	if (transport == Transport::Serial) {

		int fd = -1;

		for (int attempt = 0; attempt < 3; attempt++) {
			fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

			if (fd < 0) {
				PX4_ERR("open %s failed (%i)", device, errno);
				// sleep before trying again
				usleep(1'000'000);

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
}

UxrceddsClient::~UxrceddsClient()
{
	delete _subs;
	delete _pubs;

	if (_transport_serial) {
		uxr_close_serial_transport(_transport_serial);
		delete _transport_serial;
	}

	if (_transport_udp) {
		uxr_close_udp_transport(_transport_udp);
		delete _transport_udp;
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
		const uint32_t key = (uint32_t)_param_xrce_key.get();

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
		(void)reliable_in;

		uxrStreamId best_effort_in = uxr_create_input_best_effort_stream(&session);
		(void)best_effort_in;

		// Create entities
		uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);

		uint16_t domain_id = _param_xrce_dds_dom_id.get();

		// const char *participant_name = "px4_micro_xrce_dds";
		// uint16_t participant_req = uxr_buffer_create_participant_bin(&session, reliable_out, participant_id, domain_id,
		// 			   participant_name, UXR_REPLACE);

		char participant_xml[PARTICIPANT_XML_SIZE];
		int ret = snprintf(participant_xml, PARTICIPANT_XML_SIZE, "%s<name>%s/px4_micro_xrce_dds</name>%s",
				   _localhost_only ?
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
				   _localhost_only ?
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


		uint16_t participant_req{};

		if (_custom_participant) {
			participant_req = uxr_buffer_create_participant_ref(&session, reliable_out, participant_id, domain_id,
					  "px4_participant", UXR_REPLACE);

		} else {
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

		_connected = true;

		// Set time-callback.
		uxr_set_time_callback(&session, on_time, &_timesync);

		// Synchronize with the Agent
		bool synchronized = false;

		while (!synchronized) {
			synchronized = uxr_sync_session(&session, 1000);

			if (synchronized) {
				PX4_INFO("synchronized with time offset %-5" PRId64 "us", session.time_offset / 1000);
				//sleep(1);

			} else {
				usleep(10000);
			}
		}

		hrt_abstime last_sync_session = 0;
		hrt_abstime last_status_update = hrt_absolute_time();
		hrt_abstime last_ping = hrt_absolute_time();
		int num_pings_missed = 0;
		bool had_ping_reply = false;
		uint32_t last_num_payload_sent{};
		uint32_t last_num_payload_received{};

		while (!should_exit() && _connected) {

			_subs->update(&session, reliable_out, best_effort_out, participant_id, _client_namespace);

			uxr_run_session_timeout(&session, 0);

			// time sync session
			if (hrt_elapsed_time(&last_sync_session) > 1_s) {
				if (uxr_sync_session(&session, 100)) {
					//PX4_INFO("synchronized with time offset %-5" PRId64 "ns", session.time_offset);
					last_sync_session = hrt_absolute_time();
				}
			}

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

			px4_usleep(1000);
		}

		uxr_delete_session_retries(&session, _connected ? 1 : 0);
		_last_payload_tx_rate = 0;
		_last_payload_tx_rate = 0;
		_subs->reset();
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
		PX4_INFO("Using transport: udp");
		PX4_INFO("Agent IP: %s", _agent_ip);
		PX4_INFO("Agent port: %s", _port);

	}

#endif

	if (_transport_serial != nullptr) {
		PX4_INFO("Using transport: serial");
	}

	if (_connected) {
		PX4_INFO("Payload tx: %i B/s", _last_payload_tx_rate);
		PX4_INFO("Payload rx: %i B/s", _last_payload_rx_rate);
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

	bool localhost_only = false;
	bool custom_participant = false;

	const char *client_namespace = nullptr;//"px4";

	while ((ch = px4_getopt(argc, argv, "t:d:b:h:p:lcn:", &myoptind, &myoptarg)) != EOF) {
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

		case 'l':
			localhost_only = true;
			break;

		case 'c':
			custom_participant = true;
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

	return new UxrceddsClient(transport, device, baudrate, agent_ip, port, localhost_only, custom_participant,
				  client_namespace);
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
	PRINT_MODULE_USAGE_PARAM_FLAG('l', "Restrict to localhost (use in combination with ROS_LOCALHOST_ONLY=1)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('c', "Use custom participant config (profile_name=\"px4_participant\")", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, nullptr, "Client DDS namespace", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uxrce_dds_client_main(int argc, char *argv[])
{
	return UxrceddsClient::main(argc, argv);
}
