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
#include <uORB/topics/vehicle_imu.h>

#include "microdds_client.h"

#include <uxr/client/client.h>
#include <uxr/client/util/ping.h>
#include <ucdr/microcdr.h>

#include <termios.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>

#define STREAM_HISTORY  4
#define BUFFER_SIZE (UXR_CONFIG_SERIAL_TRANSPORT_MTU * STREAM_HISTORY) // MTU==512 by default

using namespace time_literals;

MicroddsClient::MicroddsClient(Transport transport, const char *device, int baudrate, const char *host,
			       const char *port, bool localhost_only)
	: _localhost_only(localhost_only)
{
	if (transport == Transport::Serial) {

		int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (fd < 0) {
			PX4_ERR("open %s failed (%i)", device, errno);
		}

		_transport_serial = new uxrSerialTransport();

		if (fd >= 0 && setBaudrate(fd, baudrate) == 0 && _transport_serial) {
			if (uxr_init_serial_transport(_transport_serial, fd, 0, 1)) {
				_comm = &_transport_serial->comm;
				_fd = fd;

			} else {
				PX4_ERR("uxr_init_serial_transport failed");
			}
		}

	} else if (transport == Transport::Udp) {

#if defined(CONFIG_NET) || defined(__PX4_POSIX)
		_transport_udp = new uxrUDPTransport();

		if (_transport_udp) {
			if (uxr_init_udp_transport(_transport_udp, UXR_IPv4, host, port)) {
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

MicroddsClient::~MicroddsClient()
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

void MicroddsClient::run()
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

	int polling_topic_sub = orb_subscribe(ORB_ID(vehicle_imu));

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
		uxrSession session;
		uxr_init_session(&session, _comm, 0xAAAABBBB);

		if (!uxr_create_session(&session)) {
			PX4_ERR("uxr_create_session failed");
			return;
		}

		// Streams
		// Reliable for setup, afterwards best-effort to send the data (important: need to create all 4 streams)
		uint8_t output_reliable_stream_buffer[BUFFER_SIZE] {};
		uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE,
					   STREAM_HISTORY);
		uint8_t output_data_stream_buffer[1024] {};
		uxrStreamId data_out = uxr_create_output_best_effort_stream(&session, output_data_stream_buffer,
				       sizeof(output_data_stream_buffer));

		uint8_t input_reliable_stream_buffer[BUFFER_SIZE] {};
		uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);
		uxrStreamId input_stream = uxr_create_input_best_effort_stream(&session);

		// Create entities
		uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
		const char *participant_xml = _localhost_only ?
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
					      "<name>default_xrce_participant</name>"
					      "<useBuiltinTransports>false</useBuiltinTransports>"
					      "<userTransports><transport_id>udp_localhost</transport_id></userTransports>"
					      "</rtps>"
					      "</participant>"
					      "</dds>"
					      :
					      "<dds>"
					      "<participant>"
					      "<rtps>"
					      "<name>default_xrce_participant</name>"
					      "</rtps>"
					      "</participant>"
					      "</dds>" ;
		uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0,
					   participant_xml, UXR_REPLACE);

		uint8_t request_status;

		if (!uxr_run_session_until_all_status(&session, 1000, &participant_req, &request_status, 1)) {
			PX4_ERR("create entities failed: participant: %i", request_status);
			return;
		}

		if (!_subs->init(&session, reliable_out, participant_id)) {
			PX4_ERR("subs init failed");
			return;
		}

		if (!_pubs->init(&session, reliable_out, input_stream, participant_id)) {
			PX4_ERR("pubs init failed");
			return;
		}

		_connected = true;

		hrt_abstime last_status_update = hrt_absolute_time();
		hrt_abstime last_ping = hrt_absolute_time();
		int num_pings_missed = 0;
		bool had_ping_reply = false;
		uint32_t last_num_payload_sent{};
		uint32_t last_num_payload_received{};
		bool error_printed = false;
		hrt_abstime last_read = hrt_absolute_time();

		while (!should_exit() && _connected) {
			px4_pollfd_struct_t fds[1];
			fds[0].fd = polling_topic_sub;
			fds[0].events = POLLIN;
			// we could poll on the uart/udp fd as well (on nuttx)
			int pret = px4_poll(fds, 1, 20);

			if (pret < 0) {
				if (!error_printed) {
					PX4_ERR("poll failed (%i)", pret);
					error_printed = true;
				}

			} else if (pret != 0) {
				if (fds[0].revents & POLLIN) {
					vehicle_imu_s data;
					orb_copy(ORB_ID(vehicle_imu), polling_topic_sub, &data);
				}
			}

			_subs->update(data_out);

			hrt_abstime read_start = hrt_absolute_time();

			if (read_start - last_read > 5_ms) {
				last_read = read_start;

				// Read as long as there's data or until a timeout
				pollfd fd_read;
				fd_read.fd = _fd;
				fd_read.events = POLLIN;

				do {
					uxr_run_session_timeout(&session, 0);

					if (session.on_pong_flag == 1 /* PONG_IN_SESSION_STATUS */) { // Check for a ping response
						had_ping_reply = true;
					}
				} while (poll(&fd_read, 1, 0) > 0 && hrt_absolute_time() - read_start < 2_ms);
			}

			hrt_abstime now = hrt_absolute_time();

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

		uxr_delete_session_retries(&session, _connected ? 1 : 0);
		_last_payload_tx_rate = 0;
		_last_payload_tx_rate = 0;
	}

	orb_unsubscribe(polling_topic_sub);
}

int MicroddsClient::setBaudrate(int fd, unsigned baud)
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


int microdds_client_main(int argc, char *argv[])
{
	return MicroddsClient::main(argc, argv);
}

int MicroddsClient::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MicroddsClient::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("microdds_client",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT - 4,
				      PX4_STACK_ADJUSTED(8000),
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int MicroddsClient::print_status()
{
	PX4_INFO("Running, %s", _connected ? "connected" : "disconnected");
	PX4_INFO("Payload tx: %i B/s", _last_payload_tx_rate);
	PX4_INFO("Payload rx: %i B/s", _last_payload_rx_rate);
	return 0;
}

MicroddsClient *MicroddsClient::instantiate(int argc, char *argv[])
{
	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	Transport transport = Transport::Udp;
	const char *device = nullptr;
	const char *ip = "127.0.0.1";
	int baudrate = 921600;
	const char *port = "15555";
	bool localhost_only = false;

	while ((ch = px4_getopt(argc, argv, "t:d:b:h:p:l", &myoptind, &myoptarg)) != EOF) {
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

		case 'h':
			ip = myoptarg;
			break;

		case 'p':
			port = myoptarg;
			break;

		case 'l':
			localhost_only = true;
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

	if (error_flag) {
		return nullptr;
	}

	if (transport == Transport::Serial) {
		if (!device) {
			PX4_ERR("Missing device");
			return nullptr;
		}
	}

	return new MicroddsClient(transport, device, baudrate, ip, port, localhost_only);
}

int MicroddsClient::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
MicroDDS Client used to communicate uORB topics with an Agent over serial or UDP.

### Examples
$ microdds_client start -t serial -d /dev/ttyS3 -b 921600
$ microdds_client start -t udp -h 127.0.0.1 -p 15555
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("microdds_client", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('t', "udp", "serial|udp", "Transport protocol", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 0, 0, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('h', "127.0.0.1", "<IP>", "Host IP", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 15555, 0, 3000000, "Remote Port", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('l', "Restrict to localhost (use in combination with ROS_LOCALHOST_ONLY=1)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
