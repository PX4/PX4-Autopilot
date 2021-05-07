/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <microRTPS_transport.h>
#include <microRTPS_client.h>

#include <inttypes.h>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <termios.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/time.h>

extern "C" __EXPORT int micrortps_client_main(int argc, char *argv[]);

static int _rtps_task = -1;
bool _should_exit_task = false;
Transport_node *transport_node = nullptr;
struct options _options;

struct timespec begin;
struct timespec end;

uint64_t total_rcvd{0};
uint64_t total_sent{0};
uint64_t rcvd_last_sec{0};
uint64_t sent_last_sec{0};
uint64_t received{0};
uint64_t sent{0};
int rcv_loop{0};
int send_loop{0};

static void usage(const char *name)
{
	PRINT_MODULE_USAGE_NAME("micrortps_client", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");

	PRINT_MODULE_USAGE_PARAM_STRING('t', "UART", "UART|UDP", "Transport protocol", true);
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyACM0", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 460800, 9600, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', 0, 10, 10000000, "Maximum sending data rate in B/s", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', -1, 1, 1000, "Poll timeout for UART in ms", true);
	PRINT_MODULE_USAGE_PARAM_INT('l', 10000, -1, 100000, "Limit number of iterations until the program exits (-1=infinite)",
				     true);
	PRINT_MODULE_USAGE_PARAM_INT('w', 1, 1, 1000000, "Time in us for which each read from the link iteration sleeps", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 2019, 0, 65536, "Select UDP Network Port for receiving (local)", true);
	PRINT_MODULE_USAGE_PARAM_INT('s', 2020, 0, 65536, "Select UDP Network Port for sending (remote)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('i', "127.0.0.1", "<x.x.x.x>", "Select IP address (remote)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Activate UART link SW flow control", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Activate UART link HW flow control", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('v', "Add more verbosity", true);

	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
}

static int parse_options(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "t:d:l:w:b:m:p:r:s:i:fhv", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 't': _options.transport      = strcmp(myoptarg, "UDP") == 0 ?
							    options::eTransports::UDP
							    : options::eTransports::UART;   break;

		case 'd': if (nullptr != myoptarg) strcpy(_options.device, myoptarg);   break;

		case 'l': _options.loops           =  strtol(myoptarg, nullptr, 10);    break;

		case 'w': _options.sleep_us        = strtoul(myoptarg, nullptr, 10);    break;

		case 'b': {
				int baudrate = 0;

				if (px4_get_parameter_value(myoptarg, baudrate) != 0) {
					PX4_ERR("baudrate parsing failed");
				}

				_options.baudrate = baudrate;

				break;
			}

		case 'm': _options.datarate        = strtoul(myoptarg, nullptr, 10);    break;

		case 'p': _options.poll_ms         = strtoul(myoptarg, nullptr, 10);    break;

		case 'r': _options.recv_port       = strtoul(myoptarg, nullptr, 10);    break;

		case 's': _options.send_port       = strtoul(myoptarg, nullptr, 10);    break;

		case 'i': if (nullptr != myoptarg) strcpy(_options.ip, myoptarg);       break;

		case 'f': _options.sw_flow_control = true;     				break;

		case 'h': _options.hw_flow_control = true;     				break;

		case 'v': _options.verbose_debug   = true;     				break;

		default:
			usage(argv[1]);
			return -1;
		}
	}

	if (_options.datarate > MAX_DATA_RATE) {
		_options.datarate = MAX_DATA_RATE;
		PX4_WARN("Invalid data rate. Using max datarate of %ul B/s", MAX_DATA_RATE);
	}

	if (_options.poll_ms < 1) {
		PX4_WARN("Poll timeout too low, using %ul ms", POLL_MS);
	}

	if (_options.hw_flow_control && _options.sw_flow_control) {
		PX4_ERR("HW and SW flow control set. Please set only one or another");
		return -1;
	}

	return 0;
}

static int micrortps_start(int argc, char *argv[])
{
	if (0 > parse_options(argc, argv)) {
		PX4_INFO("EXITING...");
		_rtps_task = -1;
		return -1;
	}

	switch (_options.transport) {
	case options::eTransports::UART: {
			transport_node = new UART_node(_options.device, _options.baudrate, _options.poll_ms,
						       _options.sw_flow_control, _options.hw_flow_control, _options.verbose_debug);
			PX4_INFO("UART transport: device: %s; baudrate: %" PRIu32 "; sleep: %" PRIu32 "ms; poll: %" PRIu32
				 "ms; flow_control: %s",
				 _options.device, _options.baudrate, _options.sleep_us, _options.poll_ms,
				 _options.sw_flow_control ? "SW enabled" : (_options.hw_flow_control ? "HW enabled" : "No"));
		}
		break;

	case options::eTransports::UDP: {
			transport_node = new UDP_node(_options.ip, _options.recv_port, _options.send_port,
						      _options.verbose_debug);
			PX4_INFO("UDP transport: ip address: %s; recv port: %" PRIu16 "; send port: %" PRIu16 "; sleep: %" PRIu32 "us",
				 _options.ip, _options.recv_port, _options.send_port, _options.sleep_us);

		}
		break;

	default:
		_rtps_task = -1;
		PX4_INFO("EXITING...");
		return -1;
	}

	if (0 > transport_node->init()) {
		PX4_INFO("EXITING...");
		_rtps_task = -1;
		return -1;
	}

	micrortps_start_topics(_options.datarate, begin, total_rcvd, total_sent, sent_last_sec, rcvd_last_sec, received, sent,
			       rcv_loop,
			       send_loop);

	px4_clock_gettime(CLOCK_REALTIME, &end);

	const double elapsed_secs = static_cast<double>(end.tv_sec - begin.tv_sec + (end.tv_nsec - begin.tv_nsec) / 1e9);

	PX4_INFO("RECEIVED: %" PRIu64 " messages in %d LOOPS, %" PRIu64 " bytes in %.03f seconds - avg %.02fKB/s",
		 received, rcv_loop, total_rcvd, elapsed_secs, static_cast<double>(total_rcvd / (1e3 * elapsed_secs)));
	PX4_INFO("SENT: %" PRIu64 " messages in %d LOOPS, %" PRIu64 " bytes in %.03f seconds - avg %.02fKB/s",
		 sent, send_loop, total_sent, elapsed_secs, total_sent / (1e3 * elapsed_secs));

	delete transport_node;

	transport_node = nullptr;

	PX4_INFO("Stopped!");

	fflush(stdout);

	_rtps_task = -1;

	return 0;
}

int micrortps_client_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage(argv[0]);
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		if (_rtps_task != -1) {
			PX4_INFO("Already running");
			return -1;
		}

		_rtps_task = px4_task_spawn_cmd("micrortps_client",
						SCHED_DEFAULT,
						SCHED_PRIORITY_DEFAULT,
						2650,
						(px4_main_t) micrortps_start,
						(char *const *)argv);

		if (_rtps_task < 0) {
			PX4_WARN("Could not start task");
			_rtps_task = -1;
			return -1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (_rtps_task == -1) {
			PX4_INFO("Not running");

		} else {
			px4_clock_gettime(CLOCK_REALTIME, &end);

			const double elapsed_secs = static_cast<double>(end.tv_sec - begin.tv_sec + (end.tv_nsec - begin.tv_nsec) / 1e9);

			printf("\tup and running for %.03f seconds\n", elapsed_secs);
			printf("\tnr. of messages received: %" PRIu64 "\n", received);
			printf("\tnr. of messages sent: %" PRIu64 "\n", sent);
			printf("\ttotal data read: %" PRIu64 " bytes\n", total_rcvd);
			printf("\ttotal data sent: %" PRIu64 " bytes\n", total_sent);
			printf("\trates:\n");
			printf("\t  rx: %.3f kB/s\n", rcvd_last_sec / 1e3);
			printf("\t  tx: %.3f kB/s\n", sent_last_sec / 1e3);
			printf("\t  avg rx: %.3f kB/s\n", total_rcvd / (1e3 * elapsed_secs));
			printf("\t  avg tx: %.3f kB/s\n", total_sent / (1e3 * elapsed_secs));
			printf("\t  tx rate max:");

			if (_options.datarate != 0) {
				printf(" %.1f kB/s\n", _options.datarate / 1e3);

			} else {
				printf(" Unlimited\n");
			}
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (_rtps_task == -1) {
			PX4_INFO("Not running");
			return -1;
		}

		_should_exit_task = true;

		if (nullptr != transport_node) { transport_node->close(); }

		_rtps_task = -1;
		return 0;
	}

	usage(argv[0]);
	return -1;
}
