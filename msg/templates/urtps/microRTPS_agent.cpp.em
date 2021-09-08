@###############################################
@#
@# EmPy template for generating microRTPS_agent.cpp file
@#
@###############################################
@# Start of Template
@#
@# Context:
@#  - spec (msggen.MsgSpec) Parsed specification of the .msg file
@#  - ros2_distro (List[str]) ROS2 distro name
@###############################################
@{
import genmsg.msgs
from px_generate_uorb_topic_files import MsgScope # this is in Tools/

try:
    ros2_distro = ros2_distro[0].decode("utf-8")
except AttributeError:
    ros2_distro = ros2_distro[0]

send_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.SEND]
recv_topics = [(alias[idx] if alias[idx] else s.short_name) for idx, s in enumerate(spec) if scope[idx] == MsgScope.RECEIVE]
}@
/****************************************************************************
 *
 * Copyright 2017 Proyectos y Sistemas de Mantenimiento SL (eProsima).
 * Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

#include <thread>
#include <atomic>
#include <getopt.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <ctime>
#include <csignal>
#include <termios.h>
#include <condition_variable>
#include <queue>

#include <fastcdr/Cdr.h>
#include <fastcdr/FastCdr.h>
#include <fastcdr/exceptions/Exception.h>
#include <fastrtps/Domain.h>

#include "microRTPS_transport.h"
#include "microRTPS_timesync.h"
#include "RtpsTopics.h"

@[if ros2_distro]@
#include <rclcpp/rclcpp.hpp>
@[end if]@

// Default values
#define SLEEP_US          1
#define MAX_SLEEP_US      1000000
#define BAUDRATE          460800
#define MAX_DATA_RATE     10000000
#define DEVICE            "/dev/ttyACM0"
#define POLL_MS           1
#define MAX_POLL_MS       1000
#define DEFAULT_RECV_PORT 2020
#define DEFAULT_SEND_PORT 2019
#define DEFAULT_IP        "127.0.0.1"

using namespace eprosima;
using namespace eprosima::fastrtps;

volatile sig_atomic_t running = 1;
std::unique_ptr<Transport_node> transport_node;
std::unique_ptr<RtpsTopics> topics;
uint32_t total_sent = 0, sent = 0;

struct options {
	enum class eTransports {
		UART,
		UDP
	};
	eTransports transport = options::eTransports::UART;
	char device[64] = DEVICE;
	int sleep_us = SLEEP_US;
	uint32_t baudrate = BAUDRATE;
	int poll_ms = POLL_MS;
	uint16_t recv_port = DEFAULT_RECV_PORT;
	uint16_t send_port = DEFAULT_SEND_PORT;
	char ip[16] = DEFAULT_IP;
	bool sw_flow_control = false;
	bool hw_flow_control = false;
	bool verbose_debug = false;
	std::string ns = "";
} _options;

static void usage(const char *name)
{
	printf("usage: %s [options]\n\n"
	       "  -b <baudrate>           UART device baudrate. Defaults to 460800\n"
	       "  -d <device>             UART device. Defaults to /dev/ttyACM0\n"
	       "  -f <sw-flow-control>    Activates UART link SW flow control\n"
	       "  -g <hw-flow-control>    Activates UART link HW flow control\n"
	       "  -i <ip-address>         Target remote IP address for UDP. Defaults to 127.0.0.1\n"
	       "  -n <namespace>          Topics namespace. Identifies the vehicle in a multi-agent network\n"
	       "  -o <poll-ms>            UART polling timeout in milliseconds. Defaults to 1ms\n"
	       "  -r <reception-port>     UDP port for receiving (local). Defaults to 2020\n"
	       "  -s <sending-port>       UDP port for sending (remote). Defaults to 2019\n"
	       "  -t <transport>          [UART|UDP] Defaults to UART\n"
	       "  -v <increase-verbosity> Add more verbosity\n"
	       "  -w <sleep-time-us>      Iteration time for data publishing to the DDS world, in microseconds.\n"
	       "                           Defaults to 1us\n"
	       "     <ros-args>           (ROS2 only) Allows to pass arguments to the timesync ROS2 node.\n"
	       "                           Currently used for setting the usage of simulation time by the node using\n"
	       "                           '--ros-args -p use_sim_time:=true'\n",
	       name);
}

static int parse_options(int argc, char **argv)
{
	static const struct option options[] = {
		{"baudrate", required_argument, NULL, 'b'},
		{"device", required_argument, NULL, 'd'},
		{"sw-flow-control", no_argument, NULL, 'f'},
		{"hw-flow-control", no_argument, NULL, 'g'},
		{"ip-address", required_argument, NULL, 'i'},
		{"namespace", required_argument, NULL, 'n'},
		{"poll-ms", required_argument, NULL, 'o'},
		{"reception-port", required_argument, NULL, 'r'},
		{"sending-port", required_argument, NULL, 's'},
		{"transport", required_argument, NULL, 't'},
		{"increase-verbosity", no_argument, NULL, 'v'},
		{"sleep-time-us", required_argument, NULL, 'w'},
		{"ros-args", required_argument, NULL, 0},
		{"help", no_argument, NULL, 'h'},
		{NULL, 0, NULL, 0}};

	int ch;

	while ((ch = getopt_long(argc, argv, "t:d:w:b:o:r:s:i:fghvn:", options, nullptr)) >= 0) {
		switch (ch) {
		case 't': _options.transport      = strcmp(optarg, "UDP") == 0 ?
                                                  options::eTransports::UDP
                                                  : options::eTransports::UART;    break;

		case 'd': if (nullptr != optarg) strcpy(_options.device, optarg);   break;

		case 'w': _options.sleep_us        = strtoul(optarg, nullptr, 10);  break;

		case 'b': _options.baudrate        = strtoul(optarg, nullptr, 10);  break;

		case 'o': _options.poll_ms         = strtol(optarg, nullptr, 10);   break;

		case 'r': _options.recv_port       = strtoul(optarg, nullptr, 10);  break;

		case 's': _options.send_port       = strtoul(optarg, nullptr, 10);  break;

		case 'i': if (nullptr != optarg) strcpy(_options.ip, optarg);       break;

		case 'f': _options.sw_flow_control = true;                          break;

		case 'g': _options.hw_flow_control = true;                          break;

		case 'h': usage(argv[0]); exit(0);                            	    break;

		case 'v': _options.verbose_debug = true;                            break;

		case 'n': if (nullptr != optarg) _options.ns = std::string(optarg) + "/"; break;

		default:
@[if ros2_distro]@
			break;
@[else]@
			usage(argv[0]);
			return -1;
@[end if]@
		}
	}

	if (_options.poll_ms < POLL_MS) {
		_options.poll_ms = POLL_MS;
		printf("\033[1;33m[   micrortps_agent   ]\tPoll timeout too low. Using %d ms instead\033[0m\n", POLL_MS);
	} else if (_options.poll_ms > MAX_POLL_MS) {
		_options.poll_ms = MAX_POLL_MS;
		printf("\033[1;33m[   micrortps_agent   ]\tPoll timeout too high. Using %d ms instead\033[0m\n", MAX_POLL_MS);
	}

	if (_options.sleep_us > MAX_SLEEP_US) {
		_options.sleep_us = MAX_SLEEP_US;
		printf("\033[1;33m[   micrortps_agent   ]\tPublishing iteration cycle too slow. Using %d us instead\033[0m\n", MAX_SLEEP_US);
	}

	if (_options.hw_flow_control && _options.sw_flow_control) {
		printf("\033[0;31m[   micrortps_agent   ]\tHW and SW flow control set. Please set only one or another\033[0m\n");
		return -1;
	}

	return 0;
}

@[if recv_topics]@
std::atomic<bool> exit_sender_thread(false);
std::condition_variable t_send_queue_cv;
std::mutex t_send_queue_mutex;
std::queue<uint8_t> t_send_queue;

void t_send(void *)
{
	char data_buffer[BUFFER_SIZE] = {};
	uint32_t length = 0;
	uint8_t topic_ID = 255;

	while (running && !exit_sender_thread) {
		std::unique_lock<std::mutex> lk(t_send_queue_mutex);

		while (t_send_queue.empty() && !exit_sender_thread) {
			t_send_queue_cv.wait(lk);
		}

		topic_ID = t_send_queue.front();
		t_send_queue.pop();
		lk.unlock();

		size_t header_length = transport_node->get_header_length();
		/* make room for the header to fill in later */
		eprosima::fastcdr::FastBuffer cdrbuffer(&data_buffer[header_length], sizeof(data_buffer) - header_length);
		eprosima::fastcdr::Cdr scdr(cdrbuffer);

		if (!exit_sender_thread) {
			if (topics->getMsg(topic_ID, scdr)) {
				length = scdr.getSerializedDataLength();

				if (0 < (length = transport_node->write(topic_ID, data_buffer, length))) {
					total_sent += length;
					++sent;
				}
			}
		}
	}
}
@[end if]@

void signal_handler(int signum)
{
	printf("\n\033[1;33m[   micrortps_agent   ]\tInterrupt signal (%d) received.\033[0m\n", signum);
	running = 0;
	transport_node->close();
}

int main(int argc, char **argv)
{
	printf("\033[0;37m--- MicroRTPS Agent ---\033[0m\n");

	if (-1 == parse_options(argc, argv)) {
		printf("\033[1;33m[   micrortps_agent   ]\tEXITING...\033[0m\n");
		return -1;
	}

	topics = std::make_unique<RtpsTopics>();

@[if ros2_distro]@
	// Initialize communications via the rmw implementation and set up a global signal handler.
	rclcpp::init(argc, argv, rclcpp::InitOptions());
@[end if]@

	// register signal SIGINT and signal handler
	signal(SIGINT, signal_handler);

	printf("[   micrortps_agent   ]\tStarting link...\n");

	const char* localhost_only = std::getenv("ROS_LOCALHOST_ONLY");

	if (localhost_only && strcmp(localhost_only, "1") == 0) {
		printf("[   micrortps_agent   ]\tUsing only the localhost network...\n");
	}

	/**
	 * Set the system ID to Mission Computer, in order to identify the agent side
	 *
	 * Note: theoretically a multi-agent system is possible, but this would require
	 * adjustments in the way the timesync is done (would have to create a timesync
	 * instance per agent). Keeping it contained for a 1:1 link for now is reasonable.
	 */
	const uint8_t sys_id = static_cast<uint8_t>(MicroRtps::System::MISSION_COMPUTER);

	switch (_options.transport) {
	case options::eTransports::UART: {
			transport_node = std::make_unique<UART_node>(_options.device, _options.baudrate, _options.poll_ms,
						       _options.sw_flow_control, _options.hw_flow_control, sys_id, _options.verbose_debug);
			printf("[   micrortps_agent   ]\tUART transport: device: %s; baudrate: %d; poll: %dms; flow_control: %s\n",
			       _options.device, _options.baudrate, _options.poll_ms,
			       _options.sw_flow_control ? "SW enabled" : (_options.hw_flow_control ? "HW enabled" : "No"));
		}
		break;

	case options::eTransports::UDP: {
			transport_node = std::make_unique<UDP_node>(_options.ip, _options.recv_port, _options.send_port,
                                                                   sys_id, _options.verbose_debug);
			printf("[   micrortps_agent   ]\tUDP transport: ip address: %s; recv port: %u; send port: %u\n",
			       _options.ip, _options.recv_port, _options.send_port);
		}
		break;

	default:
		printf("\033[0;37m[   micrortps_agent   ]\tEXITING...\033[0m\n");
		return -1;
	}

	if (0 > transport_node->init()) {
		printf("\033[0;37m[   micrortps_agent   ]\tEXITING...\033[0m\n");
		return -1;
	}

	sleep(1);

@[if send_topics]@
	char data_buffer[BUFFER_SIZE] = {};
	int received = 0, loop = 0;
	int length = 0, total_read = 0;
	bool receiving = false;
	uint8_t topic_ID = 255;
	std::chrono::time_point<std::chrono::steady_clock> start, end;
@[end if]@

	// Init timesync
	topics->set_timesync(std::make_shared<TimeSync>(_options.verbose_debug));

@[if recv_topics]@
	topics->init(&t_send_queue_cv, &t_send_queue_mutex, &t_send_queue, _options.ns);
@[end if]@

	running = true;
@[if recv_topics]@
	std::thread sender_thread(t_send, nullptr);
@[end if]@

	while (running) {
@[if send_topics]@
		++loop;
		if (!receiving) { start = std::chrono::steady_clock::now(); }

		// Publish messages received from UART
		if (0 < (length = transport_node->read(&topic_ID, data_buffer, BUFFER_SIZE))) {
			topics->publish(topic_ID, data_buffer, sizeof(data_buffer));
			++received;
			total_read += length;
			receiving = true;
			end = std::chrono::steady_clock::now();
		}
@[else]@
		usleep(_options.sleep_us);
@[end if]@
	}

@[if recv_topics]@
	exit_sender_thread = true;
	t_send_queue_cv.notify_one();
	sender_thread.join();

	std::chrono::duration<double> elapsed_secs = end - start;
	if (received > 0) {
		printf("[   micrortps_agent   ]\tRECEIVED: %d messages - %d bytes; %d LOOPS - %.03f seconds - %.02fKB/s\n",
		       received, total_read, loop, elapsed_secs.count(), static_cast<double>(total_read) / (1000 * elapsed_secs.count()));
	}
@[end if]@
@[if recv_topics]@
	if (sent > 0) {
		printf("[   micrortps_agent   ]\tSENT:     %lu messages - %lu bytes\n", static_cast<unsigned long>(sent),
		       static_cast<unsigned long>(total_sent));
	}
@[end if]@

@[if ros2_distro]@
	rclcpp::shutdown();
@[end if]@
	transport_node.reset();
	topics.reset();

	return 0;
}
