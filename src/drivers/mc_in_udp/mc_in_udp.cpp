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

#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/manual_control_setpoint.h>

#include "bind_socket.hpp"
#include "deserialise_msg.hpp"
#include <stdexcept>


constexpr in_port_t default_port = 51324;


class ManualControlInUDP : public ModuleBase<ManualControlInUDP>//, public ModuleParams
{
public:
	ManualControlInUDP(in_port_t port);

	virtual ~ManualControlInUDP() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ManualControlInUDP *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	uORB::PublicationMulti<manual_control_setpoint_s> _pub_manual_control_input{ORB_ID(manual_control_input)};

	int socketfd;

	std::array<uint8_t, MSG_MAX_SIZE> msg;

	manual_control_setpoint_s ms;

};

int ManualControlInUDP::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int ManualControlInUDP::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}


int ManualControlInUDP::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

ManualControlInUDP *ManualControlInUDP::instantiate(int argc, char *argv[])
{
	in_port_t port = default_port;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			port = (in_port_t)strtol(myoptarg, nullptr, 10);
			break;

		default:
			PX4_WARN("unrecognized flag");
			break;
		}
	}

	ManualControlInUDP *instance = nullptr;

	try {
		instance = new ManualControlInUDP(port);

	} catch (const std::runtime_error &e) {
		PX4_ERR("%s", e.what());
	}

	return instance;
}

ManualControlInUDP::ManualControlInUDP(in_port_t port)
{
	socketfd = bind_socket(port);

	if (socketfd < 0) {
		throw std::runtime_error("failed to bind socket");
	}

	PX4_INFO("listening on UDP port %d", port);
}

void ManualControlInUDP::run()
{
	while (!should_exit()) {
		msg.fill(0);
		const ssize_t msg_size = recv(socketfd, msg.data(), msg.size(), 0);

		if (msg_size == -1) {
			PX4_WARN("Failed to read from socket: %s", strerror(errno));
			continue;
		}

		if (deserialise_msg(msg, (size_t)msg_size, ms)) {
			_pub_manual_control_input.publish(ms);
		}
	}
}

int ManualControlInUDP::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('p', default_port, 0, 65535, "UDP port", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}


extern "C" __EXPORT int mc_in_udp_main(int argc, char *argv[]);
int mc_in_udp_main(int argc, char *argv[])
{
	return ManualControlInUDP::main(argc, argv);
}
