/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *   Author: Pavel Kirienko <pavel.kirienko@gmail.com>
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

#include <cstdlib>
#include <cstring>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <arch/board/board.h>
#include <arch/chip/chip.h>
#include "uavcan_main.hpp"

/*
 * UavcanNode
 */
UavcanNode* UavcanNode::_instance;

int UavcanNode::start(uavcan::NodeID node_id, uint32_t bitrate)
{
	if (_instance != nullptr) {
		warnx("Already started");
		return -1;
	}

	/*
	 * GPIO config.
	 * Forced pull up on CAN2 is required for Pixhawk v1 where the second interface lacks a transceiver.
	 * If no transceiver is connected, the RX pin will float, occasionally causing CAN controller to
	 * fail during initialization.
	 */
	stm32_configgpio (GPIO_CAN1_RX);
	stm32_configgpio (GPIO_CAN1_TX);
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio (GPIO_CAN2_TX);

	/*
	 * CAN driver init
	 */
	static CanInitHelper can;
	static bool can_initialized = false;
	if (!can_initialized) {
		const int can_init_res = can.init(bitrate);
		if (can_init_res < 0) {
			warnx("CAN driver init failed %i", can_init_res);
			return can_init_res;
		}
		can_initialized = true;
	}

	/*
	 * Node init
	 */
	_instance = new UavcanNode(can.driver, uavcan_stm32::SystemClock::instance());
	if (_instance == nullptr) {
		warnx("Out of memory");
		return -1;
	}
	const int node_init_res = _instance->init(node_id);
	if (node_init_res < 0) {
		delete _instance;
		_instance = nullptr;
		warnx("Node init failed %i", node_init_res);
		return node_init_res;
	}

	/*
	 * Start the task. Normally it should never exit.
	 */
	static auto run_trampoline = [](int, char*[]) {return UavcanNode::_instance->run();};
	return task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, StackSize,
	        static_cast<main_t>(run_trampoline), nullptr);
}

int UavcanNode::init(uavcan::NodeID node_id)
{
	uavcan::protocol::SoftwareVersion swver;
	swver.major = 12;                        // TODO fill version info
	swver.minor = 34;
	_node.setSoftwareVersion(swver);

	uavcan::protocol::HardwareVersion hwver;
	hwver.major = 42;                        // TODO fill version info
	hwver.minor = 42;
	_node.setHardwareVersion(hwver);

	_node.setName("org.pixhawk"); // Huh?

	_node.setNodeID(node_id);

	return _node.start();
}

int UavcanNode::run()
{
	_node.setStatusOk();
	while (true) {
		// TODO: ORB multiplexing
		const int res = _node.spin(uavcan::MonotonicDuration::getInfinite());
		if (res < 0) {
			warnx("Spin error %i", res);
			::sleep(1);
		}
	}
	return -1;
}

/*
 * App entry point
 */
static void print_usage()
{
	warnx("usage: uavcan start <node_id> [can_bitrate]");
}

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

int uavcan_main(int argc, char *argv[])
{
	constexpr long DEFAULT_CAN_BITRATE = 1000000;

	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		if (argc < 3) {
			print_usage();
			::exit(1);
		}
		/*
		 * Node ID
		 */
		const int node_id = atoi(argv[2]);
		if (node_id < 0 || node_id > uavcan::NodeID::Max || !uavcan::NodeID(node_id).isUnicast()) {
			warnx("Invalid Node ID %i", node_id);
			::exit(1);
		}
		/*
		 * CAN bitrate
		 */
		long bitrate = 0;
		if (argc > 3) {
			bitrate = atol(argv[3]);
		}
		if (bitrate <= 0) {
			bitrate = DEFAULT_CAN_BITRATE;
		}
		/*
		 * Start
		 */
		warnx("Node ID %i, bitrate %li", node_id, bitrate);
		return UavcanNode::start(node_id, bitrate);
	} else {
		print_usage();
		::exit(1);
	}
	return 0;
}
