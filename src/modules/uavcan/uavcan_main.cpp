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

extern "C" __EXPORT int uavcan_main(int argc, char *argv[]);

namespace
{

uavcan_stm32::CanInitHelper<> can_driver;

void print_usage()
{
	warnx("usage: uavcan start [can_bitrate]");
}

int test_thread(int argc, char *argv[])
{
	/*
	 * Forced pull up on CAN2 is required for Pixhawk v1 where the second interface lacks a transceiver.
	 * If no transceiver is connected, the RX pin will float, occasionally causing CAN controller to
	 * fail during initialization.
	 */
	stm32_configgpio(GPIO_CAN1_RX);
	stm32_configgpio(GPIO_CAN1_TX);
	stm32_configgpio(GPIO_CAN2_RX | GPIO_PULLUP);
	stm32_configgpio(GPIO_CAN2_TX);
	int res = can_driver.init(1000000);
	if (res < 0)
	{
		errx(res, "CAN driver init failed");
	}
	while (true)
	{
		::sleep(1);
		auto iface = static_cast<uavcan::ICanIface*>(can_driver.driver.getIface(0));
		res = iface->send(uavcan::CanFrame(), uavcan_stm32::clock::getMonotonic(), 0);
		warnx("published %i, pending %u", res, can_driver.driver.getIface(0)->getRxQueueLength());
	}
	return 0;
}

}

int uavcan_main(int argc, char *argv[])
{
	if (argc < 2) {
		print_usage();
		::exit(1);
	}

	if (!std::strcmp(argv[1], "start")) {
		static bool started = false;
		if (started)
		{
			warnx("already started");
			::exit(1);
		}
		started = true;
		(void)task_spawn_cmd("uavcan", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 3000,
			static_cast<main_t>(&test_thread), const_cast<const char**>(argv));
		return 0;
	} else {
		print_usage();
		::exit(1);
	}
	return 0;
}
