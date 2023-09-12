/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdarg.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>

using namespace time_literals;

class PowerMonitorSelectorAuterion : public ModuleBase<PowerMonitorSelectorAuterion>, public px4::ScheduledWorkItem
{

public:
	PowerMonitorSelectorAuterion();
	virtual ~PowerMonitorSelectorAuterion();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

private:

	void Run() override;

	bool init();

	int ina226_probe(uint32_t instance);

	uORB::Subscription	_actuator_armed_sub{ORB_ID(actuator_armed)};		///< system armed control topic

	struct Sensor {
		const char *name;
		const char *i2c_addr;
		const uint8_t bus_number;
		float shunt_value;
		bool started;
		const char *id;
	};

	static constexpr int RUN_INTERVAL{500_ms};
	static constexpr int SENSORS_NUMBER{4};

	Sensor _sensors[SENSORS_NUMBER] = {
		{
			.name = "ina226",
			.i2c_addr = "0x41",
			.bus_number = 1,
			.shunt_value = 0.0008f,
			.started = false,
			.id = "1"
		},
		{
			.name = "ina226",
			.i2c_addr = "0x40",
			.bus_number = 1,
			.shunt_value = 0.0005f,
			.started = false,
			.id = "1"
		},
		{
			.name = "ina226",
			.i2c_addr = "0x41",
			.bus_number = 2,
			.shunt_value = 0.0008f,
			.started = false,
			.id = "2"
		},
		{
			.name = "ina226",
			.i2c_addr = "0x40",
			.bus_number = 2,
			.shunt_value = 0.0005f,
			.started = false,
			.id = "2"
		}
	};
};
