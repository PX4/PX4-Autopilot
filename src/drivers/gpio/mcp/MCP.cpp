/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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


#include "MCP.hpp"
#include "MCP23009.hpp"
#include "MCP23017.hpp"
#include <px4_platform_common/module.h>






MCP::MCP(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample"))
{


}

MCP::~MCP()
{
	ScheduleClear();
	perf_free(_cycle_perf);
}

int MCP::init_uorb()
{
	if (!_gpio_config_sub.registerCallback() ||
	    !_gpio_request_sub.registerCallback() ||
	    !_gpio_out_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return -1;
	}

	return PX4_OK;
}

void MCP::exit_and_cleanup()
{
	_gpio_config_sub.unregisterCallback();
	_gpio_request_sub.unregisterCallback();
	_gpio_out_sub.unregisterCallback();
}

void MCP::RunImpl()
{
	perf_begin(_cycle_perf);

	gpio_config_s config;

	//printf("my: %ld, given: %ld \n", config.device_id, get_device_id());

	if (_gpio_config_sub.update(&config) && config.device_id == get_device_id()) {
		printf("CONFIG UPDATE \n");
		PinType type = PinType::Input;

		switch (config.config) {
		case config.INPUT_PULLUP:	type = PinType::InputPullUp; break;

		case config.OUTPUT:		type = PinType::Output; break;
		}

		write(config.state, config.mask);
		printf("12321: MCP configure should be called \n");
		configure(config.mask, type);

	}

	gpio_out_s output;

	if (_gpio_out_sub.update(&output) && output.device_id == get_device_id()) {
		printf("OUTPUT UPDATE \n");
		write(output.state, output.mask);
	}

	{
		gpio_in_s _gpio_in;
		_gpio_in.timestamp = hrt_absolute_time();
		_gpio_in.device_id = get_device_id();
		uint16_t input;
		read(&input);
		_gpio_in.state = input;
		_to_gpio_in.publish(_gpio_in);
	}

	perf_end(_cycle_perf);
}

void MCP::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
}


struct init_config_t {
	uint16_t interval;
	uint16_t direction;
	uint16_t state;
	uint16_t pullup;
	uint16_t int_en;
	uint16_t ref_vals;
	bool split_int;
};
