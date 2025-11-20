/****************************************************************************
 *
 *   Copyright (C) 2020 PX4 Development Team. All rights reserved.
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

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/topics/gpio_config.h>
#include <uORB/topics/gpio_in.h>
#include <uORB/topics/gpio_out.h>
#include <uORB/topics/gpio_request.h>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>


using namespace time_literals;


class MCP23017 : public device::I2C, public I2CSPIDriver<MCP23017>
{
public:
	MCP23017(const I2CSPIDriverConfig &config);
	~MCP23017() override;

	int init_uorb();
	int init(uint16_t direction, uint16_t state, uint16_t pull_up, uint16_t int_en, uint16_t ref_vals, bool split_int);
	static void print_usage();
	void RunImpl();
	int probe() override;
	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);

protected:

	void print_status() override;
	void exit_and_cleanup() override;

private:

	enum class
	Register : uint8_t {
		IODIRA   = 0x00,
		IODIRB   = 0x01,
		IPOLA    = 0x02,
		IPOLB    = 0x03,
		GPINTENA = 0x04,
		GPINTENB = 0x05,
		DEFVALA  = 0x06,
		DEFVALB  = 0x07,
		INTCONA  = 0x08,
		INTCONB  = 0x09,
		IOCONA   = 0x0a,
		IOCONB   = 0x0b,
		GPPUA    = 0x0c,
		GPPUB    = 0x0d,
		INTFA    = 0x0e,
		INTFB    = 0x0f,
		INTCAPA  = 0x10,
		INTCAPB  = 0x11,
		GPIOA    = 0x12,
		GPIOB    = 0x13,
		OLATA    = 0x14,
		OLATB    = 0x15
	};

	enum class
	PinType : uint8_t {
		Output,
		Input,
		InputPullUp,
	};

	uORB::SubscriptionCallbackWorkItem _gpio_out_sub{this, ORB_ID(gpio_out)};
	uORB::SubscriptionCallbackWorkItem _gpio_request_sub{this, ORB_ID(gpio_request)};
	uORB::SubscriptionCallbackWorkItem _gpio_config_sub{this, ORB_ID(gpio_config)};

	uORB::Publication<gpio_in_s> _to_gpio_in{ORB_ID(gpio_in)};

	perf_counter_t _cycle_perf;

	uint8_t _olatA;
	uint8_t _olatB;
	uint8_t _iodirA;
	uint8_t _iodirB;
	uint8_t _gppuA;
	uint8_t _gppuB;

	int read(uint16_t *mask);
	int write(uint16_t mask_set, uint16_t mask_clear);
	int configure(uint16_t mask, PinType type);

	int read_reg(Register address, uint8_t &data);
	int write_reg(Register address, uint8_t data);
};
