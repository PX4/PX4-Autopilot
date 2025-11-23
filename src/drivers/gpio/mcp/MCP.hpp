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

#include <px4_platform/gpio/mcp23017.hpp>
#include <px4_platform/gpio/mcp.hpp>


using namespace time_literals;

static constexpr uint8_t I2C_ADDRESS_MCP23009 = 0x25;
static constexpr uint8_t I2C_ADDRESS_MCP23017 = 0x27;

enum class
PinType : uint8_t {
	Output,
	Input,
	InputPullUp,
};

struct MCP230XX_config_t {
  uint16_t device_type;
  uint8_t bus;
  uint8_t i2c_addr;
  uint8_t first_minor = 0;
  uint8_t num_pins = 8;
  int num_banks = 1;
  uint16_t interval;
  mcp_gpio_dev_s _gpio_handle[16];

};


class MCP230XX :  public device::I2C, public I2CSPIDriver<MCP230XX> //public I2CSPIDriver<MCP> //, public device::I2C
{
public:
	MCP230XX(const I2CSPIDriverConfig &config);
	virtual ~MCP230XX();
	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();
	void print_status() override;
	int probe();
	virtual int init();


protected:
	void exit_and_cleanup() override;

	MCP230XX_config_t mcp_config;


private:
	uORB::SubscriptionCallbackWorkItem _gpio_out_sub{this, ORB_ID(gpio_out)};
	uORB::SubscriptionCallbackWorkItem _gpio_request_sub{this, ORB_ID(gpio_request)};
	uORB::SubscriptionCallbackWorkItem _gpio_config_sub{this, ORB_ID(gpio_config)};

	uORB::Publication<gpio_in_s> _to_gpio_in{ORB_ID(gpio_in)};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;

	virtual int configure(uint16_t mask, PinType type);
	virtual int read(uint16_t *mask);
	int read_reg(uint8_t address, uint8_t &data);
	int write(uint16_t mask_set, uint16_t mask_clear);
	int write_reg(uint8_t address, uint8_t value);
	int set_up();
	int sanity_check();
	int init_communication();
	int init_uorb();

	virtual void set_params() = 0;
	virtual int get_olat(int bank, uint8_t *addr) = 0;
	virtual int get_gppu(int bank, uint8_t *addr) = 0;
	virtual int get_iodir(int bank, uint8_t *addr) = 0;
	virtual int get_gpio(int bank, uint8_t *addr) = 0;
	virtual int get_probe_reg(uint8_t *addr) = 0;

	uint16_t _olat;
	uint16_t _iodir;
	uint16_t _gppu;

	uint16_t checking_freq = 10;

	enum class STATE : uint8_t {
		INIT_COMMUNICATION,
		CONFIGURE,
		CHECK,
		RUNNING
	} curr_state{STATE::INIT_COMMUNICATION};
};
