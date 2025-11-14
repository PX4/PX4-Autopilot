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
#include "MCP.hpp"

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


class MCP23009 : public MCP
{
public:
	MCP23009(const I2CSPIDriverConfig &config);
	~MCP23009() override;

	int init(uint16_t direction, uint16_t state, uint16_t pull_up);
	int probe() override;

private:

	enum class
	Register : uint8_t {
		IODIR   = 0x00,
		IPOL    = 0x01,
		GPINTEN = 0x02,
		DEFVAL  = 0x03,
		INTCON  = 0x04,
		IOCON   = 0x05,
		GPPU    = 0x06,
		INTF    = 0x07,
		INTCAP  = 0x08,
		GPIO    = 0x09,
		OLAT    = 0x0a
	};

	uint8_t _olat;
	uint8_t _iodir;
	uint8_t _gppu;

	int read(uint16_t *mask);
	int write(uint16_t mask_set, uint16_t mask_clear);

	int configure(uint16_t mask, PinType type);
	int read_reg(Register address, uint8_t &data);
	int write_reg(Register address, uint8_t data);
};
