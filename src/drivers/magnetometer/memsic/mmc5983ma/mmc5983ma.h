/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/i2c_spi_buses.h>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>

// MMC5983MA Registers
#define MMC5983MA_ADDR_XOUT_0           0x00
#define MMC5983MA_ADDR_STATUS_REG       0x08
#define MMC5983MA_ADDR_CTRL_REG0        0x09
#define MMC5983MA_ADDR_CTRL_REG1        0x0A
#define MMC5983MA_ADDR_CTRL_REG2        0x0B
#define MMC5983MA_ADDR_PRODUCT_ID       0x2F
// MMC5983MA Definitions
#define MMC5983MA_PRODUCT_ID            	0x30
#define MMC5983MA_STATUS_REG_MEAS_M_DONE (1 << 0)
#define MMC5983MA_CTRL_REG0_TM_M         (1 << 0)
#define MMC5983MA_CTRL_REG0_SET          (1 << 3)
#define MMC5983MA_CTRL_REG0_RESET        (1 << 4)

#define MMC5983MA_CTRL_REG1_BW_200HZ     (0b00000001)
#define MMC5983MA_CTRL_REG1_SW_RESET     (1 << 7)

extern device::Device *MMC5983MA_I2C_interface(const I2CSPIDriverConfig &config);

class MMC5983MA : public I2CSPIDriver<MMC5983MA>
{
public:
	MMC5983MA(device::Device *interface, const I2CSPIDriverConfig &config);
	virtual ~MMC5983MA();

	struct SensorData {
		uint8_t xout0;
		uint8_t xout1;
		uint8_t yout0;
		uint8_t yout1;
		uint8_t zout0;
		uint8_t zout1;
		uint8_t xyzout2;
		uint8_t tout;
	};

	enum class State {
		Measure,
		Collect,
	};

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int init();
	void print_status() override;

	void RunImpl();

private:
	void publish_data();

	// Read data
	uint8_t read_register_block(SensorData *data);
	uint8_t read_register(uint8_t reg);
	void write_register(uint8_t reg, uint8_t value);

	device::Device *_interface;
	PX4Magnetometer _px4_mag;
	State _state = State::Measure;
	int _sample_index = 0;
	int _collect_retries = 0;
	SensorData _measurements[2];
	perf_counter_t _sample_count;
	perf_counter_t _comms_errors;
};
