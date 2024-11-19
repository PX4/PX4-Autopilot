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

// IIS2MDC Registers
#define IIS2MDC_ADDR_CFG_REG_A  0x60
#define IIS2MDC_ADDR_CFG_REG_B  0x61
#define IIS2MDC_ADDR_CFG_REG_C  0x62
#define IIS2MDC_ADDR_STATUS_REG 0x67
#define IIS2MDC_ADDR_OUTX_L_REG 0x68
#define IIS2MDC_ADDR_WHO_AM_I   0x4F

// IIS2MDC Definitions
#define IIS2MDC_WHO_AM_I         0b01000000
#define IIS2MDC_STATUS_REG_READY 0b00001111
// CFG_REG_A
#define COMP_TEMP_EN    (1 << 7)
#define MD_CONTINUOUS   (0 << 0)
#define ODR_100         ((1 << 3) | (1 << 2))
// CFG_REG_B
#define OFF_CANC        (1 << 1)
// CFG_REG_C
#define BDU             (1 << 4)

extern device::Device *IIS2MDC_I2C_interface(const I2CSPIDriverConfig &config);

class IIS2MDC : public I2CSPIDriver<IIS2MDC>
{
public:
	IIS2MDC(device::Device *interface, const I2CSPIDriverConfig &config);
	virtual ~IIS2MDC();

	struct SensorData {
		uint8_t xout0;
		uint8_t xout1;
		uint8_t yout0;
		uint8_t yout1;
		uint8_t zout0;
		uint8_t zout1;
		uint8_t tout0;
		uint8_t tout1;
	};

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	int init();
	void print_status() override;

	void RunImpl();

private:
	uint8_t read_register_block(SensorData *data);
	uint8_t read_register(uint8_t reg);
	void write_register(uint8_t reg, uint8_t value);

	device::Device *_interface;
	PX4Magnetometer _px4_mag;
	perf_counter_t _sample_count;
	perf_counter_t _comms_errors;
};
