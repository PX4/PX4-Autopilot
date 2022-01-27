/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * @file sht3x.h
 *
 * Header file for SHT3x driver
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 *
 */

#pragma once

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_hygrometer.h>

#define SHT3X_CMD_READ_STATUS			0xF32D
#define SHT3X_CMD_CLEAR_STATUS			0x3041
#define SHT3x_CMD_PERIODIC_2HZ_MEDIUM	0x2220
#define SHT3x_CMD_FETCH_DATA			0xE000
#define SHT3x_CMD_READ_SN				0x3780


using namespace time_literals;

extern "C" __EXPORT int sht3x_main(int argc, char *argv[]);

enum sht3x_state {
	ERROR_GENERAL,
	ERROR_READOUT,
	INIT,
	MEASUREMENT
};
const char *sht_state_names[] = {"General error", "Readout error", "Initialization",
				 "Measurement"
				};


struct sht_info {
	uint32_t serial_number;
};


class SHT3X : public device::I2C, public ModuleParams, public I2CSPIDriver<SHT3X>
{
public:
	SHT3X(const I2CSPIDriverConfig &config);
	~SHT3X() = default;

	static void print_usage();
	void RunImpl();

	int    init() override;
	int    probe() override;
	int    init_sensor();
	void   print_status() override;

	void custom_method(const BusCLIArguments &cli);

	int set_pointer(uint16_t command);
	int read_data(uint16_t command, uint8_t *data_ptr, uint8_t length);
	int write_data(uint16_t command, uint8_t buffer[], uint8_t length);

	void sensor_compouse_msg(bool send);

	uint8_t calc_crc(uint8_t data[2]);

private:

	float measured_temperature = 0;
	float measured_humidity = 0;
	uint32_t measurement_time = 0;
	uint16_t measurement_index = 0;

	sht_info _sht_info;
	int _state = sht3x_state::INIT;
	int _last_state = sht3x_state::INIT;
	uint32_t _time_in_state = hrt_absolute_time();
	uint16_t _last_command = 0;
	uORB::PublicationMulti<sensor_hygrometer_s> _sensor_hygrometer_pub{ORB_ID(sensor_hygrometer)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_SHT3X>) _param_sens_en_sht3x
	)
};
