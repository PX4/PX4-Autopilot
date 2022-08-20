/****************************************************************************
 *
 *   Copyright (c) 2017-2022 PX4 Development Team. All rights reserved.
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
 *
 * Driver for Sensirion SDP3X Differential Pressure Sensor
 *
 * Datasheet: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/8_Differential_Pressure/Sensirion_Differential_Pressure_Sensors_SDP3x_Digital_Datasheet_V0.8.pdf
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

#define I2C_ADDRESS_1_SDP3X 0x21
#define I2C_ADDRESS_2_SDP3X 0x22
#define I2C_ADDRESS_3_SDP3X 0x23

static constexpr uint32_t I2C_SPEED = 100 * 1000; // 100 kHz I2C serial interface

#define SDP3X_SCALE_TEMPERATURE		200.0f
#define SDP3X_RESET_ADDR		0x00
#define SDP3X_RESET_CMD			0x06
#define SDP3X_CONT_MEAS_AVG_MODE	0x3615
#define SDP3X_CONT_MODE_STOP		0x3FF9

#define SDP3X_SCALE_PRESSURE_SDP31	60
#define SDP3X_SCALE_PRESSURE_SDP32	240
#define SDP3X_SCALE_PRESSURE_SDP33	20

// Measurement rate is 20Hz
#define SPD3X_MEAS_RATE 100
#define CONVERSION_INTERVAL	(1000000 / SPD3X_MEAS_RATE)	/* microseconds */

class SDP3X : public device::I2C, public I2CSPIDriver<SDP3X>
{
public:
	SDP3X(const I2CSPIDriverConfig &config);
	~SDP3X() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	enum class State {
		RequireConfig,
		Configuring,
		Running
	};

	int collect();

	int configure();
	int read_scale();

	bool init_sdp3x();

	/**
	 * Calculate the CRC8 for the sensor payload data
	 */
	bool crc(const uint8_t data[], unsigned size, uint8_t checksum);

	/**
	 * Write a command in Sensirion specific logic
	 */
	int write_command(uint16_t command);

	uint16_t _scale{0};
	const bool _keep_retrying;
	State _state{State::RequireConfig};

	uORB::PublicationMulti<differential_pressure_s> _differential_pressure_pub{ORB_ID(differential_pressure)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": communication errors")};
};
