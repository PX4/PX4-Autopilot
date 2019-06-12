/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/differential_pressure/PX4DifferentialPressure.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class SDP3X : public device::I2C, public px4::ScheduledWorkItem
{
public:
	SDP3X(uint8_t bus, uint8_t address);
	virtual ~SDP3X() override = default;

	void	start();
	void	stop();

private:

	void	Run() override;

	int	collect();

	int	probe() override;

	bool	init_sdp3x();

	// Calculate the CRC8 for the sensor payload data
	bool crc(const uint8_t data[], unsigned size, uint8_t checksum);

	// Write a command in Sensirion specific logic
	int write_command(uint16_t command);

	uint16_t _scale{0};

	PX4DifferentialPressure	_px4_diff_press;

	bool			_sensor_ok{true};
	bool			_collect_phase{false};

	perf_counter_t		_sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t		_comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};
};
