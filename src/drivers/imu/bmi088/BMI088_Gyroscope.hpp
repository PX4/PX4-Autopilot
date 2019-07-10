/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/spi.h>

#include "Bosch_BMI088_Gyroscope_Registers.hpp"

namespace Bosch_BMI088_Gyroscope
{

class BMI088_Gyroscope : public device::SPI, public px4::ScheduledWorkItem
{
public:
	BMI088_Gyroscope(int bus, uint32_t device, enum Rotation rotation);
	virtual ~BMI088_Gyroscope();

	virtual int     init();

	// Diagnostics - print some basic information about the driver.
	void            print_info();

	void            print_registers();

protected:

	virtual int     probe();

private:

	uint8_t		registerRead(Register reg);
	void		registerWrite(Register reg, uint8_t value);
	void		registerWriteChecked(Register reg, uint8_t value);
	void		registerSetBits(Register reg, uint8_t setbits);
	void		registerClearBits(Register reg, uint8_t clearbits);

	bool		resetFIFO();

	int         	reset();

	/**
	 * Fetch measurements from the sensor
	 */
	void		measure();

	void		Run() override;

	/**
	 * Set the BMI088_gyro measurement range.
	 *
	 * @param max_dps   The maximum DPS value the range must support.
	 * @return      OK if the value can be supported, -EINVAL otherwise.
	 */
	int		set_gyro_range(unsigned max_dps);
	int		gyro_set_sample_rate(float desired_sample_rate_hz);

	/*
	 * check that key registers still have the right value
	 */
	void		check_registers();


	PX4Gyroscope		_px4_gyro;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_interval;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
	static constexpr int BMI088_GYRO_NUM_CHECKED_REGISTERS = 2;
	uint8_t         _checked_values[BMI088_GYRO_NUM_CHECKED_REGISTERS];
	uint8_t         _checked_bad[BMI088_GYRO_NUM_CHECKED_REGISTERS];

	uint8_t		_checked_next{0};
	uint8_t		_register_wait{0};

	uint64_t	_reset_wait{0};

};

} // namespace Bosch_BMI088_Gyroscope
