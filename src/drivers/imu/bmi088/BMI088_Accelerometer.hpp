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

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/spi.h>

#include "Bosch_BMI088_Accelerometer_Registers.hpp"

namespace Bosch_BMI088_Accelerometer
{

class BMI088_Accelerometer : public device::SPI, public px4::ScheduledWorkItem
{
public:
	BMI088_Accelerometer(int bus, uint32_t device, enum Rotation rotation);
	virtual ~BMI088_Accelerometer();

	virtual int     init();

	void    	Run() override;

	bool		Start();
	bool		Stop();

	void            print_info();
	void            print_registers();

	void		data_ready_perf() { _timestamp_drdy = hrt_absolute_time(); perf_count(_drdy_interval_perf); }

protected:

	virtual int     probe();

private:

	uint8_t		registerRead(Register reg);
	void		registerWrite(Register reg, uint8_t value);
	bool		registerWriteVerified(Register reg, uint8_t value);
	bool		registerSetBits(Register reg, uint8_t setbits);
	bool		registerClearBits(Register reg, uint8_t clearbits);

	bool		resetFIFO();

	int         	reset();

	PX4Accelerometer	_px4_accel;

	hrt_abstime		_timestamp_drdy{0};

	hrt_abstime		_last_temperature_update{0};

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_interval;
	perf_counter_t		_drdy_interval_perf;
	perf_counter_t		_fifo_reset_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;

};

} // namespace Bosch_BMI088_Accelerometer
