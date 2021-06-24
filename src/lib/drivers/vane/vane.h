/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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

#include <string.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hall.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <perf/perf_counter.h>
#include <uORB/topics/sensor_hall.h>
#include <uORB/PublicationMulti.hpp>

class __EXPORT Vane : public device::I2C
{
public:
	Vane(int bus, int bus_frequency, int address, unsigned conversion_interval);
	virtual ~Vane();

	int	init() override;

	int	ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

private:
	Vane(const Vane &) = delete;
	Vane &operator=(const Vane &) = delete;

protected:
	int	probe() override;

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	virtual int	measure() = 0;
	virtual int	collect() = 0;

	bool			_sensor_ok;
	int			_measure_interval;
	bool			_collect_phase;

	uORB::PublicationMulti<sensor_hall_s>	_vane_pub{ORB_ID(sensor_hall)};

	int			_vane_orb_class_instance;

	int			_class_instance;

	unsigned		_conversion_interval;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
};
