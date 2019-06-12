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

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <lib/cdev/CDev.hpp>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/differential_pressure.h>

class PX4DifferentialPressure : public cdev::CDev
{

public:
	PX4DifferentialPressure(uint32_t device_id, uint8_t priority = ORB_PRIO_DEFAULT);
	~PX4DifferentialPressure() override;

	int	ioctl(cdev::file_t *filp, int cmd, unsigned long arg) override;

	void set_device_type(uint8_t devtype);
	void set_error_count(uint64_t error_count) { _differential_pressure_pub.get().error_count = error_count; }
	void set_temperature(float temperature) { _differential_pressure_pub.get().temperature = temperature; }

	void set_sample_rate(unsigned rate);

	void update(hrt_abstime timestamp, float differential_pressure);

	void print_status();

private:

	void configure_filter(float cutoff_freq) { _filter.set_cutoff_frequency(_sample_rate, cutoff_freq); }

	uORB::PublicationMultiData<differential_pressure_s>	_differential_pressure_pub;

	math::LowPassFilter2p	_filter{100, 10};

	float			_calibration_scale{1.0f};
	float			_calibration_offset{0.0f};

	int			_class_device_instance{-1};

	unsigned		_sample_rate{100};

};
