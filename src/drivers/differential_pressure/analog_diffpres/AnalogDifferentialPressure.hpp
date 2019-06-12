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

/**
 *
 * Driver for analog differential pressure sensor
 *
 */

#pragma once

#ifndef ADC_AIRSPEED_VOLTAGE_CHANNEL
#error "AnalogDifferentialPressure requires ADC and ADC_AIRSPEED_VOLTAGE_CHANNEL"
#endif // ADC_AIRSPEED_VOLTAGE_CHANNEL

#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/differential_pressure/PX4DifferentialPressure.hpp>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_adc.h>

class AnalogDifferentialPressure : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AnalogDifferentialPressure(uint8_t bus, uint8_t address);
	virtual ~AnalogDifferentialPressure() override = default;

	void	start();
	void	stop();

private:

	void	Run() override;

	PX4DifferentialPressure	_px4_diff_press;

	perf_counter_t		_sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t		_comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com err")};


	SENS_DPRES_ANSC


	DevHandle 	_h_adc;				/**< ADC driver handle */

	hrt_abstime	_last_adc{0};			/**< last time we took input from the ADC */

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_DPRES_ANSC>) _param_sens_dpres_ansc
	)
};
