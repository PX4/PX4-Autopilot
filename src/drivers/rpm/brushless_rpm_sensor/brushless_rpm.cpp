/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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
 * @file BrushlessRPMPWM.cpp
 * @author Jerry Mailloux <jerry@envgo.com>
 *
 * Driver for the brushless RPM sensor - attaches to 2 motor phase wires.
 * (Specific one is ose-83279-011. "Brushless RPM sensor for RCMv2 Telemetry System")
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */
#include <math.h>
#include <px4_arch/io_timer.h>
#include "brushless_rpm.hpp"


BrushlessRPMPWM::BrushlessRPMPWM() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	// figure out the measure interval based on the minimum RPM that can be read, and the
	// number of pole pairs. Add 5ms to account for any latency.  Don't measure
	// any faster than 50ms.
	_measure_interval = (uint32_t)math::max((60000000.0/(_param_rpm_pole_pairs.get()*BRUSHLESS_RPM_MIN_RPM))+5000, 50000.0);
}

BrushlessRPMPWM::~BrushlessRPMPWM()
{
	stop();
	perf_free(_sample_perf);
}

int
BrushlessRPMPWM::init()
{
	start();

	return PX4_OK;
}

void
BrushlessRPMPWM::start()
{
	ScheduleOnInterval(get_measure_interval());
}

void
BrushlessRPMPWM::stop()
{
	ScheduleClear();
}

void
BrushlessRPMPWM::Run()
{
	measure();
}

int
BrushlessRPMPWM::measure()
{
	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	uint32_t rpm;
	pwm_input_s pwm_input;



	if(_sub_pwm_input.update(&pwm_input) && (pwm_input.period != 0)) {
		// when the motor isn't turning, noise on the line causes spurious readings.
		// Get a grouping consecutive non zero readings before starting to report RPM.
		if(_consecutive_non_zero_readings++ > 5) {
			rpm = (60 * 1000000) / pwm_input.period / _param_rpm_pole_pairs.get();
		} else {
			rpm = 0;
		}
	} else {
		// if we aren't getting readings from pwm_input because there are no transitions
		// or pwm period is zero (which I don't think will happen, but we should guard against
		// anyway), set RPM to zero.
		rpm = 0;
		_consecutive_non_zero_readings = 0;
	}

	// publish
	rpm_s msg{};
	msg.indicated_frequency_rpm = (float)rpm;

	// there is no accuracy information coming from this sensor, so set accuracy to 1.
	msg.estimated_accurancy_rpm = 1;
	msg.timestamp = timestamp_sample;
	_rpm_pub.publish(msg);

	perf_end(_sample_perf);
	return PX4_OK;
}

void
BrushlessRPMPWM::print_info()
{
	perf_print_counter(_sample_perf);
	printf("poll interval: %lu \n", get_measure_interval());
}
