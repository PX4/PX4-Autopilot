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
 * @file brushless_rpm.hpp
 * @author Jerry Mailloux <jerry@envgo.com>
 *
 * Driver for brushless RPM sensor.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */
#pragma once

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/rpm.h>
#include <board_config.h>
#include <drivers/device/device.h>
#include <perf/perf_counter.h>

using namespace time_literals;

// Device limits
static constexpr uint32_t BRUSHLESS_RPM_MIN_RPM{1000};
static constexpr uint32_t BRUSHLESS_RPM_MAX_RPM{20000};

class BrushlessRPMPWM : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	BrushlessRPMPWM();
	virtual ~BrushlessRPMPWM();

	int init();
	void start();
	void stop();

	void print_info();

protected:

	int collect();
	int measure();

	void Run() override;

private:
	uint32_t _consecutive_non_zero_readings {0};
	uint32_t _measure_interval {50_ms};

	uint32_t get_measure_interval() const { return _measure_interval; };


	uORB::Subscription _sub_pwm_input{ORB_ID(pwm_input)};

	pwm_input_s 	_pwm{};

	uORB::Publication<rpm_s> _rpm_pub{ORB_ID(rpm)};

	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, "brushless_rpm: read")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RPM_POLE_PAIRS>) _param_rpm_pole_pairs
	)
};
