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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/filter/MedianFilter.hpp>
#include <px4_arch/micro_hal.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/rpm.h>

using namespace time_literals;

class RPMCapture : public ModuleBase<RPMCapture>, public px4::ScheduledWorkItem
{
public:
	RPMCapture();
	virtual ~RPMCapture();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	static int gpio_interrupt_callback(int irq, void *context, void *arg);

	/** RPMCapture is an interrupt-driven task and needs to be manually stopped */
	static void stop();

private:
	static constexpr hrt_abstime RPM_PULSE_TIMEOUT = 1_s;
	static constexpr uint32_t RPM_MAX_VALUE = 15000;

	void Run() override;

	int _channel{-1};
	uint32_t _rpm_capture_gpio{0};
	uint32_t _pulses_per_revolution{1};
	uint32_t _min_pulse_period_us{1};	///< [us] minimum pulse period
	uORB::Publication<pwm_input_s> _pwm_input_pub{ORB_ID(pwm_input)};
	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};

	hrt_abstime _hrt_timestamp{0};
	hrt_abstime _hrt_timestamp_prev{0};
	uint32_t _period{0};
	uint32_t _error_count{0};
	px4::atomic<bool> _value_processed{true};

	hrt_abstime _timestamp_last_update{0}; ///< to caluclate dt
	AlphaFilter<float> _rpm_filter;
	MedianFilter<float, 5> _rpm_median_filter;
};
