/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * @file io_controller.hpp
 * Simple IOController class
 *
 */
#pragma once

#include <px4_platform_common/app.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/Subscription.hpp>

class IOController : public px4::ScheduledWorkItem
{
public:
	IOController(const char *name, const px4::wq_config_t &config);

	~IOController() {_instance = nullptr;}

	static px4::AppState appState; /* track requests to terminate app */

	static IOController *instance() { return _instance; }

	void print_info();

	static int start();

	void setRate(uint32_t rate);

	uint32_t getRate(uint32_t rate)
	{
		return _rate;
	}
private:
	void Run() override;
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};

	const uint32_t _pwm_mask = ((1u << DIRECT_PWM_OUTPUT_CHANNELS) - 1);

	static constexpr unsigned ScheduleIntervalMs = 1;

	static IOController *_instance;

	uint32_t _rate = 400; /* in Hz */
};
