/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file LidarLitePWM.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */
#pragma once

#include "LidarLite.h"

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/topics/pwm_input.h>
#include <uORB/Subscription.hpp>
#include <board_config.h>

#if DIRECT_PWM_OUTPUT_CHANNELS >= 6
#define GPIO_VDD_RANGEFINDER_EN_CHAN 5 // use pin 6
#define LIDAR_LITE_PWM_SUPPORTED

class LidarLitePWM : public LidarLite, public px4::ScheduledWorkItem
{
public:
	LidarLitePWM(const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~LidarLitePWM();

	int init() override;
	void start() override;
	void stop() override;

protected:

	int collect() override;
	int measure() override;

private:

	void Run() override;

	uORB::Subscription _sub_pwm_input{ORB_ID(pwm_input)};

	pwm_input_s _pwm{};
};

#endif
