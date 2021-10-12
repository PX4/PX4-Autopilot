/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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
 * @file ToneAlarmInterface.cpp
 */

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>
#include <px4_platform_common/defines.h>
#include <systemlib/px4_macros.h>
#include <nuttx/timers/pwm.h>

#ifndef TONE_ALARM_PWM_OUT_PATH
#define TONE_ALARM_PWM_OUT_PATH "/dev/pwmX";
#endif

#define TONE_ALARM_PWM_DUTY 50

namespace ToneAlarmInterface
{

int pwm_fd = 0;

void init()
{
	/* Open the PWM devnode */
	pwm_fd = open(TONE_ALARM_PWM_OUT_PATH, O_RDONLY);

	if (pwm_fd < 0) {
		PX4_ERR("failed to open file %s\n", TONE_ALARM_PWM_OUT_PATH);
	}
}

hrt_abstime start_note(unsigned frequency)
{
	hrt_abstime time_started = 0;

	struct pwm_info_s pwm;
	memset(&pwm, 0, sizeof(struct pwm_info_s));

	pwm.frequency = frequency;
	pwm.channels[0].channel = 1;
	pwm.channels[0].duty = ((uint32_t)(TONE_ALARM_PWM_DUTY << 16) / 100);

	irqstate_t flags = enter_critical_section();

	// Set frequency
	if (ioctl(pwm_fd, PWMIOC_SETCHARACTERISTICS,
		  (unsigned long)((uintptr_t)&pwm)) < 0) {
		PX4_ERR("PWMIOC_SETCHARACTERISTICS) failed: %d\n", errno);
	}

	time_started = hrt_absolute_time();

	// Start
	if (ioctl(pwm_fd, PWMIOC_START, 0) < 0) {
		PX4_ERR("PWMIOC_START failed: %d\n", errno);
	}

	leave_critical_section(flags);

	return time_started;
}

void stop_note()
{
	if (ioctl(pwm_fd, PWMIOC_STOP, 0) < 0) {
		PX4_ERR("PWMIOC_STOP failed: %d\n", errno);
	}
}

} /* namespace ToneAlarmInterface */
