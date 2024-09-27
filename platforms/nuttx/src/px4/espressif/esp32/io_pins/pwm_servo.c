/****************************************************************************
 *
 *   Copyright (C) 2012, 2017 PX4 Development Team. All rights reserved.
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

/*
 * @file drv_pwm_servo.c
 *
 * Servo driver supporting PWM servos connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have output pins, does not require an interrupt.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_pwm_output.h>

#include <px4_arch/io_timer.h>

#include "esp32_ledc.h"

struct pwm_info_s pwm_info;
struct pwm_lowerhalf_s *pwm;

int up_pwm_servo_set(unsigned channel, servo_position_t value)
{
	//syslog(LOG_INFO, "PWM set ch: %d value:%d\n", channel,value);
	pwm_info.channels[channel].duty = (value*pwm_info.frequency)/(1000000/65535);
	return OK;
}

servo_position_t up_pwm_servo_get(unsigned channel)
{
	//syslog(LOG_INFO, "PWM get ch: %d\n", channel);
	return pwm_info.channels[channel].duty;
}

int up_pwm_servo_init(uint32_t channel_mask)
{
	//syslog(LOG_INFO, "channel_mask: %02X\n", channel_mask);

	// int ret = 0;

  	pwm = esp32_ledc_init(0);
  	if (!pwm)
    	{
      		syslog(LOG_ERR, "[boot] Failed to get the LEDC PWM 0 lower half\n");
    	}

 	/* Register the PWM driver at "/dev/pwm0" */
  	// ret = pwm_register("/dev/pwm0", pwm);
  	// if (ret < 0)
    	// {
      	// 	syslog(LOG_ERR, "[boot] pwm_register failed: %d\n", ret);
    	// }

	pwm->ops->setup(pwm);


	pwm_info.frequency=50;
	pwm_info.channels[0].duty=0,
	pwm_info.channels[1].duty=0,
	pwm_info.channels[2].duty=0,
	pwm_info.channels[3].duty=6553,
	pwm->ops->start(pwm,&pwm_info);

	return channel_mask;
}

void up_pwm_servo_deinit(uint32_t channel_mask)
{
	/* disable the timers */
	up_pwm_servo_arm(false, channel_mask);
}

int up_pwm_servo_set_rate_group_update(unsigned group, unsigned rate)
{
	//syslog(LOG_INFO, "group update group: %d rate:%d\n", group,rate);

	if(group == 0)
	{
		pwm_info.frequency = rate;
		return OK;
	}
	return ERROR;
}

void up_pwm_update(unsigned channels_mask)
{
	//syslog(LOG_INFO, "up_pwm_update channels_mask: %d\n", channels_mask);
	pwm->ops->start(pwm,&pwm_info);
}

uint32_t up_pwm_servo_get_rate_group(unsigned group)
{
	//syslog(LOG_INFO, "up_pwm_servo_get_rate_group: %d\n", group);
	if(group == 0)
		return 0x0F;

	return 0;
}

void
up_pwm_servo_arm(bool armed, uint32_t channel_mask)
{
	//syslog(LOG_INFO, "up_pwm_servo_arm armed:%d channel_mask:%02X\n", armed,channel_mask);
	if(channel_mask == 0x0F)
	{
		if(armed)
		{
			pwm->ops->start(pwm,&pwm_info);
		}else
		{
			pwm->ops->stop(pwm);
		}
	}
}
