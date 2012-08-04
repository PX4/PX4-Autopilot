/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file PWM servo driver.
 *
 * The pwm_servo driver supports servos connected to STM32 timer
 * blocks.
 *
 * Servo values can be set either with the PWM_SERVO_SET ioctl, or
 * by writing an array of servo_position_t values to the device.
 * Writing a value of 0 to a channel suppresses any output for that
 * channel.
 *
 * Servo values can be read back either with the PWM_SERVO_GET
 * ioctl, or by reading an array of servo_position_t values
 * from the device.
 *
 * Attempts to set a channel that is not configured are ignored,
 * and unconfigured channels always read zero.
 *
 * The PWM_SERVO_ARM / PWM_SERVO_DISARM calls globally arm
 * (enable) and disarm (disable) all servo outputs.
 */

#include <sys/ioctl.h>

#define _PWM_SERVO_BASE		0x7500
#define PWM_SERVO_ARM		_IOC(_PWM_SERVO_BASE, 0)
#define PWM_SERVO_DISARM	_IOC(_PWM_SERVO_BASE, 1)

#define PWM_SERVO_SET(_servo)	_IOC(_PWM_SERVO_BASE, 0x20 + _servo)
#define PWM_SERVO_GET(_servo)	_IOC(_PWM_SERVO_BASE, 0x40 + _servo)

typedef uint16_t	servo_position_t;

/* configuration limits */
#define PWM_SERVO_MAX_TIMERS	3
#define PWM_SERVO_MAX_CHANNELS	8

struct pwm_servo_config {
	/* rate (in Hz) of PWM updates */
	uint32_t	update_rate;

	/* array of timers dedicated to PWM servo use */
	struct pwm_servo_timer {
		uint32_t	base;
		uint32_t	clock_register;
		uint32_t	clock_bit;
		uint32_t	clock_freq;
	} timers[PWM_SERVO_MAX_TIMERS];

	/* array of channels in logical order */
	struct pwm_servo_channel {
		uint32_t	gpio;
		uint8_t		timer_index;
		uint8_t		timer_channel;
		servo_position_t default_value;
	} channels[PWM_SERVO_MAX_CHANNELS];
};

extern int	pwm_servo_init(const struct pwm_servo_config *config);


