/****************************************************************************
 *
 *   Copyright (c) 2012-2015, 2017 PX4 Development Team. All rights reserved.
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
 * @file PWM servo output interface.
 *
 * Servo values can be set with the PWM_SERVO_SET ioctl, by writing a
 * pwm_output_values structure to the device, or by publishing to the
 * output_pwm ORB topic.
 * Writing a value of 0 to a channel suppresses any output for that
 * channel.
 */

#pragma once

#include <px4_defines.h>

#include "uORB/topics/output_pwm.h"

#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_orb_dev.h"

#include "drv_pwm_output.h"

__BEGIN_DECLS

/*
 * Low-level PWM output interface.
 *
 * This is the low-level API to the platform-specific PWM driver.
 */

/**
 * Intialise the PWM servo outputs using the specified configuration.
 *
 * @param channel_mask	Bitmask of channels (LSB = channel 0) to enable.
 *			This allows some of the channels to remain configured
 *			as GPIOs or as another function.
 * @return		OK on success.
 */
__EXPORT extern int	up_ext_pwm_servo_init(uint32_t channel_mask);

/**
 * De-initialise the PWM servo outputs.
 */
__EXPORT extern void	up_ext_pwm_servo_deinit(void);

/**
 * Arm or disarm servo outputs.
 *
 * When disarmed, servos output no pulse.
 *
 * @bug This function should, but does not, guarantee that any pulse
 *      currently in progress is cleanly completed.
 *
 * @param armed		If true, outputs are armed; if false they
 *			are disarmed.
 */
__EXPORT extern void	up_ext_pwm_servo_arm(bool armed);

/**
 * Set the servo update rate for all rate groups.
 *
 * @param rate		The update rate in Hz to set.
 * @return		OK on success, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_ext_pwm_servo_set_rate(unsigned rate);

/**
 * Get a bitmap of output channels assigned to a given rate group.
 *
 * @param group		The rate group to query. Rate groups are assigned contiguously
 *			starting from zero.
 * @return		A bitmap of channels assigned to the rate group, or zero if
 *			the group number has no channels.
 */
__EXPORT extern uint32_t up_ext_pwm_servo_get_rate_group(unsigned group);

/**
 * Set the update rate for a given rate group.
 *
 * @param group		The rate group whose update rate will be changed.
 * @param rate		The update rate in Hz.
 * @return		OK if the group was adjusted, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_ext_pwm_servo_set_rate_group_update(unsigned group, unsigned rate);

/**
 * Trigger all timer's channels in Oneshot mode to fire
 * the oneshot with updated values.
 * Nothing is none if not in oneshot mode.
 *
 */
__EXPORT extern void up_ext_pwm_update(void);

/**
 * Set the current output value for a channel.
 *
 * @param channel	The channel to set.
 * @param value		The output pulse width in microseconds.
 */
__EXPORT extern int	up_ext_pwm_servo_set(unsigned channel, servo_position_t value);

/**
 * Get the current output value for a channel.
 *
 * @param channel	The channel to read.
 * @return		The output pulse width in microseconds, or zero if
 *			outputs are not armed or not configured.
 */
__EXPORT extern servo_position_t up_ext_pwm_servo_get(unsigned channel);

__END_DECLS
