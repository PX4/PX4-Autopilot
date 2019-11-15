/****************************************************************************
 * Copyright (c) 2016 James Y. Wilson. All rights reserved.
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

#include <stdint.h>

/**
 * @file
 * The functions in this module define the interface for configuring one
 * or more PWM signals.  Once the PWM signal is configured it will be generated
 * at the specified period and width, until it is changed by another function
 * call.
 *
 * The width of PWM signals specified by the caller can be changed for
 * individual I/O lines, but all I/O lines must have the same period.
 * Additional instances of this device can be opened to define a different group
 * of PWM signals at a different period.
 *
 * @par
 * Sample source code to generate a time varying PWM signal is included below.
 * @include pwm_test_imp.c
 */

/**
 * @brief
 * The maximum number of pwm signals allowed in a signal definition.
 */
#define DEV_FS_PWM_MAX_NUM_SIGNALS 8

/**
 * @brief
 * The PWM device path uses the following format:
 * /dev/pwm-{gpio pin number}
 * Pin numbers start at 1 and may go to up to the max number of GPIO pins supported
 * by the SoC.
 */
#define DEV_FS_PWM_DEVICE_TYPE_STRING  "/dev/pwm-"

/**
 * @brief
 * Error codes describing an error condition of the PWM signal generator.  POSIX
 * error codes may also be returned.
 */
#define DEV_FS_PWM_ERROR_SIGNALS_ALREADY_DEFINED -4095

/**
 * @brief
 * ioctl codes used to extend the functionality of the standard read/write file
 * semantics for PWM signal generation
 */
enum DSPAL_PWM_IOCTLS {
	PWM_IOCTL_INVALID = -1, /**< invalid IOCTL code, used to return an error */                                 //!< PWM_IOCTL_INVALID
	PWM_IOCTL_SIGNAL_DEFINITION, /**< used to define the gpio number(s) and period of the pulse width(s) */     //!< PWM_IOCTL_SIGNAL_DEFINITION
	PWM_IOCTL_GET_UPDATE_BUFFER, /**< returns a buffer used to update the pulse width in real-time */      //!< PWM_IOCTL_GET_PULSE_WIDTH_BUFFER
	PWM_IOCTL_MAX_NUM, /**< number of valid IOCTL codes defined for the PWM generator */                        //!< PWM_IOCTL_MAX_NUM
};

/**
 * @brief
 * Structure used to define the GPIO ID of the I/O line and the width of the pulse
 * used by the signal generator.  This structure is also used to return a buffer containing
 * an array of structures that can be accessed directly to effect a change in the width
 * of the pulse.  @see PWM_IOCTL_GET_UPDATE_BUFFER
 *
 */
struct dspal_pwm {
	uint32_t gpio_id;               /**< ID of the GPIO line used for PWM generation */
	uint32_t gpio_cfg;              /**< internal use only */
	uint32_t pulse_width_in_usecs;  /**< duration of the pulse in usecs */
	uint32_t pulse_state;           /**< a read-only value, indicating the current state of the pulse */
};

/**
 * @brief
 * Structure used in the ioctl: PWM_IOCTL_SIGNAL_DEFINITION
 *
 * Upon the return from the function signal generation will begin at the specified period and pulse width
 * for each I/O line specified.
 */
struct dspal_pwm_ioctl_signal_definition {
	uint32_t period_in_usecs; /**< the period of the pulses generated, can only be changed once after the device is first opened */
	uint32_t num_gpios; /**< number of signals specified in the following array */
	struct dspal_pwm *pwm_signal; /**< array defining the GPIO lines and pulse widths to be used by the signal generator */
};

/**
 * @brief
 * Structure used in the ioctl: PWM_IOCTL_GET_UPDATE_BUFFER
 *
 * Returns a buffer that can be used to effect an immediate change in the width of the pulse.
 * @par
 * To change the width of a particular pulse the new pulse width should be written to the
 * pulse_width_in_usecs structure member.  If the deadline for changing the state of the pulse
 * has already passed, or the width of the pulse is less than the previous pulse width the
 * new value will not take effect until the next period.
 *
 * No other structure member other than pulse_width_in_usecs may be modified.  To use different
 * GPIO lines or change the period a new signal definition must be created.
 *
 * The I/O lines used cannot be changed from those defined in in the PWM_IOCTL_SIGNAL_DEFINITION structure.
 */
struct dspal_pwm_ioctl_update_buffer {
	uint32_t num_gpios; /**< the number of PWM's specified in the following array */
	struct dspal_pwm
		*pwm_signal; /**< array defining the GPIO lines and pulse widths to be used by the signal generator, can only be specified once after the device is first opened */
	uint32_t reserved_1; /**< reserved value used for debugging. */
	uint32_t reserved_2; /**< reserved value used for debugging. */
};

