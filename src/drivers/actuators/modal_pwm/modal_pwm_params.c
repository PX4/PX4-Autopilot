/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * UART M0065 configuration
 *
 * Enable PWM on M0065.
 *
 * @reboot_required true
 *
 * @group MODAL PWM
 * @value 0 - Disabled
 * @value 1 - PWM 400Hz 
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(MODAL_PWM_CONFIG, 0);

/**
 * UART M0065 baud rate
 *
 * Default rate is 921600, which is used for communicating with M0065.
 *
 * @group MODAL PWM
 * @unit bit/s
 */
PARAM_DEFINE_INT32(MODAL_PWM_BAUD, 921600);

/**
 * M0065 PWM Min
 *
 * Minimum duration (microseconds) for M0065 PWM
 *
 * @min 0
 * @max 2000
 * @group MODAL PWM
 * @unit us
 */
PARAM_DEFINE_INT32(MODAL_PWM_MIN, 1000);

/**
 * M0065 PWM Max
 *
 * Maximum duration (microseconds) for M0065 PWM
 * @min 0
 * @max 2000
 * @group MODAL PWM
 * @unit us
 */
PARAM_DEFINE_INT32(MODAL_PWM_MAX, 2000);
