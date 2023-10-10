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
 * @value 1 - M0065 PWM
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(MODAL_PWM_CONFIG, 1);

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
 * Motor mappings for ModalAI PWM
 *
 *  HW Channel Idexes (PX4 Indexes) (note: silkscreen shows 0 indexed)
 *         4(1)     3(4)
 * [front]
 *         1(3)     2(2)
 */

// The following are auto generated params from control allocator pattern, put here for reference

// Default ESC1 to motor2
//PARAM_DEFINE_INT32(MODAL_IO_FUNC1, 102);

//PARAM_DEFINE_INT32(MODAL_IO_FUNC2, 103);

//PARAM_DEFINE_INT32(MODAL_IO_FUNC3, 101);

//PARAM_DEFINE_INT32(MODAL_IO_FUNC4, 104);

/**
 * M0065 PWM Min
 *
 * Minimum value for PWM
 *
 * @group MODAL PWM
 * @unit us
 */
PARAM_DEFINE_INT32(MODAL_PWM_MIN, 0);

/**
 * M0065 PWM Max
 *
 * Maximum value for PWM
 *
 * @group MODAL PWM
 * @unit us
 */
PARAM_DEFINE_INT32(MODAL_PWM_MAX, 800);


/**
 * M0065 PWM Failsafe
 *
 * Failsafe value for PWM
 *
 * @group MODAL PWM
 * @unit us
 */
PARAM_DEFINE_INT32(MODAL_PWM_FS, 0);

/**
 * UART M0065 Mode
 *
 * Selects what type of mode is enabled, if any
 *
 * @reboot_required true
 *
 * @group MODAL PWM
 * @value 0 - None
 * @value 1 - Turtle Mode enabled via AUX1
 * @value 2 - Turtle Mode enabled via AUX2
 * @value 3 - UART Passthrough Mode
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(MODAL_PWM_MODE, 0);

/**
 * UART ESC verbose logging
 *
 * @reboot_required true
 *
 * @group MODAL PWM
 * @value 0 - Disabled
 * @value 1 - Enabled
 * @min 0
 * @max 1
 */
// PARAM_DEFINE_INT32(MODAL_IO_VLOG, 0);
