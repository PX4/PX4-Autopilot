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
 * UART ESC configuration
 *
 * Selects what type of UART ESC, if any, is being used.
 *
 * @reboot_required true
 *
 * @group UART ESC
 * @value 0 - Disabled
 * @value 1 - VOXL ESC
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(UART_ESC_CONFIG, 0);

/**
 * UART ESC baud rate
 *
 * Default rate is 250Kbps, which is used in off-the-shelf MoadalAI ESC products.
 *
 * @group UART ESC
 * @unit bit/s
 */
PARAM_DEFINE_INT32(UART_ESC_BAUD, 250000);

/**
 * Motor mappings for ModalAI ESC M004
 *
 *  HW Channel Idexes (PX4 Indexes) (note: silkscreen shows 0 indexed)
 *         4(1)     3(4)
 * [front]
 *         1(3)     2(2)
 */

/**
 * UART ESC Motor 1 Mapping.  1-4 (negative for reversal).
 *
 * @group UART ESC
 * @min -4
 * @max 4
 */
PARAM_DEFINE_INT32(UART_ESC_MOTOR1, 3);

/**
 *UART ESC Motor 2 Mapping.  1-4 (negative for reversal).
 *
 * @group UART ESC
 * @min -4
 * @max 4
 */
PARAM_DEFINE_INT32(UART_ESC_MOTOR2, 2);

/**
 * UART ESC Motor 3 Mapping.  1-4 (negative for reversal).
 *
 * @group UART ESC
 * @min -4
 * @max 4
 */
PARAM_DEFINE_INT32(UART_ESC_MOTOR3, 4);

/**
 * UART ESC Motor 4 Mapping.  1-4 (negative for reversal).
 *
 * @group UART ESC
 * @min -4
 * @max 4
 */
PARAM_DEFINE_INT32(UART_ESC_MOTOR4, 1);

/**
 * UART ESC RPM Min
 *
 * Minimum RPM for ESC
 *
 * @group UART ESC
 */
PARAM_DEFINE_INT32(UART_ESC_RPM_MIN, 5500);

/**
 * UART ESC RPM Max
 *
 * Maximum RPM for ESC
 *
 * @group UART ESC
 */
PARAM_DEFINE_INT32(UART_ESC_RPM_MAX, 15000);

/**
 * UART ESC Mode
 *
 * Selects what type of mode is enabled, if any
 *
 * @reboot_required true
 *
 * @group UART ESC
 * @value 0 - None
 * @value 1 - Turtle Mode enabled via AUX1
 * @value 2 - Turtle Mode enabled via AUX2
 * @value 3 - UART Passthrough Mode
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(UART_ESC_MODE, 0);

/**
 * UART ESC Turtle Mode Crash Flip Motor Percent
 *
 * @group UART ESC
 * @min 1
 * @max 100
 * @decimal 10
 * @increment 1
 */
PARAM_DEFINE_INT32(UART_ESC_T_PERC, 90);

/**
 * UART ESC Turtle Mode Crash Flip Motor Deadband
 *
 * @group UART ESC
 * @min 0
 * @max 100
 * @decimal 10
 * @increment 1
 */
PARAM_DEFINE_INT32(UART_ESC_T_DEAD, 20);

/**
 * UART ESC Turtle Mode Crash Flip Motor STICK_MINF
 *
 * @group UART ESC
 * @min 0.0
 * @max 100.0
 * @decimal 10
 * @increment 1.0
 */
PARAM_DEFINE_FLOAT(UART_ESC_T_MINF, 0.15);

/**
 * UART ESC Turtle Mode Crash Flip Motor expo
 *
 * @group UART ESC
 * @min 0
 * @max 100
 * @decimal 10
 * @increment 1
 */
PARAM_DEFINE_INT32(UART_ESC_T_EXPO, 35);

/**
 * UART ESC Turtle Mode Yaw Reversal
 *
 * @group UART ESC
 * @min 0
 * @max 1
 * @decimal 10
 * @increment 1
 */
PARAM_DEFINE_INT32(UART_ESC_T_YAWR, 0);
/**
 * UART ESC Turtle Mode Cosphi
 *
 * @group UART ESC
 * @min 0.000
 * @max 1.000
 * @decimal 10
 * @increment 0.001
 */
PARAM_DEFINE_FLOAT(UART_ESC_T_COSP, 0.990);
