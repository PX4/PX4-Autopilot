/****************************************************************************
 *
 *   Copyright (C) 2015 Ramakrishna Kintada. All rights reserved.
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
 * @file uart_esc_params.c
 *
 * Parameters defined for the uart esc driver
 */
#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * ESC model
 *
 * See esc_model_t enum definition in uart_esc_dev.h for all supported ESC
 * model enum values.
 * ESC_200QX = 0
 * ESC_350QX = 1
 * ESC_210QC = 2
 *
 * Default is 210QC
 */
PARAM_DEFINE_INT32(UART_ESC_MODEL, 2);

/**
 * ESC UART baud rate
 *
 * Default rate is 250Kbps, which is used in off-the-shelf QRP ESC products.
 */
PARAM_DEFINE_INT32(UART_ESC_BAUDRATE, 250000);

/**
 * The PX4 default motor mappings are
 *         1     4
 * [front]
 *         3     2
 *
 * The following paramters define the motor mappings in reference to the
 * PX4 motor mapping convention.
 */
/**
 * Default PX4 motor mappings
 *         1     4
 * [front]
 *         3     2
 */
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR1, 1);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR2, 2);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR3, 3);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR4, 4);

/**
 * Motor mappings for 350QX
 *         4     3
 * [front]
 *         1     2
 */
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR1, 4);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR2, 2);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR3, 1);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR4, 3);

/**
 * Motor mappings for 200QX
 *         2     3
 * [front]
 *         1     4
 */
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR1, 2);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR2, 4);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR3, 1);
// PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR4, 3);

/**
 * Motor mappings for 210QC
 *         4     3
 * [front]
 *         1     2
 */
PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR1, 4);
PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR2, 2);
PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR3, 1);
PARAM_DEFINE_INT32(UART_ESC_PX4MOTOR4, 3);
