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
 * VOXL2_IO UART baud rate
 *
 * Default rate is 921600, which is used for communicating with VOXL2_IO board
 *
 * @group VOXL2 IO
 * @unit bit/s
 */
PARAM_DEFINE_INT32(VOXL2_IO_BAUD, 921600);


/**
 * VOXL2_IO Disabled PWM
 *
 * Pulse duration in disabled state (microseconds) for VOXL2_IO board
 *
 * @min 0
 * @max 2000
 * @group VOXL2 IO
 * @unit us
 */
PARAM_DEFINE_INT32(VOXL2_IO_DIS, 1000);

/**
 * VOXL2_IO Min PWM
 *
 * Minimum duration (microseconds) for VOXL2_IO board
 *
 * @min 0
 * @max 2000
 * @group VOXL2 IO
 * @unit us
 */

PARAM_DEFINE_INT32(VOXL2_IO_MIN, 1100);

/**
 * VOXL2_IO Max PWM
 *
 * Maximum duration (microseconds) for VOXL2_IO board
 * @min 0
 * @max 2000
 * @group VOXL2 IO
 * @unit us
 */
PARAM_DEFINE_INT32(VOXL2_IO_MAX, 2000);


/**
 * VOXL2_IO Calibration Min PWM
 *
 * Minimum duration (microseconds) for VOXL2_IO board
 *
 * @min 0
 * @max 2000
 * @group VOXL2 IO
 * @unit us
 */

PARAM_DEFINE_INT32(VOXL2_IO_CMIN, 1050);

/**
 * VOXL2_IO Calibration Max PWM
 *
 * Maximum duration (microseconds) for VOXL2_IO board
 * @min 0
 * @max 2000
 * @group VOXL2 IO
 * @unit us
 */
PARAM_DEFINE_INT32(VOXL2_IO_CMAX, 2000);
