/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * Murata SCH16T IMU (external SPI)
 *
 * @reboot_required true
 * @min 0
 * @max 1
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enabled
  */
PARAM_DEFINE_INT32(SENS_EN_SCH16T, 0);

/**
 * Gyro filter settings
 *
 * @value 0 13 Hz
 * @value 1 30 Hz
 * @value 2 68 Hz
 * @value 3 235 Hz
 * @value 4 280 Hz
 * @value 5 370 Hz
 * @value 6 No filter
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_INT32(SCH16T_GYRO_FILT, 2);

/**
 * Accel filter settings
 *
 * @value 0 13 Hz
 * @value 1 30 Hz
 * @value 2 68 Hz
 * @value 3 235 Hz
 * @value 4 280 Hz
 * @value 5 370 Hz
 * @value 6 No filter
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_INT32(SCH16T_ACC_FILT, 6);

/**
 * Gyro and Accel decimation settings
 *
 * @value 0 None
 * @value 1 5900 Hz
 * @value 2 2950 Hz
 * @value 3 1475 Hz
 * @value 4 738 Hz
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_INT32(SCH16T_DECIM, 4);
