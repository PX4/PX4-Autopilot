/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * TFmini Rangefinder (i2c)
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 * @value 0 Disabled
 * @value 1 TFmini_s_I2C
 * @value 2 TFmini_I2C
 * @value 3 TFmini_Plus_I2C
 */
PARAM_DEFINE_INT32(SENS_EN_TFMINI2C, 0);

/**
 * TFmini-s Rangefinder (i2c) Selector
 *
 * @reboot_required true
 * @min 1
 * @max 32
 * @group Sensors
 * @value 1 Orientation ONLY DOWNWARD_FACING Enable
 * @value 2 Orientation DOWNWARD_FACING + FOURWARD_FACING Enable
 * @value 31 Orientation DOWNWARD_FACING + FOURWARD_FACING + LEFT_FACING Enable
 * @value 32 Orientation DOWNWARD_FACING + FOURWARD_FACING + RIGHT_FACING Enable
 * @value 4 Orientation DOWNWARD_FACING + FOURWARD_FACING + LEFT_FACING + RIGHT_FACING Enable
 * @value 5 Orientation Except for BACKWARD_FACING Enable
 * @value 6 ALL Orientations Enable
 */
PARAM_DEFINE_INT32(SENS_SEL_TFMINIS, 1);
