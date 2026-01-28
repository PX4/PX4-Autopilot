/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * AFBR Rangefinder Mode
 *
 * This parameter defines the mode of the AFBR Rangefinder.
 *
 * @reboot_required true
 * @min 0
 * @max 3
 * @group Sensors
 *
 * @value 0 Short Range Mode
 * @value 1 Long Range Mode
 * @value 2 High Speed Short Range Mode
 * @value 3 High Speed Long Range Mode
 */
PARAM_DEFINE_INT32(SENS_AFBR_MODE, 0);

/**
 * AFBR Rangefinder Short Range Rate
 *
 * This parameter defines measurement rate of the AFBR Rangefinder in short range mode.
 *
 * @min 1
 * @max 100
 * @group Sensors
 *
 */
PARAM_DEFINE_INT32(SENS_AFBR_S_RATE, 50);

/**
 * AFBR Rangefinder Long Range Rate
 *
 * This parameter defines measurement rate of the AFBR Rangefinder in long range mode.
 *
 * @min 1
 * @max 100
 * @group Sensors
 *
 */
PARAM_DEFINE_INT32(SENS_AFBR_L_RATE, 25);

/**
 * AFBR Rangefinder Short/Long Range Threshold
 *
 * This parameter defines the threshold for switching between short and long range mode.
 * The mode will switch from short to long range when the distance is greater than the threshold plus the hysteresis.
 * The mode will switch from long to short range when the distance is less than the threshold minus the hysteresis.
 *
 * @unit m
 * @min 1
 * @max 50
 * @group Sensors
 *
 */
PARAM_DEFINE_INT32(SENS_AFBR_THRESH, 4);


/**
 * AFBR Rangefinder Short/Long Range Threshold Hysteresis
 *
 * This parameter defines the hysteresis for switching between short and long range mode.
 *
 * @unit m
 * @min 1
 * @max 10
 * @group Sensors
 *
 */
PARAM_DEFINE_INT32(SENS_AFBR_HYSTER, 1);
