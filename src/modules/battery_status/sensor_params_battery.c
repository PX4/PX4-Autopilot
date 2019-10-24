/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * Scaling from ADC counts to volt on the ADC input (battery voltage)
 *
 * This is not the battery voltage, but the intermediate ADC voltage.
 * A value of -1 signifies that the board defaults are used, which is
 * highly recommended.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_CNT_V_VOLT, -1.0f);

/**
 * Scaling from ADC counts to volt on the ADC input (battery current)
 *
 * This is not the battery current, but the intermediate ADC voltage.
 * A value of -1 signifies that the board defaults are used, which is
 * highly recommended.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_CNT_V_CURR, -1.0);

/**
 * Offset in volt as seen by the ADC input of the current sensor.
 *
 * This offset will be subtracted before calculating the battery
 * current based on the voltage.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_V_OFFS_CURR, 0.0);

/**
 * Battery voltage divider (V divider)
 *
 * This is the divider from battery voltage to 3.3V ADC voltage.
 * If using e.g. Mauch power modules the value from the datasheet
 * can be applied straight here. A value of -1 means to use
 * the board default.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_V_DIV, -1.0);

/**
 * Battery current per volt (A/V)
 *
 * The voltage seen by the 3.3V ADC multiplied by this factor
 * will determine the battery current. A value of -1 means to use
 * the board default.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT_A_PER_V, -1.0);

/**
 * Battery monitoring source.
 *
 * This parameter controls the source of battery data. The value 'Power Module'
 * means that measurements are expected to come from a power module. If the value is set to
 * 'External' then the system expects to receive mavlink battery status messages.
 *
 * @min 0
 * @max 1
 * @value 0 Power Module
 * @value 1 External
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_SOURCE, 0);

/**
 * Battery ADC Channel
 *
 * This parameter specifies the ADC channel used to monitor voltage of main power battery.
 * A value of -1 means to use the board default.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_ADC_CHANNEL, -1);
