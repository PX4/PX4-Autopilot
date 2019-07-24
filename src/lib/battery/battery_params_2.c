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
 * @file battery_params_2.c
 * @author Timothy Scott <timothy@auterion.com>
 *
 * Defines parameters for Battery 2.
 */

/**
 * Empty cell voltage (5C load)
 *
 * Defines the voltage where a single cell of battery 2 is considered empty.
 * The voltage should be chosen before the steep dropoff to 2.8V. A typical
 * lithium battery can only be discharged down to 10% before it drops off
 * to a voltage level damaging the cells.
 *
 * @group Battery Calibration
 * @unit V
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT2_V_EMPTY, 3.5f);

/**
 * Full cell voltage (5C load)
 *
 * Defines the voltage where a single cell of battery 2 is considered full
 * under a mild load. This will never be the nominal voltage of 4.2V
 *
 * @group Battery Calibration
 * @unit V
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT2_V_CHARGED, 4.05f);

/**
 * Voltage drop per cell on full throttle
 *
 * This implicitely defines the internal resistance
 * to maximum current ratio for battery 2 and assumes linearity.
 * A good value to use is the difference between the
 * 5C and 20-25C load. Not used if BAT2_R_INTERNAL is
 * set.
 *
 * @group Battery Calibration
 * @unit V
 * @min 0.07
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT2_V_LOAD_DROP, 0.3f);

/**
 * Explicitly defines the per cell internal resistance for battery 2
 *
 * If non-negative, then this will be used in place of
 * BAT2_V_LOAD_DROP for all calculations.
 *
 * @group Battery Calibration
 * @unit Ohms
 * @min -1.0
 * @max 0.2
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT2_R_INTERNAL, -1.0f);


/**
 * Number of cells for battery 2.
 *
 * Defines the number of cells the attached battery consists of.
 *
 * @group Battery Calibration
 * @unit S
 * @value 0 Unconfigured
 * @value 2 2S Battery
 * @value 3 3S Battery
 * @value 4 4S Battery
 * @value 5 5S Battery
 * @value 6 6S Battery
 * @value 7 7S Battery
 * @value 8 8S Battery
 * @value 9 9S Battery
 * @value 10 10S Battery
 * @value 11 11S Battery
 * @value 12 12S Battery
 * @value 13 13S Battery
 * @value 14 14S Battery
 * @value 15 15S Battery
 * @value 16 16S Battery
 * @reboot_required true
 */
PARAM_DEFINE_INT32(BAT2_N_CELLS, 0);

/**
 * Battery 2 capacity.
 *
 * Defines the capacity of battery 2.
 *
 * @group Battery Calibration
 * @unit mAh
 * @decimal 0
 * @min -1.0
 * @max 100000
 * @increment 50
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(BAT2_CAPACITY, -1.0f);

/**
 * Battery 2 voltage divider (V divider)
 *
 * This is the divider from battery 2 voltage to 3.3V ADC voltage.
 * If using e.g. Mauch power modules the value from the datasheet
 * can be applied straight here. A value of -1 means to use
 * the board default.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT2_V_DIV, -1.0);

/**
 * Battery 2 current per volt (A/V)
 *
 * The voltage seen by the 3.3V ADC multiplied by this factor
 * will determine the battery current. A value of -1 means to use
 * the board default.
 *
 * @group Battery Calibration
 * @decimal 8
 */
PARAM_DEFINE_FLOAT(BAT2_A_PER_V, -1.0);

/**
 * Battery 2 ADC Channel
 *
 * This parameter specifies the ADC channel used to monitor voltage of main power battery.
 * A value of -1 means to use the board default.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT2_ADC_CHANNEL, -1);