/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file commander_params.c
 *
 * Parameters defined by the sensors task.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

/**
 * Empty cell voltage.
 *
 * Defines the voltage where a single cell of the battery is considered empty.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_EMPTY, 3.4f);

/**
 * Full cell voltage.
 *
 * Defines the voltage where a single cell of the battery is considered full.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_CHARGED, 4.2f);

/**
 * Voltage drop per cell on 100% load
 *
 * This implicitely defines the internal resistance
 * to maximum current ratio and assumes linearity.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_V_LOAD_DROP, 0.07f);

/**
 * Number of cells.
 *
 * Defines the number of cells the attached battery consists of.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_INT32(BAT_N_CELLS, 3);

/**
 * Battery capacity.
 *
 * Defines the capacity of the attached battery.
 *
 * @group Battery Calibration
 */
PARAM_DEFINE_FLOAT(BAT_CAPACITY, -1.0f);

/**
 * Datalink loss mode enabled.
 *
 * Set to 1 to enable actions triggered when the datalink is lost.
 *
 * @group commander
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(COM_DL_LOSS_EN, 0);
