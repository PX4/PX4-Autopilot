/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Julian Oes <julian@px4.io>
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Roll trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 */
PARAM_DEFINE_FLOAT(TRIM_ROLL, 0.0f);

/**
 * Pitch trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 */
PARAM_DEFINE_FLOAT(TRIM_PITCH, 0.0f);

/**
 * Yaw trim
 *
 * The trim value is the actuator control value the system needs
 * for straight and level flight. It can be calibrated by
 * flying manually straight and level using the RC trims and
 * copying them using the GCS.
 *
 * @group Radio Calibration
 * @min -0.25
 * @max 0.25
 */
PARAM_DEFINE_FLOAT(TRIM_YAW, 0.0f);

/**
 * Number of battery cells.
 *
 * Defines the number of cells the attached battery consists of.
 *
 * @group Battery Calibration
 * @unit S
 * @min 1
 * @max 10
 */
PARAM_DEFINE_INT32(BAT_N_CELLS, 0);

/**
 * Full battery cell voltage.
 *
 * Defines the voltage at which one battery cell is considered full.
 * The default value corresponds to a fully-charged standard Lithium-Polymer (LiPo) cell.
 *
 * @group Battery Calibration
 * @unit V
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(BAT_V_CELL_FULL, 4.2f);

/**
 * Low battery cell voltage.
 *
 * Defines the voltage at which one battery cell is considered low.
 * The default value corresponds to a low-voltage standard Lithium-Polymer (LiPo) cell.
 *
 * @group Battery Calibration
 * @unit V
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(BAT_V_CELL_LOW, 3.8f);

/**
 * Critical battery cell voltage.
 *
 * Defines the voltage at which one battery cell is considered critically empty.
 * The default value corresponds to a critical-voltage standard Lithium-Polymer (LiPo) cell.
 *
 * @group Battery Calibration
 * @unit V
 * @min 0.0
 */
PARAM_DEFINE_FLOAT(BAT_V_CELL_CRIT, 3.2f);

/**
 * Battery capacity.
 *
 * Defines the capacity of the attached battery.
 * Setting this value to -1 disables remaining battery estimation based on discharged capacity.
 *
 * @group Battery Calibration
 * @unit mAh
 */
PARAM_DEFINE_INT32(BAT_CAPACITY, 0);


/**
 * Low battery charge.
 *
 * Defines the percentage charge at which the battery is considered low.
 *
 * @group Battery Calibration
 * @unit %
 * @min 0
 * @max 100
 */
PARAM_DEFINE_INT32(BAT_PERC_LOW, 35);

/**
 * Critical battery charge.
 *
 * Defines the percentage charge at which the battery is considered critically empty.
 *
 * @group Battery Calibration
 * @unit %
 * @min 0
 * @max 100
 */
PARAM_DEFINE_INT32(BAT_PERC_CRIT, 25);


/**
 * Datalink loss mode enabled.
 *
 * Set to 1 to enable actions triggered when the datalink is lost.
 *
 * @group Commander
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(COM_DL_LOSS_EN, 0);

/**
 * Datalink loss time threshold
 *
 * After this amount of seconds without datalink the data link lost mode triggers
 *
 * @group Commander
 * @unit second
 * @min 0
 * @max 30
 */
PARAM_DEFINE_INT32(COM_DL_LOSS_T, 10);

/**
 * Datalink regain time threshold
 *
 * After a data link loss: after this this amount of seconds with a healthy datalink the 'datalink loss'
 * flag is set back to false
 *
 * @group Commander
 * @unit second
 * @min 0
 * @max 30
 */
PARAM_DEFINE_INT32(COM_DL_REG_T, 0);

/**
 * Engine Failure Throttle Threshold
 *
 * Engine failure triggers only above this throttle value
 *
 * @group Commander
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(COM_EF_THROT, 0.5f);

/**
 * Engine Failure Current/Throttle Threshold
 *
 * Engine failure triggers only below this current value
 *
 * @group Commander
 * @min 0.0
 * @max 30.0
 * @unit ampere
 */
PARAM_DEFINE_FLOAT(COM_EF_C2T, 5.0f);

/**
 * Engine Failure Time Threshold
 *
 * Engine failure triggers only if the throttle threshold and the
 * current to throttle threshold are violated for this time
 *
 * @group Commander
 * @unit second
 * @min 0.0
 * @max 60.0
 */
PARAM_DEFINE_FLOAT(COM_EF_TIME, 10.0f);

/**
 * RC loss time threshold
 *
 * After this amount of seconds without RC connection the rc lost flag is set to true
 *
 * @group Commander
 * @unit second
 * @min 0
 * @max 35
 */
PARAM_DEFINE_FLOAT(COM_RC_LOSS_T, 0.5f);

/**
 * Home set horizontal threshold
 *
 * The home position will be set if the estimated positioning accuracy is below the threshold.
 *
 * @group Commander
 * @unit meter
 * @min 2
 * @max 15
 */
PARAM_DEFINE_FLOAT(COM_HOME_H_T, 5.0f);

/**
 * Home set vertical threshold
 *
 * The home position will be set if the estimated positioning accuracy is below the threshold.
 *
 * @group Commander
 * @unit meter
 * @min 5
 * @max 25
 */
PARAM_DEFINE_FLOAT(COM_HOME_V_T, 10.0f);

/**
 * Autosaving of params
 *
 * If not equal to zero the commander will automatically save parameters to persistent storage once changed.
 * Default is on, as the interoperability with currently deployed GCS solutions depends on parameters
 * being sticky. Developers can default it to off.
 *
 * @group Commander
 * @min 0
 * @max 1
 */
PARAM_DEFINE_INT32(COM_AUTOS_PAR, 1);

/**
 * RC control input mode
 *
 * The default value of 0 requires a valid RC transmitter setup.
 * Setting this to 1 disables RC input handling and the associated checks. A value of
 * 2 will generate RC control data from manual input received via MAVLink instead
 * of directly forwarding the manual input data.
 *
 * @group Commander
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(COM_RC_IN_MODE, 0);
