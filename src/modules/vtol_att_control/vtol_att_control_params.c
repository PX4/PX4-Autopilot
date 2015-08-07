/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file vtol_att_control_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 */

#include <systemlib/param/param.h>

/**
 * VTOL number of engines
 *
 * @min 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_MOT_COUNT, 0);

/**
 * Idle speed of VTOL when in multicopter mode
 *
 * @min 900
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_IDLE_PWM_MC, 900);

/**
 * Minimum airspeed in multicopter mode
 *
 * This is the minimum speed of the air flowing over the control surfaces.
 *
 * @min 0.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_MIN, 10.0f);

/**
 * Maximum airspeed in multicopter mode
 *
 * This is the maximum speed of the air flowing over the control surfaces.
 *
 * @min 0.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_MAX, 30.0f);

/**
 * Trim airspeed when in multicopter mode
 *
 * This is the airflow over the control surfaces for which no airspeed scaling is applied in multicopter mode.
 *
 * @min 0.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_TRIM, 10.0f);

/**
 * Permanent stabilization in fw mode
 *
 * If set to one this parameter will cause permanent attitude stabilization in fw mode.
 * This parameter has been introduced for pure convenience sake.
 *
 * @min 0
 * @max 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_PERM_STAB, 0);

/**
 * Fixed wing pitch trim
 *
 * This parameter allows to adjust the neutral elevon position in fixed wing mode.
 *
 * @min -1
 * @max 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_PITCH_TRIM, 0.0f);

/**
 * Motor max power
 *
 * Indicates the maximum power the motor is able to produce. Used to calculate
 * propeller efficiency map.
 *
 * @min 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_POWER_MAX, 120.0f);

/**
 * Propeller efficiency parameter
 *
 * Influences propeller efficiency at different power settings. Should be tuned beforehand.
 *
 * @min 0.0
 * @max 0.9
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PROP_EFF, 0.0f);

/**
 * Total airspeed estimate low-pass filter gain
 *
 * Gain for tuning the low-pass filter for the total airspeed estimate
 *
 * @min 0.0
 * @max 0.99
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_LP_GAIN, 0.3f);

/**
 * VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2)
 *
 * @min 0
 * @max 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TYPE, 0);

/**
 * Lock elevons in multicopter mode
 *
 * If set to 1 the elevons are locked in multicopter mode
 *
 * @min 0
 * @max 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_ELEV_MC_LOCK, 0);

/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @min 0.0
 * @max 5
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_DUR, 3.0f);

/**
 * Duration of a back transition
 *
 * Time in seconds used for a back transition
 *
 * @min 0.0
 * @max 5
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_DUR, 2.0f);

/**
 * Transition blending airspeed
 *
 * Airspeed at which we can start blending both fw and mc controls. Set to 0 to disable.
 *
 * @min 0.0
 * @max 20.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_BLEND, 8.0f);

/**
 * Transition airspeed
 *
 * Airspeed at which we can switch to fw mode
 *
 * @min 1.0
 * @max 20
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_TRANS, 10.0f);
