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
 * @author Roman Bapst <roman@px4.io>
 * @author Sander Smeets <sander@droneslab.com>
 */

/**
 * VTOL number of engines
 *
 * @min 0
 * @max 8
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_MOT_COUNT, 0);

/**
 * Idle speed of VTOL when in multicopter mode
 *
 * @unit us
 * @min 900
 * @max 2000
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_IDLE_PWM_MC, 900);

/**
 * Minimum airspeed in multicopter mode
 *
 * This is the minimum speed of the air flowing over the control surfaces.
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_MIN, 10.0f);

/**
 * Maximum airspeed in multicopter mode
 *
 * This is the maximum speed of the air flowing over the control surfaces.
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_MAX, 30.0f);

/**
 * Trim airspeed when in multicopter mode
 *
 * This is the airflow over the control surfaces for which no airspeed scaling is applied in multicopter mode.
 *
 * @unit m/s
 * @min 0.0
 * @max 30.0
 * @increment 0.5
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_MC_ARSPD_TRIM, 10.0f);

/**
 * Permanent stabilization in fw mode
 *
 * If set to one this parameter will cause permanent attitude stabilization in fw mode.
 * This parameter has been introduced for pure convenience sake.
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_PERM_STAB, 0);

/**
 * Fixed wing pitch trim
 *
 * This parameter allows to adjust the neutral elevon position in fixed wing mode.
 *
 * @min -1.0
 * @max 1.0
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_PITCH_TRIM, 0.0f);

/**
 * Motor max power
 *
 * Indicates the maximum power the motor is able to produce. Used to calculate
 * propeller efficiency map.
 *
 * @unit W
 * @min 1
 * @max 10000
 * @increment 1
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_POWER_MAX, 120.0f);

/**
 * Propeller efficiency parameter
 *
 * Influences propeller efficiency at different power settings. Should be tuned beforehand.
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PROP_EFF, 0.0f);

/**
 * Total airspeed estimate low-pass filter gain
 *
 * Gain for tuning the low-pass filter for the total airspeed estimate
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_LP_GAIN, 0.3f);

/**
 * VTOL Type (Tailsitter=0, Tiltrotor=1, Standard=2)
 *
 * @value 0 Tailsitter
 * @value 1 Tiltrotor
 * @value 2 Standard
 * @min 0
 * @max 2
 * @decimal 0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_TYPE, 0);

/**
 * Lock elevons in multicopter mode
 *
 * If set to 1 the elevons are locked in multicopter mode
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_ELEV_MC_LOCK, 0);

/**
 * Duration of a front transition
 *
 * Time in seconds used for a transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TRANS_DUR, 5.0f);

/**
 * Duration of a back transition
 *
 * Time in seconds used for a back transition
 *
 * @unit s
 * @min 0.00
 * @max 20.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_DUR, 4.0f);

/**
 * Transition blending airspeed
 *
 * Airspeed at which we can start blending both fw and mc controls. Set to 0 to disable.
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_BLEND, 8.0f);

/**
 * Transition airspeed
 *
 * Airspeed at which we can switch to fw mode
 *
 * @unit m/s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_ARSP_TRANS, 10.0f);

/**
 * Optimal recovery strategy for pitch-weak tailsitters
 *
 * @boolean
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_OPT_RECOV_EN, 0);

/**
 * Weather-vane yaw rate scale.
 *
 * The desired yawrate from the controller will be scaled in order to avoid
 * yaw fighting against the wind.
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_WV_YAWR_SCL, 0.15f);

/**
 * Front transition timeout
 *
 * Time in seconds after which transition will be cancelled. Disabled if set to 0.
 *
 * @unit s
 * @min 0.00
 * @max 30.00
 * @increment 1
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_TIMEOUT, 15.0f);

/**
 * Front transition minimum time
 *
 * Minimum time in seconds for front transition.
 *
 * @unit s
 * @min 0.0
 * @max 10.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_MIN_TM, 2.0f);

/**
 * QuadChute Altitude
 *
 * Minimum altitude for fixed wing flight, when in fixed wing the altitude drops below this altitude
 * the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0.0
 * @max 200.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_MIN_ALT, 0.0f);


/**
 * QuadChute Max Pith
 *
 * Maximum pitch angle before QuadChute engages
 * Above this the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_P, 0);

/**
 * QuadChute Max Roll
 *
 * Maximum roll angle before QuadChute engages
 * Above this the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0
 * @max 180
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FW_QC_R, 0);

/**
 * Airspeed less front transition time (open loop)
 *
 * The duration of the front transition when there is no airspeed feedback available.
 *
 * @unit seconds
 * @min 1.0
 * @max 30.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_F_TR_OL_TM, 6.0f);
