/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file fw_rate_control_params.c
 *
 * Parameters defined by the fixed-wing rate control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Minimum Airspeed (CAS)
 *
 * The minimal airspeed (calibrated airspeed) the user is able to command.
 * Further, if the airspeed falls below this value, the TECS controller will try to
 * increase airspeed more aggressively.
 * Has to be set according to the vehicle's stall speed (which should be set in FW_AIRSPD_STALL),
 * with some margin between the stall speed and minimum airspeed.
 * This value corresponds to the desired minimum speed with the default load factor (level flight, default weight),
 * and is automatically adpated to the current load factor (calculated from roll setpoint and WEIGHT_GROSS/WEIGHT_BASE).
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MIN, 10.0f);

/**
 * Maximum Airspeed (CAS)
 *
 * The maximal airspeed (calibrated airspeed) the user is able to command.
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 20.0f);

/**
 * Airspeed mode
 *
 * On vehicles without airspeed sensor this parameter can be used to
 * enable flying without an airspeed reading
 *
 * @value 0 Use airspeed in controller
 * @value 1 Do not use airspeed in controller
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ARSP_MODE, 0);

/**
 * Trim (Cruise) Airspeed
 *
 * The trim CAS (calibrated airspeed) of the vehicle. If an airspeed controller is active,
 * this is the default airspeed setpoint that the controller will try to achieve.
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_TRIM, 15.0f);

/**
 * Stall Airspeed (CAS)
 *
 * The stall airspeed (calibrated airspeed) of the vehicle.
 * It is used for airspeed sensor failure detection and for the control
 * surface scaling airspeed limits.
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_STALL, 7.0f);

/**
 * Pitch rate proportional gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_P, 0.08f);

/**
 * Pitch rate derivative gain.
 *
 * Pitch rate differential gain.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_D, 0.f);

/**
 * Pitch rate integrator gain.
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I, 0.1f);

/**
 * Pitch rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.4f);

/**
 * Roll rate proportional Gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

/**
 * Roll rate derivative Gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations.
 * If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_D, 0.00f);

/**
 * Roll rate integrator Gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I, 0.1f);

/**
 * Roll integrator anti-windup
 *
 * The portion of the integrator part in the control surface deflection is limited to this value.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

/**
 * Yaw rate proportional gain
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05f);

/**
 * Yaw rate derivative gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations.
 * If value is too big oscillations will appear again.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10
 * @decimal 3
 * @increment 0.005
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_D, 0.0f);

/**
 * Yaw rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 10
 * @decimal 1
 * @increment 0.5
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

/**
 * Roll rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output. Use this
 * to obtain a tigher response of the controller without introducing
 * noise amplification.
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RR_FF, 0.5f);

/**
 * Pitch rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_PR_FF, 0.5f);

/**
 * Yaw rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

/**
 * Acro body x max rate.
 *
 * This is the rate the controller is trying to achieve if the user applies full roll
 * stick input in acro mode.
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_X_MAX, 90);

/**
 * Acro body pitch max rate setpoint.
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Y_MAX, 90);

/**
 * Acro body yaw max rate setpoint.
 *
 * @min 10
 * @max 720
 * @unit deg
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Z_MAX, 45);

/**
 * Enable throttle scale by battery level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery.
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_BAT_SCALE_EN, 0);

/**
 * Enable airspeed scaling
 *
 * This enables a logic that automatically adjusts the output of the rate controller to take
 * into account the real torque produced by an aerodynamic control surface given
 * the current deviation from the trim airspeed (FW_AIRSPD_TRIM).
 *
 * Enable when using aerodynamic control surfaces (e.g.: plane)
 * Disable when using rotor wings (e.g.: autogyro)
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ARSP_SCALE_EN, 1);

/**
* Roll trim increment at minimum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMIN, 0.0f);

/**
* Pitch trim increment at minimum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMIN, 0.0f);

/**
* Yaw trim increment at minimum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MIN.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMIN, 0.0f);

/**
* Roll trim increment at maximum airspeed
*
* This increment is added to TRIM_ROLL when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_R_VMAX, 0.0f);

/**
* Pitch trim increment at maximum airspeed
*
* This increment is added to TRIM_PITCH when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_P_VMAX, 0.0f);

/**
* Yaw trim increment at maximum airspeed
*
* This increment is added to TRIM_YAW when airspeed is FW_AIRSPD_MAX.
 *
 * @group FW Rate Control
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 */
PARAM_DEFINE_FLOAT(FW_DTRIM_Y_VMAX, 0.0f);

/**
 * Manual roll scale
 *
 * Scale factor applied to the desired roll actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_SC, 1.0f);

/**
 * Manual pitch scale
 *
 * Scale factor applied to the desired pitch actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_SC, 1.0f);

/**
 * Manual yaw scale
 *
 * Scale factor applied to the desired yaw actuator command in full manual mode. This parameter allows
 * to adjust the throws of the control surfaces.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_Y_SC, 1.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator to counteract this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_RLL_TO_YAW_FF, 0.0f);

/**
 * Spoiler input in manual flight
 *
 * Chose source for manual setting of spoilers in manual flight modes.
 *
 * @value 0 Disabled
 * @value 1 Flaps channel
 * @value 2 Aux1
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_SPOILERS_MAN, 0);

/**
 * Enable yaw rate controller in Acro
 *
 * If this parameter is set to 1, the yaw rate controller is enabled in Fixed-wing Acro mode.
 * Otherwise the pilot commands directly the yaw actuator.
 * It is disabled by default because an active yaw rate controller will fight against the
 * natural turn coordination of the plane.
 *
 * @boolean
 * @group FW Rate Control
 */
PARAM_DEFINE_INT32(FW_ACRO_YAW_EN, 0);
