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
 * @file fw_att_control_params.c
 *
 * Parameters defined by the fixed-wing attitude control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Attitude Roll Time Constant
 *
 * This defines the latency between a roll step input and the achieved setpoint
 * (inverse to a P gain). Half a second is a good start value and fits for
 * most average systems. Smaller systems may require smaller values, but as
 * this will wear out servos faster, the value should only be decreased as
 * needed.
 *
 * @unit s
 * @min 0.4
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_TC, 0.4f);

/**
 * Attitude Pitch Time Constant
 *
 * This defines the latency between a pitch step input and the achieved setpoint
 * (inverse to a P gain). Half a second is a good start value and fits for
 * most average systems. Smaller systems may require smaller values, but as
 * this will wear out servos faster, the value should only be decreased as
 * needed.
 *
 * @unit s
 * @min 0.2
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_TC, 0.4f);

/**
 * Pitch rate proportional gain.
 *
 * This defines how much the elevator input will be commanded depending on the
 * current body angular rate error.
 *
 * @unit %/rad/s
 * @min 0.005
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_P, 0.08f);

/**
 * Pitch rate integrator gain.
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.005
 * @max 0.5
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_I, 0.02f);

/**
 * Maximum positive / up pitch rate.
 *
 * This limits the maximum pitch up angular rate the controller will output (in
 * degrees per second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_POS, 60.0f);

/**
 * Maximum negative / down pitch rate.
 *
 * This limits the maximum pitch down up angular rate the controller will
 * output (in degrees per second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_P_RMAX_NEG, 60.0f);

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
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PR_IMAX, 0.4f);

/**
 * Roll rate proportional Gain
 *
 * This defines how much the aileron input will be commanded depending on the
 * current body angular rate error.
 *
 * @unit %/rad/s
 * @min 0.005
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_P, 0.05f);

/**
 * Roll rate integrator Gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.005
 * @max 0.2
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_I, 0.01f);

/**
 * Roll Integrator Anti-Windup
 *
 * The portion of the integrator part in the control surface deflection is limited to this value.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RR_IMAX, 0.2f);

/**
 * Maximum Roll Rate
 *
 * This limits the maximum roll rate the controller will output (in degrees per
 * second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_R_RMAX, 70.0f);

/**
 * Yaw rate proportional gain
 *
 * This defines how much the rudder input will be commanded depending on the
 * current body angular rate error.
 *
 * @unit %/rad/s
 * @min 0.005
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_P, 0.05f);

/**
 * Yaw rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_I, 0.0f);

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
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_IMAX, 0.2f);

/**
 * Maximum Yaw Rate
 *
 * This limits the maximum yaw rate the controller will output (in degrees per
 * second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_Y_RMAX, 0.0f);

/**
 * Roll control to yaw control feedforward gain.
 *
 * This gain can be used to counteract the "adverse yaw" effect for fixed wings.
 * When the plane enters a roll it will tend to yaw the nose out of the turn.
 * This gain enables the use of a yaw actuator (rudder, airbrakes, ...) to counteract
 * this effect.
 *
 * @min 0.0
 * @decimal 1
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RLL_TO_YAW_FF, 0.0f);

/**
 * Enable wheel steering controller
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_W_EN, 0);


/**
 * Wheel steering rate proportional gain
 *
 * This defines how much the wheel steering input will be commanded depending on the
 * current body angular rate error.
 *
 * @unit %/rad/s
 * @min 0.005
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_P, 0.5f);

/**
 * Wheel steering rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.005
 * @max 0.5
 * @decimal 3
 * @increment 0.005
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_I, 0.1f);

/**
 * Wheel steering rate integrator limit
 *
 * The portion of the integrator part in the control surface deflection is
 * limited to this value
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_IMAX, 1.0f);

/**
 * Maximum wheel steering rate
 *
 * This limits the maximum wheel steering rate the controller will output (in degrees per
 * second). Setting a value of zero disables the limit.
 *
 * @unit deg/s
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_W_RMAX, 0.0f);

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
 * @group FW Attitude Control
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
 * @group FW Attitude Control
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
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YR_FF, 0.3f);

/**
 * Wheel steering rate feed forward
 *
 * Direct feed forward from rate setpoint to control surface output
 *
 * @unit %/rad/s
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_WR_FF, 0.2f);

/**
 * Minimal speed for yaw coordination
 *
 * For airspeeds above this value, the yaw rate is calculated for a coordinated
 * turn. Set to a very high value to disable.
 *
 * @unit m/s
 * @min 0.0
 * @max 1000.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_YCO_VMIN, 1000.0f);

/**
 * Method used for yaw coordination
 *
 * The param value sets the method used to calculate the yaw rate
 * 0: open-loop zero lateral acceleration based on kinematic constraints
 * 1: closed-loop: try to reduce lateral acceleration to 0 by measuring the acceleration
 *
 * @min 0
 * @max 1
 * @value 0 open-loop
 * @value 1 closed-loop
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_YCO_METHOD, 0);

/**
 * Roll Setpoint Offset
 *
 * An airframe specific offset of the roll setpoint in degrees, the value is
 * added to the roll setpoint and should correspond to the typical cruise speed
 * of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RSP_OFF, 0.0f);

/**
 * Pitch Setpoint Offset
 *
 * An airframe specific offset of the pitch setpoint in degrees, the value is
 * added to the pitch setpoint and should correspond to the typical cruise
 * speed of the airframe.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_PSP_OFF, 0.0f);

/**
 * Max Manual Roll
 *
 * Max roll for manual control in attitude stabilized mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_R_MAX, 45.0f);

/**
 * Max Manual Pitch
 *
 * Max pitch for manual control in attitude stabilized mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.5
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_P_MAX, 45.0f);

/**
 * Scale factor for flaps
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_SCL, 1.0f);

/**
 * Scale factor for flaperons
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPERON_SCL, 0.0f);

/**
 * Disable airspeed sensor
 *
 * For small wings or VTOL without airspeed sensor this parameter can be used to
 * enable flying without an airspeed reading
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_ARSP_MODE, 0);

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
 * @group FW Attitude Control
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
 * @group FW Attitude Control
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
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_MAN_Y_SC, 1.0f);

/**
 * Whether to scale throttle by battery power level
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The fixed wing
 * should constantly behave as if it was fully charged with reduced max thrust
 * at lower battery percentages. i.e. if cruise speed is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group FW Attitude Control
 */
PARAM_DEFINE_INT32(FW_BAT_SCALE_EN, 0);

/**
 * Acro body x max rate.
 *
 * This is the rate the controller is trying to achieve if the user applies full roll
 * stick input in acro mode.
 *
 * @min 45
 * @max 720
 * @unit degrees
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_X_MAX, 90);

/**
 * Acro body y max rate.
 *
 * This is the body y rate the controller is trying to achieve if the user applies full pitch
 * stick input in acro mode.
 *
 * @min 45
 * @max 720
 * @unit degrees
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Y_MAX, 90);

/**
 * Acro body z max rate.
 *
 * This is the body z rate the controller is trying to achieve if the user applies full yaw
 * stick input in acro mode.
 *
 * @min 10
 * @max 180
 * @unit degrees
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_ACRO_Z_MAX, 45);

/**
 * Threshold for Rattitude mode
 *
 * Manual input needed in order to override attitude control rate setpoints
 * and instead pass manual stick inputs as rate setpoints
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_RATT_TH, 0.8f);
