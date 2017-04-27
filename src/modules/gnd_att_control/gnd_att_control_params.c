/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file gnd_att_control_params.c
 *
 * Parameters defined by the attitude control task for ground rovers
 *
 * This is a modification of the fixed wing params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 * 
 * All the ackowledgments and credits for the fw wing app are reported in those files.  
 * 
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Attitude Wheel Time Constant
 *
 * This defines the latency between a steering step input and the achieved setpoint
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
PARAM_DEFINE_FLOAT(GND_WR_TC, 0.4f);

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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_WR_P, 1.0f);

/**
 * Wheel steering rate integrator gain
 *
 * This gain defines how much control response will result out of a steady
 * state error. It trims any constant error.
 *
 * @unit %/rad
 * @min 0.00
 * @max 0.5
 * @decimal 3
 * @increment 0.005
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_WR_I, 0.00f);

/**
 * Wheel steering rate integrator gain
 *
 *
 * @unit %/rad
 * @min 0.00
 * @max 30
 * @decimal 3
 * @increment 0.005
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_WR_D, 0.00f);

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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_WR_IMAX, 0.0f);

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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_W_RMAX, 90.0f);


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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_WR_FF, 0.0f);

/**
 * Airspeed mode
 *
 * The param value sets the method used to publish the control state airspeed.
 *
 * @min 0
 * @max 2
 * @value 0 use measured airspeed
 * @value 1 use vehicle ground velocity as airspeed
 * @value 2 declare airspeed invalid
 * @group GND Attitude Control
 */
PARAM_DEFINE_INT32(GND_ARSP_MODE, 0);

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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_MAN_Y_SC, 1.0f);

/**
 * Groundspeed speed trim
 *
 * This allows to scale the turning radius depending on the speed.
 *
 * @unit norm
 * @min 0.0
 * @decimal 2
 * @increment 0.1
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_GSPD_SP_TRIM, 1.0f);


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
 * @group GND Attitude Control
 */
PARAM_DEFINE_INT32(GND_BAT_SCALE_EN, 0);
