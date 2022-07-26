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
 * @file rover_pos_control_params.c
 *
 * Parameters defined by the position control task for ground rovers
 *
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */


/// L1 GUIDANCE parameters
/**
 * L1 distance
 *
 * This is the distance at which the next waypoint is activated. This should be set
 * to about 2-4x of GND_WHEEL_BASE and not smaller than one meter (due to GPS accuracy).
 *
 *
 * @unit m
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_L1_DIST, 1.0f);

/**
 * L1 period
 *
 * This is the L1 distance and defines the tracking
 * point ahead of the rover it's following.
 * Use values around 2-5m for a 0.3m wheel base. Tuning instructions: Shorten
 * slowly during tuning until response is sharp without oscillation.
 *
 * @unit m
 * @min 0.5
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_L1_PERIOD, 5.0f);

/**
 * L1 damping
 *
 * Damping factor for L1 control.
 *
 * @min 0.6
 * @max 0.9
 * @decimal 2
 * @increment 0.05
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_L1_DAMPING, 0.75f);


/// THROTTLE settings
/**
 * Cruise throttle
 *
 * This is the throttle setting required to achieve the desired cruise speed.
 * 10% is ok for a traxxas stampede vxl with ESC set to training mode
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_CRUISE, 0.1f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MIN, 0.0f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller.
 * For a Traxxas stampede vxl with the ESC set to training, 30 % is enough
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MAX, 0.5f);


/// POSITION control parameters
/**
 * Trim ground speed
 *
 * Rover will try to achieve this speed while in position control mode
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_TRIM, 3.0f);

/**
 * Maximum ground speed
 *
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_MAX, 10.0f);


/// VELOCITY control parameters
/**
 * Control mode for speed
 *
 * This allows the user to choose between closed loop control or open loop cruise throttle speed.
 * Open loop will set the throttle output to GND_THR_CRUISE value by default. But if there's a cruise_speed
 * set in the posiiont setpoint triplet, it will override with that value instead.
 *
 * @min 0
 * @max 1
 * @value 0 open loop control
 * @value 1 closed loop PID control
 * @group Rover Position Control
 */
PARAM_DEFINE_INT32(GND_SP_CTRL_MODE, 1);

/**
 * Speed proportional gain
 *
 * This is the proportional gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_P, 2.0f);

/**
 * Speed Integral gain
 *
 * This is the integral gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_I, 3.0f);

/**
 * Speed proportional gain
 *
 * This is the derivative gain for the speed closed loop controller
 *
 * @unit %m/s
 * @min 0.00
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_D, 0.001f);

/**
 * Speed integral maximum value
 *
 * This is the maximum value the integral can reach to prevent wind-up.
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_IMAX, 1.0f);

/**
 * Speed to throttle scaler
 *
 * This is a gain to map the speed control output to the throttle linearly.
 * PID output of the speed control is in acceleration setpoint unit [m/s^2].
 * So this scalar converts the acceleration to throttle setpoint in range [0, 1]
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_THR_SC, 1.0f);


/// ATTITUDE control parameters
/**
 * Attitude control P gain
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_ATT_P, 1.0f);


/// RATE control parameters
/**
 * Rover Rate Proportional Gain
 *
 * @min 0.0
 * @max 10.0
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_P, 0.0f);

/**
 * Rover Rate Integral Gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_I, 0.0f);

/**
 * Rover Rate Derivative Gain
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_D, 0.0f);

/**
 * Rover Rate Feed forward Gain
 *
 * @min 0.0
 * @max 1.5
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_FF, 1.0f);

/**
 * Rover Rate Maximum Integral Gain
 *
 * @unit rad/s
 * @min 0.0
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_IMAX, 1.0f);

/**
 * Rover Maximum Rate Setpoint
 *
 * This is the angular rate setpoint that will be commanded with full
 * yaw control command (e.g. roll stick to either right/left). This
 * must be a physically achievable rate (e.g. speed boats may have much higher value)
 *
 * @unit rad/s
 * @min 0.0
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_MAX, 1.0f);

/**
 * Rover Rate Integral Minimum speed
 *
 * @unit m/s
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_RATE_IMINSPD, 0.3f);


/// ROVER GEOMETRY parameters
/**
 * Distance from front axle to rear axle
 *
 * A value of 0.31 is typical for 1/10 RC cars.
 *
 * @unit m
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_WHEEL_BASE, 0.31f);

/**
 * Maximum wheel turn angle for Ackerman steering.
 *
 * Magnitude of the angle wheel turns when maximum steering actuator output is applied.
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @increment 0.1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_MAX_ANG, 45.0f);


/// Other parameters
/**
 * Maximum manual yaw rate in stabilized mode
 *
 * Full roll stick deflection in either direction will achieve the yaw setpoint
 * changing at the rate specified with this parameter
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group Rover Position Control
 */
PARAM_DEFINE_FLOAT(GND_MAN_Y_MAX, 150.0f);
