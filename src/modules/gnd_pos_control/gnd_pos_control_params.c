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
 * @file gnd_pos_control_params.c
 *
 * Parameters defined by the position control task for ground rovers
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
 */

/**
 * L1 distance
 *
 * This is the waypoint radius
 *
 *
 * @unit m
 * @min 0.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_L1_DIST, 5.0f);

/**
 * L1 period
 *
 * This is the L1 distance and defines the tracking
 * point ahead of the rover it's following.
 * Using values around 2-5 for a traxxas stampede. Shorten
 * slowly during tuning until response is sharp without oscillation.
 *
 * @unit m
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_L1_PERIOD, 10.0f);

/**
 * L1 damping
 *
 * Damping factor for L1 control.
 *
 * @min 0.6
 * @max 0.9
 * @decimal 2
 * @increment 0.05
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_L1_DAMPING, 0.75f);

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
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_CRUISE, 0.1f);

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
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MAX, 0.3f);

/**
 * Negative pitch limit
 *
 * The minimum negative pitch the controller will output.
 *
 * @unit deg
 * @min -60.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_P_LIM_MIN, -20.0f);

/**
 * Positive pitch limit
 *
 * The maximum positive pitch the controller will output.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_P_LIM_MAX, 20.0f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 * Set to 0 for rover
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground, it should be 0 for a rover
 *
 *
 * @unit norm
 * @min 0.0
 * @max 0.4
 * @decimal 2
 * @increment 0.01
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_IDLE, 0.0f);

/**
 * Control mode for speed
 *
 * This allows the user to choose between closed loop gps speed or open loop cruise throttle speed
 * @min 0
 * @max 1
 * @value 0 open loop control
 * @value 1 Close the loop with gps speed
 * @group GND Attitude Control
 */
PARAM_DEFINE_INT32(GND_SP_CTRL_MODE, 0);

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
 * @group GND Attitude Control
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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_I, 0.1f);

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
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_D, 0.0f);

/**
 * Speed integral maximum value
 *
 * This is the maxim value the integral can reach to prevent wind-up.
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_IMAX, 1.0f);

/**
 * Speed to throttle scaler
 *
 * This is a gain to map the speed control output to the throttle linearly.
 *
 * @unit %m/s
 * @min 0.005
 * @max 50.0
 * @decimal 3
 * @increment 0.005
 * @group GND Attitude Control
 */
PARAM_DEFINE_FLOAT(GND_SPEED_THR_SC, 1.0f);



