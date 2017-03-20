/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file GND_pos_control_l1_params.c
 *
 * Parameters defined by the L1 position control task
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Marco Zorzi <mzorzi@student.ethz.ch>
 */

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * L1 period
 *
 * This is the L1 distance and defines the tracking
 * point ahead of the aircraft its following.
 * A value of 18-25 meters works for most aircraft. Shorten
 * slowly during tuning until response is sharp without oscillation.
 *
 * @unit m
 * @min 12.0
 * @max 50.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_L1_PERIOD, 20.0f);

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
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_CRUISE, 0.3f);

/**
 * Throttle max slew rate
 *
 * Maximum slew rate for the commanded throttle
 *
 * @min 0.0
 * @max 1.0
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_SLEW_MAX, 0.0f);

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
PARAM_DEFINE_FLOAT(GND_P_LIM_MIN, -45.0f);

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
PARAM_DEFINE_FLOAT(GND_P_LIM_MAX, 45.0f);

/**
 * Controller roll limit
 *
 * The maximum roll the controller will output.
 *
 * @unit deg
 * @min 35.0
 * @max 65.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_R_LIM, 50.0f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller.
 * For overpowered aircraft, this should be reduced to a value that
 * provides sufficient thrust to climb at the maximum pitch angle PTCH_MAX.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_MAX, 0.5f);

/**
 * Throttle limit min
 *
 * This is the minimum throttle % that can be used by the controller.
 * For electric aircraft this will normally be set to zero, but can be set
 * to a small non-zero value if a folding prop is fitted to prevent the
 * prop from folding and unfolding repeatedly in-flight or to provide
 * some aerodynamic drag from a turning prop to improve the descent rate.
 *
 * For aircraft with internal combustion engine this parameter should be set
 * for desired idle rpm.
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
 * This is the minimum throttle while on the ground
 *
 * For aircraft with internal combustion engine this parameter should be set
 * above desired idle rpm.
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
 * Throttle limit value before flare
 *
 * This throttle value will be set as throttle limit at GND_LND_TLALT,
 * before aircraft will flare.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_THR_LND_MAX, 0.0f);

/**
 * Climbout Altitude difference
 *
 * If the altitude error exceeds this parameter, the system will climb out
 * with maximum throttle and minimum airspeed until it is closer than this
 * distance to the desired altitude. Mostly used for takeoff waypoints / modes.
 * Set to 0 to disable climbout mode (not recommended).
 *
 * @unit m
 * @min 0.0
 * @max 150.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_CLMBOUT_DIFF, 10.0f);

/**
 * Landing slope angle
 *
 * @unit deg
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_ANG, 5.0f);

/**
 *
 *
 * @unit m
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_HVIRT, 10.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * @unit m
 * @min 0.0
 * @max 25.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_FLALT, 8.0f);

/**
 * Landing throttle limit altitude (relative landing altitude)
 *
 * Default of -1.0 lets the system default to applying throttle
 * limiting at 2/3 of the flare altitude.
 *
 * @unit m
 * @min -1.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_TLALT, -1.0f);

/**
 * Landing heading hold horizontal distance
 *
 * @unit m
 * @min 0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_HHDIST, 15.0f);

/**
 * Use terrain estimate during landing
 *
 * @boolean
 * @group GND POS Control
 */
PARAM_DEFINE_INT32(GND_LND_USETER, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during flare, a positive sign means nose up
 * Applied once GND_LND_TLALT is reached
 *
 * @unit deg
 * @min 0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during flare, a positive sign means nose up
 * Applied once GND_LND_TLALT is reached
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_FL_PMAX, 15.0f);

/**
 * Min. airspeed scaling factor for landing
 *
 * Multiplying this factor with the minimum airspeed of the plane
 * gives the target airspeed the landing approach.
 * GND_AIRSPD_MIN * GND_LND_AIRSP_SC
 *
 * @unit norm
 * @min 1.0
 * @max 1.5
 * @decimal 2
 * @increment 0.01
 * @group GND POS Control
 */
PARAM_DEFINE_FLOAT(GND_LND_AIRSP_SC, 1.3f);

