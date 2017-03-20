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
PARAM_DEFINE_FLOAT(GND_L1_DIST, 2.0f);
