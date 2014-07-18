/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file fw_pos_control_l1_params.c
 *
 * Parameters defined by the L1 position control task
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 */

/**
 * L1 period
 *
 * This is the L1 distance and defines the tracking
 * point ahead of the aircraft its following.
 * A value of 25 meters works for most aircraft. Shorten
 * slowly during tuning until response is sharp without oscillation.
 *
 * @min 1.0
 * @max 100.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_L1_PERIOD, 25.0f);

/**
 * L1 damping
 *
 * Damping factor for L1 control.
 *
 * @min 0.6
 * @max 0.9
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_L1_DAMPING, 0.75f);

/**
 * Cruise throttle
 *
 * This is the throttle setting required to achieve the desired cruise speed. Most airframes have a value of 0.5-0.7.
 *
 * @min 0.0
 * @max 1.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_CRUISE, 0.7f);

/**
 * Negative pitch limit
 *
 * The minimum negative pitch the controller will output.
 *
 * @unit degrees
 * @min -60.0
 * @max 0.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -45.0f);

/**
 * Positive pitch limit
 *
 * The maximum positive pitch the controller will output.
 *
 * @unit degrees
 * @min 0.0
 * @max 60.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 45.0f);

/**
 * Controller roll limit
 *
 * The maximum roll the controller will output.
 *
 * @unit degrees
 * @min 0.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 45.0f);

/**
 * Throttle limit max
 *
 * This is the maximum throttle % that can be used by the controller. 
 * For overpowered aircraft, this should be reduced to a value that 
 * provides sufficient thrust to climb at the maximum pitch angle PTCH_MAX.
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

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
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Throttle limit value before flare
 *
 * This throttle value will be set as throttle limit at FW_LND_TLALT, 
 * before arcraft will flare.
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_LND_MAX, 1.0f);

/**
 * Speed rate P factor
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_T_SRATE_P, 0.05f);

/**
 * Landing slope angle
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 *
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HVIRT, 10.0f);

/**
 * Landing flare altitude (relative)
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 15.0f);

/**
 * Landing throttle limit altitude (relative)
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_TLALT, 5.0f);

/**
 * Landing heading hold horizontal distance
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HHDIST, 15.0f);

/**
 * Relative altitude threshold for range finder measurements for use during landing
 *
 * range finder measurements will only be used if the estimated relative altitude (gobal_pos.alt - landing_waypoint.alt) is < FW_LND_RFRALT
 * set to < 0 to disable
 * the correct value of this parameter depends on your range measuring device as well as on the terrain at the landing location
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_RFRALT, -1.0f);
