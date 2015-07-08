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
PARAM_DEFINE_FLOAT(FW_L1_PERIOD, 20.0f);

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
PARAM_DEFINE_FLOAT(FW_THR_CRUISE, 0.6f);

/**
 * Throttle max slew rate
 *
 * Maximum slew rate for the commanded throttle
 *
 * @min 0.0
 * @max 1.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX, 0.0f);

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
 * @min 35.0
 * @max 65.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 50.0f);

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
 * @min 0.0
 * @max 1.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Throttle limit value before flare
 *
 * This throttle value will be set as throttle limit at FW_LND_TLALT,
 * before arcraft will flare.
 *
 * @min 0.0
 * @max 1.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_THR_LND_MAX, 1.0f);

/**
 * Climbout Altitude difference
 *
 * If the altitude error exceeds this parameter, the system will climb out
 * with maximum throttle and minimum airspeed until it is closer than this
 * distance to the desired altitude. Mostly used for takeoff waypoints / modes.
 * Set to zero to disable climbout mode (not recommended).
 *
 * @min 0.0
 * @max 150.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_CLMBOUT_DIFF, 25.0f);

/**
 * Maximum climb rate
 *
 * This is the best climb rate that the aircraft can achieve with
 * the throttle set to THR_MAX and the airspeed set to the
 * default value. For electric aircraft make sure this number can be
 * achieved towards the end of flight when the battery voltage has reduced.
 * The setting of this parameter can be checked by commanding a positive
 * altitude change of 100m in loiter, RTL or guided mode. If the throttle
 * required to climb is close to THR_MAX and the aircraft is maintaining
 * airspeed, then this parameter is set correctly. If the airspeed starts
 * to reduce, then the parameter is set to high, and if the throttle
 * demand required to climb and maintain speed is noticeably less than
 * FW_THR_MAX, then either FW_T_CLMB_MAX should be increased or
 * FW_THR_MAX reduced.
 *
 * @min 2.0
 * @max 10.0
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX, 5.0f);

/**
 * Minimum descent rate
 *
 * This is the sink rate of the aircraft with the throttle
 * set to THR_MIN and flown at the same airspeed as used
 * to measure FW_T_CLMB_MAX.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MIN, 2.0f);

/**
 * Maximum descent rate
 *
 * This sets the maximum descent rate that the controller will use.
 * If this value is too large, the aircraft can over-speed on descent.
 * This should be set to a value that can be achieved without
 * exceeding the lower pitch angle limit and without over-speeding
 * the aircraft.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);

/**
 * TECS time constant
 *
 * This is the time constant of the TECS control algorithm (in seconds).
 * Smaller values make it faster to respond, larger values make it slower
 * to respond.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_TIME_CONST, 5.0f);

/**
 * TECS Throttle time constant
 *
 * This is the time constant of the TECS throttle control algorithm (in seconds).
 * Smaller values make it faster to respond, larger values make it slower
 * to respond.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_THRO_CONST, 8.0f);

/**
 * Throttle damping factor
 *
 * This is the damping gain for the throttle demand loop.
 * Increase to add damping to correct for oscillations in speed and height.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_THR_DAMP, 0.5f);

/**
 * Integrator gain
 *
 * This is the integrator gain on the control loop.
 * Increasing this gain increases the speed at which speed
 * and height offsets are trimmed out, but reduces damping and
 * increases overshoot.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_INTEG_GAIN, 0.1f);

/**
 * Maximum vertical acceleration
 *
 * This is the maximum vertical acceleration (in metres/second square)
 * either up or down that the controller will use to correct speed
 * or height errors. The default value of 7 m/s/s (equivalent to +- 0.7 g)
 * allows for reasonably aggressive pitch changes if required to recover
 * from under-speed conditions.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 7.0f);

/**
 * Complementary filter "omega" parameter for height
 *
 * This is the cross-over frequency (in radians/second) of the complementary
 * filter used to fuse vertical acceleration and barometric height to obtain
 * an estimate of height rate and height. Increasing this frequency weights
 * the solution more towards use of the barometer, whilst reducing it weights
 * the solution more towards use of the accelerometer data.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_HGT_OMEGA, 3.0f);

/**
 * Complementary filter "omega" parameter for speed
 *
 * This is the cross-over frequency (in radians/second) of the complementary
 * filter used to fuse longitudinal acceleration and airspeed to obtain an
 * improved airspeed estimate. Increasing this frequency weights the solution
 * more towards use of the arispeed sensor, whilst reducing it weights the
 * solution more towards use of the accelerometer data.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_OMEGA, 2.0f);

/**
 * Roll -> Throttle feedforward
 *
 * Increasing this gain turn increases the amount of throttle that will
 * be used to compensate for the additional drag created by turning.
 * Ideally this should be set to  approximately 10 x the extra sink rate
 * in m/s created by a 45 degree bank turn. Increase this gain if
 * the aircraft initially loses energy in turns and reduce if the
 * aircraft initially gains energy in turns. Efficient high aspect-ratio
 * aircraft (eg powered sailplanes) can use a lower value, whereas
 * inefficient low aspect-ratio models (eg delta wings) can use a higher value.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 10.0f);

/**
 * Speed <--> Altitude priority
 *
 * This parameter adjusts the amount of weighting that the pitch control
 * applies to speed vs height errors. Setting it to 0.0 will cause the
 * pitch control to control height and ignore speed errors. This will
 * normally improve height accuracy but give larger airspeed errors.
 * Setting it to 2.0 will cause the pitch control loop to control speed
 * and ignore height errors. This will normally reduce airspeed errors,
 * but give larger height errors. The default value of 1.0 allows the pitch
 * control to simultaneously control height and speed.
 * Note to Glider Pilots - set this parameter to 2.0 (The glider will
 * adjust its pitch angle to maintain airspeed, ignoring changes in height).
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPDWEIGHT, 1.0f);

/**
 * Pitch damping factor
 *
 * This is the damping gain for the pitch demand loop. Increase to add
 * damping to correct for oscillations in height. The default value of 0.0
 * will work well provided the pitch to servo controller has been tuned
 * properly.
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.0f);

/**
 * Height rate P factor
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_HRATE_P, 0.05f);

/**
 * Height rate FF factor
 *
 * @group Fixed Wing TECS
 */
PARAM_DEFINE_FLOAT(FW_T_HRATE_FF, 0.0f);

/**
 * Speed rate P factor
 *
 * @group Fixed Wing TECS
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
 * Landing flare altitude (relative to landing altitude)
 *
 * @unit meter
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 8.0f);

/**
 * Landing throttle limit altitude (relative landing altitude)
 *
 * Default of -1.0f lets the system default to applying throttle
 * limiting at 2/3 of the flare altitude.
 *
 * @unit meter
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_TLALT, -1.0f);

/**
 * Landing heading hold horizontal distance
 *
 * @group L1 Control
 */
PARAM_DEFINE_FLOAT(FW_LND_HHDIST, 15.0f);

/**
 * Enable or disable usage of terrain estimate during landing
 *
 * 0: disabled, 1: enabled
 *
 * @group L1 Control
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during flare, a positive sign means nose up
 * Applied once FW_LND_TLALT is reached
 *
 */
PARAM_DEFINE_FLOAT(FW_FLARE_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during flare, a positive sign means nose up
 * Applied once FW_LND_TLALT is reached
 *
 */
PARAM_DEFINE_FLOAT(FW_FLARE_PMAX, 15.0f);
