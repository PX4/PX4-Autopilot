/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Trim throttle
 *
 * Required throttle (at sea level, standard atmosphere) for level flight at FW_AIRSPD_TRIM
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_THR_TRIM, 0.6f);

/**
 * Throttle at min airspeed
 *
 * Required throttle (at sea level, standard atmosphere) for level flight at minimum airspeed FW_AIRSPD_MIN
 *
 * Set to 0 to disable mapping of airspeed to trim throttle below FW_AIRSPD_TRIM.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_THR_ASPD_MIN, 0.f);

/**
 * Throttle at max airspeed
 *
 * Required throttle (at sea level, standard atmosphere) for level flight at maximum airspeed FW_AIRSPD_MAX
 *
 * Set to 0 to disable mapping of airspeed to trim throttle.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.01
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_THR_ASPD_MAX, 0.f);


/**
 * Service ceiling
 *
 * Altitude in standard atmosphere at which the vehicle in normal configuration (WEIGHT_BASE) is still able to achieve a maximum climb rate of
 * 0.5m/s at maximum throttle (FW_THR_MAX). Used to compensate for air density in FW_T_CLMB_MAX.
 * Set negative to disable.
 *
 * @min -1.0
 * @unit m
 * @decimal 0
 * @increment 1.0
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_SERVICE_CEIL, -1.0f);

/**
 * Vehicle base weight.
 *
 * This is the weight of the vehicle at which it's performance limits were derived. A zero or negative value
 * disables trim throttle and minimum airspeed compensation based on weight.
 *
 * @unit kg
 * @decimal 1
 * @increment 0.5
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(WEIGHT_BASE, -1.0f);

/**
 * Vehicle gross weight.
 *
 * This is the actual weight of the vehicle at any time. This value will differ from WEIGHT_BASE in case weight was added
 * or removed from the base weight. Examples are the addition of payloads or larger batteries. A zero or negative value
 * disables trim throttle and minimum airspeed compensation based on weight.
 *
 * @unit kg
 * @decimal 1
 * @increment 0.1
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(WEIGHT_GROSS, -1.0f);

/**
 * Maximum climb rate
 *
 * This is the maximum calibrated climb rate that the aircraft can achieve with
 * the throttle set to FW_THR_MAX and the airspeed set to the
 * trim value. For electric aircraft make sure this number can be
 * achieved towards the end of flight when the battery voltage has reduced.
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_MAX, 5.0f);

/**
 * Minimum descent rate
 *
 * This is the minimum calibrated sink rate of the aircraft with the throttle
 * set to THR_MIN and flown at the same airspeed as used
 * to measure FW_T_CLMB_MAX.
 *
 * @unit m/s
 * @min 1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.5
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MIN, 2.0f);

/**
 * Trim (Cruise) Airspeed
 *
 * The trim CAS (calibrated airspeed) of the vehicle. If an airspeed controller is active,
 * this is the default airspeed setpoint that the controller will try to achieve.
 * This value corresponds to the trim airspeed with the default load factor (level flight, default weight).
 *
 * @unit m/s
 * @min 0.5
 * @decimal 1
 * @increment 0.5
 * @group FW Performance
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
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_STALL, 7.0f);

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
 * @group FW Performance
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
 * @group FW Performance
 */
PARAM_DEFINE_FLOAT(FW_AIRSPD_MAX, 20.0f);
