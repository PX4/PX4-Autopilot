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
 * Path navigation roll slew rate limit.
 *
 * The maximum change in roll angle setpoint per second.
 * This limit is applied in all Auto modes, plus manual Position and Altitude modes.
 *
 * @unit deg/s
 * @min 0
 * @decimal 0
 * @increment 1
 * @group FW Path Control
 */
PARAM_DEFINE_FLOAT(FW_PN_R_SLEW_MAX, 90.0f);

/**
 * NPFG period
 *
 * Period of the NPFG control law.
 *
 * @unit s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD, 10.0f);

/**
 * NPFG damping ratio
 *
 * Damping ratio of the NPFG control law.
 *
 * @min 0.10
 * @max 1.00
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_DAMPING, 0.7f);

/**
 * Enable automatic lower bound on the NPFG period
 *
 * Avoids limit cycling from a too aggressively tuned period/damping combination.
 * If set to false, also disables the upper bound NPFG_PERIOD_UB.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_LB_PERIOD, 1);

/**
 * Enable automatic upper bound on the NPFG period
 *
 * Adapts period to maintain track keeping in variable winds and path curvature.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_UB_PERIOD, 1);

/**
 * Enable track keeping excess wind handling logic.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_TRACK_KEEP, 1);

/**
 * Enable minimum forward ground speed maintaining excess wind handling logic
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_EN_MIN_GSP, 1);

/**
 * Enable wind excess regulation.
 *
 * Disabling this parameter further disables all other airspeed incrementation options.
 *
 * @boolean
 * @group FW NPFG Control
 */
PARAM_DEFINE_INT32(NPFG_WIND_REG, 1);

/**
 * Maximum, minimum forward ground speed for track keeping in excess wind
 *
 * The maximum value of the minimum forward ground speed that may be commanded
 * by the track keeping excess wind handling logic. Commanded in full at the normalized
 * track error fraction of the track error boundary and reduced to zero on track.
 *
 * @unit m/s
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_GSP_MAX_TK, 5.0f);

/**
 * Roll time constant
 *
 * Time constant of roll controller command / response, modeled as first order delay.
 * Used to determine lower period bound. Setting zero disables automatic period bounding.
 *
 * @unit s
 * @min 0.00
 * @max 2.00
 * @decimal 2
 * @increment 0.05
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_ROLL_TC, 0.5f);

/**
 * NPFG switch distance multiplier
 *
 * Multiplied by the track error boundary to determine when the aircraft switches
 * to the next waypoint and/or path segment. Should be less than 1.
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_SW_DST_MLT, 0.32f);

/**
 * Period safety factor
 *
 * Multiplied by period for conservative minimum period bounding (when period lower
 * bounding is enabled). 1.0 bounds at marginal stability.
 *
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.1
 * @group FW NPFG Control
 */
PARAM_DEFINE_FLOAT(NPFG_PERIOD_SF, 1.5f);


/**
 * Throttle max slew rate
 *
 * Maximum slew rate for the commanded throttle
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_SLEW_MAX, 0.0f);

/**
 * Minimum pitch angle
 *
 * The minimum pitch angle setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min -60.0
 * @max 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MIN, -30.0f);

/**
 * Maximum pitch angle
 *
 * The maximum pitch angle setpoint setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min 0.0
 * @max 60.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_P_LIM_MAX, 30.0f);

/**
 * Maximum roll angle
 *
 * The maximum roll angle setpoint for setpoint for a height-rate or altitude controlled mode.
 *
 * @unit deg
 * @min 35.0
 * @max 65.0
 * @decimal 1
 * @increment 0.5
 * @group FW Path Control
 */
PARAM_DEFINE_FLOAT(FW_R_LIM, 50.0f);

/**
 * Throttle limit max
 *
 * Maximum throttle limit in altitude controlled modes.
 * Should be set accordingly to achieve FW_T_CLMB_MAX.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_MAX, 1.0f);

/**
 * Throttle limit min
 *
 * Minimum throttle limit in altitude controlled modes.
 * Usually set to 0 but can be increased to prevent the motor from stopping when
 * descending, which can increase achievable descent rates.
 *
 * For aircraft with internal combustion engine this parameter should be set
 * for desired idle rpm.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_MIN, 0.0f);

/**
 * Idle throttle
 *
 * This is the minimum throttle while on the ground
 *
 * For aircraft with internal combustion engines, this parameter should be set
 * above the desired idle rpm. For electric motors, idle should typically be set
 * to zero.
 *
 * Note that in automatic modes, "landed" conditions will engage idle throttle.
 *
 * @unit norm
 * @min 0.0
 * @max 0.4
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_THR_IDLE, 0.0f);

/**
 * Maximum landing slope angle
 *
 * Typically the desired landing slope angle when landing configuration (flaps, airspeed) is enabled.
 * Set this value within the vehicle's performance limits.
 *
 * @unit deg
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_ANG, 5.0f);

/**
 * Minimum pitch during takeoff.
 *
 * @unit deg
 * @min -5.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group FW Path Control
 */
PARAM_DEFINE_FLOAT(FW_TKO_PITCH_MIN, 10.0f);

/**
 * Takeoff Airspeed
 *
 * The calibrated airspeed setpoint TECS will stabilize to during the takeoff climbout.
 *
 * If set <= 0.0, FW_AIRSPD_MIN will be set by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_TKO_AIRSPD, -1.0f);

/**
 * Landing flare altitude (relative to landing altitude)
 *
 * NOTE: max(FW_LND_FLALT, FW_LND_FL_TIME * |z-velocity|) is taken as the flare altitude
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FLALT, 0.5f);

/**
 * Use terrain estimation during landing. This is critical for detecting when to flare, and should be enabled if possible.
 *
 * NOTE: terrain estimate is currently solely derived from a distance sensor.
 *
 * If enabled and no measurement is found within a given timeout, the landing waypoint altitude will be used OR the landing
 * will be aborted, depending on the criteria set in FW_LND_ABORT.
 *
 * If disabled, FW_LND_ABORT terrain based criteria are ignored.
 *
 * @min 0
 * @max 2
 * @value 0 Disable the terrain estimate
 * @value 1 Use the terrain estimate to trigger the flare (only)
 * @value 2 Calculate landing glide slope relative to the terrain estimate
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_USETER, 1);

/**
 * Early landing configuration deployment
 *
 * When disabled, the landing configuration (flaps, landing airspeed, etc.) is only activated
 * on the final approach to landing. When enabled, it is already activated when entering the
 * final loiter-down (loiter-to-alt) waypoint before the landing approach.
 *
 * @boolean
 *
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_EARLYCFG, 0);

/**
 * Flare, minimum pitch
 *
 * Minimum pitch during flare, a positive sign means nose up
 * Applied once flaring is triggered
 *
 * @unit deg
 * @min -5
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMIN, 2.5f);

/**
 * Flare, maximum pitch
 *
 * Maximum pitch during flare, a positive sign means nose up
 * Applied once flaring is triggered
 *
 * @unit deg
 * @min 0
 * @max 45.0
 * @decimal 1
 * @increment 0.5
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_PMAX, 15.0f);

/**
 * Landing airspeed
 *
 * The calibrated airspeed setpoint during landing.
 *
 * If set <= 0.0, landing airspeed = FW_AIRSPD_MIN by default.
 *
 * @unit m/s
 * @min -1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_AIRSPD, -1.f);

/**
 * Altitude time constant factor for landing
 *
 * Set this parameter to less than 1.0 to make TECS react faster to altitude errors during
 * landing than during normal flight. During landing, the TECS
 * altitude time constant (FW_T_ALT_TC) is multiplied by this value.
 *
 * @unit
 * @min 0.2
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_THRTC_SC, 1.0f);

/*
 * TECS parameters
 *
 */

/**
 * Maximum descent rate
 *
 * This sets the maximum descent rate that the controller will use.
 * If this value is too large, the aircraft can over-speed on descent.
 * This should be set to a value that can be achieved without
 * exceeding the lower pitch angle limit and without over-speeding
 * the aircraft.
 *
 * @unit m/s
 * @min 1.0
 * @max 15.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_MAX, 5.0f);

/**
 * Throttle damping factor
 *
 * This is the damping gain for the throttle demand loop.
 * Increase to add damping to correct for oscillations in speed and height.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_THR_DAMPING, 0.05f);

/**
 * Integrator gain throttle
 *
 * Integrator gain on the throttle part of the control loop.
 * Increase it to trim out speed and height offsets faster,
 * with the downside of possible overshoots and oscillations.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_THR_INTEG, 0.02f);

/**
 * Integrator gain pitch
 *
 * Integrator gain on the pitch part of the control loop.
 * Increase it to trim out speed and height offsets faster,
 * with the downside of possible overshoots and oscillations.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_I_GAIN_PIT, 0.1f);

/**
 * Maximum vertical acceleration
 *
 * This is the maximum vertical acceleration (in m/s/s)
 * either up or down that the controller will use to correct speed
 * or height errors. The default value of 7 m/s/s (equivalent to +- 0.7 g)
 * allows for reasonably aggressive pitch changes if required to recover
 * from under-speed conditions.
 *
 * @unit m/s^2
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_VERT_ACC, 7.0f);

/**
 * Airspeed measurement standard deviation for airspeed filter.
 *
 * This is the measurement standard deviation for the airspeed used in the airspeed filter in TECS.
 *
 * @unit m/s
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_STD, 0.2f);

/**
 * Airspeed rate measurement standard deviation for airspeed filter.
 *
 * This is the measurement standard deviation for the airspeed rate used in the airspeed filter in TECS.
 *
 * @unit m/s^2
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_DEV_STD, 0.2f);

/**
 * Process noise standard deviation for the airspeed rate in the airspeed filter.
 *
 * This is the process noise standard deviation in the airspeed filter filter defining the noise in the
 * airspeed rate for the constant airspeed rate model. This is used to define how much the airspeed and
 * the airspeed rate are filtered. The smaller the value the more the measurements are smoothed with the
 * drawback for delays.
 *
 * @unit m/s^2
 * @min 0.01
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SPD_PRC_STD, 0.2f);


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
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_RLL2THR, 15.0f);

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
 * Set to 2 for gliders.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 1
 * @increment 1.0
 * @group FW TECS
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
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_PTCH_DAMP, 0.1f);

/**
 * Altitude error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_ALT_TC, 5.0f);

/**
 * Minimum altitude error needed to descend with max airspeed. A negative value disables fast descend.
 *
 * @min -1.0
 * @decimal 0
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_F_ALT_ERR, -1.0f);

/**
 * Height rate feed forward
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_HRATE_FF, 0.3f);

/**
 * True airspeed error time constant.
 *
 * @min 2.0
 * @decimal 2
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_TAS_TC, 5.0f);

/**
 * Minimum groundspeed
 *
 * The controller will increase the commanded airspeed to maintain
 * this minimum groundspeed to the next waypoint.
 *
 * @unit m/s
 * @min 0.0
 * @max 40
 * @decimal 1
 * @increment 0.5
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_GND_SPD_MIN, 5.0f);

/**
 * RC stick configuration fixed-wing.
 *
 * Set RC/joystick configuration for fixed-wing manual position and altitude controlled flight.
 *
 * @min 0
 * @max 3
 * @bit 0 Alternative stick configuration (height rate on throttle stick, airspeed on pitch stick)
 * @bit 1 Enable airspeed setpoint via sticks in altitude and position flight mode
 * @group FW Path Control
 */
PARAM_DEFINE_INT32(FW_POS_STK_CONF, 2);

/**
 * Specific total energy rate first order filter time constant.
 *
 * This filter is applied to the specific total energy rate used for throttle damping.
 *
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_STE_R_TC, 0.4f);

/**
 * Specific total energy balance rate feedforward gain.
 *
 *
 * @min 0.5
 * @max 3
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SEB_R_FF, 1.0f);

/**
 * Default target climbrate.
 *
 * The default rate at which the vehicle will climb in autonomous modes to achieve altitude setpoints.
 * In manual modes this defines the maximum rate at which the altitude setpoint can be increased.
 *
 *
 * @unit m/s
 * @min 0.5
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_CLMB_R_SP, 3.0f);

/**
 * Default target sinkrate.
 *
 *
 * The default rate at which the vehicle will sink in autonomous modes to achieve altitude setpoints.
 * In manual modes this defines the maximum rate at which the altitude setpoint can be decreased.
 *
 * @unit m/s
 * @min 0.5
 * @max 15
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_T_SINK_R_SP, 2.0f);

/**
 * GPS failure loiter time
 *
 * The time in seconds the system should do open loop loiter and wait for GPS recovery
 * before it starts descending. Set to 0 to disable. Roll angle is set to FW_GPSF_R.
 * Does only apply for fixed-wing vehicles or VTOLs with NAV_FORCE_VT set to 0.
 *
 * @unit s
 * @min 0
 * @max 3600
 * @group Mission
 */
PARAM_DEFINE_INT32(FW_GPSF_LT, 30);

/**
 * GPS failure fixed roll angle
 *
 * Roll in degrees during the loiter after the vehicle has lost GPS in an auto mode (e.g. mission or loiter).
 *
 * @unit deg
 * @min 0.0
 * @max 30.0
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(FW_GPSF_R, 15.0f);


/**
 * The aircraft's wing span (length from tip to tip).
 *
 * This is used for limiting the roll setpoint near the ground. (if multiple wings, take the longest span)
 *
 * @unit m
 * @min 0.1
 * @decimal 1
 * @increment 0.1
 * @group FW Geometry
 */
PARAM_DEFINE_FLOAT(FW_WING_SPAN, 3.0);

/**
 * Height (AGL) of the wings when the aircraft is on the ground.
 *
 * This is used to constrain a minimum altitude below which we keep wings level to avoid wing tip strike. It's safer
 * to give a slight margin here (> 0m)
 *
 * @unit m
 * @min 0.0
 * @decimal 1
 * @increment 1
 * @group FW Geometry
 */
PARAM_DEFINE_FLOAT(FW_WING_HEIGHT, 0.5);

/**
 * Landing flare time
 *
 * Multiplied by the descent rate to calculate a dynamic altitude at which
 * to trigger the flare.
 *
 * NOTE: max(FW_LND_FLALT, FW_LND_FL_TIME * descent rate) is taken as the flare altitude
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_TIME, 1.0f);

/**
 * Landing touchdown time (since flare start)
 *
 * This is the time after the start of flaring that we expect the vehicle to touch the runway.
 * At this time, a 0.5s clamp down ramp will engage, constraining the pitch setpoint to RWTO_PSP.
 * If enabled, ensure that RWTO_PSP is configured appropriately for full gear contact on ground roll.
 *
 * Set to -1.0 to disable touchdown clamping. E.g. it may not be desirable to clamp on belly landings.
 *
 * The touchdown time will be constrained to be greater than or equal to the flare time (FW_LND_FL_TIME).
 *
 * @unit s
 * @min -1.0
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_TIME, -1.0f);

/**
 * Landing flare sink rate
 *
 * TECS will attempt to control the aircraft to this sink rate via pitch angle (throttle killed during flare)
 *
 * @unit m/s
 * @min 0.0
 * @max 2
 * @decimal 2
 * @increment 0.1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_FL_SINK, 0.25f);

/**
 * Maximum lateral position offset for the touchdown point
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 1
 * @increment 1
 * @group FW Auto Landing
 */
PARAM_DEFINE_FLOAT(FW_LND_TD_OFF, 3.0);

/**
 * Landing touchdown nudging option.
 *
 * Approach angle nudging: shifts the touchdown point laterally while keeping the approach entrance point constant
 * Approach path nudging: shifts the touchdown point laterally along with the entire approach path
 *
 * This is useful for manually adjusting the landing point in real time when map or GNSS errors cause an offset from the
 * desired landing vector. Nuding is done with yaw stick, constrained to FW_LND_TD_OFF (in meters) and the direction is
 * relative to the vehicle heading (stick deflection to the right = land point moves to the right as seen by the vehicle).
 *
 * @min 0
 * @max 2
 * @value 0 Disable nudging
 * @value 1 Nudge approach angle
 * @value 2 Nudge approach path
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_NUDGE, 2);

/**
 * Bit mask to set the automatic landing abort conditions.
 *
 * Terrain estimation:
 * bit 0: Abort if terrain is not found
 * bit 1: Abort if terrain times out (after a first successful measurement)
 *
 * The last estimate is always used as ground, whether the last valid measurement or the land waypoint, depending on the
 * selected abort criteria, until an abort condition is entered. If FW_LND_USETER == 0, these bits are ignored.
 *
 * TODO: Extend automatic abort conditions
 * e.g. glide slope tracking error (horizontal and vertical)
 *
 * @min 0
 * @max 3
 * @bit 0 Abort if terrain is not found (only applies to mission landings)
 * @bit 1 Abort if terrain times out (after a first successful measurement)
 * @group FW Auto Landing
 */
PARAM_DEFINE_INT32(FW_LND_ABORT, 3);

/**
 * Wind-based airspeed scaling factor
 *
 * Multiplying this factor with the current absolute wind estimate gives the airspeed offset
 * added to the minimum airspeed setpoint limit. This helps to make the
 * system more robust against disturbances (turbulence) in high wind.
 * Only applies to AUTO flight mode.
 *
 * @min 0
 * @decimal 2
 * @increment 0.01
 * @group FW TECS
 */
PARAM_DEFINE_FLOAT(FW_WIND_ARSP_SC, 0.f);

/**
 * Fixed-wing launch detection
 *
 * Enables automatic launch detection based on measured acceleration. Use for hand- or catapult-launched vehicles.
 * Not compatible with runway takeoff.
 *
 * @boolean
 * @group FW Launch detection
 */
PARAM_DEFINE_INT32(FW_LAUN_DETCN_ON, 0);

/**
 * Flaps setting during take-off
 *
 * Sets a fraction of full flaps during take-off.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_TO_SCL, 0.0f);

/**
 * Flaps setting during landing
 *
 * Sets a fraction of full flaps during landing.
 * Also applies to flaperons if enabled in the mixer/allocation.
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Rate Control
 */
PARAM_DEFINE_FLOAT(FW_FLAPS_LND_SCL, 1.0f);

/**
 * Spoiler landing setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_LND, 0.f);

/**
 * Spoiler descend setting
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(FW_SPOILERS_DESC, 0.f);
