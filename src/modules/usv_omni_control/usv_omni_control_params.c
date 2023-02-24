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
 * @file usv_omni_control_params.c
 *
 * Parameters defined by the position control task for unmanned underwater vehicles (USVs)
 *
 * This is a modification of the fixed wing/ground rover params and it is designed for ground rovers.
 * It has been developed starting from the fw  module, simplified and improved with dedicated items.
 *
 * All the ackowledgments and credits for the fw wing/rover app are reported in those files.
 *
 * @author Tim Hansen <t.hansen@jacobs-university.de>
 * @author Daniel Duecker <daniel.duecker@tuhh.de>
 */

/*
 * Controller parameters, accessible via MAVLink
 *
 */
/**
 * Minimum collective thrust in auto thrust control
 *
 * It's recommended to set it > 0 to avoid free fall with zero thrust.
 * Note: Without airmode zero thrust leads to zero roll/pitch control authority. (see MC_AIRMODE)
 *
 * @unit norm
 * @min 0.05
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_THR_MIN, 0.12f);

/**
 * Thrust curve in Manual Mode
 *
 * This parameter defines how the throttle stick input is mapped to commanded thrust
 * in Manual/Stabilized flight mode.
 *
 * In case the default is used ('Rescale to hover thrust'), the stick input is linearly
 * rescaled, such that a centered stick corresponds to the hover throttle (see USV_THR_HOVER).
 *
 * Select 'No Rescale' to directly map the stick 1:1 to the output. This can be useful
 * in case the hover thrust is very low and the default would lead to too much distortion
 * (e.g. if hover thrust is set to 20%, 80% of the upper thrust range is squeezed into the
 * upper half of the stick range).
 *
 * Note: In case USV_THR_HOVER is set to 50%, the modes 0 and 1 are the same.
 *
 * @value 0 Rescale to hover thrust
 * @value 1 No Rescale
 * @group USV Omni Control
 */
PARAM_DEFINE_INT32(USV_THR_CURVE, 0);

/**
 * Horizontal thrust margin
 *
 * Margin that is kept for horizontal control when prioritizing vertical thrust.
 * To avoid completely starving horizontal control with high vertical error.
 *
 * @unit norm
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_THR_XY_MARG, 0.3f);

/**
 * Maximum thrust in auto thrust control
 *
 * Limit max allowed thrust
 *
 * @unit norm
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_THR_MAX, 0.8f);


/**
 * Proportional gain for horizontal position error
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_P, 0.95f);

/**
 * Proportional gain for horizontal velocity error
 *
 * defined as correction acceleration in m/s^2 per m/s velocity error
 *
 * @min 1.2
 * @max 5.0
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_VEL_P_ACC, 1.8f);

/**
 * Integral gain for horizontal velocity error
 *
 * defined as correction acceleration in m/s^2 per m velocity integral
 * Non-zero value allows to eliminate steady state errors in the presence of disturbances like wind.
 *
 * @min 0.0
 * @max 60.0
 * @decimal 3
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_VEL_I_ACC, 0.4f);

/**
 * Differential gain for horizontal velocity error. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * defined as correction acceleration in m/s^2 per m/s^2 velocity derivative
 *
 * @min 0.1
 * @max 2.0
 * @decimal 3
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_VEL_D_ACC, 0.2f);

/**
 * Default horizontal velocity in mission
 *
 * Horizontal velocity used when flying autonomously in e.g. Missions, RTL, Goto.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_CRUISE, 5.0f);

/**
 * Proportional gain for horizontal trajectory position error
 *
 * @min 0.1
 * @max 1.0
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_TRAJ_P, 0.5f);

/**
 * Maximum horizontal error allowed by the trajectory generator
 *
 * The integration speed of the trajectory setpoint is linearly
 * reduced with the horizontal position tracking error. When the
 * error is above this parameter, the integration of the
 * trajectory is stopped to wait for the drone.
 *
 * This value can be adjusted depending on the tracking
 * capabilities of the vehicle.
 *
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_ERR_MAX, 2.0f);

/**
 * Maximum horizontal velocity setpoint in Position mode
 *
 * If velocity setpoint larger than USV_XY_VEL_MAX is set, then
 * the setpoint will be capped to USV_XY_VEL_MAX
 *
 * The maximum sideways and backward speed can be set differently
 * using USV_VEL_MAN_SIDE and USV_VEL_MAN_BACK, respectively.
 *
 * @unit m/s
 * @min 3.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_VEL_MANUAL, 10.0f);

/**
 * Maximum sideways velocity in Position mode
 *
 * If set to a negative value or larger than
 * USV_VEL_MANUAL then USV_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1.0
 * @max 20.0
 * @increment 0.1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_VEL_MAN_SIDE, -1.0f);

/**
 * Maximum backward velocity in Position mode
 *
 * If set to a negative value or larger than
 * USV_VEL_MANUAL then USV_VEL_MANUAL is used.
 *
 * @unit m/s
 * @min -1.0
 * @max 20.0
 * @increment 0.1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_VEL_MAN_BACK, -1.0f);

/**
 * Maximum horizontal velocity
 *
 * Maximum horizontal velocity in AUTO mode. If higher speeds
 * are commanded in a mission they will be capped to this velocity.
 *
 * @unit m/s
 * @min 0.0
 * @max 20.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_VEL_MAX, 12.0f);

/**
 * Maximum tilt angle in air
 *
 * Limits maximum tilt in AUTO and POSCTRL modes during flight.
 *
 * @unit deg
 * @min 20.0
 * @max 89.0
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_TILTMAX_AIR, 45.0f);


/**
 * Land crawl descend rate
 *
 * Used below USV_LAND_ALT3 if distance sensor data is availabe.
 *
 * @unit m/s
 * @min 0.1
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_LAND_CRWL, 0.3f);

/**
 * Enable user assisted descent for autonomous land routine
 *
 * When enabled, descent speed will be:
 * stick full up - 0
 * stick centered - USV_LAND_SPEED
 * stick full down - 2 * USV_LAND_SPEED
 *
 * Additionally, the vehicle can be yawed and moved laterally using the other sticks.
 * Manual override during auto modes has to be disabled to use this feature (see COM_RC_OVERRIDE).
 *
 * @min 0
 * @max 1
 * @value 0 Fixed descent speed of USV_LAND_SPEED
 * @value 1 User assisted descent speed
 * @group USV Omni Control
 */
PARAM_DEFINE_INT32(USV_LAND_RC_HELP, 0);

/**
 * Takeoff climb rate
 *
 * @unit m/s
 * @min 1
 * @max 5
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_TKO_SPEED, 1.5f);

/**
 * Maximal tilt angle in manual or altitude mode
 *
 * @unit deg
 * @min 0.0
 * @max 90.0
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_MAN_TILT_MAX, 35.0f);

/**
 * Max manual yaw rate
 *
 * @unit deg/s
 * @min 0.0
 * @max 400
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_MAN_Y_MAX, 150.0f);

/**
 * Manual yaw rate input filter time constant
 *
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0.0
 * @max 5.0
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_MAN_Y_TAU, 0.08f);

/**
 * Deadzone of sticks where position hold is enabled
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_HOLD_DZ, 0.1f);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * @unit m/s
 * @min 0.0
 * @max 3.0
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_HOLD_MAX_Z, 0.6f);

/**
 * Low pass filter cut freq. for numerical velocity derivative
 *
 * @unit Hz
 * @min 0.0
 * @max 10
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_VELD_LP, 5.0f);

/**
 * Maximum horizontal acceleration for auto mode and for manual mode
 *
 * USV_POS_MODE
 * 1 just deceleration
 * 3 acceleration and deceleration
 * 4 just acceleration
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_ACC_HOR_MAX, 5.0f);

/**
 * Acceleration for auto and for manual
 *
 * Note: In manual, this parameter is only used in USV_POS_MODE 4.
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */

PARAM_DEFINE_FLOAT(USV_ACC_HOR, 3.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes upward
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_ACC_UP_MAX, 4.0f);

/**
 * Maximum vertical acceleration in velocity controlled modes down
 *
 * @unit m/s^2
 * @min 2.0
 * @max 15.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_ACC_DOWN_MAX, 3.0f);

/**
 * Maximum jerk limit
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility (how fast it can change directions or break).
 *
 * Setting this to the maximum value essentially disables the limit.
 *
 * Note: This is only used when USV_POS_MODE is set to a smoothing mode 3 or 4.
 *
 * @unit m/s^3
 * @min 0.5
 * @max 500.0
 * @increment 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_JERK_MAX, 8.0f);

/**
 * Jerk limit in auto mode
 *
 * Limit the maximum jerk of the vehicle (how fast the acceleration can change).
 * A lower value leads to smoother vehicle motions, but it also limits its
 * agility.
 *
 * @unit m/s^3
 * @min 1.0
 * @max 80.0
 * @increment 1
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_JERK_AUTO, 4.0f);

/**
 * Altitude control mode.
 *
 * Set to 0 to control height relative to the earth frame origin. This origin may move up and down in
 * flight due to sensor drift.
 * Set to 1 to control height relative to estimated distance to ground. The vehicle will move up and down
 * with terrain height variation. Requires a distance to ground sensor. The height controller will
 * revert to using height above origin if the distance to ground estimate becomes invalid as indicated
 * by the local_position.distance_bottom_valid message being false.
 * Set to 2 to control height relative to ground (requires a distance sensor) when stationary and relative
 * to earth frame origin when moving horizontally.
 * The speed threshold is controlled by the USV_HOLD_MAX_XY parameter.
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group USV Omni Control
 */
PARAM_DEFINE_INT32(USV_ALT_MODE, 0);

/**
 * Manual position control stick exponential curve sensitivity
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_MAN_EXPO, 0.6f);

/**
 * Manual control stick vertical exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_Z_MAN_EXPO, 0.6f);

/**
 * Manual control stick yaw rotation exponential curve
 *
 * The higher the value the less sensitivity the stick has around zero
 * while still reaching the maximum value with full stick deflection.
 *
 * 0 Purely linear input curve (default)
 * 1 Purely cubic input curve
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_YAW_EXPO, 0.6f);

/**
 * Max yaw rate in auto mode
 *
 * Limit the rate of change of the yaw setpoint in autonomous mode
 * to avoid large control output and mixer saturation.
 *
 * @unit deg/s
 * @min 0.0
 * @max 360.0
 * @decimal 1
 * @increment 5
 * @group USV Attitude Control
 */
PARAM_DEFINE_FLOAT(USV_YAWRAUTOMAX, 45.0f);

/**
 * Altitude for 1. step of slow landing (descend)
 *
 * Below this altitude descending velocity gets limited to a value
 * between "USV_Z_VEL_MAX_DN" (or "USV_Z_V_AUTO_DN") and "USV_LAND_SPEED"
 * Value needs to be higher than "USV_LAND_ALT2"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_LAND_ALT1, 10.0f);

/**
 * Altitude for 2. step of slow landing (landing)
 *
 * Below this altitude descending velocity gets
 * limited to "USV_LAND_SPEED"
 * Value needs to be lower than "USV_LAND_ALT1"
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_LAND_ALT2, 5.0f);

/**
 * Altitude for 3. step of slow landing
 *
 * Below this altitude descending velocity gets
 * limited to "USV_LAND_CRWL", if LIDAR available.
 * No effect if LIDAR not available
 *
 * @unit m
 * @min 0
 * @max 122
 * @decimal 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_LAND_ALT3, 1.0f);

/**
 * Position control smooth takeoff ramp time constant
 *
 * Increasing this value will make automatic and manual takeoff slower.
 * If it's too slow the drone might scratch the ground and tip over.
 * A time constant of 0 disables the ramp
 *
 * @min 0
 * @max 5
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_TKO_RAMP_T, 3.0f);

/**
 * Manual-Position control sub-mode
 *
 * The supported sub-modes are:
 * 0 Simple position control where sticks map directly to velocity setpoints
 *   without smoothing. Useful for velocity control tuning.
 * 3 Smooth position control with maximum acceleration and jerk limits based on
 *   jerk optimized trajectory generator (different algorithm than 1).
 * 4 Smooth position control where sticks map to acceleration and there's a virtual brake drag
 *
 * @value 0 Simple position control
 * @value 3 Smooth position control (Jerk optimized)
 * @value 4 Acceleration based input
 * @group USV Omni Control
 */
PARAM_DEFINE_INT32(USV_POS_MODE, 4);

/**
 * Yaw mode.
 *
 * Specifies the heading in Auto.
 *
 * @min 0
 * @max 4
 * @value 0 towards waypoint
 * @value 1 towards home
 * @value 2 away from home
 * @value 3 along trajectory
 * @value 4 towards waypoint (yaw first)
 * @group Mission
 */
PARAM_DEFINE_INT32(USV_YAW_MODE, 0);

/**
 * Overall Horizontal Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * USV_XY_VEL_MAX or USV_VEL_MANUAL).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -20
 * @max 20
 * @decimal 1
 * @increment 1
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_XY_VEL_ALL, -10.0f);

/**
 * Overall Vertical Velocity Limit
 *
 * If set to a value greater than zero, other parameters are automatically set (such as
 * USV_Z_VEL_MAX_UP or USV_LAND_SPEED).
 * If set to a negative value, the existing individual parameters are used.
 *
 * @min -3
 * @max 8
 * @decimal 1
 * @increment 0.5
 * @group USV Omni Control
 */
PARAM_DEFINE_FLOAT(USV_Z_VEL_ALL, -3.0f);

// TODO: What's the diff??
/**
 * Stabilization mode(1) or Position Control(0)
 *
 * @value 0 Position Control
 * @value 1 Stabilization Mode
 * @group USV Omni Control
 */
PARAM_DEFINE_INT32(USV_STAB_MODE, 1);
