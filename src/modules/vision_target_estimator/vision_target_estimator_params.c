/****************************************************************************
 *
 *   Copyright (c) 2014-2018 PX4 Development Team. All rights reserved.
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
 * @file vision_target_estimator_params.c
 * Vision target Estimator algorithm parameters.
 *
 * @author Jonas Perolini <jonsper@me.com>
 *
 */

/**
 * Vision target Estimator module enable
 *
 * @boolean
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_EN, 1);

/**
 * Vision target Estimator module enable orientation estimation
 *
 * @boolean
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_YAW_EN, 0);

/**
 * Vision target Estimator module enable position estimation
 *
 * @boolean
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_POS_EN, 1);

/**
 * Use the target relative velocity to aid the EKF state estimation.
 *
 * @boolean
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_EKF_AID, 1);

/**
 * Integer bitmask controlling data fusion and aiding methods.
 *
 * Set bits in the following positions to enable:
 * 0 : Set to true to use the target's GPS position data if available. (+1)
 * 1 : Set to true to use the relative GPS velocity data if available. (If the target is moving, a target velocity estimate is required) (+2)
 * 2 : Set to true to use the target relative position from vision-based data if available (+4)
 * 3 : Set to true to use the mission landing point. Ignored if target GPS position enabled. (+8)
 *
 * @group Vision target Estimator
 * @min 0
 * @max 63
 * @bit 0 target GPS position
 * @bit 1 relative GPS velocity
 * @bit 2 vision relative position
 * @bit 3 mission landing position
 *
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_AID_MASK, 14);

/**
 * Target mode
 *
 * Configure the mode of the target. Depending on the mode, the state of the estimator (Kalman filter) varies. For static targets, the target observations can be used to aid position estimation.
 *
 * Mode Static: The target is static, the state of the Kalman filter is: [relative position, relative velocity, bias]. If the observations have a low variance,they can be used to aid position estimation.
 * Mode Moving: The target may be moving around, the state of the Kalman filter is: [relative position, drone velocity, bias, target's acceleration, target's velocity].
 *
 * @min 0
 * @max 2
 * @group Vision target Estimator
 * @value 0 Static
 * @value 1 Moving
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_MODE, 0);


/**
 * Vision Target Timeout
 *
 * Time after which the target is considered lost without any new measurements.
 *
 * @unit s
 * @min 0.0
 * @max 50
 * @decimal 1
 * @increment 0.5
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_BTOUT, 3.0f);

/**
 * Drone acceleration uncertainty
 *
 * Variance of drone's acceleration used for landing target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit (m/s^2)^2
 * @min 0.01
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_ACC_D_UNC, 1.0f);

/**
 * Target acceleration uncertainty
 *
 * Variance of target acceleration (in NED frame) used for target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit (m/s^2)^2
 * @min 0.01
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_ACC_T_UNC, 1.0f);

/**
 * Bias uncertainty
 *
 * Variance of GPS bias used for landing target position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit m^2
 * @min 0.01
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_BIAS_UNC, 0.05f);

/**
 * Bias limit
 *
 * Maximal bias between drone GPS and landing target GPS.
 *
 * @unit m^2
 * @min 0.01
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_BIAS_LIM, 1.f);


/**
 * Initial landing target and drone relative position uncertainty
 *
 * Initial variance of the relative landing target position in x,y,z direction
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_POS_UNC_IN, 0.5f);

/**
 * Initial landing target and drone relative velocity uncertainty
 *
 * Initial variance of the relative landing target velocity in x,y,z directions
 *
 * @unit (m/s)^2
 * @min 0.001
 * @decimal 3
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_VEL_UNC_IN, 0.5f);

/**
 * Initial GPS bias uncertainty
 *
 * Initial variance of the bias between the GPS on the target and the GPS on the drone
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_BIA_UNC_IN, 1.0f);

/**
 * Initial Orientation uncertainty
 *
 * Initial variance of the orientation (yaw) of the landing target
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_YAW_UNC_IN, 1.0f);

/**
 * Initial landing target absolute acceleration uncertainty
 *
 * Initial variance of the relative landing target acceleration in x,y,z directions
 *
 * @unit (m/s^2)^2
 * @min 0.001
 * @decimal 3
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_ACC_UNC_IN, 0.1f);

/**
 * Measurement noise for gps horizontal velocity.
 *
 * minimum allowed observation noise for gps velocity fusion (m/sec)
 *
 * @min 0.01
 * @max 5.0
 * @unit m/s
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_GPS_V_NOISE, 0.3f);

/**
 * Measurement noise for gps position.
 *
 * minimum allowed observation noise for gps position fusion (m)
 *
 * @min 0.01
 * @max 10.0
 * @unit m
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_GPS_P_NOISE, 0.5f);

/**
 * Whether to set the external vision observation noise from the parameter or from vision message
 *
 * If set to true the observation noise is set from the parameters directly, if set to false the measurement noise is taken from the vision message and the parameter are used as a lower bound.
 *
 * @boolean
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_INT32(VTE_EV_NOISE_MD, 0);

/**
 * Measurement noise for vision angle observations used to lower bound or replace the uncertainty included in the message
 *
 * @min 0.05
 * @unit rad
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_EVA_NOISE, 0.05f);

/**
 * Measurement noise for vision position observations used to lower bound or replace the uncertainty included in the message.
 *
 * If used to replace the uncertainty in the message, the measurement noise is lineraly scaled with the altitude i.e. unc = VTE_EVP_NOISE^2 * max(dist_bottom, 1)
 *
 * @min 0.01
 * @unit m
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_EVP_NOISE, 0.1f);


/**
 * Maximal time to estimate the future position of the target using the velocity estimation.
 *
 * When the target is moving, the position setpoint is set where the target will be after a given time in the future. (new position = velocity * time).
 * This param should be set depending on the expected target speed. The greater the speed, the greater VTE_MOVING_T_MAX.
 *
 * @min 0.1
 * @max 60
 * @unit s
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_MOVING_T_MAX, 3.f);


/**
 * Minimal time to estimate the future position of the target using the velocity estimation.
 *
 * When the target is moving, the position setpoint is set where the target will be after a given time in the future. (new position = velocity * time).
 * As the drone gets close to the target, the time of intersection between the target and the drone gets smaller. To avoid missing the target, a minimal time is required.
 * This param should be set depending on the expected target speed. The greater the speed, the greater VTE_MOVING_T_MIN.
 *
 * @min 0.1
 * @max 30
 * @unit s
 * @decimal 2
 *
 * @group Vision target Estimator
 */
PARAM_DEFINE_FLOAT(VTE_MOVING_T_MIN, 2.f);