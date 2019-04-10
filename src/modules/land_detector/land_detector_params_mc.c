/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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
 * Multicopter max climb rate
 *
 * Maximum vertical velocity allowed in the landed state (m/s up and down)
 *
 * @unit m/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_Z_VEL_MAX, 0.50f);

/**
 * Multicopter max horizontal velocity
 *
 * Maximum horizontal velocity allowed in the landed state (m/s)
 *
 * @unit m/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_XY_VEL_MAX, 1.5f);

/**
 * Multicopter max rotation
 *
 * Maximum allowed angular velocity around each axis allowed in the landed state.
 *
 * @unit deg/s
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_ROT_MAX, 20.0f);

/**
 * Multicopter specific force threshold
 *
 * Multicopter threshold on the specific force measured by accelerometers in m/s^2 for free-fall detection
 *
 * @unit m/s^2
 * @min 0.1
 * @max 10
 * @decimal 2
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_FFALL_THR, 2.0f);

/**
 * Multicopter free-fall trigger time
 *
 * Seconds (decimal) that freefall conditions have to met before triggering a freefall.
 * Minimal value is limited by LAND_DETECTOR_UPDATE_RATE=50Hz in landDetector.h
 *
 * @unit s
 * @min 0.02
 * @max 5
 * @decimal 2
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDMC_FFALL_TTRI, 0.3);

/**
 * Maximum altitude for multicopters
 *
 * The system will obey this limit as a
 * hard altitude limit. This setting will
 * be consolidated with the GF_MAX_VER_DIST
 * parameter.
 * A negative value indicates no altitude limitation.
 *
 * @unit m
 * @min -1
 * @max 10000
 * @decimal 2
 * @group Land Detector
 *
 */
PARAM_DEFINE_FLOAT(LNDMC_ALT_MAX, -1.0f);

/**
 * Low throttle detection threshold.
 *
 * Defines the commanded throttle value below which the land detector
 * considers the vehicle to have "low thrust". This is one condition that
 * is used to detect the ground contact state. The value is calculated as
 * val = (MPC_THR_HOVER - MPC_THR_MIN) * LNDMC_LOW_T_THR + MPC_THR_MIN
 * Increase this value if the system takes long time to detect landing.
 *
 * @unit norm
 * @min 0.1
 * @max 0.9
 * @decimal 2
 * @group Land Detector
 *
 */
PARAM_DEFINE_FLOAT(LNDMC_LOW_T_THR, 0.3);
