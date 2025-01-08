/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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
 * Fixed-wing land detector: Max horizontal velocity threshold
 *
 * Maximum horizontal velocity allowed in the landed state.
 * A factor of 0.7 is applied in case of airspeed-less flying
 * (either because no sensor is present or sensor data got invalid in flight).
 *
 * @unit m/s
 * @min 0.5
 * @max 10
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_VEL_XY_MAX, 5.0f);

/**
 * Fixed-wing land detector: Max vertiacal velocity threshold
 *
 * Maximum vertical velocity allowed in the landed state.
 *
 * @unit m/s
 * @min 0.1
 * @max 20
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_VEL_Z_MAX, 1.0f);

/**
 * Fixed-wing land detector: Max horizontal acceleration
 *
 * Maximum horizontal (x,y body axes) acceleration allowed in the landed state
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_XYACC_MAX, 8.0f);

/**
 * Fixed-wing land detector: Max airspeed
 *
 * Maximum airspeed allowed in the landed state
 *
 * @unit m/s
 * @min 2
 * @max 20
 * @decimal 1
 *
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_AIRSPD_MAX, 6.00f);

/**
 * Fixed-wing land detection trigger time
 *
 * Time the land conditions (speeds and acceleration) have to be satisfied to detect a landing.
 *
 * @unit s
 * @min 0.1
 * @decimal 1
 *
 * @reboot_required true
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_TRIG_TIME, 2.f);

/**
 * Fixed-wing land detector: max rotational speed
 *
 * Maximum allowed norm of the angular velocity in the landed state.
 *
 * @unit deg/s
 * @decimal 1
 * @group Land Detector
 */
PARAM_DEFINE_FLOAT(LNDFW_ROT_MAX, 0.5f);
