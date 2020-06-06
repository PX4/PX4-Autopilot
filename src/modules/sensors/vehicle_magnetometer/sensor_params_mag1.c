/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * ID of Magnetometer the calibration is for.
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG1_ID, 0);

/**
 * Mag 1 enabled
 *
 * @boolean
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG1_EN, 1);

/**
 * Rotation of magnetometer 1 relative to airframe.
 *
 * An internal magnetometer will force a value of -1, so a GCS
 * should only attempt to configure the rotation if the value is
 * greater than or equal to zero.
 *
 * @value -1 Internal mag
 * @value 0 No rotation
 * @value 1 Yaw 45°
 * @value 2 Yaw 90°
 * @value 3 Yaw 135°
 * @value 4 Yaw 180°
 * @value 5 Yaw 225°
 * @value 6 Yaw 270°
 * @value 7 Yaw 315°
 * @value 8 Roll 180°
 * @value 9 Roll 180°, Yaw 45°
 * @value 10 Roll 180°, Yaw 90°
 * @value 11 Roll 180°, Yaw 135°
 * @value 12 Pitch 180°
 * @value 13 Roll 180°, Yaw 225°
 * @value 14 Roll 180°, Yaw 270°
 * @value 15 Roll 180°, Yaw 315°
 * @value 16 Roll 90°
 * @value 17 Roll 90°, Yaw 45°
 * @value 18 Roll 90°, Yaw 90°
 * @value 19 Roll 90°, Yaw 135°
 * @value 20 Roll 270°
 * @value 21 Roll 270°, Yaw 45°
 * @value 22 Roll 270°, Yaw 90°
 * @value 23 Roll 270°, Yaw 135°
 * @value 24 Pitch 90°
 * @value 25 Pitch 270°
 *
 * @min -1
 * @max 30
 * @reboot_required true
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG1_ROT, -1);

/**
 * Magnetometer X-axis offset
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XOFF, 0.0f);

/**
 * Magnetometer Y-axis offset
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YOFF, 0.0f);

/**
 * Magnetometer Z-axis offset
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZOFF, 0.0f);

/**
 * Magnetometer X-axis scaling factor
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_XSCALE, 1.0f);

/**
 * Magnetometer Y-axis scaling factor
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_YSCALE, 1.0f);

/**
 * Magnetometer Z-axis scaling factor
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_MAG1_ZSCALE, 1.0f);

/**
* Coefficient describing linear relationship between
* X component of magnetometer in body frame axis
* and either current or throttle depending on value of CAL_MAG_COMP_TYP
* Unit for throttle-based compensation is [G] and
* for current-based compensation [G/kA]
*
* @category system
* @group Sensor Calibration
*/
PARAM_DEFINE_FLOAT(CAL_MAG1_XCOMP, 0.0f);

/**
* Coefficient describing linear relationship between
* Y component of magnetometer in body frame axis
* and either current or throttle depending on value of CAL_MAG_COMP_TYP
* Unit for throttle-based compensation is [G] and
* for current-based compensation [G/kA]
*
* @category system
* @group Sensor Calibration
*/
PARAM_DEFINE_FLOAT(CAL_MAG1_YCOMP, 0.0f);

/**
* Coefficient describing linear relationship between
* Z component of magnetometer in body frame axis
* and either current or throttle depending on value of CAL_MAG_COMP_TYP
* Unit for throttle-based compensation is [G] and
* for current-based compensation [G/kA]
*
* @category system
* @group Sensor Calibration
*/
PARAM_DEFINE_FLOAT(CAL_MAG1_ZCOMP, 0.0f);
