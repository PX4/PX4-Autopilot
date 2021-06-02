/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Driver ID of the angle of attack vane (corresponds to sensor I2C address)
 *
 * setting -1 disables the vane, 0-3 correspond to hall driver IDs
 *
 * @min -1
 * @max 3
 * @reboot_required true
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_AOA_ID, -1);

/**
 * Angle of attack vane angular mounting offset in degrees
 *
 * This value is subtracted (as a bias) from the measured angle of attack
 *
 * @unit deg
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_AOA_OFF, 0.0);

/**
 * Minimum calibrated angle of attack vane angle in degrees
 *
 * @unit deg
 * @min -90.0
 * @max -1.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_AOA_MIN, -30.0);

/**
 * Maximum calibrated angle of attack vane angle in degrees
 *
 * @unit deg
 * @min 1.0
 * @max 90.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_AOA_MAX, 30.0);

/**
 * Reverse vane direction (mounting dependent)
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_AOA_REV, 1.0);

/**
 * Calibrated polynomial coefficient 0 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = vane angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_AOA_P0, 0);

/**
 * Calibrated polynomial coefficient 1 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_AOA_P1, 0);

/**
 * Calibrated polynomial coefficient 2 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_AOA_P2, 0);

/**
 * Calibrated polynomial coefficient 3 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_AOA_P3, 0);

/**
 * Driver ID of the sideslip vane (corresponds to sensor I2C address)
 *
 * setting -1 disables the vane, 0-3 correspond to hall driver IDs
 *
 * @min -1
 * @max 3
 * @reboot_required true
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_SLIP_ID, -1);

/**
 * Sideslip vane angular mounting offset in degrees
 *
 * This value is subtracted (as a bias) from the measured sideslip
 *
 * @unit deg
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_SLIP_OFF, 0.0);

/**
 * Minimum calibrated sideslip vane angle in degrees
 *
 * @unit deg
 * @min -90.0
 * @max -1.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_SLIP_MIN, -30.0);

/**
 * Maximum calibrated sideslip vane angle in degrees
 *
 * @unit deg
 * @min 1.0
 * @max 90.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_SLIP_MAX, 30.0);

/**
 * Reverse vane direction (mounting dependent)
 *
 * @min -1.0
 * @max 1.0
 * @value -1.0 Reverse
 * @value 1.0 Normal
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_AV_SLIP_REV, 1.0);

/**
 * Calibrated polynomial coefficient 0 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = vane angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_SLIP_P0, 0);

/**
 * Calibrated polynomial coefficient 1 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_SLIP_P1, 0);

/**
 * Calibrated polynomial coefficient 2 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_SLIP_P2, 0);

/**
 * Calibrated polynomial coefficient 3 (*1e7)
 *
 * y = p0 + p1 * x + p2 * x^2 + ...
 * y = angle in degrees
 * x = magnetic field measurement in Tesla (assumes use of hall effect sensor)
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_AV_SLIP_P3, 0);
