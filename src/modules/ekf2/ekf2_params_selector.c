/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * Selector error reduce threshold
 *
 * EKF2 instances have to be better than the selected by at least this amount before their relative score can be reduced.
 *
 * @group EKF2
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_ERR_RED, 0.2f);

/**
 * Selector angular rate threshold
 *
 * EKF2 selector angular rate error threshold for comparing gyros. Angular rate vector differences larger than this will result in accumulated angular error.
 *
 * @group EKF2
 * @unit deg/s
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_RAT, 7.0f);

/**
 * Selector angular threshold.
 *
 * EKF2 selector maximum accumulated angular error threshold for comparing gyros. Accumulated angular error larger than this will result in the sensor being declared faulty.
 *
 * @group EKF2
 * @unit deg
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_ANG, 15.0f);

/**
 * Selector acceleration threshold
 *
 * EKF2 selector acceleration error threshold for comparing accelerometers. Acceleration vector differences larger than this will result in accumulated velocity error.
 *
 * @group EKF2
 * @unit m/s^2
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_ACC, 1.0f);

/**
 * Selector angular threshold.
 *
 * EKF2 selector maximum accumulated velocity threshold for comparing accelerometers. Accumulated velocity error larger than this will result in the sensor being declared faulty.
 *
 * @group EKF2
 * @unit m/s
 */
PARAM_DEFINE_FLOAT(EKF2_SEL_IMU_VEL, 2.0f);

/**
 * Manual instance selection channel
 *
 * Defines which RC_MAP_AUXn parameter maps the RC channel used to select a desired
 * EKF2 instance.
 * Use EKF2_SEL_RC_INST to define which instance is selected when this switch is active.
 * When manual selection is active and the switch is off, instance 0 is selected.
 * Manual selection overrides automatic instance switch.
 *
 * @value 0 Disable
 * @value 1 Aux1
 * @value 2 Aux2
 * @value 3 Aux3
 * @value 4 Aux4
 * @value 5 Aux5
 * @value 6 Aux6
 * @min 0
 * @max 6
 * @group EKF2
 */
PARAM_DEFINE_INT32(EKF2_SEL_RC_MAP, 0);

/**
 * Manual instance selection
 *
 * Defines which estimator instance is used when manual switch is active.
 * Use EKF2_SEL_RC_MAP to define which aux switch is used to trigger the switch.
 * When manual selection is active and the switch is off, instance 0 is selected.
 * Manual selection overrides automatic instance switch.
 *
 * @min 0
 * @max 10
 * @group EKF2
 */
PARAM_DEFINE_INT32(EKF2_SEL_RC_INST, 0);
