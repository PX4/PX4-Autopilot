/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * Coefficient describing linear relationship between normalized throttle and
 * X component of magnetometer in body frame axis.
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_DMAGX_D_THR, 0);

/**
 * Coefficient describing linear relationship between normalized throttle and
 * Y component of magnetometer in body frame axis.
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_DMAGY_D_THR, 0);

/**
 * Coefficient describing linear relationship between normalized throttle and
 * Z component of magnetometer in body frame axis.
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(CAL_DMAGZ_D_THR, 0);

/**
 * Magnetometer ID of the sensor for which we want to apply the compensation.
 *
 * @category system
 * @group Sensor Calibration
 */
PARAM_DEFINE_INT32(CAL_MAG_COMP_ID, 0);