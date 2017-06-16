/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file mpu9x50_params.c
 *
 * Parameters defined by the mpu9x50 driver
 */

#include <px4_config.h>
#include <systemlib/param/param.h>

/**
 * Low pass filter frequency for Gyro
 *
 * @value 0 MPU9X50_GYRO_LPF_250HZ
 * @value 1 MPU9X50_GYRO_LPF_184HZ
 * @value 2 MPU9X50_GYRO_LPF_92HZ
 * @value 3 MPU9X50_GYRO_LPF_41HZ
 * @value 4 MPU9X50_GYRO_LPF_20HZ
 * @value 5 MPU9X50_GYRO_LPF_10HZ
 * @value 6 MPU9X50_GYRO_LPF_5HZ
 * @value 7 MPU9X50_GYRO_LPF_3600HZ_NOLPF
 *
 * @group MPU9x50 Configuration
 */
PARAM_DEFINE_INT32(MPU_GYRO_LPF_ENM, 4);

/**
 * Low pass filter frequency for Accelerometer
 *
 *
 * @value 0 MPU9X50_ACC_LPF_460HZ
 * @value 1 MPU9X50_ACC_LPF_184HZ
 * @value 2 MPU9X50_ACC_LPF_92HZ
 * @value 3 MPU9X50_ACC_LPF_41HZ
 * @value 4 MPU9X50_ACC_LPF_20HZ
 * @value 5 MPU9X50_ACC_LPF_10HZ
 * @value 6 MPU9X50_ACC_LPF_5HZ
 * @value 7 MPU9X50_ACC_LPF_460HZ_NOLPF
 *
 * @group MPU9x50 Configuration
 */
PARAM_DEFINE_INT32(MPU_ACC_LPF_ENM, 4);

/**
 * Sample rate in Hz
 *
 * @value 0 MPU9x50_SAMPLE_RATE_100HZ
 * @value 1 MPU9x50_SAMPLE_RATE_200HZ
 * @value 2 MPU9x50_SAMPLE_RATE_500HZ
 * @value 3 MPU9x50_SAMPLE_RATE_1000HZ
 *
 * @group MPU9x50 Configuration
 */
PARAM_DEFINE_INT32(MPU_SAMPLE_R_ENM, 2);
