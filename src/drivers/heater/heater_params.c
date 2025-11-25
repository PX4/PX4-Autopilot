/****************************************************************************
 *
 *   Copyright (c) 2018-19 PX4 Development Team. All rights reserved.
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
 * @file heater_params.c
 * Heater parameters.
 *
 * @author Mark Sauder <mcsauder@gmail.com>
 * @author Alex Klimaj <alexklimaj@gmail.com>
 * @author Jake Dahl <dahl.jakejacob@gmail.com>
 * @author CaFeZn <1837781998@qq.com>
 */

/****************************************************************************
 *  多 IMU 独立加热参数组（PX4 v1.16+ 自定义扩展）
 *  改造后支持最多 2 颗 IMU 独立加热（BMI088 + ICM45686）
 *  每颗 IMU 使用独立的 device_id、目标温度、PID 参数和 PWM 通道
 ****************************************************************************/

/**
 * Target IMU device ID to regulate temperature.
 *
 * @category system
 * @group Sensors
 */
PARAM_DEFINE_INT32(IMU_1_ID, 2818058);        // BMI088 标准 device_id
PARAM_DEFINE_INT32(IMU_2_ID, 3014666);        // ICM45686 标准 device_id

/**
 * Target IMU temperature.
 *
 * @category system
 * @group Sensors
 * @unit celcius
 * @min 0
 * @max 85.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(IMU_1_TEMP, 48.0f);        // BMI088 推荐温度
PARAM_DEFINE_FLOAT(IMU_2_TEMP, 42.0f);        // ICM45686 推荐温度（不能太高）

/**
 * IMU heater controller feedforward value.
 *
 * @category system
 * @group Sensors
 * @unit %
 * @min 0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(IMU_1_TEMP_FF, 0.05f);
PARAM_DEFINE_FLOAT(IMU_2_TEMP_FF, 0.05f);

/**
 * IMU heater controller integrator gain value.
 *
 * @category system
 * @group Sensors
 * @unit us/C
 * @min 0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(IMU_1_TEMP_I, 0.025f);
PARAM_DEFINE_FLOAT(IMU_2_TEMP_I, 0.025f);

/**
 * IMU heater controller proportional gain value.
 *
 * @category system
 * @group Sensors
 * @unit us/C
 * @min 0
 * @max 2.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(IMU_1_TEMP_P, 1.0f);
PARAM_DEFINE_FLOAT(IMU_2_TEMP_P, 1.0f);
