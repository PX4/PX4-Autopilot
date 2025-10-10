/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * Enable simulated GPS sensor
 *
 * @group Simulation
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_GPSSIM, 1);

/**
 * Enable simulated barometer sensor
 *
 * @group Simulation
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_BAROSIM, 0);

/**
 * Enable simulated magnetometer sensor
 *
 * @group Simulation
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_MAGSIM, 0);

/**
 * Enable simulated airspeed sensor
 *
 * @group Simulation
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_ARSPDSIM, 0);

/**
 * Enable simulated AGP sensor
 *
 * @group Simulation
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_AGPSIM, 0);

/**
 * Enable simulated distance sensor
 *
 * @group Simulation
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_DISTSIM, 0);

/**
 * Number of GPS satellites used in simulation
 *
 * @group Simulator
 * @min 0
 * @max 50
 */
PARAM_DEFINE_INT32(SIM_GPS_USED, 10);

/**
 * simulated barometer pressure offset
 *
 * @group Simulator
 */
PARAM_DEFINE_FLOAT(SIM_BARO_OFF_P, 0.0f);

/**
 * simulated barometer temperature offset
 *
 * @group Simulator
 * @unit celcius
 */
PARAM_DEFINE_FLOAT(SIM_BARO_OFF_T, 0.0f);

/**
 * simulated magnetometer X offset
 *
 * @group Simulator
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(SIM_MAG_OFFSET_X, 0.0f);

/**
 * simulated magnetometer Y offset
 *
 * @group Simulator
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(SIM_MAG_OFFSET_Y, 0.0f);

/**
 * simulated magnetometer Z offset
 *
 * @group Simulator
 * @unit gauss
 */
PARAM_DEFINE_FLOAT(SIM_MAG_OFFSET_Z, 0.0f);

/**
 * AGP failure mode
 *
 * Stuck: freeze the measurement to the current location
 * Drift: add a linearly growing bias to the sensor data
 *
 * @group Simulator
 * @min 0
 * @max 3
 * @bit 0 Stuck
 * @bit 1 Drift
 */
PARAM_DEFINE_INT32(SIM_AGP_FAIL, 0);

/**
 * distance sensor minimum range
 *
 * @unit m
 * @min 0.0
 * @max 10.0
 * @decimal 4
 * @increment 0.01
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_DISTSNSR_MIN, 0.0f);

/**
 * distance sensor maximum range
 *
 * @unit m
 * @min 0.0
 * @max 1000.0
 * @decimal 4
 * @increment 0.01
 * @group Simulation In Hardware
 */
PARAM_DEFINE_FLOAT(SIH_DISTSNSR_MAX, 100.0f);
