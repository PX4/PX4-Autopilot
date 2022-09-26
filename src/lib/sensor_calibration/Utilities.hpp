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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <lib/conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <matrix/math.hpp>

namespace calibration
{

/**
 * @brief Find sensor's calibration index if it exists.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param device_id
 * @return int8_t Valid calibration index on success, -1 otherwise
 */
int8_t FindCurrentCalibrationIndex(const char *sensor_type, uint32_t device_id);

/**
 * @brief Find sensor's calibration index if it exists, otherwise select an available slot.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param device_id
 * @param preferred_index preferred index (optional)
 * @return int8_t Valid calibration index on success, -1 otherwise
 */
int8_t FindAvailableCalibrationIndex(const char *sensor_type, uint32_t device_id, int8_t preferred_index = -1);

/**
 * @brief Get sensor calibration parameter value.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance
 * @return int32_t The calibration value.
 */
int32_t GetCalibrationParamInt32(const char *sensor_type, const char *cal_type, uint8_t instance);
float GetCalibrationParamFloat(const char *sensor_type, const char *cal_type, uint8_t instance);

/**
 * @brief Set a single calibration paramter.
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Calibration index (0 - 3)
 * @param value int32_t parameter value
 * @return true if the parameter name was valid and value saved successfully, false otherwise.
 */
template<typename T>
bool SetCalibrationParam(const char *sensor_type, const char *cal_type, uint8_t instance, T value)
{
	char str[20] {};

	// eg CAL_MAGn_ID/CAL_MAGn_ROT
	sprintf(str, "CAL_%s%u_%s", sensor_type, instance, cal_type);

	int ret = param_set_no_notification(param_find(str), &value);

	if (ret != PX4_OK) {
		PX4_ERR("failed to set %s", str);
	}

	return ret == PX4_OK;
}

/**
 * @brief Get the Calibration Params Vector 3f object
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Calibration index (0 - 3)
 * @return matrix::Vector3f Vector of calibration values.
 */
matrix::Vector3f GetCalibrationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance);

/**
 * @brief Set the Calibration Params Vector 3f object
 *
 * @param sensor_type Calibration parameter abbreviated sensor string ("ACC", "GYRO", "MAG")
 * @param cal_type Calibration parameter abbreviated type ("OFF", "SCALE", "ROT", "PRIO")
 * @param instance Calibration index (0 - 3)
 * @param values Vector of calibration values x, y, z.
 * @return true if the parameter name was valid and all values saved successfully, false otherwise.
 */
bool SetCalibrationParamsVector3f(const char *sensor_type, const char *cal_type, uint8_t instance,
				  matrix::Vector3f values);

/**
 * @brief Get the board sensor level adjustment (SENS_BOARD_X_OFF, SENS_BOARD_Y_OFF, SENS_BOARD_Z_OFF).
 *
 * @return matrix::Eulerf
 */
matrix::Eulerf GetSensorLevelAdjustment();

/**
 * @brief Get the board rotation.
 *
 * @return enum Rotation
 */
Rotation GetBoardRotation();

/**
 * @brief Get the board rotation Dcm.
 *
 * @return matrix::Dcmf
 */
matrix::Dcmf GetBoardRotationMatrix();

/**
 * @brief Determine if device is on an external bus
 *
 * @return true if device is on an external bus
 */
bool DeviceExternal(uint32_t device_id);


struct InFlightCalibration {
	uint32_t device_id{0};
	matrix::Vector3f offset{};
	matrix::Vector3f variance{};
	uint8_t estimator_instance{0};
};

template<typename T, int MAX_LEARNED = 3>
bool SensorCalibrationApplyLearned(T &calibration, int sensor_index,
				   InFlightCalibration(&learned_calibrations)[MAX_LEARNED])
{
	bool updated = false;

	// State variance assumed for bias storage.
	// This is a reference variance used to calculate the fraction of learned bias that will be used to update the stored value.
	// Larger values cause a larger fraction of the learned biases to be used.
	static constexpr float BIAS_VAR_REF = 0.000'001f;
	matrix::Vector3f state_variance{BIAS_VAR_REF, BIAS_VAR_REF, BIAS_VAR_REF};

	// apply all valid saved offsets
	// iterate through available bias estimates and fuse them sequentially using a Kalman Filter scheme
	for (auto &inflight_cal : learned_calibrations) {
		if ((inflight_cal.device_id != 0) && (inflight_cal.device_id == calibration.device_id()) && calibration.enabled()) {

			const matrix::Vector3f offset_orig{calibration.offset()};

			// new offset
			matrix::Vector3f offset_new{};

			if (calibration.calibrated()) {
				// calculate weighting using ratio of variances and update stored bias values
				const matrix::Vector3f &obs{inflight_cal.offset};            // observation
				const matrix::Vector3f &obs_variance{inflight_cal.variance}; // observation variance

				for (int axis_index = 0; axis_index < 3; axis_index++) {
					float innovation = offset_orig(axis_index) - obs(axis_index);
					float innovation_variance = state_variance(axis_index) + obs_variance(axis_index);

					float kalman_gain = state_variance(axis_index) / innovation_variance;

					offset_new(axis_index) = offset_orig(axis_index) - (innovation * kalman_gain);

					state_variance(axis_index) = fmaxf(state_variance(axis_index) * (1.f - kalman_gain), 0.f);
				}

			} else {
				// sensor wasn't calibrated, use full offset
				offset_new = inflight_cal.offset;
			}

			if (calibration.set_offset(offset_new)) {
				PX4_INFO("%s %d (%" PRIu32 ") EST:%d offset: [%.2f, %.2f, %.2f]->[%.2f, %.2f, %.2f] (full [%.3f, %.3f, %.3f])",
					 calibration.SensorString(), sensor_index, calibration.device_id(), inflight_cal.estimator_instance,
					 (double)offset_orig(0), (double)offset_orig(1), (double)offset_orig(2),
					 (double)offset_new(0), (double)offset_new(1), (double)offset_new(2),
					 (double)inflight_cal.offset(0), (double)inflight_cal.offset(1), (double)inflight_cal.offset(2));

				updated = true;
			}

			// clear
			inflight_cal = {};
		}
	}

	if (updated) {
		calibration.ParametersSave(sensor_index);
	}

	return updated;
}

} // namespace calibration
