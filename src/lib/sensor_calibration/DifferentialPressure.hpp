/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>

#include <inttypes.h>
#include <math.h>

namespace calibration
{
class DifferentialPressure
{
public:
	static constexpr int MAX_SENSOR_COUNT = 3; ///< matches MAX_NUM_AIRSPEED_SENSORS in the airspeed selector

	static constexpr const char *SensorString() { return "DPRES"; }

	DifferentialPressure();
	explicit DifferentialPressure(uint32_t device_id);

	~DifferentialPressure() = default;

	void PrintStatus();

	bool set_calibration_index(int calibration_index);
	void set_device_id(uint32_t device_id);
	bool set_offset(const float &offset);

	bool calibrated() const { return (_device_id != 0) && (_calibration_index >= 0); }
	uint8_t calibration_count() const { return _calibration_count; }
	int8_t calibration_index() const { return _calibration_index; }
	uint32_t device_id() const { return _device_id; }
	bool external() const { return _external; }
	const float &offset() const { return _offset; }

	/// remove the zero offset from a raw differential pressure reading [Pa]
	inline float Correct(const float &data) const { return data - _offset; }

	inline float Uncorrect(const float &corrected_data) const { return corrected_data + _offset; }

	bool ParametersLoad();
	bool ParametersSave(int desired_calibration_index = -1, bool force = false);
	void ParametersUpdate();

	/**
	 * Assign the imported SENS_DPRES_OFF calibration to the first sensor using slot 0.
	 * Does nothing if slot 0 is already claimed.
	 *
	 * @return true if the slot was adopted (the caller should save)
	 */
	bool AdoptUnclaimedCalibration();

	void Reset();

private:
	float _offset{0};

	int8_t _calibration_index{-1};
	uint32_t _device_id{0};
	uint8_t _calibration_count{0};

	bool _external{false};
};
} // namespace calibration
