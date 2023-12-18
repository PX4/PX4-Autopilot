/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include <lib/matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_correction.h>

#include <lib/sensor/configuration/Magnetometer.hpp>

namespace sensor
{
namespace calibration
{
class Magnetometer
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr const char *SensorString() { return "MAG"; }

	Magnetometer();
	explicit Magnetometer(uint32_t device_id);

	~Magnetometer() = default;

	void PrintStatus();

	bool set_calibration_index(int calibration_index);
	void set_device_id(uint32_t device_id);

	bool set_offset(const matrix::Vector3f &offset);
	bool set_scale(const matrix::Vector3f &scale);
	bool set_offdiagonal(const matrix::Vector3f &offdiagonal);

	void set_rotation(Rotation rotation) { _configuration.set_rotation(rotation); }

	bool calibrated() const { return (_configuration.device_id() != 0) && (_calibration_index >= 0); }
	uint8_t calibration_count() const { return _calibration_count; }
	int8_t calibration_index() const { return _calibration_index; }

	const matrix::Vector3f &offset() const { return _offset; }
	const matrix::Matrix3f &scale() const { return _scale; }

	uint32_t device_id() const { return _configuration.device_id(); }
	bool enabled() const { return _configuration.enabled(); }
	bool external() const { return _configuration.external(); }

	const int32_t &priority() const { return _configuration.priority(); }
	const matrix::Dcmf &rotation() const { return _configuration.rotation(); }
	const Rotation &rotation_enum() const { return _configuration.rotation_enum(); }

	// apply offsets and scale
	// rotate corrected measurements from sensor to body frame
	inline matrix::Vector3f Correct(const matrix::Vector3f &data) const
	{
		return _configuration.rotation() * (_scale * ((data + _power * _power_compensation) - _offset));
	}

	// Compute sensor offset from bias (board frame)
	matrix::Vector3f BiasCorrectedSensorOffset(const matrix::Vector3f &bias) const
	{
		// updated calibration offset = existing offset + bias rotated to sensor frame and unscaled
		return _offset + (_scale.I() * _configuration.rotation().I() * bias);
	}

	bool ParametersLoad();
	bool ParametersSave(int desired_calibration_index = -1, bool force = false);
	void ParametersUpdate();

	void Reset();

	void SensorCorrectionsUpdate(bool force = false);

	void UpdatePower(float power) { _power = power; }

private:
	sensor::configuration::Magnetometer _configuration{};

	uORB::Subscription _sensor_correction_sub{ORB_ID(sensor_correction)};

	matrix::Vector3f _offset;
	matrix::Matrix3f _scale;
	matrix::Vector3f _thermal_offset;
	matrix::Vector3f _power_compensation;

	float _power{0.f};

	int8_t _calibration_index{-1};
	uint8_t _calibration_count{0};
};

} // namespace calibration
} // namespace sensor
