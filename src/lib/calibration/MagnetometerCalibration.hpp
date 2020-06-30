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
#include <lib/matrix/matrix/math.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>

namespace sensors
{

class MagnetometerCalibration
{
public:
	MagnetometerCalibration() = default;
	~MagnetometerCalibration() = default;

	void PrintStatus();

	void set_device_id(uint32_t device_id);
	void set_external(bool external = true) { _external = external; }

	uint32_t device_id() const { return _device_id; }
	int32_t priority() const { return _priority; }
	bool enabled() const { return (_priority > 0); }
	bool external() const { return _external; }

	// apply offsets and scale
	// rotate corrected measurements from sensor to body frame
	matrix::Vector3f Correct(const matrix::Vector3f &data);

	void ParametersUpdate();
	void SensorCorrectionsUpdate(bool force = false);

	void UpdatePower(float power) { _power = power; }

private:

	static constexpr int MAX_SENSOR_COUNT = 4;

	int FindCalibrationIndex(uint32_t device_id) const;

	static constexpr const char *SensorString() { return "MAG"; }

	static constexpr uint8_t MAG_DEFAULT_PRIORITY = 50;
	static constexpr uint8_t MAG_DEFAULT_EXTERNAL_PRIORITY = 75;

	matrix::Dcmf _rotation;

	matrix::Vector3f _offset;
	matrix::Matrix3f _scale;
	matrix::Vector3f _power_compensation;
	float _power{0.f};

	uint32_t _device_id{0};

	int32_t _priority{MAG_DEFAULT_PRIORITY};
	bool _external{false};
};

} // namespace sensors
