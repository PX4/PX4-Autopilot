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

#include <lib/conversion/rotation.h>
#include <lib/matrix/matrix/math.hpp>

namespace sensor
{
namespace configuration
{

class Gyroscope
{
public:
	static constexpr int MAX_SENSOR_COUNT = 4;

	static constexpr uint8_t DEFAULT_PRIORITY = 50;
	static constexpr uint8_t DEFAULT_EXTERNAL_PRIORITY = 75;

	static constexpr const char *SensorString() { return "GYRO"; }

	Gyroscope();
	explicit Gyroscope(uint32_t device_id);

	~Gyroscope() = default;

	void PrintStatus();

	bool set_configuration_index(int configuration_index);
	void set_device_id(uint32_t device_id);

	void set_rotation(Rotation rotation);

	bool configured() const { return (_device_id != 0) && (_configuration_index >= 0); }
	uint8_t configuration_count() const { return _configuration_count; }
	int8_t configuration_index() const { return _configuration_index; }
	uint32_t device_id() const { return _device_id; }
	bool enabled() const { return (_priority > 0); }
	bool external() const { return _external; }

	const int32_t &priority() const { return _priority; }
	const matrix::Dcmf &rotation() const { return _rotation; }
	const Rotation &rotation_enum() const { return _rotation_enum; }

	bool ParametersLoad();
	bool ParametersSave(int desired_configuration_index = -1, bool force = false);
	void ParametersUpdate();

	void Reset();

private:
	Rotation _rotation_enum{ROTATION_NONE};

	matrix::Dcmf _rotation;

	uint32_t _device_id{0};
	int32_t _priority{-1};

	bool _external{false};

	int8_t _configuration_index{-1};
	uint8_t _configuration_count{0};
};

} // namespace configuration
} // namespace sensor
