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

#include "RingBuffer.hpp"

#include <Integrator.hpp>

#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/sensor_optical_flow.h>

namespace sensors
{

/**
 * Accumulates optical flow frames into windows and pairs each frame with the
 * gyro integral over exactly its own window.
 *
 * Holds no parameters: the flight controller and a CAN flow node use it with
 * their own configuration.
 */
class OpticalFlowAccumulator
{
public:
	OpticalFlowAccumulator();

	struct Window {
		matrix::Vector2f pixel_flow{};
		matrix::Vector3f delta_angle{};
		uint32_t integration_timespan_us{0};
		uint8_t quality{0};
		uint8_t frame_count{0}; // tracked frames; 0 means every frame in the window was rejected
	};

	void addGyroSample(uint64_t timestamp_sample, const matrix::Vector3f &angular_velocity, float dt);

	// a sensor that supplies its own delta angle needs no gyro samples
	bool gyroRequired() const { return !_delta_angle_from_sensor; }

	// rotation applied to a delta angle supplied by the sensor, which arrives in the sensor frame
	void setSensorRotation(const matrix::Dcmf &rotation) { _sensor_rotation = rotation; }

	/**
	 * @return true if the accumulation was discarded before adding this frame
	 */
	bool addFrame(const sensor_optical_flow_s &flow);

	// tracked and rejected time, which together set the publication cadence
	uint32_t accumulatedTimespanUs() const { return _integration_timespan_us + _rejected_timespan_us; }

	Window window() const;

	void reset();

private:
	struct GyroSample {
		uint64_t time_us{};
		matrix::Vector3f data{};
		float dt{0.f};
	};

	// A polled zero-motion flow sample ends one frame period before the poll and spans up to the
	// backup interval, so at 1 kHz the integration window can start ~47 ms before the sample arrives.
	RingBuffer<GyroSample, 64> _gyro_buffer{};

	IntegratorConing _gyro_integrator{};

	matrix::Dcmf _sensor_rotation{matrix::eye<float, 3>()};

	uint64_t _timestamp_sample_last{0};
	matrix::Vector2f _flow_integral{};
	matrix::Vector3f _delta_angle{};
	uint32_t _integration_timespan_us{0};
	uint32_t _rejected_timespan_us{0};
	uint16_t _quality_sum{0};
	uint8_t _frame_count{0};

	bool _delta_angle_from_sensor{false};
};

} // namespace sensors
