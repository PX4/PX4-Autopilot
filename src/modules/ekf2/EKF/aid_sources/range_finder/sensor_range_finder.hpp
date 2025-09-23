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

/**
 * @file sensor_range_finder.hpp
 * Range finder class containing all the required checks
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */
#ifndef EKF_SENSOR_RANGE_FINDER_HPP
#define EKF_SENSOR_RANGE_FINDER_HPP

#include <matrix/math.hpp>
#include <lib/mathlib/math/filter/MedianFilter.hpp>

namespace estimator
{
namespace sensor
{

struct rangeSample {
	uint64_t    time_us{};  // timestamp of the measurement (uSec)
	float       range{};    // range (distance to ground) measurement (m)
	int8_t      quality{};  // Signal quality in percent (0...100%), where 0 = invalid signal, 100 = perfect signal, and -1 = unknown signal quality.
};

class SensorRangeFinder
{
public:
	SensorRangeFinder() = default;
	~SensorRangeFinder() = default;

	struct Parameters {
		float ekf2_imu_pos_x{};
		float ekf2_imu_pos_y{};
		float ekf2_imu_pos_z{};

		float ekf2_rng_pos_x{};
		float ekf2_rng_pos_y{};
		float ekf2_rng_pos_z{};

		float ekf2_rng_pitch{};

		float range_cos_max_tilt{0.7071f}; // 45 degrees max tilt
	};

	// This is required because of the ring buffer
	// TODO: move the ring buffer here
	rangeSample *sample() { return &_sample; }

	void setSample(const rangeSample &sample);
	void setPitchOffset(float new_pitch_offset);
	void setCosMaxTilt(float cos_max_tilt) { _range_cos_max_tilt = cos_max_tilt; }
	void setLimits(float min_distance, float max_distance);

	float getCosTilt() const { return _cos_tilt_rng_to_earth; }
	float getDistBottom() const { return _sample.range * _cos_tilt_rng_to_earth; }
	float getValidMinVal() const { return _min_distance; }
	float getValidMaxVal() const { return _max_distance; }

	void updateParameters(Parameters &parameters) { _parameters = parameters; };
	void updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth);

	bool isTiltOk() const { return _cos_tilt_rng_to_earth > _range_cos_max_tilt; }
	bool timedOut(uint64_t time_now) const;

private:

	rangeSample _sample{};

	Parameters _parameters{};

	// Tilt check
	float _cos_tilt_rng_to_earth{1.f};  // 2,2 element of the rotation matrix from sensor frame to earth frame
	float _range_cos_max_tilt{0.7071f}; // cosine of the maximum tilt angle from the vertical that permits use of range finder and flow data
	float _pitch_offset_rad{3.14f};     // range finder tilt rotation about the Y body axis
	float _sin_pitch_offset{0.0f};      // sine of the range finder tilt rotation about the Y body axis
	float _cos_pitch_offset{-1.0f};     // cosine of the range finder tilt rotation about the Y body axis

	// Range check
	float _min_distance{}; // minimum distance that the rangefinder can measure (m)
	float _max_distance{}; // maximum distance that the rangefinder can measure (m)
};

} // namespace sensor
} // namespace estimator
#endif // !EKF_SENSOR_RANGE_FINDER_HPP
