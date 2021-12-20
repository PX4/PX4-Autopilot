/****************************************************************************
 *
 *   Copyright (c) 2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

#include "Sensor.hpp"
#include <matrix/math.hpp>

namespace estimator
{
namespace sensor
{

class SensorRangeFinder : public Sensor
{
public:
	SensorRangeFinder() = default;
	~SensorRangeFinder() override = default;

	void runChecks(uint64_t current_time_us, const matrix::Dcmf &R_to_earth);
	bool isHealthy() const override { return _is_sample_valid; }
	bool isDataHealthy() const override { return _is_sample_ready && _is_sample_valid; }
	bool isDataReady() const { return _is_sample_ready; }
	bool isRegularlySendingData() const override { return _is_regularly_sending_data; }

	void setSample(const rangeSample &sample)
	{
		_sample = sample;
		_is_sample_ready = true;
	}

	// This is required because of the ring buffer
	// TODO: move the ring buffer here
	rangeSample *getSampleAddress() { return &_sample; }

	void setPitchOffset(float new_pitch_offset)
	{
		if (fabsf(_pitch_offset_rad - new_pitch_offset) > FLT_EPSILON) {
			_sin_pitch_offset = sinf(new_pitch_offset);
			_cos_pitch_offset = cosf(new_pitch_offset);
			_pitch_offset_rad = new_pitch_offset;
		}
	}

	void setCosMaxTilt(float cos_max_tilt) { _range_cos_max_tilt = cos_max_tilt; }

	void setLimits(float min_distance, float max_distance)
	{
		_rng_valid_min_val = min_distance;
		_rng_valid_max_val = max_distance;
	}

	void setQualityHysteresis(float valid_quality_threshold_s)
	{
		_quality_hyst_us = uint64_t(valid_quality_threshold_s * 1e6f);
	}

	float getCosTilt() const { return _cos_tilt_rng_to_earth; }

	void setRange(float rng) { _sample.rng = rng; }
	float getRange() const { return _sample.rng; }

	float getDistBottom() const { return _sample.rng * _cos_tilt_rng_to_earth; }

	void setDataReadiness(bool is_ready) { _is_sample_ready = is_ready; }
	void setValidity(bool is_valid) { _is_sample_valid = is_valid; }

	float getValidMinVal() const { return _rng_valid_min_val; }
	float getValidMaxVal() const { return _rng_valid_max_val; }

private:
	void updateSensorToEarthRotation(const matrix::Dcmf &R_to_earth);

	void updateValidity(uint64_t current_time_us);
	void updateDtDataLpf(uint64_t current_time_us);
	bool isSampleOutOfDate(uint64_t current_time_us) const;
	bool isDataContinuous() const { return _dt_data_lpf < 2e6f; }
	bool isTiltOk() const { return _cos_tilt_rng_to_earth > _range_cos_max_tilt; }
	bool isDataInRange() const;
	void updateStuckCheck();

	rangeSample _sample{};

	bool _is_sample_ready{};	///< true when new range finder data has fallen behind the fusion time horizon and is available to be fused
	bool _is_sample_valid{};	///< true if range finder sample retrieved from buffer is valid
	bool _is_regularly_sending_data{false}; ///< true if the interval between two samples is less than the maximum expected interval
	uint64_t _time_last_valid_us{};	///< time the last range finder measurement was ready (uSec)

	/*
	 * Stuck check
	 */
	bool _is_stuck{};
	float _stuck_threshold{0.1f};	///< minimum variation in range finder reading required to declare a range finder 'unstuck' when readings recommence after being out of range (m)
	float _stuck_min_val{};		///< minimum value for new rng measurement when being stuck
	float _stuck_max_val{};		///< maximum value for new rng measurement when being stuck

	/*
	 * Data regularity check
	 */
	static constexpr float _dt_update{0.01f}; 	///< delta time since last ekf update TODO: this should be a parameter
	float _dt_data_lpf{};	///< filtered value of the delta time elapsed since the last range measurement came into the filter (uSec)

	/*
	 * Tilt check
	 */
	float _cos_tilt_rng_to_earth{};		///< 2,2 element of the rotation matrix from sensor frame to earth frame
	float _range_cos_max_tilt{0.7071f};	///< cosine of the maximum tilt angle from the vertical that permits use of range finder and flow data
	float _pitch_offset_rad{3.14f}; 		///< range finder tilt rotation about the Y body axis
	float _sin_pitch_offset{0.0f}; 		///< sine of the range finder tilt rotation about the Y body axis
	float _cos_pitch_offset{-1.0f}; 		///< cosine of the range finder tilt rotation about the Y body axis

	/*
	 * Range check
	 */
	float _rng_valid_min_val{};	///< minimum distance that the rangefinder can measure (m)
	float _rng_valid_max_val{};	///< maximum distance that the rangefinder can measure (m)

	/*
	 * Quality check
	 */
	uint64_t _time_bad_quality_us{};	///< timestamp at which range finder signal quality was 0 (used for hysteresis)
	uint64_t _quality_hyst_us{};		///< minimum duration during which the reported range finder signal quality needs to be non-zero in order to be declared valid (us)
};

} // namespace sensor
} // namespace estimator
#endif // !EKF_SENSOR_RANGE_FINDER_HPP
