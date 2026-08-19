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

#ifndef EKF_OPTICAL_FLOW_SOURCE_HPP
#define EKF_OPTICAL_FLOW_SOURCE_HPP

#include "../../common.h"

#if defined(CONFIG_EKF2_OPTICAL_FLOW)

#include <lib/ringbuffer/TimestampedRingBuffer.hpp>
#include <mathlib/math/filter/AlphaFilter.hpp>
#include <uORB/topics/estimator_aid_source2d.h>

class Ekf;

class OpticalFlowSource
{
public:
	struct Params {
		int32_t ctrl{0};
		int32_t gyr_src{static_cast<int32_t>(estimator::FlowGyroSource::Auto)};
		float n_min{0.15f};	///< best quality observation noise for optical flow LOS rate measurements (rad/sec)
		float n_max{0.5f};	///< worst quality observation noise for optical flow LOS rate measurements (rad/sec)
		int32_t qmin{1};	///< minimum acceptable quality integer from the flow sensor when in air
		int32_t qmin_gnd{0};	///< minimum acceptable quality integer from the flow sensor when on ground
		float gate{3.0f};	///< optical flow fusion innovation consistency gate size (STD)
	};

	Params params{};

	~OpticalFlowSource() { delete _buffer; }

	void setSlot(uint8_t slot) { _slot = slot; }

	bool allocate(uint8_t buffer_length);

	void setData(const estimator::flowSample &sample, uint64_t min_obs_interval_us, float dt_ekf_avg);

	void setLimits(float max_flow_rate, float min_dist, float max_dist)
	{
		_max_rate = max_flow_rate;
		_min_distance = min_dist;
		_max_distance = max_dist;
	}

	void setPositionBody(const matrix::Vector3f &pos) { _pos_body = pos; }

	bool update(Ekf &ekf, const estimator::imuSample &imu_delayed, bool allow_fusion);

	void stop();

private:
	friend class Ekf;

	void reset(Ekf &ekf);
	void resetTerrain(Ekf &ekf);

	void calcBodyRateComp(const matrix::Vector3f &ref_body_rate);

	float calcOptFlowMeasVar(const estimator::flowSample &flow_sample) const;

	TimestampedRingBuffer<estimator::flowSample> *_buffer{nullptr};
	estimator::flowSample _sample_delayed{};

	estimator_aid_source2d_s _aid_src{};

	matrix::Vector3f _pos_body{};	///< xyz position of the sensor focal point in body frame (m)

	matrix::Vector3f _gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	matrix::Vector2f _vel_body{};	///< velocity from corrected flow measurement (body frame)(m/s)
	AlphaFilter<matrix::Vector2f> _vel_body_lpf{};	///< filtered velocity from corrected flow measurement (body frame)(m/s)
	matrix::Vector2f _rate_compensated{};	///< measured angular rate of the image about the X and Y body axes after removal of body rotation (rad/s), RH rotation is positive
	AlphaFilter<matrix::Vector2f> _rate_compensated_lpf{};
	uint32_t _counter{0};	///< number of flow samples read for initialization

	// Sensor limitations
	float _max_rate{1.0f};		///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float _min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float _max_distance{10.f};	///< maximum distance that the optical flow sensor can operate at (m)

	bool _active{false};
	bool _terrain{false};

	uint8_t _slot{0};
};

#endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // !EKF_OPTICAL_FLOW_SOURCE_HPP
