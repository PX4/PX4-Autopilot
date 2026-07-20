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

// Per-sensor optical flow state (one instance per configured flow sensor slot)
struct OpticalFlowSource {
	TimestampedRingBuffer<estimator::flowSample> *buffer{nullptr};
	estimator::flowSample sample_delayed{};

	estimator_aid_source2d_s aid_src{};

	matrix::Vector3f gyro_bias{};	///< bias errors in optical flow sensor rate gyro outputs (rad/sec)
	matrix::Vector2f vel_body{};	///< velocity from corrected flow measurement (body frame)(m/s)
	AlphaFilter<matrix::Vector2f> vel_body_lpf{};	///< filtered velocity from corrected flow measurement (body frame)(m/s)
	matrix::Vector2f rate_compensated{};	///< measured angular rate of the image about the X and Y body axes after removal of body rotation (rad/s), RH rotation is positive
	AlphaFilter<matrix::Vector2f> rate_compensated_lpf{};
	uint32_t counter{0};	///< number of flow samples read for initialization

	// Sensor limits reported by the optical flow sensor
	float max_rate{1.0f};		///< maximum angular flow rate that the optical flow sensor can measure (rad/s)
	float min_distance{0.0f};	///< minimum distance that the optical flow sensor can operate at (m)
	float max_distance{10.f};	///< maximum distance that the optical flow sensor can operate at (m)

	bool active{false};	///< true when flow fusion from this sensor is active
	bool terrain{false};	///< true when flow fusion from this sensor is updating the terrain state

	~OpticalFlowSource() { delete buffer; }
};

#endif // CONFIG_EKF2_OPTICAL_FLOW

#endif // !EKF_OPTICAL_FLOW_SOURCE_HPP
