/****************************************************************************
 *
 *   Copyright (c) 2019 Estimation and Control Library (ECL). All rights reserved.
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
 * Downsamples IMU data to a lower rate such that EKF predicition can happen less frequent
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_IMU_DOWN_SAMPLER_HPP
#define EKF_IMU_DOWN_SAMPLER_HPP

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "common.h"

using namespace estimator;

class ImuDownSampler
{
public:
	explicit ImuDownSampler(float target_dt_sec);
	~ImuDownSampler() = default;

	bool update(const imuSample &imu_sample_new);

	imuSample getDownSampledImuAndTriggerReset()
	{
		_do_reset = true;
		return _imu_down_sampled;
	}

private:
	void reset();

	imuSample _imu_down_sampled{};
	Quatf _delta_angle_accumulated{};
	const float _target_dt;  // [sec]
	float _imu_collection_time_adj{0.f};
	bool _do_reset{true};
};
#endif // !EKF_IMU_DOWN_SAMPLER_HPP
