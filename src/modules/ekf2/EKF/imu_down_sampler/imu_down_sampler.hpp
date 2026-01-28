/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Downsamples IMU data to a lower rate such that EKF predicition can happen less frequent
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_IMU_DOWN_SAMPLER_HPP
#define EKF_IMU_DOWN_SAMPLER_HPP

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

#include "../common.h"

using namespace estimator;

class ImuDownSampler
{
public:
	explicit ImuDownSampler(int32_t &target_dt_us);
	~ImuDownSampler() = default;

	bool update(const imuSample &imu_sample_new);

	imuSample getDownSampledImuAndTriggerReset()
	{
		imuSample imu{_imu_down_sampled};
		reset();
		return imu;
	}

private:
	void reset();

	imuSample _imu_down_sampled{};
	Quatf _delta_angle_accumulated{};

	int _accumulated_samples{0};
	int _required_samples{1};

	int32_t &_target_dt_us;

	float _target_dt_s{0.010f};
	float _min_dt_s{0.005f};

	float _delta_ang_dt_avg{0.005f};
};
#endif // !EKF_IMU_DOWN_SAMPLER_HPP
