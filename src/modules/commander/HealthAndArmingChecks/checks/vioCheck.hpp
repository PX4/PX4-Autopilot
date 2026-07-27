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

/**
 * @file vio_check.hpp
 *
 * Health & arming check for the Visual-Inertial Odometry (VIO) / external
 * vision (EV) source. This is the generic scaffold: the data subscriptions,
 * the parameter hooks and the pre-arm vs. in-flight branches are wired up,
 * the concrete pass/fail logic is left as TODOs to be filled in.
 *
 * Runs as part of the HealthAndArmingChecks framework at 10 Hz (both before
 * arming and continuously during flight).
 */

#pragma once

#include "../Common.hpp"
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/topics/formic_vio_features.h>
#include <uORB/topics/features_filter.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>

class VioChecks : public HealthAndArmingCheckBase
{
public:
	VioChecks() = default;
	~VioChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:

	// Low-pass filter tuning. The VIO feature stream is not perfectly periodic, so
	// these are nominal values: a ~20 Hz sample rate with a 1 Hz cutoff smooths
	// per-frame spikes while still tracking sustained drops in feature count.
	static constexpr float FILTER_SAMPLE_FREQ = 20.0f;
	static constexpr float FILTER_CUTOFF_FREQ = 1.0f;


	void checkFeaturesQuality(const Context &context, Report &reporter);
	bool get_vio_features();
	void checkEstimateQuality(const Context &context, Report &reporter);

	uORB::Subscription _vio_features_sub{ORB_ID(formic_vio_features)}; 
	uORB::Publication<features_filter_s> _features_filter_pub{ORB_ID(features_filter)};


	math::LowPassFilter2p<float> _slam_features_filter{FILTER_SAMPLE_FREQ, FILTER_CUTOFF_FREQ};
	math::LowPassFilter2p<float> _msckf_features_filter{FILTER_SAMPLE_FREQ, FILTER_CUTOFF_FREQ};

	hrt_abstime _last_valid_sample{0};
	float _slam_features_filtered = 0.0f;   // low-pass filtered feature count
	float _msckf_features_filtered = 0.0f;

	bool on_startup{true}; // true until the first checkAndReport() call, then false
	hrt_abstime _startup_timer_start = 0;



	// DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase)

	DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
					(ParamInt<px4::params::FORMIC_VIO_MF>) _param_formic_vio_minimum_features
				       )

	/// TODO NAOR:
	// 1) dont do this check at the start of the data or when the qaue isnt full 
	// 2) report if the data low - later connect this to stop the copy of the data until the data good again 


};
