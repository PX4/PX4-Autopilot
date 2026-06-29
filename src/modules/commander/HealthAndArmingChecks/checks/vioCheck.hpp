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
#include <uORB/topics/formic_vio_features.h>
// #include <containers/RingQueue.hpp>
#include <containers/IntrusiveQueue.hpp>
// TODO: include the actual VIO/EV topics this check consumes, e.g.:
// #include <uORB/topics/vehicle_visual_odometry.h>
// #include <uORB/topics/estimator_status.h>

class VioChecks : public HealthAndArmingCheckBase
{
public:
	VioChecks() = default;
	~VioChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	// --- Sub-checks: fill in the actual logic in vio_check.cpp ---

	/**
	 * Verify the VIO/EV data stream is present and recent.
	 * Relevant both pre-arm and in flight.
	 */
	void checkDataStream(const Context &context, Report &reporter);
	void get_vio_features();

	/**
	 * Verify the VIO estimate quality (e.g. covariance / tracking confidence).
	 * Primarily an in-flight monitor.
	 */
	void checkEstimateQuality(const Context &context, Report &reporter);

	uORB::Subscription _vio_features_sub{ORB_ID(formic_vio_features)}; // TODO: replace with the actual VIO/EV topic(s)
	// static constexpr size_t MAX_VIO_FEATURES = 50;
	// formic_vio_features_s _vio_features{};
	IntrusiveQueue<formic_vio_features_s *> _vio_features_queue{}; // TODO: replace with a RingQueue if needed
	// std::array<formic_vio_features_s, MAX_VIO_FEATURES> _vio_features{};

	// --- State / subscriptions (placeholders) ---
	// TODO: subscribe to the real topic(s), e.g.:
	// uORB::Subscription _visual_odometry_sub{ORB_ID(vehicle_visual_odometry)};
	hrt_abstime _last_valid_sample{0};

	static constexpr size_t MAX_QUEUE_SIZE = 10;

	// TODO: once this check has parameters, declare them here so they are
	// auto-updated, e.g.:
	// DEFINE_PARAMETERS_CUSTOM_PARENT(HealthAndArmingCheckBase,
	//                 (ParamInt<px4::params::EV_CHECK_EN>)   _param_ev_check_en,
	//                 (ParamFloat<px4::params::EV_TIMEOUT>)  _param_ev_timeout
	//                )
};
