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

#include "vioCheck.hpp"

using namespace time_literals;

void VioChecks::checkAndReport(const Context &context, Report &reporter)
{
	// TODO: gate the whole check on a dedicated enable parameter once one exists.

	// Sub-checks. Each one decides for itself whether it is relevant pre-arm,
	// in flight, or both, and reports via the `reporter`.
	checkDataStream(context, reporter);
	// checkEstimateQuality(context, reporter);
}

void VioChecks::checkDataStream(const Context &context, Report &reporter)
{
    get_vio_features();
    // It's a good PX4 practice to zero-initialize the struct to prevent garbage data
    features_filter_s pub_data{};
    pub_data.slam_features_filterd = _slam_features_filtered;
    pub_data.msckf_features_filterd = _msckf_features_filtered;
    pub_data.timestamp = hrt_absolute_time();
    pub_data.at_problem = (pub_data.slam_features_filterd < THERSHOLD);
    _features_filter_pub.publish(pub_data);

	const bool data_stale = false; // TODO: replace with the real condition

	if (data_stale) {
		/* EVENT
		 * @description
		 * No recent visual odometry (VIO/EV) data was received.
		 */
		reporter.healthFailure(NavModes::All, health_component_t::system,
				       events::ID("check_vio_no_data"),
				       events::Log::Error, "No visual odometry data");

		if (reporter.mavlink_log_pub()) {
			mavlink_log_critical(reporter.mavlink_log_pub(), "Preflight Fail: No visual odometry data");
		}
	}
}

void VioChecks::checkEstimateQuality(const Context &context, Report &reporter)
{
	if (!context.isArmed()) {
		// the check runs at disarm and arm 
		return;
	}

	const bool quality_bad = false; // TODO: replace with the real condition

	if (quality_bad) {
		/* EVENT
		 * @description
		 * Visual odometry (VIO/EV) estimate quality is degraded.
		 */
		reporter.healthFailure(NavModes::All, health_component_t::system,
				       events::ID("check_vio_quality"),
				       events::Log::Error, "Visual odometry quality degraded");
	}
}




void VioChecks::get_vio_features(){
	if (_vio_features_sub.updated()) {
		formic_vio_features_s vio_features{};
		_vio_features_sub.copy(&vio_features);

		// If there was a gap (>1 s) since the previous sample, the filter state is
		// stale — re-seed it with the fresh sample instead of ramping from the old one.
		if (_last_valid_sample != 0 && (vio_features.timestamp - _last_valid_sample) > 1_s) {
			_slam_features_filter.reset(vio_features.slam_features);
			_msckf_features_filter.reset(vio_features.msckf_features);
			on_startup = true;
			_startup_timer_start = hrt_absolute_time();
		}
		_last_valid_sample = vio_features.timestamp;

		// Feed the low-pass filters and keep the filtered feature counts.
		_slam_features_filtered = _slam_features_filter.apply(vio_features.slam_features);
		_msckf_features_filtered = _msckf_features_filter.apply(vio_features.msckf_features);

	}

	if (on_startup){
		if (hrt_elapsed_time(&_startup_timer_start) > 2_s){
			// wait some time to start check this error - at the start the slam feathers is at zero 
			on_startup = false;
		}
	}
}


