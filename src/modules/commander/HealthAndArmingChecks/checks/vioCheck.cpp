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
	// TODO: gate the whole check on an enable parameter, e.g.:
	// if (!_param_ev_check_en.get()) {
	//     return;
	// }

	// Sub-checks. Each one decides for itself whether it is relevant pre-arm,
	// in flight, or both, and reports via the `reporter`.
	checkDataStream(context, reporter);
	checkEstimateQuality(context, reporter);

	// TODO: if the VIO source is detected/configured, advertise it as present
	// so it shows up in the health report, e.g.:
	// reporter.setIsPresent(health_component_t::???);
}

void VioChecks::checkDataStream(const Context &context, Report &reporter)
{
	// Relevant both before arming and during flight: make sure we are still
	// receiving fresh VIO/EV samples.
	//
	// TODO: copy the latest sample and evaluate timeout / staleness, e.g.:
	//   vehicle_odometry_s odom;
	//   if (_visual_odometry_sub.update(&odom)) {
	//       _last_valid_sample = odom.timestamp;
	//   }
	//   const bool data_stale = (hrt_elapsed_time(&_last_valid_sample) > _param_ev_timeout.get() * 1_s);

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
	// Primarily an in-flight monitor: the data may be arriving but its quality
	// (covariance, tracking confidence, ...) may have degraded.
	if (!context.isArmed()) {
		// TODO: decide whether quality should also be enforced pre-arm.
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
	// if (_vio_features_sub.updated()) {
	// 	formic_vio_features_s vio_features{};
	// 	_vio_features_sub.copy(&vio_features);
	// 	_last_valid_sample = vio_features.timestamp;
	// 	_vio_features_queue.push(new formic_vio_features_s(vio_features));
	// 	if (_vio_features_queue.size() >= MAX_QUEUE_SIZE) {
		
	// 		delete _vio_features_queue.pop();
	// 	}
	// 	if (hrt_abstime elapsed_time(&_last_valid_sample) > 1_s){
	// 		_vio_features_queue.empty();
	// 	}
	// }
}



