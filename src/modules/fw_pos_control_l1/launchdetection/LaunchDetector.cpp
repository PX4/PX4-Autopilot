/****************************************************************************
 *
 *   Copyright (c) 2013-2022 PX4 Development Team. All rights reserved.
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
 * @file launchDetection.cpp
 * Auto launch detection for catapult/hand-launch vehicles
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "LaunchDetector.h"

#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <px4_platform_common/events.h>

namespace launchdetection
{

void LaunchDetector::update(const float dt, const float accel_x,  orb_advert_t *mavlink_log_pub)
{
	switch (state_) {
	case launch_detection_status_s::STATE_WAITING_FOR_LAUNCH:

		_info_delay_counter_s_ += dt;

		/* Inform user that launchdetection is running every kInfoDelay seconds */
		if (_info_delay_counter_s_ >= kInfoDelay) {
			mavlink_log_info(mavlink_log_pub, "Launch detection running\t");
			events::send(events::ID("launch_detection_running_info"), events::Log::Info, "Launch detection running");
			_info_delay_counter_s_ = 0.f; // reset counter
		}

		/* Detect a acceleration that is longer and stronger as the minimum given by the params */
		if (accel_x > _param_laun_cat_a.get()) {
			_info_delay_counter_s_ += dt;

			if (_info_delay_counter_s_ > _param_laun_cat_t.get()) {
				if (_param_laun_cat_mdel.get() > 0.f) {
					state_ = launch_detection_status_s::STATE_LAUNCH_DETECTED_DISABLED_MOTOR;
					mavlink_log_info(mavlink_log_pub, "Launch detected: enable control, waiting %8.1fs until throttling up\t",
							 (double)_param_laun_cat_mdel.get());
					events::send<float>(events::ID("launch_detection_wait_for_throttle"), {events::Log::Info, events::LogInternal::Info},
							    "Launch detected: enablecontrol, waiting {1:.1}s until full throttle", (double)_param_laun_cat_mdel.get());

				} else {
					/* No motor delay set: go directly to enablemotors state */
					state_ = launch_detection_status_s::STATE_FLYING;
					mavlink_log_info(mavlink_log_pub, "Launch detected: enable motors (no motor delay)\t");
					events::send(events::ID("launch_detection_no_motor_delay"), {events::Log::Info, events::LogInternal::Info},
						     "Launch detected: enable motors (no motor delay)");
				}
			}

		} else {
			reset();
		}

		break;

	case launch_detection_status_s::STATE_LAUNCH_DETECTED_DISABLED_MOTOR:
		/* Vehicle is currently controlling attitude but at idle throttle. Waiting until delay is
		 * over to allow full throttle */
		motor_delay_counter_ += dt;

		if (motor_delay_counter_ > _param_laun_cat_mdel.get()) {
			mavlink_log_info(mavlink_log_pub, "Launch detected: enable motors\t");
			events::send(events::ID("launch_detection_enable_motors"), {events::Log::Info, events::LogInternal::Info},
				     "Launch detected: enable motors");
			state_ = launch_detection_status_s::STATE_FLYING;
		}

		_info_delay_counter_s_ = kInfoDelay; // reset counter

		break;

	default:
		_info_delay_counter_s_ = kInfoDelay; // reset counter
		break;

	}
}

uint LaunchDetector::getLaunchDetected() const
{
	return state_;
}

void LaunchDetector::reset()
{
	_info_delay_counter_s_ = 0.f;
	motor_delay_counter_ = 0.f;
	state_ = launch_detection_status_s::STATE_WAITING_FOR_LAUNCH;
}


} // namespace launchdetection
