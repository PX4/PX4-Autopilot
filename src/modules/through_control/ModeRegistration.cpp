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

#include "ModeRegistration.hpp"

#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>
#include <string.h>

using namespace time_literals;

namespace through
{

namespace
{

/**
 * Collects arming check failure events into an arming_check_reply.
 *
 * The method name and the call layout below match what Tools/px4events/srcparser.py
 * extracts ("reporter.armingCheckFailureExt"), so the messages end up in the
 * event metadata and the GCS can display them (VERIFICATION.md V1 #10).
 */
struct ArmingCheckReporter {
	arming_check_reply_s &reply;

	void armingCheckFailureExt(uint32_t event_id, events::Log log_level, const char *message)
	{
		(void)message; // only used by the metadata generator

		if (reply.num_events >= sizeof(reply.events) / sizeof(reply.events[0])) {
			return;
		}

		event_s &event = reply.events[reply.num_events++];
		event = {};
		event.timestamp = hrt_absolute_time();
		event.id = event_id;
		// the commander fills the leading arguments (navigation mode group, health component index)
		const events::LogLevels log_levels{log_level};
		event.log_levels = ((uint8_t)log_levels.internal << 4) | (uint8_t)log_levels.external;
	}
};

} // namespace

ModeRegistration::ModeRegistration(const char *name)
{
	strncpy(_name, name, sizeof(_name) - 1);
}

void ModeRegistration::update(hrt_abstime now)
{
	switch (_state) {
	case State::Unregistered:
		sendRegistrationRequest(now);
		break;

	case State::WaitingForReply: {
			register_ext_component_reply_s reply;
			int tries = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

			while (_register_ext_component_reply_sub.update(&reply) && --tries >= 0) {
				if (reply.request_id != _request_id) {
					continue;
				}

				if (reply.success && reply.mode_id >= 0 && reply.arming_check_id >= 0) {
					_arming_check_id = reply.arming_check_id;
					_mode_id = reply.mode_id;
					_state = State::Registered;
					PX4_INFO("mode '%s' registered, mode_id (nav_state): %d, arming_check_id: %d",
						 _name, _mode_id, _arming_check_id);
					publishControlModeConfig();
					return;
				}

				PX4_WARN("mode registration rejected (attempt %d/%d)", _num_attempts, MAX_REGISTRATION_ATTEMPTS);
			}

			if (now > _last_request_time + 1_s) {
				if (_num_attempts >= MAX_REGISTRATION_ATTEMPTS) {
					PX4_ERR("mode registration failed after %d attempts", _num_attempts);
					_state = State::Failed;

				} else {
					sendRegistrationRequest(now);
				}
			}
		}
		break;

	case State::Registered:
	case State::Failed:
		break;
	}
}

void ModeRegistration::sendRegistrationRequest(hrt_abstime now)
{
	++_num_attempts;
	// new ID per attempt, so a late reply to an earlier attempt is not mistaken for this one
	_request_id = (static_cast<uint64_t>(now) << 8) | static_cast<uint64_t>(_num_attempts);

	register_ext_component_request_s request{};
	request.request_id = _request_id;
	strncpy(request.name, _name, sizeof(request.name) - 1);
	request.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
	request.register_arming_check = true;
	request.register_mode = true;
	request.register_mode_executor = false;
	request.enable_replace_internal_mode = false;
	request.activate_mode_immediately = false;
	request.timestamp = hrt_absolute_time();
	_register_ext_component_request_pub.publish(request);

	_last_request_time = now;
	_state = State::WaitingForReply;
}

void ModeRegistration::publishControlModeConfig()
{
	// Attitude/rate control with control allocation only: mc_att_control consumes our
	// vehicle_attitude_setpoint, while mc_pos_control and manual stabilized stay inactive
	// (VERIFICATION.md V1 #5, V4). flag_armed and flag_multicopter_position_control_enabled
	// are overwritten by the commander.
	vehicle_control_mode_s config{};
	config.source_id = _mode_id;
	config.flag_control_manual_enabled = false;
	config.flag_control_auto_enabled = false;
	config.flag_control_offboard_enabled = false;
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = false;
	config.flag_control_altitude_enabled = false;
	config.flag_control_climb_rate_enabled = false;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = true;
	config.flag_control_rates_enabled = true;
	config.flag_control_allocation_enabled = true;
	config.flag_control_termination_enabled = false;
	config.timestamp = hrt_absolute_time();
	_config_control_setpoints_pub.publish(config);
}

void ModeRegistration::handleArmingCheckRequest(ArmingFailure failure)
{
	if (!registered() || !_arming_check_request_sub.updated()) {
		return;
	}

	arming_check_request_s request;

	if (!_arming_check_request_sub.copy(&request)) {
		return;
	}

	arming_check_reply_s reply{};
	reply.request_id = request.request_id;
	reply.registration_id = _arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.num_events = 0;
	reply.can_arm_and_run = (failure == ArmingFailure::None);

	if (!reply.can_arm_and_run) {
		addArmingFailureEvent(reply, failure);
	}

	reply.mode_req_angular_velocity = true;
	reply.mode_req_attitude = true;
	reply.mode_req_local_alt = true;
	reply.mode_req_local_position = true;
	reply.mode_req_local_position_relaxed = false;
	reply.mode_req_global_position = false;
	reply.mode_req_global_position_relaxed = false;
	reply.mode_req_mission = false;
	reply.mode_req_home_position = false;
	reply.mode_req_prevent_arming = true; // can only be entered in air
	reply.mode_req_manual_control = false;

	reply.timestamp = hrt_absolute_time();
	_arming_check_reply_pub.publish(reply);
}

void ModeRegistration::addArmingFailureEvent(arming_check_reply_s &reply, ArmingFailure failure)
{
	ArmingCheckReporter reporter{reply};

	switch (failure) {
	case ArmingFailure::None:
		break;

	case ArmingFailure::NotInAir:
		/* EVENT
		 * @description
		 * THROUGH can only be entered while flying.
		 */
		reporter.armingCheckFailureExt(events::ID("through_not_in_air"), events::Log::Error, "THROUGH: vehicle not in air");
		break;

	case ArmingFailure::AltitudeLow:
		/* EVENT
		 * @description
		 * Climb above THRU_MIN_ALT before switching to THROUGH.
		 */
		reporter.armingCheckFailureExt(events::ID("through_altitude_low"), events::Log::Error, "THROUGH: altitude too low");
		break;

	case ArmingFailure::NotLocked:
		/* EVENT
		 * @description
		 * Lock the target in the vision node (key 'l' or the set_lock service) before switching to THROUGH.
		 */
		reporter.armingCheckFailureExt(events::ID("through_target_not_locked"), events::Log::Error, "THROUGH: target not locked");
		break;

	case ArmingFailure::TargetStale:
		/* EVENT
		 * @description
		 * No recent vision_target message (older than THRU_LOST_TO). Check the vision node and the DDS agent.
		 */
		reporter.armingCheckFailureExt(events::ID("through_target_stale"), events::Log::Error, "THROUGH: no vision data");
		break;

	case ArmingFailure::TargetInvalid:
		/* EVENT
		 * @description
		 * The vision node does not currently detect the target.
		 */
		reporter.armingCheckFailureExt(events::ID("through_target_not_detected"), events::Log::Error, "THROUGH: target not detected");
		break;
	}
}

void ModeRegistration::unregister()
{
	if (_state != State::Registered) {
		return;
	}

	unregister_ext_component_s unregister{};
	strncpy(unregister.name, _name, sizeof(unregister.name) - 1);
	unregister.arming_check_id = _arming_check_id;
	unregister.mode_id = _mode_id;
	unregister.mode_executor_id = -1;
	unregister.timestamp = hrt_absolute_time();
	_unregister_ext_component_pub.publish(unregister);

	_state = State::Unregistered;
	_arming_check_id = -1;
	_mode_id = -1;
}

const char *ModeRegistration::armingFailureString(ArmingFailure failure)
{
	switch (failure) {
	case ArmingFailure::None: return "ok";

	case ArmingFailure::NotInAir: return "not in air";

	case ArmingFailure::AltitudeLow: return "altitude low";

	case ArmingFailure::NotLocked: return "target not locked";

	case ArmingFailure::TargetStale: return "no vision data";

	case ArmingFailure::TargetInvalid: return "target not detected";
	}

	return "unknown";
}

} // namespace through
