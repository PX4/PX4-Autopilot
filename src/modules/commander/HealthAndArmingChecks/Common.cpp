/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "Common.hpp"

void Report::healthFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
			   const char *message, events::Log log_level)
{
	required_modes = applyCurrentNavState(required_modes);
	healthFailure(required_modes, component, log_level);
	addEvent(event_id, message, log_level, (uint8_t)required_modes, (uint8_t)component);
}

void Report::armingCheckFailure(ModeCategory required_modes, HealthComponentIndex component, uint32_t event_id,
				const char *message, events::Log log_level)
{
	required_modes = applyCurrentNavState(required_modes);
	armingCheckFailure(required_modes, component, log_level);
	addEvent(event_id, message, log_level, (uint8_t)required_modes, (uint8_t)component);
}

void Report::setIsPresent(HealthComponentIndex component)
{
	HealthResults &health = _results[_current_result].health;
	health.is_present = health.is_present | (events::common::enums::health_component_t)(1 << (uint8_t)component);
}

void Report::healthFailure(ModeCategory required_modes, HealthComponentIndex component, events::Log log_level)
{
	// update current health results
	HealthResults &health = _results[_current_result].health;

	if (log_level <= events::Log::Error) {
		health.error = health.error | (events::common::enums::health_component_t)(1 << (uint8_t)component);

	} else if (log_level <= events::Log::Warning) {
		health.warning = health.warning | (events::common::enums::health_component_t)(1 << (uint8_t)component);
	}

	// clear relevant arming bits
	ModeCategory &can_arm = _results[_current_result].arming_checks.can_arm;
	can_arm = can_arm & ~required_modes;
}

void Report::armingCheckFailure(ModeCategory required_modes, HealthComponentIndex component, events::Log log_level)
{
	// update current arming check results
	ArmingCheckResults &arming_checks = _results[_current_result].arming_checks;

	if (log_level <= events::Log::Error) {
		arming_checks.error = arming_checks.error | (events::common::enums::health_component_t)(1 << (uint8_t)component);

	} else if (log_level <= events::Log::Warning) {
		arming_checks.warning = arming_checks.warning | (events::common::enums::health_component_t)(1 << (uint8_t)component);
	}

	// clear relevant arming bits
	ModeCategory &can_arm = arming_checks.can_arm;
	can_arm = can_arm & ~required_modes;
}

void Report::reset(uint8_t nav_state)
{
	_current_result = (_current_result + 1) % 2;
	_results[_current_result].reset();
	_results[_current_result].nav_state = nav_state;
	_next_buffer_idx = 0;
	_buffer_overflowed = false;
}

ModeCategory Report::applyCurrentNavState(ModeCategory modes) const
{
	ModeCategory mode_cat = ModeCategory::None;

	switch (_results[_current_result].nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
	case vehicle_status_s::NAVIGATION_STATE_ACRO:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
	case vehicle_status_s::NAVIGATION_STATE_STAB:
	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		mode_cat = ModeCategory::Altitude;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		mode_cat = ModeCategory::Position;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		mode_cat = ModeCategory::Autonomous;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		mode_cat = ModeCategory::Mission;
		break;
	}

	static_assert(vehicle_status_s::NAVIGATION_STATE_MAX == 22, "New nav state added, update the logic above");

	if (mode_cat == ModeCategory::None) {
		if (modes != ModeCategory::None) {
			// be conservative: apply to current mode for unknown/unhandled navigation states
			modes = modes | ModeCategory::Current;
		}

	} else {
		// check if modes affects the current navigation state as well (which will then clear the arming bit)
		if ((uint8_t)(modes & mode_cat) != 0) {
			modes = modes | ModeCategory::Current;
		}
	}

	return modes;
}

void Report::finalize()
{
	_results[_current_result].arming_checks.valid = true;
	_already_reported = false;
}

void Report::report(bool force)
{
	const hrt_abstime now = hrt_absolute_time();
	const bool has_difference = _had_unreported_difference || _results[0] != _results[1];

	if (now - _last_report < min_reporting_interval && !force) {
		if (has_difference) {
			_had_unreported_difference = true;
		}

		PX4_DEBUG("not reporting yet (%i)", has_difference);
		return;
	}

	const bool need_to_report = (has_difference && !_already_reported) || force;

	if (!need_to_report) {
		PX4_DEBUG("not reporting, no update (%i)", has_difference);
		return;
	}

	const Results &current_results = _results[_current_result];

	// If we have too many events, the result is still correct, we just don't report everything
	const int max_num_events = event_s::ORB_QUEUE_LENGTH - 2;

	if (_buffer_overflowed || current_results.num_events > max_num_events) {
		PX4_WARN("Too many arming check events (%i, %i > %i). Not reporting all", _buffer_overflowed,
			 current_results.num_events, max_num_events);
	}

	if (!current_results.arming_checks.valid) {
		PX4_ERR("BUG: arming checks not valid");
	}

	_last_report = now;
	_already_reported = true;
	_had_unreported_difference = false;
	PX4_DEBUG("Sending Arming/Health report (num events: %i, can_arm: 0x%x)", _results[_current_result].num_events + 2,
		  (int)current_results.arming_checks.can_arm);

	// send arming summary
	events::EventType event = events::common::create_arming_check_summary({events::Log::Protocol, events::LogInternal::Disabled},
				  current_results.arming_checks.error, current_results.arming_checks.warning,
				  (events::common::enums::navigation_mode_category_t)current_results.arming_checks.can_arm);
	events::send(event);

	// send all events
	int offset = 0;

	for (int i = 0; i < max_num_events && offset < _next_buffer_idx; ++i) {
		EventBufferHeader *header = (EventBufferHeader *)(_event_buffer + offset);
		memcpy(&event.id, &header->id, sizeof(event.id));
		event.log_levels = ((uint8_t)events::LogInternal::Disabled << 4) | header->log_level;
		memcpy(event.arguments, _event_buffer + offset + sizeof(EventBufferHeader), header->size);
		memset(event.arguments + header->size, 0, sizeof(event.arguments) - header->size);
		events::send(event);
		offset += sizeof(EventBufferHeader) + header->size;
	}

	// send health summary
	event = events::common::create_health_summary({events::Log::Protocol, events::LogInternal::Disabled},
			current_results.health.is_present,
			current_results.health.error, current_results.health.warning);
	events::send(event);
}

