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
 * @file detect_and_avoid.cpp
 *
 * Helper class to do detect and avoid traffic
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "../navigator.h"

#include <cinttypes>
#include <lib/mathlib/mathlib.h>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>

#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;

DetectAndAvoid::DetectAndAvoid(Navigator *navigator) :
	MissionBlock(navigator, 0),
	ModuleParams(navigator)
{
	_detect_and_avoid_pub.advertise();
	_detect_and_avoid_most_urgent_pub.advertise();
}

void DetectAndAvoid::reset()
{
	_is_activated = false;
#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	stop_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC
	clear_conflicts();
}

void DetectAndAvoid::clear_conflicts()
{
	_conflict_tracker.clear();
	_most_urgent_conflict_changed = true;
	_previous_action = DaaAction::kDisabled;
	_prev_most_urgent_conflict_level = detect_and_avoid_s::DAA_CONFLICT_LVL_NONE;
	_time_last_buffer_clean = 0;
	_time_last_status_notif = 0;
	_time_last_traffic_ignored = 0;
	_time_last_landed_warning = 0;
}

void DetectAndAvoid::on_inactivation()
{
	PX4_DEBUG("DAA inactivation");
	reset();
	publish_most_urgent_conflict_if_changed();
}

void DetectAndAvoid::on_activation()
{
	reset();
	update_activation_status();
	publish_most_urgent_conflict_if_changed();
}

void DetectAndAvoid::update_activation_status()
{
	ModuleParams::updateParams();

	if (!_param_daa_en.get()) {
		if (_is_activated) {
			PX4_DEBUG("DAA: module disabled");
			reset();
		}

		return;
	}

	if (!_adsb_conflict_detector.try_updating_params()) {
		if (_is_activated) {
			reset();
		}

		mavlink_log_critical(&_mavlink_log_pub, "DAA invalid params. Traffic warnings and actions disabled.\t");
		events::send(events::ID("navigator_traffic_init_failed"), events::Log::Critical,
			     "DAA invalid params. Traffic warnings and actions disabled");
		return;
	}

	if (!_is_activated) {
		PX4_DEBUG("DAA: init ok");
		_is_activated = true;
	}
}

bool DetectAndAvoid::has_elapsed(hrt_abstime &last_time, const hrt_abstime interval)
{
	if ((last_time == 0) || (hrt_elapsed_time(&last_time) > interval)) {
		last_time = hrt_absolute_time();
		return true;
	}

	return false;
}

void DetectAndAvoid::on_active()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		update_activation_status();
	}

	if (_is_activated) {
		process_traffic();
	}

	publish_most_urgent_conflict_if_changed();
}

void DetectAndAvoid::process_traffic()
{
	if (!uav_pose_valid_and_updated()) {
		if (!_conflict_tracker.empty()
		    || _conflict_tracker.most_urgent().conflict_level != detect_and_avoid_s::DAA_CONFLICT_LVL_NONE) {
			PX4_DEBUG("DAA: uav pose stale, clearing conflicts");
			clear_conflicts(); // do not call reset() to stay activated once the pose recovers
		}

		return;
	}

#if defined(CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC)
	process_fake_traffic();
#endif // CONFIG_NAVIGATOR_ADSB_FAKE_TRAFFIC

	new_conflicts_pending_notif_s new_conflicts_pending_notif{};

	if (has_elapsed(_time_last_buffer_clean, kRemoveStaleConflictsTime)) {
		conflict_tracker_changes_s stale_changes{};

		const bool stale_conflicts_removed = _conflict_tracker.remove_stale_conflicts(hrt_absolute_time(),
						     static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s, stale_changes);
		handle_tracker_changes(stale_changes, new_conflicts_pending_notif);

		if (stale_conflicts_removed) {
			update_most_urgent_conflict();
		}
	}

	if (process_transponder_queue(new_conflicts_pending_notif)) {
		update_most_urgent_conflict();
	}

	notify_if_needed(new_conflicts_pending_notif);

	if (_most_urgent_conflict_changed) {
		evaluate_and_publish_action();

		_prev_most_urgent_conflict_level = _conflict_tracker.most_urgent().conflict_level;
	}
}

bool DetectAndAvoid::process_transponder_queue(new_conflicts_pending_notif_s &new_conflicts_pending_notif)
{
	bool buffer_updated = false;
	transponder_report_s transponder_report{};

	for (uint8_t processed_reports = 0; processed_reports < transponder_report_s::ORB_QUEUE_LENGTH; ++processed_reports) {

		if (!_traffic_sub.update(&transponder_report)) {
			break;
		}

#if defined(DEBUG_BUILD)
		debug_print_transponder_report(transponder_report);
#endif

		if (!transponder_data_valid(transponder_report)) {
			PX4_DEBUG("DAA: transponder data not valid.");
			continue;
		}

		const bool report_processed = process_transponder_report(transponder_report, new_conflicts_pending_notif);
		buffer_updated = buffer_updated || report_processed;
	}

	return buffer_updated;
}

bool DetectAndAvoid::process_transponder_report(const transponder_report_s &transponder_report,
		new_conflicts_pending_notif_s &new_conflicts_pending_notif)
{
	DaaEncodedId encoded_id{};

	if (!identify_traffic_report(transponder_report, encoded_id)) {
		return false;
	}

	const daa_input_s daa_input = prepare_daa_input(transponder_report);

	detect_and_avoid_s daa_output{};

	if (!_adsb_conflict_detector.calculate_daa_output(daa_input, daa_output)) {
		PX4_DEBUG("DAA: Failed to calculate DAA output, skipping report.");
		return false;
	}

	if (daa_output.conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE
	    && !_conflict_tracker.contains(encoded_id)) {
		return false;
	}

	conflict_info_s current_conflict{};
	current_conflict.encoded_id = encoded_id;
	current_conflict.latest_update_timestamp = transponder_report.timestamp;
	current_conflict.conflict_level = daa_output.conflict_level;
	current_conflict.aircraft_dist = hypotf(daa_output.aircraft_dist_hor, daa_output.aircraft_dist_vert);

	daa_output.unique_id = encoded_id.id;
	daa_output.unique_id_encoding = encoded_id.encoding;
	daa_output.timestamp = transponder_report.timestamp;
	_detect_and_avoid_pub.publish(daa_output);

#if defined(DEBUG_BUILD)
	PX4_DEBUG("DAA: transponder data processed, conflict detected.");
	debug_print_conflict_info(current_conflict);
#endif

	conflict_tracker_changes_s tracker_changes{};
	const bool buffer_updated = _conflict_tracker.apply_conflict(current_conflict, tracker_changes);

	handle_tracker_changes(tracker_changes, new_conflicts_pending_notif);

	return buffer_updated;
}

void DetectAndAvoid::handle_tracker_changes(const conflict_tracker_changes_s &changes,
		new_conflicts_pending_notif_s &new_conflicts_pending_notif)
{
	for (size_t i = 0; i < changes.size(); ++i) {
		const conflict_tracker_change_s &change = changes[i];

		switch (change.type) {
		case ConflictTrackerChangeType::kConflictAdded:
			if (conflict_lvl_requires_warning(change.conflict.conflict_level)) {
				// Defer "new conflict" warnings until the full ORB queue has been processed.
				new_conflicts_pending_notif.push_back(change.conflict.encoded_id);
			}

			break;

		case ConflictTrackerChangeType::kConflictLevelChanged:
			maybe_notify_secondary_level_change(change);
			break;

		case ConflictTrackerChangeType::kConflictRemoved:

			// Traffic removed before their deferred "new" warning was sent were never
			// user-visible and must not emit a misleading removal warning.
			// E.g. in the same transponder queue a conflict is added and removed
			if (conflict_lvl_requires_warning(change.conflict.conflict_level)
			    && !pending_new_conflict_notification_exists(change.conflict.encoded_id, new_conflicts_pending_notif)) {
				notify_traffic_removed(change.conflict, change.remove_cause);
			}

			break;

		case ConflictTrackerChangeType::kReportIgnored:
			maybe_notify_ignored_traffic(change.conflict, change.ignore_cause);
			break;
		}
	}
}

void DetectAndAvoid::maybe_notify_secondary_level_change(const conflict_tracker_change_s &change)
{
	const bool level_change_requires_notification = conflict_lvl_requires_warning(change.previous_level)
			|| conflict_lvl_requires_warning(change.conflict.conflict_level);

	// Known traffic, only notify if level changed. If most urgent, no need to notify, will be done in DAA status.
	if (!level_change_requires_notification
	    || _conflict_tracker.most_urgent().encoded_id == change.conflict.encoded_id) {
		return;
	}

	conflict_info_s current_most_urgent_conflict{};

	if (!_conflict_tracker.find_most_urgent(current_most_urgent_conflict)
	    || current_most_urgent_conflict.encoded_id != change.conflict.encoded_id) {
		notify_conflict_level(change.conflict, change.previous_level, ConflictNotifyKind::kSecondary);
	}
}

bool DetectAndAvoid::identify_traffic_report(const transponder_report_s &transponder_report,
		DaaEncodedId &encoded_id) const
{
	encoded_id = DaaEncodedId::from_report(transponder_report);

	if (encoded_id.id == 0) {
		PX4_DEBUG("DAA: No valid unique ID, skipping report");
		return false;
	}

	if (is_self_detection(encoded_id)) {
		PX4_DEBUG("DAA: Self detection, skipping report.");
		return false;
	}

#if defined(DEBUG_BUILD)
	char encoded_id_str[kUtmGuidMsgLength];
	encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));
	PX4_DEBUG("DAA: unique ID: %s (int:%lu)", encoded_id_str, encoded_id.id);
#endif

	return true;
}

#if !defined(CONSTRAINED_FLASH) && !defined(__PX4_NUTTX)
void DetectAndAvoid::print_status() const
{
	PX4_INFO("DAA: buf %u/%u, lvl %u, dist %.1fm",
		 static_cast<unsigned>(_conflict_tracker.size()),
		 static_cast<unsigned>(kDaaMaxTraffic),
		 static_cast<unsigned>(_conflict_tracker.most_urgent().conflict_level),
		 static_cast<double>(_conflict_tracker.most_urgent().aircraft_dist));
}
#endif // !CONSTRAINED_FLASH && !__PX4_NUTTX

void DetectAndAvoid::publish_most_urgent_conflict_if_changed()
{
	if (!_most_urgent_conflict_changed) {
		return;
	}

	_most_urgent_conflict_changed = false;

	const conflict_info_s &most_urgent_conflict = _conflict_tracker.most_urgent();

	detect_and_avoid_most_urgent_s daa_status{};

	daa_status.timestamp = hrt_absolute_time();
	daa_status.unique_id = most_urgent_conflict.encoded_id.id;
	daa_status.unique_id_encoding = most_urgent_conflict.encoded_id.encoding;

	daa_status.has_action = conflict_lvl_requires_action(most_urgent_conflict.conflict_level);
	daa_status.conflict_level = most_urgent_conflict.conflict_level;
	daa_status.aircraft_dist = most_urgent_conflict.aircraft_dist;

	_detect_and_avoid_most_urgent_pub.publish(daa_status);
}

void DetectAndAvoid::update_most_urgent_conflict()
{
	// Every call follows a buffer mutation, so the published status must be
	// refreshed even if the most urgent conflict itself is unchanged.
	_most_urgent_conflict_changed = true;

#if defined(DEBUG_BUILD)
	debug_print_buffer_status();
#endif

	_conflict_tracker.refresh_most_urgent();
}

bool DetectAndAvoid::eval_conflict_escalation_action(const DaaAction requested_action) const
{
	if (requested_action <= DaaAction::kWarnOnly) {
		PX4_DEBUG("DAA: Escalation to Warn, no action published");
		return false;
	}

	DaaAction current_nav_state = nav_state_to_equivalent_daa_action(_navigator->get_vstatus()->nav_state);

	// Only publish command if requested command is more critical than current navigator state.
	if (requested_action <= current_nav_state) {
		PX4_DEBUG("DAA: Requested action: %d less critical than current nav state: %d, no action published",
			  (int)requested_action,
			  (int)_navigator->get_vstatus()->nav_state);
		return false;
	}

	return true;
}

void DetectAndAvoid::evaluate_and_publish_action()
{
	const conflict_info_s &most_urgent_conflict = _conflict_tracker.most_urgent();

	if (most_urgent_conflict.conflict_level == _prev_most_urgent_conflict_level) {
		return; // No change in conflict level, early return
	}

	const bool conflict_escalated = most_urgent_conflict.conflict_level > _prev_most_urgent_conflict_level;

	const DaaAction requested_action = get_action_from_conflict_level(most_urgent_conflict.conflict_level);
	PX4_DEBUG("DAA: Conflict %s, attempt to publish action %d",  conflict_escalated ? "escalation" : "reduction",
		  (int)requested_action);

	if (_navigator->get_land_detected()->landed) {

		if (requested_action <= DaaAction::kWarnOnly) {
			PX4_DEBUG("DAA: drone landed and no action required. No action sent");
			return;
		}

		const NotifyLandedActCause cause =
			(_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED)
			? NotifyLandedActCause::kConflictAndArmed
			: NotifyLandedActCause::kConflictAndDisarmed;

		if (must_notify(most_urgent_conflict.conflict_level, _time_last_landed_warning,
				static_cast<hrt_abstime>(math::max<int32_t>(0, _param_daa_notif_state.get())) * 1_s,
				most_urgent_conflict.conflict_level)) {
			notify_action_on_ground(cause);
			_time_last_landed_warning = hrt_absolute_time();
		}

		return;
	}

	if (conflict_escalated && eval_conflict_escalation_action(requested_action)) {
		// Publish even if _previous_action == requested_action: it may not match vehicle_status.
		PX4_DEBUG("DAA: publish action %d", (int)requested_action);
		publish_action_command(requested_action);

		// Avoid spamming if the vehicle command does not go through.
		if (_previous_action != requested_action) {
			notify_new_action(most_urgent_conflict, requested_action);
		}

		_previous_action = requested_action;
		return;
	}

	if (!conflict_escalated) {

		PX4_DEBUG("DAA: De-escalation, prev act %d, nav state %d, ", static_cast<int>(_previous_action),
			  _navigator->get_vstatus()->nav_state);
	}
}

void DetectAndAvoid::notify_if_needed(const new_conflicts_pending_notif_s &new_conflicts_pending_notif)
{
	const conflict_info_s &most_urgent_conflict = _conflict_tracker.most_urgent();

	const bool main_conflict_pending_notif =
		pending_new_conflict_notification_exists(most_urgent_conflict.encoded_id, new_conflicts_pending_notif);

	// Necessary to avoid a double notification when a new conflict is also the main conflict
	if (main_conflict_pending_notif) {
		notify_conflict_level(most_urgent_conflict, _prev_most_urgent_conflict_level, ConflictNotifyKind::kMostUrgentNew);
		_time_last_status_notif = hrt_absolute_time();
	}

	for (const DaaEncodedId &new_conflict_id : new_conflicts_pending_notif) {
		if (main_conflict_pending_notif && new_conflict_id == most_urgent_conflict.encoded_id) {
			continue;
		}

		conflict_info_s new_conflict{};

		// No need to notify new conflicts that are not in the buffer anymore
		if (!_conflict_tracker.get_conflict(new_conflict_id, new_conflict)) {
			continue;
		}

		notify_new_conflict(new_conflict);
	}

	if (main_conflict_pending_notif) {
		return;
	}

	// Always notify if the most urgent conflict level has changed.
	// Note that the conflict level can de-escalate but the action will not change.
	if (must_notify(most_urgent_conflict.conflict_level, _time_last_status_notif,
			static_cast<hrt_abstime>(math::max<int32_t>(0, _param_daa_notif_state.get())) * 1_s,
			_prev_most_urgent_conflict_level)) {
		notify_conflict_level(most_urgent_conflict, _prev_most_urgent_conflict_level, ConflictNotifyKind::kMostUrgent);

		_time_last_status_notif = hrt_absolute_time();
	}
}

bool DetectAndAvoid::is_self_detection(const DaaEncodedId &encoded_id) const
{
	switch (encoded_id.encoding) {
	case detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO: {
			const int32_t own_icao = _vehicle_adsb_icao.get();

			if (own_icao >= 0 && static_cast<uint32_t>(encoded_id.id) == static_cast<uint32_t>(own_icao)) {
				PX4_DEBUG("DAA: Received own main ICAO.");
				return true;
			}

			const int32_t own_icao_2 = _vehicle_adsb_icao_2.get();

			if (own_icao_2 >= 0 && static_cast<uint32_t>(encoded_id.id) == static_cast<uint32_t>(own_icao_2)) {
				PX4_DEBUG("DAA: Received own secondary ICAO.");
				return true;
			}

			break;
		}

	case detect_and_avoid_s::UNIQUE_ID_ENCODING_ADSB_CALLSIGN: {

			// Extract the lower and upper 32 bits (first and last 4 characters respectively)
			const uint32_t lower = static_cast<uint32_t>(encoded_id.id & 0xFFFFFFFF);
			const uint32_t upper = static_cast<uint32_t>((encoded_id.id >> 32) & 0xFFFFFFFF);

			if (lower == static_cast<uint32_t>(_vehicle_adsb_callsign_1.get()) &&
			    upper == static_cast<uint32_t>(_vehicle_adsb_callsign_2.get())) {
				PX4_DEBUG("DAA: Received own Callsign.");
				return true;
			}

			break;
		}

	case detect_and_avoid_s::UNIQUE_ID_ENCODING_UAS_ID: {
#ifndef BOARD_HAS_NO_UUID
			px4_guid_t px4_guid {};

			if (board_get_px4_guid(px4_guid) != PX4_GUID_BYTE_LENGTH) {
				PX4_DEBUG("DAA: Failed to get own UAS ID.");
				return false;
			}

			if (encoded_id.id == DaaEncodedId::last_uas_id_bytes_to_uint64(px4_guid)) {
				PX4_DEBUG("DAA: Received own UAS ID.");
				return true;
			}

#endif

			break;
		}

	default:
		break;
	}

	return false;
}

daa_input_s DetectAndAvoid::prepare_daa_input(const transponder_report_s &transponder_report)
{
	daa_input_s daa_input{};

	// Process uav pose
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();
	daa_input.uav_lat_lon = matrix::Vector2d(global_position.lat, global_position.lon);
	daa_input.uav_alt = global_position.alt;
	const vehicle_local_position_s &local_position = *_navigator->get_local_position();
	daa_input.uav_vel_ned = matrix::Vector3f(local_position.vx, local_position.vy, local_position.vz);

	// Set infinite velocities to zero to at least detect the fixed-size boundaries in the F3442
	for (int i = 0; i < 3; ++i) {
		if (!PX4_ISFINITE(daa_input.uav_vel_ned(i))) {
			daa_input.uav_vel_ned(i) = 0.f;
		}
	}

	static constexpr float kMinGroundSpeedForCourseHeading{0.5f};
	const float uav_horizontal_speed = daa_input.uav_vel_ned.xy().norm();
	daa_input.uav_heading = (uav_horizontal_speed < kMinGroundSpeedForCourseHeading) ?
				local_position.heading : atan2f(daa_input.uav_vel_ned(1), daa_input.uav_vel_ned(0));

	daa_input.transponder_report = transponder_report;

	// Default to zero velocity if vel not valid PX4_ADSB_FLAGS_VALID_VELOCITY
	if (!(daa_input.transponder_report.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY)) {
		daa_input.transponder_report.ver_velocity = 0.f;
		daa_input.transponder_report.hor_velocity = 0.f;
	}

	// Over-write transponder vertical velocity if requested
#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442

	if (_param_daa_en_dflt_vel.get()) {
		// If velocity is zero, assume positive velocity i.e. aircraft going up
		const float velocity_sign = (daa_input.transponder_report.ver_velocity >= 0) ? 1.f : -1.f;
		daa_input.transponder_report.ver_velocity = velocity_sign * _param_daa_dflt_vel.get();
	}

#endif // CONFIG_NAVIGATOR_ADSB_F3442

	return daa_input;
}

bool DetectAndAvoid::uav_pose_valid_and_updated() const
{
	const vehicle_global_position_s &global_position = *_navigator->get_global_position();

	if (!PX4_ISFINITE(global_position.lat) || !PX4_ISFINITE(global_position.lon) || !PX4_ISFINITE(global_position.alt)) {
		PX4_DEBUG("DAA: invalid global pose");
		return false;
	}

	if (global_position.timestamp == 0 || hrt_elapsed_time(&global_position.timestamp) > kRemoveStaleConflictsTime) {
		PX4_DEBUG("DAA: stale global pose");
		return false;
	}

	const hrt_abstime local_position_timestamp = _navigator->get_local_position()->timestamp;

	if (local_position_timestamp == 0 || hrt_elapsed_time(&local_position_timestamp) > kRemoveStaleConflictsTime) {
		PX4_DEBUG("DAA: stale local position");
		return false;
	}

	return true;
}

bool DetectAndAvoid::transponder_data_valid(const transponder_report_s &transponder_report) const
{
	if (!PX4_ISFINITE(transponder_report.lat) || !PX4_ISFINITE(transponder_report.lon)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid lat/lon.");
		return false;
	}

	if (!PX4_ISFINITE(transponder_report.altitude)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid altitude.");
		return false;
	}

	uint16_t required_flags = transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS |
				  transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442
	required_flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING |
			  transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	if ((transponder_report.flags & required_flags) != required_flags) {
		PX4_DEBUG("DAA: transponder data rejected, missing flags.");
		return false;
	}

#if !defined(CONFIG_NAVIGATOR_ADSB_F3442) || !CONFIG_NAVIGATOR_ADSB_F3442

	if (!PX4_ISFINITE(transponder_report.heading)) {
		PX4_DEBUG("DAA: transponder data rejected, invalid heading.");
		return false;
	}

#endif // !CONFIG_NAVIGATOR_ADSB_F3442

	const hrt_abstime timeout_us = static_cast<hrt_abstime>(_param_daa_traff_tout.get()) * 1_s;

	if (transponder_report.timestamp == 0 || hrt_elapsed_time(&transponder_report.timestamp) > timeout_us) {
		PX4_DEBUG("DAA: transponder data rejected, too old.");
		return false;
	}

	return true;
}

bool DetectAndAvoid::pending_new_conflict_notification_exists(const DaaEncodedId &target_encoded_id,
		const new_conflicts_pending_notif_s &new_conflicts_pending_notif)
{
	for (size_t i = 0; i < new_conflicts_pending_notif.size(); ++i) {
		if (new_conflicts_pending_notif[i] == target_encoded_id) {
			return true;
		}
	}

	return false;
}

// Helper function for notifying about ignored traffic if necessary
void DetectAndAvoid::maybe_notify_ignored_traffic(const conflict_info_s &conflict, const IgnoreTrafficCause cause)
{
	static constexpr uint64_t kIgnoredTrafficNotifTime{2_s};

	if (must_notify(conflict.conflict_level, _time_last_traffic_ignored, kIgnoredTrafficNotifTime,
			conflict.conflict_level)) {
		notify_traffic_ignored(conflict, cause);
	}
}

DaaAction DetectAndAvoid::get_action_from_conflict_level(const uint8_t conflict_level) const
{
	if (conflict_level < detect_and_avoid_s::DAA_CONFLICT_LVL_LOW
	    || conflict_level > detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL) {
		return DaaAction::kDisabled;
	}

#if defined(CONFIG_NAVIGATOR_ADSB_F3442) && CONFIG_NAVIGATOR_ADSB_F3442
	// F3442 zones are nested from CRITICAL to LOW. If the action for the
	// breached zone is disabled, fall back to the next larger breached zone.
	DaaAction requested_action{DaaAction::kDisabled};

	switch (conflict_level) {
	case detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL:
		requested_action = action_param_to_daa_action(_param_daa_lvl_crit_act.get());

		if (requested_action != DaaAction::kDisabled) {
			return requested_action;
		}

	// FALLTHROUGH

	case detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH:
		requested_action = action_param_to_daa_action(_param_daa_lvl_high_act.get());

		if (requested_action != DaaAction::kDisabled) {
			return requested_action;
		}

	// FALLTHROUGH

	case detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM:
		requested_action = action_param_to_daa_action(_param_daa_lvl_med_act.get());

		if (requested_action != DaaAction::kDisabled) {
			return requested_action;
		}

	// FALLTHROUGH

	case detect_and_avoid_s::DAA_CONFLICT_LVL_LOW:
		return action_param_to_daa_action(_param_daa_lvl_low_act.get());
	}

	return DaaAction::kDisabled;
#else
	return action_param_to_daa_action(_param_nav_traff_avoid.get());
#endif // CONFIG_NAVIGATOR_ADSB_F3442
}

DaaAction DetectAndAvoid::action_param_to_daa_action(const int32_t action_param)
{
	switch (action_param) {
	case 0:
		return DaaAction::kDisabled;

	case 1:
		return DaaAction::kWarnOnly;

	case 2:
		return DaaAction::kReturnMode;

	case 3:
		return DaaAction::kLandMode;

	case 4:
		return DaaAction::kPositionHoldMode;

	case 5:
		return DaaAction::kTerminate;

	default:
		return DaaAction::kDisabled;
	}
}

uint8_t DetectAndAvoid::daa_action_to_action_param(const DaaAction action)
{
	// Inverse of action_param_to_daa_action. Operator messages report actions with the same
	// numbering as the NAV_TRAFF_AVOID / DAA_LVL_*_ACT parameters, not the internal severity ladder.
	switch (action) {
	case DaaAction::kDisabled:
		return 0;

	case DaaAction::kWarnOnly:
		return 1;

	case DaaAction::kReturnMode:
		return 2;

	case DaaAction::kLandMode:
		return 3;

	case DaaAction::kPositionHoldMode:
		return 4;

	case DaaAction::kTerminate:
		return 5;

	default:
		return 0;
	}
}

void DetectAndAvoid::publish_action_command(const DaaAction requested_action)
{
	vehicle_command_s vcmd = {};

	switch (requested_action) {
	case DaaAction::kDisabled:
	case DaaAction::kWarnOnly: {
			// No action early return
			return;
		}

	case DaaAction::kPositionHoldMode: {
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
			vcmd.param1 = 1;
			vcmd.param2 = PX4_CUSTOM_MAIN_MODE_AUTO;
			vcmd.param3 = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
			break;
		}

	case DaaAction::kReturnMode: {
			_navigator->get_rtl()->set_return_alt_min(true);
			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_RETURN_TO_LAUNCH;
			break;
		}

	case DaaAction::kLandMode: {
			vcmd.command = vehicle_command_s::VEHICLE_CMD_NAV_LAND;
			break;
		}

	case DaaAction::kTerminate: {
			vcmd.param1 = 1;
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;
			break;
		}

	default:
		break;
	}

	_navigator->publish_vehicle_command(vcmd);
}

DaaAction DetectAndAvoid::nav_state_to_equivalent_daa_action(const uint8_t nav_state) const
{
	switch (nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_VTOL_TAKEOFF: {
			return DaaAction::kDisabled;
		}

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER: {
			return DaaAction::kPositionHoldMode;
		}

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL: {
			return DaaAction::kReturnMode;
		}

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:

	// manual modes overwritten by terminate
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
	case vehicle_status_s::NAVIGATION_STATE_ACRO:
	case vehicle_status_s::NAVIGATION_STATE_STAB: {
			return DaaAction::kLandMode;
		}

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION: {
			return DaaAction::kTerminate;
		}

	// OFFBOARD special handling, responsibility is given to offboard
	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD: {
			return DaaAction::kMaxActionValue;
		}

	default:
		break;
	}

	// Unknown states are treated as highest priority so DAA will not override them.
	return DaaAction::kMaxActionValue;
}
