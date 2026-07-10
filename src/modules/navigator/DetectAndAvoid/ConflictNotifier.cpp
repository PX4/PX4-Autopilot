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
 * @file ConflictNotifier.cpp
 *
 * All DAA operator messaging (MAVLink STATUSTEXT and PX4 events) in one place.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include "ConflictNotifier.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

using namespace time_literals;

uint32_t ConflictNotifier::distance_meters_for_log(const float distance)
{
	if (!PX4_ISFINITE(distance)) {
		return UINT32_MAX;
	}

	const double absolute_distance = fabs(static_cast<double>(distance));
	return absolute_distance >= static_cast<double>(UINT32_MAX)
	       ? UINT32_MAX
	       : static_cast<uint32_t>(absolute_distance);
}

void ConflictNotifier::reset()
{
	_time_last_status_notif = 0;
	_time_last_traffic_ignored = 0;
	_time_last_landed_warning = 0;
}

void ConflictNotifier::report_cycle(const conflict_tracker_changes_s &changes, const ConflictTracker &tracker,
				    const cycle_context_s &context)
{
	// Defer new-conflict messages until the final tracker state for this cycle is known
	// so a conflict added and removed in the same cycle is never announced
	static new_conflicts_pending_notif_s new_conflicts_pending_notif{};
	new_conflicts_pending_notif.clear();

	for (size_t i = 0; i < changes.size(); ++i) {
		const conflict_tracker_change_s &change = changes[i];

		switch (change.type) {
		case ConflictTrackerChangeType::kConflictAdded:
			if (level_requires_warning(context.warning_levels_mask, change.conflict.conflict_level)
			    && !pending_new_conflict_notification_exists(change.conflict.encoded_id, new_conflicts_pending_notif)
			    && !new_conflicts_pending_notif.push_back(change.conflict.encoded_id)) {
				PX4_ERR("DAA: pending notifications overflow");
			}

			break;

		case ConflictTrackerChangeType::kConflictLevelChanged:
			if (!pending_new_conflict_notification_exists(change.conflict.encoded_id, new_conflicts_pending_notif)) {
				maybe_notify_secondary_level_change(change, context.warning_levels_mask);
			}

			break;

		case ConflictTrackerChangeType::kConflictRemoved:

			// Do not notify traffic removed if the traffic was never visible to the user
			if (level_requires_warning(context.warning_levels_mask, change.conflict.conflict_level)
			    && !pending_new_conflict_notification_exists(change.conflict.encoded_id, new_conflicts_pending_notif)) {
				notify_traffic_removed(change.conflict, change.remove_cause);
			}

			break;

		case ConflictTrackerChangeType::kReportIgnored:
			maybe_notify_ignored_traffic(change.conflict, change.ignore_cause, context.warning_levels_mask);
			break;
		}
	}

	const conflict_info_s &most_urgent_conflict = tracker.most_urgent();

	const bool main_conflict_pending_notif =
		pending_new_conflict_notification_exists(most_urgent_conflict.encoded_id, new_conflicts_pending_notif);

	// Necessary to avoid a double notification when a new conflict is also the main conflict
	if (main_conflict_pending_notif) {
		notify_conflict_level(most_urgent_conflict, context.prev_most_urgent_level, ConflictNotifyKind::kMostUrgentNew);
		_time_last_status_notif = hrt_absolute_time();
	}

	for (const DaaEncodedId &new_conflict_id : new_conflicts_pending_notif) {
		if (main_conflict_pending_notif && new_conflict_id == most_urgent_conflict.encoded_id) {
			continue;
		}

		// No need to notify new conflicts that are not in the buffer anymore
		const conflict_info_s new_conflict = tracker.get_conflict(new_conflict_id);

		if (new_conflict.encoded_id.id == 0) {
			continue;
		}

		notify_new_conflict(new_conflict);
	}

	if (main_conflict_pending_notif) {
		return;
	}

	if (must_notify(most_urgent_conflict.conflict_level, _time_last_status_notif, context.status_notif_interval,
			context.prev_most_urgent_level, context.warning_levels_mask)) {
		notify_conflict_level(most_urgent_conflict, context.prev_most_urgent_level, ConflictNotifyKind::kMostUrgent);

		_time_last_status_notif = hrt_absolute_time();
	}
}

bool ConflictNotifier::must_notify(const uint8_t current_conflict_level, const hrt_abstime time_last_notified,
				   const hrt_abstime interval, const uint8_t previous_conflict_level,
				   const uint8_t warning_levels_mask)
{
	const bool current_lvl_requires_warning = level_requires_warning(warning_levels_mask, current_conflict_level);
	const bool time_to_notify = (time_last_notified == 0)
				    || (interval > 0 && (hrt_absolute_time() - time_last_notified) > interval);

	// Force notification if conflict level changed and at least one of the levels requires notifications.
	const bool prev_action_requires_warning = previous_conflict_level != 0
			&& level_requires_warning(warning_levels_mask, previous_conflict_level);
	const bool force_notification = (previous_conflict_level != current_conflict_level) &&
					(prev_action_requires_warning || current_lvl_requires_warning);

	return force_notification || (current_lvl_requires_warning && time_to_notify);
}

void ConflictNotifier::maybe_notify_secondary_level_change(const conflict_tracker_change_s &change,
		const uint8_t warning_levels_mask)
{
	const bool level_change_requires_notification = level_requires_warning(warning_levels_mask, change.previous_level)
			|| level_requires_warning(warning_levels_mask, change.conflict.conflict_level);

	// Known traffic, only notify if level changed. If most urgent, no need to notify, will be done in DAA status.
	if (!level_change_requires_notification || change.conflict_is_most_urgent) {
		return;
	}

	notify_conflict_level(change.conflict, change.previous_level, ConflictNotifyKind::kSecondary);
}

void ConflictNotifier::maybe_notify_ignored_traffic(const conflict_info_s &conflict, const IgnoreTrafficCause cause,
		const uint8_t warning_levels_mask)
{
	static constexpr uint64_t kIgnoredTrafficNotifTime{2_s};

	if (must_notify(conflict.conflict_level, _time_last_traffic_ignored, kIgnoredTrafficNotifTime,
			conflict.conflict_level, warning_levels_mask)) {
		notify_traffic_ignored(conflict, cause);
	}
}

void ConflictNotifier::maybe_notify_action_on_ground(const NotifyLandedActCause cause, const uint8_t conflict_level,
		const cycle_context_s &context)
{
	if (must_notify(conflict_level, _time_last_landed_warning, context.status_notif_interval,
			conflict_level, context.warning_levels_mask)) {
		notify_action_on_ground(cause);
		_time_last_landed_warning = hrt_absolute_time();
	}
}

void ConflictNotifier::notify_traffic_ignored(const conflict_info_s &conflict_info, const IgnoreTrafficCause cause)
{
	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	_time_last_traffic_ignored = hrt_absolute_time();

	mavlink_log_info(&_mavlink_log_pub, "DAA %s ignored (%u) lvl %u.\t",
			 encoded_id_str,
			 static_cast<unsigned>(cause),
			 static_cast<unsigned>(conflict_info.conflict_level));
	/* EVENT
	 * @description
	 * - ID: {1}
	 * - ID encoding: {2} (0:ICAO, 1:callsign, 2:UAS ID)
	 * - cause: {3} (0:buffer full)
	 * - conflict level: {4}
	 */
	events::send<uint64_t, uint8_t, uint8_t, uint8_t>(events::ID("navigator_traffic_ignore"), events::Log::Warning,
			"DAA: ignored",
			conflict_info.encoded_id.id, conflict_info.encoded_id.encoding,
			static_cast<uint8_t>(cause), conflict_info.conflict_level);
}

void ConflictNotifier::notify_traffic_removed(const conflict_info_s &conflict_info, const RemoveBufferCause cause)
{
	const uint32_t time_since_last_comm = static_cast<uint32_t>((hrt_absolute_time() -
					      conflict_info.latest_update_timestamp) / 1_s);

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	mavlink_log_warning(&_mavlink_log_pub, "DAA %s out (%u) lvl %u (%us).\t",
			    encoded_id_str,
			    static_cast<unsigned>(cause),
			    static_cast<unsigned>(conflict_info.conflict_level),
			    static_cast<unsigned>(time_since_last_comm));
	/* EVENT
	 * @description
	 * - ID: {1}
	 * - ID encoding: {2} (0:ICAO, 1:callsign, 2:UAS ID)
	 * - cause: {3} (0:stale, 1:buffer full)
	 * - conflict level: {4}
	 * - last seen: {5} s ago
	 */
	events::send<uint64_t, uint8_t, uint8_t, uint8_t, uint32_t>(events::ID("navigator_traffic_remove"),
			events::Log::Warning,
			"DAA: removed",
			conflict_info.encoded_id.id, conflict_info.encoded_id.encoding,
			static_cast<uint8_t>(cause), conflict_info.conflict_level, time_since_last_comm);
}

bool ConflictNotifier::mavlink_log_conflict_by_level(const uint8_t conflict_level, const char *message,
		events::Log &log_level)
{
	switch (conflict_level) {
	case detect_and_avoid_s::DAA_CONFLICT_LVL_NONE:
		return false;

	case detect_and_avoid_s::DAA_CONFLICT_LVL_LOW:
	case detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM:
		log_level = events::Log::Warning;
		break;

	case detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH:
		log_level = events::Log::Critical;
		break;

	case detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL:
		log_level = events::Log::Emergency;
		break;

	default:
		return false;
	}

	mavlink_log_warning(&_mavlink_log_pub, "%s.\t", message);
	return true;
}

void ConflictNotifier::notify_conflict_level(const conflict_info_s &conflict_info,
		const uint8_t previous_conflict_level, const ConflictNotifyKind kind)
{
	const uint8_t conflict_level = conflict_info.conflict_level;

	if (kind == ConflictNotifyKind::kSecondary && conflict_level == previous_conflict_level) {
		return;
	}

	const uint32_t aircraft_dist = distance_meters_for_log(conflict_info.aircraft_dist);

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	const char *const prefix = (kind == ConflictNotifyKind::kSecondary) ? "DAA SEC:"
				   : (kind == ConflictNotifyKind::kMostUrgentNew) ? "DAA New and Main:" : "DAA Main:";

	events::Log log_level = events::Log::Info;

	if (conflict_level >= previous_conflict_level) {
		const bool escalation = conflict_level > previous_conflict_level;

		char message_buffer[kMaxLogMessageSize];
		snprintf(message_buffer, sizeof(message_buffer), escalation ? "%s %s lvl UP %u. %u m" : "%s %s lvl %u. %u m",
			 prefix, encoded_id_str, static_cast<unsigned>(conflict_level), static_cast<unsigned>(aircraft_dist));

		if (escalation || kind == ConflictNotifyKind::kMostUrgentNew) {
			if (!mavlink_log_conflict_by_level(conflict_level, message_buffer, log_level)) {
				return;
			}

		} else {
			mavlink_log_info(&_mavlink_log_pub, "%s.\t", message_buffer);
		}

	} else if (conflict_level == detect_and_avoid_s::DAA_CONFLICT_LVL_NONE) {
		if (kind == ConflictNotifyKind::kSecondary) {
			mavlink_log_info(&_mavlink_log_pub, "%s %s solved. %u m.\t", prefix, encoded_id_str,
					 static_cast<unsigned>(aircraft_dist));

		} else {
			mavlink_log_info(&_mavlink_log_pub, "DAA all conflicts solved.\t");
		}

	} else {
		mavlink_log_info(&_mavlink_log_pub, "%s %s lvl DOWN %u. %u m.\t", prefix, encoded_id_str,
				 static_cast<unsigned>(conflict_level), static_cast<unsigned>(aircraft_dist));
	}

	// A single structured event covers every transition: the current and previous levels make the
	// direction (escalation, reduction, solved) explicit, and the kind distinguishes the most-urgent
	// conflict from secondary traffic without a separate event ID per case.
	/* EVENT
	 * @description
	 * - ID: {1}
	 * - ID encoding: {2} (0:ICAO, 1:callsign, 2:UAS ID)
	 * - conflict level: {3}
	 * - previous level: {4}
	 * - distance: {5m}
	 * - kind: {6} (0:main, 1:new+main, 2:secondary)
	 */
	events::send<uint64_t, uint8_t, uint8_t, uint8_t, uint32_t, uint8_t>(events::ID("navigator_traffic_conflict_update"),
			log_level, "DAA conflict update",
			conflict_info.encoded_id.id, conflict_info.encoded_id.encoding,
			conflict_level, previous_conflict_level, aircraft_dist,
			static_cast<uint8_t>(kind));
}

void ConflictNotifier::notify_new_conflict(const conflict_info_s &conflict_info)
{
	const uint32_t aircraft_dist = distance_meters_for_log(conflict_info.aircraft_dist);
	const uint8_t conflict_level = conflict_info.conflict_level;

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	char message_buffer[kMaxLogMessageSize];
	snprintf(message_buffer, sizeof(message_buffer), "DAA New %s lvl %u. %u m",
		 encoded_id_str, static_cast<unsigned>(conflict_level), static_cast<unsigned>(aircraft_dist));

	events::Log log_level = events::Log::Warning;

	if (mavlink_log_conflict_by_level(conflict_level, message_buffer, log_level)) {

		/* EVENT
		 * @description
		 * - ID: {1}
		 * - ID encoding: {2} (0:ICAO, 1:callsign, 2:UAS ID)
		 * - conflict level: {3}
		 * - distance: {4m}
		 */
		events::send<uint64_t, uint8_t, uint8_t, uint32_t>(events::ID("navigator_new_traffic"),
				log_level,
				"New traffic",
				conflict_info.encoded_id.id, conflict_info.encoded_id.encoding,
				conflict_info.conflict_level, aircraft_dist);
	}
}

void ConflictNotifier::notify_action_on_ground(const NotifyLandedActCause cause)
{
	const char *blocked_action = nullptr;
	events::Log log_level{events::Log::Warning};

	switch (cause) {
	case NotifyLandedActCause::kConflictAndArmed:
		blocked_action = "takeoff";
		log_level = events::Log::Critical;
		break;

	case NotifyLandedActCause::kConflictAndDisarmed:
		blocked_action = "arm";
		log_level = events::Log::Warning;
		break;

	default:
		PX4_DEBUG("DAA: invalid landed cause");
		return;
	}

	char message_buffer[kMaxLogMessageSize];
	snprintf(message_buffer, sizeof(message_buffer), "DAA do not %s until air conflict solved!", blocked_action);

	if (cause == NotifyLandedActCause::kConflictAndArmed) {
		mavlink_log_critical(&_mavlink_log_pub, "%s.\t", message_buffer);

	} else {
		mavlink_log_warning(&_mavlink_log_pub, "%s.\t", message_buffer);
	}

	events::send(events::ID("navigator_traffic_ground_conflict"), log_level,
		     "DAA: resolve air-traffic conflict before flight");
}

void ConflictNotifier::notify_new_action(const conflict_info_s &conflict_info, const DaaAction action)
{
	const uint32_t aircraft_dist = distance_meters_for_log(conflict_info.aircraft_dist);
	const uint8_t conflict_level = conflict_info.conflict_level;

	const char *action_name = nullptr;
	events::Log log_level = events::Log::Warning;

	switch (action) {
	case DaaAction::kPositionHoldMode:
		action_name = "Hold";
		break;

	case DaaAction::kReturnMode:
		action_name = "Return";
		log_level = events::Log::Critical;
		break;

	case DaaAction::kLandMode:
		action_name = "Land";
		log_level = events::Log::Critical;
		break;

	case DaaAction::kTerminate:
		action_name = "Terminate";
		log_level = events::Log::Emergency;
		break;

	case DaaAction::kDisabled:
	case DaaAction::kWarnOnly:
	default:
		return;
	}

	char encoded_id_str[kUtmGuidMsgLength];
	conflict_info.encoded_id.to_string(encoded_id_str, sizeof(encoded_id_str));

	char message_buffer[kMaxLogMessageSize];
	snprintf(message_buffer, sizeof(message_buffer), "DAA %s: %s! lvl %u. %u m",
		 encoded_id_str, action_name, static_cast<unsigned>(conflict_level), static_cast<unsigned>(aircraft_dist));

	if (action == DaaAction::kTerminate) {
		mavlink_log_emergency(&_mavlink_log_pub, "%s.\t", message_buffer);

	} else {
		mavlink_log_warning(&_mavlink_log_pub, "%s.\t", message_buffer);
	}

	/* EVENT
	 * @description
	 * - ID: {1}
	 * - ID encoding: {2} (0:ICAO, 1:callsign, 2:UAS ID)
	 * - action: {3} (2:return, 3:land, 4:hold, 5:terminate)
	 * - conflict level: {4}
	 * - distance: {5m}
	 */
	events::send<uint64_t, uint8_t, uint8_t, uint8_t, uint32_t>(events::ID("navigator_traffic_action"), log_level,
			"DAA automated action",
			conflict_info.encoded_id.id, conflict_info.encoded_id.encoding,
			DaaActionPolicy::daa_action_to_action_param(action), conflict_level, aircraft_dist);
}


bool ConflictNotifier::pending_new_conflict_notification_exists(const DaaEncodedId &target_encoded_id,
		const new_conflicts_pending_notif_s &new_conflicts_pending_notif)
{
	for (size_t i = 0; i < new_conflicts_pending_notif.size(); ++i) {
		if (new_conflicts_pending_notif[i] == target_encoded_id) {
			return true;
		}
	}

	return false;
}
