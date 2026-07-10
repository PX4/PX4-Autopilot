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
 * @file ConflictNotifier.h
 *
 * All DAA operator messaging (MAVLink STATUSTEXT and PX4 events) in one place.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/adsb/ConflictTracker.h>
#include <lib/adsb/DaaActionPolicy.h>
#include <px4_platform_common/events.h>
#include <uORB/uORB.h>
#include <uORB/topics/transponder_report.h>

class ConflictNotifier
{
public:
	ConflictNotifier() = default;
	~ConflictNotifier() = default;

	struct cycle_context_s {
		uint8_t prev_most_urgent_level{0};
		uint8_t warning_levels_mask{0};
		hrt_abstime status_notif_interval{0};
	};

	/**
	 * @brief Decide and send all of this cycle's conflict messages.
	 *
	 * Processes the tracker changes in the order they were recorded, then sends the deferred "new conflict"
	 * warnings and the most-urgent status.
	 *
	 * Conflicts that were removed but have a "new" warning
	 * is still pending from the same cycle are not sent because they were never user-visible.
	 *
	 * Call exactly once per cycle; the periodic most-urgent status relies on it.
	 */
	void report_cycle(const conflict_tracker_changes_s &changes, const ConflictTracker &tracker,
			  const cycle_context_s &context);

	void notify_new_action(const conflict_info_s &conflict_info, const DaaAction action);

	void maybe_notify_action_on_ground(const NotifyLandedActCause cause, const uint8_t conflict_level,
					   const cycle_context_s &context);

	void reset();

private:
	// Selects the message prefix and structured event severity used when a tracked conflict's level changes.
	// The most-urgent conflict drives DAA actions and is reported with
	// level-based event severity; secondary conflicts stay informational.
	enum class ConflictNotifyKind : uint8_t {
		kMostUrgent = 0,	// already tracked most urgent conflict ("DAA Main:")
		kMostUrgentNew = 1,	// most urgent conflict that appeared this cycle ("DAA New and Main:")
		kSecondary = 2		// any other tracked conflict ("DAA SEC:")
	};

	using new_conflicts_pending_notif_s = px4::Array<DaaEncodedId, transponder_report_s::ORB_QUEUE_LENGTH>;

	static bool must_notify(const uint8_t current_conflict_level, const hrt_abstime time_last_notified,
				const hrt_abstime interval, const uint8_t previous_conflict_level,
				const uint8_t warning_levels_mask);

	// Announce a tracked conflict's level change, unless it was (about to become) the most urgent at
	// recording time, which the DAA status reports instead.
	void maybe_notify_secondary_level_change(const conflict_tracker_change_s &change,
			const uint8_t warning_levels_mask);

	void maybe_notify_ignored_traffic(const conflict_info_s &conflict, const IgnoreTrafficCause cause,
					  const uint8_t warning_levels_mask);

	void notify_traffic_ignored(const conflict_info_s &conflict_info, const IgnoreTrafficCause cause);
	void notify_traffic_removed(const conflict_info_s &conflict_info, const RemoveBufferCause cause);
	void notify_new_conflict(const conflict_info_s &conflict_info);

	/**
	 * @brief Notify the operator about a tracked conflict whose level changed (or a periodic
	 * update of the most-urgent conflict).
	 *
	 * Handles escalation, de-escalation and the "solved" transition for both the most urgent
	 * ("main") and secondary conflicts. @p kind selects the message prefix and whether the
	 * severity follows the conflict level.
	 *
	 * Secondary conflicts emit nothing when the level is unchanged.
	 * The most urgent conflict emits a periodic status when the level is unchanged.
	 *
	 */
	void notify_conflict_level(const conflict_info_s &conflict_info, const uint8_t previous_conflict_level,
				   const ConflictNotifyKind kind);

	void notify_action_on_ground(const NotifyLandedActCause cause);

	bool mavlink_log_conflict_by_level(const uint8_t conflict_level, const char *message, events::Log &log_level);

	// Distance in meters for operator messages, non-finite or out-of-range maps to UINT32_MAX.
	static uint32_t distance_meters_for_log(const float distance);

	static constexpr size_t kMaxLogMessageSize{128};

	static bool level_requires_warning(const uint8_t warning_levels_mask, const uint8_t conflict_level)
	{
		return conflict_level < 8 && (warning_levels_mask & (1u << conflict_level));
	}

	static bool pending_new_conflict_notification_exists(const DaaEncodedId &target_encoded_id,
			const new_conflicts_pending_notif_s &new_conflicts_pending_notif);

	orb_advert_t _mavlink_log_pub{nullptr};

	/* Timers to avoid spamming */
	hrt_abstime _time_last_status_notif{0};
	hrt_abstime _time_last_traffic_ignored{0};
	hrt_abstime _time_last_landed_warning{0};
};
