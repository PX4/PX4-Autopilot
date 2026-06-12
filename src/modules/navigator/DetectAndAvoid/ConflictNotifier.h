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
 * The notifier owns the message formatting, the severity, and the message period.
 * Conflict messages are decided once per cycle in report_cycle() from the tracker's change records.
 * Action messages are requested by the action evaluator but formatted and throttled here.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/adsb/ConflictTracker.h>
#include <lib/adsb/DaaActionPolicy.h>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/transponder_report.h>

static constexpr uint8_t kMaxLogMsgSize{128};

// Worst case changes in one cycle: a full stale sweep plus two changes per drained report.
static constexpr uint8_t kMaxCycleChanges{kDaaMaxTraffic + 2 * transponder_report_s::ORB_QUEUE_LENGTH};

// All tracker changes collected over one cycle, handed to the notifier in a single call.
using conflict_cycle_changes_s = px4::Array<conflict_tracker_change_s, kMaxCycleChanges>;

class ConflictNotifier
{
public:
	ConflictNotifier() = default;
	~ConflictNotifier() = default;

	// Per-cycle constants the message policy depends on, derived by the module.
	struct cycle_context_s {
		uint8_t prev_most_urgent_level{0};
		// Bit i set = conflict level i requires an operator warning (from the action params).
		uint8_t warning_levels_mask{0};
		// Period of the most-urgent status and of the landed warning (DAA_NOTIF_STATE).
		hrt_abstime status_notif_interval{0};
	};

	/**
	 * @brief Decide and send all of this cycle's conflict messages.
	 *
	 * Processes the tracker changes in the order they were recorded (secondary level
	 * changes, removals, ignored reports), then sends the deferred "new conflict"
	 * warnings and the most-urgent status. Conflicts that were removed but have a "new" warning
	 * is still pending from the same cycle are not sent because they were never user-visible.
	 *
	 * Call exactly once per cycle; the periodic most-urgent status relies on it.
	 */
	void report_cycle(const conflict_cycle_changes_s &changes, const ConflictTracker &tracker,
			  const cycle_context_s &context);

	/** @brief Send the user-facing message and event for a newly-published DAA action. */
	void notify_new_action(const conflict_info_s &conflict_info, const DaaAction action);

	/**
	 * @brief Warn the operator that air traffic is present while the vehicle is on the ground.
	 *
	 * Limit notif rate to one warning per status interval.
	 */
	void maybe_notify_action_on_ground(const NotifyLandedActCause cause, const uint8_t conflict_level,
					   const cycle_context_s &context);

	/** @brief Restart all rate limit timers (called when the conflict state is cleared). */
	void reset();

private:
	// Selects the message prefix and severity used when a tracked conflict's level changes.
	// The most-urgent conflict drives DAA actions and is reported with
	// level-based severity; secondary conflicts stay informational.
	enum class ConflictNotifyKind : uint8_t {
		kMostUrgent = 0,	// already tracked most urgent conflict ("DAA Main:")
		kMostUrgentNew = 1,	// most urgent conflict that appeared this cycle ("DAA New and Main:")
		kSecondary = 2		// any other tracked conflict ("DAA SEC:")
	};

	using new_conflicts_pending_notif_s = px4::Array<DaaEncodedId, transponder_report_s::ORB_QUEUE_LENGTH>;

	/**
	 * @brief Decide whether to notify about a conflict level right now.
	 *
	 * Forces a notification on level transitions when either side requires a warning,
	 * otherwise one notification per @p interval.
	 */
	bool must_notify(const uint8_t current_conflict_level, const hrt_abstime time_last_notified,
			 const hrt_abstime interval, const uint8_t previous_conflict_level,
			 const uint8_t warning_levels_mask) const;

	/**
	 * @brief Announce a tracked conflict's level change unless it was (about to become) the most
	 * urgent conflict at recording time, which is reported through the DAA status instead.
	 */
	void maybe_notify_secondary_level_change(const conflict_tracker_change_s &change,
			const uint8_t warning_levels_mask);

	/** @brief Helper function for notifying about ignored traffic if necessary. */
	void maybe_notify_ignored_traffic(const conflict_info_s &conflict, const IgnoreTrafficCause cause,
					  const uint8_t warning_levels_mask);

	/** @brief Warn the user that a traffic report was dropped and why. */
	void notify_traffic_ignored(const conflict_info_s &conflict_info, const IgnoreTrafficCause cause);

	/** @brief Tell the user that a previous conflict was removed (stale or buffer full). */
	void notify_traffic_removed(const conflict_info_s &conflict_info, const RemoveBufferCause cause);

	/** @brief Emit the first notification for a newly added conflict. */
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

	/**
	 * @brief Send @p message_buffer over MAVLink with a severity derived from @p conflict_level.
	 *
	 * Returns false (and sends nothing) for NONE or invalid levels. Sets
	 * @p log_level so the caller can use the same severity for a matching event.
	 */
	bool mavlink_log_conflict_by_level(const uint8_t conflict_level, const char message_buffer[kMaxLogMsgSize],
					   events::Log &log_level);

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
