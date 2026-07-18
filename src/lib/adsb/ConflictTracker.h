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
 * @file ConflictTracker.h
 *
 * Fixed-size buffer of active DAA conflicts.
 *
 * Buffer changes that may require a user-facing message are reported as
 * conflict_tracker_change_s entries so the caller can apply its own
 * notification policy.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include <containers/Array.hpp>
#include <drivers/drv_hrt.h>
#include <uORB/topics/detect_and_avoid.h>
#include <uORB/topics/transponder_report.h>

#include "DaaEncodedId.h"

static constexpr uint8_t kDaaMaxTraffic{5};

struct conflict_info_s {
	DaaEncodedId encoded_id{};
	hrt_abstime latest_update_timestamp{0};
	uint8_t conflict_level{detect_and_avoid_s::DAA_CONFLICT_LVL_NONE};
	float aircraft_dist{0.f}; // Distance to aircraft = sqrtf(dist_hor^2 + dist_vert^2)
};

enum class RemoveBufferCause : uint8_t {
	kStaleConflict = 0,
	kBufferFull = 1
};

enum class IgnoreTrafficCause : uint8_t {
	kBufferFull = 0
};

enum class ConflictTrackerChangeType : uint8_t {
	kConflictAdded = 0,
	kConflictLevelChanged = 1,
	kConflictRemoved = 2,
	kReportIgnored = 3
};

struct conflict_tracker_change_s {
	ConflictTrackerChangeType type{ConflictTrackerChangeType::kConflictAdded};
	conflict_info_s conflict{};	// the affected entry (for kConflictRemoved: the removed entry)
	uint8_t previous_level{detect_and_avoid_s::DAA_CONFLICT_LVL_NONE};	// kConflictLevelChanged only
	RemoveBufferCause remove_cause{RemoveBufferCause::kStaleConflict};	// kConflictRemoved only
	IgnoreTrafficCause ignore_cause{IgnoreTrafficCause::kBufferFull};	// kReportIgnored only
	// Required because the caller processes the full Uorb queue before emitting warnings
	bool conflict_is_most_urgent{false};
};

// A stale sweep can remove the full buffer and each queued report can replace one entry.
static constexpr uint16_t kMaxConflictChangesPerCycle{kDaaMaxTraffic + 2 * transponder_report_s::ORB_QUEUE_LENGTH};

using conflict_tracker_changes_s = px4::Array<conflict_tracker_change_s, kMaxConflictChangesPerCycle>;

class ConflictTracker
{
public:
	ConflictTracker() { reset_most_urgent(); }
	~ConflictTracker() = default;

	/**
	 * @brief Apply one evaluated report to the tracked set.
	 *
	 * Inserts new conflicts (evicting the least urgent entry when the buffer is full),
	 * overwrites known conflicts, and drops known conflicts that report
	 * DAA_CONFLICT_LVL_NONE.
	 *
	 * Emitted changes are appended to @p changes. Returns true if the tracked set changed.
	 */
	bool apply_conflict(const conflict_info_s &conflict, conflict_tracker_changes_s &changes);

	/**
	 * @brief Drop entries whose last update is older than @p timeout_us at time @p now.
	 *
	 * Emits one kConflictRemoved (kStaleConflict) change per dropped entry.
	 * Returns true if at least one entry was removed.
	 */
	bool remove_stale_conflicts(const hrt_abstime now, const hrt_abstime timeout_us, conflict_tracker_changes_s &changes);

	// Drop all conflicts and reset the most-urgent cache.
	void clear();

	// Recompute the cached most-urgent conflict from the buffer.
	void refresh_most_urgent();

	// Cached value; only refresh_most_urgent() and clear() update it.
	const conflict_info_s &most_urgent() const { return _most_urgent; }

	// Highest-priority conflict in the buffer, or id=0 when empty.
	conflict_info_s find_most_urgent() const;

	bool contains(const DaaEncodedId &encoded_id) const { return find_conflict_idx(encoded_id) >= 0; }

	// Tracked entry for encoded_id, or id=0 when not tracked.
	conflict_info_s get_conflict(const DaaEncodedId &encoded_id) const;

	size_t size() const { return _buffer.size(); }
	bool empty() const { return _buffer.empty(); }

	// Priority rules: higher conflict level first, ties broken by aircraft distance.
	// Public so the priority policy can be unit tested directly.
	static bool is_conflict_more_important(const conflict_info_s &new_conflict, const conflict_info_s &base_conflict);
	static bool is_conflict_less_important(const conflict_info_s &new_conflict, const conflict_info_s &base_conflict);

private:
	enum class ConflictPriority : uint8_t {
		kMostUrgent,
		kLeastUrgent
	};

	/** @brief Insert a new conflict, replacing the least urgent entry when the buffer is full. */
	bool add_conflict(const conflict_info_s &conflict, conflict_tracker_changes_s &changes);

	/**
	 * @brief Overwrite a tracked entry from a new report.
	 *
	 * If the level changed to DAA_CONFLICT_LVL_NONE, the conflict is removed form the buffer.
	 *
	 * Emits kConflictLevelChanged when the level differs from the stored entry.
	 */
	bool update_conflict(const conflict_info_s &conflict, const int conflict_idx, conflict_tracker_changes_s &changes);

	/** @brief Return the index of the most or least urgent entry, or -1 if the buffer is empty. */
	int find_conflict_idx_by_priority(const ConflictPriority priority) const;

	/** @brief Return the buffer index of @p encoded_id, or -1 if not tracked. */
	int find_conflict_idx(const DaaEncodedId &encoded_id) const;

	void reset_most_urgent();

	bool is_valid_idx(const int idx) const { return idx >= 0 && idx < static_cast<int>(_buffer.size()); }

	px4::Array<conflict_info_s, kDaaMaxTraffic> _buffer{};
	conflict_info_s _most_urgent{};
};
