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
 * @file ConflictTrackerTest.cpp
 * @brief Unit tests for the ConflictTracker conflict buffer.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#include <gtest/gtest.h>

#include <limits>

#include <lib/adsb/ConflictTracker.h>

namespace
{

conflict_info_s make_conflict(const uint64_t icao_id, const uint8_t conflict_level, const float aircraft_dist,
			      const hrt_abstime timestamp = 1000000)
{
	conflict_info_s conflict{};
	conflict.encoded_id.id = icao_id;
	conflict.encoded_id.encoding = detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO;
	conflict.conflict_level = conflict_level;
	conflict.aircraft_dist = aircraft_dist;
	conflict.latest_update_timestamp = timestamp;
	return conflict;
}

} // namespace

// Accepted conflict enters the buffer and emits kConflictAdded; most_urgent updates only on refresh.
TEST(ConflictTrackerTest, AddsNewConflictAndRefreshesMostUrgent)
{
	ConflictTracker tracker;

	EXPECT_TRUE(tracker.empty());
	EXPECT_EQ(tracker.most_urgent().conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);

	const conflict_info_s conflict = make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 800.f);
	conflict_tracker_changes_s changes{};

	EXPECT_TRUE(tracker.apply_conflict(conflict, changes));
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictAdded);
	EXPECT_EQ(changes[0].conflict.encoded_id, conflict.encoded_id);

	EXPECT_EQ(tracker.size(), 1u);
	EXPECT_TRUE(tracker.contains(conflict.encoded_id));

	const conflict_info_s tracked = tracker.get_conflict(conflict.encoded_id);
	ASSERT_NE(tracked.encoded_id.id, 0u);
	EXPECT_EQ(tracked.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);

	// most_urgent stays empty until refresh
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);
	tracker.refresh_most_urgent();
	EXPECT_EQ(tracker.most_urgent().encoded_id, conflict.encoded_id);
}

// NONE-level traffic that isn't tracked never takes a buffer slot.
TEST(ConflictTrackerTest, IgnoresUntrackedTrafficWithoutConflict)
{
	ConflictTracker tracker;

	const conflict_info_s no_conflict = make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE, 5000.f);
	conflict_tracker_changes_s changes{};

	EXPECT_FALSE(tracker.apply_conflict(no_conflict, changes));
	EXPECT_EQ(changes.size(), 0u);
	EXPECT_TRUE(tracker.empty());
}

TEST(ConflictTrackerTest, RejectsInvalidConflictData)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	EXPECT_FALSE(tracker.apply_conflict(
			     make_conflict(0, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 100.f), changes));
	EXPECT_FALSE(tracker.apply_conflict(
			     make_conflict(1, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL + 1, 100.f), changes));
	EXPECT_FALSE(tracker.apply_conflict(
			     make_conflict(1, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH,
					   std::numeric_limits<float>::quiet_NaN()), changes));
	EXPECT_TRUE(tracker.empty());
	EXPECT_TRUE(changes.empty());
}

// kConflictLevelChanged is emitted only when the level differs; same-level just overwrites.
TEST(ConflictTrackerTest, EmitsLevelChangedOnlyWhenLevelDiffers)
{
	ConflictTracker tracker;

	conflict_tracker_changes_s changes{};
	tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f, 1000000), changes);

	// same level: overwrite, no change emitted
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 1900.f, 2000000),
					   changes));

	EXPECT_EQ(changes.size(), 0u);
	const conflict_info_s tracked = tracker.get_conflict(make_conflict(0x6E9F7B, 0, 0.f).encoded_id);
	ASSERT_NE(tracked.encoded_id.id, 0u);
	EXPECT_EQ(tracked.latest_update_timestamp, 2000000u);
	EXPECT_FLOAT_EQ(tracked.aircraft_dist, 1900.f);

	// escalation: one level-changed change carrying the previous level
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 600.f, 3000000),
					   changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictLevelChanged);
	EXPECT_EQ(changes[0].conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_EQ(changes[0].previous_level, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW);

	// de-escalation to a still-active level
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 900.f,
					   4000000), changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].previous_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_EQ(tracker.size(), 1u);
}

// A resolved conflict leaves the buffer and is reported as a level change to NONE, not a removal,
// so the caller can announce "solved" instead of a drop warning.
TEST(ConflictTrackerTest, ResolvedConflictLeavesBufferAsLevelChange)
{
	ConflictTracker tracker;

	conflict_tracker_changes_s changes{};
	tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 600.f), changes);
	tracker.refresh_most_urgent();

	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE, 5000.f),
					   changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictLevelChanged);
	EXPECT_EQ(changes[0].conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(changes[0].previous_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_TRUE(tracker.empty());

	// refresh resets the cache to no-conflict
	tracker.refresh_most_urgent();
	EXPECT_EQ(tracker.most_urgent().conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);
}

// conflict_is_most_urgent must be set per change, resolved from both the cache and the live buffer
// (the caller defers secondary warnings until the whole transponder queue is processed).
TEST(ConflictTrackerTest, LevelChangeCapturesMostUrgent)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	// critical (most urgent) plus a secondary low conflict
	tracker.apply_conflict(make_conflict(0xA, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f), changes);
	tracker.apply_conflict(make_conflict(0xB, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f), changes);
	tracker.refresh_most_urgent();

	// secondary escalates but stays below the most urgent: not flagged
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0xB, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 1500.f),
					   changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_FALSE(changes[0].conflict_is_most_urgent);

	// most urgent changes level: flagged via the cache
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0xA, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 400.f), changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_TRUE(changes[0].conflict_is_most_urgent);

	// a new conflict overtakes as most urgent while the cache still points at 0xA: flagged via buffer
	changes = {};
	tracker.apply_conflict(make_conflict(0xC, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 3000.f), changes);
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0xC, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 50.f),
					   changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_TRUE(changes[0].conflict_is_most_urgent);
}

// A more urgent conflict displaces the least urgent when the buffer is full, reporting the removal
// before the insertion.
TEST(ConflictTrackerTest, EvictsLeastUrgentWhenFull)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	// fill so that 0x100 is least urgent (lowest level, largest distance)
	for (uint64_t i = 0; i < kDaaMaxTraffic; ++i) {
		changes = {};
		ASSERT_TRUE(tracker.apply_conflict(make_conflict(0x100 + i, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW + (i > 0),
						   2000.f - static_cast<float>(i) * 100.f), changes));
	}

	ASSERT_EQ(tracker.size(), static_cast<size_t>(kDaaMaxTraffic));

	const conflict_info_s critical = make_conflict(0xBEEF, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f);
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(critical, changes));

	// removal first, then insertion
	ASSERT_EQ(changes.size(), 2u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictRemoved);
	EXPECT_EQ(changes[0].remove_cause, RemoveBufferCause::kBufferFull);
	EXPECT_EQ(changes[0].conflict.encoded_id.id, 0x100u);
	EXPECT_EQ(changes[1].type, ConflictTrackerChangeType::kConflictAdded);
	EXPECT_EQ(changes[1].conflict.encoded_id.id, 0xBEEFu);

	EXPECT_EQ(tracker.size(), static_cast<size_t>(kDaaMaxTraffic));
	EXPECT_FALSE(tracker.contains(make_conflict(0x100, 0, 0.f).encoded_id));

	tracker.refresh_most_urgent();
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0xBEEFu);
}

// A full buffer rejects weaker traffic and keeps its entries.
TEST(ConflictTrackerTest, RejectsLessImportantWhenFull)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	for (uint64_t i = 0; i < kDaaMaxTraffic; ++i) {
		changes = {};
		ASSERT_TRUE(tracker.apply_conflict(make_conflict(0x200 + i, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH,
						   600.f + static_cast<float>(i)), changes));
	}

	changes = {};
	EXPECT_FALSE(tracker.apply_conflict(make_conflict(0xBEEF, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f), changes));

	// reported ignored (buffer full), nothing displaced
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kReportIgnored);
	EXPECT_EQ(changes[0].ignore_cause, IgnoreTrafficCause::kBufferFull);
	EXPECT_EQ(changes[0].conflict.encoded_id.id, 0xBEEFu);
	EXPECT_EQ(tracker.size(), static_cast<size_t>(kDaaMaxTraffic));
	EXPECT_FALSE(tracker.contains(make_conflict(0xBEEF, 0, 0.f).encoded_id));
}

// Priority: level dominates, distance only breaks ties.
TEST(ConflictTrackerTest, PriorityIsLevelFirstThenDistance)
{
	const conflict_info_s far_critical = make_conflict(1, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 3000.f);
	const conflict_info_s close_low = make_conflict(2, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 10.f);
	const conflict_info_s close_critical = make_conflict(3, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f);
	const conflict_info_s same_as_close_critical =
		make_conflict(4, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f);

	// level dominates distance
	EXPECT_TRUE(ConflictTracker::is_conflict_more_important(far_critical, close_low));
	EXPECT_TRUE(ConflictTracker::is_conflict_less_important(close_low, far_critical));

	// same level: distance breaks the tie
	EXPECT_TRUE(ConflictTracker::is_conflict_more_important(close_critical, far_critical));
	EXPECT_TRUE(ConflictTracker::is_conflict_less_important(far_critical, close_critical));
	EXPECT_FALSE(ConflictTracker::is_conflict_more_important(close_critical, same_as_close_critical));
	EXPECT_FALSE(ConflictTracker::is_conflict_less_important(close_critical, same_as_close_critical));

	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	// insert out of priority order; closest critical wins
	tracker.apply_conflict(close_low, changes);
	tracker.apply_conflict(far_critical, changes);
	tracker.apply_conflict(close_critical, changes);

	const conflict_info_s most_urgent = tracker.find_most_urgent();
	ASSERT_NE(most_urgent.encoded_id.id, 0u);
	EXPECT_EQ(most_urgent.encoded_id.id, 3u);
}

// Stale sweep drops timed-out and never-stamped entries, keeps fresh ones, and reports each drop.
TEST(ConflictTrackerTest, RemovesOnlyStaleConflicts)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	static constexpr hrt_abstime kTimeout{10000000};	// 10 s
	static constexpr hrt_abstime kNow{20000000};		// 20 s

	tracker.apply_conflict(make_conflict(1, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f, kNow - 1000000), changes);
	tracker.apply_conflict(make_conflict(2, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 600.f, kNow - kTimeout - 1),
			       changes);
	tracker.apply_conflict(make_conflict(3, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 900.f, 0), changes);
	tracker.apply_conflict(make_conflict(4, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 1200.f, kNow + 1), changes);

	changes = {};
	EXPECT_TRUE(tracker.remove_stale_conflicts(kNow, kTimeout, changes));

	// Timed-out, missing, and future timestamps are invalid; the fresh entry survives.
	ASSERT_EQ(changes.size(), 3u);

	for (size_t i = 0; i < changes.size(); ++i) {
		EXPECT_EQ(changes[i].type, ConflictTrackerChangeType::kConflictRemoved);
		EXPECT_EQ(changes[i].remove_cause, RemoveBufferCause::kStaleConflict);
	}

	EXPECT_EQ(tracker.size(), 1u);
	EXPECT_TRUE(tracker.contains(make_conflict(1, 0, 0.f).encoded_id));

	// second sweep removes nothing
	changes = {};
	EXPECT_FALSE(tracker.remove_stale_conflicts(kNow, kTimeout, changes));
	EXPECT_EQ(changes.size(), 0u);
}

// clear() empties the buffer and resets the most-urgent cache.
TEST(ConflictTrackerTest, ClearResetsBufferAndMostUrgent)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f), changes);
	tracker.refresh_most_urgent();
	EXPECT_EQ(tracker.most_urgent().conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL);

	tracker.clear();

	EXPECT_TRUE(tracker.empty());
	EXPECT_EQ(tracker.most_urgent().conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);
	EXPECT_EQ(tracker.most_urgent().encoded_id.encoding, detect_and_avoid_s::UNIQUE_ID_ENCODING_ICAO);
}
