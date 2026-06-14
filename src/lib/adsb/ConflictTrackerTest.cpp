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

// WHY: Every accepted conflict must enter the buffer and return as an added change so the caller can announce it.
// WHAT: Apply a new conflict, verify the kConflictAdded change, the buffer content and the most-urgent cache.
TEST(ConflictTrackerTest, AddsNewConflictAndRefreshesMostUrgent)
{
	ConflictTracker tracker;

	// GIVEN: An empty tracker with a cleared most-urgent cache.
	EXPECT_TRUE(tracker.empty());
	EXPECT_EQ(tracker.most_urgent().conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);

	// WHEN: A new medium conflict is applied.
	const conflict_info_s conflict = make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 800.f);
	conflict_tracker_changes_s changes{};

	// THEN: The tracked set changes, one added change is emitted and the conflict is retrievable.
	EXPECT_TRUE(tracker.apply_conflict(conflict, changes));
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictAdded);
	EXPECT_EQ(changes[0].conflict.encoded_id, conflict.encoded_id);

	EXPECT_EQ(tracker.size(), 1u);
	EXPECT_TRUE(tracker.contains(conflict.encoded_id));

	conflict_info_s tracked{};
	ASSERT_TRUE(tracker.get_conflict(conflict.encoded_id, tracked));
	EXPECT_EQ(tracked.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM);

	// AND: The cached most-urgent conflict only changes after an explicit refresh.
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);
	tracker.refresh_most_urgent();
	EXPECT_EQ(tracker.most_urgent().encoded_id, conflict.encoded_id);
}

// WHY: Traffic that is not in conflict and not yet tracked must never occupy a buffer slot.
// WHAT: Apply a NONE-level report for an unknown ID and verify it is not added.
TEST(ConflictTrackerTest, IgnoresUntrackedTrafficWithoutConflict)
{
	ConflictTracker tracker;

	const conflict_info_s no_conflict = make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE, 5000.f);
	conflict_tracker_changes_s changes{};

	EXPECT_FALSE(tracker.apply_conflict(no_conflict, changes));
	EXPECT_EQ(changes.size(), 0u);
	EXPECT_TRUE(tracker.empty());
}

// WHY: The caller's notification policy depends on level transitions, so the tracker must report
// them exactly once per change and stay silent on same-level.
// WHAT: Update a tracked conflict with the same level, an escalation and a de-escalation.
TEST(ConflictTrackerTest, EmitsLevelChangedOnlyWhenLevelDiffers)
{
	ConflictTracker tracker;

	conflict_tracker_changes_s changes{};
	tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f, 1000000), changes);

	// WHEN: The same conflict is refreshed at the same level.
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 1900.f, 2000000),
					   changes));

	// THEN: The entry is overwritten without a change.
	EXPECT_EQ(changes.size(), 0u);
	conflict_info_s tracked{};
	ASSERT_TRUE(tracker.get_conflict(make_conflict(0x6E9F7B, 0, 0.f).encoded_id, tracked));
	EXPECT_EQ(tracked.latest_update_timestamp, 2000000u);
	EXPECT_FLOAT_EQ(tracked.aircraft_dist, 1900.f);

	// WHEN: The conflict escalates.
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 600.f, 3000000),
					   changes));

	// THEN: One level-changed change carries the previous level.
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictLevelChanged);
	EXPECT_EQ(changes[0].conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_EQ(changes[0].previous_level, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW);

	// WHEN: The conflict de-escalates to a still-active level.
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 900.f,
					   4000000), changes));

	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].previous_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_EQ(tracker.size(), 1u);
}

// WHY: A resolved conflict must leave the buffer, and the caller must see the transition to NONE
// (not a removal change) so it can announce "solved" instead of a misleading drop warning.
// WHAT: Apply a NONE-level update to a tracked conflict.
TEST(ConflictTrackerTest, ResolvedConflictLeavesBufferAsLevelChange)
{
	ConflictTracker tracker;

	conflict_tracker_changes_s changes{};
	tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 600.f), changes);
	tracker.refresh_most_urgent();

	// WHEN: The tracked conflict reports no conflict anymore.
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0x6E9F7B, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE, 5000.f),
					   changes));

	// THEN: The entry is dropped and reported as a level change to NONE.
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kConflictLevelChanged);
	EXPECT_EQ(changes[0].conflict.conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(changes[0].previous_level, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH);
	EXPECT_TRUE(tracker.empty());

	// AND: Refreshing resets the most-urgent cache to the no-conflict state.
	tracker.refresh_most_urgent();
	EXPECT_EQ(tracker.most_urgent().conflict_level, detect_and_avoid_s::DAA_CONFLICT_LVL_NONE);
	EXPECT_EQ(tracker.most_urgent().encoded_id.id, 0u);
}

// WHY: The caller does not emit secondary conflict warnings, and decides this after the transponder report
// uorb queue has been processed. The change must therefore be captured when the change is recorded.
// WHAT: Change levels of the most urgent and of a secondary conflict and check the captured flag.
TEST(ConflictTrackerTest, LevelChangeCapturesMostUrgent)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	// GIVEN: A critical conflict (cached as most urgent) and a secondary low conflict.
	tracker.apply_conflict(make_conflict(0xA, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f), changes);
	tracker.apply_conflict(make_conflict(0xB, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f), changes);
	tracker.refresh_most_urgent();

	// WHEN: The secondary conflict escalates but stays below the most urgent one.
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0xB, detect_and_avoid_s::DAA_CONFLICT_LVL_MEDIUM, 1500.f),
					   changes));

	// THEN: The change is not flagged as the most urgent conflict.
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_FALSE(changes[0].conflict_is_most_urgent);

	// WHEN: The most urgent conflict itself changes level.
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0xA, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH, 400.f), changes));

	// THEN: The change is flagged via the cached most urgent entry.
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_TRUE(changes[0].conflict_is_most_urgent);

	// WHEN: A new conflict escalates to most urgent (cache still points at 0xA).
	changes = {};
	tracker.apply_conflict(make_conflict(0xC, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 3000.f), changes);
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(make_conflict(0xC, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 50.f),
					   changes));

	// THEN: The change is flagged via the buffer selection, despite the stale cache.
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_TRUE(changes[0].conflict_is_most_urgent);
}

// WHY: When the buffer is full, a more urgent conflict must displace the least urgent entry and the
// eviction must be reported so the operator can be told a tracked conflict was dropped.
// WHAT: Fill the buffer, add a more urgent conflict and inspect the emitted change sequence.
TEST(ConflictTrackerTest, EvictsLeastUrgentWhenFull)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	// GIVEN: A full buffer where 0x100 is the least urgent entry (lowest level, largest distance).
	for (uint64_t i = 0; i < kDaaMaxTraffic; ++i) {
		changes = {};
		ASSERT_TRUE(tracker.apply_conflict(make_conflict(0x100 + i, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW + (i > 0),
						   2000.f - static_cast<float>(i) * 100.f), changes));
	}

	ASSERT_EQ(tracker.size(), static_cast<size_t>(kDaaMaxTraffic));

	// WHEN: A critical conflict arrives.
	const conflict_info_s critical = make_conflict(0xBEEF, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f);
	changes = {};
	EXPECT_TRUE(tracker.apply_conflict(critical, changes));

	// THEN: The least urgent entry is evicted (removal first, then the insertion).
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

// WHY: A full buffer must never trade a tracked conflict for less important traffic.
// WHAT: Offer a weaker conflict to a full buffer and verify the ignored change and untouched buffer.
TEST(ConflictTrackerTest, RejectsLessImportantWhenFull)
{
	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	for (uint64_t i = 0; i < kDaaMaxTraffic; ++i) {
		changes = {};
		ASSERT_TRUE(tracker.apply_conflict(make_conflict(0x200 + i, detect_and_avoid_s::DAA_CONFLICT_LVL_HIGH,
						   600.f + static_cast<float>(i)), changes));
	}

	// WHEN: A weaker (lower-level) conflict arrives on the full buffer.
	changes = {};
	EXPECT_FALSE(tracker.apply_conflict(make_conflict(0xBEEF, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 2000.f), changes));

	// THEN: It is reported as ignored due to the full buffer and nothing is displaced.
	ASSERT_EQ(changes.size(), 1u);
	EXPECT_EQ(changes[0].type, ConflictTrackerChangeType::kReportIgnored);
	EXPECT_EQ(changes[0].ignore_cause, IgnoreTrafficCause::kBufferFull);
	EXPECT_EQ(changes[0].conflict.encoded_id.id, 0xBEEFu);
	EXPECT_EQ(tracker.size(), static_cast<size_t>(kDaaMaxTraffic));
	EXPECT_FALSE(tracker.contains(make_conflict(0xBEEF, 0, 0.f).encoded_id));
}

// WHY: Conflict priority drives both most-urgent selection and eviction; level must dominate and
// distance must only break ties.
// WHAT: Exercise the comparison rules and the most-urgent selection directly.
TEST(ConflictTrackerTest, PriorityIsLevelFirstThenDistance)
{
	const conflict_info_s far_critical = make_conflict(1, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 3000.f);
	const conflict_info_s close_low = make_conflict(2, detect_and_avoid_s::DAA_CONFLICT_LVL_LOW, 10.f);
	const conflict_info_s close_critical = make_conflict(3, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f);
	const conflict_info_s same_as_close_critical =
		make_conflict(4, detect_and_avoid_s::DAA_CONFLICT_LVL_CRITICAL, 100.f);

	// GIVEN: A high-level conflict is farther away than a low-level conflict.
	// THEN: Level dominates distance.
	EXPECT_TRUE(ConflictTracker::is_conflict_more_important(far_critical, close_low));
	EXPECT_TRUE(ConflictTracker::is_conflict_less_important(close_low, far_critical));

	// GIVEN: Two conflicts have the same level.
	// THEN: Distance breaks the tie.
	EXPECT_TRUE(ConflictTracker::is_conflict_more_important(close_critical, far_critical));
	EXPECT_TRUE(ConflictTracker::is_conflict_less_important(far_critical, close_critical));
	EXPECT_FALSE(ConflictTracker::is_conflict_more_important(close_critical, same_as_close_critical));
	EXPECT_FALSE(ConflictTracker::is_conflict_less_important(close_critical, same_as_close_critical));

	ConflictTracker tracker;
	conflict_tracker_changes_s changes{};

	// WHEN: The conflicts are inserted in non-priority order.
	tracker.apply_conflict(close_low, changes);
	tracker.apply_conflict(far_critical, changes);
	tracker.apply_conflict(close_critical, changes);

	// THEN: The closest critical conflict is selected.
	conflict_info_s most_urgent{};
	ASSERT_TRUE(tracker.find_most_urgent(most_urgent));
	EXPECT_EQ(most_urgent.encoded_id.id, 3u);
}

// WHY: Conflicts that stop reporting must be dropped after the timeout so the buffer only holds
// live traffic, and each drop must be reported for the operator warning.
// WHAT: Mix new, stale and never-stamped entries and run a stale sweep.
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

	// WHEN: A stale sweep runs at kNow.
	changes = {};
	EXPECT_TRUE(tracker.remove_stale_conflicts(kNow, kTimeout, changes));

	// THEN: The timed-out and never-stamped entries are dropped, the fresh one survives.
	ASSERT_EQ(changes.size(), 2u);

	for (size_t i = 0; i < changes.size(); ++i) {
		EXPECT_EQ(changes[i].type, ConflictTrackerChangeType::kConflictRemoved);
		EXPECT_EQ(changes[i].remove_cause, RemoveBufferCause::kStaleConflict);
	}

	EXPECT_EQ(tracker.size(), 1u);
	EXPECT_TRUE(tracker.contains(make_conflict(1, 0, 0.f).encoded_id));

	// AND: A second sweep finds nothing to remove.
	changes = {};
	EXPECT_FALSE(tracker.remove_stale_conflicts(kNow, kTimeout, changes));
	EXPECT_EQ(changes.size(), 0u);
}

// WHY: Deactivation paths rely on clear().
// WHAT: Fill the tracker, refresh the most-urgent cache, clear and verify the no-conflict state.
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
