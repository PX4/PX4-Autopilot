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
 * @file test_RTL_data.h
 *
 * Reusable mission and safe-point datasets for MissionRoutePlanner tests.
 *
 * default_dataset: 16-item mission with VTOL transitions and 7 rally points.
 *   The route doubles back on itself (segments 7-9 and 11-12 run roughly
 *   parallel), which stresses safe-point selection.
 *
 * corner_dataset: 16-item mission with sharp corners, a DO_JUMP loop at
 *   index 8 (jumps to 2, repeat 7), a stacked landing waypoint, and
 *   8 rally points. Critical for testing corner pruning, loop handling,
 *   and small-segment edge cases.
 *
 * @author Jonas Perolini <jonspero@me.com>
 */

#pragma once

#include "test_RTL_helpers.h"

#include <uORB/topics/vtol_vehicle_status.h>
#include <vector>

// These reusable datasets stay header-only because only a few navigator test include them.

namespace default_dataset
{

/**
 * 16-item mission (Takeoff + 12 WP + 2 VTOL transitions + Land).
 * The route runs NE then turns back SW, creating overlapping parallel
 * segments that exercise coinciding-segment and reverse-direction logic.
 */
static inline std::vector<mission_item_s> mission()
{
	return {
		makeTakeoffItem(46.09466641672733,  2.295537755442782, 540.f), // 0
		makePositionItem(46.10460551341307,  2.301953599406406, 600.f), // 1
		makePositionItem(46.106480052366116, 2.30210380311124,  590.f), // 2
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // 3
		makePositionItem(46.107937983038426, 2.300794885111972, 560.f), // 4
		makePositionItem(46.10859255161369,  2.298992440653964, 580.f), // 5
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC), // 6
		makePositionItem(46.11046175067246,  2.294146512950372, 580.f), // 7
		makePositionItem(46.113034607574434, 2.286511729126381, 600.f), // 8
		makePositionItem(46.11542884915861,  2.279091522023582, 630.f), // 9
		makePositionItem(46.11567567490963,  2.279309532739262, 570.f), // 10
		makePositionItem(46.11282851840987,  2.288196442991115, 550.f), // 11
		makePositionItem(46.110820312690294, 2.294204591184474, 600.f), // 12
		makePositionItem(46.10890129219073,  2.299118398099758, 610.f), // 13
		makePositionItem(46.11050016732537,  2.304017836312573, 560.f), // 14
		makeLandItem(46.11229326186966,      2.307146255618782, 500.f), // 15
	};
}

/**
 * 7 safe points (rally points) distributed along the mission route.
 * Indices refer to the approximate route segment each projects onto.
 */
static inline std::vector<mission_item_s> safePoints()
{
	return {
		makeSafePointAbsolute(46.108443786710971, 2.297313377810641, 466.1f), // 0: near seg 5-7
		makeSafePointAbsolute(46.104017847204972, 2.300789520693942, 470.8f), // 1: near seg 0-1
		makeSafePointAbsolute(46.11080194289591,  2.296645334195615, 460.3f), // 2: near seg 12-13
		makeSafePointAbsolute(46.111289783907274, 2.281240036015899, 464.9f), // 3: near seg 8-9
		makeSafePointAbsolute(46.096823692098205, 2.299346386046479, 482.6f), // 4: near seg 0-1
		makeSafePointAbsolute(46.114213349356625, 2.281871260136872, 463.3f), // 5: near seg 8-9
		makeSafePointAbsolute(46.094113942087709, 2.295211377278368, 493.3f), // 6: SW of takeoff
	};
}

// Typical test velocity magnitude
static constexpr float kVel = 15.f;

} // namespace default_dataset


namespace corner_dataset
{

/**
 * 16-item mission with sharp corners, a DO_JUMP loop, and a stacked landing
 * waypoint. The DO_JUMP at index 8 jumps back to index 2 with 7 repeats.
 * Waypoints 14 and 15 are at the same position (stacked land).
 */
static inline std::vector<mission_item_s> mission()
{
	return {
		makeTakeoffItem(46.10188327806167,   2.32610839977013,  530.f), // 0
		makePositionItem(46.103702117894784, 2.324670735738148, 570.f), // 1
		makePositionItem(46.10337108607288,  2.323329631230702, 600.f), // 2
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW), // 3
		makePositionItem(46.104806782736595, 2.321001473805775, 610.f), // 4
		makePositionItem(46.10207669537365,  2.318458739659657, 650.f), // 5
		makeVtolTransitionItem(vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC), // 6
		makePositionItem(46.102210599269064, 2.316656295201649, 650.f), // 7
		makeDoJump(2, 7, 0), // 8: jump to 2, repeat 7
		makePositionItem(46.1027015774365,   2.315851632497181, 601.f), // 9
		makePositionItem(46.1030958446177,   2.316299561402668, 580.f), // 10
		makePositionItem(46.10317209407736,  2.317212853572239, 565.f), // 11
		makePositionItem(46.103601692720687, 2.318304512641300, 500.f), // 12
		makePositionItem(46.103719299519405, 2.318351571600030, 510.f), // 13
		makePositionItem(46.103979492170090, 2.318099201681987, 500.f), // 14: above land
		makeLandItem(46.103979492170090,     2.318099201681987, 300.f), // 15: stacked on 14
	};
}

/**
 * 8 safe points for the corner mission. Indices refer to approximate segments.
 */
static inline std::vector<mission_item_s> safePoints()
{
	return {
		makeSafePointAbsolute(46.105409317866005, 2.321001473805775, 476.4f), // 0: corner 3-4-5
		makeSafePointAbsolute(46.103038964406146, 2.3240377344106333, 481.8f), // 1: seg 1-2
		makeSafePointAbsolute(46.10126223727336,  2.317977168050347, 464.1f), // 2: seg 5-7
		makeSafePointAbsolute(46.101994865055303, 2.316167307537449, 452.4f), // 3: seg 7-9
		makeSafePointAbsolute(46.102883833370775, 2.317119380687647, 447.4f), // 4: seg 9-10
		makeSafePointAbsolute(46.103632822324009, 2.318480317632745, 462.7f), // 5: seg 11-12
		makeSafePointAbsolute(46.104048320985939, 2.318046927984097, 400.0f), // 6: on land corner
		makeSafePointAbsolute(46.102604747110156, 2.320271613962173, 465.0f), // 7: on jump seg 7-2
	};
}

static constexpr float kVelFast = 30.f;
static constexpr float kVelDiag = 15.f;

} // namespace corner_dataset


namespace uturn_penalty_dataset
{

static constexpr int kMissionIndex = 1;

static inline std::vector<mission_item_s> mission()
{
	using rtl_test_reference::kAlt;
	using rtl_test_reference::kBaseLat;
	using rtl_test_reference::kBaseLon;

	return {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 2000.f, 0.f, kAlt),
	};
}

static inline std::vector<mission_item_s> safePoints()
{
	using rtl_test_reference::kAlt;
	using rtl_test_reference::kBaseLat;
	using rtl_test_reference::kBaseLon;

	return {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 300.f, 20.f, kAlt),
		makeSafePointFromOffset(kBaseLat, kBaseLon, 1100.f, 20.f, kAlt),
	};
}

static inline MissionRoutePlanner::Position vehiclePosition()
{
	using rtl_test_reference::kAlt;
	using rtl_test_reference::kBaseLat;
	using rtl_test_reference::kBaseLon;
	return makePositionFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt);
}

} // namespace uturn_penalty_dataset

namespace direct_to_safe_point_dataset
{

static constexpr int kMissionIndex = 0;

static inline std::vector<mission_item_s> mission()
{
	using rtl_test_reference::kAlt;
	using rtl_test_reference::kBaseLat;
	using rtl_test_reference::kBaseLon;

	return {
		makeTakeoffItemFromOffset(kBaseLat, kBaseLon, 0.f, 0.f, kAlt),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 500.f, 0.f, kAlt + 50.f),
		makePositionItemFromOffset(kBaseLat, kBaseLon, 1000.f, 0.f, kAlt + 80.f),
		makeLandItemFromOffset(kBaseLat, kBaseLon, 1500.f, 0.f, kAlt - 10.f),
	};
}

static inline std::vector<mission_item_s> safePoints()
{
	using rtl_test_reference::kAlt;
	using rtl_test_reference::kBaseLat;
	using rtl_test_reference::kBaseLon;

	return {
		makeSafePointFromOffset(kBaseLat, kBaseLon, 255.f, 0.f, kAlt + 50.f),
	};
}

static inline MissionRoutePlanner::Position vehiclePosition()
{
	using rtl_test_reference::kAlt;
	using rtl_test_reference::kBaseLat;
	using rtl_test_reference::kBaseLon;
	return makePositionFromOffset(kBaseLat, kBaseLon, 250.f, 0.f, kAlt + 50.f);
}

} // namespace direct_to_safe_point_dataset
