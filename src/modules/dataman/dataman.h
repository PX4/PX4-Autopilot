/****************************************************************************
 *
 *   Copyright (c) 2013-2023 PX4 Development Team. All rights reserved.
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
 * @file dataman.h
 *
 * DATAMANAGER driver.
 */
#pragma once

#include <string.h>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>

/** Types of items that the data manager can store */
typedef enum {
	DM_KEY_SAFE_POINTS_0 = 0,	///< Safe points storage 0
	DM_KEY_SAFE_POINTS_1,		///< Safe points storage 1 (alternate between 0 and 1)
	DM_KEY_SAFE_POINTS_STATE,	///< Persistent safe point state
	DM_KEY_FENCE_POINTS_0,		///< Fence vertex storage 0
	DM_KEY_FENCE_POINTS_1,		///< Fence vertex storage 1 (alternate between 0 and 1)
	DM_KEY_FENCE_POINTS_STATE,	///< Persistent fence vertex state
	DM_KEY_WAYPOINTS_OFFBOARD_0,	///< Mission way point coordinates sent over mavlink
	DM_KEY_WAYPOINTS_OFFBOARD_1,	///< (alternate between 0 and 1)
	DM_KEY_MISSION_STATE,			///< Persistent mission state
	DM_KEY_COMPAT,
	DM_KEY_NUM_KEYS					///< Total number of item types defined
} dm_item_t;

/** Types of function calls supported by the worker task */
typedef enum {
	DM_GET_ID = 0,		///< Get dataman client ID
	DM_WRITE,			///< Write index for given item
	DM_READ,			///< Read index for given item
	DM_CLEAR,			///< Clear all index for given item
	DM_NUMBER_OF_FUNCS
} dm_function_t;

/** The maximum number of instances for each item type */
enum {
	DM_KEY_SAFE_POINTS_MAX = 32,
	DM_KEY_SAFE_POINTS_STATE_MAX = 1,
	DM_KEY_FENCE_POINTS_MAX = 64,
	DM_KEY_FENCE_POINTS_STATE_MAX = 1,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX = CONFIG_NUM_MISSION_ITMES_SUPPORTED,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX = CONFIG_NUM_MISSION_ITMES_SUPPORTED,
	DM_KEY_MISSION_STATE_MAX = 1,
	DM_KEY_COMPAT_MAX = 1
};

/* table of maximum number of instances for each item type */
static const unsigned g_per_item_max_index[DM_KEY_NUM_KEYS] = {
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_SAFE_POINTS_STATE_MAX,
	DM_KEY_FENCE_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX,
	DM_KEY_FENCE_POINTS_STATE_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX,
	DM_KEY_MISSION_STATE_MAX,
	DM_KEY_COMPAT_MAX
};

struct dataman_compat_s {
	uint64_t key;
};

constexpr uint32_t MISSION_SAFE_POINT_SIZE = sizeof(struct mission_item_s);
constexpr uint32_t MISSION_SAFE_POINT_STATE_SIZE = sizeof(struct mission_stats_entry_s);
constexpr uint32_t MISSION_FENCE_POINT_SIZE = sizeof(struct mission_fence_point_s);
constexpr uint32_t MISSION_FENCE_POINT_STATE_SIZE = sizeof(struct mission_stats_entry_s);
constexpr uint32_t MISSION_ITEM_SIZE = sizeof(struct mission_item_s);
constexpr uint32_t MISSION_SIZE = sizeof(struct mission_s);
constexpr uint32_t DATAMAN_COMPAT_SIZE = sizeof(struct dataman_compat_s);

/** The table of the size of each item type */
static constexpr size_t g_per_item_size[DM_KEY_NUM_KEYS] = {
	MISSION_SAFE_POINT_SIZE,
	MISSION_SAFE_POINT_SIZE,
	MISSION_SAFE_POINT_STATE_SIZE,
	MISSION_FENCE_POINT_SIZE,
	MISSION_FENCE_POINT_SIZE,
	MISSION_FENCE_POINT_STATE_SIZE,
	MISSION_ITEM_SIZE,
	MISSION_ITEM_SIZE,
	MISSION_SIZE,
	DATAMAN_COMPAT_SIZE
};

/* increment this define whenever a binary incompatible change is performed */
#define DM_COMPAT_VERSION	5ULL

#define DM_COMPAT_KEY ((DM_COMPAT_VERSION << 32) + (sizeof(struct mission_item_s) << 24) + \
		       (sizeof(struct mission_s) << 16) + (sizeof(struct mission_stats_entry_s) << 12) + \
		       (sizeof(struct mission_fence_point_s) << 8) + (sizeof(struct mission_item_s) << 4) + \
		       sizeof(struct dataman_compat_s))
