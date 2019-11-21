/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>

struct dm_operations_t {
	ssize_t (*write)(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count);
	ssize_t (*read)(dm_item_t item, unsigned index, void *buf, size_t count);
	int (*clear)(dm_item_t item);
	int (*restart)(dm_reset_reason reason);
	int (*initialize)(unsigned max_offset);
	void (*shutdown)();
	int (*wait)(px4_sem_t *sem);
};

struct dm_operations_data_t {
	union {
		struct {
			int fd;
		} file;
		struct {
			uint8_t *data;
			uint8_t *data_end;
		} ram;
#if defined(FLASH_BASED_DATAMAN)
		struct {
			uint8_t *data;
			uint8_t *data_end;
			/* sync above with RAM backend */
			timespec flush_timeout;
		} ram_flash;
#endif
	};
	bool running;
};

static constexpr size_t DM_SECTOR_HDR_SIZE = 4;	/* data manager per item header overhead */

/* Table of the len of each item type */
static constexpr size_t g_per_item_size[DM_KEY_NUM_KEYS] = {
	sizeof(struct mission_safe_point_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_fence_point_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_item_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_item_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_item_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct mission_s) + DM_SECTOR_HDR_SIZE,
	sizeof(struct dataman_compat_s) + DM_SECTOR_HDR_SIZE
};


/* table of maximum number of instances for each item type */
static constexpr unsigned g_per_item_max_index[DM_KEY_NUM_KEYS] = {
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX,
	DM_KEY_WAYPOINTS_ONBOARD_MAX,
	DM_KEY_MISSION_STATE_MAX,
	DM_KEY_COMPAT_MAX
};

int dm_calculate_offset(dm_item_t item, unsigned index);
