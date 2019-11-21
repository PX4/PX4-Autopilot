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

#include <px4_platform_common/px4_config.h>

#include "dataman.h"
#include "dataman_internal.h"

#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>

extern dm_operations_data_t dm_operations_data;

extern px4_sem_t g_init_sema;

/* Each data item is stored as follows
 *
 * byte 0: Length of user data item
 * byte 1: Persistence of this data item
 * byte 2: Unused (for future use)
 * byte 3: Unused (for future use)
 * byte DM_SECTOR_HDR_SIZE... : data item value
 *
 * The total size must not exceed g_per_item_max_index[item]
 */

/* write to the data manager RAM buffer  */
static ssize_t _ram_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			  size_t count)
{
	/* Get the offset for this item */
	const int offset = dm_calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure caller has not given us more data than we can handle */
	if (count > (g_per_item_size[item] - DM_SECTOR_HDR_SIZE)) {
		return -E2BIG;
	}

	uint8_t *buffer = &dm_operations_data.ram.data[offset];

	if (buffer > dm_operations_data.ram.data_end) {
		return -1;
	}

	/* Write out the data, prefixed with length and persistence level */
	buffer[0] = count;
	buffer[1] = persistence;
	buffer[2] = 0;
	buffer[3] = 0;

	if (count > 0) {
		memcpy(buffer + DM_SECTOR_HDR_SIZE, buf, count);
	}

	/* All is well... return the number of user data written */
	return count;
}

/* Retrieve from the data manager RAM buffer*/
static ssize_t _ram_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	if (item >= DM_KEY_NUM_KEYS) {
		return -1;
	}

	/* Get the offset for this item */
	int offset = dm_calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure the caller hasn't asked for more data than we can handle */
	if (count > (g_per_item_size[item] - DM_SECTOR_HDR_SIZE)) {
		return -E2BIG;
	}

	/* Read the prefix and data */
	uint8_t *buffer = &dm_operations_data.ram.data[offset];

	if (buffer > dm_operations_data.ram.data_end) {
		return -1;
	}

	/* See if we got data */
	if (buffer[0] > 0) {
		/* We got more than requested!!! */
		if (buffer[0] > count) {
			return -1;
		}

		/* Looks good, copy it to the caller's buffer */
		memcpy(buf, buffer + DM_SECTOR_HDR_SIZE, buffer[0]);
	}

	/* Return the number of bytes of caller data read */
	return buffer[0];
}

static int _ram_clear(dm_item_t item)
{
	int result = 0;

	/* Get the offset of 1st item of this type */
	int offset = dm_calculate_offset(item, 0);

	/* Check for item type out of range */
	if (offset < 0) {
		return -1;
	}

	/* Clear all items of this type */
	for (int i = 0; (unsigned)i < g_per_item_max_index[item]; i++) {
		uint8_t *buf = &dm_operations_data.ram.data[offset];

		if (buf > dm_operations_data.ram.data_end) {
			result = -1;
			break;
		}

		buf[0] = 0;
		offset += g_per_item_size[item];
	}

	return result;
}

/* Tell the data manager about the type of the last reset */
static int _ram_restart(dm_reset_reason reason)
{
	uint8_t *buffer = dm_operations_data.ram.data;

	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */
	for (int item = (int)DM_KEY_SAFE_POINTS; item < (int)DM_KEY_NUM_KEYS; item++) {
		for (unsigned i = 0; i < g_per_item_max_index[item]; i++) {
			/* check if segment contains data */
			if (buffer[0]) {
				bool clear_entry = false;

				/* Whether data gets deleted depends on reset type and data segment's persistence setting */
				if (reason == DM_INIT_REASON_POWER_ON) {
					if (buffer[1] > DM_PERSIST_POWER_ON_RESET) {
						clear_entry = true;
					}

				} else {
					if (buffer[1] > DM_PERSIST_IN_FLIGHT_RESET) {
						clear_entry = true;
					}
				}

				/* Set segment to unused if data does not persist */
				if (clear_entry) {
					buffer[0] = 0;
				}
			}

			buffer += g_per_item_size[item];
		}
	}

	return 0;
}

static int _ram_initialize(unsigned max_offset)
{
	/* In memory */
	dm_operations_data.ram.data = (uint8_t *)malloc(max_offset);

	if (dm_operations_data.ram.data == nullptr) {
		PX4_WARN("Could not allocate %d bytes of memory", max_offset);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	memset(dm_operations_data.ram.data, 0, max_offset);
	dm_operations_data.ram.data_end = &dm_operations_data.ram.data[max_offset - 1];
	dm_operations_data.running = true;

	return 0;
}

static void _ram_shutdown()
{
	free(dm_operations_data.ram.data);
	dm_operations_data.running = false;
}

dm_operations_t dm_ram_operations = {
	.write   = _ram_write,
	.read    = _ram_read,
	.clear   = _ram_clear,
	.restart = _ram_restart,
	.initialize = _ram_initialize,
	.shutdown = _ram_shutdown,
	.wait = px4_sem_wait,
};

