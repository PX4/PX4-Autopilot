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

char *k_data_manager_device_path = nullptr;

/* write to the data manager file */
static ssize_t _file_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count)
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

	unsigned char buffer[g_per_item_size[item]];

	/* Write out the data, prefixed with length and persistence level */
	buffer[0] = count;
	buffer[1] = persistence;
	buffer[2] = 0;
	buffer[3] = 0;

	if (count > 0) {
		memcpy(buffer + DM_SECTOR_HDR_SIZE, buf, count);
	}

	count += DM_SECTOR_HDR_SIZE;

	if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
		return -1;
	}

	if ((write(dm_operations_data.file.fd, buffer, count)) != (ssize_t)count) {
		return -1;
	}

	/* Make sure data is written to physical media */
	fsync(dm_operations_data.file.fd);

	/* All is well... return the number of user data written */
	return count - DM_SECTOR_HDR_SIZE;
}

/* Retrieve from the data manager file */
static ssize_t _file_read(dm_item_t item, unsigned index, void *buf, size_t count)
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
	unsigned char buffer[g_per_item_size[item]];
	int len = -1;

	if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) == offset) {
		len = read(dm_operations_data.file.fd, buffer, count + DM_SECTOR_HDR_SIZE);
	}

	/* Check for read error */
	if (len < 0) {
		return -errno;
	}

	/* A zero length entry is a empty entry */
	if (len == 0) {
		buffer[0] = 0;
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

static int _file_clear(dm_item_t item)
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
		char buf[1];

		if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
			result = -1;
			break;
		}

		/* Avoid SD flash wear by only doing writes where necessary */
		if (read(dm_operations_data.file.fd, buf, 1) < 1) {
			break;
		}

		/* If item has length greater than 0 it needs to be overwritten */
		if (buf[0]) {
			if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
				result = -1;
				break;
			}

			buf[0] = 0;

			if (write(dm_operations_data.file.fd, buf, 1) != 1) {
				result = -1;
				break;
			}
		}

		offset += g_per_item_size[item];
	}

	/* Make sure data is actually written to physical media */
	fsync(dm_operations_data.file.fd);
	return result;
}

static int _file_restart(dm_reset_reason reason)
{
	int offset = 0;
	int result = 0;
	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */
	for (int item = (int)DM_KEY_SAFE_POINTS; item < (int)DM_KEY_NUM_KEYS; item++) {
		for (unsigned i = 0; i < g_per_item_max_index[item]; i++) {
			/* Get data segment at current offset */
			if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
				result = -1;
				item = DM_KEY_NUM_KEYS;
				break;
			}

			uint8_t buffer[2];
			ssize_t len = read(dm_operations_data.file.fd, buffer, sizeof(buffer));

			if (len != sizeof(buffer)) {
				result = -1;
				item = DM_KEY_NUM_KEYS;
				break;
			}

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
					if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
						result = -1;
						item = DM_KEY_NUM_KEYS;
						break;
					}

					buffer[0] = 0;

					len = write(dm_operations_data.file.fd, buffer, 1);

					if (len != 1) {
						result = -1;
						item = DM_KEY_NUM_KEYS;
						break;
					}
				}
			}

			offset += g_per_item_size[item];
		}
	}

	fsync(dm_operations_data.file.fd);

	/* tell the caller how it went */
	return result;
}

static int _file_initialize(unsigned max_offset)
{
	/* See if the data manage file exists and is a multiple of the sector size */
	dm_operations_data.file.fd = open(k_data_manager_device_path, O_RDONLY | O_BINARY);

	if (dm_operations_data.file.fd >= 0) {
		// Read the mission state and check the hash
		struct dataman_compat_s compat_state;
		int ret = _file_read(DM_KEY_COMPAT, 0, &compat_state, sizeof(compat_state));

		bool incompat = true;

		if (ret == sizeof(compat_state)) {
			if (compat_state.key == DM_COMPAT_KEY) {
				incompat = false;
			}
		}

		close(dm_operations_data.file.fd);

		if (incompat) {
			unlink(k_data_manager_device_path);
		}
	}

	/* Open or create the data manager file */
	dm_operations_data.file.fd = open(k_data_manager_device_path, O_RDWR | O_CREAT | O_BINARY, PX4_O_MODE_666);

	if (dm_operations_data.file.fd < 0) {
		PX4_WARN("Could not open data manager file %s", k_data_manager_device_path);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	if ((unsigned)lseek(dm_operations_data.file.fd, max_offset, SEEK_SET) != max_offset) {
		close(dm_operations_data.file.fd);
		PX4_WARN("Could not seek data manager file %s", k_data_manager_device_path);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	/* Write current compat info */
	struct dataman_compat_s compat_state;
	compat_state.key = DM_COMPAT_KEY;
	int ret = _file_write(DM_KEY_COMPAT, 0, DM_PERSIST_POWER_ON_RESET, &compat_state, sizeof(compat_state));

	if (ret != sizeof(compat_state)) {
		PX4_ERR("Failed writing compat: %d", ret);
	}

	fsync(dm_operations_data.file.fd);
	dm_operations_data.running = true;

	return 0;
}

static void _file_shutdown()
{
	close(dm_operations_data.file.fd);
	dm_operations_data.running = false;
}

dm_operations_t dm_file_operations = {
	.write   = _file_write,
	.read    = _file_read,
	.clear   = _file_clear,
	.restart = _file_restart,
	.initialize = _file_initialize,
	.shutdown = _file_shutdown,
	.wait = px4_sem_wait,
};

