/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file dataman.c
 *
 * DATAMANAGER driver.
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>

#include <lib/dataman/dataman.h>

static char *k_data_manager_device_path = "/fs/microsd/data";

static sem_t g_mutex = SEM_INITIALIZER(1);	/* Mutual exclusion during IO operations */
static int g_initialized = 0;			/* Initialization flag */

/* table of maximum number of instances for each item type */
static const unsigned g_key_sizes[DM_KEY_NUM_KEYS] = {
	DM_KEY_RTL_POINT_MAX,
	DM_KEY_RETURN_POINT_MAX,
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_WAY_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX
};

/* Table of offset for index 0 of each item type */
static unsigned int g_key_offsets[DM_KEY_NUM_KEYS];

/* Local lock and unlock functions */
static void
lock(void)
{
	sem_wait(&g_mutex);
}

static void
unlock(void)
{
	sem_post(&g_mutex);
}

/* Calculate the offset in file of specific item */
static int
calculate_offset(dm_item_t item, unsigned char index)
{
	/* If offset table hasn't been initialized the initialized it */
	if (!g_initialized) {
		g_key_offsets[0] = 0;

		for (unsigned i = 0; i < (DM_KEY_NUM_KEYS - 1); i++)
			g_key_offsets[i + 1] = g_key_offsets[i] + (g_key_sizes[i] * (DM_MAX_DATA_SIZE + 2));

		g_initialized = 1;
	}

	/* Make sure the item type is valid */
	if (item >= DM_KEY_NUM_KEYS)
		return -1;

	/* Make sure the index for this item type is valid */
	if (index >= g_key_sizes[item])
		return -1;

	/* Calulate and return the item index based on type and index */
	return g_key_offsets[item] + (index * (DM_MAX_DATA_SIZE + 2));
}

/* Open the global data manager file */
__EXPORT int
dm_open(void)
{
	return open(k_data_manager_device_path, O_RDWR | O_CREAT | O_BINARY);
}

/* Close the global data manager file */
__EXPORT void
dm_close(int fd)
{
	close(fd);
}

/* write to the data manager file */
__EXPORT ssize_t
dm_write(int fd, dm_item_t item, unsigned char index, dm_persitence_t persistence, const char *buf, size_t count)
{
	unsigned char buffer[DM_MAX_DATA_SIZE + 2];
	size_t len;
	int offset;

	/* Get the offset for this item */
	offset = calculate_offset(item, index);

	if (offset < 0)
		return -1;

	/* Make sure caller has not given us more data than we can handle */
	if (count > DM_MAX_DATA_SIZE)
		return -1;

	/* Write out the data, prefixed with length and persistence level */
	buffer[0] = count;
	buffer[1] = persistence;
	memcpy(buffer + 2, buf, count);
	count += 2;

	len = -1;
	lock();

	if (lseek(fd, offset, SEEK_SET) == offset)
		len = write(fd, buffer, count);

	unlock();

	if (len != count)
		return -1;

	/* All is well... return the number of user data written */
	return count - 2;
}

/* Retrieve from the data manager file */
__EXPORT ssize_t
dm_read(int fd, dm_item_t item, unsigned char index, char *buf, size_t count)
{
	unsigned char buffer[DM_MAX_DATA_SIZE + 2];
	int len, offset;

	/* Get the offset for this item */
	offset = calculate_offset(item, index);

	if (offset < 0)
		return -1;

	/* Make sure the caller hasn't asked for more data than we can handle */
	if (count > DM_MAX_DATA_SIZE)
		return -1;

	/* Read the prefix and data */
	len = -1;
	lock();

	if (lseek(fd, offset, SEEK_SET) == offset)
		len = read(fd, buffer, count + 2);

	unlock();

	/* Check for length issues */
	if (len < 0)
		return -1;

	if (len == 0)
		buffer[0] = 0;

	if (buffer[0] > 0) {
		if (buffer[0] > count)
			return -1;

		/* Looks good, copy it to the caller's buffer */
		memcpy(buf, buffer + 2, buffer[0]);
	}

	/* Return the number of bytes of caller data read */
	return buffer[0];
}

/* Tell the data manager about the type of the last reset */
__EXPORT int
dm_restart(dm_reset_reason reason)
{
	unsigned char buffer[2];
	int offset, fd, result = 0;

	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	fd = dm_open();

	if (fd < 0)
		return fd;

	/* Loop through all of the data segments and delete those that are not persistent */
	offset = 0;

	lock();

	while (1) {
		size_t len;

		/* Get data segment at current offset */
		if (lseek(fd, offset, SEEK_SET) != offset) {
			result = -1;
			break;
		}

		len = read(fd, buffer, sizeof(buffer));

		if (len == 0)
			break;

		/* check if segment contains data */
		if (buffer[0]) {
			int clear_entry = 0;

			/* Whether data gets deleted depends on reset type and data segment's persistence setting */
			if (reason == DM_INIT_REASON_POWER_ON) {
				if (buffer[1] != DM_PERSIST_POWER_ON_RESET) {
					clear_entry = 1;
				}

			} else {
				if ((buffer[1] != DM_PERSIST_POWER_ON_RESET) && (buffer[1] != DM_PERSIST_IN_FLIGHT_RESET)) {
					clear_entry = 1;
				}
			}

			/* Set segment to unused if data does not persist */
			if (clear_entry) {
				if (lseek(fd, offset, SEEK_SET) != offset) {
					result = -1;
					break;
				}

				buffer[0] = 0;

				len = write(fd, buffer, 1);

				if (len != 1) {
					result = -1;
					break;
				}
			}
		}

		offset += DM_MAX_DATA_SIZE + 2;
	}

	unlock();

	/* tell the caller how it went */
	return result;
}
