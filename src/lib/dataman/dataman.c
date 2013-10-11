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
 * @file dataman.cpp
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


static sem_t g_mutex = SEM_INITIALIZER(1);
static int g_initialized = 0;

static const unsigned g_key_sizes[DM_KEY_NUM_KEYS] = {
	DM_KEY_RTL_POINT_MAX,
	DM_KEY_RETURN_POINT_MAX,
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_WAY_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX
};

static unsigned int g_key_offsets[DM_KEY_NUM_KEYS];

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

static int
calculate_offset(unsigned char item, unsigned char index)
{
	if (!g_initialized) {
		g_key_offsets[0] = 0;

		for (unsigned i = 0; i < (DM_KEY_NUM_KEYS - 1); i++)
			g_key_offsets[i + 1] = g_key_offsets[i] + (g_key_sizes[i] * (DM_MAX_DATA_SIZE + 2));

		g_initialized = 1;
	}

	if (item >= DM_KEY_NUM_KEYS)
		return -1;

	if (index >= g_key_sizes[item])
		return -1;

	return g_key_offsets[item] + (index * (DM_MAX_DATA_SIZE + 2));
}

__EXPORT int
dm_open(void)
{
	return open(DATAMANAGER_DEVICE_PATH, O_RDWR | O_CREAT | O_BINARY);
}

__EXPORT void
dm_close(int fd)
{
	close(fd);
}

__EXPORT ssize_t
dm_write(int fd, unsigned char item, unsigned char index, unsigned char persistence, const char *buf, size_t count)
{
	unsigned char buffer[DM_MAX_DATA_SIZE + 2];
	size_t len;
	int offset;

	offset = calculate_offset(item, index);

	if (offset < 0)
		return -1;

	/* Make sure caller has given us data we can handle */
	if (count > DM_MAX_DATA_SIZE)
		return -1;

	/* Write out the data */
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

	return count - 2;
}

__EXPORT ssize_t
dm_read(int fd, unsigned char item, unsigned char index, char *buf, size_t count)
{
	unsigned char buffer[DM_MAX_DATA_SIZE + 2];
	int len, offset;

	offset = calculate_offset(item, index);

	if (offset < 0)
		return -1;

	/* Make sure the caller hasn't asked for something we can't handle */
	if (count > DM_MAX_DATA_SIZE)
		return -1;

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

	return buffer[0];
}

__EXPORT int
dm_restart(unsigned char reason)
{
	unsigned char buffer[2];
	int offset, fd, result = 0;

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

	return result;
}

