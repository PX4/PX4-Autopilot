/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: 	Lorenz Meier
 *              Jean Cyr
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
 * @file navigator_main.c
 * Implementation of the main navigation state machine.
 *
 * Handles missions, geo fencing and failsafe navigation behavior.
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
//#include <arch/board/board.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <queue.h>

#include "dataman.h"

/**
 * data manager app start / stop handling function
 *
 * @ingroup apps
 */

__EXPORT int
dataman_main(int argc, char *argv[]);

/* Retrieve from the data manager store */
__EXPORT ssize_t
dmread(
	dm_item_t item,			/* The item type to retrieve */
	unsigned char index,		/* The index of the item */
	void *buffer,			/* Pointer to caller data buffer */
	size_t buflen			/* Length in bytes of data to retrieve */
);

/* write to the data manager store */
__EXPORT ssize_t
dmwrite(
	dm_item_t  item,		/* The item type to store */
	unsigned char index,		/* The index of the item */
	dm_persitence_t persistence,	/* The persistence level of this item */
	const void *buffer,		/* Pointer to caller data buffer */
	size_t buflen			/* Length in bytes of data to retrieve */
);

/* Retrieve from the data manager store */
__EXPORT int
dm_clear(
	dm_item_t item			/* The item type to clear */
);

/* Tell the data manager about the type of the last reset */
__EXPORT int
dm_restart(
	dm_reset_reason restart_type	/* The last reset type */
);

/* Types of function calls supported by the worker task */
typedef enum {
	dm_write_func = 0,
	dm_read_func,
	dm_clear_func,
	dm_restart_func,
	dm_number_of_funcs
} dm_function_t;

/* Work task work item */
typedef struct {
	sq_entry_t link;	/**< list linkage */
	sem_t wait_sem;
	dm_function_t func;
	union {
		struct {
			dm_item_t item;
			unsigned char index;
			dm_persitence_t persistence;
			const void *buf;
			size_t count;
			ssize_t result;
		} write_params;
		struct {
			dm_item_t item;
			unsigned char index;
			void *buf;
			size_t count;
			ssize_t result;
		} read_params;
		struct {
			dm_item_t item;
			int result;
		} clear_params;
		struct {
			dm_reset_reason reason;
			int result;
		} restart_params;
	};
} dataman_q_item_t;

/* Usage statistics */
static unsigned g_func_counts[dm_number_of_funcs];

static sem_t g_work_q_mutex;	/* Mutual exclusion on work queue adds and deletes */
static sem_t g_work_mutex;	/* Mutual exclusion during IO operations */
static sem_t g_initialized;     /* Initialization lock */

/* table of maximum number of instances for each item type */
static const unsigned g_key_sizes[DM_KEY_NUM_KEYS] = {
	DM_KEY_RTL_POINT_MAX,
	DM_KEY_RETURN_POINT_MAX,
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX,
	DM_KEY_WAY_POINTS_MAX,
};

/* Table of offset for index 0 of each item type */
static unsigned int g_key_offsets[DM_KEY_NUM_KEYS];

/* The data manager store file handle and file name */
static int g_fd = -1;
static const char *k_data_manager_device_path = "/fs/microsd/data";

/* The data manager work queue */
static sq_queue_t g_dataman_work;

static bool g_task_should_exit;		/**< if true, sensor task should exit */

static const unsigned k_sector_size = DM_MAX_DATA_SIZE + 2;

static dataman_q_item_t *
create_work_item(void)
{
	dataman_q_item_t *item;

	if ((item = (dataman_q_item_t *)malloc(sizeof(dataman_q_item_t))))
		sem_init(&item->wait_sem, 1, 0); /* Caller will wait on this... initially locked */

	return item;
}

static void
destroy_work_item(dataman_q_item_t *item)
{
	sem_destroy(&item->wait_sem);
	free(item);
}

static void
enqueue_work_item(dataman_q_item_t *item)
{
        /* put the work item on the work queue */
        sem_wait(&g_work_q_mutex);
        sq_addlast(&item->link, &g_dataman_work);
        sem_post(&g_work_q_mutex);
        /* tell the work thread that work is available */
        sem_post(&g_work_mutex);
}

static dataman_q_item_t *
dequeue_work_item(void)
{
        dataman_q_item_t *work;
        sem_wait(&g_work_q_mutex);
        work = (dataman_q_item_t *)sq_remfirst(&g_dataman_work);
        sem_post(&g_work_q_mutex);
        return work;
}

/* Calculate the offset in file of specific item */
static int
calculate_offset(dm_item_t item, unsigned char index)
{

	/* Make sure the item type is valid */
	if (item >= DM_KEY_NUM_KEYS)
		return -1;

	/* Make sure the index for this item type is valid */
	if (index >= g_key_sizes[item])
		return -1;

	/* Calculate and return the item index based on type and index */
	return g_key_offsets[item] + (index * k_sector_size);
}

/* Each data item is stored as follows
 *
 * byte 0: Length of user data item
 * byte 1: Persistence of this data item
 * byte 2... : data item value
 *
 * The total size must not exceed k_sector_size
 */

/* write to the data manager file */
static ssize_t
_write(dm_item_t item, unsigned char index, dm_persitence_t persistence, const void *buf, size_t count)
{
	unsigned char buffer[k_sector_size];
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

	if (lseek(g_fd, offset, SEEK_SET) == offset)
		if ((len = write(g_fd, buffer, count)) == count)
			fsync(g_fd);

	if (len != count)
		return -1;

	/* All is well... return the number of user data written */
	return count - 2;
}

/* Retrieve from the data manager file */
static ssize_t
_read(dm_item_t item, unsigned char index, void *buf, size_t count)
{
	unsigned char buffer[k_sector_size];
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
	/* Try to get fresh data */
	fsync(g_fd);

	if (lseek(g_fd, offset, SEEK_SET) == offset)
		len = read(g_fd, buffer, count + 2);

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

static int
_clear(dm_item_t item)
{
	int i, result = 0;

	int offset = calculate_offset(item, 0);
	if (offset < 0)
		return -1;

	fsync(g_fd);

	for (i = 0; (unsigned)i < g_key_sizes[item]; i++) {
		char buf[1];

		if (lseek(g_fd, offset, SEEK_SET) != offset) {
			result = -1;
			break;
		}

		if (read(g_fd, buf, 1) < 1)
			break;

		if (buf[0]) {
			if (lseek(g_fd, offset, SEEK_SET) != offset) {
				result = -1;
				break;
			}

			buf[0] = 0;

			if (write(g_fd, buf, 1) != 1) {
				result = -1;
				break;
			}
		}

		offset += k_sector_size;
	}

	fsync(g_fd);
	return result;
}

/* Tell the data manager about the type of the last reset */
static int
_restart(dm_reset_reason reason)
{
	unsigned char buffer[2];
	int offset, result = 0;

	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */
	offset = 0;

	fsync(g_fd);

	while (1) {
		size_t len;

		/* Get data segment at current offset */
		if (lseek(g_fd, offset, SEEK_SET) != offset) {
			result = -1;
			break;
		}

		len = read(g_fd, buffer, sizeof(buffer));

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
				if (lseek(g_fd, offset, SEEK_SET) != offset) {
					result = -1;
					break;
				}

				buffer[0] = 0;

				len = write(g_fd, buffer, 1);

				if (len != 1) {
					result = -1;
					break;
				}
			}
		}

		offset += k_sector_size;
	}

	fsync(g_fd);

	/* tell the caller how it went */
	return result;
}

/* write to the data manager file */
__EXPORT ssize_t
dm_write(dm_item_t item, unsigned char index, dm_persitence_t persistence, const void *buf, size_t count)
{
	dataman_q_item_t *work;

	if (g_fd < 0)
		return -1;

	work = create_work_item();

	work->func = dm_write_func;
	work->write_params.item = item;
	work->write_params.index = index;
	work->write_params.persistence = persistence;
	work->write_params.buf = buf;
	work->write_params.count = count;
	enqueue_work_item(work);
	sem_wait(&work->wait_sem);
	ssize_t result = work->write_params.result;
	destroy_work_item(work);
	return result;
}

/* Retrieve from the data manager file */
__EXPORT ssize_t
dm_read(dm_item_t item, unsigned char index, void *buf, size_t count)
{
	dataman_q_item_t *work;

	if (g_fd < 0)
		return -1;

	work = create_work_item();

	work->func = dm_read_func;
	work->read_params.item = item;
	work->read_params.index = index;
	work->read_params.buf = buf;
	work->read_params.count = count;
	enqueue_work_item(work);
	sem_wait(&work->wait_sem);
	ssize_t result = work->read_params.result;
	destroy_work_item(work);
	return result;
}

__EXPORT int
dm_clear(dm_item_t item)
{
	dataman_q_item_t *work;

	if (g_fd < 0)
		return -1;

	work = create_work_item();

	if (work == NULL)
		return;

	work->func = dm_clear_func;
	work->clear_params.item = item;
	enqueue_work_item(work);
	sem_wait(&work->wait_sem);
	int result = work->clear_params.result;
	destroy_work_item(work);
	return result;
}

/* Tell the data manager about the type of the last reset */
__EXPORT int
dm_restart(dm_reset_reason reason)
{
	dataman_q_item_t *work;

	if (g_fd < 0)
		return -1;

	work = create_work_item();

	if (work == NULL)
		return -1;

	work->func = dm_restart_func;
	work->restart_params.reason = reason;
	enqueue_work_item(work);
	sem_wait(&work->wait_sem);
	int result = work->restart_params.result;
	destroy_work_item(work);
	return result;
}

int
task_main(int argc, char *argv[])
{
        dataman_q_item_t *work;

	/* inform about start */
	warnx("Initializing..");

	g_key_offsets[0] = 0;

	for (unsigned i = 0; i < (DM_KEY_NUM_KEYS - 1); i++)
		g_key_offsets[i + 1] = g_key_offsets[i] + (g_key_sizes[i] * k_sector_size);

	for (unsigned i = 0; i < dm_number_of_funcs; i++)
		g_func_counts[i] = 0;

	g_task_should_exit = false;
	sem_init(&g_work_q_mutex, 1, 1);
	sem_init(&g_work_mutex, 1, 0);

	g_fd = open(k_data_manager_device_path, O_RDWR | O_CREAT | O_BINARY);

	sem_post(&g_initialized);

	while (true) {

                /* do we need to exit ??? */
		if (g_task_should_exit)
			break;
                /* wait for work */
		sem_wait(&g_work_mutex);
                /* make sure we still don't need to exit */
		if (g_task_should_exit)
			break;

		if ((work = dequeue_work_item()) == NULL)
			continue; /* In theory this shouldn't happen */

		switch (work->func) {
		case dm_write_func:
			g_func_counts[dm_write_func]++;
			work->write_params.result =
				_write(work->write_params.item,
					   work->write_params.index,
					   work->write_params.persistence,
					   work->write_params.buf,
					   work->write_params.count);
			break;

		case dm_read_func:
			g_func_counts[dm_read_func]++;
			work->read_params.result =
				_read(work->read_params.item,
					  work->read_params.index,
					  work->read_params.buf,
					  work->read_params.count);
			break;

		case dm_clear_func:
			g_func_counts[dm_clear_func]++;
			work->clear_params.result =
				_clear(work->clear_params.item);
			break;

		case dm_restart_func:
			g_func_counts[dm_restart_func]++;
			work->restart_params.result =
				_restart(work->restart_params.reason);
			break;
		}
                /* Inform the caller that work is done */
		sem_post(&work->wait_sem);
	}

        /* all done... clean up */
	int fd = g_fd;
	g_fd = -1;
	close(fd);

	sem_destroy(&g_work_q_mutex);
	sem_destroy(&g_work_mutex);

	return 0;
}

static int
start(void)
{
        int task;

	sem_init(&g_initialized, 1, 0);
	/* start the task */
	if ((task = task_spawn_cmd("dataman", SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 2048, task_main, NULL)) <= 0) {
		warn("task start failed");
		return -1;
	}
        /* wait for the thread to actuall initialize */
	sem_wait(&g_initialized);

	return 0;
}

static void
status(void)
{
        /* display usage statistics */
	warnx("Writes   %d", g_func_counts[dm_write_func]);
	warnx("Reads    %d", g_func_counts[dm_read_func]);
	warnx("Clears   %d", g_func_counts[dm_clear_func]);
	warnx("Restarts %d", g_func_counts[dm_restart_func]);
}

static void
stop(void)
{
        /* Tell the worker task to shut down */
	g_task_should_exit = true;
	sem_post(&g_work_mutex);
}

static void
usage(void)
{
	errx(1, "usage: dataman {start|stop|status}");
}

int
dataman_main(int argc, char *argv[])
{
	if (argc < 2)
		usage();

	if (!strcmp(argv[1], "start")) {

		if (g_fd >= 0)
			errx(1, "already running");

		start();

		if (g_fd < 0)
			errx(1, "start failed");

		return 0;
	}

	if (g_fd < 0)
		errx(1, "not running");

	if (!strcmp(argv[1], "stop"))
		stop();
	else if (!strcmp(argv[1], "status"))
		status();
	else
		usage();

	return 0;
}
