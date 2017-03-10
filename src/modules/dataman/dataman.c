/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * DATAMANAGER driver.
 *
 * @author Jean Cyr
 * @author Lorenz Meier
 * @author Julian Oes
 * @author Thomas Gubler
 * @author David Sidrane
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <systemlib/err.h>
#include <queue.h>
#include <string.h>
#include <semaphore.h>
#include <unistd.h>
#include <platforms/px4_getopt.h>

#include "dataman.h"
#include <systemlib/param/param.h>

/**
 * data manager app start / stop handling function
 *
 * @ingroup apps
 */

__EXPORT int dataman_main(int argc, char *argv[]);
__EXPORT ssize_t dm_read(dm_item_t item, unsigned index, void *buffer, size_t buflen);
__EXPORT ssize_t dm_write(dm_item_t  item, unsigned index, dm_persitence_t persistence, const void *buffer,
			  size_t buflen);
__EXPORT int dm_clear(dm_item_t item);
__EXPORT void dm_lock(dm_item_t item);
__EXPORT void dm_unlock(dm_item_t item);
__EXPORT int dm_restart(dm_reset_reason restart_type);

/* Private File based Operations */
static ssize_t _file_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			   size_t count);
static ssize_t _file_read(dm_item_t item, unsigned index, void *buf, size_t count);
static int  _file_clear(dm_item_t item);
static int  _file_restart(dm_reset_reason reason);
static int _file_initialize(unsigned max_offset);
static void _file_shutdown(void);

/* Private Ram based Operations */
static ssize_t _ram_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			  size_t count);
static ssize_t _ram_read(dm_item_t item, unsigned index, void *buf, size_t count);
static int  _ram_clear(dm_item_t item);
static int  _ram_restart(dm_reset_reason reason);
static int _ram_initialize(unsigned max_offset);
static void _ram_shutdown(void);

typedef struct dm_operations_t {
	ssize_t (*write)(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count);
	ssize_t (*read)(dm_item_t item, unsigned index, void *buf, size_t count);
	int (*clear)(dm_item_t item);
	int (*restart)(dm_reset_reason reason);
	int (*initialize)(unsigned max_offset);
	void (*shutdown)(void);
} dm_operations_t;

static dm_operations_t dm_file_operations = {
	.write   = _file_write,
	.read    = _file_read,
	.clear   = _file_clear,
	.restart = _file_restart,
	.initialize = _file_initialize,
	.shutdown = _file_shutdown,
};

static dm_operations_t dm_ram_operations = {
	.write   = _ram_write,
	.read    = _ram_read,
	.clear   = _ram_clear,
	.restart = _ram_restart,
	.initialize = _ram_initialize,
	.shutdown = _ram_shutdown,
};

static dm_operations_t *g_dm_ops;

static struct {
	union {
		struct {
			int fd;
		} file;
		struct {
			uint8_t *data;
			uint8_t *data_end;
		} ram;
	};
	bool running;
} dm_operations_data;

/** Types of function calls supported by the worker task */
typedef enum {
	dm_write_func = 0,
	dm_read_func,
	dm_clear_func,
	dm_restart_func,
	dm_number_of_funcs
} dm_function_t;

/** Work task work item */
typedef struct {
	sq_entry_t link;	/**< list linkage */
	px4_sem_t wait_sem;
	unsigned char first;
	unsigned char func;
	ssize_t result;
	union {
		struct {
			dm_item_t item;
			unsigned index;
			dm_persitence_t persistence;
			const void *buf;
			size_t count;
		} write_params;
		struct {
			dm_item_t item;
			unsigned index;
			void *buf;
			size_t count;
		} read_params;
		struct {
			dm_item_t item;
		} clear_params;
		struct {
			dm_reset_reason reason;
		} restart_params;
	};
} work_q_item_t;

const size_t k_work_item_allocation_chunk_size = 8;

/* Usage statistics */
static unsigned g_func_counts[dm_number_of_funcs];

/* table of maximum number of instances for each item type */
static const unsigned g_per_item_max_index[DM_KEY_NUM_KEYS] = {
	DM_KEY_SAFE_POINTS_MAX,
	DM_KEY_FENCE_POINTS_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX,
	DM_KEY_WAYPOINTS_ONBOARD_MAX,
	DM_KEY_MISSION_STATE_MAX,
	DM_KEY_COMPAT_MAX
};

/* Table of offset for index 0 of each item type */
static unsigned int g_key_offsets[DM_KEY_NUM_KEYS];

/* Item type lock mutexes */
static px4_sem_t *g_item_locks[DM_KEY_NUM_KEYS];
static px4_sem_t g_sys_state_mutex;

/* The data manager store file handle and file name */
#if defined(__PX4_POSIX_EAGLE) || defined(__PX4_POSIX_EXCELSIOR)
static const char *default_device_path = PX4_ROOTFSDIR"/dataman";
#else
static const char *default_device_path = PX4_ROOTFSDIR"/fs/microsd/dataman";
#endif
static char *k_data_manager_device_path = NULL;

static enum {
	BACKEND_NONE = 0,
	BACKEND_FILE,
	BACKEND_RAM,
	BACKEND_LAST
} backend = BACKEND_NONE;

/* The data manager work queues */

typedef struct {
	sq_queue_t q;		/* Nuttx queue */
	px4_sem_t mutex;		/* Mutual exclusion on work queue adds and deletes */
	unsigned size;		/* Current size of queue */
	unsigned max_size;	/* Maximum queue size reached */
} work_q_t;

static work_q_t g_free_q;	/* queue of free work items. So that we don't always need to call malloc and free*/
static work_q_t g_work_q;	/* pending work items. To be consumed by worker thread */

static px4_sem_t g_work_queued_sema;	/* To notify worker thread a work item has been queued */
static px4_sem_t g_init_sema;

static bool g_task_should_exit;	/**< if true, dataman task should exit */

#define DM_SECTOR_HDR_SIZE 4	/* data manager per item header overhead */
static const unsigned k_sector_size = DM_MAX_DATA_SIZE + DM_SECTOR_HDR_SIZE; /* total item sorage space */

static void init_q(work_q_t *q)
{
	sq_init(&(q->q));		/* Initialize the NuttX queue structure */
	px4_sem_init(&(q->mutex), 1, 1);	/* Queue is initially unlocked */
	q->size = q->max_size = 0;	/* Queue is initially empty */
}

static inline void
destroy_q(work_q_t *q)
{
	px4_sem_destroy(&(q->mutex));	/* Destroy the queue lock */
}

static inline void
lock_queue(work_q_t *q)
{
	px4_sem_wait(&(q->mutex));	/* Acquire the queue lock */
}

static inline void
unlock_queue(work_q_t *q)
{
	px4_sem_post(&(q->mutex));	/* Release the queue lock */
}

static work_q_item_t *
create_work_item(void)
{
	work_q_item_t *item;

	/* Try to reuse item from free item queue */
	lock_queue(&g_free_q);

	if ((item = (work_q_item_t *)sq_remfirst(&(g_free_q.q)))) {
		g_free_q.size--;
	}

	unlock_queue(&g_free_q);

	/* If we there weren't any free items then obtain memory for a new ones */
	if (item == NULL) {
		item = (work_q_item_t *)malloc(k_work_item_allocation_chunk_size * sizeof(work_q_item_t));

		if (item) {
			item->first = 1;
			lock_queue(&g_free_q);

			for (size_t i = 1; i < k_work_item_allocation_chunk_size; i++) {
				(item + i)->first = 0;
				sq_addfirst(&(item + i)->link, &(g_free_q.q));
			}

			/* Update the queue size and potentially the maximum queue size */
			g_free_q.size += k_work_item_allocation_chunk_size - 1;

			if (g_free_q.size > g_free_q.max_size) {
				g_free_q.max_size = g_free_q.size;
			}

			unlock_queue(&g_free_q);
		}
	}

	/* If we got one then lock the item*/
	if (item) {
		px4_sem_init(&item->wait_sem, 1, 0);        /* Caller will wait on this... initially locked */

		/* item->wait_sem use case is a signal */

		px4_sem_setprotocol(&item->wait_sem, SEM_PRIO_NONE);
	}

	/* return the item pointer, or NULL if all failed */
	return item;
}

/* Work queue management functions */

static inline void
destroy_work_item(work_q_item_t *item)
{
	px4_sem_destroy(&item->wait_sem); /* Destroy the item lock */
	/* Return the item to the free item queue for later reuse */
	lock_queue(&g_free_q);
	sq_addfirst(&item->link, &(g_free_q.q));

	/* Update the queue size and potentially the maximum queue size */
	if (++g_free_q.size > g_free_q.max_size) {
		g_free_q.max_size = g_free_q.size;
	}

	unlock_queue(&g_free_q);
}

static inline work_q_item_t *
dequeue_work_item(void)
{
	work_q_item_t *work;

	/* retrieve the 1st item on the work queue */
	lock_queue(&g_work_q);

	if ((work = (work_q_item_t *)sq_remfirst(&g_work_q.q))) {
		g_work_q.size--;
	}

	unlock_queue(&g_work_q);
	return work;
}

static int
enqueue_work_item_and_wait_for_result(work_q_item_t *item)
{
	/* put the work item at the end of the work queue */
	lock_queue(&g_work_q);
	sq_addlast(&item->link, &(g_work_q.q));

	/* Adjust the queue size and potentially the maximum queue size */
	if (++g_work_q.size > g_work_q.max_size) {
		g_work_q.max_size = g_work_q.size;
	}

	unlock_queue(&g_work_q);

	/* tell the work thread that work is available */
	px4_sem_post(&g_work_queued_sema);

	/* wait for the result */
	px4_sem_wait(&item->wait_sem);

	int result = item->result;

	destroy_work_item(item);

	return result;
}

static bool is_running(void)
{
	return dm_operations_data.running;
}

/* Calculate the offset in file of specific item */
static int
calculate_offset(dm_item_t item, unsigned index)
{

	/* Make sure the item type is valid */
	if (item >= DM_KEY_NUM_KEYS) {
		return -1;
	}

	/* Make sure the index for this item type is valid */
	if (index >= g_per_item_max_index[item]) {
		return -1;
	}

	/* Calculate and return the item index based on type and index */
	return g_key_offsets[item] + (index * k_sector_size);
}

/* Each data item is stored as follows
 *
 * byte 0: Length of user data item
 * byte 1: Persistence of this data item
 * byte 2: Unused (for future use)
 * byte 3: Unused (for future use)
 * byte DM_SECTOR_HDR_SIZE... : data item value
 *
 * The total size must not exceed k_sector_size
 */

/* write to the data manager RAM buffer  */
static ssize_t _ram_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf,
			  size_t count)
{

	/* Get the offset for this item */
	int offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure caller has not given us more data than we can handle */
	if (count > DM_MAX_DATA_SIZE) {
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

/* write to the data manager file */
static ssize_t
_file_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count)
{
	unsigned char buffer[k_sector_size];
	size_t len;
	int offset;

	/* Get the offset for this item */
	offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure caller has not given us more data than we can handle */
	if (count > DM_MAX_DATA_SIZE) {
		return -E2BIG;
	}

	/* Write out the data, prefixed with length and persistence level */
	buffer[0] = count;
	buffer[1] = persistence;
	buffer[2] = 0;
	buffer[3] = 0;

	if (count > 0) {
		memcpy(buffer + DM_SECTOR_HDR_SIZE, buf, count);
	}

	count += DM_SECTOR_HDR_SIZE;

	len = -1;

	/* Seek to the right spot in the data manager file and write the data item */
	if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) == offset) {
		if ((len = write(dm_operations_data.file.fd, buffer, count)) == count) {
			fsync(dm_operations_data.file.fd);        /* Make sure data is written to physical media */
		}
	}

	/* Make sure the write succeeded */
	if (len != count) {
		return -1;
	}

	/* All is well... return the number of user data written */
	return count - DM_SECTOR_HDR_SIZE;
}

/* Retrieve from the data manager RAM buffer*/
static ssize_t _ram_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	/* Get the offset for this item */
	int offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure the caller hasn't asked for more data than we can handle */
	if (count > DM_MAX_DATA_SIZE) {
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

/* Retrieve from the data manager file */
static ssize_t
_file_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	unsigned char buffer[k_sector_size];
	int len, offset;

	/* Get the offset for this item */
	offset = calculate_offset(item, index);

	/* If item type or index out of range, return error */
	if (offset < 0) {
		return -1;
	}

	/* Make sure the caller hasn't asked for more data than we can handle */
	if (count > DM_MAX_DATA_SIZE) {
		return -E2BIG;
	}

	/* Read the prefix and data */
	len = -1;

	if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) == offset) {
		len = read(dm_operations_data.file.fd, buffer, count + DM_SECTOR_HDR_SIZE);
	}

	/* Check for read error */
	if (len < 0) {
		return -1;
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

static int  _ram_clear(dm_item_t item)
{
	int i;
	int result = 0;

	/* Get the offset of 1st item of this type */
	int offset = calculate_offset(item, 0);

	/* Check for item type out of range */
	if (offset < 0) {
		return -1;
	}

	/* Clear all items of this type */
	for (i = 0; (unsigned)i < g_per_item_max_index[item]; i++) {
		uint8_t *buf = &dm_operations_data.ram.data[offset];

		if (buf > dm_operations_data.ram.data_end) {
			result = -1;
			break;
		}

		buf[0] = 0;
		offset += k_sector_size;
	}

	return result;
}

static int
_file_clear(dm_item_t item)
{
	int i, result = 0;

	/* Get the offset of 1st item of this type */
	int offset = calculate_offset(item, 0);

	/* Check for item type out of range */
	if (offset < 0) {
		return -1;
	}

	/* Clear all items of this type */
	for (i = 0; (unsigned)i < g_per_item_max_index[item]; i++) {
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

		offset += k_sector_size;
	}

	/* Make sure data is actually written to physical media */
	fsync(dm_operations_data.file.fd);
	return result;
}


/* Tell the data manager about the type of the last reset */
static int  _ram_restart(dm_reset_reason reason)
{
	int offset = 0;
	int result = 0;

	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */
	while (1) {

		/* Get data segment at current offset */
		uint8_t *buffer = &dm_operations_data.ram.data[offset];

		if (buffer >= dm_operations_data.ram.data_end) {
			break;
		}

		/* check if segment contains data */
		if (buffer[0]) {
			int clear_entry = 0;

			/* Whether data gets deleted depends on reset type and data segment's persistence setting */
			if (reason == DM_INIT_REASON_POWER_ON) {
				if (buffer[1] > DM_PERSIST_POWER_ON_RESET) {
					clear_entry = 1;
				}

			} else {
				if (buffer[1] > DM_PERSIST_IN_FLIGHT_RESET) {
					clear_entry = 1;
				}
			}

			/* Set segment to unused if data does not persist */
			if (clear_entry) {
				buffer[0] = 0;
			}
		}

		offset += k_sector_size;
	}

	/* tell the caller how it went */
	return result;
}

static int
_file_restart(dm_reset_reason reason)
{
	unsigned char buffer[2];
	int offset = 0, result = 0;

	/* We need to scan the entire file and invalidate and data that should not persist after the last reset */

	/* Loop through all of the data segments and delete those that are not persistent */
	while (1) {
		size_t len;

		/* Get data segment at current offset */
		if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
			/* must be at eof */
			break;
		}

		len = read(dm_operations_data.file.fd, buffer, sizeof(buffer));

		if (len != sizeof(buffer)) {
			/* must be at eof */
			break;
		}

		/* check if segment contains data */
		if (buffer[0]) {
			int clear_entry = 0;

			/* Whether data gets deleted depends on reset type and data segment's persistence setting */
			if (reason == DM_INIT_REASON_POWER_ON) {
				if (buffer[1] > DM_PERSIST_POWER_ON_RESET) {
					clear_entry = 1;
				}

			} else {
				if (buffer[1] > DM_PERSIST_IN_FLIGHT_RESET) {
					clear_entry = 1;
				}
			}

			/* Set segment to unused if data does not persist */
			if (clear_entry) {
				if (lseek(dm_operations_data.file.fd, offset, SEEK_SET) != offset) {
					result = -1;
					break;
				}

				buffer[0] = 0;

				len = write(dm_operations_data.file.fd, buffer, 1);

				if (len != 1) {
					result = -1;
					break;
				}
			}
		}

		offset += k_sector_size;
	}

	fsync(dm_operations_data.file.fd);

	/* tell the caller how it went */
	return result;
}

static int
_file_initialize(unsigned max_offset)
{
	/* See if the data manage file exists and is a multiple of the sector size */
	dm_operations_data.file.fd = open(k_data_manager_device_path, O_RDONLY | O_BINARY);

	if (dm_operations_data.file.fd >= 0) {
		// Read the mission state and check the hash
		struct dataman_compat_s compat_state;
		int ret = g_dm_ops->read(DM_KEY_COMPAT, 0, &compat_state, sizeof(compat_state));

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
	int ret = g_dm_ops->write(DM_KEY_COMPAT, 0, DM_PERSIST_POWER_ON_RESET, &compat_state, sizeof(compat_state));

	if (ret != sizeof(compat_state)) {
		PX4_ERR("Failed writing compat: %d", ret);
	}

	fsync(dm_operations_data.file.fd);
	dm_operations_data.running = true;

	return 0;
}

static int
_ram_initialize(unsigned max_offset)
{
	/* In memory */
	dm_operations_data.ram.data = malloc(max_offset);

	if (dm_operations_data.ram.data == NULL) {
		PX4_WARN("Could not allocate %d bytes of memory", max_offset);
		px4_sem_post(&g_init_sema); /* Don't want to hang startup */
		return -1;
	}

	memset(dm_operations_data.ram.data, 0, max_offset);
	dm_operations_data.ram.data_end = &dm_operations_data.ram.data[max_offset - 1];
	dm_operations_data.running = true;

	return 0;
}

static void
_file_shutdown(void)
{
	close(dm_operations_data.file.fd);
	dm_operations_data.running = false;
}

static void
_ram_shutdown(void)
{
	free(dm_operations_data.ram.data);
	dm_operations_data.running = false;
}

/** Write to the data manager file */
__EXPORT ssize_t
dm_write(dm_item_t item, unsigned index, dm_persitence_t persistence, const void *buf, size_t count)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	/* get a work item and queue up a write request */
	if ((work = create_work_item()) == NULL) {
		return -1;
	}

	work->func = dm_write_func;
	work->write_params.item = item;
	work->write_params.index = index;
	work->write_params.persistence = persistence;
	work->write_params.buf = buf;
	work->write_params.count = count;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	return (ssize_t)enqueue_work_item_and_wait_for_result(work);
}

/** Retrieve from the data manager file */
__EXPORT ssize_t
dm_read(dm_item_t item, unsigned index, void *buf, size_t count)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	/* get a work item and queue up a read request */
	if ((work = create_work_item()) == NULL) {
		return -1;
	}

	work->func = dm_read_func;
	work->read_params.item = item;
	work->read_params.index = index;
	work->read_params.buf = buf;
	work->read_params.count = count;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	return (ssize_t)enqueue_work_item_and_wait_for_result(work);
}

/** Clear a data Item */
__EXPORT int
dm_clear(dm_item_t item)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	/* get a work item and queue up a clear request */
	if ((work = create_work_item()) == NULL) {
		return -1;
	}

	work->func = dm_clear_func;
	work->clear_params.item = item;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	return enqueue_work_item_and_wait_for_result(work);
}

/** Lock a data Item */
__EXPORT void
dm_lock(dm_item_t item)
{
	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return;
	}

	if (item >= DM_KEY_NUM_KEYS) {
		return;
	}

	if (g_item_locks[item]) {
		px4_sem_wait(g_item_locks[item]);
	}
}

/** Unlock a data Item */
__EXPORT void
dm_unlock(dm_item_t item)
{
	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return;
	}

	if (item >= DM_KEY_NUM_KEYS) {
		return;
	}

	if (g_item_locks[item]) {
		px4_sem_post(g_item_locks[item]);
	}
}

/** Tell the data manager about the type of the last reset */
__EXPORT int
dm_restart(dm_reset_reason reason)
{
	work_q_item_t *work;

	/* Make sure data manager has been started and is not shutting down */
	if (!is_running() || g_task_should_exit) {
		return -1;
	}

	/* get a work item and queue up a restart request */
	if ((work = create_work_item()) == NULL) {
		return -1;
	}

	work->func = dm_restart_func;
	work->restart_params.reason = reason;

	/* Enqueue the item on the work queue and wait for the worker thread to complete processing it */
	return enqueue_work_item_and_wait_for_result(work);
}

static int
task_main(int argc, char *argv[])
{
	/* Dataman can use disk or RAM */
	switch (backend) {
	case BACKEND_FILE:
		g_dm_ops = &dm_file_operations;
		break;

	case BACKEND_RAM:
		g_dm_ops = &dm_ram_operations;
		break;

	default:
		PX4_WARN("No valid backend set.");
		return -1;
	}

	work_q_item_t *work;

	/* Initialize global variables */
	g_key_offsets[0] = 0;

	for (unsigned i = 0; i < (DM_KEY_NUM_KEYS - 1); i++) {
		g_key_offsets[i + 1] = g_key_offsets[i] + (g_per_item_max_index[i] * k_sector_size);
	}

	unsigned max_offset = g_key_offsets[DM_KEY_NUM_KEYS - 1] + (g_per_item_max_index[DM_KEY_NUM_KEYS - 1] * k_sector_size);

	for (unsigned i = 0; i < dm_number_of_funcs; i++) {
		g_func_counts[i] = 0;
	}

	/* Initialize the item type locks, for now only DM_KEY_MISSION_STATE supports locking */
	px4_sem_init(&g_sys_state_mutex, 1, 1); /* Initially unlocked */

	for (unsigned i = 0; i < DM_KEY_NUM_KEYS; i++) {
		g_item_locks[i] = NULL;
	}

	g_item_locks[DM_KEY_MISSION_STATE] = &g_sys_state_mutex;

	g_task_should_exit = false;

	init_q(&g_work_q);
	init_q(&g_free_q);

	px4_sem_init(&g_work_queued_sema, 1, 0);

	/* g_work_queued_sema use case is a signal */

	px4_sem_setprotocol(&g_work_queued_sema, SEM_PRIO_NONE);

	int ret = g_dm_ops->initialize(max_offset);

	if (ret) {
		g_task_should_exit = true;
		goto end;
	}

	/* see if we need to erase any items based on restart type */
	int sys_restart_val;

	const char *restart_type_str = "Unkown restart";

	if (param_get(param_find("SYS_RESTART_TYPE"), &sys_restart_val) == OK) {
		if (sys_restart_val == DM_INIT_REASON_POWER_ON) {
			restart_type_str = "Power on restart";
			g_dm_ops->restart(DM_INIT_REASON_POWER_ON);

		} else if (sys_restart_val == DM_INIT_REASON_IN_FLIGHT) {
			restart_type_str = "In flight restart";
			g_dm_ops->restart(DM_INIT_REASON_IN_FLIGHT);
		}
	}

	if (backend == BACKEND_FILE && sys_restart_val != DM_INIT_REASON_POWER_ON) {
		PX4_INFO("%s, data manager file '%s' size is %d bytes",
			 restart_type_str, k_data_manager_device_path, max_offset);

	} else if (backend == BACKEND_RAM) {
		PX4_INFO("%s, data manager RAM size is %d bytes",
			 restart_type_str, max_offset);
	}

	/* Tell startup that the worker thread has completed its initialization */
	px4_sem_post(&g_init_sema);

	/* Start the endless loop, waiting for then processing work requests */
	while (true) {

		/* do we need to exit ??? */
		if (!g_task_should_exit) {
			/* wait for work */
			px4_sem_wait(&g_work_queued_sema);
		}

		/* Empty the work queue */
		while ((work = dequeue_work_item())) {

			/* handle each work item with the appropriate handler */
			switch (work->func) {
			case dm_write_func:
				g_func_counts[dm_write_func]++;
				work->result =
					g_dm_ops->write(work->write_params.item, work->write_params.index, work->write_params.persistence,
							work->write_params.buf,
							work->write_params.count);
				break;

			case dm_read_func:
				g_func_counts[dm_read_func]++;
				work->result =
					g_dm_ops->read(work->read_params.item, work->read_params.index, work->read_params.buf, work->read_params.count);
				break;

			case dm_clear_func:
				g_func_counts[dm_clear_func]++;
				work->result = g_dm_ops->clear(work->clear_params.item);
				break;

			case dm_restart_func:
				g_func_counts[dm_restart_func]++;
				work->result = g_dm_ops->restart(work->restart_params.reason);
				break;

			default: /* should never happen */
				work->result = -1;
				break;
			}

			/* Inform the caller that work is done */
			px4_sem_post(&work->wait_sem);
		}

		/* time to go???? */
		if (g_task_should_exit) {
			break;
		}
	}

	g_dm_ops->shutdown();

	/* The work queue is now empty, empty the free queue */
	for (;;) {
		if ((work = (work_q_item_t *)sq_remfirst(&(g_free_q.q))) == NULL) {
			break;
		}

		if (work->first) {
			free(work);
		}
	}

end:
	backend = BACKEND_NONE;
	destroy_q(&g_work_q);
	destroy_q(&g_free_q);
	px4_sem_destroy(&g_work_queued_sema);
	px4_sem_destroy(&g_sys_state_mutex);

	return 0;
}

static int
start(void)
{
	int task;

	px4_sem_init(&g_init_sema, 1, 0);

	/* g_init_sema use case is a signal */

	px4_sem_setprotocol(&g_init_sema, SEM_PRIO_NONE);

	/* start the worker thread with low priority for disk IO */
	if ((task = px4_task_spawn_cmd("dataman", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT - 10, 1200, task_main, NULL)) <= 0) {
		warn("task start failed");
		return -1;
	}

	/* wait for the thread to actually initialize */
	px4_sem_wait(&g_init_sema);
	px4_sem_destroy(&g_init_sema);

	return 0;
}

static void
status(void)
{
	/* display usage statistics */
	PX4_INFO("Writes   %d", g_func_counts[dm_write_func]);
	PX4_INFO("Reads    %d", g_func_counts[dm_read_func]);
	PX4_INFO("Clears   %d", g_func_counts[dm_clear_func]);
	PX4_INFO("Restarts %d", g_func_counts[dm_restart_func]);
	PX4_INFO("Max Q lengths work %d, free %d", g_work_q.max_size, g_free_q.max_size);
}

static void
stop(void)
{
	/* Tell the worker task to shut down */
	g_task_should_exit = true;
	px4_sem_post(&g_work_queued_sema);
}

static void
usage(void)
{
	PX4_INFO("usage: dataman {start [-f datafile]|[-r]|stop|status|poweronrestart|inflightrestart}");
}

int
dataman_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "start")) {

		if (is_running()) {
			PX4_WARN("dataman already running");
			return -1;
		}

		int ch;
		int dmoptind = 1;
		const char *dmoptarg = NULL;

		/* jump over start and look at options first */

		while ((ch = px4_getopt(argc, argv, "f:r", &dmoptind, &dmoptarg)) != EOF) {
			switch (ch) {
			case 'f':
				if (backend != BACKEND_NONE) {
					PX4_WARN("-f and -r are mutually exclusive");
					usage();
					return -1;
				}

				backend = BACKEND_FILE;
				k_data_manager_device_path = strdup(dmoptarg);
				PX4_INFO("dataman file set to: %s", k_data_manager_device_path);
				break;

			case 'r':
				if (backend != BACKEND_NONE) {
					PX4_WARN("-f and -r are mutually exclusive");
					usage();
					return -1;
				}

				backend = BACKEND_RAM;
				break;


			//no break
			default:
				usage();
				return -1;
			}
		}

		if (backend == BACKEND_NONE) {
			backend = BACKEND_FILE;
			k_data_manager_device_path = strdup(default_device_path);
		}

		start();

		if (!is_running()) {
			PX4_ERR("dataman start failed");
			free(k_data_manager_device_path);
			k_data_manager_device_path = NULL;
			return -1;
		}

		return 0;
	}

	/* Worker thread should be running for all other commands */
	if (!is_running()) {
		PX4_WARN("dataman worker thread not running");
		usage();
		return -1;
	}

	if (!strcmp(argv[1], "stop")) {
		stop();
		free(k_data_manager_device_path);
		k_data_manager_device_path = NULL;

	} else if (!strcmp(argv[1], "status")) {
		status();

	} else if (!strcmp(argv[1], "poweronrestart")) {
		dm_restart(DM_INIT_REASON_POWER_ON);

	} else if (!strcmp(argv[1], "inflightrestart")) {
		dm_restart(DM_INIT_REASON_IN_FLIGHT);

	} else {
		usage();
		return -1;
	}

	return 0;
}
