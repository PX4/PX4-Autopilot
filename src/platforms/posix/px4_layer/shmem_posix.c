/****************************************************************************
 *
 * Copyright (c) 2015 Vijay Venkatraman. All rights reserved.
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

#include <px4_defines.h>
#include <px4_posix.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <systemlib/err.h>
#include <errno.h>
#include <semaphore.h>

#include <sys/stat.h>

#include <drivers/drv_hrt.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include "systemlib/param/param.h"

#include <shmem.h>

#define SHMEM_DEBUG

int mem_fd;
unsigned char *map_base, *virt_addr;
struct shmem_info *shmem_info_p;
static void *map_memory(off_t target);

int get_shmem_lock(const char *caller_file_name, int caller_line_number);
void release_shmem_lock(void);
void init_shared_memory(void);
void copy_params_to_shmem(struct param_info_s *);
void update_to_shmem(param_t param, union param_value_u value);
int update_from_shmem(param_t param, union param_value_u *value);
uint64_t update_from_shmem_prev_time = 0, update_from_shmem_current_time = 0;
static unsigned char adsp_changed_index[MAX_SHMEM_PARAMS / 8 + 1];

struct param_wbuf_s {
	param_t param;
	union param_value_u val;
	bool unsaved;
};
extern struct param_wbuf_s *param_find_changed(param_t param);

#define MEMDEVICE	"/dev/mem"

static void *map_memory(off_t target)
{

	if ((mem_fd = open(MEMDEVICE, O_RDWR | O_SYNC)) == -1) {
		PX4_ERR("Cannot open %s\n", MEMDEVICE);
		exit(1);
	}

	/* Map one page */
	map_base = (unsigned char *) mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE,
					  MAP_SHARED, mem_fd, target & ~MAP_MASK);

	if (map_base == (void *) - 1) {
		PX4_ERR("Cannot mmap /dev/atl_mem\n");
		exit(1);
	}

	PX4_DEBUG("Initializing map memory: mem_fd: %d, 0x%X", mem_fd, map_base + (target & MAP_MASK) + LOCK_SIZE);

	return (map_base + (target & MAP_MASK) + LOCK_SIZE);

}

int get_shmem_lock(const char *caller_file_name, int caller_line_number)
{
	int i = 0;

	/*ioctl calls cmpxchg*/
	while (ioctl(mem_fd, LOCK_MEM) != 0) {
		PX4_INFO("Could not get lock, file name: %s, line number: %d\n", caller_file_name, caller_line_number);
		usleep(100000); //sleep for 100 msec
		i++;

		if (i > 100) {
			break;
		}
	}

	if (i > 100) {
		return -1;
	}

	return 0; //got the lock
}

void release_shmem_lock(void)
{
	ioctl(mem_fd, UNLOCK_MEM);
}

void init_shared_memory(void)
{

	virt_addr = map_memory(MAP_ADDRESS); //16K space
	shmem_info_p = (struct shmem_info *) virt_addr;

	//PX4_INFO("linux memory mapped\n");
}

void copy_params_to_shmem(struct param_info_s *param_info_base)
{
	param_t param;
	unsigned int i;

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_ERR("Could not get shmem lock\n");
		return;
	}

	//PX4_INFO("%d krait params allocated\n", param_count());
	for (param = 0; param < param_count(); param++) {
		struct param_wbuf_s *s = param_find_changed(param);

		if (s == NULL) {
			shmem_info_p->params_val[param] = param_info_base[param].val;

		} else {
			shmem_info_p->params_val[param] = s->val;
		}

#ifdef SHMEM_DEBUG

		if (param_type(param) == PARAM_TYPE_INT32) {
			{
				PX4_INFO("%d: written %d for param %s to shared mem",
					 param, shmem_info_p->params_val[param].i, param_name(param));
			}

		} else if (param_type(param) == PARAM_TYPE_FLOAT) {
			{
				PX4_INFO("%d: written %f for param %s to shared mem",
					 param, (double)shmem_info_p->params_val[param].f, param_name(param));
			}
		}

#endif
	}

	//PX4_INFO("written %u params to shmem offset %lu\n", param_count(), (unsigned char*)&shmem_info_p->params_count-(unsigned char*)shmem_info_p);

	for (i = 0; i < MAX_SHMEM_PARAMS / 8 + 1; i++) {
		shmem_info_p->krait_changed_index[i] = 0;
		adsp_changed_index[i] = 0;
	}

	release_shmem_lock();
}

/*update value and param's change bit in shared memory*/
void update_to_shmem(param_t param, union param_value_u value)
{
	unsigned int byte_changed, bit_changed;

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		fprintf(stderr, "Could not get shmem lock\n");
		return;
	}

	shmem_info_p->params_val[param] = value;

	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->krait_changed_index[byte_changed] |= bit_changed;

	//PX4_INFO("set %d bit on krait changed index[%d] to %d\n", bit_changed, byte_changed, shmem_info_p->krait_changed_index[byte_changed]);

#ifdef SHMEM_DEBUG

	if (param_type(param) == PARAM_TYPE_INT32) {
		PX4_INFO("Set value %d for param %s to shmem, set krait index %d:%d\n",
			 value.i, param_name(param), byte_changed, bit_changed);

	} else if (param_type(param) == PARAM_TYPE_FLOAT) {
		PX4_INFO("Set value %f for param %s to shmem, set krait index %d:%d\n",
			 (double)value.f, param_name(param), byte_changed, bit_changed);
	}

#endif

	release_shmem_lock();

}

static void update_index_from_shmem(void)
{
	unsigned int i;

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		fprintf(stderr, "Could not get shmem lock\n");
		return;
	}

	//PX4_INFO("Updating index from shmem\n");

	for (i = 0; i < MAX_SHMEM_PARAMS / 8 + 1; i++) {
		adsp_changed_index[i] = shmem_info_p->adsp_changed_index[i];
	}

	release_shmem_lock();
}

static void update_value_from_shmem(param_t param, union param_value_u *value)
{
	unsigned int byte_changed, bit_changed;

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		fprintf(stderr, "Could not get shmem lock\n");
		return;
	}

	*value = shmem_info_p->params_val[param];

	/*also clear the index since we are holding the lock*/
	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->adsp_changed_index[byte_changed] &= ~bit_changed;

	release_shmem_lock();

#ifdef SHMEM_DEBUG

	if (param_type(param) == PARAM_TYPE_INT32) {
		PX4_INFO(
			"Got value %d for param %s from shmem, cleared adsp index %d:%d\n",
			value->i, param_name(param), byte_changed, bit_changed);

	} else if (param_type(param) == PARAM_TYPE_FLOAT) {
		PX4_INFO(
			"Got value %f for param %s from shmem, cleared adsp index %d:%d\n",
			(double)value->f, param_name(param), byte_changed, bit_changed);
	}

#endif
}

int update_from_shmem(param_t param, union param_value_u *value)
{
	unsigned int byte_changed, bit_changed;
	unsigned int retval = 0;

	update_from_shmem_current_time = hrt_absolute_time();

	if ((update_from_shmem_current_time - update_from_shmem_prev_time)
	    > 1000000) { //update every 1 second
		update_from_shmem_prev_time = update_from_shmem_current_time;
		update_index_from_shmem();
	}

	byte_changed = param / 8;
	bit_changed = 1 << param % 8;

	if (adsp_changed_index[byte_changed] & bit_changed) {
		update_value_from_shmem(param, value);
		adsp_changed_index[byte_changed] &= ~bit_changed; //clear the bit
		retval = 1;
	}

	//else {PX4_INFO("no change to param %s\n", param_name(param));}

	//PX4_INFO("%s %d bit on adsp index[%d]\n", (retval)?"cleared":"unchanged", bit_changed, byte_changed);

	return retval;
}

