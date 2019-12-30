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

#include <px4_platform_common/defines.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <atomic_ops.h>

#include <parameters/param.h>

#include <px4_platform_common/shmem.h>
#include <drivers/drv_hrt.h>

//#define SHMEM_DEBUG
//#define PARAM_LOCK_DEBUG

static atomic_word_t mem_lock;

static unsigned char *map_base, *virt_addr;
static uint64_t update_from_shmem_prev_time = 0, update_from_shmem_current_time = 0;
static unsigned char krait_changed_index[MAX_SHMEM_PARAMS / 8 + 1];

struct shmem_info *shmem_info_p;

// Small helper to get log2 for ints
static unsigned log2_for_int(unsigned v)
{
	unsigned r = 0;

	while (v >>= 1) {
		++r;
	}

	return r;
}

struct param_wbuf_s {
	union param_value_u val;
	param_t param;
	bool unsaved;
};
extern struct param_wbuf_s *param_find_changed(param_t param);

int get_shmem_lock(const char *caller_file_name, int caller_line_number)
{
	unsigned int i = 0;

#ifdef PARAM_LOCK_DEBUG
	PX4_INFO("lock value %d before get from %s, line: %d\n", mem_lock.value, strrchr(caller_file_name, '/'),
		 caller_line_number);
#endif

	while (!atomic_compare_and_set(&mem_lock, 1, 0)) {
		i++;
		usleep(1000);

		if (i > 100) {
			break;
		}
	}

	if (i > 100) {
		PX4_INFO("Could not get lock, file name: %s, line number: %d.\n",
			 strrchr(caller_file_name, '/'), caller_line_number);
		return -1;

	} else {
		PX4_DEBUG("Lock acquired, file name: %s, line number: %d\n",
			  caller_file_name, caller_line_number);
	}

	return 0; //got the lock

}

void release_shmem_lock(const char *caller_file_name, int caller_line_number)
{
	atomic_set(&mem_lock, 1);

#ifdef PARAM_LOCK_DEBUG
	PX4_INFO("release lock, file name: %s, line number: %d.\n",
		 strrchr(caller_file_name, '/'), caller_line_number);
#endif

	return;
}

void init_shared_memory(void)
{
	int i;

	if (shmem_info_p) {
		return;
	}

	//virt_addr = map_memory(MAP_ADDRESS);
	map_base = (unsigned char *)calloc(MAP_SIZE, 1); //16KB

	if (map_base == NULL) {
		PX4_INFO("adsp memory malloc failed\n");
		return;
	}

	virt_addr = map_base;
	shmem_info_p = (struct shmem_info *) virt_addr;

	atomic_init(&mem_lock, 1);

	for (i = 0; i < MAX_SHMEM_PARAMS / 8 + 1; i++) {
		shmem_info_p->krait_changed_index[i] = 0;
	}

	PX4_INFO("adsp memory mapped\n");
}

void copy_params_to_shmem(const param_info_s *param_info_base)
{
	param_t param;
	unsigned int i;

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_INFO("Could not get shmem lock\n");
		return;
	}

	//else PX4_INFO("Got lock\n");

	for (param = 0; param < param_count(); param++) {
		//{PX4_INFO("writing to offset %d\n", (unsigned char*)(shmem_info_p->adsp_params[param].name)-(unsigned char*)shmem_info_p);}
		struct param_wbuf_s *s = param_find_changed(param);

		if (s == NULL) {
			shmem_info_p->params_val[param] = param_info_base[param].val;
		}

		else {
			shmem_info_p->params_val[param] = s->val;
		}

#ifdef SHMEM_DEBUG

		if (param_type(param) == PARAM_TYPE_INT32) {
			PX4_INFO("%d: written %d for param %s to shared mem", param, shmem_info_p->params_val[param].i, param_name(param));

		} else if (param_type(param) == PARAM_TYPE_FLOAT) {
			PX4_INFO("%d: written %f for param %s to shared mem", param, shmem_info_p->params_val[param].f, param_name(param));
		}

#endif
	}

	for (i = 0; i < MAX_SHMEM_PARAMS / 8 + 1; i++) {
		shmem_info_p->adsp_changed_index[i] = 0;
		krait_changed_index[i] = 0;
	}

	release_shmem_lock(__FILE__, __LINE__);
	//PX4_INFO("Released lock\n");

}

/*update value and param's change bit in shared memory*/
void update_to_shmem(param_t param, union param_value_u value)
{
	unsigned int byte_changed, bit_changed;

	if (!handle_in_range(param)) {
		return;
	}

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_ERR("Could not get shmem lock\n");
		return;
	}

	shmem_info_p->params_val[param] = value;

	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->adsp_changed_index[byte_changed] |= bit_changed;

	//PX4_INFO("set %d bit on adsp index[%d] to %d\n", bit_changed, byte_changed, shmem_info_p->adsp_changed_index[byte_changed]);

#ifdef SHMEM_DEBUG

	if (param_type(param) == PARAM_TYPE_INT32) {
		PX4_INFO("Set value %d for param %s to shmem, set adsp index %d:%d\n", value.i, param_name(param), byte_changed,
			 bit_changed);
	}

	else if (param_type(param) == PARAM_TYPE_FLOAT) {
		PX4_INFO("Set value %f for param %s to shmem, set adsp index %d:%d\n", value.f, param_name(param), byte_changed,
			 bit_changed);
	}

#endif

	release_shmem_lock(__FILE__, __LINE__);

}

void update_index_from_shmem(void)
{
	unsigned int i;
	param_t params[MAX_SHMEM_PARAMS / 8 + 1];

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_ERR("Could not get shmem lock\n");
		return;
	}

	for (i = 0; i < MAX_SHMEM_PARAMS / 8 + 1; i++) {
		// Check if any param has been changed.
		if (krait_changed_index[i] != shmem_info_p->krait_changed_index[i]) {

			// If a param has changed, we need to find out which one.
			// From the byte and bit that is different, we can resolve the param number.
			unsigned bit = log2_for_int(
					       krait_changed_index[i]
					       ^ shmem_info_p->krait_changed_index[i]);
			param_t param_to_get = i * 8 + bit;

			// Update our krait_changed_index as well.
			krait_changed_index[i] = shmem_info_p->krait_changed_index[i];
			params[i] = param_to_get;

		} else {
			params[i] = 0xFFFF;
		}
	}

	release_shmem_lock(__FILE__, __LINE__);

	// FIXME: this is a hack but it gets the param so that it gets added
	// to the local list param_values in param_shmem.c.
	for (i = 0; i < MAX_SHMEM_PARAMS / 8 + 1; i++) {
		if (params[i] != 0xFFFF) {
			int32_t dummy;
			param_get(params[i], &dummy);
		}
	}
}

static void update_value_from_shmem(param_t param, union param_value_u *value)
{
	unsigned int byte_changed, bit_changed;

	if (get_shmem_lock(__FILE__, __LINE__) != 0) {
		PX4_ERR("Could not get shmem lock\n");
		return;
	}

	*value = shmem_info_p->params_val[param];

	/*also clear the index since we are holding the lock*/
	byte_changed = param / 8;
	bit_changed = 1 << param % 8;
	shmem_info_p->krait_changed_index[byte_changed] &= ~bit_changed;

	release_shmem_lock(__FILE__, __LINE__);

#ifdef SHMEM_DEBUG

	if (param_type(param) == PARAM_TYPE_INT32) {
		PX4_INFO("Got value %d for param %s from shmem, cleared krait index %d:%d\n", value->i, param_name(param), byte_changed,
			 bit_changed);
	}

	else if (param_type(param) == PARAM_TYPE_FLOAT) {
		PX4_INFO("Got value %f for param %s from shmem, cleared krait index %d:%d\n", value->f, param_name(param), byte_changed,
			 bit_changed);
	}

#endif
}

int update_from_shmem(param_t param, union param_value_u *value)
{
	unsigned int byte_changed, bit_changed;
	unsigned int retval = 0;

	if (!handle_in_range(param) || value == NULL) {
		return retval;
	}

	update_from_shmem_current_time = hrt_absolute_time();

	if ((update_from_shmem_current_time - update_from_shmem_prev_time)
	    > 1000000) { //update every 1 second
		update_from_shmem_prev_time = update_from_shmem_current_time;
		update_index_from_shmem();
	}

	byte_changed = param / 8;
	bit_changed = 1 << param % 8;

	if (krait_changed_index[byte_changed] & bit_changed) {
		update_value_from_shmem(param, value);
		krait_changed_index[byte_changed] &= ~bit_changed;
		retval = 1;
	}

	//else {PX4_INFO("no change to param %s\n", param_name(param));}

	PX4_DEBUG("%s %d bit on krait changed index[%d]\n",
		  (retval) ? "cleared" : "unchanged", bit_changed, byte_changed);

	return retval;
}
