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
#include <px4_platform_common/posix.h>
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

#include <parameters/param.h>

#include <shmem.h>
#include "px4muorb.h"

//#define SHMEM_DEBUG

static uint64_t update_from_shmem_prev_time = 0, update_from_shmem_current_time = 0;
extern unsigned char *adsp_changed_index;

struct param_wbuf_s {
	union param_value_u val;
	param_t param;
	bool unsaved;
};

/*update value and param's change bit in shared memory*/
void update_to_shmem(param_t param, union param_value_u value)
{
	if (px4muorb_param_update_to_shmem(param, (unsigned char *) &value, sizeof(value))) {
		PX4_ERR("krait update param %u failed", param);
	}
}

void update_index_from_shmem(void)
{
	if (!adsp_changed_index) {
		PX4_ERR("%s no param buffer", __FUNCTION__);
		return;
	}

	px4muorb_param_update_index_from_shmem(adsp_changed_index, PARAM_BUFFER_SIZE);
}

static void update_value_from_shmem(param_t param, union param_value_u *value)
{
	if (px4muorb_param_update_value_from_shmem(param, (unsigned char *) value, sizeof(union param_value_u))) {
		PX4_ERR("%s get param failed", __FUNCTION__);
	}
}

int update_from_shmem(param_t param, union param_value_u *value)
{
	unsigned int byte_changed, bit_changed;
	unsigned int retval = 0;

	if (!adsp_changed_index) {
		PX4_ERR("%s no param buffer", __FUNCTION__);
		return 0;
	}

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

	//else {PX4_INFO("no change to param %s", param_name(param));}

	PX4_DEBUG("%s %d bit on adsp index[%d]",
		  (retval) ? "cleared" : "unchanged", bit_changed, byte_changed);

	return retval;
}
