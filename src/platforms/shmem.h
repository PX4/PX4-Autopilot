/****************************************************************************
 *
 * Copyright (c) 2015 Ramakrishna Kintada. All rights reserved.
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

#define MAX_SHMEM_PARAMS 3850 //MAP_SIZE - (LOCK_SIZE - sizeof(struct shmem_info))

struct shmem_info {
	union param_value_u params_val[MAX_SHMEM_PARAMS];
	unsigned char krait_changed_index[MAX_SHMEM_PARAMS / 8 + 1]; /*bit map of all params changed by krait*/
	unsigned char adsp_changed_index[MAX_SHMEM_PARAMS / 8 + 1]; /*bit map of all params changed by adsp*/

#ifdef __PX4_NUTTX
};
#else
} __attribute__((packed));
#endif

#define MAP_ADDRESS	0xfbfc000
#define MAP_SIZE 	16384
#define MAP_MASK 	(MAP_SIZE - 1)

#define LOCK_OFFSET	0
#define LOCK_SIZE	4
#define LOCK_MEM        1
#define UNLOCK_MEM      2

#define TYPE_MASK 	0x1

extern bool handle_in_range(param_t);
