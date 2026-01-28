/****************************************************************************
 *
 *   Copyright (c) 2014, 2021 Andrew Tridgell. All rights reserved.
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
 * @file reflect.c
 *
 * simple data reflector for load testing terminals (especially USB)
 *
 * @author Andrew Tridgell
 */

#include <px4_platform_common/px4_config.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <systemlib/err.h>

__EXPORT int reflect_main(int argc, char *argv[]);

// memory corruption checking
#define MAX_BLOCKS 1000
struct block {
	uint32_t v[256];
};

#define VALUE(i) ((i*7) ^ 0xDEADBEEF)

static uint32_t allocate_blocks(struct block **blocks)
{
	uint32_t nblocks = 0;

	while (nblocks < MAX_BLOCKS) {
		blocks[nblocks] = calloc(1, sizeof(struct block));

		if (blocks[nblocks] == NULL) {
			break;
		}

		for (uint32_t i = 0; i < sizeof(blocks[nblocks]->v) / sizeof(uint32_t); i++) {
			blocks[nblocks]->v[i] = VALUE(i);
		}

		nblocks++;
	}

	printf("Allocated %" PRIu32 " blocks\n", nblocks);

	return nblocks;
}

static void check_blocks(struct block **blocks, uint32_t nblocks)
{
	for (uint32_t n = 0; n < nblocks; n++) {
		for (uint32_t i = 0; i < sizeof(blocks[nblocks]->v) / sizeof(uint32_t); i++) {
			assert(blocks[n]->v[i] == VALUE(i));
		}
	}
}

int
reflect_main(int argc, char *argv[])
{
	uint32_t total = 0;
	uint32_t nblocks = 0;
	printf("Starting reflector\n");

	struct block **blocks = NULL;
	blocks = malloc(sizeof(struct block *) * MAX_BLOCKS);

	if (blocks == NULL) {
		return -1;
	}

	while (nblocks < MAX_BLOCKS) {
		blocks[nblocks] = NULL;
		nblocks++;
	}

	nblocks = allocate_blocks(blocks);

	while (true) {
		char buf[128];
		ssize_t n = read(0, buf, sizeof(buf));

		if (n < 0) {
			break;
		}

		if (n > 0) {
			if (write(1, buf, n) < 0) {
				return -1;
			}
		}

		total += n;

		if (total > 1024000) {
			check_blocks(blocks, nblocks);
			total = 0;
		}
	}

	return OK;
}
