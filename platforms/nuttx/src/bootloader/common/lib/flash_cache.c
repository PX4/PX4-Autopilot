/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <string.h>
#include "flash_cache.h"

#include "hw_config.h"

#include <nuttx/progmem.h>

extern ssize_t arch_flash_write(uintptr_t address, const void *buffer, size_t buflen);

flash_cache_line_t flash_cache[FC_NUMBER_LINES];


static inline void fcl_reset(flash_cache_line_t *fcl)
{
	memset(fcl, 0xff, sizeof(flash_cache_line_t));
}

inline void fc_reset(void)
{
	for (unsigned w = 0; w < FC_NUMBER_LINES; w++) {
		fcl_reset(&flash_cache[w]);
	}

	flash_cache[0].start_address = APP_LOAD_ADDRESS;
}

static inline flash_cache_line_t *fc_line_select(uintptr_t address)
{
	for (unsigned w = 0; w < FC_NUMBER_LINES; w++) {
		if (flash_cache[w].start_address == (address & FC_ADDRESS_MASK)) {
			return &flash_cache[w];
		}
	}

	return NULL;
}

inline int fc_is_dirty(flash_cache_line_t *fl)
{
	return fl->index != FC_CLEAN;
}


int fc_flush(flash_cache_line_t *fl)
{
	const size_t bytes = sizeof(fl->words);
	size_t rv = arch_flash_write(fl->start_address, fl->words, bytes);

	if (rv == bytes) {
		rv = 0;
	}

	return rv;
}

int fc_write(uintptr_t address, uint32_t word)
{
	flash_cache_line_t *fc = fc_line_select(address);
	flash_cache_line_t *fc1 = &flash_cache[1];
	uint32_t index = FC_ADDR2INDX(address);
	int rv = 0;

	if (fc == NULL && index == 0) {
		fc = fc1;
		fc->start_address = address;
	}

	if (fc) {

		fc->words[index] = word;

		// Are we back writing the first word?

		if (fc == &flash_cache[0] &&  index == 0 && fc->index == 7) {

			if (fc_is_dirty(fc1)) {

				// write out last fragment of data

				rv = fc_flush(fc1);

				if (rv != 0) {
					fcl_reset(fc1);
					return -1;
				}
			}

			rv = fc_flush(fc);
			fcl_reset(fc);
			return rv;
		}

		fc->index = index;
	}

	return rv;
}

uint32_t fc_read(uintptr_t address)
{
	// Assume a cache miss read from FLASH memory

	uint32_t rv = *(uint32_t *) address;

	flash_cache_line_t *fc = fc_line_select(address);

	if (fc) {

		// Cache hit retrieve word from cache

		uint32_t index = FC_ADDR2INDX(address);
		rv = fc->words[index];

		// Reading the last word in cache (not first words)

		if (fc != &flash_cache[0]  && index == FC_LAST_WORD) {
			if (fc_flush(fc)) {
				rv ^= 0xffffffff;

			} else {
				fcl_reset(fc);
			}
		}
	}

	return rv;
}
