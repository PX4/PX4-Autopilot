/****************************************************************************
 *
 *   Copyright (c) 2020 Technology Innovation Institute. All rights reserved.
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

#include "hw_config.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "image_toc.h"

#include "bl.h"

/* Helper macros to define flash start and end addresses, based on info from
 * hw_config.h
 */
#define FLASH_START_ADDRESS (APP_LOAD_ADDRESS & (~(BOARD_FLASH_SIZE - 1)))
#define FLASH_END_ADDRESS (FLASH_START_ADDRESS + BOARD_FLASH_SIZE)

bool find_toc(const image_toc_entry_t **toc_entries, uint8_t *len)
{
	const uintptr_t toc_start_u32 = APP_LOAD_ADDRESS + BOOT_DELAY_ADDRESS + 8;
	const image_toc_start_t *toc_start = (const image_toc_start_t *)toc_start_u32;
	const image_toc_entry_t *entry = (const image_toc_entry_t *)(toc_start_u32 + sizeof(image_toc_start_t));

	int i = 0;
	uint8_t sig_idx;
	const uint32_t toc_end_magic = TOC_END_MAGIC;

	if (toc_start->magic == TOC_START_MAGIC &&
	    toc_start->version <= TOC_VERSION) {

		/* Count the entries in TOC */
		while (i < MAX_TOC_ENTRIES &&
		       (uintptr_t)&entry[i] <= FLASH_END_ADDRESS - sizeof(uintptr_t) &&
		       memcmp(&entry[i], &toc_end_magic, sizeof(toc_end_magic))) {
			i++;
		}

		/* The number of toc entries found must be within bounds, and the
		 * application has to lie within the flashable area. Also ensure that
		 * end > start.
		 */

		if (i <= MAX_TOC_ENTRIES && i > 0 &&
		    (uintptr_t)entry[0].start == APP_LOAD_ADDRESS &&
		    (uintptr_t)entry[0].end <= (FLASH_END_ADDRESS - sizeof(uintptr_t)) &&
		    (uintptr_t)entry[0].end > (uintptr_t)entry[0].start) {
			sig_idx = entry[0].signature_idx;

			/* The signature idx for the first app must be within the TOC, and
			* the signature must be within the flash area, not overlapping the
			* app. Also ensure that end > start.
			*/

			if (sig_idx > 0 &&
			    sig_idx < MAX_TOC_ENTRIES &&
			    (uintptr_t)entry[sig_idx].start >= (uintptr_t)entry[0].end &&
			    (uintptr_t)entry[sig_idx].end <= FLASH_END_ADDRESS &&
			    (uintptr_t)entry[sig_idx].end > (uintptr_t)entry[sig_idx].start) {
				*toc_entries = entry;
				*len = i;
				return true;
			}
		}
	}

	*toc_entries = NULL;
	*len = 0;
	return false;
}
