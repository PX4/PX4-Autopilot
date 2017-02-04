/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_identity.c
 * Implementation of STM32 based Board identity API
 */

#include <px4_config.h>
#include <stdio.h>
#include <string.h>

const raw_uuid_uint32_reorder_t px4_legacy_word32_order = PX4_CPU_UUID_WORD32_LEGACY_FORMAT_ORDER;

void board_get_uuid_raw(raw_uuid_byte_t *raw_uuid)
{
	memcpy(raw_uuid, (uint8_t *) STM32_SYSMEM_UID, PX4_CPU_UUID_BYTE_LENGTH);
}

void board_get_uuid(raw_uuid_byte_t uuid, uuid_uint8_reorder_t reorder)
{
	raw_uuid_byte_t raw_uuid;
	board_get_uuid_raw(&raw_uuid);

	for (int i = 0; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		uuid[i] = raw_uuid[reorder[i]];
	}
}

__EXPORT void board_get_uuid_raw32(raw_uuid_uint32_t raw_uuid_words,
				   raw_uuid_uint32_reorder_t *optional_reorder)
{
	if (optional_reorder == NULL) {
		optional_reorder = &px4_legacy_word32_order;
	}

	uint32_t *chip_uuid = (uint32_t *) STM32_SYSMEM_UID;

	for (int i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		raw_uuid_words[i] = chip_uuid[(*optional_reorder)[i]];
	}
}

int board_get_uuid_formated32(char *format_buffer, int size,
			      const char *format,
			      const char *seperator,
			      raw_uuid_uint32_reorder_t *optional_reorder)
{
	raw_uuid_uint32_t uuid;
	board_get_uuid_raw32(uuid, optional_reorder);
	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (int i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - ((i * 2 * sizeof(uint32_t)) + 1), format, uuid[i]);

		if (sep_size && i < PX4_CPU_UUID_WORD32_LENGTH - 1) {
			strcat(&format_buffer[offset], seperator);
			offset += sep_size;
		}
	}

	return 0;
}
