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
 * Implementation of Non Arch specific Board identity API
 */

#include <px4_platform_common/config.h>
#include <stdio.h>
#include <string.h>
#if defined(BOARD_OVERRIDE_UUID) || defined(BOARD_OVERRIDE_MFGUID) || defined(BOARD_OVERRIDE_PX4_GUID)
static const uint16_t soc_arch_id = PX4_SOC_ARCH_ID;
static const char board_uuid[17] = BOARD_OVERRIDE_UUID;

void board_get_uuid32(uuid_uint32_t uuid_words)
{
	unsigned int len = strlen(board_uuid);

	if (len > PX4_CPU_UUID_BYTE_LENGTH) {
		len = PX4_CPU_UUID_BYTE_LENGTH;
	}

	uint8_t *bp = (uint8_t *) uuid_words;

	for (unsigned int i = 0; i < len; i++) {
		*bp++ = board_uuid[i];
	}

	for (unsigned int i = len; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		*bp++ = '0';
	}
}

int board_get_uuid32_formated(char *format_buffer, int size,
			      const char *format,
			      const char *seperator)
{
	uuid_uint32_t uuid;
	board_get_uuid32(uuid);
	int offset = 0;
	int sep_size = seperator ? strlen(seperator) : 0;

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, format, uuid[i]);

		if (sep_size && i < PX4_CPU_UUID_WORD32_LENGTH - 1) {
			strcat(&format_buffer[offset], seperator);
			offset += sep_size;
		}
	}

	return 0;
}

int board_get_mfguid_formated(char *format_buffer, int size)
{
	board_get_uuid32_formated(format_buffer, size, "%02x", NULL);
	return strlen(format_buffer);
}


int board_get_px4_guid_formated(char *format_buffer, int size)
{
	int offset = snprintf(format_buffer, size, "%04x", soc_arch_id);
	size -= offset;
	return board_get_mfguid_formated(&format_buffer[offset], size);
}
#endif
