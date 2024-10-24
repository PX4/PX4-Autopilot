/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
 *   Author: @author Jukka Laitinen <jukkax@ssrc.tii.ae>
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
 * @file usr_mcu_version.c
 * Implementation of generic user-space version API
 */

#ifndef MODULE_NAME
#define MODULE_NAME "usr_mcu_version"
#endif

#include <systemlib/px4_macros.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform/board_determine_hw_info.h>

#include <lib/systemlib/px4_macros.h>

#include <uORB/uORB.h>
#include <uORB/topics/guid.h>
#include <uORB/topics/hw_info.h>
#include <uORB/topics/system_version.h>
#include <uORB/topics/system_version_string.h>

#include "board_config.h"

#define HW_INFO_SIZE 64

#ifndef max
#  define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#  define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

static unsigned hw_version = 0;
static unsigned hw_revision = 0;
static char hw_info[HW_INFO_SIZE] = { 0 };
static uint16_t soc_arch_id = 0;
static mfguid_t device_serial_number = { 0 };

__EXPORT const char *board_get_hw_type_name(void)
{
	return (const char *) hw_info;
}

__EXPORT int board_get_hw_version(void)
{
	return  hw_version;
}

__EXPORT int board_get_hw_revision()
{
	return  hw_revision;
}

__EXPORT void board_get_uuid32(uuid_uint32_t uuid_words)
{

	/* DEPRECATED. Use board_get_px4_guid() */

	uint32_t chip_uuid[PX4_CPU_UUID_WORD32_LENGTH];
	memset((uint8_t *)chip_uuid, 0, PX4_CPU_UUID_WORD32_LENGTH * 4);

	for (unsigned i = 0; i < PX4_CPU_UUID_WORD32_LENGTH; i++) {
		uuid_words[i] = chip_uuid[i];
	}
}

int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	const struct  {
		const char *revstr;
		char rev;
		const char *errata;
	} hw_version_table[] = BOARD_REVISIONS;

	unsigned len = sizeof(hw_version_table) / sizeof(hw_version_table[0]);

	if (hw_version < len) {
		*rev = hw_version_table[hw_version].rev;
		*revstr = hw_version_table[hw_version].revstr;
		*errata = hw_version_table[hw_version].errata;
	}

	return hw_version;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	uint8_t *pb = (uint8_t *) &px4_guid[0];

	memset(pb, 0, sizeof(px4_guid_t));

	static_assert(sizeof(device_serial_number) == 16);
	static_assert(sizeof(px4_guid_t) >= sizeof(device_serial_number) + 2);

	*pb++ = (soc_arch_id >> 8) & 0xff;
	*pb++ = (soc_arch_id & 0xff);

	memcpy(pb, device_serial_number, sizeof(device_serial_number));

	return PX4_GUID_BYTE_LENGTH;
}

int board_get_px4_guid_formated(char *format_buffer, int size)
{
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	int offset  = 0;

	/* size should be 2 per byte + 1 for termination
	 * So it needs to be odd
	 */
	size = size & 1 ? size : size - 1;

	/* Discard from MSD */
	for (unsigned i = PX4_GUID_BYTE_LENGTH - size / 2; offset < size && i < PX4_GUID_BYTE_LENGTH; i++) {
		offset += snprintf(&format_buffer[offset], size - offset, "%02x", px4_guid[i]);
	}

	return offset;
}

int board_determine_hw_info(void)
{
	orb_sub_t hwinfo_sub;
	orb_sub_t guid_sub;
	orb_sub_t system_version_sub;
	struct system_version_s ver;
	struct guid_s guid;
	struct hw_info_s hwinfo;

	/* System hw_version, hw_revision, soc_arch_id */
	system_version_sub = orb_subscribe(ORB_ID(system_version));

	if (!orb_sub_valid(system_version_sub)) {
		PX4_ERR("Failed to subscribe to system version (%i)", errno);
		return -1;
	}

	orb_copy(ORB_ID(system_version), system_version_sub, &ver);
	orb_unsubscribe(system_version_sub);

	hw_version = ver.hw_version;
	hw_revision = ver.hw_revision;
	soc_arch_id = ver.soc_arch_id;

	/* Hardware information string */
	hwinfo_sub = orb_subscribe(ORB_ID(hw_info));

	if (!orb_sub_valid(hwinfo_sub)) {
		PX4_ERR("Failed to subscribe to hwinfo_sub (%i)", errno);
		return -1;
	}

	orb_copy(ORB_ID(hw_info), hwinfo_sub, &hwinfo);
	orb_unsubscribe(hwinfo_sub);

	memcpy(hw_info, hwinfo.hw_info,  min(sizeof(hwinfo.hw_info), sizeof(hw_info)));

	/* Drone guid  */
	guid_sub = orb_subscribe(ORB_ID(guid));

	if (!orb_sub_valid(guid_sub)) {
		PX4_ERR("Failed to subscribe to guid (%i)", errno);
		return -1;
	}

	orb_copy(ORB_ID(guid), guid_sub, &guid);
	orb_unsubscribe(guid_sub);

	memcpy(device_serial_number, &guid, min(sizeof(device_serial_number), sizeof(guid)));

	return 0;
}
