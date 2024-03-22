/****************************************************************************
 *
 *   Copyright (C) 2024 Technology Innovation Institute. All rights reserved.
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
 * Implementation of MPFS version API
 */

#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/board_determine_hw_info.h>
#include <stdint.h>

#include <board_config.h>
#include <lib/systemlib/px4_macros.h>
#include <px4_arch/device_info.h>

#include <uORB/uORB.h>
#include <uORB/topics/guid.h>
#include <uORB/topics/hw_info.h>
#include <uORB/topics/system_version.h>
#include <uORB/topics/system_version_string.h>

#include <src/lib/version/build_git_version.h>

#ifndef arraySize
#define arraySize(a) (sizeof((a))/sizeof(((a)[0])))
#endif

#define HW_INFO_SIZE (int) arraySize(HW_INFO_INIT_PREFIX) + HW_INFO_VER_DIGITS + HW_INFO_REV_DIGITS + 1

#ifndef max
#  define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#  define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

static unsigned hw_version = 0;
static unsigned hw_revision = 0;
static char hw_info[HW_INFO_SIZE] = {0};

static mfguid_t device_serial_number = { 0 };

devinfo_t device_boot_info __attribute__((section(".deviceinfo")));

static const uint16_t soc_arch_id = PX4_SOC_ARCH_ID_MPFS;

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

const char *board_bl_version_string(void)
{
	return device_boot_info.bl_version;
}

int board_get_px4_guid(px4_guid_t px4_guid)
{
	uint8_t *pb = (uint8_t *) &px4_guid[0];

	memset(pb, 0, sizeof(px4_guid_t));

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

/* Parse git tag string to a 64-bit hex as follows:

 * 0xvvmmrrpphhhhhhhd , where
 * v = major version
 * m = minor version
 * r = revision
 * p = patch version
 * h = git hash
 * d = dirty (0 if clean)
 */

static uint64_t parse_tag_to_version(const char *ver_str)
{
	uint64_t out = 0;
	unsigned len = strlen(ver_str);
	unsigned g_count = 0;
	unsigned dot_count = 0;
	unsigned dash_count = 0;
	uint64_t ver = 0;
	unsigned i;
	char c;

	for (i = 0; i < len; i++) {
		c = ver_str[i];

		if (g_count == 0 &&
		    c >= '0' && c <= '9') {
			ver = ver * 10 + c - '0';
		}

		/* Bits 63-32 are version numbers, 8 bits each (major minor revision patch) */

		if (c == '.' || (dash_count == 0 && c == '-')) {
			dot_count++;
			out |= ((ver & 0xff) << (64 - 8 * dot_count));
			ver = 0;
		}

		if (c == '-') {
			dash_count++;
		}

		/* Bits 31-4 are the 7 digits of git hash */

		if (g_count > 0 && g_count < 8) {
			if (c >= '0' && c <= '9') {
				out |= (uint64_t)(c - '0') << (32 - 4 * g_count++);

			} else if (c >= 'a' && c <= 'f') {
				out |= (uint64_t)(c - 'a' + 10) << (32 - 4 * g_count++);

			} else {
				g_count = 8;
			}
		}

		if (c == 'g') {
			g_count++;
		}

		/* If d(i)rty, set bits 3-0 */

		if (g_count > 7 && c == 'i') {
			out |= 0xf;
			break;
		}
	}

	return out;
}

/************************************************************************************
  * Name: board_determine_hw_info
 *
 * Description:
 *	Uses the HW revision and version detection
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   0  - on success or negated errono
 *   1) The values for integer value of this boards hardware revision is set
 *   2) The integer value of this boards hardware version is set.
 *   3) hw_info is populated
 *
 ************************************************************************************/

int board_determine_hw_info(void)
{
	struct system_version_string_s ver_str;
	struct system_version_s ver;
	struct guid_s guid;
	struct hw_info_s hwinfo;
	orb_advert_t ver_str_pub = orb_advertise(ORB_ID(system_version_string), NULL);
	orb_advert_t ver_pub = orb_advertise(ORB_ID(system_version), NULL);
	orb_advert_t mfguid_pub = orb_advertise(ORB_ID(guid), NULL);
	orb_advert_t hw_info_pub = orb_advertise(ORB_ID(hw_info), NULL);
	uint64_t timestamp = hrt_absolute_time();

	memset(&ver_str, 0, sizeof(ver_str));
	memset(&ver, 0, sizeof(ver));

	snprintf(hw_info, sizeof(hw_info), HW_INFO_INIT_PREFIX HW_INFO_SUFFIX,
		 hw_version, hw_revision);

	/* HW version */

	snprintf(ver_str.hw_version, sizeof(ver_str.hw_version), HW_INFO_INIT_PREFIX HW_INFO_SUFFIX, hw_version, hw_revision);
	ver.hw_version = hw_version;

	/* HW revision */

	ver.hw_revision = hw_revision;

	/* SoC architecture ID */

	ver.soc_arch_id = soc_arch_id;

	/* PX4 version */

	strncpy(ver_str.sw_version, PX4_GIT_TAG_STR, sizeof(ver_str.sw_version));
	ver.sw_version = parse_tag_to_version(PX4_GIT_TAG_STR);

	/* NuttX version */

	snprintf(ver_str.os_version, sizeof(ver_str.os_version), NUTTX_GIT_TAG_STR "-g" NUTTX_GIT_VERSION_STR);
	ver.os_version = parse_tag_to_version(ver_str.os_version);

	/* Bootloader version */

	strncpy(ver_str.bl_version, device_boot_info.bl_version, sizeof(ver_str.bl_version));
	ver.bl_version = parse_tag_to_version(device_boot_info.bl_version);

	/* FPGA version */

	strncpy(ver_str.component_version1, "0", 2);
	ver.component_version1 = 0;

	/* Make local copies of guid and hwinfo */

	memcpy(guid.mfguid, device_serial_number, min(sizeof(device_serial_number), sizeof(guid.mfguid)));
	memcpy(hwinfo.hw_info, hw_info, min(sizeof(hwinfo.hw_info), sizeof(hw_info)));

	/* Then publish the topics */

	ver_str.timestamp = timestamp;
	ver.timestamp = timestamp;
	guid.timestamp = timestamp;
	hwinfo.timestamp = timestamp;

	orb_publish(ORB_ID(system_version_string), &ver_str_pub, &ver_str);
	orb_publish(ORB_ID(system_version), &ver_pub, &ver);
	orb_publish(ORB_ID(guid), &mfguid_pub, &guid);
	orb_publish(ORB_ID(hw_info), &hw_info_pub, &hwinfo);

	return OK;
}

int board_get_mfguid(mfguid_t mfgid)
{
	/* TODO */
	memset(mfgid, 0, sizeof(mfguid_t));
	return 0;
}
