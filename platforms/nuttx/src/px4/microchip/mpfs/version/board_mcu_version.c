/****************************************************************************
 *
 *   Copyright (C) 2021 Technology Innovation Institute. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <stdint.h>

#include "board_config.h"

#define MPFS_SYS_SERVICE_CR        0x37020050
#define MPFS_SYS_SERVICE_SR        0x37020054
#define MPFS_SYS_SERVICE_MAILBOX   0x37020800
#define SERVICE_CR_REQ             (1 << 0)
#define SERVICE_SR_BUSY            (1 << 1)

#define getreg8(a)                 (*(volatile uint8_t *)(a))
#define getreg32(a)                (*(volatile uint32_t *)(a))
#define putreg32(v,a)              (*(volatile uint32_t *)(a) = (v))

#define MPFS_ICICLE                0x0
#define SALUKI_VERSION_1	   0x1
#define SALUKI_VERSION_2	   0x2
// ...

static int hw_version = 0;
static int hw_revision = 0;
static char hw_info[] = HW_INFO_INIT;

static uint8_t device_serial_number[PX4_CPU_UUID_BYTE_LENGTH] = { 0 };

static void read_dsn(uint8_t *retval);
static void determine_hw(void);

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

	int len = sizeof(hw_version_table) / sizeof(hw_version_table[0]);

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

	memset(pb, 0, PX4_GUID_BYTE_LENGTH);

	*pb++ = (soc_arch_id >> 8) & 0xff;
	*pb++ = (soc_arch_id & 0xff);

	memcpy(pb, device_serial_number, PX4_CPU_UUID_BYTE_LENGTH);

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
	determine_hw();

	hw_info[HW_INFO_INIT_REV] = board_get_hw_revision() + '0';
	hw_info[HW_INFO_INIT_VER] = board_get_hw_version() + '0';

	return OK;
}

void determine_hw(void)
{
	uint8_t pin1, pin2, pin3;

	/* read device serial number */
	read_dsn(device_serial_number);

	/* determine hw version number */
	px4_arch_configgpio(GPIO_HW_VERSION_PIN1);
	px4_arch_configgpio(GPIO_HW_VERSION_PIN2);
	px4_arch_configgpio(GPIO_HW_VERSION_PIN3);

	/* wait pins to set */
	usleep(5);

	pin1 = px4_arch_gpioread(GPIO_HW_VERSION_PIN1);
	pin2 = px4_arch_gpioread(GPIO_HW_VERSION_PIN2);
	pin3 = px4_arch_gpioread(GPIO_HW_VERSION_PIN3);

	hw_version = (pin3 << 2) | (pin2 << 1) | pin1;

#ifdef BOARD_HAS_MULTIPURPOSE_VERSION_PINS
	/* NOTE: utilizes same GPIOs as LEDs. Restore GPIO pin configuration */
	px4_arch_configgpio(GPIO_nLED_RED);
	px4_arch_configgpio(GPIO_nLED_GREEN);
	px4_arch_configgpio(GPIO_nLED_BLUE);
#endif

}

void read_dsn(uint8_t *retval)
{
	unsigned int reg;
	uint8_t *serial;

	serial = retval;

	putreg32(SERVICE_CR_REQ, MPFS_SYS_SERVICE_CR);

	/* Need to wait that request is ready and not busy anymore */
	do {
		reg = getreg32(MPFS_SYS_SERVICE_CR);
	} while (SERVICE_CR_REQ == (reg & SERVICE_CR_REQ));

	do {
		reg = getreg32(MPFS_SYS_SERVICE_SR);
	} while (SERVICE_SR_BUSY == (reg & SERVICE_SR_BUSY));

	/* Read serial from service mailbox */
	uint8_t *p = (uint8_t *)MPFS_SYS_SERVICE_MAILBOX;

	for (uint8_t i = 0; i < PX4_CPU_UUID_BYTE_LENGTH; i++) {
		p = p + i;
		serial[i] = getreg8(p);
	}
}
