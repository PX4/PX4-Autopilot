/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file extflash.c
 *
 * Bootloader driver for the on-board Winbond W25Q256 QSPI flash.
 *
 * The CubeRed primary places the read-only part of the firmware (romfs and the
 * mavlink module) in a .extflash section that is linked at the QSPI
 * memory-mapped address (0x90000000). The native PX4 bootloader programs that
 * section into the external flash using the EXTF_* protocol commands and then
 * switches the QSPI controller into memory-mapped mode so the app can execute
 * and read from it directly.
 *
 * The flash is accessed with 3-byte (24-bit) addressing, which covers the low
 * 16 MiB - the region the .extflash section lives in. The bootloader runs from
 * internal flash, so (unlike the application driver these routines are based
 * on) the QSPI controller stays in indirect/command mode for the whole
 * programming session and is only switched to memory-mapped mode right before
 * the jump to the application.
 *
 * Based on boards/spracing/h7extreme/src/flash_w25q128.c.
 */

#include "board_config.h"
#include "qspi.h"
#include "bl.h"

/* W25Q256 geometry */
#define W25Q_SECTOR_SIZE		(4 * 1024)	/* 4 KiB sub-sector erase */
#define W25Q_PAGE_SIZE			256

/* fast read quad I/O (3-byte address, 6 dummy cycles incl. mode byte) */
#define W25Q_FAST_READ_QUAD		0xEB
#define W25Q_FAST_READ_QUAD_DUMMIES	6
#define W25Q_ADDRESS_SIZE		3

/* commands */
#define W25Q_READ_STATUS		0x05
#define W25Q_WRITE_ENABLE		0x06
#define W25Q_WRITE_DISABLE		0x04
#define W25Q_PAGE_PROGRAM		0x02
#define W25Q_SUBSECTOR_ERASE		0x20

/* status register bits */
#define W25Q_STATUS_BUSY		(1 << 0)
#define W25Q_STATUS_WEL			(1 << 1)

static struct qspi_dev_s *g_qspi;

/* memory-mapped read configuration, also reused for indirect reads */
static const struct qspi_meminfo_s g_read_meminfo = {
	.flags   = QSPIMEM_QUADIO,
	.addrlen = W25Q_ADDRESS_SIZE,
	.dummies = W25Q_FAST_READ_QUAD_DUMMIES,
	.cmd     = W25Q_FAST_READ_QUAD,
};

static int w25q_command(uint8_t cmd)
{
	struct qspi_cmdinfo_s cmdinfo = {
		.flags   = 0,
		.addrlen = 0,
		.cmd     = cmd,
		.buflen  = 0,
		.addr    = 0,
		.buffer  = NULL,
	};
	return QSPI_COMMAND(g_qspi, &cmdinfo);
}

static int w25q_command_address(uint8_t cmd, uint32_t addr)
{
	struct qspi_cmdinfo_s cmdinfo = {
		.flags   = QSPICMD_ADDRESS,
		.addrlen = W25Q_ADDRESS_SIZE,
		.cmd     = cmd,
		.buflen  = 0,
		.addr    = addr,
		.buffer  = NULL,
	};
	return QSPI_COMMAND(g_qspi, &cmdinfo);
}

static uint8_t w25q_read_status(void)
{
	uint8_t status = 0;
	struct qspi_cmdinfo_s cmdinfo = {
		.flags   = QSPICMD_READDATA,
		.addrlen = 0,
		.cmd     = W25Q_READ_STATUS,
		.buflen  = 1,
		.addr    = 0,
		.buffer  = &status,
	};
	QSPI_COMMAND(g_qspi, &cmdinfo);
	return status;
}

static void w25q_write_enable(void)
{
	do {
		w25q_command(W25Q_WRITE_ENABLE);
	} while ((w25q_read_status() & W25Q_STATUS_WEL) == 0);
}

static bool w25q_program_page(uint32_t offset, const uint8_t *buffer, unsigned length)
{
	struct qspi_meminfo_s meminfo = {
		.flags   = QSPIMEM_WRITE,
		.addrlen = W25Q_ADDRESS_SIZE,
		.dummies = 0,
		.cmd     = W25Q_PAGE_PROGRAM,
		.buflen  = length,
		.addr    = offset,
		.buffer  = (void *)buffer,
	};

	w25q_write_enable();

	if (QSPI_MEMORY(g_qspi, &meminfo) < 0) {
		return false;
	}

	/* wait for the page program to finish */
	while ((w25q_read_status() & W25Q_STATUS_BUSY) != 0);

	return true;
}

/****************************************************************************
 * Hooks consumed by the common bootloader (see bl.h)
 ****************************************************************************/

void board_extf_init(void)
{
	g_qspi = stm32h7_qspi_initialize(0);
}

uint32_t extf_func_sector_size(void)
{
	return W25Q_SECTOR_SIZE;
}

bool extf_func_start_sector_erase(unsigned sector)
{
	if ((w25q_read_status() & W25Q_STATUS_BUSY) != 0) {
		return false;
	}

	w25q_write_enable();
	w25q_command_address(W25Q_SUBSECTOR_ERASE, (uint32_t)sector * W25Q_SECTOR_SIZE);
	return true;
}

bool extf_func_is_busy(void)
{
	return (w25q_read_status() & W25Q_STATUS_BUSY) != 0;
}

bool extf_func_program(uintptr_t offset, const uint8_t *buffer, unsigned length)
{
	/* page programs must not wrap around within a 256-byte page */
	while (length > 0) {
		unsigned page_off = offset % W25Q_PAGE_SIZE;
		unsigned n = W25Q_PAGE_SIZE - page_off;

		if (n > length) {
			n = length;
		}

		if (!w25q_program_page((uint32_t)offset, buffer, n)) {
			return false;
		}

		offset += n;
		buffer += n;
		length -= n;
	}

	return true;
}

uint32_t extf_func_read_word(uintptr_t offset)
{
	uint32_t word = 0xffffffff;
	struct qspi_meminfo_s meminfo = g_read_meminfo;

	meminfo.addr   = (uint32_t)offset;
	meminfo.buflen = sizeof(word);
	meminfo.buffer = &word;

	QSPI_MEMORY(g_qspi, &meminfo);
	return word;
}

void board_extf_enable_xip(void)
{
	stm32h7_qspi_enter_memorymapped(g_qspi, &g_read_meminfo, 0);
}
