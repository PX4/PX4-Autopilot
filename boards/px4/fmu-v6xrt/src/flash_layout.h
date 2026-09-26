/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

/*
 * FlexSPI1 boot NOR partitioning (64 MiB octal NOR, XIP at 0x30000000):
 *
 *   0x30000000  bootloader                          128 KiB
 *   0x30020000  app slot A                    8 MiB - 128 KiB
 *   0x30800000  reserved: A/B slot B / app growth      8 MiB
 *   0x31000000  littlefs                              46 MiB
 *   0x33E00000  reserved tail                          2 MiB
 *   0x34000000  end of device
 *
 * The littlefs edges are fixed once formatted (block_count lives in the
 * superblock and NuttX never grows it): moving either wipes deployed volumes.
 */

#pragma once

#define FLEXSPI_NOR_PAGE_SIZE       256u
#define FLEXSPI_NOR_SECTOR_SIZE     4096u
#define FLEXSPI_NOR_BLOCK_SIZE      (64u * 1024u)
#define FLEXSPI_NOR_TOTAL_SIZE      (64u * 1024u * 1024u)

#define FLASH_BOOTLOADER_OFFSET     0u
#define FLASH_BOOTLOADER_SIZE       (128u * 1024u)

#define FLASH_APP_SLOT_A_OFFSET     (FLASH_BOOTLOADER_OFFSET + FLASH_BOOTLOADER_SIZE)
#define FLASH_APP_SLOT_A_SIZE       (8u * 1024u * 1024u - FLASH_BOOTLOADER_SIZE)

#define FLASH_APP_SLOT_B_OFFSET     (FLASH_APP_SLOT_A_OFFSET + FLASH_APP_SLOT_A_SIZE)
#define FLASH_APP_SLOT_B_SIZE       (8u * 1024u * 1024u)

#define FLASH_STORAGE_OFFSET        (FLASH_APP_SLOT_B_OFFSET + FLASH_APP_SLOT_B_SIZE)
#define FLASH_STORAGE_SIZE          (46u * 1024u * 1024u)
#define FLASH_STORAGE_SECTORS       (FLASH_STORAGE_SIZE / FLEXSPI_NOR_SECTOR_SIZE)

#define FLASH_RESERVED_TAIL_OFFSET  (FLASH_STORAGE_OFFSET + FLASH_STORAGE_SIZE)
#define FLASH_RESERVED_TAIL_SIZE    (2u * 1024u * 1024u)

_Static_assert(FLASH_RESERVED_TAIL_OFFSET + FLASH_RESERVED_TAIL_SIZE == FLEXSPI_NOR_TOTAL_SIZE,
	       "flash regions must tile the device");
_Static_assert(FLASH_STORAGE_OFFSET % FLEXSPI_NOR_BLOCK_SIZE == 0, "storage must be block aligned");
_Static_assert(FLASH_STORAGE_SIZE % FLEXSPI_NOR_SECTOR_SIZE == 0, "storage must be a sector multiple");

#ifdef BOARD_FLASH_SECTORS
_Static_assert((unsigned)BOARD_FLASH_SECTORS * FLEXSPI_NOR_SECTOR_SIZE <= FLASH_STORAGE_OFFSET,
	       "bootloader erase window overlaps the filesystem");
#endif

#ifdef BOARD_FLASH_SIZE
_Static_assert((unsigned)BOARD_FLASH_SIZE <= FLASH_APP_SLOT_A_OFFSET + FLASH_APP_SLOT_A_SIZE,
	       "app image does not fit in slot A");
#endif
