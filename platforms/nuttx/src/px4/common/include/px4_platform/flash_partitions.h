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
 * MCUboot-shaped partitioning of a flash device, laid out in order from the
 * device base: boot_partition, slot0_partition, slot1_partition (equal to
 * slot0, as MCUboot swap requires), scratch_partition, storage_partition,
 * and whatever remains is reserved to the end of the device.
 *
 * The board defines the sizes before including this header:
 *   FLASH_PARTITION_DEVICE_SIZE   whole device
 *   FLASH_PARTITION_SECTOR_SIZE   erase granularity
 *   FLASH_PARTITION_BLOCK_SIZE    large-erase granularity; storage is aligned to it
 *   FLASH_BOOT_PARTITION_SIZE, FLASH_SLOT_PARTITION_SIZE, FLASH_STORAGE_PARTITION_SIZE
 *
 * Optional:
 *   FLASH_SLOT1_PARTITION_SIZE    defaults to slot0; 0 for a single-slot board
 *   FLASH_SCRATCH_PARTITION_SIZE  defaults to 0
 *   FLASH_STORAGE_PARTITION_OFFSET  defaults to the end of scratch; a board may
 *                                 place storage anywhere at or after it, e.g. at
 *                                 the top of a storage-only die (boot and slot
 *                                 sizes 0)
 *
 * When included from a bootloader unit that defines BOARD_FLASH_SECTORS and
 * BOARD_FLASH_SIZE, the erase window and image size are checked as well.
 */

#pragma once

#ifndef FLASH_SLOT1_PARTITION_SIZE
#  define FLASH_SLOT1_PARTITION_SIZE     FLASH_SLOT_PARTITION_SIZE
#endif

#ifndef FLASH_SCRATCH_PARTITION_SIZE
#  define FLASH_SCRATCH_PARTITION_SIZE   0u
#endif

#define FLASH_BOOT_PARTITION_OFFSET      0u

#define FLASH_SLOT0_PARTITION_OFFSET     (FLASH_BOOT_PARTITION_OFFSET + FLASH_BOOT_PARTITION_SIZE)
#define FLASH_SLOT0_PARTITION_SIZE       FLASH_SLOT_PARTITION_SIZE

#define FLASH_SLOT1_PARTITION_OFFSET     (FLASH_SLOT0_PARTITION_OFFSET + FLASH_SLOT0_PARTITION_SIZE)

#define FLASH_SCRATCH_PARTITION_OFFSET   (FLASH_SLOT1_PARTITION_OFFSET + FLASH_SLOT1_PARTITION_SIZE)

#ifndef FLASH_STORAGE_PARTITION_OFFSET
#  define FLASH_STORAGE_PARTITION_OFFSET (FLASH_SCRATCH_PARTITION_OFFSET + FLASH_SCRATCH_PARTITION_SIZE)
#endif
#define FLASH_STORAGE_PARTITION_SECTORS  (FLASH_STORAGE_PARTITION_SIZE / FLASH_PARTITION_SECTOR_SIZE)

#define FLASH_RESERVED_OFFSET            (FLASH_STORAGE_PARTITION_OFFSET + FLASH_STORAGE_PARTITION_SIZE)
#define FLASH_RESERVED_SIZE              (FLASH_PARTITION_DEVICE_SIZE - FLASH_RESERVED_OFFSET)

_Static_assert(FLASH_SLOT1_PARTITION_SIZE == 0u || FLASH_SLOT1_PARTITION_SIZE == FLASH_SLOT0_PARTITION_SIZE,
	       "slot1 must be empty or equal to slot0");
_Static_assert(FLASH_STORAGE_PARTITION_OFFSET >= FLASH_SCRATCH_PARTITION_OFFSET + FLASH_SCRATCH_PARTITION_SIZE,
	       "storage overlaps the image partitions");
_Static_assert(FLASH_RESERVED_OFFSET <= FLASH_PARTITION_DEVICE_SIZE, "flash partitions exceed the device");
_Static_assert(FLASH_SLOT0_PARTITION_OFFSET % FLASH_PARTITION_SECTOR_SIZE == 0, "slot0 must be sector aligned");
_Static_assert(FLASH_SLOT1_PARTITION_OFFSET % FLASH_PARTITION_SECTOR_SIZE == 0, "slot1 must be sector aligned");
_Static_assert(FLASH_SCRATCH_PARTITION_OFFSET % FLASH_PARTITION_SECTOR_SIZE == 0, "scratch must be sector aligned");
_Static_assert(FLASH_STORAGE_PARTITION_OFFSET % FLASH_PARTITION_BLOCK_SIZE == 0, "storage must be block aligned");
_Static_assert(FLASH_STORAGE_PARTITION_SIZE % FLASH_PARTITION_SECTOR_SIZE == 0, "storage must be a sector multiple");

#ifdef BOARD_FLASH_SECTORS
_Static_assert((unsigned)BOARD_FLASH_SECTORS * FLASH_PARTITION_SECTOR_SIZE <= FLASH_SLOT1_PARTITION_OFFSET,
	       "bootloader erase window overlaps slot1");
#endif

#ifdef BOARD_FLASH_SIZE
_Static_assert((unsigned)BOARD_FLASH_SIZE <= FLASH_SLOT0_PARTITION_OFFSET + FLASH_SLOT0_PARTITION_SIZE,
	       "app image does not fit in slot0");
#endif
