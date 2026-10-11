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
 *   0x30000000  boot_partition       128 KiB
 *   0x30020000  slot0_partition      8 MiB - 128 KiB   running image
 *   0x30800000  slot1_partition      8 MiB - 128 KiB
 *   0x30FE0000  scratch_partition    128 KiB
 *   0x31000000  storage_partition    46 MiB            littlefs
 *   0x33E00000  reserved             2 MiB
 *   0x34000000  end of device
 *
 * The storage edges are fixed once formatted (block_count lives in the
 * superblock and NuttX never grows it): moving either wipes deployed volumes.
 */

#pragma once

#define FLEXSPI_NOR_SECTOR_SIZE         4096u
#define FLEXSPI_NOR_BLOCK_SIZE          (64u * 1024u)
#define FLEXSPI_NOR_TOTAL_SIZE          (64u * 1024u * 1024u)

#define FLASH_PARTITION_DEVICE_SIZE     FLEXSPI_NOR_TOTAL_SIZE
#define FLASH_PARTITION_SECTOR_SIZE     FLEXSPI_NOR_SECTOR_SIZE
#define FLASH_PARTITION_BLOCK_SIZE      FLEXSPI_NOR_BLOCK_SIZE

#define FLASH_BOOT_PARTITION_SIZE       (128u * 1024u)
#define FLASH_SLOT_PARTITION_SIZE       (8u * 1024u * 1024u - FLASH_BOOT_PARTITION_SIZE)
#define FLASH_SCRATCH_PARTITION_SIZE    (128u * 1024u)
#define FLASH_STORAGE_PARTITION_SIZE    (46u * 1024u * 1024u)

#include <px4_platform/flash_partitions.h>

_Static_assert(FLASH_STORAGE_PARTITION_OFFSET == 0x01000000u, "storage_partition must stay at 0x31000000");
_Static_assert(FLASH_RESERVED_SIZE == 2u * 1024u * 1024u, "reserved tail must stay 2 MiB");
