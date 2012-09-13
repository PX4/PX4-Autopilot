/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_chipid.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ****************************************************************************************/

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_CHIPID_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_CHIPID_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* CHIPID register offsets **************************************************************/

#define SAM3U_CHIPID_CIDR      0x00 /* Chip ID Register */
#define SAM3U_CHIPID_EXID      0x04 /* Chip ID Extension Register */

/* CHIPID register adresses *************************************************************/

#define SAM3U_CHIPID_CIDR      (SAM3U_CHIPID_BASE+SAM3U_CHIPID_CIDR)
#define SAM3U_CHIPID_EXID      (SAM3U_CHIPID_BASE+SAM3U_CHIPID_EXID)

/* CHIPID register bit definitions ******************************************************/

#define CHIPID_CIDR_VERSION_SHIFT       (0)      /* Bits 0-4:  Version of the Device */
#define CHIPID_CIDR_VERSION_MASK        (0x1f << CHIPID_CIDR_VERSION_SHIFT)
#define CHIPID_CIDR_EPROC_SHIFT         (5)      /* Bits 5-7:  Embedded Processor */
#define CHIPID_CIDR_EPROC_MASK          (7 << CHIPID_CIDR_EPROC_SHIFT)
#  define CHIPID_CIDR_EPROC_ARM946ES    (1 << CHIPID_CIDR_EPROC_SHIFT) /* ARM946E-S */
#  define CHIPID_CIDR_EPROC_ARM7TDMI    (2 << CHIPID_CIDR_EPROC_SHIFT) /* ARM7TDMI */
#  define CHIPID_CIDR_EPROC_CORTEXM3    (3 << CHIPID_CIDR_EPROC_SHIFT) /* Cortex-M3 */
#  define CHIPID_CIDR_EPROC_ARM920T     (4 << CHIPID_CIDR_EPROC_SHIFT) /* ARM920T */
#  define CHIPID_CIDR_EPROC_ARM926EJS   (5 << CHIPID_CIDR_EPROC_SHIFT) /* ARM926EJ-S */
#define CHIPID_CIDR_NVPSIZ_SHIFT        (8)      /* Bits 8-11:  Nonvolatile Program Memory Size */
#define CHIPID_CIDR_NVPSIZ_MASK         (15 << CHIPID_CIDR_NVPSIZ_SHIFT)
#  define CHIPID_CIDR_NVPSIZ_NONE       (0  << CHIPID_CIDR_NVPSIZ_SHIFT) /* None */
#  define CHIPID_CIDR_NVPSIZ_8KB        (1  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 8K bytes */
#  define CHIPID_CIDR_NVPSIZ_16KB       (2  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 16K bytes */
#  define CHIPID_CIDR_NVPSIZ_32KB       (3  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 32K bytes */
#  define CHIPID_CIDR_NVPSIZ_64KB       (5  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 64K bytes */
#  define CHIPID_CIDR_NVPSIZ_128KB      (7  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 128K bytes */
#  define CHIPID_CIDR_NVPSIZ_256KB      (9  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 256K bytes */
#  define CHIPID_CIDR_NVPSIZ_512KB      (10 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 512K bytes */
#  define CHIPID_CIDR_NVPSIZ_1MB        (12 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 1024K bytes */
#  define CHIPID_CIDR_NVPSIZ_2MB        (14 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 2048K bytes */
#define CHIPID_CIDR_NVPSIZ2_SHIFT       (12)      /* Bits 12-15:  Nonvolatile Program Memory Size */
#define CHIPID_CIDR_NVPSIZ2_MASK        (15 << CHIPID_CIDR_NVPSIZ_SHIFT)
#  define CHIPID_CIDR_NVPSIZ2_NONE      (0  << CHIPID_CIDR_NVPSIZ_SHIFT) /* None */
#  define CHIPID_CIDR_NVPSIZ2_8KB       (1  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 8K bytes */
#  define CHIPID_CIDR_NVPSIZ2_16KB      (2  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 16K bytes */
#  define CHIPID_CIDR_NVPSIZ2_32KB      (3  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 32K bytes */
#  define CHIPID_CIDR_NVPSIZ2_64KB      (5  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 64K bytes */
#  define CHIPID_CIDR_NVPSIZ2_128KB     (7  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 128K bytes */
#  define CHIPID_CIDR_NVPSIZ2_256KB     (9  << CHIPID_CIDR_NVPSIZ_SHIFT) /* 256K bytes */
#  define CHIPID_CIDR_NVPSIZ2_512KB     (10 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 512K bytes */
#  define CHIPID_CIDR_NVPSIZ2_1MB       (12 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 1024K bytes */
#  define CHIPID_CIDR_NVPSIZ2_2MB       (14 << CHIPID_CIDR_NVPSIZ_SHIFT) /* 2048K bytes */
#define CHIPID_CIDR_SRAMSIZ_SHIFT       (16)      /* Bits 16-19:  Internal SRAM Size */
#define CHIPID_CIDR_SRAMSIZ_MASK        (15 << CHIPID_CIDR_SRAMSIZ_SHIFT)
#  define CHIPID_CIDR_SRAMSIZ_48KB      (0  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 48K bytes */
#  define CHIPID_CIDR_SRAMSIZ_1KB       (1  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 1K bytes */
#  define CHIPID_CIDR_SRAMSIZ_2KB       (2  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 2K bytes */
#  define CHIPID_CIDR_SRAMSIZ_6KB       (3  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 6K bytes */
#  define CHIPID_CIDR_SRAMSIZ_112KB     (4  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 112K bytes */
#  define CHIPID_CIDR_SRAMSIZ_4KB       (5  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 4K bytes */
#  define CHIPID_CIDR_SRAMSIZ_80KB      (6  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 80K bytes */
#  define CHIPID_CIDR_SRAMSIZ_160KB     (7  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 160K bytes */
#  define CHIPID_CIDR_SRAMSIZ_8KB       (8  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 8K bytes */
#  define CHIPID_CIDR_SRAMSIZ_16KB      (9  << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 16K bytes */
#  define CHIPID_CIDR_SRAMSIZ_32KB      (10 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 32K bytes */
#  define CHIPID_CIDR_SRAMSIZ_64KB      (11 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 64K bytes */
#  define CHIPID_CIDR_SRAMSIZ_128KB     (12 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 128K bytes */
#  define CHIPID_CIDR_SRAMSIZ_256KB     (13 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 256K bytes */
#  define CHIPID_CIDR_SRAMSIZ_96KB      (14 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 96K bytes */
#  define CHIPID_CIDR_SRAMSIZ_512KB     (15 << CHIPID_CIDR_SRAMSIZ_SHIFT) /* 512K bytes */
#define CHIPID_CIDR_ARCH_SHIFT          (20)      /* Bits 20-27:  Architecture Identifier */
#define CHIPID_CIDR_ARCH_MASK           (0xff << CHIPID_CIDR_ARCH_SHIFT)
#  define CHIPID_CIDR_ARCH_AT91SAM9XX   (0x19 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM9xx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM9XEXX (0x29 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM9XExx Series */
#  define CHIPID_CIDR_ARCH_AT91X34      (0x34 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x34 Series */
#  define CHIPID_CIDR_ARCH_CAP7         (0x37 << CHIPID_CIDR_ARCH_SHIFT) /* CAP7 Series */
#  define CHIPID_CIDR_ARCH_CAP9         (0x39 << CHIPID_CIDR_ARCH_SHIFT) /* CAP9 Series */
#  define CHIPID_CIDR_ARCH_CAP11        (0x3b << CHIPID_CIDR_ARCH_SHIFT) /* CAP11 Series */
#  define CHIPID_CIDR_ARCH_AT91X40      (0x40 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x40 Series */
#  define CHIPID_CIDR_ARCH_AT91X42      (0x42 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x42 Series */
#  define CHIPID_CIDR_ARCH_AT91X55      (0x55 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x55 Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7AXX  (0x60 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Axx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7AQXX (0x61 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7AQxx Series */
#  define CHIPID_CIDR_ARCH_AT91X63      (0x63 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x63 Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7SXX  (0x70 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Sxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7XCXX (0x71 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7XCxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7SEXX (0x72 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7SExx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7LXX  (0x73 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Lxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7XXX  (0x75 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7Xxx Series */
#  define CHIPID_CIDR_ARCH_AT91SAM7SLXX (0x76 << CHIPID_CIDR_ARCH_SHIFT) /* AT91SAM7SLxx Series */
#  define CHIPID_CIDR_ARCH_SAM3UXC      (0x80 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3UxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3UXE      (0x81 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3UxE Series (144-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3AXC      (0x83 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3AxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3XXC      (0x84 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3XxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3XXE      (0x85 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3XxE Series (144-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3XXG      (0x86 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3XxG Series (208/217-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SXA      (0x88 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SxA Series (48-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SXB      (0x89 << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SxB Series (64-pin version) */
#  define CHIPID_CIDR_ARCH_SAM3SXC      (0x8a << CHIPID_CIDR_ARCH_SHIFT) /* SAM3SxC Series (100-pin version) */
#  define CHIPID_CIDR_ARCH_AT91X92      (0x92 << CHIPID_CIDR_ARCH_SHIFT) /* AT91x92 Series */
#  define CHIPID_CIDR_ARCH_AT75CXX      (0xf0 << CHIPID_CIDR_ARCH_SHIFT) /* AT75Cxx Series */
#define CHIPID_CIDR_NVPTYP_SHIFT        (28)      /* Bits 28-30:  Nonvolatile Program Memory Type */
#define CHIPID_CIDR_NVPTYP_MASK         (7 << CHIPID_CIDR_NVPTYP_SHIFT)
#  define CHIPID_CIDR_NVPTYP ROM        (0 << CHIPID_CIDR_NVPTYP_SHIFT) /* ROM */
#  define CHIPID_CIDR_NVPTYP FLASH      (1 << CHIPID_CIDR_NVPTYP_SHIFT) /* ROMless or on-chip Flash */
#  define CHIPID_CIDR_NVPTYP SRAM       (4 << CHIPID_CIDR_NVPTYP_SHIFT) /* SRAM emulating ROM */
#  define CHIPID_CIDR_NVPTYP EFLASH     (2 << CHIPID_CIDR_NVPTYP_SHIFT) /* Embedded Flash Memory */
#  define CHIPID_CIDR_NVPTYP REFLASH    (3 << CHIPID_CIDR_NVPTYP_SHIFT) /* ROM and Embedded Flash Memory */
#define CHIPID_CIDR_EXT                 (1 << 31) /* Bit 31: Extension Flag */

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_CHIPID_H */
