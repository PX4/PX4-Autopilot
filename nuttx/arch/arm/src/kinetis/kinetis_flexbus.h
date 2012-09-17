/************************************************************************************
 * arch/arm/src/kinetis/kinetis_flexbus.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXBUS_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXBUS_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_FB_CS_OFFSET(n)    (0x0000+(12*(n)))
#define KINETIS_FB_CSAR_OFFSET     0x0000 /* Chip select n address register */
#define KINETIS_FB_CSMR_OFFSET     0x0004 /* Chip select n mask register */
#define KINETIS_FB_CSCR_OFFSET     0x0008 /* Chip select n control register */

#define KINETIS_FB_CSAR0_OFFSET    0x0000 /* Chip select 0 address register */
#define KINETIS_FB_CSMR0_OFFSET    0x0004 /* Chip select 0 mask register */
#define KINETIS_FB_CSCR0_OFFSET    0x0008 /* Chip select 0 control register */

#define KINETIS_FB_CSAR1_OFFSET    0x000c /* Chip select 1 address register */
#define KINETIS_FB_CSMR1_OFFSET    0x0010 /* Chip select 1 mask register */
#define KINETIS_FB_CSCR1_OFFSET    0x0014 /* Chip select 1 control register */

#define KINETIS_FB_CSAR2_OFFSET    0x0018 /* Chip select 2 address register */
#define KINETIS_FB_CSMR2_OFFSET    0x001c /* Chip select 2 mask register */
#define KINETIS_FB_CSCR2_OFFSET    0x0020 /* Chip select 2 control register */

#define KINETIS_FB_CSAR3_OFFSET    0x0024 /* Chip select 3 address register */
#define KINETIS_FB_CSMR3_OFFSET    0x0028 /* Chip select 3 mask register */
#define KINETIS_FB_CSCR3_OFFSET    0x002c /* Chip select 3 control register */

#define KINETIS_FB_CSAR4_OFFSET    0x0030 /* Chip select 4 address register */
#define KINETIS_FB_CSMR4_OFFSET    0x0034 /* Chip select 4 mask register */
#define KINETIS_FB_CSCR4_OFFSET    0x0038 /* Chip select 4 control register */

#define KINETIS_FB_CSAR5_OFFSET    0x003c /* Chip select 5 address register */
#define KINETIS_FB_CSMR5_OFFSET    0x0040 /* Chip select 5 mask register */
#define KINETIS_FB_CSCR5_OFFSET    0x0044 /* Chip select 5 control register */

#define KINETIS_FB_CSPMCR_OFFSET   0x0060 /* Chip select port multiplexing control register */

/* Register Addresses ***************************************************************/
# define    0x4000c000 /* FlexBus */

#define KINETIS_FB_CS_BASE(n)      (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CS_OFFSET(n))
#define KINETIS_FB_CSAR(n)         (KINETIS_FB_CS_BASE(n)+KINETIS_FB_CSAR_OFFSET)
#define KINETIS_FB_CSMR(n)         (KINETIS_FB_CS_BASE(n)+KINETIS_FB_CSMR_OFFSET)
#define KINETIS_FB_CSCR(n)         (KINETIS_FB_CS_BASE(n)+KINETIS_FB_CSCR_OFFSET)

#define KINETIS_FB_CSAR0           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSAR0_OFFSET)
#define KINETIS_FB_CSMR0           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSMR0_OFFSET)
#define KINETIS_FB_CSCR0           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSCR0_OFFSET)

#define KINETIS_FB_CSAR1           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSAR1_OFFSET)
#define KINETIS_FB_CSMR1           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSMR1_OFFSET)
#define KINETIS_FB_CSCR1           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSCR1_OFFSET)

#define KINETIS_FB_CSAR2           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSAR2_OFFSET)
#define KINETIS_FB_CSMR2           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSMR2_OFFSET)
#define KINETIS_FB_CSCR2           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSCR2_OFFSET)

#define KINETIS_FB_CSAR3           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSAR3_OFFSET)
#define KINETIS_FB_CSMR3           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSMR3_OFFSET)
#define KINETIS_FB_CSCR3           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSCR3_OFFSET)

#define KINETIS_FB_CSAR4           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSAR4_OFFSET)
#define KINETIS_FB_CSMR4           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSMR4_OFFSET)
#define KINETIS_FB_CSCR4           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSCR4_OFFSET)

#define KINETIS_FB_CSAR5           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSAR5_OFFSET)
#define KINETIS_FB_CSMR5           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSMR5_OFFSET)
#define KINETIS_FB_CSCR5           (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSCR5_OFFSET)

#define KINETIS_FB_CSPMCR          (KINETIS_FLEXBUSC_BASE+KINETIS_FB_CSPMCR_OFFSET)

/* Register Bit Definitions *********************************************************/

/* Chip select address register (32-bit) */

#define FB_CSAR_BA_SHIFT           (16)      /* Bits 16-31: Base address */
#define FB_CSAR_BA_MASK            (0xffff << FB_CSAR_BA_SHIFT)
                                             /* Bits 0-15: Reserved */

/* Chip select mask register (32-bit) */

#define FB_CSMR_V                  (1 << 0)  /* Bit 0:  Valid */
                                             /* Bits 1-7: Reserved */
#define FB_CSMR_WP                 (1 << 8)  /* Bit 8:  Write protect
                                             /* Bits 9-15: Reserved */
#define FB_CSMR_BAM_SHIFT          (16)      /* Bits 16-31: Base address mask */
#define FB_CSMR_BAM_MASK           (0xffff << FB_CSMR_BAM_SHIFT)

/* Chip select control register (32-bit) */
                                             /* Bits 0-1: Reserved */
#define FB_CSCR_BSTW               (1 << 3)  /* Bit 3:  Burst-write enable */
#define FB_CSCR_BSTR               (1 << 4)  /* Bit 4:  Burst-read enable */
#define FB_CSCR_BEM                (1 << 5)  /* Bit 5:  Byte-enable mode */
#define FB_CSCR_PS_SHIFT           (6)       /* Bits 6-7: Port size */
#define FB_CSCR_PS_MASK            (3 << FB_CSCR_PS_SHIFT)
#  define FB_CSCR_PS_32BIT         (0 << FB_CSCR_PS_SHIFT) /* 32-bit port size */
#  define FB_CSCR_PS_8BIT          (1 << FB_CSCR_PS_SHIFT) /* 8-bit port size */
#  define FB_CSCR_PS_16BIT         (2 << FB_CSCR_PS_SHIFT) /* 16-bit port size */
#define FB_CSCR_AA                 (1 << 8)  /* Bit 8:  Auto-acknowledge enable */
#define FB_CSCR_BLS                (1 << 9)  /* Bit 9:  Byte-lane shift */
#define FB_CSCR_WS_SHIFT           (10)      /* Bits 19-15: Wait states */
#define FB_CSCR_WS_SHIFT           (10)      /* Bits 19-15: Wait states */
#define FB_CSCR_WRAH_SHIFT         (16)      /* Bits 16-17: Write address hold or deselect */
#define FB_CSCR_WRAH_MASK          (3 << FB_CSCR_WRAH_SHIFT)
#  define FB_CSCR_WRAH_HOLD1       (0 << FB_CSCR_WRAH_SHIFT) /* Hold one cycle after FB_CSn */
#  define FB_CSCR_WRAH_HOLD2       (1 << FB_CSCR_WRAH_SHIFT) /* Hold two cycles after FB_CSn */
#  define FB_CSCR_WRAH_HOLD3       (2 << FB_CSCR_WRAH_SHIFT) /* Hold three cycles after FB_CSn */
#  define FB_CSCR_WRAH_HOLD4       (3 << FB_CSCR_WRAH_SHIFT) /* Hold four cycles after FB_CSn */
#define FB_CSCR_RDAH_SHIFT         (18)      /* Bits 18-19: Read address hold or deselect */
#define FB_CSCR_RDAH_MASK          (3 << FB_CSCR_RDAH_SHIFT)
#  define FB_CSCR_RDAH_10CYCLES    (0 << FB_CSCR_RDAH_SHIFT) /* AA=0:1 cycle else 0 cycles */
#  define FB_CSCR_RDAH_21CYCLES    (1 << FB_CSCR_RDAH_SHIFT) /* AA=0:2 cycles else 1 cycle */
#  define FB_CSCR_RDAH_32CYCLES    (2 << FB_CSCR_RDAH_SHIFT) /* AA=0:3 cycles else 2 cycles */
#  define FB_CSCR_RDAH_43CYCLES    (3 << FB_CSCR_RDAH_SHIFT) /* AA=0:4 cycles else 3 cycles */
#define FB_CSCR_ASET_SHIFT         (20)      /* Bits 20-21: Address setup */
#define FB_CSCR_ASET_MASK          (3 << FB_CSCR_ASET_SHIFT)
#  define FB_CSCR_ASET_1STRISING   (0 << FB_CSCR_ASET_SHIFT) /* Assert CR on first rising clock edge */
#  define FB_CSCR_ASET_2NDRISING   (1 << FB_CSCR_ASET_SHIFT) /* Assert CR on second rising clock edge */
#  define FB_CSCR_ASET_3RDRISING   (2 << FB_CSCR_ASET_SHIFT) /* Assert CR on third rising clock edge */
#  define FB_CSCR_ASET_4thRISING   (3 << FB_CSCR_ASET_SHIFT) /* Assert CR on fourth rising clock edge */
#define FB_CSCR_EXTS               (1 << 22) /* Bit 22: Extended address latch enable */
#define FB_CSCR_SWSEN              (1 << 23) /* Bit 23: Secondary wait state enable */
                                             /* Bits 24-25: Reserved */
#define FB_CSCR_SWS_SHIFT          (26)      /* Bits 26-31: Secondary wait states */
#define FB_CSCR_SWS_MASK           (0x3f << FB_CSCR_SWS_SHIFT)

/* Chip select port multiplexing control register (32-bit) */
                                             /* Bits 0-11: Reserved */
#define FB_CSPMCR_GROUP5_SHIFT     (12)      /* Bits 12-15: FlexBus signal group 5 multiplex control */
#define FB_CSPMCR_GROUP5_MASK      (15 << FB_CSPMCR_GROUP5_SHIFT)
#  define FB_CSPMCR_GROUP5_TA      (0 << FB_CSPMCR_GROUP5_SHIFT) /* FB_TA */
#  define FB_CSPMCR_GROUP5_CS3     (1 << FB_CSPMCR_GROUP5_SHIFT) /* FB_CS3 */
#  define FB_CSPMCR_GROUP5_BE70    (2 << FB_CSPMCR_GROUP5_SHIFT) /* FB_BE_7_0 */
#define FB_CSPMCR_GROUP4_SHIFT     (16)      /* Bits 16-19: FlexBus signal group 4 multiplex control */
#define FB_CSPMCR_GROUP4_MASK      (15 << FB_CSPMCR_GROUP4_SHIFT)
#  define FB_CSPMCR_GROUP4_TBST    (0 << FB_CSPMCR_GROUP4_SHIFT) /* FB_TBST */
#  define FB_CSPMCR_GROUP4_CS2     (1 << FB_CSPMCR_GROUP4_SHIFT) /* FB_CS2 */
#  define FB_CSPMCR_GROUP4_BE158   (2 << FB_CSPMCR_GROUP4_SHIFT) /* FB_BE_15_8 */
#define FB_CSPMCR_GROUP3_SHIFT     (20)      /* Bits 29-23: FlexBus signal group 3 multiplex control */
#define FB_CSPMCR_GROUP3_MASK      (15 << FB_CSPMCR_GROUP3_SHIFT)
#  define FB_CSPMCR_GROUP3_CS5     (0 << FB_CSPMCR_GROUP3_SHIFT) /* FB_CS5 */
#  define FB_CSPMCR_GROUP3_TSIZ1   (1 << FB_CSPMCR_GROUP3_SHIFT) /* FB_TSIZ1 */
#  define FB_CSPMCR_GROUP3_BE2316  (2 << FB_CSPMCR_GROUP3_SHIFT) /* FB_BE_23_16 */
#define FB_CSPMCR_GROUP2_SHIFT     (24)      /* Bits 24-27: FlexBus signal group 2 multiplex control */
#define FB_CSPMCR_GROUP2_MASK      (15 << FB_CSPMCR_GROUP2_SHIFT)
#  define FB_CSPMCR_GROUP2_CS4     (0 << FB_CSPMCR_GROUP2_SHIFT) /* FB_CS4 */
#  define FB_CSPMCR_GROUP2_TSIZ0   (1 << FB_CSPMCR_GROUP2_SHIFT) /* FB_TSIZ0 */
#  define FB_CSPMCR_GROUP2_BE3124  (2 << FB_CSPMCR_GROUP2_SHIFT) /* FB_BE_31_24 */
#define FB_CSPMCR_GROUP1_SHIFT     (28)      /* Bits 28-31: FlexBus signal group 1 multiplex control */
#define FB_CSPMCR_GROUP1_MASK      (15 << FB_CSPMCR_GROUP1_MASK)
#  define FB_CSPMCR_GROUP1_ALE     (0 << FB_CSPMCR_GROUP1_MASK) /* FB_ALE */
#  define FB_CSPMCR_GROUP1_CS1     (1 << FB_CSPMCR_GROUP1_MASK) /* FB_CS1 */
#  define FB_CSPMCR_GROUP1_TS      (2 << FB_CSPMCR_GROUP1_MASK) /* FB_TS */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_FLEXBUS_H */
