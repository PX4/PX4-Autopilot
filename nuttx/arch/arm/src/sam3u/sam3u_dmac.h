/****************************************************************************************
 * arch/arm/src/sam3u/sam3u_dmac.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_DMAC_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_DMAC_H

/****************************************************************************************
 * Included Files
 ****************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "sam3u_memorymap.h"

/****************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************/

/* DMAC register offsets ****************************************************************/

/* Global Registers */

#define SAM3U_DMAC_GCFG_OFFSET          0x00 /* DMAC Global Configuration Register */
#define SAM3U_DMAC_EN_OFFSET            0x04 /* DMAC Enable Register */
#define SAM3U_DMAC_SREQ_OFFSET          0x08 /* DMAC Software Single Request Register */
#define SAM3U_DMAC_CREQ_OFFSET          0x0c /* DMAC Software Chunk Transfer Request Register */
#define SAM3U_DMAC_LAST_OFFSET          0x10 /* DMAC Software Last Transfer Flag Register */
                                             /* 0x014-0x18: Reserved */
#define SAM3U_DMAC_EBCIER_OFFSET        0x18 /* DMAC Error Enable */
#define SAM3U_DMAC_EBCIDR_OFFSET        0x1C /* DMAC Error Disable */
#define SAM3U_DMAC_EBCIMR_OFFSET        0x20 /* DMAC Error Mask */
#define SAM3U_DMAC_EBCISR_OFFSET        0x24 /* DMAC Error Status */
#define SAM3U_DMAC_CHER_OFFSET          0x28 /* DMAC Channel Handler Enable Register */
#define SAM3U_DMAC_CHDR_OFFSET          0x2c /* DMAC Channel Handler Disable Register */
#define SAM3U_DMAC_CHSR_OFFSET          0x30 /* DMAC Channel Handler Status Register */
                                             /* 0x034-0x38: Reserved */
/* DMA channel registers */

#define SAM3U_DMACHAN_OFFSET(n)         (0x3c+((n)*0x28))
#define SAM3U_DMACHAN0_OFFSET           0x3c /* 0x3c-0x60: Channel 0 */
#define SAM3U_DMACHAN1_OFFSET           0x64 /* 0x64-0x88: Channel 1 */
#define SAM3U_DMACHAN2_OFFSET           0x8c /* 0x8c-0xb0: Channel 2 */
#define SAM3U_DMACHAN3_OFFSET           0xb4 /* 0xb4-0xd8: Channel 3 */

#define SAM3U_DMACHAN_SADDR_OFFSET      0x00 /* DMAC Channel Source Address Register */
#define SAM3U_DMACHAN_DADDR_OFFSET      0x04 /* DMAC Channel Destination Address Register */
#define SAM3U_DMACHAN_DSCR_OFFSET       0x08 /* DMAC Channel Descriptor Address Register */
#define SAM3U_DMACHAN_CTRLA_OFFSET      0x0c /* DMAC Channel Control A Register */
#define SAM3U_DMACHAN_CTRLB_OFFSET      0x10 /* DMAC Channel Control B Register */
#define SAM3U_DMACHAN_CFG_OFFSET        0x14 /* DMAC Channel Configuration Register */
                                             /* 0x18-0x24: Reserved */

                                             /* 0x017c-0x1fc: Reserved */

/* DMAC register adresses ***************************************************************/

/* Global Registers */

#define SAM3U_DMAC_GCFG                (SAM3U_DMAC_BASE+SAM3U_DMAC_GCFG_OFFSET)
#define SAM3U_DMAC_EN                  (SAM3U_DMAC_BASE+SAM3U_DMAC_EN_OFFSET)
#define SAM3U_DMAC_SREQ                (SAM3U_DMAC_BASE+SAM3U_DMAC_SREQ_OFFSET)
#define SAM3U_DMAC_CREQ                (SAM3U_DMAC_BASE+SAM3U_DMAC_CREQ_OFFSET)
#define SAM3U_DMAC_LAST                (SAM3U_DMAC_BASE+SAM3U_DMAC_LAST_OFFSET)
#define SAM3U_DMAC_EBCIER              (SAM3U_DMAC_BASE+SAM3U_DMAC_EBCIER_OFFSET)
#define SAM3U_DMAC_EBCIDR              (SAM3U_DMAC_BASE+SAM3U_DMAC_EBCIDR_OFFSET)
#define SAM3U_DMAC_EBCIMR              (SAM3U_DMAC_BASE+SAM3U_DMAC_EBCIMR_OFFSET)
#define SAM3U_DMAC_EBCISR              (SAM3U_DMAC_BASE+SAM3U_DMAC_EBCISR_OFFSET)
#define SAM3U_DMAC_CHER                (SAM3U_DMAC_BASE+SAM3U_DMAC_CHER_OFFSET)
#define SAM3U_DMAC_CHDR                (SAM3U_DMAC_BASE+SAM3U_DMAC_CHDR_OFFSET)
#define SAM3U_DMAC_CHSR                (SAM3U_DMAC_BASE+SAM3U_DMAC_CHSR_OFFSET)

/* DMA channel registers */

#define SAM3U_DMACHAN_BASE(n)          (SAM3U_DMAC_BASE+SAM3U_DMACHAN_OFFSET(n))
#define SAM3U_DMACHAN0_BASE            (SAM3U_DMAC_BASE+SAM3U_DMACHAN0_OFFSET)
#define SAM3U_DMACHAN1_BASE            (SAM3U_DMAC_BASE+SAM3U_DMACHAN1_OFFSET)
#define SAM3U_DMACHAN2_BASE            (SAM3U_DMAC_BASE+SAM3U_DMACHAN2_OFFSET)
#define SAM3U_DMACHAN3_BASE            (SAM3U_DMAC_BASE+SAM3U_DMACHAN3_OFFSET)

#define SAM3U_DMACHAN_SADDR(n)         (SAM3U_DMACHAN_BASE(n)+SAM3U_DMACHAN_SADDR_OFFSET)
#define SAM3U_DMACHAN_DADDR(n)         (SAM3U_DMACHAN_BASE(n)+SAM3U_DMACHAN_DADDR_OFFSET)
#define SAM3U_DMACHAN_DSCR(n)          (SAM3U_DMACHAN_BASE(n)+SAM3U_DMACHAN_DSCR_OFFSET)
#define SAM3U_DMACHAN_CTRLA(n)         (SAM3U_DMACHAN_BASE(n)+SAM3U_DMACHAN_CTRLA_OFFSET)
#define SAM3U_DMACHAN_CTRLB(n)         (SAM3U_DMACHAN_BASE(n)+SAM3U_DMACHAN_CTRLB_OFFSET)
#define SAM3U_DMACHAN_CFG(n)           (SAM3U_DMACHAN_BASE(n)+SAM3U_DMACHAN_CFG_OFFSET)

#define SAM3U_DMACHAN0_SADDR           (SAM3U_DMACHAN0_BASE+SAM3U_DMACHAN_SADDR_OFFSET)
#define SAM3U_DMACHAN0_DADDR           (SAM3U_DMACHAN0_BASE+SAM3U_DMACHAN_DADDR_OFFSET)
#define SAM3U_DMACHAN0_DSCR            (SAM3U_DMACHAN0_BASE+SAM3U_DMACHAN_DSCR_OFFSET)
#define SAM3U_DMACHAN0_CTRLA           (SAM3U_DMACHAN0_BASE+SAM3U_DMACHAN_CTRLA_OFFSET)
#define SAM3U_DMACHAN0_CTRLB           (SAM3U_DMACHAN0_BASE+SAM3U_DMACHAN_CTRLB_OFFSET)
#define SAM3U_DMACHAN0_CFG             (SAM3U_DMACHAN0_BASE+SAM3U_DMACHAN_CFG_OFFSET)

#define SAM3U_DMACHAN1_SADDR           (SAM3U_DMACHAN1_BASE+SAM3U_DMACHAN_SADDR_OFFSET)
#define SAM3U_DMACHAN1_DADDR           (SAM3U_DMACHAN1_BASE+SAM3U_DMACHAN_DADDR_OFFSET)
#define SAM3U_DMACHAN1_DSCR            (SAM3U_DMACHAN1_BASE+SAM3U_DMACHAN_DSCR_OFFSET)
#define SAM3U_DMACHAN1_CTRLA           (SAM3U_DMACHAN1_BASE+SAM3U_DMACHAN_CTRLA_OFFSET)
#define SAM3U_DMACHAN1_CTRLB           (SAM3U_DMACHAN1_BASE+SAM3U_DMACHAN_CTRLB_OFFSET)
#define SAM3U_DMACHAN1_CFG             (SAM3U_DMACHAN1_BASE+SAM3U_DMACHAN_CFG_OFFSET)

#define SAM3U_DMACHAN2_SADDR           (SAM3U_DMACHAN2_BASE+SAM3U_DMACHAN_SADDR_OFFSET)
#define SAM3U_DMACHAN2_DADDR           (SAM3U_DMACHAN2_BASE+SAM3U_DMACHAN_DADDR_OFFSET)
#define SAM3U_DMACHAN2_DSCR            (SAM3U_DMACHAN2_BASE+SAM3U_DMACHAN_DSCR_OFFSET)
#define SAM3U_DMACHAN2_CTRLA           (SAM3U_DMACHAN2_BASE+SAM3U_DMACHAN_CTRLA_OFFSET)
#define SAM3U_DMACHAN2_CTRLB           (SAM3U_DMACHAN2_BASE+SAM3U_DMACHAN_CTRLB_OFFSET)
#define SAM3U_DMACHAN2_CFG             (SAM3U_DMACHAN2_BASE+SAM3U_DMACHAN_CFG_OFFSET)

#define SAM3U_DMACHAN3_SADDR           (SAM3U_DMACHAN3_BASE+SAM3U_DMACHAN_SADDR_OFFSET)
#define SAM3U_DMACHAN3_DADDR           (SAM3U_DMACHAN3_BASE+SAM3U_DMACHAN_DADDR_OFFSET)
#define SAM3U_DMACHAN3_DSCR            (SAM3U_DMACHAN3_BASE+SAM3U_DMACHAN_DSCR_OFFSET)
#define SAM3U_DMACHAN3_CTRLA           (SAM3U_DMACHAN3_BASE+SAM3U_DMACHAN_CTRLA_OFFSET)
#define SAM3U_DMACHAN3_CTRLB           (SAM3U_DMACHAN3_BASE+SAM3U_DMACHAN_CTRLB_OFFSET)
#define SAM3U_DMACHAN3_CFG             (SAM3U_DMACHAN3_BASE+SAM3U_DMACHAN_CFG_OFFSET)

/* DMAC register bit definitions ********************************************************/

/* Global Registers */

/* DMAC Global Configuration Register */

#define DMAC_GCFG_ARB_CFG              (1 << 4)  /* Bit 4:  Round robin (vs fixed) arbiter */

/* DMAC Enable Register */

#define DMAC_EN_ENABLE                 (1 << 0)  /* Bit 0:  DMA controller enable */

/* DMAC Software Single Request Register */

#define DMAC_SREQ_SHIFT(n)             ((n)<<1)
#define DMAC_SREQ_MASK(n)              (3 << DMAC_SREQ_SHIFT(n))
#define DMAC_SREQ0_SHIFT               (0)      /* Bits 0-1:  Channel 0 */
#define DMAC_SREQ0_MASK                (3 << DMAC_SREQ0_SHIFT)
#define DMAC_SREQ1_SHIFT               (2)      /* Bits 2-3:  Channel 1 */
#define DMAC_SREQ1_MASK                (3 << DMAC_SREQ1_SHIFT)
#define DMAC_SREQ2_SHIFT               (4)      /* Bits 4-5:  Channel 2 */
#define DMAC_SREQ2_MASK                (3 << DMAC_SREQ2_SHIFT)
#define DMAC_SREQ3_SHIFT               (6)      /* Bits 6-7:  Channel 3 */
#define DMAC_SREQ3_MASK                (3 << DMAC_SREQ3_SHIFT)

#define DMAC_SREQ_SSREQ_SHIFT          (0)      /* Bits 0, 2, 4, 6:   Request a source single transfer */
#  define DMAC_SREQ_SSREQ(n)           (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ_SHIFT(n)))
#  define DMAC_SREQ_SSREQ0             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ0_SHIFT)
#  define DMAC_SREQ_SSREQ1             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ1_SHIFT)
#  define DMAC_SREQ_SSREQ2             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ2_SHIFT)
#  define DMAC_SREQ_SSREQ3             (1 << (DMAC_SREQ_SSREQ_SHIFT+DMAC_SREQ3_SHIFT)
#define DMAC_SREQ_DSREQ_SHIFT          (1)      /* Bits 1, 3, 5, 7:   Request a destination single transfer */
#  define DMAC_SREQ_DSREQ(n)           (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ_SHIFT(n))))
#  define DMAC_SREQ_DSREQ0             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ0_SHIFT)
#  define DMAC_SREQ_DSREQ1             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ1_SHIFT)
#  define DMAC_SREQ_DSREQ2             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ2_SHIFT)
#  define DMAC_SREQ_DSREQ3             (1 << (DMAC_SREQ_DSREQ_SHIFT+DMAC_SREQ3_SHIFT)

/* DMAC Software Chunk Transfer Request Register */

#define DMAC_CREQ_SHIFT(n)             ((n)<<1)
#define DMAC_CREQ_MASK(n)              (3 << DMAC_CREQ_SHIFT(n))
#define DMAC_CREQ0_SHIFT               (0)      /* Bits 0-1:  Channel 0 */
#define DMAC_CREQ0_MASK                (3 << DMAC_CREQ0_SHIFT)
#define DMAC_CREQ1_SHIFT               (2)      /* Bits 2-3:  Channel 1 */
#define DMAC_CREQ1_MASK                (3 << DMAC_CREQ1_SHIFT)
#define DMAC_CREQ2_SHIFT               (4)      /* Bits 4-5:  Channel 2 */
#define DMAC_CREQ2_MASK                (3 << DMAC_CREQ2_SHIFT)
#define DMAC_CREQ3_SHIFT               (6)      /* Bits 6-7:  Channel 3 */
#define DMAC_CREQ3_MASK                (3 << DMAC_CREQ3_SHIFT)

#define DMAC_CREQ_SCREQ_SHIFT          (0)      /* Bits 0, 2, 4, 6:   Request a source chunk transfer */
#  define DMAC_CREQ_SCREQ(n)           (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ_SHIFT(n)))
#  define DMAC_CREQ_SCREQ0             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ0_SHIFT)
#  define DMAC_CREQ_SCREQ1             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ1_SHIFT)
#  define DMAC_CREQ_SCREQ2             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ2_SHIFT)
#  define DMAC_CREQ_SCREQ3             (1 << (DMAC_CREQ_SCREQ_SHIFT+DMAC_CREQ3_SHIFT)
#define DMAC_CREQ_DCREQ_SHIFT          (1)      /* Bits 1, 3, 5, 7:   Request a destination chunk transfer */
#  define DMAC_CREQ_DCREQ(n)           (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ_SHIFT(n))))
#  define DMAC_CREQ_DCREQ0             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ0_SHIFT)
#  define DMAC_CREQ_DCREQ1             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ1_SHIFT)
#  define DMAC_CREQ_DCREQ2             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ2_SHIFT)
#  define DMAC_CREQ_DCREQ3             (1 << (DMAC_CREQ_DCREQ_SHIFT+DMAC_CREQ3_SHIFT)

/* DMAC Software Last Transfer Flag Register */

#define DMAC_LAST_SHIFT(n)             ((n)<<1)
#define DMAC_LAST_MASK(n)              (3 << DMAC_LAST_SHIFT(n))
#define DMAC_LAST0_SHIFT               (0)      /* Bits 0-1:  Channel 0 */
#define DMAC_LAST0_MASK                (3 << DMAC_LAST0_SHIFT)
#define DMAC_LAST1_SHIFT               (2)      /* Bits 2-3:  Channel 1 */
#define DMAC_LAST1_MASK                (3 << DMAC_LAST1_SHIFT)
#define DMAC_LAST2_SHIFT               (4)      /* Bits 4-5:  Channel 2 */
#define DMAC_LAST2_MASK                (3 << DMAC_LAST2_SHIFT)
#define DMAC_LAST3_SHIFT               (6)      /* Bits 6-7:  Channel 3 */
#define DMAC_LAST3_MASK                (3 << DMAC_LAST3_SHIFT)

#define DMAC_LAST_SLAST_SHIFT          (0)      /* Bits 0, 2, 4, 6:   Indicates the last transfer */
#  define DMAC_LAST_SLAST(n)           (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST_SHIFT(n)))
#  define DMAC_LAST_SLAST0             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST0_SHIFT)
#  define DMAC_LAST_SLAST1             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST1_SHIFT)
#  define DMAC_LAST_SLAST2             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST2_SHIFT)
#  define DMAC_LAST_SLAST3             (1 << (DMAC_LAST_SLAST_SHIFT+DMAC_LAST3_SHIFT)
#define DMAC_LAST_DLAST_SHIFT          (1)      /* Bits 1, 3, 5, 7:   Indicates the last transfer */
#  define DMAC_LAST_DLAST(n)           (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST_SHIFT(n))))
#  define DMAC_LAST_DLAST0             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST0_SHIFT)
#  define DMAC_LAST_DLAST1             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST1_SHIFT)
#  define DMAC_LAST_DLAST2             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST2_SHIFT)
#  define DMAC_LAST_DLAST3             (1 << (DMAC_LAST_DLAST_SHIFT+DMAC_LAST3_SHIFT)

/* DMAC Error, Buffer Transfer and Chained Buffer Transfer Interrupt Enable Register,
 * DMAC Error, Buffer Transfer and Chained Buffer Transfer Interrupt Disable Register,
 * DMAC Error, Buffer Transfer and Chained Buffer Transfer Interrupt Mask Register, and
 * DMAC Error, Buffer Transfer and Chained Buffer Transfer Status Register common
 * bit field definitions
 */

#define DMAC_EBC_BTC_SHIFT             (0)       /* Bits 0-3:  Buffer Transfer Completed Interrupt Enable  */
#define DMAC_EBC_BTC_MASK              (15 << DMAC_EBC_BTC_SHIFT)
#  define DMAC_EBC_BTC(n)              (1 << (DMAC_EBC_BTC_SHIFT+(n)))
#  define DMAC_EBC_BTC0                (1 << (DMAC_EBC_BTC_SHIFT+0))
#  define DMAC_EBC_BTC1                (1 << (DMAC_EBC_BTC_SHIFT+1))
#  define DMAC_EBC_BTC2                (1 << (DMAC_EBC_BTC_SHIFT+2))
#  define DMAC_EBC_BTC3                (1 << (DMAC_EBC_BTC_SHIFT+3))
#define DMAC_EBC_CBTC_SHIFT            (8)       /* Bits 8-11:  Chained Buffer Transfer Completed Interrupt Enable */
#define DMAC_EBC_CBTC_MASK             (15 << DMAC_EBC_CBTC_SHIFT)
#  define DMAC_EBC_CBTC(n)             (1 << (DMAC_EBC_CBTC_SHIFT+(n)))
#  define DMAC_EBC_CBTC0               (1 << (DMAC_EBC_CBTC_SHIFT+0))
#  define DMAC_EBC_CBTC1               (1 << (DMAC_EBC_CBTC_SHIFT+1))
#  define DMAC_EBC_CBTC2               (1 << (DMAC_EBC_CBTC_SHIFT+2))
#  define DMAC_EBC_CBTC3               (1 << (DMAC_EBC_CBTC_SHIFT+3))
#define DMAC_EBC_ERR_SHIFT             (16)      /* Bits 16-19:  Access Error Interrupt Enable */
#define DMAC_EBC_ERR_MASK              (15 << DMAC_EBC_ERR_SHIFT)
#  define DMAC_EBC_ERR(n)              (1 << (DMAC_EBC_ERR_SHIFT+(n)))
#  define DMAC_EBC_ERR0                (1 << (DMAC_EBC_ERR_SHIFT+0))
#  define DMAC_EBC_ERR1                (1 << (DMAC_EBC_ERR_SHIFT+1))
#  define DMAC_EBC_ERR2                (1 << (DMAC_EBC_ERR_SHIFT+2))
#  define DMAC_EBC_ERR3                (1 << (DMAC_EBC_ERR_SHIFT+3))

#define DMAC_EBC_BTCINTS(n)            (0x00010001 << (n))  /* BTC + ERR interrupts */
#define DMAC_EBC_CBTCINTS(n)           (0x00010100 << (n))  /* CBT + ERR interrupts */
#define DMAC_EBC_CHANINTS(n)           (0x00010101 << (n))  /* All channel interrupts */
#define DMAC_EBC_ALLINTS               (0x000f0f0f)         /* All interrupts */

/* DMAC Channel Handler Enable Register */

#define DMAC_CHER_ENA_SHIFT            (0)       /* Bits 0-3:  Enable channel  */
#define DMAC_CHER_ENA_MASK             (15 << DMAC_CHER_ENA_SHIFT)
#  define DMAC_CHER_ENA(n)             (1 << (DMAC_CHER_ENA_SHIFT+(n)))
#  define DMAC_CHER_ENA0               (1 << (DMAC_CHER_ENA_SHIFT+0))
#  define DMAC_CHER_ENA1               (1 << (DMAC_CHER_ENA_SHIFT+1))
#  define DMAC_CHER_ENA2               (1 << (DMAC_CHER_ENA_SHIFT+2))
#  define DMAC_CHER_ENA3               (1 << (DMAC_CHER_ENA_SHIFT+3))
#define DMAC_CHER_SUSP_SHIFT           (8)       /* Bits 8-11: Freeze channel and its context */
#define DMAC_CHER_SUSP_MASK            (15 << DMAC_CHER_SUSP_SHIFT)
#  define DMAC_CHER_SUSP(n)            (1 << (DMAC_CHER_SUSP_SHIFT+(n)))
#  define DMAC_CHER_SUSP0              (1 << (DMAC_CHER_SUSP_SHIFT+0))
#  define DMAC_CHER_SUSP1              (1 << (DMAC_CHER_SUSP_SHIFT+1))
#  define DMAC_CHER_SUSP2              (1 << (DMAC_CHER_SUSP_SHIFT+2))
#  define DMAC_CHER_SUSP3              (1 << (DMAC_CHER_SUSP_SHIFT+3))
#define DMAC_CHER_KEEP_SHIFT           (24)      /* Bits 24-27:  Resume channel from automatic stall */
#define DMAC_CHER_KEEP_MASK            (15 << DMAC_CHER_KEEP_SHIFT)
#  define DMAC_CHER_KEEP(n)            (1 << (DMAC_CHER_KEEP_SHIFT+(n)))
#  define DMAC_CHER_KEEP0              (1 << (DMAC_CHER_KEEP_SHIFT+0))
#  define DMAC_CHER_KEEP1              (1 << (DMAC_CHER_KEEP_SHIFT+1))
#  define DMAC_CHER_KEEP2              (1 << (DMAC_CHER_KEEP_SHIFT+2))
#  define DMAC_CHER_KEEP3              (1 << (DMAC_CHER_KEEP_SHIFT+3))

/* DMAC Channel Handler Disable Register */

#define DMAC_CHDR_DIS_SHIFT            (0)       /* Bits 0-3:  Disable DMAC channel  */
#define DMAC_CHDR_DIS_MASK             (15 << DMAC_CHDR_DIS_SHIFT)
#  define DMAC_CHDR_DIS(n)             (1 << (DMAC_CHDR_DIS_SHIFT+(n)))
#  define DMAC_CHDR_DIS0               (1 << (DMAC_CHDR_DIS_SHIFT+0))
#  define DMAC_CHDR_DIS1               (1 << (DMAC_CHDR_DIS_SHIFT+1))
#  define DMAC_CHDR_DIS2               (1 << (DMAC_CHDR_DIS_SHIFT+2))
#  define DMAC_CHDR_DIS3               (1 << (DMAC_CHDR_DIS_SHIFT+3))
#  define DMAC_CHDR_DIS_ALL            DMAC_CHDR_DIS_MASK
#define DMAC_CHDR_RES_SHIFT            (8)       /* Bits 8-11:  Resume trasnfer, restoring context */
#define DMAC_CHDR_RES_MASK             (15 << DMAC_CHDR_RES_SHIFT)
#  define DMAC_CHDR_RES(n)             (1 << (DMAC_CHDR_RES_SHIFT+(n)))
#  define DMAC_CHDR_RES0               (1 << (DMAC_CHDR_RES_SHIFT+0))
#  define DMAC_CHDR_RES1               (1 << (DMAC_CHDR_RES_SHIFT+1))
#  define DMAC_CHDR_RES2               (1 << (DMAC_CHDR_RES_SHIFT+2))
#  define DMAC_CHDR_RES3               (1 << (DMAC_CHDR_RES_SHIFT+3))

/* DMAC Channel Handler Status Register */

#define DMAC_CHSR_ENA_SHIFT            (0)       /* Bits 0-3:  Indicates that the channel is stalling  */
#define DMAC_CHSR_ENA_MASK             (15 << DMAC_CHSR_ENA_SHIFT)
#  define DMAC_CHSR_ENA(n)             (1 << (DMAC_CHSR_ENA_SHIFT+(n)))
#  define DMAC_CHSR_ENA0               (1 << (DMAC_CHSR_ENA_SHIFT+0))
#  define DMAC_CHSR_ENA1               (1 << (DMAC_CHSR_ENA_SHIFT+1))
#  define DMAC_CHSR_ENA2               (1 << (DMAC_CHSR_ENA_SHIFT+2))
#  define DMAC_CHSR_ENA3               (1 << (DMAC_CHSR_ENA_SHIFT+3))
#define DMAC_CHSR_SUSP_SHIFT           (8)       /* Bits 8-11:  Indicates that the channel is empty */
#define DMAC_CHSR_SUSP_MASK            (15 << DMAC_CHSR_SUSP_SHIFT)
#  define DMAC_CHSR_SUSP(n)            (1 << (DMAC_CHSR_SUSP_SHIFT+(n)))
#  define DMAC_CHSR_SUSP0              (1 << (DMAC_CHSR_SUSP_SHIFT+0))
#  define DMAC_CHSR_SUSP1              (1 << (DMAC_CHSR_SUSP_SHIFT+1))
#  define DMAC_CHSR_SUSP2              (1 << (DMAC_CHSR_SUSP_SHIFT+2))
#  define DMAC_CHSR_SUSP3              (1 << (DMAC_CHSR_SUSP_SHIFT+3))
#define DMAC_CHSR_EMPT_SHIFT           (16)      /* Bits 16-19:  Access Error Interrupt Enable */
#define DMAC_CHSR_EMPT_MASK            (15 << DMAC_CHSR_EMPT_SHIFT)
#  define DMAC_CHSR_EMPT(n)            (1 << (DMAC_CHSR_EMPT_SHIFT+(n)))
#  define DMAC_CHSR_EMPT0              (1 << (DMAC_CHSR_EMPT_SHIFT+0))
#  define DMAC_CHSR_EMPT1              (1 << (DMAC_CHSR_EMPT_SHIFT+1))
#  define DMAC_CHSR_EMPT2              (1 << (DMAC_CHSR_EMPT_SHIFT+2))
#  define DMAC_CHSR_EMPT3              (1 << (DMAC_CHSR_EMPT_SHIFT+3))
#define DMAC_CHSR_STAL_SHIFT           (24)      /* Bits 24-27:  Access Error Interrupt Enable */
#define DMAC_CHSR_STAL_MASK            (15 << DMAC_CHSR_STAL_SHIFT)
#  define DMAC_CHSR_STAL(n)            (1 << (DMAC_CHSR_STAL_SHIFT+(n)))
#  define DMAC_CHSR_STAL0              (1 << (DMAC_CHSR_STAL_SHIFT+0))
#  define DMAC_CHSR_STAL1              (1 << (DMAC_CHSR_STAL_SHIFT+1))
#  define DMAC_CHSR_STAL2              (1 << (DMAC_CHSR_STAL_SHIFT+2))
#  define DMAC_CHSR_STAL3              (1 << (DMAC_CHSR_STAL_SHIFT+3))

/* DMA channel registers */
/* DMAC Channel n [n = 0..3] Control A Register */

#define DMACHAN_CTRLA_BTSIZE_MAX       (0xfff)
#define DMACHAN_CTRLA_BTSIZE_SHIFT     (0)       /* Bits 0-11: Buffer Transfer Size */
#define DMACHAN_CTRLA_BTSIZE_MASK      (DMACHAN_CTRLA_BTSIZE_MAX << DMACHAN_CTRLA_BTSIZE_SHIFT)
#define DMACHAN_CTRLA_SCSIZE           (1 << 16) /* Bit 16: Source Chunk Transfer Size */
#  define DMACHAN_CTRLA_SCSIZE_1       (0)
#  define DMACHAN_CTRLA_SCSIZE_4       DMACHAN_CTRLA_SCSIZE
#define DMACHAN_CTRLA_DCSIZE           (1 << 20) /* Bit 20:  Destination Chunk Transfer size */
#  define DMACHAN_CTRLA_DCSIZE_1       (0)
#  define DMACHAN_CTRLA_DCSIZE_4       DMACHAN_CTRLA_DCSIZE
#define DMACHAN_CTRLA_SRCWIDTH_SHIFT   (24)      /* Bits 24-25 */
#define DMACHAN_CTRLA_SRCWIDTH_MASK    (3 << DMACHAN_CTRLA_SRCWIDTH_SHIFT)
#  define DMACHAN_CTRLA_SRCWIDTH_BYTE  (0 << DMACHAN_CTRLA_SRCWIDTH_SHIFT)
#  define DMACHAN_CTRLA_SRCWIDTH_HWORD (1 << DMACHAN_CTRLA_SRCWIDTH_SHIFT)
#  define DMACHAN_CTRLA_SRCWIDTH_WORD  (2 << DMACHAN_CTRLA_SRCWIDTH_SHIFT)
#define DMACHAN_CTRLA_DSTWIDTH_SHIFT   (28)      /* Bits 28-29 */
#define DMACHAN_CTRLA_DSTWIDTH_MASK    (3 << DMACHAN_CTRLA_DSTWIDTH_SHIFT)
#  define DMACHAN_CTRLA_DSTWIDTH_BYTE  (0 << DMACHAN_CTRLA_DSTWIDTH_SHIFT)
#  define DMACHAN_CTRLA_DSTWIDTH_HWORD (1 << DMACHAN_CTRLA_DSTWIDTH_SHIFT)
#  define DMACHAN_CTRLA_DSTWIDTH_WORD  (2 << DMACHAN_CTRLA_DSTWIDTH_SHIFT)
#define DMACHAN_CTRLA_DONE             (1 << 31) /* Bit 31: Auto disable DMAC */

/* DMAC Channel n [n = 0..3] Control B Register */

#define DMACHAN_CTRLB_SRCDSCR         (1 << 16) /* Bit 16: Source buffer descriptor fetch operation disabled */
#define DMACHAN_CTRLB_DSTDSCR         (1 << 20) /* Bit 20: Dest buffer descriptor fetch operation disabled */
#define DMACHAN_CTRLB_FC_SHIFT        (21)      /* Bits 21-22:  Flow controller  */
#define DMACHAN_CTRLB_FC_MASK         (3 << DMACHAN_CTRLB_FC_SHIFT)
#  define DMACHAN_CTRLB_FC_M2M        (0 << DMACHAN_CTRLB_FC_SHIFT) /* Memory-to-Memory  */
#  define DMACHAN_CTRLB_FC_M2P        (1 << DMACHAN_CTRLB_FC_SHIFT) /* Memory-to-Peripheral */
#  define DMACHAN_CTRLB_FC_P2M        (2 << DMACHAN_CTRLB_FC_SHIFT) /* Peripheral-to-Memory  */
#  define DMACHAN_CTRLB_FC_P2P        (3 << DMACHAN_CTRLB_FC_SHIFT) /* Peripheral-to-Peripheral */
#define DMACHAN_CTRLB_SRCINCR_SHIFT   (24)      /* Bits 24-25 */
#define DMACHAN_CTRLB_SRCINCR_MASK    (3 << DMACHAN_CTRLB_SRCINCR_SHIFT)
#  define DMACHAN_CTRLB_SRCINCR_INCR  (0 << DMACHAN_CTRLB_SRCINCR_SHIFT) /* Incrementing address */
#  define DMACHAN_CTRLB_SRCINCR_FIXED (2 << DMACHAN_CTRLB_SRCINCR_SHIFT) /* Fixed address */
#define DMACHAN_CTRLB_DSTINCR_SHIFT   (28)      /* Bits 28-29 */  
#define DMACHAN_CTRLB_DSTINCR_MASK    (3 << DMACHAN_CTRLB_DSTINCR_SHIFT)
#  define DMACHAN_CTRLB_DSTINCR_INCR  (0 << DMACHAN_CTRLB_DSTINCR_SHIFT) /* Incrementing address */
#  define DMACHAN_CTRLB_DSTINCR_FIXED (2 << DMACHAN_CTRLB_DSTINCR_SHIFT) /* Fixed address */
#define DMACHAN_CTRLB_IEN             (1 << 30)  /* Bit 30:  Clear sets BTC[n] flag in EBCISR */

/* DMAC Channel n [n = 0..3] Configuration Register */

#define DMACHAN_CFG_SRCPER_SHIFT      (0)       /* Bits 0-3:  Channel source associated with peripheral ID */
#define DMACHAN_CFG_SRCPER_MASK       (15 << DMACHAN_CFG_SRCPER_SHIFT)
#define DMACHAN_CFG_DSTPER_SHIFT      (4)       /* Bits 4-7:  Channel dest associated with peripheral ID */
#define DMACHAN_CFG_DSTPER_MASK       (15 << DMACHAN_CFG_DSTPER_SHIFT)
#define DMACHAN_CFG_SRCH2SEL          (1 << 9)  /* Bit 9:  HW handshake triggers transfer */
#define DMACHAN_CFG_DSTH2SEL          (1 << 13) /* Bit 13: HW handshake trigger transfer */
#define DMACHAN_CFG_SOD               (1 << 16) /* Bit 16: Stop on done */
#define DMACHAN_CFG_LOCKIF            (1 << 20) /* Bit 20: Enable lock interface capability */
#define DMACHAN_CFG_LOCKB             (1 << 21) /* Bit 21: Enable AHB Bus Locking capability */
#define DMACHAN_CFG_LOCKIFL           (1 << 22) /* Bit 22: Lock Master Interface Arbiter */
#define DMACHAN_CFG_AHBPRO_SHIFT      (24)      /* Bits 24-26: Bus access privilege */
#define DMACHAN_CFG_AHBPRO_MASK       (7 << DMACHAN_CFG_AHBPRO_SHIFT)
#  define DMACHAN_CFG_AHBPRO_PRIV     (1 << DMACHAN_CFG_AHBPRO_SHIFT)
#  define DMACHAN_CFG_AHBPRO_BUFF     (2 << DMACHAN_CFG_AHBPRO_SHIFT)
#  define DMACHAN_CFG_AHBPRO_CACHE    (4 << DMACHAN_CFG_AHBPRO_SHIFT)
#define DMACHAN_CFG_FIFOCFG_SHIFT     (28)      /* Bits 28-29 */
#define DMACHAN_CFG_FIFOCFG_MASK      (3 << DMACHAN_CFG_FIFOCFG_SHIFT)
#  define DMACHAN_CFG_FIFOCFG_LARGEST (0 << DMACHAN_CFG_FIFOCFG_SHIFT) /* Largest length AHB burst */
#  define DMACHAN_CFG_FIFOCFG_HALF    (1 << DMACHAN_CFG_FIFOCFG_SHIFT) /* Half FIFO size */
#  define DMACHAN_CFG_FIFOCFG_SINGLE  (2 << DMACHAN_CFG_FIFOCFG_SHIFT) /* Single AHB access */

/* DMA Peripheral IDs *******************************************************************/

#define DMACHAN_PID_MCI0               0
#define DMACHAN_PID_SSC                3
#define DMACHAN_PID_MCI1               13

/****************************************************************************************
 * Public Types
 ****************************************************************************************/

/* DMA multi buffer transfer link list entry structure */

struct dma_linklist_s
{
  uint32_t src;    /* Source address */
  uint32_t dest;   /* Destination address */
  uint32_t ctrla;  /* Control A value */
  uint32_t ctrlb;  /* Control B value */
  uint32_t next;   /* Next descriptor address */
};

/****************************************************************************************
 * Public Data
 ****************************************************************************************/

/****************************************************************************************
 * Public Functions
 ****************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_DMAC_H */
