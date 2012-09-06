/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_mpmc.h
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
 ************************************************************************************************/

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_MPMC_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_MPMC_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* MPMC register base address offset into the MPMC domain ***************************************/

#define LPC31_MPMC_VBASE                (LPC31_MPMC_VADDR)
#define LPC31_MPMC_PBASE                (LPC31_MPMC_PADDR)

/* MPMC register offsets (with respect to the base of the MPMC domain) **************************/

#define LPC31_MPMC_CONTROL_OFFSET       0x000 /* Control Register */
#define LPC31_MPMC_STATUS_OFFSET        0x004 /* Status Register */
#define LPC31_MPMC_CONFIG_OFFSET        0x008 /* Configuration register */
#define LPC31_MPMC_DYNCONTROL_OFFSET    0x020 /* Dynamic Memory Control Register */
#define LPC31_MPMC_DYNREFRESH_OFFSET    0x024 /* Dynamic Memory Refresh Timer Register */
#define LPC31_MPMC_DYNREADCONFIG_OFFSET 0x028 /* Dynamic Memory Read Configuration Register */
#define LPC31_MPMC_DYNTRP_OFFSET        0x030 /* Dynamic Memory Precharge Command Period Register */
#define LPC31_MPMC_DYNTRAS_OFFSET       0x034 /* Dynamic Memory Active To Precharge Command Period Register */
#define LPC31_MPMC_DYNTSREX_OFFSET      0x038 /* Dynamic Memory Self-refresh Exit Time Register */
#define LPC31_MPMC_DYNTAPR_OFFSET       0x03c /* Dynamic Memory Last Data Out To Active Time Register */
#define LPC31_MPMC_DYNTDAL_OFFSET       0x040 /* Dynamic Memory Data-in To Active Command Time Register */
#define LPC31_MPMC_DYNTWR_OFFSET        0x044 /* Dynamic Memory Write Recovery Time Register */
#define LPC31_MPMC_DYNTRC_OFFSET        0x048 /* Dynamic Memory Active To Active Command Period Register */
#define LPC31_MPMC_DYNTRFC_OFFSET       0x04c /* Dynamic Memory Auto-refresh Period Register */
#define LPC31_MPMC_DYNTXSR_OFFSET       0x050 /* Dynamic Memory Exit Self-refresh Register */
#define LPC31_MPMC_DYNTRRD_OFFSET       0x054 /* Dynamic Memory Active Bank A to Active Bank B Time Register */
#define LPC31_MPMC_DYNTMRD_OFFSET       0x058 /* Dynamic Memory Load Mode Register To Active Command Time Register */
#define LPC31_MPMC_STEXTWAIT_OFFSET     0x080 /* Static Memory Extended Wait Register */
#define LPC31_MPMC_DYNCONFIG0_OFFSET    0x100 /* Dynamic Memory Configuration Registers 0 */
#define LPC31_MPMC_DYNRASCAS0_OFFSET    0x104 /* Dynamic Memory RAS and CAS Delay Registers 0 */
                                                /* 0x120-0x164: reserved */
#define LPC31_MPMC_STCONFIG0_OFFSET     0x200 /* Static Memory Configuration Registers 0 */
#define LPC31_MPMC_STWAITWEN0_OFFSET    0x204 /* Static Memory Write Enable Delay Registers 0 */
#define LPC31_MPMC_STWAITOEN0_OFFSET    0x208 /* Static Memory Output Enable Delay Registers 0 */
#define LPC31_MPMC_STWAITRD0_OFFSET     0x20c /* Static Memory Read Delay Registers 0 */
#define LPC31_MPMC_STWAITPAGE0_OFFSET   0x210 /* Static Memory Page Mode Read Delay Registers 0 */
#define LPC31_MPMC_STWAITWR0_OFFSET     0x214 /* Static Memory Write Delay Registers 0 */
#define LPC31_MPMC_STWAITTURN0_OFFSET   0x218 /* Static Memory Turn Round Delay Registers 0 */
#define LPC31_MPMC_STCONFIG1_OFFSET     0x220 /* tatic Memory Configuration Registers 1 */
#define LPC31_MPMC_STWAITWEN1_OFFSET    0x224 /* Static Memory Write Enable Delay Registers 1 */
#define LPC31_MPMC_STWAITOEN1_OFFSET    0x228 /* Static Memory Output Enable Delay Registers 1 */
#define LPC31_MPMC_STWAITRD1_OFFSET     0x22c /* Static Memory Read Delay Registers 1 */
#define LPC31_MPMC_STWAITPAGE1_OFFSET   0x230 /* Static Memory Page Mode Read Delay Registers 1 */
#define LPC31_MPMC_STWAITWR1_OFFSET     0x234 /* Static Memory Write Delay Registers 1 */
#define LPC31_MPMC_STWAITTURN1_OFFSET   0x238 /* Static Memory Turn Round Delay Registers 1 */
                                                /* 0x240-0x278: Reserverd */

/* MPMC register (virtual) addresses ************************************************************/

#define LPC31_MPMC_CONTROL              (LPC31_MPMC_VBASE+LPC31_MPMC_CONTROL_OFFSET)
#define LPC31_MPMC_STATUS               (LPC31_MPMC_VBASE+LPC31_MPMC_STATUS_OFFSET)
#define LPC31_MPMC_CONFIG               (LPC31_MPMC_VBASE+LPC31_MPMC_CONFIG_OFFSET)
#define LPC31_MPMC_DYNCONTROL           (LPC31_MPMC_VBASE+LPC31_MPMC_DYNCONTROL_OFFSET)
#define LPC31_MPMC_DYNREFRESH           (LPC31_MPMC_VBASE+LPC31_MPMC_DYNREFRESH_OFFSET)
#define LPC31_MPMC_DYNREADCONFIG        (LPC31_MPMC_VBASE+LPC31_MPMC_DYNREADCONFIG_OFFSET)
#define LPC31_MPMC_DYNTRP               (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTRP_OFFSET)
#define LPC31_MPMC_DYNTRAS              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTRAS_OFFSET)
#define LPC31_MPMC_DYNTSREX             (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTSREX_OFFSET)
#define LPC31_MPMC_DYNTAPR              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTAPR_OFFSET)
#define LPC31_MPMC_DYNTDAL              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTDAL_OFFSET)
#define LPC31_MPMC_DYNTWR               (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTWR_OFFSET)
#define LPC31_MPMC_DYNTRC               (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTRC_OFFSET)
#define LPC31_MPMC_DYNTRFC              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTRFC_OFFSET)
#define LPC31_MPMC_DYNTXSR              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTXSR_OFFSET)
#define LPC31_MPMC_DYNTRRD              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTRRD_OFFSET)
#define LPC31_MPMC_DYNTMRD              (LPC31_MPMC_VBASE+LPC31_MPMC_DYNTMRD_OFFSET)
#define LPC31_MPMC_STEXTWAIT            (LPC31_MPMC_VBASE+LPC31_MPMC_STEXTWAIT_OFFSET)
#define LPC31_MPMC_DYNCONFIG0           (LPC31_MPMC_VBASE+LPC31_MPMC_DYNCONFIG0_OFFSET)
#define LPC31_MPMC_DYNRASCAS0           (LPC31_MPMC_VBASE+LPC31_MPMC_DYNRASCAS0_OFFSET)
#define LPC31_MPMC_STCONFIG0            (LPC31_MPMC_VBASE+LPC31_MPMC_STCONFIG0_OFFSET)
#define LPC31_MPMC_STWAITWEN0           (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITWEN0_OFFSET)
#define LPC31_MPMC_STWAITOEN0           (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITOEN0_OFFSET)
#define LPC31_MPMC_STWAITRD0            (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITRD0_OFFSET)
#define LPC31_MPMC_STWAITPAGE0          (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITPAGE0_OFFSET)
#define LPC31_MPMC_STWAITWR0            (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITWR0_OFFSET)
#define LPC31_MPMC_STWAITTURN0          (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITTURN0_OFFSET)
#define LPC31_MPMC_STCONFIG1            (LPC31_MPMC_VBASE+LPC31_MPMC_STCONFIG1_OFFSET)
#define LPC31_MPMC_STWAITWEN1           (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITWEN1_OFFSET)
#define LPC31_MPMC_STWAITOEN1           (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITOEN1_OFFSET)
#define LPC31_MPMC_STWAITRD1            (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITRD1_OFFSET)
#define LPC31_MPMC_STWAITPAGE1          (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITPAGE1_OFFSET)
#define LPC31_MPMC_STWAITWR1            (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITWR1_OFFSET)
#define LPC31_MPMC_STWAITTURN1          (LPC31_MPMC_VBASE+LPC31_MPMC_STWAITTURN1_OFFSET)

/* MPMC register bit definitions ****************************************************************/
/* MPMCControl (address 0x17008000) */

#define MPMC_CONTROL_L                    (1 << 2)  /* Bit 2:  Indicates normal or low-power mode */
#define MPMC_CONTROL_M                    (1 << 1)  /* Bit 1:  Indicates normal or reset memory map */
#define MPMC_CONTROL_E                    (1 << 0)  /* Bit 0:  Indicates when the MPMC is enabled or disabled */

/* MPMCStatus (address 0x17008004) */

#define MPMC_STATUS_SA                    (1 << 2)  /* Bit 2:  Indicates the operating self-refresh mode */
#define MPMC_STATUS_S                     (1 << 1)  /* Bit 1:  Write buffer status */
#define MPMC_STATUS_B                     (1 << 0)  /* Bit 0:  MPMC is busy performing memory transactions */

/* MPMCConfig (address 0x17008008) */

#define MPMC_CONFIG_CLK                   (1 << 8)  /* Bit 8:  Indicates Clock ratio 1:2 */
#define MPMC_CONFIG_N                     (1 << 0)  /* Bit 0:  Indicates big-endian mode */

/* MPMCDynamicControl (address 0x17008020) */

#define MPMC_DYNCONTROL_DP                (1 << 13) /* Bit 13: Low-power SDRAM deep-sleep mode */
#define MPMC_DYNCONTROL_I_SHIFT           (8)       /* Bits 7-8: I SDRAM initialization */
#define MPMC_DYNCONTROL_I_MASK            (3 << MPMC_DYNCONTROL_I_SHIFT)
#  define MPMC_DYNCONTROL_INORMAL         (0 << MPMC_DYNCONTROL_I_SHIFT) /* SDRAM NORMAL operation command */
#  define MPMC_DYNCONTROL_IMODE           (1 << MPMC_DYNCONTROL_I_SHIFT) /* SDRAM MODE command */
#  define MPMC_DYNCONTROL_IPALL           (2 << MPMC_DYNCONTROL_I_SHIFT) /* SDRAM PALL (pre charge all) */
#  define MPMC_DYNCONTROL_INOP            (3 << MPMC_DYNCONTROL_I_SHIFT) /* SDRAM NOP (no operation) */
#define MPMC_DYNCONTROL_MMC               (1 << 5)  /* Bit 5:  Memory clock control */
#define MPMC_DYNCONTROL_SR                (1 << 2)  /* Bit 2:  Self-refresh request */
#define MPMC_DYNCONTROL_CS                (1 << 1)  /* Bit 1:  Dynamic-memory clock control */
#define MPMC_DYNCONTROL_CE                (1 << 0)  /* Bit 0:  Dynamic-memory clock enable */

/* MPMCDynamicRefresh (address 0x17008024) */

#define MPMC_DYNREFRESH_TIMER_SHIFT       (0)       /* Bits 0-10: Refresh timer */
#define MPMC_DYNREFRESH_TIMER_MASK        (0x07ff << MPMC_DYNREFRESH_TIMER_SHIFT)

/* MPMCDynamicReadConfig (address 0x17008028) */

#define MPMC_DYNREADCONFIG_RD_SHIFT       (0)      /* Bits 0-1: Read data strategy */
#define MPMC_DYNREADCONFIG_RD_MASK        (2 << MPMC_DYNREADCONFIG_RD_SHIFT)
#  define MPMC_DYNREADCONFIG_CLKOUTDEL    (0 << MPMC_DYNREADCONFIG_RD_SHIFT) /* Clock out delay */
#  define MPMC_DYNREADCONFIG_CMDDEL       (1 << MPMC_DYNREADCONFIG_RD_SHIFT) /* Command delay */
#  define MPMC_DYNREADCONFIG_CMDDELP1     (2 << MPMC_DYNREADCONFIG_RD_SHIFT) /* Command delay plus 1*/
#  define MPMC_DYNREADCONFIG_CMDDELP2     (3 << MPMC_DYNREADCONFIG_RD_SHIFT) /* Command delay plus 2 */

/* MPMCDynamictRP (address 0x17008030) */

#define MPMC_DYNTRP_SHIFT                 (0)      /* Bits 0-3: Precharge command period */
#define MPMC_DYNTRP_MASK                  (15 << MPMC_DYNTRP_SHIFT)

/* MPMCDynamictRAS (address 0x17008034) */

#define MPMC_DYNTRAS_SHIFT                (0)      /* Bits 0-3: Active to pre charge command period */
#define MPMC_DYNTRAS_MASK                 (15 << MPMC_DYNTRAS_SHIFT)

/* MPMCDynamictSREX (address 0x17008038) */

#define MPMC_DYNTSREX_SHIFT               (0)      /* Bits 0-3: Self-refresh exit time */
#define MPMC_DYNTSREX_MASK                (15 << MPMC_DYNTSREX_SHIFT)

/* MPMCDynamictAPR (address 0x1700803c) */

#define MPMC_DYNTAPR_SHIFT                (0)      /* Bits 0-3: Last-data-out to active command time */
#define MPMC_DYNTAPR_MASK                 (15 << MPMC_DYNTAPR_SHIFT)

/* MPMCDynamictDAL (address 0x17008040) */

#define MPMC_DYNTDAL_SHIFT                (0)      /* Bits 0-3: Data-in to active command */
#define MPMC_DYNTDAL_MASK                 (15 << MPMC_DYNTDAL_SHIFT)

/* MPMCDynamictWR (address 0x17008044) */

#define MPMC_DYNTWR_SHIFT                 (0)      /* Bits 0-3: Write recovery time */
#define MPMC_DYNTWR_MASK                  (15 << MPMC_DYNTWR_SHIFT)

/* MPMCDynamictRC (address 0x17008048) */

#define MPMC_DYNTRC_SHIFT                 (0)      /* Bits 0-4: Active to active command period */
#define MPMC_DYNTRC_MASK                  (31 << MPMC_DYNTRC_SHIFT)

/* MPMCDynamictRFC (address 0x1700804c) */

#define MPMC_DYNTRFC_SHIFT                (0)      /* Bits 0-4: Auto-refresh period and auto-refresh to active command period, */
#define MPMC_DYNTRFC_MASK                 (31 << MPMC_DYNTRFC_SHIFT)

/* MPMCDynamictXSR (address 0x1700804c) */

#define MPMC_DYNTXSR_SHIFT                (0)      /* Bits 0-4: Exit self-refresh to active command time */
#define MPMC_DYNTXSR_MASK                 (31 << MPMC_DYNTXSR_SHIFT)

/* MPMCDynamictRRD (address 0x17008050) */

#define MPMC_DYNTRRD_SHIFT                (0)      /* Bits 0-3: Active bank A to active bank B latency */
#define MPMC_DYNTRRD_MASK                 (15 << MPMC_DYNTRRD_SHIFT)

/* MPMCDynamictMRD (address 0x17008054) */

#define MPMC_DYNTMRD_SHIFT                (0)      /* Bits 0-3: Load mode register to active command time */
#define MPMC_DYNTMRD_MASK                 (15 << MPMC_DYNTMRD_SHIFT)

/* MPMCStaticExtendedWait (address 0x17008080) */

#define MPMC_DYNSTEXTWAIT_SHIFT           (0)      /* Bits 0-9: External wait time out */
#define MPMC_DYNSTEXTWAIT_MASK            (0x03ff << MPMC_DYNSTEXTWAIT_SHIFT)

/* MPMCDynamicConfig0 (address 0x17008100) */

#define MPMC_DYNCONFIG0_P                 (1 << 20) /* Bit 20: Write protect */
#define MPMC_DYNCONFIG0_B                 (1 << 19) /* Bit 19: Buffer enable */
#define MPMC_DYNCONFIG0_AM                (1 << 14) /* Bit 14: Address mapping */
#define MPMC_DYNCONFIG0_AM_SHIFT          (7)       /* Bits 7-12: Address mapping */
#define MPMC_DYNCONFIG0_AM_MASK           (0x3c << MPMC_DYNCONFIG0_AM_SHIFT)
                                                                             /* 16-bit external bus high-performance address mapping */
#  define MPMC_DYNCONFIG_HP16_2MX8        (0x00 << MPMC_DYNCONFIG0_AM_SHIFT) /* 16Mb (2Mx8), 2 banks, row length=11, column length=9 */
#  define MPMC_DYNCONFIG_HP16_1MX16       (0x01 << MPMC_DYNCONFIG0_AM_SHIFT) /* 16Mb (1Mx16), 2 banks, row length=11, column length=8 */
#  define MPMC_DYNCONFIG_HP16_8MX8        (0x04 << MPMC_DYNCONFIG0_AM_SHIFT) /* 64Mb (8Mx8), 4 banks, row length=12, column length=9 */
#  define MPMC_DYNCONFIG_HP16_4MX16       (0x05 << MPMC_DYNCONFIG0_AM_SHIFT) /* 64Mb (4Mx16), 4 banks, row length=12, column length=8 */
#  define MPMC_DYNCONFIG_HP16_16MX8       (0x08 << MPMC_DYNCONFIG0_AM_SHIFT) /* 128Mb (16Mx8), 4 banks, row length=12, column length=10 */
#  define MPMC_DYNCONFIG_HP16_8MX16       (0x09 << MPMC_DYNCONFIG0_AM_SHIFT) /* 128Mb (8Mx16), 4 banks, row length=12, column length=9 */
#  define MPMC_DYNCONFIG_HP16_32MX8       (0x0c << MPMC_DYNCONFIG0_AM_SHIFT) /* 256Mb (32Mx8), 4 banks, row length=13, column length=10 */
#  define MPMC_DYNCONFIG_HP16_16MX16      (0x0d << MPMC_DYNCONFIG0_AM_SHIFT) /* 256Mb (16Mx16), 4 banks, row length=13, column length=9 */
#  define MPMC_DYNCONFIG_HP16_64MX8       (0x10 << MPMC_DYNCONFIG0_AM_SHIFT) /* 512Mb (64Mx8), 4 banks, row length=13, column length=11 */
#  define MPMC_DYNCONFIG_HP16_32MX16      (0x11 << MPMC_DYNCONFIG0_AM_SHIFT) /* 512Mb (32Mx16), 4 banks, row length=13, column length=10 */
                                                                             /* 16-bit external bus low power SDRAM address mapping */
#  define MPMC_DYNCONFIG_LP16_2MX8        (0x20 << MPMC_DYNCONFIG0_AM_SHIFT) /* 6Mb (2Mx8), 2 banks, row length=11, column length=9 */
#  define MPMC_DYNCONFIG_LP16_1MX16       (0x21 << MPMC_DYNCONFIG0_AM_SHIFT) /* 16Mb (1Mx16), 2 banks, row length=11, column length=8 */
#  define MPMC_DYNCONFIG_LP16_8MX8        (0x24 << MPMC_DYNCONFIG0_AM_SHIFT) /* 64Mb (8Mx8), 4 banks, row length=12, column length=9 */
#  define MPMC_DYNCONFIG_LP16_4MX16       (0x25 << MPMC_DYNCONFIG0_AM_SHIFT) /* 64Mb (4Mx16), 4 banks, row length=12, column length=8 */
#  define MPMC_DYNCONFIG_LP16_16MX8       (0x28 << MPMC_DYNCONFIG0_AM_SHIFT) /* 128Mb (16Mx8), 4 banks, row length=12, column length=10 */
#  define MPMC_DYNCONFIG_LP16_8MX16       (0x29 << MPMC_DYNCONFIG0_AM_SHIFT) /* 128Mb (8Mx16), 4 banks, row length=12, column length=9 */
#  define MPMC_DYNCONFIG_LP16_32MX8       (0x2c << MPMC_DYNCONFIG0_AM_SHIFT) /* 256Mb (32Mx8), 4 banks, row length=13, column length=10 */
#  define MPMC_DYNCONFIG_LP16_16MX16      (0x2d << MPMC_DYNCONFIG0_AM_SHIFT) /* 256Mb (16Mx16), 4 banks, row length=13, column length=9 */
#  define MPMC_DYNCONFIG_LP16_64MX8       (0x30 << MPMC_DYNCONFIG0_AM_SHIFT) /* 512Mb (64Mx8), 4 banks, row length=13, column length=11 */
#  define MPMC_DYNCONFIG_LP16_32MX16      (0x31 << MPMC_DYNCONFIG0_AM_SHIFT) /* 512Mb (32Mx16), 4 banks, row length=13, column length=10 */
#define MPMC_DYNCONFIG0_MD_SHIFT          (3)       /* Bits 3-4: Memory device */
#define MPMC_DYNCONFIG0_MD_MASK           (3 << MPMC_DYNCONFIG0_MD_SHIFT)
#  define MPMC_DYNCONFIG0_MDSDRAM         (0 << MPMC_DYNCONFIG0_MD_SHIFT) /* SDRAM */
#  define MPMC_DYNCONFIG0_MDLPSDRAM       (1 << MPMC_DYNCONFIG0_MD_SHIFT) /* low-power SDRAM */
#  define MPMC_DYNCONFIG0_MDMSF           (2 << MPMC_DYNCONFIG0_MD_SHIFT) /* Micron SyncFlash */

/* MPMCDynamicRasCas0 (address 0x17008104) */

#define MPMC_DYNRASCAS0_CAS_SHIFT         (8)       /* Bits 8-9: CAS latency */
#define MPMC_DYNRASCAS0_CAS_MASK          (3 << MPMC_DYNRASCAS0_CAS_SHIFT)
#  define MPMC_DYNRASCAS0_CAS1CLK         (1 << MPMC_DYNRASCAS0_CAS_SHIFT) /* one clock cycle */
#  define MPMC_DYNRASCAS0_CAS2CLK         (2 << MPMC_DYNRASCAS0_CAS_SHIFT) /* two clock cycles */
#  define MPMC_DYNRASCAS0_CAS3CLK         (3 << MPMC_DYNRASCAS0_CAS_SHIFT) /* three clock cycles */
#define MPMC_DYNRASCAS0_RAS_SHIFT         (0)      /* Bits 0-1: RAS latency */
#define MPMC_DYNRASCAS0_RAS_MASK          (3 << MPMC_DYNRASCAS0_RAS_SHIFT)
#  define MPMC_DYNRASCAS0_RAS1CLK         (1 << MPMC_DYNRASCAS0_RAS_SHIFT) /* one clock cycle */
#  define MPMC_DYNRASCAS0_RAS2CLK         (2 << MPMC_DYNRASCAS0_RAS_SHIFT) /* two clock cycles */
#  define MPMC_DYNRASCAS0_RAS3CLK         (3 << MPMC_DYNRASCAS0_RAS_SHIFT) /* three clock cycles */

/* MPMCStaticConfig0 (address 0x17008120) and MPMCStaticConfig1 (address 0x17008220) */

#define MPMC_STCONFIG_WP                  (1 << 20) /* Bit 20: Write protect */
#define MPMC_STCONFIG_B                   (1 << 19) /* Bit 19: Buffer enable */
#define MPMC_STCONFIG_EW                  (1 << 8)  /* Bit 8:  Extended wait */
#define MPMC_STCONFIG_BLS                 (1 << 7)  /* Bit 7:  BLS EBI_NCAS_BLOUT_0/1 EBI_nWE config */
#define MPMC_STCONFIG_PC                  (1 << 6)  /* Bit 6:  Chip select polarity */
#define MPMC_STCONFIG_PM                  (1 << 63  /* Bit 3:  Page mode */
#define MPMC_STCONFIG_MW_SHIFT            (0)       /* Bits 0-1: Memory width */
#define MPMC_STCONFIG_MW_MASK             (3 << MPMC_STCONFIG_MW_SHIFT)
#  define MPMC_STCONFIG_MW8BIT            (0 << MPMC_STCONFIG_MW_SHIFT) /* 8-bit */
#  define MPMC_STCONFIG_MW16BIT           (1 << MPMC_STCONFIG_MW_SHIFT) /* 16-bit */

/* MPMCStaticWaitWen0 (address 0x17008204) and MPMCStaticWaitWen1 (address 0x17008224) */

#define MPMC_STWAITWEN_SHIFT              (0)       /* WAITWEN Delay from chip select to write enable */
#define MPMC_STWAITWEN_MASK               (15 << MPMC_STWAITWEN_SHIFT)

/* MPMCStaticWaitOen (address 0x17008208) and MPMCStaticWaitOen1 (address 0x17008228) */

#define MPMC_STWAITOEN_SHIFT              (0)       /* WAITOEN Delay from chip select to output enable */
#define MPMC_STWAITOEN_MASK               (15 << MPMC_STWAITOEN_SHIFT)

/* MPMCStaticWaitRd0 (address 0x1700820c) and MPMCStaticWaitRd1 (address 0x17008022c) */

#define MPMC_STWAITRD_SHIFT               (0)       /* Read first access wait state */
#define MPMC_STWAITRD_MASK                (31 << MPMC_STWAITRD_SHIFT)

/* MPMCStaticWaitPage0 (address 0x17008210) and MPMCStaticWaitPage1 (address 0x17008230) */

#define MPMC_STWAITPAGE_SHIFT             (0)       /* Read after the first read wait states */
#define MPMC_STWAITPAGE_MASK              (31 << MPMC_STWAITPAGE_SHIFT)

/* MPMCStaticWaitWr0 (address 0x17008214) and MPMCStaticWaitWr1 (address 0x17008234) */

#define MPMC_STWAITWR_SHIFT               (0)       /* Time for write accesses after the first read */
#define MPMC_STWAITWR_MASK                (31 << MPMC_STWAITWR_SHIFT)

/* MPMCStaticWaitTurn0 (address 0x17008218) and MPMCStaticWaitTurn1 (address 0x17008238) */

#define MPMC_STWAITTURN_SHIFT             (0)       /* Bus turnaround cycles */
#define MPMC_STWAITTURN_MASK              (15 << MPMC_STWAITTURN_SHIFT)

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_MPMC_H */
