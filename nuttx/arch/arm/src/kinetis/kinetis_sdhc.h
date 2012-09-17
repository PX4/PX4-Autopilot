/************************************************************************************
 * arch/arm/src/kinetis/kinetis_sdhc.h
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

#ifndef __ARCH_ARM_SRC_KINETIS_KINETIS_SDHC_H
#define __ARCH_ARM_SRC_KINETIS_KINETIS_SDHC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define KINETIS_SDHC_DSADDR_OFFSET     0x0000 /* DMA System Address Register */
#define KINETIS_SDHC_BLKATTR_OFFSET    0x0004 /* Block Attributes Register */
#define KINETIS_SDHC_CMDARG_OFFSET     0x0008 /* Command Argument Register */
#define KINETIS_SDHC_XFERTYP_OFFSET    0x000c /* Transfer Type Register */
#define KINETIS_SDHC_CMDRSP0_OFFSET    0x0010 /* Command Response 0 */
#define KINETIS_SDHC_CMDRSP1_OFFSET    0x0014 /* Command Response 1 */
#define KINETIS_SDHC_CMDRSP2_OFFSET    0x0018 /* Command Response 2 */
#define KINETIS_SDHC_CMDRSP3_OFFSET    0x001c /* Command Response 3 */
#define KINETIS_SDHC_DATPORT_OFFSET    0x0020 /* Buffer Data Port Register */
#define KINETIS_SDHC_PRSSTAT_OFFSET    0x0024 /* Present State Register */
#define KINETIS_SDHC_PROCTL_OFFSET     0x0028 /* Protocol Control Register */
#define KINETIS_SDHC_SYSCTL_OFFSET     0x002c /* System Control Register */
#define KINETIS_SDHC_IRQSTAT_OFFSET    0x0030 /* Interrupt Status Register */
#define KINETIS_SDHC_IRQSTATEN_OFFSET  0x0034 /* Interrupt Status Enable Register */
#define KINETIS_SDHC_IRQSIGEN_OFFSET   0x0038 /* Interrupt Signal Enable Register */
#define KINETIS_SDHC_AC12ERR_OFFSET    0x003c /* Auto CMD12 Error Status Register */
#define KINETIS_SDHC_HTCAPBLT_OFFSET   0x0040 /* Host Controller Capabilities */
#define KINETIS_SDHC_WML_OFFSET        0x0044 /* Watermark Level Register */
#define KINETIS_SDHC_FEVT_OFFSET       0x0050 /* Force Event Register */
#define KINETIS_SDHC_ADMAES_OFFSET     0x0054 /* ADMA Error Status Register */
#define KINETIS_SDHC_ADSADDR_OFFSET    0x0058 /* ADMA System Address Register */
#define KINETIS_SDHC_VENDOR_OFFSET     0x00c0 /* Vendor Specific Register */
#define KINETIS_SDHC_MMCBOOT_OFFSET    0x00c4 /* MMC Boot Register */
#define KINETIS_SDHC_HOSTVER_OFFSET    0x00fc /* Host Controller Version */

/* Register Addresses ***************************************************************/

#define KINETIS_SDHC_DSADDR            (KINETIS_SDHC_BASE+KINETIS_SDHC_DSADDR_OFFSET)
#define KINETIS_SDHC_BLKATTR           (KINETIS_SDHC_BASE+KINETIS_SDHC_BLKATTR_OFFSET)
#define KINETIS_SDHC_CMDARG            (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDARG_OFFSET)
#define KINETIS_SDHC_XFERTYP           (KINETIS_SDHC_BASE+KINETIS_SDHC_XFERTYP_OFFSET)
#define KINETIS_SDHC_CMDRSP0           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP0_OFFSET)
#define KINETIS_SDHC_CMDRSP1           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP1_OFFSET)
#define KINETIS_SDHC_CMDRSP2           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP2_OFFSET)
#define KINETIS_SDHC_CMDRSP3           (KINETIS_SDHC_BASE+KINETIS_SDHC_CMDRSP3_OFFSET)
#define KINETIS_SDHC_DATPORT           (KINETIS_SDHC_BASE+KINETIS_SDHC_DATPORT_OFFSET)
#define KINETIS_SDHC_PRSSTAT           (KINETIS_SDHC_BASE+KINETIS_SDHC_PRSSTAT_OFFSET)
#define KINETIS_SDHC_PROCTL            (KINETIS_SDHC_BASE+KINETIS_SDHC_PROCTL_OFFSET)
#define KINETIS_SDHC_SYSCTL            (KINETIS_SDHC_BASE+KINETIS_SDHC_SYSCTL_OFFSET)
#define KINETIS_SDHC_IRQSTAT           (KINETIS_SDHC_BASE+KINETIS_SDHC_IRQSTAT_OFFSET)
#define KINETIS_SDHC_IRQSTATEN         (KINETIS_SDHC_BASE+KINETIS_SDHC_IRQSTATEN_OFFSET)
#define KINETIS_SDHC_IRQSIGEN          (KINETIS_SDHC_BASE+KINETIS_SDHC_IRQSIGEN_OFFSET)
#define KINETIS_SDHC_AC12ERR           (KINETIS_SDHC_BASE+KINETIS_SDHC_AC12ERR_OFFSET)
#define KINETIS_SDHC_HTCAPBLT          (KINETIS_SDHC_BASE+KINETIS_SDHC_HTCAPBLT_OFFSET)
#define KINETIS_SDHC_WML               (KINETIS_SDHC_BASE+KINETIS_SDHC_WML_OFFSET)
#define KINETIS_SDHC_FEVT              (KINETIS_SDHC_BASE+KINETIS_SDHC_FEVT_OFFSET)
#define KINETIS_SDHC_ADMAES            (KINETIS_SDHC_BASE+KINETIS_SDHC_ADMAES_OFFSET)
#define KINETIS_SDHC_ADSADDR           (KINETIS_SDHC_BASE+KINETIS_SDHC_ADSADDR_OFFSET)
#define KINETIS_SDHC_VENDOR            (KINETIS_SDHC_BASE+KINETIS_SDHC_VENDOR_OFFSET)
#define KINETIS_SDHC_MMCBOOT           (KINETIS_SDHC_BASE+KINETIS_SDHC_MMCBOOT_OFFSET)
#define KINETIS_SDHC_HOSTVER           (KINETIS_SDHC_BASE+KINETIS_SDHC_HOSTVER_OFFSET)

/* Register Bit Definitions *********************************************************/

/* DMA System Address Register */

#define SDHC_DSADDR_SHIFT               (1)       /* Bits 1-31: DMA System Address */
#define SDHC_DSADDR_MASK                (0xfffffffe)
                                                  /* Bits 0-1: Reserved */
/* Block Attributes Register */

#define SDHC_BLKATTR_SIZE_SHIFT         (0)       /* Bits 0-12: Transfer Block Size */
#define SDHC_BLKATTR_SIZE_MASK          (0x1fff << SDHC_BLKATTR_BLKSIZE_SHIFT)
                                                  /* Bits 13-15: Reserved */
#define SDHC_BLKATTR_CNT_SHIFT          (16)      /* Bits 16-31: Blocks Count For Current Transfer */
#define SDHC_BLKATTR_CNT_MASK           (0xffff << SDHC_BLKATTR_BLKCNT_SHIFT)

/* Command Argument Register (32-bit cmd/arg data) */

/* Transfer Type Register */

#define SDHC_XFERTYP_DMAEN              (1 << 0)  /* Bit 0:  DMA Enable */
#define SDHC_XFERTYP_BCEN               (1 << 1)  /* Bit 1:  Block Count Enable */
#define SDHC_XFERTYP_AC12EN             (1 << 2)  /* Bit 2:  Auto CMD12 Enable */
                                                  /* Bit 3: Reserved */
#define SDHC_XFERTYP_DTDSEL             (1 << 4)  /* Bit 4:  Data Transfer Direction Select */
#define SDHC_XFERTYP_MSBSEL             (1 << 5)  /* Bit 5:  Multi/Single Block Select */
                                                  /* Bits 6-15: Reserved */
#define SDHC_XFERTYP_RSPTYP_SHIFT       (16)      /* Bits 16-17: Response Type Select */
#define SDHC_XFERTYP_RSPTYP_MASK        (3 << SDHC_XFERTYP_RSPTYP_SHIFT)
#  define SDHC_XFERTYP_RSPTYP_NONE      (0 << SDHC_XFERTYP_RSPTYP_SHIFT) /* No response */
#  define SDHC_XFERTYP_RSPTYP_LEN136    (1 << SDHC_XFERTYP_RSPTYP_SHIFT) /* Response length 136 */
#  define SDHC_XFERTYP_RSPTYP_LEN48     (2 << SDHC_XFERTYP_RSPTYP_SHIFT) /* Response length 48 */
#  define SDHC_XFERTYP_RSPTYP_LEN48BSY  (3 << SDHC_XFERTYP_RSPTYP_SHIFT) /* Response length 48, check busy */
                                                  /* Bit 18: Reserved */
#define SDHC_XFERTYP_CCCEN              (1 << 19) /* Bit 19: Command CRC Check Enable */
#define SDHC_XFERTYP_CICEN              (1 << 20) /* Bit 20: Command Index Check Enable */
#define SDHC_XFERTYP_DPSEL              (1 << 21) /* Bit 21: Data Present Select */
#define SDHC_XFERTYP_CMDTYP_SHIFT       (22)      /* Bits 22-23: Command Type */
#define SDHC_XFERTYP_CMDTYP_MASK        (3 << SDHC_XFERTYP_CMDTYP_SHIFT)
#  define SDHC_XFERTYP_CMDTYP_NORMAL    (0 << SDHC_XFERTYP_CMDTYP_SHIFT) /* Normal other commands */
#  define SDHC_XFERTYP_CMDTYP_SUSPEND   (1 << SDHC_XFERTYP_CMDTYP_SHIFT) /* Suspend CMD52 for writing bus suspend in CCCR */
#  define SDHC_XFERTYP_CMDTYP_RESUME    (2 << SDHC_XFERTYP_CMDTYP_SHIFT) /* Resume CMD52 for writing function select in CCCR */
#  define SDHC_XFERTYP_CMDTYP_ABORT     (3 << SDHC_XFERTYP_CMDTYP_SHIFT) /* Abort CMD12, CMD52 for writing I/O abort in CCCR */
#define SDHC_XFERTYP_CMDINX_SHIFT       (24)      /* Bits 24-29: Command Index */
#define SDHC_XFERTYP_CMDINX_MASK        (63 << SDHC_XFERTYP_CMDINX_SHIFT)
                                                  /* Bits 30-31: Reserved */
/* Command Response 0-3 (32-bit response data) */

/* Buffer Data Port Register (32-bit data content) */

/* Present State Register */

#define SDHC_PRSSTAT_CIHB               (1 << 0)  /* Bit 0:  Command Inhibit (CMD) */
#define SDHC_PRSSTAT_CDIHB              (1 << 1)  /* Bit 1:  Command Inhibit (DAT) */
#define SDHC_PRSSTAT_DLA                (1 << 2)  /* Bit 2:  Data Line Active */
#define SDHC_PRSSTAT_SDSTB              (1 << 3)  /* Bit 3:  SD Clock Stable */
#define SDHC_PRSSTAT_IPGOFF             (1 << 4)  /* Bit 4:  Bus Clock */
#define SDHC_PRSSTAT_HCKOFF             (1 << 5)  /* Bit 5:  System Clock */
#define SDHC_PRSSTAT_PEROFF             (1 << 6)  /* Bit 6:  SDHC clock */
#define SDHC_PRSSTAT_SDOFF              (1 << 7)  /* Bit 7:  SD Clock Gated Off Internally */
#define SDHC_PRSSTAT_WTA                (1 << 8)  /* Bit 8:  Write Transfer Active */
#define SDHC_PRSSTAT_RTA                (1 << 9)  /* Bit 9:  Read Transfer Active */
#define SDHC_PRSSTAT_BWEN               (1 << 10) /* Bit 10: Buffer Write Enable */
#define SDHC_PRSSTAT_BREN               (1 << 11) /* Bit 11: Buffer Read Enable */
                                                  /* Bits 12-15: Reserved */
#define SDHC_PRSSTAT_CINS               (1 << 16) /* Bit 16: Card Inserted */
                                                  /* Bits 17-22: Reserved */
#define SDHC_PRSSTAT_CLSL               (1 << 23) /* Bit 23: CMD Line Signal Level */
#define SDHC_PRSSTAT_DLSL_SHIFT         (24)      /* Bits 24-31: DAT Line Signal Level */
#define SDHC_PRSSTAT_DLSL_MASK          (0xff << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT0        (0x01 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT1        (0x02 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT2        (0x04 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT3        (0x08 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT4        (0x10 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT5        (0x20 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT6        (0x40 << SDHC_PRSSTAT_DLSL_SHIFT)
#  define SDHC_PRSSTAT_DLSL_DAT7        (0x80 << SDHC_PRSSTAT_DLSL_SHIFT)

/* Protocol Control Register */

#define SDHC_PROCTL_LCTL                (1 << 0)  /* Bit 0:  LED Control */
#define SDHC_PROCTL_DTW_SHIFT           (1)       /* Bits 1-2: Data Transfer Width */
#define SDHC_PROCTL_DTW_MASK            (3 << SDHC_PROCTL_DTW_SHIFT)
#  define SDHC_PROCTL_DTW_1BIT          (0 << SDHC_PROCTL_DTW_SHIFT) /* 1-bit mode */
#  define SDHC_PROCTL_DTW_4BIT          (1 << SDHC_PROCTL_DTW_SHIFT) /* 4-bit mode */
#  define SDHC_PROCTL_DTW_8BIT          (2 << SDHC_PROCTL_DTW_SHIFT) /* 8-bit mode */
#define SDHC_PROCTL_D3CD                (1 << 3)  /* Bit nn: DAT3 as Card Detection Pin */
#define SDHC_PROCTL_EMODE_SHIFT         (4)       /* Bits 4-5: Endian mode */
#define SDHC_PROCTL_EMODE_MASK          (3 << SDHC_PROCTL_EMODE_SHIFT)
#  define SDHC_PROCTL_EMODE_BE          (0 << SDHC_PROCTL_EMODE_SHIFT) /* Big endian mode */
#  define SDHC_PROCTL_EMODE_HWBE        (1 << SDHC_PROCTL_EMODE_SHIFT) /* Half word big endian mode */
#  define SDHC_PROCTL_EMODE_LE          (2 << SDHC_PROCTL_EMODE_SHIFT) /* Little endian mode */
#define SDHC_PROCTL_CDTL                (1 << 6)  /* Bit 6:  Card Detect Test Level */
#define SDHC_PROCTL_CDSS                (1 << 7)  /* Bit 7:  Card Detect Signal Selection */
#define SDHC_PROCTL_DMAS_SHIFT          (8)       /* Bits 8-9: DMA Select */
#define SDHC_PROCTL_DMAS_MASK           (3 << SDHC_PROCTL_DMAS_SHIFT)
#  define SDHC_PROCTL_DMAS_NODMA        (0 << SDHC_PROCTL_DMAS_SHIFT) /* No DMA or simple DMA is selected */
#  define SDHC_PROCTL_DMAS_ADMA1        (1 << SDHC_PROCTL_DMAS_SHIFT) /* ADMA1 is selected */
#  define SDHC_PROCTL_DMAS_ADMA2        (2 << SDHC_PROCTL_DMAS_SHIFT) /* ADMA2 is selected */
                                                  /* Bits 10-15: Reserved */
#define SDHC_PROCTL_SABGREQ             (1 << 16) /* Bit 16: Stop At Block Gap Request */
#define SDHC_PROCTL_CREQ                (1 << 17) /* Bit 17: Continue Request */
#define SDHC_PROCTL_RWCTL               (1 << 18) /* Bit 18: Read Wait Control */
#define SDHC_PROCTL_IABG                (1 << 19) /* Bit 19: Interrupt At Block Gap */
                                                  /* Bits 20-23: Reserved */
#define SDHC_PROCTL_WECINT              (1 << 24) /* Bit 24: Wakeup Event Enable On Card Interrupt */
#define SDHC_PROCTL_WECINS              (1 << 25) /* Bit 25: Wakeup Event Enable On SD Card Insertion */
#define SDHC_PROCTL_WECRM               (1 << 26) /* Bit 26: Wakeup Event Enable On SD Card Removal */
                                                  /* Bits 27-31: Reserved */
/* System Control Register */

#define SDHC_SYSCTL_IPGEN               (1 << 0)  /* Bit 0:  IPG Clock Enable */
#define SDHC_SYSCTL_HCKEN               (1 << 1)  /* Bit 1:  System Clock Enable */
#define SDHC_SYSCTL_PEREN               (1 << 2)  /* Bit 2:  Peripheral Clock Enable */
#define SDHC_SYSCTL_SDCLKEN             (1 << 3)  /* Bit 3:  SD Clock Enable */
#define SDHC_SYSCTL_DVS_SHIFT           (4)       /* Bits 4-7: Divisor */
#define SDHC_SYSCTL_DVS_MASK            (15 << SDHC_SYSCTL_DVS_SHIFT)
#  define SDHC_SYSCTL_DVS_DIV(n)        (((n)-1) << SDHC_SYSCTL_DVS_SHIFT) /* Divide by n, n=1..16 */
#define SDHC_SYSCTL_SDCLKFS_SHIFT       (8)       /* Bits 8-15: SDCLK Frequency Select */
#define SDHC_SYSCTL_SDCLKFS_MASK        (0xff << SDHC_SYSCTL_SDCLKFS_SHIFT)
#  define SDHC_SYSCTL_SDCLKFS_BYPASS    (0x00 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Bypass the prescaler */
#  define SDHC_SYSCTL_SDCLKFS_DIV2      (0x01 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 2 */
#  define SDHC_SYSCTL_SDCLKFS_DIV4      (0x02 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 4 */
#  define SDHC_SYSCTL_SDCLKFS_DIV8      (0x04 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 8 */
#  define SDHC_SYSCTL_SDCLKFS_DIV16     (0x08 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 16 */
#  define SDHC_SYSCTL_SDCLKFS_DIV32     (0x10 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 32 */
#  define SDHC_SYSCTL_SDCLKFS_DIV64     (0x20 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 64 */
#  define SDHC_SYSCTL_SDCLKFS_DIV128    (0x40 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 128 */
#  define SDHC_SYSCTL_SDCLKFS_DIV256    (0x80 << SDHC_SYSCTL_SDCLKFS_SHIFT) /* Base clock / 256 */
#define SDHC_SYSCTL_DTOCV_SHIFT         (16)      /* Bits 16-19: Data Timeout Counter Value */
#define SDHC_SYSCTL_DTOCV_MASK          (15 << SDHC_SYSCTL_DTOCV_SHIFT)
#  define SDHC_SYSCTL_DTOCV_MUL(n)      (((n)-213) << SDHC_SYSCTL_DTOCV_SHIFT) /* SDCLK x n, n=213..227 */
                                                  /* Bits 20-23: Reserved */
#define SDHC_SYSCTL_RSTA                (1 << 24) /* Bit 24: Software Reset For ALL */
#define SDHC_SYSCTL_RSTC                (1 << 25) /* Bit 25: Software Reset For CMD Line */
#define SDHC_SYSCTL_RSTD                (1 << 26) /* Bit 26: Software Reset For DAT Line */
#define SDHC_SYSCTL_INITA               (1 << 27) /* Bit 27: Initialization Active */
                                                  /* Bits 28-31: Reserved */
/* Interrupt Status Register, Interrupt Status Enable Register, and Interrupt Signal Enable Register
 * Common interrupt bit definitions
 */

#define SDHC_INT_CC                     (1 << 0)  /* Bit 0:  Command Complete */
#define SDHC_INT_TC                     (1 << 1)  /* Bit 1:  Transfer Complete */
#define SDHC_INT_BGE                    (1 << 2)  /* Bit 2:  Block Gap Event */
#define SDHC_INT_DINT                   (1 << 3)  /* Bit 3:  DMA Interrupt */
#define SDHC_INT_BWR                    (1 << 4)  /* Bit 4:  Buffer Write Ready */
#define SDHC_INT_BRR                    (1 << 5)  /* Bit 5:  Buffer Read Ready */
#define SDHC_INT_CINS                   (1 << 6)  /* Bit 6:  Card Insertion */
#define SDHC_INT_CRM                    (1 << 7)  /* Bit 7:  Card Removal */
#define SDHC_INT_CINT                   (1 << 8)  /* Bit 8:  Card Interrupt */
                                                  /* Bits 9-15: Reserved */
#define SDHC_INT_CTOE                   (1 << 16) /* Bit 16: Command Timeout Error */
#define SDHC_INT_CCE                    (1 << 17) /* Bit 17: Command CRC Error */
#define SDHC_INT_CEBE                   (1 << 18) /* Bit 18: Command End Bit Error */
#define SDHC_INT_CIE                    (1 << 19) /* Bit 19: Command Index Error */
#define SDHC_INT_DTOE                   (1 << 20) /* Bit 20: Data Timeout Error */
#define SDHC_INT_DCE                    (1 << 21) /* Bit 21: Data CRC Error */
#define SDHC_INT_DEBE                   (1 << 22) /* Bit 22: Data End Bit Error */
                                                  /* Bit 23: Reserved */
#define SDHC_INT_AC12E                  (1 << 24) /* Bit 24: Auto CMD12 Error */
                                                  /* Bits 25-27: Reserved */
#define SDHC_INT_DMAE                   (1 << 28) /* Bit 28: DMA Error */
                                                  /* Bits 29-31: Reserved */
#define SDHC_INT_ALL                    0x117f01ff

/* Auto CMD12 Error Status Register */

#define SDHC_AC12ERR_NE                 (1 << 0)  /* Bit 0:  Auto CMD12 Not Executed */
#define SDHC_AC12ERR_TOE                (1 << 1)  /* Bit 1:  Auto CMD12 Timeout Error */
#define SDHC_AC12ERR_EBE                (1 << 2)  /* Bit 2:  Auto CMD12 End Bit Error */
#define SDHC_AC12ERR_CE                 (1 << 3)  /* Bit 3:  Auto CMD12 CRC Error */
#define SDHC_AC12ERR_IE                 (1 << 4)  /* Bit 4:  Auto CMD12 Index Error */
                                                  /* Bits 5-6: Reserved */
#define SDHC_AC12ERR_CNI                (1 << 7)  /* Bit 7: Command Not Issued By Auto CMD12 Error */
                                                  /* Bits 8-31: Reserved */
/* Host Controller Capabilities */
                                                  /* Bits 0-15: Reserved */
#define SDHC_HTCAPBLT_MBL_SHIFT         (16)      /* Bits 16-18: Max Block Length */
#define SDHC_HTCAPBLT_MBL_MASK          (7 << SDHC_HTCAPBLT_MBL_SHIFT)
#  define SDHC_HTCAPBLT_MBL_512BYTES    (0 << SDHC_HTCAPBLT_MBL_SHIFT)
#  define SDHC_HTCAPBLT_MBL_1KB         (1 << SDHC_HTCAPBLT_MBL_SHIFT)
#  define SDHC_HTCAPBLT_MBL_2KB         (2 << SDHC_HTCAPBLT_MBL_SHIFT)
#  define SDHC_HTCAPBLT_MBL_4KB         (3 << SDHC_HTCAPBLT_MBL_SHIFT)
                                                  /* Bit 19: Reserved */
#define SDHC_HTCAPBLT_ADMAS             (1 << 20) /* Bit 20: ADMA Support */
#define SDHC_HTCAPBLT_HSS               (1 << 21) /* Bit 21: High Speed Support */
#define SDHC_HTCAPBLT_DMAS              (1 << 22) /* Bit 22: DMA Support */
#define SDHC_HTCAPBLT_SRS               (1 << 23) /* Bit 23: Suspend/Resume Support */
#define SDHC_HTCAPBLT_VS33              (1 << 24) /* Bit 24: Voltage Support 3.3 V */
#define SDHC_HTCAPBLT_VS30              (1 << 25) /* Bit 25: Voltage Support 3.0 V */
#define SDHC_HTCAPBLT_VS18              (1 << 26) /* Bit 26: Voltage Support 1.8 */
                                                  /* Bits 27-31: Reserved */
/* Watermark Level Register */

#define SDHC_WML_RD_SHIFT               (0)       /* Bits 0-7: Read Watermark Level */
#define SDHC_WML_RD_MASK                (0xff << SDHC_WML_RDWML_SHIFT)
                                                  /* Bits 8-15: Reserved */
#define SDHC_WML_WR_SHIFT               (16)      /* Bits 16-23: Write Watermark Level */
#define SDHC_WML_WR_MASK                (0xff << SDHC_WML_WRWML_SHIFT)
                                                  /* Bits 24-31: Reserved */
/* Force Event Register */

#define SDHC_FEVT_AC12NE                (1 << 0)  /* Bit 0:  Force Event Auto Command 12 Not Executed */
#define SDHC_FEVT_AC12TOE               (1 << 1)  /* Bit 1:  Force Event Auto Command 12 Time Out Error */
#define SDHC_FEVT_AC12CE                (1 << 2)  /* Bit 2:  Force Event Auto Command 12 CRC Error */
#define SDHC_FEVT_AC12EBE               (1 << 3)  /* Bit 3:  Force Event Auto Command 12 End Bit Error */
#define SDHC_FEVT_AC12IE                (1 << 4)  /* Bit 4:  Force Event Auto Command 12 Index Error */
                                                  /* Bits 5-6: Reserved */
#define SDHC_FEVT_CNIBAC12E             (1 << 7)  /* Bit 7:  Force Event Command Not Executed By Auto Command 12 Error */
                                                  /* Bits 8-15: Reserved */
#define SDHC_FEVT_CTOE                  (1 << 16) /* Bit 16: Force Event Command Time Out Error */
#define SDHC_FEVT_CCE                   (1 << 17) /* Bit 17: Force Event Command CRC Error */
#define SDHC_FEVT_CEBE                  (1 << 18) /* Bit 18: Force Event Command End Bit Error */
#define SDHC_FEVT_CIE                   (1 << 19) /* Bit 19: Force Event Command Index Error */
#define SDHC_FEVT_DTOE                  (1 << 20) /* Bit 20: Force Event Data Time Out Error */
#define SDHC_FEVT_DCE                   (1 << 21) /* Bit 21: Force Event Data CRC Error */
#define SDHC_FEVT_DEBE                  (1 << 22) /* Bit 22: Force Event Data End Bit Error */
                                                  /* Bit 23: Reserved */
#define SDHC_FEVT_AC12E                 (1 << 24) /* Bit 24: Force Event Auto Command 12 Error */
                                                  /* Bits 25-27: Reserved */
#define SDHC_FEVT_DMAE                  (1 << 28) /* Bit 28: Force Event DMA Error */
                                                  /* Bits 29-30: Reserved */
#define SDHC_FEVT_CINT                  (1 << 31) /* Bit 31: Force Event Card Interrupt */

/* ADMA Error Status Register */

#define SDHC_ADMAES_SHIFT               (0)       /* Bits 0-1: ADMA Error State (when ADMA Error is occurred) */
#define SDHC_ADMAES_MASK                (3 << SDHC_ADMAES_ADMAES_SHIFT)
#  define SDHC_ADMAES_STOP              (0 << SDHC_ADMAES_ADMAES_SHIFT) /* Stop DMA */
#  define SDHC_ADMAES_FDS               (1 << SDHC_ADMAES_ADMAES_SHIFT) /* Fetch descriptor */
#  define SDHC_ADMAES_CADR              (2 << SDHC_ADMAES_ADMAES_SHIFT) /* Change address */
#  define SDHC_ADMAES_TFR               (3 << SDHC_ADMAES_ADMAES_SHIFT) /* Transfer data */
#define SDHC_ADMAES_LME                 (1 << 2)  /* Bit 2:  ADMA Length Mismatch Error */
#define SDHC_ADMAES_DCE                 (1 << 3)  /* Bit 3:  ADMA Descriptor Error */
                                                  /* Bits 4-31: Reserved */
/* ADMA System Address Register */

#define SDHC_ADSADDR_SHIFT              (1)       /* Bits 1-31: ADMA System Address */
#define SDHC_ADSADDR_MASK               (0xfffffffe)
                                                  /* Bits 0-1: Reserved */

/* Vendor Specific Register */

#define SDHC_VENDOR_EXTDMAEN            (1 << 0)  /* Bit 0:  External DMA Request Enable */
#define SDHC_VENDOR_EXBLKNU             (1 << 1)  /* Bit 1:  Exact block number block read enable for SDIO CMD53 */
                                                  /* Bits 2-15: Reserved */
#define SDHC_VENDOR_INTSTVAL_SHIFT      (16)      /* Bits 16-23: Internal State Value */
#define SDHC_VENDOR_INTSTVAL_MASK       (0xff << SDHC_VENDOR_INTSTVAL_SHIFT)
                                                  /* Bits 24-31: Reserved */
/* MMC Boot Register */

#define SDHC_MMCBOOT_DTOCVACK_SHIFT     (0)       /* Bits 0-3: Boot ACK time out counter value */
#define SDHC_MMCBOOT_DTOCVACK_MASK      (15 << SDHC_MMCBOOT_DTOCVACK_SHIFT)
#  define SDHC_MMCBOOT_DTOCVACK_MUL(n)  ((n-8) << SDHC_MMCBOOT_DTOCVACK_SHIFT) /* SDCLK x 2^n, n=8..22 */
#define SDHC_MMCBOOT_BOOTACK            (1 << 4)  /* Bit 4:  Boot ack mode select */
#define SDHC_MMCBOOT_BOOTMODE           (1 << 5)  /* Bit 5:  Boot mode select */
#define SDHC_MMCBOOT_BOOTEN             (1 << 6)  /* Bit 6:  Boot mode enable */
#define SDHC_MMCBOOT_AUTOSABGEN         (1 << 7)  /* Bit 7:  Enable auto stop at block gap function */
                                                  /* Bits 8-15: Reserved */
#define SDHC_MMCBOOT_BOOTBLKCNT_SHIFT   (16)      /* Bits 16-31: Stop at block gap value of automatic mode */
#define SDHC_MMCBOOT_BOOTBLKCNT_MASK    (0xffff << SDHC_MMCBOOT_BOOTBLKCNT_SHIFT)

/* Host Controller Version */

#define SDHC_HOSTVER_SVN_SHIFT          (0)       /* Bits 0-7: Specification Version Number */
#define SDHC_HOSTVER_SVN_MASK           (0xff << SDHC_HOSTVER_SVN_SHIFT)
#define SDHC_HOSTVER_VVN_SHIFT          (8)       /* Bits 8-15: Vendor Version Number */
#define SDHC_HOSTVER_VVN_MASK           (0xff << SDHC_HOSTVER_VVN_SHIFT)
                                                  /* Bits 16-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_KINETIS_KINETIS_SDHC_H */
