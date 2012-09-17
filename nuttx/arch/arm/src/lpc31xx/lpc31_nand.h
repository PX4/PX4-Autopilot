/************************************************************************************************
 * arch/arm/src/lpc31xx/lpc31_nand.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC31XX_LPC31_NAND_H
#define __ARCH_ARM_SRC_LPC31XX_LPC31_NAND_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "lpc31_memorymap.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

/* NAND FLASH controller register base address offset into the APB4 domain **********************/

#define LPC31_NAND_VBASE                (LPC31_APB4_VADDR+LPC31_APB4_NAND_OFFSET)
#define LPC31_NAND_PBASE                (LPC31_APB4_PADDR+LPC31_APB4_NAND_OFFSET)

/* NAND FLASH controller register offsets (with respect to the base of the APB4 domain) *********/

#define LPC31_NAND_IRQSTATUS1_OFFSET    0x00 /* Interrrupt status register (first 32-bits) */
#define LPC31_NAND_IRQMASK1_OFFSET      0x04 /* Interrupt mask register (first 32-bits) */
#define LPC31_NAND_IRQSTATUSRAW1_OFFSET 0x08 /* Unmasked register status (first 32-bits) */
#define LPC31_NAND_CONFIG_OFFSET        0x0c /* NAND Flash controller configuration register */
#define LPC31_NAND_IOCONFIG_OFFSET      0x10 /* Default value settings for IO signals */
#define LPC31_NAND_TIMING1_OFFSET       0x14 /* First NAND FLASH controller timing register */
#define LPC31_NAND_TIMING2_OFFSET       0x18 /* Second NAND FLASH controller timing register */
#define LPC31_NAND_SETCMD_OFFSET        0x20 /* NAND FLASH device command register */
#define LPC31_NAND_SETADDR_OFFSET       0x24 /* NAND FLASH device address register */
#define LPC31_NAND_WRITEDATA_OFFSET     0x28 /* NAND FLASH device write data register */
#define LPC31_NAND_SETCE_OFFSET         0x2c /* Set all CE and WP_n signals */
#define LPC31_NAND_READDATA_OFFSET      0x30 /* NAND FLASH device read data register */
#define LPC31_NAND_CHECKSTS_OFFSET      0x34 /* Check status of interrupts */
#define LPC31_NAND_CONTROLFLOW_OFFSET   0x38 /* Commands to read and write pages */
#define LPC31_NAND_GPIO1_OFFSET         0x40 /* Program IO pins that can be used as GPIO */
#define LPC31_NAND_GPIO2_OFFSET         0x44 /* Program IO pins that can be used as GPIO */
#define LPC31_NAND_IRQSTATUS2_OFFSET    0x48 /* Interrrupt status register (second 32-bits) */
#define LPC31_NAND_IRQMASK3_OFFSET      0x4c /* Interrupt mask register (second 32-bits) */
#define LPC31_NAND_IRQSTATUSRAW2_OFFSET 0x50 /* Unmasked register status (second 32-bits) */
#define LPC31_NAND_AESKEY1_OFFSET       0x54 /* First word of 128-bit AES key (LPC3154 only) */
#define LPC31_NAND_AESKEY2_OFFSET       0x58 /* Second word of 128-bit AES key (LPC3154 only) */
#define LPC31_NAND_AESKEY3_OFFSET       0x5c /* Third word of 128-bit AES key (LPC3154 only) */
#define LPC31_NAND_AESKEY4_OFFSET       0x60 /* Fourth word of 128-bit AES key (LPC3154 only) */
#define LPC31_NAND_AESIV1_OFFSET        0x64 /* First word of 128-bit initial AES value (LPC3154 only) */
#define LPC31_NAND_AESIV2_OFFSET        0x68 /* Second word of 128-bit initial AES value (LPC3154 only) */
#define LPC31_NAND_AESIV3_OFFSET        0x6c /* Third word of 128-bit initial AES value (LPC3154 only) */
#define LPC31_NAND_AESIV4_OFFSET        0x70 /* Fourth word of 128-bit initial AES value (LPC3154 only) */
#define LPC31_NAND_AESSTATE_OFFSET      0x74 /* Register to display AES state (LPC3154 only) */
#define LPC31_NAND_ECCERRSTATUS_OFFSET  0x78 /* ECC error status register */
#define LPC31_NAND_AESFROMAHB_OFFSET    0x7c /* Enable AES engine from AHB */

/* NAND FLASH controller register (virtual) addresses *******************************************/

#define LPC31_NAND_IRQSTATUS1           (LPC31_NAND_VBASE+LPC31_NAND_IRQSTATUS1_OFFSET)
#define LPC31_NAND_IRQMASK1             (LPC31_NAND_VBASE+LPC31_NAND_IRQMASK1_OFFSET)
#define LPC31_NAND_IRQSTATUSRAW1        (LPC31_NAND_VBASE+LPC31_NAND_IRQSTATUSRAW1_OFFSET)
#define LPC31_NAND_CONFIG               (LPC31_NAND_VBASE+LPC31_NAND_CONFIG_OFFSET)
#define LPC31_NAND_IOCONFIG             (LPC31_NAND_VBASE+LPC31_NAND_IOCONFIG_OFFSET)
#define LPC31_NAND_TIMING1              (LPC31_NAND_VBASE+LPC31_NAND_TIMING1_OFFSET)
#define LPC31_NAND_TIMING2              (LPC31_NAND_VBASE+LPC31_NAND_TIMING2_OFFSET)
#define LPC31_NAND_SETCMD               (LPC31_NAND_VBASE+LPC31_NAND_SETCMD_OFFSET)
#define LPC31_NAND_SETADDR              (LPC31_NAND_VBASE+LPC31_NAND_SETADDR_OFFSET)
#define LPC31_NAND_WRITEDATA            (LPC31_NAND_VBASE+LPC31_NAND_WRITEDATA_OFFSET)
#define LPC31_NAND_SETCE                (LPC31_NAND_VBASE+LPC31_NAND_SETCE_OFFSET)
#define LPC31_NAND_READDATA             (LPC31_NAND_VBASE+LPC31_NAND_READDATA_OFFSET)
#define LPC31_NAND_CHECKSTS             (LPC31_NAND_VBASE+LPC31_NAND_CHECKSTS_OFFSET)
#define LPC31_NAND_CONTROLFLOW          (LPC31_NAND_VBASE+LPC31_NAND_CONTROLFLOW_OFFSET)
#define LPC31_NAND_GPIO1                (LPC31_NAND_VBASE+LPC31_NAND_GPIO1_OFFSET)
#define LPC31_NAND_GPIO2                (LPC31_NAND_VBASE+LPC31_NAND_GPIO2_OFFSET)
#define LPC31_NAND_IRQSTATUS2           (LPC31_NAND_VBASE+LPC31_NAND_IRQSTATUS2_OFFSET)
#define LPC31_NAND_IRQMASK3             (LPC31_NAND_VBASE+LPC31_NAND_IRQMASK3_OFFSET)
#define LPC31_NAND_IRQSTATUSRAW2        (LPC31_NAND_VBASE+LPC31_NAND_IRQSTATUSRAW2_OFFSET)
#define LPC31_NAND_AESKEY1              (LPC31_NAND_VBASE+LPC31_NAND_AESKEY1_OFFSET)
#define LPC31_NAND_AESKEY2              (LPC31_NAND_VBASE+LPC31_NAND_AESKEY2_OFFSET)
#define LPC31_NAND_AESKEY3              (LPC31_NAND_VBASE+LPC31_NAND_AESKEY3_OFFSET)
#define LPC31_NAND_AESKEY4              (LPC31_NAND_VBASE+LPC31_NAND_AESKEY4_OFFSET)
#define LPC31_NAND_AESIV1               (LPC31_NAND_VBASE+LPC31_NAND_AESIV1_OFFSET)
#define LPC31_NAND_AESIV2               (LPC31_NAND_VBASE+LPC31_NAND_AESIV2_OFFSET)
#define LPC31_NAND_AESIV3               (LPC31_NAND_VBASE+LPC31_NAND_AESIV3_OFFSET)
#define LPC31_NAND_AESIV4               (LPC31_NAND_VBASE+LPC31_NAND_AESIV4_OFFSET)
#define LPC31_NAND_AESSTATE             (LPC31_NAND_VBASE+LPC31_NAND_AESSTATE_OFFSET)
#define LPC31_NAND_ECCERRSTATUS         (LPC31_NAND_VBASE+LPC31_NAND_ECCERRSTATUS_OFFSET)
#define LPC31_NAND_AESFROMAHB           (LPC31_NAND_VBASE+LPC31_NAND_AESFROMAHB_OFFSET)

/* NAND FLASH controller register bit definitions ***********************************************/
/* NandIRQStatus1 register description (NandIRQStatus1, address 0x17000800) */

#define NAND_IRQSTATUS1_MNANDRYBN3        (1 << 31) /* Bit 31: mNAND_RYBN3 positive edge */
#define NAND_IRQSTATUS1_MNANDRYBN2        (1 << 30) /* Bit 30: mNAND_RYBN2 positive edge */
#define NAND_IRQSTATUS1_MNANDRYBN1        (1 << 29) /* Bit 29: mNAND_RYBN1 positive edge */
#define NAND_IRQSTATUS1_MNANDRYBN0        (1 << 28) /* Bit 28: mNAND_RYBN0 positive edge */
#define NAND_IRQSTATUS1_RAM1ERASED        (1 << 27) /* Bit 27: RAM 1 erased */
#define NAND_IRQSTATUS1_RAM0ERASED        (1 << 26) /* Bit 26: RAM 0 erased */
#define NAND_IRQSTATUS1_WRPG1DONE         (1 << 25) /* Bit 25: Write page 1 done */
#define NAND_IRQSTATUS1_WRPG0DONE         (1 << 24) /* Bit 24: Write page 0 done */
#define NAND_IRQSTATUS1_RDPG1DONE         (1 << 23) /* Bit 23: Read page 1 done */
#define NAND_IRQSTATUS1_RDPG0DONE         (1 << 22) /* Bit 22: Read page 0 done */
#define NAND_IRQSTATUS1_RAM0DECODE        (1 << 21) /* Bit 21: RAM 0 decoded */
#define NAND_IRQSTATUS1_RAM0ENCODE        (1 << 20) /* Bit 20: RAM 0 encoded */
#define NAND_IRQSTATUS1_RAM1DECODE        (1 << 19) /* Bit 19: RAM 1 decoded */
#define NAND_IRQSTATUS1_RAM1ENCODE        (1 << 18) /* Bit 18: RAM 1 encoded */
#define NAND_IRQSTATUS1_RAM00ERRS         (1 << 17) /* Bit 17: RAM 0 decoded with 0 errors */
#define NAND_IRQSTATUS1_RAM01ERR          (1 << 16) /* Bit 16: RAM 0 decoded with 1 error (depends on mode) */
#define NAND_IRQSTATUS1_RAM02ERR          (1 << 15) /* Bit 15: RAM 0 decoded with 2 error */
#define NAND_IRQSTATUS1_RAM03ERR          (1 << 14) /* Bit 14: RAM 0 decoded with 3 error */
#define NAND_IRQSTATUS1_RAM04ERR          (1 << 13) /* Bit 13: RAM 0 decoded with 4 error */
#define NAND_IRQSTATUS1_RAM05ERR          (1 << 12) /* Bit 12: RAM 0 decoded with 5 error */
#define NAND_IRQSTATUS1_RAM0UNCORR        (1 << 11) /* Bit 11: RAM 0 uncorrectable */
#define NAND_IRQSTATUS1_RAM10ERR          (1 << 10) /* Bit 10: RAM 1 decoded with 0 errors */
#define NAND_IRQSTATUS1_RAM11ERR          (1 << 9)  /* Bit 9:  RAM 1 decoded with 1 error (depends on mode) */
#define NAND_IRQSTATUS1_RAM12ERR          (1 << 8)  /* Bit 8:  RAM 1 decoded with 2 error */
#define NAND_IRQSTATUS1_RAM13ERR          (1 << 7)  /* Bit 7:  RAM 1 decoded with 3 error */
#define NAND_IRQSTATUS1_RAM14ERR          (1 << 6)  /* Bit 6:  RAM 1 decoded with 4 error */
#define NAND_IRQSTATUS1_RAM15ERR          (1 << 5)  /* Bit 5:  RAM 1 decoded with 5 error */
#define NAND_IRQSTATUS1_RAM1UNCORR        (1 << 4)  /* Bit 4:  RAM 1 uncorrectable */

#define NAND_IRQSTATUS1_RAM1AESDONE       (1 << 1)  /* Bit 1:  RAM 1 AES done (LPC3154 only) */
#define NAND_IRQSTATUS1_RAM0AESDONE       (1 << 0)  /* Bit 0:  RAM 0 AES done (LPC3154 only) */

/* NandIRQMask1 register description (NandIRQMask1, address 0x17000804) */

#define NAND_IRQIRQMASK1_MNANDRYBN3       (1 << 31) /* Bit 31: mNAND_RYBN3 positive edge */
#define NAND_IRQIRQMASK1_MNANDRYBN2       (1 << 30) /* Bit 30: mNAND_RYBN2 positive edge */
#define NAND_IRQIRQMASK1_MNANDRYBN1       (1 << 29) /* Bit 29: mNAND_RYBN1 positive edge */
#define NAND_IRQIRQMASK1_MNANDRYBN0       (1 << 28) /* Bit 28: mNAND_RYBN0 positive edge */
#define NAND_IRQIRQMASK1_RAM1ERASED       (1 << 27) /* Bit 27: RAM 1 erased */
#define NAND_IRQIRQMASK1_RAM0ERASED       (1 << 26) /* Bit 26: RAM 0 erased */
#define NAND_IRQIRQMASK1_WRPG1DONE        (1 << 25) /* Bit 25: Write page 1 done */
#define NAND_IRQIRQMASK1_WRPG0DONE        (1 << 24) /* Bit 24: Write page 0 done */
#define NAND_IRQIRQMASK1_RDPG1DONE        (1 << 23) /* Bit 23: Read page 1 done */
#define NAND_IRQIRQMASK1_RDPG0DONE        (1 << 22) /* Bit 22: Read page 0 done */
#define NAND_IRQIRQMASK1_RAM0DECODE       (1 << 21) /* Bit 21: RAM 0 decoded */
#define NAND_IRQIRQMASK1_RAM0ENCODE       (1 << 20) /* Bit 20: RAM 0 encoded */
#define NAND_IRQIRQMASK1_RAM1DECODE       (1 << 19) /* Bit 19: RAM 1 decoded */
#define NAND_IRQIRQMASK1_RAM1ENCODE       (1 << 18) /* Bit 18: RAM 1 encoded */
#define NAND_IRQIRQMASK1_RAM00ERRS        (1 << 17) /* Bit 17: RAM 0 decoded with 0 errors */
#define NAND_IRQIRQMASK1_RAM01ERR         (1 << 16) /* Bit 16: RAM 0 decoded with 1 error (depends on mode) */
#define NAND_IRQIRQMASK1_RAM02ERR         (1 << 15) /* Bit 15: RAM 0 decoded with 2 error */
#define NAND_IRQIRQMASK1_RAM03ERR         (1 << 14) /* Bit 14: RAM 0 decoded with 3 error */
#define NAND_IRQIRQMASK1_RAM04ERR         (1 << 13) /* Bit 13: RAM 0 decoded with 4 error */
#define NAND_IRQIRQMASK1_RAM05ERR         (1 << 12) /* Bit 12: RAM 0 decoded with 5 error */
#define NAND_IRQIRQMASK1_RAM0UNCORR       (1 << 11) /* Bit 11: RAM 0 uncorrectable */
#define NAND_IRQIRQMASK1_RAM10ERR         (1 << 10) /* Bit 10: RAM 1 decoded with 0 errors */
#define NAND_IRQIRQMASK1_RAM11ERR         (1 << 9)  /* Bit 9:  RAM 1 decoded with 1 error (depends on mode) */
#define NAND_IRQIRQMASK1_RAM12ERR         (1 << 8)  /* Bit 8:  RAM 1 decoded with 2 error */
#define NAND_IRQIRQMASK1_RAM13ERR         (1 << 7)  /* Bit 7:  RAM 1 decoded with 3 error */
#define NAND_IRQIRQMASK1_RAM14ERR         (1 << 6)  /* Bit 6:  RAM 1 decoded with 4 error */
#define NAND_IRQIRQMASK1_RAM15ERR         (1 << 5)  /* Bit 5:  RAM 1 decoded with 5 error */
#define NAND_IRQIRQMASK1_RAM1UNCORR       (1 << 4)  /* Bit 4:  RAM 1 uncorrectable */

#define NAND_IRQIRQMASK1_RAM1AESDONE      (1 << 1)  /* Bit 1:  RAM 1 AES done (LPC3154 only) */
#define NAND_IRQIRQMASK1_RAM0AESDONE      (1 << 0)  /* Bit 0:  RAM 0 AES done (LPC3154 only) */

/* NandIRQStatusRaw1 register description (NandIRQStatusRaw1, address 0x17000808) */

#define NAND_IRQSTATUSRAW1_MNANDRYBN3     (1 << 31) /* Bit 31: mNAND_RYBN3 positive edge */
#define NAND_IRQSTATUSRAW1_MNANDRYBN2     (1 << 30) /* Bit 30: mNAND_RYBN2 positive edge */
#define NAND_IRQSTATUSRAW1_MNANDRYBN1     (1 << 29) /* Bit 29: mNAND_RYBN1 positive edge */
#define NAND_IRQSTATUSRAW1_MNANDRYBN0     (1 << 28) /* Bit 28: mNAND_RYBN0 positive edge */
#define NAND_IRQSTATUSRAW1_RAM1ERASED     (1 << 27) /* Bit 27: RAM 1 erased */
#define NAND_IRQSTATUSRAW1_RAM0ERASED     (1 << 26) /* Bit 26: RAM 0 erased */
#define NAND_IRQSTATUSRAW1_WRPG1DONE      (1 << 25) /* Bit 25: Write page 1 done */
#define NAND_IRQSTATUSRAW1_WRPG0DONE      (1 << 24) /* Bit 24: Write page 0 done */
#define NAND_IRQSTATUSRAW1_RDPG1DONE      (1 << 23) /* Bit 23: Read page 1 done */
#define NAND_IRQSTATUSRAW1_RDPG0DONE      (1 << 22) /* Bit 22: Read page 0 done */
#define NAND_IRQSTATUSRAW1_RAM0DECODE     (1 << 21) /* Bit 21: RAM 0 decoded */
#define NAND_IRQSTATUSRAW1_RAM0ENCODE     (1 << 20) /* Bit 20: RAM 0 encoded */
#define NAND_IRQSTATUSRAW1_RAM1DECODE     (1 << 19) /* Bit 19: RAM 1 decoded */
#define NAND_IRQSTATUSRAW1_RAM1ENCODE     (1 << 18) /* Bit 18: RAM 1 encoded */
#define NAND_IRQSTATUSRAW1_RAM00ERRS      (1 << 17) /* Bit 17: RAM 0 decoded with 0 errors */
#define NAND_IRQSTATUSRAW1_RAM01ERR       (1 << 16) /* Bit 16: RAM 0 decoded with 1 error (depends on mode) */
#define NAND_IRQSTATUSRAW1_RAM02ERR       (1 << 15) /* Bit 15: RAM 0 decoded with 2 error */
#define NAND_IRQSTATUSRAW1_RAM03ERR       (1 << 14) /* Bit 14: RAM 0 decoded with 3 error */
#define NAND_IRQSTATUSRAW1_RAM04ERR       (1 << 13) /* Bit 13: RAM 0 decoded with 4 error */
#define NAND_IRQSTATUSRAW1_RAM05ERR       (1 << 12) /* Bit 12: RAM 0 decoded with 5 error */
#define NAND_IRQSTATUSRAW1_RAM0UNCORR     (1 << 11) /* Bit 11: RAM 0 uncorrectable */
#define NAND_IRQSTATUSRAW1_RAM10ERR       (1 << 10) /* Bit 10: RAM 1 decoded with 0 errors */
#define NAND_IRQSTATUSRAW1_RAM11ERR       (1 << 9)  /* Bit 9:  RAM 1 decoded with 1 error (depends on mode) */
#define NAND_IRQSTATUSRAW1_RAM12ERR       (1 << 8)  /* Bit 8:  RAM 1 decoded with 2 error */
#define NAND_IRQSTATUSRAW1_RAM13ERR       (1 << 7)  /* Bit 7:  RAM 1 decoded with 3 error */
#define NAND_IRQSTATUSRAW1_RAM14ERR       (1 << 6)  /* Bit 6:  RAM 1 decoded with 4 error */
#define NAND_IRQSTATUSRAW1_RAM15ERR       (1 << 5)  /* Bit 5:  RAM 1 decoded with 5 error */
#define NAND_IRQSTATUSRAW1_RAM1UNCORR     (1 << 4)  /* Bit 4:  RAM 1 uncorrectable */

#define NAND_IRQSTATUSRAW1_RAM1AESDONE    (1 << 1)  /* Bit 1:  RAM 1 AES done (LPC3154 only) */
#define NAND_IRQSTATUSRAW1_RAM0AESDONE    (1 << 0)  /* Bit 0:  RAM 0 AES done (LPC3154 only) */

/* NandConfig register description (NandConfig, address 0x1700080c) */

#define NAND_CONFIG_ECC_MODE              (1 << 12) /* Bit 12: ECC mode 0 */
#define NAND_CONFIG_TL_SHIFT              (10)      /* Bits 10-11: Transfer limit */
#define NAND_CONFIG_TL_MASK               (3 <<NAND_CONFIG_TL_SHIFT)
#  define NAND_CONFIG_TL_528              (0 <<NAND_CONFIG_TL_SHIFT)
#  define NAND_CONFIG_TL_516              (1 <<NAND_CONFIG_TL_SHIFT)
#  define NAND_CONFIG_TL_512              (2 <<NAND_CONFIG_TL_SHIFT)
#define NAND_CONFIG_DC                    (1 << 8)  /* Bit 8:  Deactivate CE enable */
#define NAND_CONFIG_M                     (1 << 7)  /* Bit 7:  512 mode */
#define NAND_CONFIG_LC_SHIFT              (5)       /* Bits 5-6: Latency Configuration */
#define NAND_CONFIG_LC_MASK               (3 << NAND_CONFIG_LC_SHIFT)
#  define NAND_CONFIG_LC_0WAITSTATES      (0 << NAND_CONFIG_LC_SHIFT)
#  define NAND_CONFIG_LC_1WAITSTATES      (1 << NAND_CONFIG_LC_SHIFT)
#  define NAND_CONFIG_LC_2WAITSTATES      (2 << NAND_CONFIG_LC_SHIFT)
#define NAND_CONFIG_ES                    (1 << 4)  /* Bit 4:  Endianess setting */
#define NAND_CONFIG_DE                    (1 << 3)  /* Bit 3:  DMA external enable */
#define NAND_CONFIG_AO                    (1 << 2)  /* Bit 2:  AES on (LPC3154 only) */
#define NAND_CONFIG_WD                    (1 << 1)  /* Bit 1:  Wide device */
#define NAND_CONFIG_EC                    (1 << 0)  /* Bit 0:  ECC on */

/* NandIOConfig register description (NandIOConfig, address 0x1700 0810) */

#define NAND_IOCONFIG_NI                  (1 << 24) /* Bit 24: Nand IO drive default */
#define NAND_IOCONFIG_DN_SHIFT            (8)       /* Bits 8-23: Data to NAND default */
#define NAND_IOCONFIG_DN_MASK             (0xffff << NAND_IOCONFIG_DN_SHIFT)
#define NAND_IOCONFIG_CD_SHIFT            (6)       /* Bits 6-7: CLE default */
#define NAND_IOCONFIG_CD_MASK             (3 << NAND_IOCONFIG_CD_SHIFT)
#define NAND_IOCONFIG_AD_SHIFT            (4)       /* Bits 4-5: ALE default */
#define NAND_IOCONFIG_AD_MASK             (3 << NAND_IOCONFIG_AD_SHIFT)
#define NAND_IOCONFIG_WD_SHIFT            (2)       /* Bits 2-3: WE_n default */
#define NAND_IOCONFIG_WD_MASK             (3 << NAND_IOCONFIG_WD_SHIFT)
#define NAND_IOCONFIG_RD_SHIFT            (0)       /* Bits 0-1: RE_n default */
#define NAND_IOCONFIG_RD_MASK             (3 << NAND_IOCONFIG_RD_SHIFT)

/* NandTiming1 register description (NandTiming1, address 0x17000814) */

#define NAND_TIMING1_TSRD_SHIFT           (20)      /* Bits 20-21: Single data input delay */
#define NAND_TIMING1_TSRD_MASK            (3 << NAND_TIMING1_TSRD_SHIFT
#define NAND_TIMING1_TALS_SHIFT           (16)      /* Bits 16-18: Address setup time */
#define NAND_TIMING1_TALS_MASK            (7 << NAND_TIMING1_TALS_SHIFT
#define NAND_TIMING1_TALH_SHIFT           (12)      /* Bits 12-14: Address hold time */
#define NAND_TIMING1_TALH_MASK            (7 << NAND_TIMING1_TALH_SHIFT
#define NAND_TIMING1_TCLS_SHIFT           (4)       /* Bits 4-6: Command setup time */
#define NAND_TIMING1_TCLS_MASK            (7 << NAND_TIMING1_TCLS_SHIFT
#define NAND_TIMING1_TCLH_SHIFT           (0)       /* Bits 0-2: Command hold time */
#define NAND_TIMING1_TCLH_MASK            (7 << NAND_TIMING1_TCLH_SHIFT

/* NandTiming2 register description (NandTiming2, address 0x17000818) */

#define NAND_TIMING2_TDRD_SHIFT           (28)      /* Bits 28-30: Data input delay */
#define NAND_TIMING2_TDRD_MASK            (7 << NAND_TIMING2_TDRD_SHIFT)
#define NAND_TIMING2_TEBIDEL_SHIFT        (24)      /* Bits 24-26: EBI delay time */
#define NAND_TIMING2_TEBIDEL_MASK         (7 << NAND_TIMING2_TEBIDEL_SHIFT)
#define NAND_TIMING2_TCH_SHIFT            (22)      /* Bits 20-22: Chip select hold time */
#define NAND_TIMING2_TCH_MASK             (7 << NAND_TIMING2_TCH_SHIFT)
#define NAND_TIMING2_TCS_SHIFT            (16)      /* Bits 16-18: Chip select setup time */
#define NAND_TIMING2_TCS_MASK             (7 << NAND_TIMING2_TCS_SHIFT)
#define NAND_TIMING2_TREH_SHIFT           (12)      /* Bits 12-14: Read enable high hold */
#define NAND_TIMING2_TREH_MASK            (7 << NAND_TIMING2_TREH_SHIFT)
#define NAND_TIMING2_TRP_SHIFT            (8)       /* Bits 8-10: Read enable pulse width */
#define NAND_TIMING2_TRP_MASK             (7 << NAND_TIMING2_TRP_SHIFT)
#define NAND_TIMING2_TWH_SHIFT            (4)       /* Bits 4-6: Write enable high hold */
#define NAND_TIMING2_TWH_MASK             (7 << NAND_TIMING2_TWH_SHIFT)
#define NAND_TIMING2_TWP_SHIFT            (0)       /* Bits 0-2: Write enable pulse width */
#define NAND_TIMING2_TWP_MASK             (7 << NAND_TIMING2_TWP_SHIFT)

/* NandSetCmd register description (NandSetCmd, address 0x17000820) */

#define NAND_SETCMD_CV_SHIFT              (0)       /* Bits 0-15: Command value */
#define NAND_SETCMD_CV_MASK               (0xffff << NAND_SETCMD_CV_SHIFT)

/* NandSetAddr register description (NandSetAddr, address 0x17000824) */

#define NAND_SETADDR_AV_SHIFT             (0)       /* Bits 0-15: Address value */
#define NAND_SETADDR_AV_MASK              (0xffff << NAND_SETADDR_AV_SHIFT)

/* NandWriteData register description (NandWriteData, address 0x17000828) */

#define NAND_WRITEDATA_WV_SHIFT           (0)       /* Bits 0-15: Write value */
#define NAND_WRITEDATA_WV_MASK            (0xffff << NAND_SETADDR_AV_SHIFT)

/* NandSetCE register description (NandSetCE, address 0x1700082c) */

#define NAND_SETCE_WP                     (1 << 4)  /* Bit 4:  WP_n pin value */
#define NAND_SETCE_CEV_MASK               (0x0f)
#define NAND_SETCE_CE1N                   (1 << 3)  /* Bit 3:  Chip select CE1_n value */
#define NAND_SETCE_CE2N                   (1 << 2)  /* Bit 2:  Chip select CE2_n value */
#define NAND_SETCE_CE3N                   (1 << 1)  /* Bit 1:  Chip select CE3_n value */
#define NAND_SETCE_CE4N                   (1 << 0)  /* Bit 0:  Chip select CE4_n value */

/* NandReadData register description (NandReadData, address 0x17000830) */

#define NAND_READDATA_RV_SHIFT            (0)       /* Bits 0-15: Read value */
#define NAND_READDATA_RV_MASK             (0xffff << NAND_READDATA_RV_SHIFT)

/* NandCheckSTS register description (NandCheckSTS, address 0x17000834) */

#define NAND_CHECKSTS_R3R                 (1 << 8)  /* Bit 8:  mNAND_RYBN3 rising edge */
#define NAND_CHECKSTS_R2R                 (1 << 7)  /* Bit 7:  mNAND_RYBN2 rising edge */
#define NAND_CHECKSTS_R1R                 (1 << 6)  /* Bit 6:  mNAND_RYBN1 rising edge */
#define NAND_CHECKSTS_R0R                 (1 << 5)  /* Bit 5:  mNAND_RYBN0 rising edge */
#define NAND_CHECKSTS_R3                  (1 << 4)  /* Bit 4:  mNAND_RYBN3 value */
#define NAND_CHECKSTS_R2                  (1 << 3)  /* Bit 3:  mNAND_RYBN2 value */
#define NAND_CHECKSTS_R1                  (1 << 2)  /* Bit 2:  mNAND_RYBN1 value */
#define NAND_CHECKSTS_R0                  (1 << 1)  /* Bit 1:  mNAND_RYBN0 value */
#define NAND_CHECKSTS_VB                  (1 << 0)  /* Bit 0:  APB busy */

/* NandControlFlow register description (NandControlFlow, address 0x17000838) */

#define NAND_CONTROLFLOW_W1               (1 << 5)  /* Bit 5:  Starts write SRAM1 to NAND */
#define NAND_CONTROLFLOW_W0               (1 << 4)  /* Bit 4:  Starts write SRAM0 to NAND */
#define NAND_CONTROLFLOW_R1               (1 << 1)  /* Bit 1:  Starts read from NAND to SRAM1 */
#define NAND_CONTROLFLOW_R0               (1 << 0)  /* Bit 0:  Starts read from NAND to SRAM0 */

/* NandGPIO1 register description (NandGPIO1, address 0x17000840) */

#define NAND_GPIO1_GPIOCONF               (1 << 26) /* Bit 26: GPIO mode */
#define NAND_GPIO1_WPN                    (1 << 25) /* Bit 25: Value on WP_n */
#define NAND_GPIO1_CLE                    (1 << 24) /* Bit 24: Value on CLE */
#define NAND_GPIO1_ALE                    (1 << 23) /* Bit 23: Value on ALE */
#define NAND_GPIO1_REN                    (1 << 22) /* Bit 22: Value on RE_n */
#define NAND_GPIO1_WEN                    (1 << 21) /* Bit 21: Value on WE_n */
#define NAND_GPIO1_CE4N                   (1 << 20) /* Bit 20: Value on NAND_NCS_3 */
#define NAND_GPIO1_CE3N                   (1 << 19) /* Bit 19: Value on NAND_NCS_2 */
#define NAND_GPIO1_CE2N                   (1 << 18) /* Bit 18: Value on NAND_NCS_1 */
#define NAND_GPIO1_CE1N                   (1 << 17) /* Bit 17: Value on NAND_NCS_0 */
#define NAND_GPIO1_IODRIVE                (1 << 16) /* Bit 16: Program value on NAND IO drive */
#define NAND_GPIO1_IODATA_SHIFT           (0)       /* Bit 0-15: Program value on data to NAND IO */
#define NAND_GPIO1_IODATA_MASK            (0xffff << NAND_GPIO1_IODATA_SHIFT)

/* NandGPIO2 register description (NandGPIO2, address 0x17000844) */

#define NAND_GPIO2_RnB3                   (1 << 19) /* Bit 19: Read value from mNAND_RYBN3 */
#define NAND_GPIO2_RnB2                   (1 << 18) /* Bit 18: Read value from mNAND_RYBN2 */
#define NAND_GPIO2_RnB1                   (1 << 17) /* Bit 17: Read value from mNAND_RYBN1 */
#define NAND_GPIO2_RnB0                   (1 << 16) /* Bit 16: Read value from mNAND_RYBN0 */
#define NAND_GPIO2_READDATA_SHIFT         (0)       /* Bit 0-15: Read data from NAND IO */
#define NAND_GPIO2_READDATA_MASK          (0xffff << NAND_GPIO2_READDATA_SHIFT)

/* NandIRQStatus2 register description (NandIRQStatus2, address 0x1700 0848) */

#define NAND_IRQSTATUS2_PGWHILEAPB        (1 << 4)  /* Bit 4:  Page access while APB access */
#define NAND_IRQSTATUS2_APBWHILEPG        (1 << 3)  /* Bit 3:  APB access while page access */
#define NAND_IRQSTATUS2_FLASHBUSY         (1 << 2)  /* Bit 2:  Flash access while busy */
#define NAND_IRQSTATUS2_RAM1BUSY          (1 << 1)  /* Bit 1:  RAM1 access while busy */
#define NAND_IRQSTATUS2_RAM0BUSY          (1 << 0)  /* Bit 0:  RAM0 access while busy */

/* NandIRQMask2 register description (NandIRQMask2, address 0x1700 084C) */

#define NAND_IRQMASK2_PGWHILEAPB          (1 << 4)  /* Bit 4:  Page access while APB access */
#define NAND_IRQMASK2_APBWHILEPG          (1 << 3)  /* Bit 3:  APB access while page access */
#define NAND_IRQMASK2_FLASHBUSY           (1 << 2)  /* Bit 2:  Flash access while busy */
#define NAND_IRQMASK2_RAM1BUSY            (1 << 1)  /* Bit 1:  RAM1 access while busy */
#define NAND_IRQMASK2_RAM0BUSY            (1 << 0)  /* Bit 0:  RAM0 access while busy */

/* NandIRQStatusRaw2 register description (NandIRQStatusRaw2, address 0x1700 0850) */

#define NAND_IRQSTATUSRAW2_PGWHILEAPB     (1 << 4)  /* Bit 4:  Page access while APB access */
#define NAND_IRQSTATUSRAW2_APBWHILEPG     (1 << 3)  /* Bit 3:  APB access while page access */
#define NAND_IRQSTATUSRAW2_FLASHBUSY      (1 << 2)  /* Bit 2:  Flash access while busy */
#define NAND_IRQSTATUSRAW2_RAM1BUSY       (1 << 1)  /* Bit 1:  RAM1 access while busy */
#define NAND_IRQSTATUSRAW2_RAM0BUSY       (1 << 0)  /* Bit 0:  RAM0 access while busy */

/* First-fourth words of 128-bit AES key (32-bit values, no bit fields -- LPC3154 only) */
/* First-fourth words of 128-bit initial AES value (32-bit values, no bit fields -- LPC3154 only) */

/* Register to display AES state (LPC3154 only) */

#define NAND_AESSTATE_SHIFT               (0)       /* Bits 0-1:  AES state */
#define NAND_AESSTATE_MASK                (3 << NAND_AESSTATE_SHIFT)
#  define NAND_AESSTATE_BUSY              (0 << NAND_AESSTATE_SHIFT) /* AES state machine busy */
#  define NAND_AESSTATE_KEYSETUP          (1 << NAND_AESSTATE_SHIFT) /* AES key setup needed */
#  define NAND_AESSTATE_IDLE              (3 << NAND_AESSTATE_SHIFT) /* AES module is IDLE */

/* NandECCErrStatus register description (NandECCErrStatus, address 0x1700 0878) */

#define NAND_ECCERRSTATUS_NERR1_SHIFT     (4)       /* Bits 4-7: Number of errors in RAM1 */
#define NAND_ECCERRSTATUS_NERR1_MASK      (0x0f << NAND_ECCERRSTATUS_NERR1_SHIFT)
#define NAND_ECCERRSTATUS_NERR0_SHIFT     (0)       /* Bits 0-3: Number of errors in RAM0 */
#define NAND_ECCERRSTATUS_NERR0_MASK      (0x0f << NAND_ECCERRSTATUS_NERR0_SHIFT)

/* Enable AES engine from AHB */

#define NAND_AESFROMAHB_MODE              (1 << 7)  /* Bit 7:  Set AES from AHB mode */
#define NAND_AESFROMAHB_DECRYPTRAM1       (1 << 1)  /* Bit 1:  Decrypt RAM1 */
#define NAND_AESFROMAHB_DECRYPTRAM0       (1 << 0)  /* Bit 0:  Decrypt RAM0 */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC31XX_LPC31_NAND_H */
