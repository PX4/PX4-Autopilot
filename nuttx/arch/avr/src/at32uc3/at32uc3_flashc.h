/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_flashc.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_FLASHC_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_FLASHC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define AVR32_FLASHC_FCR_OFFSET     0x00 /* Flash Control Register */
#define AVR32_FLASHC_FCMD_OFFSET    0x04 /* Flash Command Register */
#define AVR32_FLASHC_FSR_OFFSET     0x08 /* Flash Status Register */
#define AVR32_FLASHC_FGPFRHI_OFFSET 0x0c /* Flash General Purpose Fuse Register Hi */
#define AVR32_FLASHC_FGPFRLO_OFFSET 0x10 /* Flash General Purpose Fuse Register Lo */

/* Register Addresses ***************************************************************/

#define AVR32_FLASHC_FCR            (AVR32_HFLASHC_BASE+AVR32_FLASHC_FCR_OFFSET)
#define AVR32_FLASHC_FCMD           (AVR32_HFLASHC_BASE+AVR32_FLASHC_FCMD_OFFSET)
#define AVR32_FLASHC_FSR            (AVR32_HFLASHC_BASE+AVR32_FLASHC_FSR_OFFSET)
#define AVR32_FLASHC_FGPFRHI        (AVR32_HFLASHC_BASE+AVR32_FLASHC_FGPFRHI_OFFSET)
#define AVR32_FLASHC_FGPFRLO        (AVR32_HFLASHC_BASE+AVR32_FLASHC_FGPFRLO_OFFSET)

/* Register Bit-field Definitions ***************************************************/

/* Flash Control Register */

#define FLASHC_FCR_FRDY             (1 << 0)  /* Bit 0:  Flash Ready Interrupt Enable */
#define FLASHC_FCR_LOCKE            (1 << 2)  /* Bit 2:  Lock Error Interrupt Enable */
#define FLASHC_FCR_PROGE            (1 << 3)  /* Bit 3:  Programming Error Interrupt Enable */
#define FLASHC_FCR_FWS              (1 << 6)  /* Bit 6:  Flash Wait State */

/* Flash Command Register */

#define FLASHC_FCMD_CMD_SHIFT       (0)       /* Bits 0-5: Command */
#define FLASHC_FCMD_CMD_MASK        (0x3f << FLASHC_FCMD_CMD_SHIFT)
#define FLASHC_FCMD_PAGEN_SHIFT     (8)       /* Bits 8-23: Page number */
#define FLASHC_FCMD_PAGEN_MASK      (0xffff << FLASHC_FCMD_PAGEN_SHIFT)
#define FLASHC_FCMD_KEY_SHIFT       (14)      /* Bits 24-31: Write protection key */
#define FLASHC_FCMD_KEY_MASK        (0xff << FLASHC_FCMD_KEY_SHIFT)

/* Flash Status Register */

#define FLASHC_FSR_FRDY             (1 << 0)  /* Bit 0:  Flash Ready Status */
#define FLASHC_FSR_LOCKE            (1 << 2)  /* Bit 2:  Lock Error Status */
#define FLASHC_FSR_PROGE            (1 << 3)  /* Bit 3:  Programming Error Status */
#define FLASHC_FSR_SECURITY         (1 << 4)  /* Bit 4:  Security Bit Status */
#define FLASHC_FSR_QPRR             (1 << 5)  /* Bit 5:  Quick Page Read Result */
#define FLASHC_FSR_FSZ_SHIFT        (13)      /* Bits 13-15: Flash Size */
#define FLASHC_FSR_FSZ_MASK         (7 << FLASHC_FSR_FSZ_SHIFT)
#  define FLASHC_FSR_FSZ_23KB       (0 << FLASHC_FSR_FSZ_SHIFT) /* 32 Kbytes */
#  define FLASHC_FSR_FSZ_64KB       (1 << FLASHC_FSR_FSZ_SHIFT) /* 64 Kbytes */
#  define FLASHC_FSR_FSZ_128KB      (2 << FLASHC_FSR_FSZ_SHIFT) /* 128 Kbytes */
#  define FLASHC_FSR_FSZ_256KB      (3 << FLASHC_FSR_FSZ_SHIFT) /* 256 Kbytes */
#  define FLASHC_FSR_FSZ_384KB      (4 << FLASHC_FSR_FSZ_SHIFT) /* 384 Kbytes */
#  define FLASHC_FSR_FSZ_512KGB     (5 << FLASHC_FSR_FSZ_SHIFT) /* 512 Kbytes */
#  define FLASHC_FSR_FSZ_768KB      (6 << FLASHC_FSR_FSZ_SHIFT) /* 768 Kbytes */
#  define FLASHC_FSR_FSZ_1MB        (7 << FLASHC_FSR_FSZ_SHIFT) /* 1024 Kbytes */
#define FLASHC_FSR_LOCK(n)          (1 << ((n)+16)
#define FLASHC_FSR_LOCK0            (1 << 16) /* Bit 16: Lock Region 0 Lock Status */
#define FLASHC_FSR_LOCK1            (1 << 17) /* Bit 17: Lock Region 1 Lock Status */
#define FLASHC_FSR_LOCK2            (1 << 18) /* Bit 18: Lock Region 2 Lock Status */
#define FLASHC_FSR_LOCK3            (1 << 19) /* Bit 19: Lock Region 3 Lock Status */
#define FLASHC_FSR_LOCK4            (1 << 20) /* Bit 20: Lock Region 4 Lock Status */
#define FLASHC_FSR_LOCK5            (1 << 21) /* Bit 21: Lock Region 5 Lock Status */
#define FLASHC_FSR_LOCK6            (1 << 22) /* Bit 22: Lock Region 6 Lock Status */
#define FLASHC_FSR_LOCK7            (1 << 23) /* Bit 23: Lock Region 7 Lock Status */
#define FLASHC_FSR_LOCK8            (1 << 24) /* Bit 24: Lock Region 8 Lock Status */
#define FLASHC_FSR_LOCK9            (1 << 25) /* Bit 25: Lock Region 9 Lock Status */
#define FLASHC_FSR_LOCK10           (1 << 26) /* Bit 26: Lock Region 10 Lock Status */
#define FLASHC_FSR_LOCK11           (1 << 27) /* Bit 27: Lock Region 11 Lock Status */
#define FLASHC_FSR_LOCK12           (1 << 28) /* Bit 28: Lock Region 12 Lock Status */
#define FLASHC_FSR_LOCK13           (1 << 29) /* Bit 29: Lock Region 13 Lock Status */
#define FLASHC_FSR_LOCK14           (1 << 30) /* Bit 30: Lock Region 14 Lock Status */
#define FLASHC_FSR_LOCK15           (1 << 31) /* Bit 31: Lock Region 15 Lock Status */

/* Flash General Purpose Fuse Register Hi */

#define FLASHC_FGPFRHI(n)           (1 << ((n)-32))
#define FLASHC_FGPFRHI32            (1 << 0)  /* Bit 0:  General Purpose Fuse 32 */
#define FLASHC_FGPFRHI33            (1 << 1)  /* Bit 1:  General Purpose Fuse 33 */
#define FLASHC_FGPFRHI34            (1 << 2)  /* Bit 2:  General Purpose Fuse 34 */
#define FLASHC_FGPFRHI35            (1 << 3)  /* Bit 3:  General Purpose Fuse 35 */
#define FLASHC_FGPFRHI36            (1 << 4)  /* Bit 4:  General Purpose Fuse 36 */
#define FLASHC_FGPFRHI37            (1 << 5)  /* Bit 5:  General Purpose Fuse 37 */
#define FLASHC_FGPFRHI38            (1 << 6)  /* Bit 6:  General Purpose Fuse 38 */
#define FLASHC_FGPFRHI39            (1 << 7)  /* Bit 7:  General Purpose Fuse 39 */
#define FLASHC_FGPFRHI40            (1 << 8)  /* Bit 8:  General Purpose Fuse 40 */
#define FLASHC_FGPFRHI41            (1 << 9)  /* Bit 9:  General Purpose Fuse 41 */
#define FLASHC_FGPFRHI42            (1 << 10) /* Bit 10: General Purpose Fuse 42 */
#define FLASHC_FGPFRHI43            (1 << 11) /* Bit 11: General Purpose Fuse 43 */
#define FLASHC_FGPFRHI44            (1 << 12) /* Bit 12: General Purpose Fuse 44 */
#define FLASHC_FGPFRHI45            (1 << 13) /* Bit 13: General Purpose Fuse 45 */
#define FLASHC_FGPFRHI46            (1 << 14) /* Bit 14: General Purpose Fuse 46 */
#define FLASHC_FGPFRHI47            (1 << 15) /* Bit 15: General Purpose Fuse 47 */
#define FLASHC_FGPFRHI48            (1 << 16) /* Bit 16: General Purpose Fuse 48 */
#define FLASHC_FGPFRHI49            (1 << 17) /* Bit 17: General Purpose Fuse 49 */
#define FLASHC_FGPFRHI50            (1 << 18) /* Bit 18: General Purpose Fuse 50 */
#define FLASHC_FGPFRHI51            (1 << 19) /* Bit 19: General Purpose Fuse 51 */
#define FLASHC_FGPFRHI52            (1 << 20) /* Bit 20: General Purpose Fuse 52 */
#define FLASHC_FGPFRHI53            (1 << 21) /* Bit 21: General Purpose Fuse 53 */
#define FLASHC_FGPFRHI54            (1 << 22) /* Bit 22: General Purpose Fuse 54 */
#define FLASHC_FGPFRHI55            (1 << 23) /* Bit 23: General Purpose Fuse 55 */
#define FLASHC_FGPFRHI56            (1 << 24) /* Bit 24: General Purpose Fuse 56 */
#define FLASHC_FGPFRHI57            (1 << 25) /* Bit 25: General Purpose Fuse 57 */
#define FLASHC_FGPFRHI58            (1 << 26) /* Bit 26: General Purpose Fuse 58 */
#define FLASHC_FGPFRHI59            (1 << 27) /* Bit 27: General Purpose Fuse 59 */
#define FLASHC_FGPFRHI60            (1 << 28) /* Bit 28: General Purpose Fuse 60 */
#define FLASHC_FGPFRHI61            (1 << 29) /* Bit 29: General Purpose Fuse 61 */
#define FLASHC_FGPFRHI62            (1 << 30) /* Bit 30: General Purpose Fuse 62 */
#define FLASHC_FGPFRHI63            (1 << 31) /* Bit 31: General Purpose Fuse 63 */

/* Flash General Purpose Fuse Register Lo */

#define FLASHC_FGPFRLO(n)           (1 << (n))
#define FLASHC_FGPFRLO00            (1 << 0)  /* Bit 0:  General Purpose Fuse 00 */
#define FLASHC_FGPFRLO01            (1 << 1)  /* Bit 1:  General Purpose Fuse 01 */
#define FLASHC_FGPFRLO02            (1 << 2)  /* Bit 2:  General Purpose Fuse 02 */
#define FLASHC_FGPFRLO03            (1 << 3)  /* Bit 3:  General Purpose Fuse 03 */
#define FLASHC_FGPFRLO04            (1 << 4)  /* Bit 4:  General Purpose Fuse 04 */
#define FLASHC_FGPFRLO05            (1 << 5)  /* Bit 5:  General Purpose Fuse 05 */
#define FLASHC_FGPFRLO06            (1 << 6)  /* Bit 6:  General Purpose Fuse 06 */
#define FLASHC_FGPFRLO07            (1 << 7)  /* Bit 7:  General Purpose Fuse 07 */
#define FLASHC_FGPFRLO08            (1 << 8)  /* Bit 8:  General Purpose Fuse 08 */
#define FLASHC_FGPFRLO09            (1 << 9)  /* Bit 9:  General Purpose Fuse 09 */
#define FLASHC_FGPFRLO10            (1 << 10) /* Bit 10: General Purpose Fuse 10 */
#define FLASHC_FGPFRLO11            (1 << 11) /* Bit 11: General Purpose Fuse 11 */
#define FLASHC_FGPFRLO12            (1 << 12) /* Bit 12: General Purpose Fuse 12 */
#define FLASHC_FGPFRLO13            (1 << 13) /* Bit 13: General Purpose Fuse 13 */
#define FLASHC_FGPFRLO14            (1 << 14) /* Bit 14: General Purpose Fuse 14 */
#define FLASHC_FGPFRLO15            (1 << 15) /* Bit 15: General Purpose Fuse 15 */
#define FLASHC_FGPFRLO16            (1 << 16) /* Bit 16: General Purpose Fuse 16 */
#define FLASHC_FGPFRLO17            (1 << 17) /* Bit 17: General Purpose Fuse 17 */
#define FLASHC_FGPFRLO18            (1 << 18) /* Bit 18: General Purpose Fuse 18 */
#define FLASHC_FGPFRLO19            (1 << 19) /* Bit 19: General Purpose Fuse 19 */
#define FLASHC_FGPFRLO20            (1 << 20) /* Bit 20: General Purpose Fuse 20 */
#define FLASHC_FGPFRLO21            (1 << 21) /* Bit 21: General Purpose Fuse 21 */
#define FLASHC_FGPFRLO22            (1 << 22) /* Bit 22: General Purpose Fuse 22 */
#define FLASHC_FGPFRLO23            (1 << 23) /* Bit 23: General Purpose Fuse 23 */
#define FLASHC_FGPFRLO24            (1 << 24) /* Bit 24: General Purpose Fuse 24 */
#define FLASHC_FGPFRLO25            (1 << 25) /* Bit 25: General Purpose Fuse 25 */
#define FLASHC_FGPFRLO26            (1 << 26) /* Bit 26: General Purpose Fuse 26 */
#define FLASHC_FGPFRLO27            (1 << 27) /* Bit 27: General Purpose Fuse 27 */
#define FLASHC_FGPFRLO28            (1 << 28) /* Bit 28: General Purpose Fuse 28 */
#define FLASHC_FGPFRLO29            (1 << 29) /* Bit 29: General Purpose Fuse 29 */
#define FLASHC_FGPFRLO30            (1 << 30) /* Bit 30: General Purpose Fuse 30 */
#define FLASHC_FGPFRLO31            (1 << 31) /* Bit 31: General Purpose Fuse 31 */

/* Flash Command Set ****************************************************************/

#define FLASH_CMD_NOP                0 /* No operation */
#define FLASH_CMD_WP                 1 /* Write Page */
#define FLASH_CMD_EP                 2 /* Erase Page */
#define FLASH_CMD_CPB                3 /* Clear Page Buffer */
#define FLASH_CMD_LP                 4 /* Lock region containing given page */
#define FLASH_CMD_UP                 5 /* Unlock region containing given page */
#define FLASH_CMD_EA                 6 /* Erase All */
#define FLASH_CMD_WGPB               7 /* Write General-Purpose Fuse Bit */
#define FLASH_CMD_EGPB               8 /* Erase General-Purpose Fuse Bit */
#define FLASH_CMD_SSB                9 /* Set Security Bit */
#define FLASH_CMD_PGPFB             10 /* Program GP Fuse Byte */
#define FLASH_CMD_EAGP              11 /* Erase All GPFuses */
#define FLASH_CMD_QPR               12 /* Quick Page Read */
#define FLASH_CMD_WUP               13 /* Write User Page */
#define FLASH_CMD_EUP               14 /* Erase User Page */
#define FLASH_CMD_QPRUP             15 /* Quick Page Read User Page */

/* Other constants ******************************************************************/

/* Maximum CPU frequency for 0 and 1 FLASH wait states */

#define AVR32_FLASHC_FWS0_MAXFREQ   33000000
#define AVR32_FLASHC_FWS1_MAXFREQ   66000000

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_FLASHC_H */

