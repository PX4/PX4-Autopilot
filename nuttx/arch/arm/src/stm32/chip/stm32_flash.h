/************************************************************************************
 * arch/arm/src/stm32/chip/stm32_flash.h
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
 ************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#if defined(CONFIG_STM32_LOWDENSITY)
#  define STM32_FLASH_NPAGES        32
#  define STM32_FLASH_PAGESIZE      1024
#elif  defined(CONFIG_STM32_MEDIUMDENSITY)
#  define STM32_FLASH_NPAGES        128
#  define STM32_FLASH_PAGESIZE      1024
#elif  defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define STM32_FLASH_NPAGES        128
#  define STM32_FLASH_PAGESIZE      2048
#elif defined(CONFIG_STM32_HIGHDENSITY)
#  define STM32_FLASH_NPAGES        256
#  define STM32_FLASH_PAGESIZE      2048
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_FLASH_NPAGES        8
#  define STM32_FLASH_PAGESIZE      (128*1024)
#endif

#define STM32_FLASH_SIZE            (STM32_FLASH_NPAGES * STM32_FLASH_PAGESIZE)

/* Register Offsets *****************************************************************/

#define STM32_FLASH_ACR_OFFSET     0x0000
#define STM32_FLASH_KEYR_OFFSET    0x0004
#define STM32_FLASH_OPTKEYR_OFFSET 0x0008
#define STM32_FLASH_SR_OFFSET      0x000c
#define STM32_FLASH_CR_OFFSET      0x0010

#if defined(CONFIG_STM32_STM32F10XX)
#  define STM32_FLASH_AR_OFFSET    0x0014
#  define STM32_FLASH_OBR_OFFSET   0x001c
#  define STM32_FLASH_WRPR_OFFSET  0x0020
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define STM32_FLASH_OPTCR_OFFSET 0x0014
#endif

/* Register Addresses ***************************************************************/

#define STM32_FLASH_ACR            (STM32_FLASHIF_BASE+STM32_FLASH_ACR_OFFSET)
#define STM32_FLASH_KEYR           (STM32_FLASHIF_BASE+STM32_FLASH_KEYR_OFFSET)
#define STM32_FLASH_OPTKEYR        (STM32_FLASHIF_BASE+STM32_FLASH_OPTKEYR_OFFSET)
#define STM32_FLASH_SR             (STM32_FLASHIF_BASE+STM32_FLASH_SR_OFFSET)
#define STM32_FLASH_CR             (STM32_FLASHIF_BASE+STM32_FLASH_CR_OFFSET)

#if defined(CONFIG_STM32_STM32F10XX)
#  define STM32_FLASH_AR           (STM32_FLASHIF_BASE+STM32_FLASH_AR_OFFSET)
#  define STM32_FLASH_OBR          (STM32_FLASHIF_BASE+STM32_FLASH_OBR_OFFSET)
#  define STM32_FLASH_WRPR         (STM32_FLASHIF_BASE+STM32_FLASH_WRPR_OFFSET)
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
# define STM32_FLASH_OPTCR         (STM32_FLASHIF_BASE+STM32_FLASH_OPTCR_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/
/* Flash Access Control Register (ACR) */

#define FLASH_ACR_LATENCY_SHIFT     (0)
#define FLASH_ACR_LATENCY_MASK      (7 << FLASH_ACR_LATENCY_SHIFT)
#  define FLASH_ACR_LATENCY(n)      ((n) << FLASH_ACR_LATENCY_SHIFT)  /* n wait states */
#  define FLASH_ACR_LATENCY_0       (0 << FLASH_ACR_LATENCY_SHIFT)    /* 000: Zero wait states */
#  define FLASH_ACR_LATENCY_1       (1 << FLASH_ACR_LATENCY_SHIFT)    /* 001: One wait state */
#  define FLASH_ACR_LATENCY_2       (2 << FLASH_ACR_LATENCY_SHIFT)    /* 010: Two wait states */
#  define FLASH_ACR_LATENCY_3       (3 << FLASH_ACR_LATENCY_SHIFT)    /* 011: Three wait states */
#  define FLASH_ACR_LATENCY_4       (4 << FLASH_ACR_LATENCY_SHIFT)    /* 100: Four wait states */
#  define FLASH_ACR_LATENCY_5       (5 << FLASH_ACR_LATENCY_SHIFT)    /* 101: Five wait states */
#  define FLASH_ACR_LATENCY_6       (6 << FLASH_ACR_LATENCY_SHIFT)    /* 110: Six wait states */
#  define FLASH_ACR_LATENCY_7       (7 << FLASH_ACR_LATENCY_SHIFT)    /* 111: Seven wait states */

#if defined(CONFIG_STM32_STM32F10XX)
#  define FLASH_ACR_HLFCYA          (1 << 3)                /* FLASH half cycle access */
#  define FLASH_ACR_PRTFBE          (1 << 4)                /* FLASH prefetch enable */
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_ACR_PRFTEN          (1 << 8)                /* FLASH prefetch enable */
#  define FLASH_ACR_ICEN            (1 << 9)                /* Bit 9: Instruction cache enable */
#  define FLASH_ACR_DCEN            (1 << 10)               /* Bit 10: Data cache enable */
#  define FLASH_ACR_ICRST           (1 << 11)               /* Bit 11: Instruction cache reset */
#  define FLASH_ACR_DCRST           (1 << 12)               /* Bit 12: Data cache reset */
#endif

/* Flash Status Register (SR) */

#if defined(CONFIG_STM32_STM32F10XX)
#  define FLASH_SR_BSY              (1 << 0)                /* Busy */
#  define FLASH_SR_PGERR            (1 << 2)                /* Programming Error */
#  define FLASH_SR_WRPRT_ERR        (1 << 4)                /* Write Protection Error */
#  define FLASH_SR_EOP              (1 << 5)                /* End of Operation */
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_SR_EOP              (1 << 0)                /* Bit 0: End of operation */
#  define FLASH_SR_OPERR            (1 << 1)                /* Bit 1: Operation error */
#  define FLASH_SR_WRPERR           (1 << 4)                /* Bit 4: Write protection error */
#  define FLASH_SR_PGAERR           (1 << 5)                /* Bit 5: Programming alignment error */
#  define FLASH_SR_PGPERR           (1 << 6)                /* Bit 6: Programming parallelism error */
#  define FLASH_SR_PGSERR           (1 << 7)                /* Bit 7: Programming sequence error */
#  define FLASH_SR_BSY              (1 << 16)               /* Bit 16: Busy */
#endif

/* Flash Control Register (CR) */

#if defined(CONFIG_STM32_STM32F10XX)
#  define FLASH_CR_PG               (1 << 0)                /* Program Page */
#  define FLASH_CR_PER              (1 << 1)                /* Page Erase */
#  define FLASH_CR_MER              (1 << 2)                /* Mass Erase */
#  define FLASH_CR_OPTPG            (1 << 4)                /* Option Byte Programming */
#  define FLASH_CR_OPTER            (1 << 5)                /* Option Byte Erase */
#  define FLASH_CR_STRT             (1 << 6)                /* Start Erase */
#  define FLASH_CR_LOCK             (1 << 7)                /* Page Locked or Lock Page */
#  define FLASH_CR_OPTWRE           (1 << 9)                /* Option Bytes Write Enable */
#  define FLASH_CR_ERRIE            (1 << 10)               /* Error Interrupt Enable */
#  define FLASH_CR_EOPIE            (1 << 12)               /* End of Program Interrupt Enable */
#elif defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_CR_PG               (1 << 0)                /* Bit 0: Programming */
#  define FLASH_CR_SER              (1 << 1)                /* Bit 1: Sector Erase */
#  define FLASH_CR_MER              (1 << 2)                /* Bit 2: Mass Erase */
#  define FLASH_CR_SNB_SHIFT        (3)                     /* Bits 3-6: Sector number */
#  define FLASH_CR_SNB_MASK         (15 << FLASH_CR_SNB_SHIFT)
#    define FLASH_CR_SNB(n)         ((n) << FLASH_CR_SNB_SHIFT) /* Sector n, n=0..11 */
#  define FLASH_CR_PSIZE_SHIFT      (8)                     /* Bits 8-9: Program size */
#  define FLASH_CR_PSIZE_MASK       (3 << FLASH_CR_PSIZE_SHIFT)
#    define FLASH_CR_PSIZE_X8       (0 << FLASH_CR_PSIZE_SHIFT) /* 00 program x8 */
#    define FLASH_CR_PSIZE_X16      (1 << FLASH_CR_PSIZE_SHIFT) /* 01 program x16 */
#    define FLASH_CR_PSIZE_X32      (2 << FLASH_CR_PSIZE_SHIFT) /* 10 program x32 */
#    define FLASH_CR_PSIZE_X64      (3 << FLASH_CR_PSIZE_SHIFT) /* 11 program x64 */
#  define FLASH_CR_EOPIE            (1 << 24)               /* Bit 24: End of operation interrupt enable */
#  define FLASH_CR_ERRIE            (1 << 25)               /* Bit 25: Error interrupt enable */
#  define FLASH_CR_LOCK             (1 << 31)               /* Bit 31: Lock */
#endif

/* Flash Option Control Register (OPTCR) */

#if defined(CONFIG_STM32_STM32F20XX) || defined(CONFIG_STM32_STM32F40XX)
#  define FLASH_OPTCR_OPTLOCK       (1 << 0)               /* Bit 0: Option lock */
#  define FLASH_OPTCR_OPTSTRT       (1 << 1)               /* Bit 1: Option start */
#  define FLASH_OPTCR_BORLEV_SHIFT  (2)                    /* Bits 2-3: BOR reset Level */
#  define FLASH_OPTCR_BORLEV_MASK   (3 << FLASH_OPTCR_BORLEV_SHIFT)
#    define FLASH_OPTCR_VBOR3       (0 << FLASH_OPTCR_BORLEV_SHIFT) /* 00: BOR Level 3 */
#    define FLASH_OPTCR_VBOR2       (1 << FLASH_OPTCR_BORLEV_SHIFT) /* 01: BOR Level 2 */
#    define FLASH_OPTCR_VBOR1       (2 << FLASH_OPTCR_BORLEV_SHIFT) /* 10: BOR Level 1 */
#    define FLASH_OPTCR_VBOR0       (3 << FLASH_OPTCR_BORLEV_SHIFT) /* 11: BOR off */
#  define FLASH_OPTCR_USER_SHIFT    (5)                    /* Bits 5-7: User option bytes */
#  define FLASH_OPTCR_USER_MASK     (7 << FLASH_OPTCR_USER_SHIFT)
#    define FLASH_OPTCR_NRST_STDBY  (1 << 7)               /* Bit 7: nRST_STDBY */
#    define FLASH_OPTCR_NRST_STOP   (1 << 6)               /* Bit 6: nRST_STOP */
#    define FLASH_OPTCR_WDG_SW      (1 << 5)               /* Bit 5: WDG_SW */
#  define FLASH_OPTCR_RDP_SHIFT     (8)                    /* Bits 8-15: Read protect */
#  define FLASH_OPTCR_RDP_MASK      (0xff << FLASH_OPTCR_RDP_SHIFT)
#  define FLASH_OPTCR_NWRP_SHIFT    (16)                   /* Bits 16-27: Not write protect */
#  define FLASH_OPTCR_NWRP_MASK     (0xfff << FLASH_OPTCR_NWRP_SHIFT)
#endif

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_FLASH_H */

