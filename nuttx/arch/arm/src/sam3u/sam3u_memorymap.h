/************************************************************************************************
 * arch/arm/src/sam3u/sam3u_memorymap.h
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_SAM3U_SAM3U_MEMORYMAP_H
#define __ARCH_ARM_SRC_SAM3U_SAM3U_MEMORYMAP_H

/************************************************************************************************
 * Included Files
 ************************************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/

#define   SAM3U_CODE_BASE        0x00000000 /* 0x00000000-0x1fffffff: Code space */
#  define SAM3U_BOOTMEMORY_BASE  0x00000000 /* 0x00000000-0x0007ffff:   Boot Memory */
#  define SAM3U_INTFLASH0_BASE   0x00080000 /* 0x00080000-0x000fffff:   Internal FLASH 0 */
#  define SAM3U_INTFLASH1_BASE   0x00100000 /* 0x00100000-0x0017ffff:   Internal FLASH 1 */
#  define SAM3U_INTROM_BASE      0x00180000 /* 0x00180000-0x001fffff:   Internal ROM */
                                            /* 0x00200000-0x1fffffff:   Reserved */
#define   SAM3U_INTSRAM_BASE     0x20000000 /* 0x20000000-0x3fffffff: Internal SRAM */
#  define SAM3U_INTSRAM0_BASE    0x20000000 /* 0x20000000-0x2007ffff:   SRAM0 (see chip.h) */
#  define SAM3U_INTSRAM1_BASE    0x20080000 /* 0x20080000-0x200fffff:   SRAM1 (see chip.h) */
#  define SAM3U_NFCSRAM_BASE     0x20100000 /* 0x20100000-0x207fffff:   NAND FLASH controller (SRAM) */
#  define SAM3U_UDPHPSDMS_BASE   0x20180000 /* 0x20180000-0x201fffff:   USB Device High Speed (DMA) */
                                            /* 0x20200000-0x2fffffff:   Undefined */
#  define SAM3U_BBSRAM_BASE      0x22000000 /* 0x22000000-0x23ffffff:   32Mb bit-band alias */
                                            /* 0x24000000-0x3fffffff:   Undefined */
#define SAM3U_PERIPHERALS_BASE   0x40000000 /* 0x40000000-0x5fffffff: Peripherals */
#  define SAM3U_MCI_BASE         0x40000000 /* 0x40000000-0x400003ff:   High Speed Multimedia Card Interface */
#  define SAM3U_SSC_BASE         0x40004000 /* 0x40004000-0x40007fff:   Synchronous Serial Controller */
#  define SAM3U_SPI_BASE         0x40008000 /* 0x40008000-0x4000bfff:   Serial Peripheral Interface */
                                            /* 0x4000c000-0x4007ffff:   Reserved */
#  define SAM3U_TC_BASE          0x40080000 /* 0x40080000-0x40083fff:   Timer Counters */
#    define SAM3U_TCN_BASE(n)    (0x40080000+((n)<<6))
#    define SAM3U_TC0_BASE       0x40080000 /* 0x40080000-0x4008003f:     Timer Counter 0 */
#    define SAM3U_TC1_BASE       0x40080040 /* 0x40080040-0x4008007f:     Timer Counter 1 */
#    define SAM3U_TC2_BASE       0x40080080 /* 0x40080080-0x400800bf:     Timer Counter 2 */
#  define SAM3U_TWI_BASE         0x40084000 /* 0x40084000-0x4008ffff:   Two-Wire Interface */
#    define SAM3U_TWIN_BASE(n)   (0x40084000+((n)<<14))
#    define SAM3U_TWI0_BASE      0x40084000 /* 0x40084000-0x40087fff:     Two-Wire Interface 0 */
#    define SAM3U_TWI1_BASE      0x40088000 /* 0x40088000-0x4008bfff:     Two-Wire Interface 1 */
#  define SAM3U_PWM_BASE         0x4008c000 /* 0x4008c000-0x4008ffff:   Pulse Width Modulation Controller */
#  define SAM3U_USART_BASE       0x40090000 /* 0x40090000-0x4009ffff:   USART */
#    define SAM3U_USARTN_BASE(n) (0x40090000+((n)<<14))
#    define SAM3U_USART0_BASE    0x40090000 /* 0x40090000-0x40093fff:     USART0 */
#    define SAM3U_USART1_BASE    0x40094000 /* 0x40094000-0x40097fff:     USART1 */
#    define SAM3U_USART2_BASE    0x40098000 /* 0x40098000-0x4009bfff:     USART2 */
#    define SAM3U_USART3_BASE    0x4009c000 /* 0x4009c000-0x4009ffff:     USART3 */
                                            /* 0x400a0000-0x400a3fff:   Reserved */
#  define SAM3U_UDPHS_BASE       0x400a4000 /* 0x400a4000-0x400a7fff:   USB Device High Speed */
#  define SAM3U_ADC12B_BASE      0x400a8000 /* 0x400a8000-0x400abfff:   12-bit ADC Controller */
#  define SAM3U_ADC_BASE         0x400ac000 /* 0x400ac000-0x400affff:   10-bit ADC Controller */
#  define SAM3U_DMAC_BASE        0x400b0000 /* 0x400b0000-0x400b3fff:   DMA controller */
                                            /* 0x400b4000-0x400dffff:   Reserved */
#  define SAM3U_SYSCTRLR_BASE    0x400e0000 /* 0x400e0000-0x400e25ff:   System controller */
                                            /* 0x400e2600-0x400fffff:   Reserved */
                                            /* 0x40100000-0x41ffffff:   Reserved */
#  define SAM3U_BBPERIPH__BASE   0x42000000 /* 0x42000000-0x43ffffff:   32Mb bit-band alias */
                                            /* 0x44000000-0x5fffffff:   Reserved */
#define   SAM3U_EXTSRAM_BASE     0x60000000 /* 0x60000000-0x9fffffff: External SRAM */
#  define SAM3U_EXTCS_BASE       0x60000000 /* 0x60000000-0x63ffffff:   Chip selects */
#    define SAM3U_EXTCSN_BASE(n) (0x60000000*((n)<<24))
#    define SAM3U_EXTCS0_BASE    0x60000000 /* 0x60000000-0x60ffffff:     Chip select 0 */
#    define SAM3U_EXTCS1_BASE    0x61000000 /* 0x61000000-0x601fffff:     Chip select 1 */
#    define SAM3U_EXTCS2_BASE    0x62000000 /* 0x62000000-0x62ffffff:     Chip select 2 */
#    define SAM3U_EXTCS3_BASE    0x63000000 /* 0x63000000-0x63ffffff:     Chip select 3 */
                                            /* 0x64000000-0x67ffffff:   Reserved */
#  define SAM3U_NFC_BASE         0x68000000 /* 0x68000000-0x68ffffff:   NAND FLASH controller */
                                            /* 0x69000000-0x9fffffff:   Reserved */
                                            /* 0xa0000000-0xdfffffff:   Reserved */
#define   SAM3U_SYSTEM_BASE      0xe0000000 /* 0xe0000000-0xffffffff: System */

/* System Controller Register Blocks:  0x400e0000-0x4007ffff */

#define SAM3U_SMC_BASE           0x400e0000 /* 0x400e0000-0x400e01ff: Static Memory Controller */
#define SAM3U_MATRIX_BASE        0x400e0200 /* 0x400e0200-0x400e03ff: MATRIX */
#define SAM3U_PMC_BASE           0x400e0400 /* 0x400e0400-0x400e05ff: Power Management Controller */
#define SAM3U_UART_BASE          0x400e0600 /* 0x400e0600-0x400e073f: UART */
#define SAM3U_CHIPID_BASE        0x400e0740 /* 0x400e0740-0x400e07ff: CHIP ID */
#define SAM3U_EEFC_BASE          0x400e0800 /* 0x400e0800-0x400e0bff: Enhanced Embedded Flash Controllers*/
#  define SAM3U_EEFCN_BASE(n)    (0x400e0800+((n)<<9))
#  define SAM3U_EEFC0_BASE       0x400e0800 /* 0x400e0800-0x400e09ff:   Enhanced Embedded Flash Controller 0 */
#  define SAM3U_EEFC1_BASE       0x400e0a00 /* 0x400e0a00-0x400e0bff:   Enhanced Embedded Flash Controller 1 */
#define SAM3U_PIO_BASE           0x400e0c00 /* 0x400e0c00-0x400e11ff: Parallel I/O Controllers */
#  define SAM3U_PION_BASE(n)     (0x400e0c00+((n)<<9))
#  define SAM3U_PIOA_BASE        0x400e0c00 /* 0x400e0c00-0x400e0dff:   Parallel I/O Controller A */
#  define SAM3U_PIOB_BASE        0x400e0e00 /* 0x400e0e00-0x400e0fff:   Parallel I/O Controller B */
#  define SAM3U_PIOC_BASE        0x400e1000 /* 0x400e1000-0x400e11ff:   Parallel I/O Controller C */
#define SAM3U_RSTC_BASE          0x400e1200 /* 0x400e1200-0x400e120f: Reset Controller */
#define SAM3U_SUPC_BASE          0x400e1210 /* 0x400e1210-0x400e122f: Supply Controller */
#define SAM3U_RTT_BASE           0x400e1230 /* 0x400e1230-0x400e124f: Real Time Timer */
#define SAM3U_WDT_BASE           0x400e1250 /* 0x400e1250-0x400e125f: Watchdog Timer */
#define SAM3U_RTC_BASE           0x400e1260 /* 0x400e1260-0x400e128f: Real Time Clock */
#define SAM3U_GPBR_BASE          0x400e1290 /* 0x400e1290-0x400e13ff: GPBR */
                                            /* 0x490e1400-0x4007ffff: Reserved */

/************************************************************************************************
 * Public Types
 ************************************************************************************************/

/************************************************************************************************
 * Public Data
 ************************************************************************************************/

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/

#endif /* __ARCH_ARM_SRC_SAM3U_SAM3U_MEMORYMAP_H */
