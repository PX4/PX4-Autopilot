/************************************************************************************
 * arch/avr/src/at32uc3/at32uc3_memorymap.h
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

#ifndef __ARCH_AVR_SRC_AT32UC3_AT32UC3_MEMORYMAP_H
#define __ARCH_AVR_SRC_AT32UC3_AT32UC3_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Physical memory map */

#define AVR32_ONCHIP_SRAM_BASE  0x00000000 /* 16-64Kb SRAM */
#define AVR32_GPIO_LBUS_BASE    0x40000000 /* Local bus mapped GPIO registers */
#define AVR32_ONCHIP_FLASH_BASE 0x80000000 /* 64-512Kb Flash Array */
#  define AVR32_APPL_BASE       0x80002000 /* 8Kb offset to application w/bootloader */
#  define AVR32_USER_FLASH_BASE 0x80800000 /* Flash User Page */
#  define AVR32_BTLDR_CONFIG    0x808001fc /* Bootloader configuration word */
#define AVR32_USBDATA_BASE      0xd0000000 /* USB data (64Kb) */
#define AVR32_HSBPB_BRIDGEB     0xfffe0000 /* HSB-PB Bridge B (64Kb) */
#define AVR32_HSBPB_BRIDGEA     0xffff0000 /* HSB-PB Bridge A (64Kb) */

/* Memory map for systems without an MMU */

#define AVR32_P1_BASE           0x80000000 /* 512MB non-translated space, cacheable */
#define AVR32_P2_BASE           0xa0000000 /* 512MB non-translated space, non-cacheable */
#define AVR32_P3_BASE           0xc0000000 /* 512MB translated space, cacheable */
#define AVR32_P4_BASE           0xe0000000 /* 512MB system space, non-cacheable */

/* Reset vector addess */

#if defined(CONFIG_ARCH_CHIP_AT32UC3A)
#  define AVR32_VECTOR_BASE     AVR32_P1_BASE
#elif defined(CONFIG_ARCH_CHIP_AT32UC3B)
#  define AVR32_VECTOR_BASE     AVR32_P2_BASE
#else
#  warning "Unknown vector base address"
#endif

/* Peripheral Address Map */

#define AVR32_USB_BASE          0xfffe0000   /* USB 2.0 Interface */
#define AVR32_HMATRIX_BASE      0xfffe1000   /* HSB Matrix */
#define AVR32_HFLASHC_BASE      0xfffe1400   /* Flash Controller */
#define AVR32_PDCA_BASE         0xffff0000   /* Peripheral DMA Controller */
#define AVR32_INTC_BASE         0xffff0800   /* Interrupt controller */
#define AVR32_PM_BASE           0xffff0c00   /* Power Manager */
#define AVR32_RTC_BASE          0xffff0d00   /* Real Time Counter */
#define AVR32_WDT_BASE          0xffff0d30   /* Watchdog Timer */
#define AVR32_EIM_BASE          0xffff0d80   /* External Interrupt Controller */
#define AVR32_GPIO_BASE         0xffff1000   /* General Purpose Input/Output */
#define AVR32_USART0_BASE       0xffff1400   /* USART0 */
#define AVR32_USART1_BASE       0xffff1800   /* USART1 */
#define AVR32_USART2_BASE       0xffff1c00   /* USART2 */
#define AVR32_SPI0_BASE         0xffff2400   /* Serial Peripheral Interface 0 */
#define AVR32_TWI_BASE          0xffff2c00   /* Two-wire Interface */
#define AVR32_PWM_BASE          0xffff3000   /* Pulse Width Modulation Controller */
#define AVR32_SSC_BASE          0xffff3400   /* Synchronous Serial Controller */
#define AVR32_TC_BASE           0xffff3800   /* Timer/Counter */
#define AVR32_ADC_BASE          0xffff3c00   /* Analog to Digital Converter */
#define AVR32_ABDAC_BASE        0xffff4000   /* Audio Bitstream DAC */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_AVR_SRC_AT32UC3_AT32UC3_MEMORYMAP_H */

