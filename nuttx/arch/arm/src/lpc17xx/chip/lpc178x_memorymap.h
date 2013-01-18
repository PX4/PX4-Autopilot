/************************************************************************************
 * arch/arm/src/lpc17xx/lpc178x_memorymap.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Rommel Marcelo
 *            Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __ARCH_ARM_SRC_LPC17XX_LPC178X_MEMORYMAP_H
#define __ARCH_ARM_SRC_LPC17XX_LPC178X_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Memory Map ***********************************************************************/

#define LPC17_FLASH_BASE    0x00000000 /* -0x1fffffff: On-chip non-volatile memory */
#define LPC17_SRAM_BASE     0x10000000 /* -0x10007fff: On-chip SRAM (devices <=32Kb) */
#define LPC17_ROM_BASE      0x1fff0000 /* -0x1fffffff: 8Kb Boot ROM with flash services */
#define LPC17_AHBSRAM_BASE  0x20000000 /* -0x3fffffff: On-chip Peripheral SRAM (devices >32Kb) */    
#  define LPC17_SRAM_BANK0  0x20000000 /* -0x20003fff: On-chip Peripheral SRAM Bank0 (devices >=32Kb) */
#  define LPC17_SRAM_BANK1  0x20004000 /* -0x20007fff: On-chip Peripheral SRAM Bank1 (devices 64Kb) */
#define LPC17_AHB_BASE      0x20080000 /* -0x2008ffff: DMA Controller, Ethernet, and USB */
#define LPC17_SPIFI_BASE    0x28000000
#define LPC17_APB_BASE      0x40000000 /* -0x5fffffff: APB Peripherals */
#  define LPC17_APB0_BASE   0x40000000 /* -0x4007ffff: APB0 Peripherals */
#  define LPC17_APB1_BASE   0x40080000 /* -0x400fffff: APB1 Peripherals */

/* Off chip Memory via External Memory Interface */

#define LPC17_EXTRAM_BASE   0x80000000 /*  */
# define LPC17_EXTSRAM_CS0  0x80000000 /* Chip select 0 /up to 64MB/  */
# define LPC17_EXTSRAM_CS1  0x90000000 /* Chip select 1 /up to 64MB/  */
# define LPC17_EXTSRAM_CS2  0x98000000 /* Chip select 2 /up to 64MB/  */
# define LPC17_EXTSRAM_CS3  0x9c000000 /* Chip select 3 /up to 64MB/  */

# define LPC17_EXTDRAM_CS0  0x9c000000 /* Chip select 0 /up to 256MB/  */
# define LPC17_EXTDRAM_CS1  0x9c000000 /* Chip select 1 /up to 256MB/  */
# define LPC17_EXTDRAM_CS2  0x9c000000 /* Chip select 2 /up to 256MB/  */
# define LPC17_EXTDRAM_CS3  0x9c000000 /* Chip select 3 /up to 256MB/  */

#define LPC17_CORTEXM3_BASE 0xe0000000 /* -0xe00fffff: (see armv7-m/nvic.h) */
#define LPC17_SCS_BASE      0xe000e000
#define LPC17_DEBUGMCU_BASE 0xe0042000

/* AHB SRAM Bank sizes **************************************************************/

#define LPC17_BANK0_SIZE    (16*1024)  /* Size of AHB SRAM Bank0 (if present) */
#define LPC17_BANK1_SIZE    (16*1024)  /* Size of AHB SRAM Bank1 (if present) */

/* APB0 Peripherals *****************************************************************/

#define LPC17_WDT_BASE      0x40000000 /* -0x40003fff: Watchdog timer */
#define LPC17_TMR0_BASE     0x40004000 /* -0x40007fff: Timer 0 */
#define LPC17_TMR1_BASE     0x40008000 /* -0x4000bfff: Timer 1 */
#define LPC17_UART0_BASE    0x4000c000 /* -0x4000ffff: UART 0 */
#define LPC17_UART1_BASE    0x40010000 /* -0x40013fff: UART 1 */
#define LPC17_PWM0_BASE     0x40014000 /* -0x40017fff: PWM 0 */
#define LPC17_PWM1_BASE     0x40018000 /* -0x4001bfff: PWM 1 */
#define LPC17_I2C0_BASE     0x4001c000 /* -0x4001ffff: I2C 0 */
                                       /* -0x40023fff: Reserved */
#define LPC17_RTC_BASE      0x40024000 /* -0x40027fff: RTC + backup registers */
#define LPC17_GPIOINT_BASE  0x40028000 /* -0x4002bfff: GPIO interrupts */
#define LPC17_PINCONN_BASE  0x4002c000 /* -0x4002ffff: Pin connect block */
#define LPC17_SSP1_BASE     0x40030000 /* -0x40033fff: SSP 1 */
#define LPC17_ADC_BASE      0x40034000 /* -0x40037fff: ADC */
#define LPC17_CANAFRAM_BASE 0x40038000 /* -0x4003bfff: CAN acceptance filter (AF) RAM */
#define LPC17_CANAF_BASE    0x4003c000 /* -0x4003ffff: CAN acceptance filter (AF) registers */
#define LPC17_CAN_BASE      0x40040000 /* -0x40043fff: CAN common registers */
#define LPC17_CAN1_BASE     0x40044000 /* -0x40047fff: CAN controller l */
#define LPC17_CAN2_BASE     0x40048000 /* -0x4004bfff: CAN controller 2 */
                                       /* -0x4005bfff: Reserved */
#define LPC17_I2C1_BASE     0x4005c000 /* -0x4005ffff: I2C 1 */
                                       /* -0x4007ffff: Reserved */

/* APB1 Peripherals *****************************************************************/

                                       /* -0x40087fff: Reserved */
#define LPC17_SSP0_BASE     0x40088000 /* -0x4008bfff: SSP 0 */
#define LPC17_DAC_BASE      0x4008c000 /* -0x4008ffff: DAC */
#define LPC17_TMR2_BASE     0x40090000 /* -0x40093fff: Timer 2 */
#define LPC17_TMR3_BASE     0x40094000 /* -0x40097fff: Timer 3 */
#define LPC17_UART2_BASE    0x40098000 /* -0x4009bfff: UART 2 */
#define LPC17_UART3_BASE    0x4009c000 /* -0x4009ffff: UART 3 */
#define LPC17_I2C2_BASE     0x400a0000 /* -0x400a3fff: I2C 2 */
                                       /* -0x400a7fff: Reserved */
#define LPC17_I2S_BASE      0x400a8000 /* -0x400abfff: I2S */
#define LPC17_SSP2_BASE     0x400ac000 /* -0x400affff: SSP2 */
                                       /* -0x400b3fff: Reserved */
                                       /* -0x400b7fff: Reserved */
#define LPC17_MCPWM_BASE    0x400b8000 /* -0x400bbfff: Motor control PWM */
#define LPC17_QEI_BASE      0x400bc000 /* -0x400bffff: Quadrature encoder interface */
#define LPC17_MCI_BASE      0x400c0000 /* -0x400fbfff: SD interface */
#define LPC17_SYSCON_BASE   0x400fc000 /* -0x400fffff: System control */

/* AHB Peripherals ******************************************************************/

#define LPC17_GPDMA_BASE    0x20080000 /*  GPDMA controller */
#define LPC17_ETH_BASE      0x20084000 /*  Ethernet controller */
#define LPC17_LCD_BASE      0x20088000 /*  LCD controller */
#define LPC17_USB_BASE      0x2008c000 /*  USB controller */
#define LPC17_CRC_BASE      0x20090000 /*  CRC engine */
#define LPC17_GPIO_BASE     0x20098000 /*  GPIO */
#define LPC17_EMC_BASE      0x2009c000 /*  External Memory Controller */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_LPC178X_MEMORYMAP_H */
