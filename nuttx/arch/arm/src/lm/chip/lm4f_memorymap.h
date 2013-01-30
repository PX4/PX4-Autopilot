/************************************************************************************
 * arch/arm/src/lm/chip/lm4f_memorymap.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jose Pablo Carballo <jcarballo@nx-engineering.com>
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM4F_MEMORYMAP_H
#define __ARCH_ARM_SRC_LM_CHIP_LM4F_MEMORYMAP_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Memory map ***********************************************************************/

#if defined(CONFIG_ARCH_CHIP_LM4F120)
#  define LM_FLASH_BASE     0x00000000 /* -0x0003ffff: On-chip FLASH */
                                       /* -0x00ffffff: Reserved */
#  define LM_ROM_BASE       0x01000000 /* -0x1fffffff: Reserved for ROM */
#  define LM_SRAM_BASE      0x20000000 /* -0x20007fff: Bit-banded on-chip SRAM */
                                       /* -0x21ffffff: Reserved */
#  define LM_ASRAM_BASE     0x22000000 /* -0x220fffff: Bit-band alias of 20000000- */
                                       /* -0x3fffffff: Reserved */
#  define LM_PERIPH_BASE    0x40000000 /* -0x4001ffff: FiRM Peripherals */
                                       /* -0x41ffffff: Peripherals */
#  define LM_APERIPH_BASE   0x42000000 /* -0x43ffffff: Bit-band alias of 40000000- */
                                       /* -0xdfffffff: Reserved */
#  define LM_ITM_BASE       0xe0000000 /* -0xe0000fff: Instrumentation Trace Macrocell */
#  define LM_DWT_BASE       0xe0001000 /* -0xe0001fff: Data Watchpoint and Trace */
#  define LM_FPB_BASE       0xe0002000 /* -0xe0002fff: Flash Patch and Breakpoint */
                                       /* -0xe000dfff: Reserved */
#  define LM_NVIC_BASE      0xe000e000 /* -0xe000efff: Nested Vectored Interrupt Controller */
                                       /* -0xe003ffff: Reserved */
#  define LM_TPIU_BASE      0xe0040000 /* -0xe0040fff: Trace Port Interface Unit */
#  define LM_ETM_BASE       0xe0041000 /* -0xe0041fff: Embedded Trace Macrocell */
                                       /* -0xffffffff: Reserved */
#else
#  error "Memory map not specified for this LM4F chip"
#endif

/* Peripheral base addresses ********************************************************/

#if defined(CONFIG_ARCH_CHIP_LM4F120)
/* FiRM Peripheral Base Addresses */

#  define LM_WDOG0_BASE     (LM_PERIPH_BASE + 0x00000) /* -0x00fff: Watchdog Timer 0 */
#  define LM_WDOG1_BASE     (LM_PERIPH_BASE + 0x01000) /* -0x00fff: Watchdog Timer 1 */
                                                       /* -0x03fff: Reserved */
#  define LM_GPIOA_BASE     (LM_PERIPH_BASE + 0x04000) /* -0x04fff: GPIO Port A */
#  define LM_GPIOB_BASE     (LM_PERIPH_BASE + 0x05000) /* -0x05fff: GPIO Port B */
#  define LM_GPIOC_BASE     (LM_PERIPH_BASE + 0x06000) /* -0x06fff: GPIO Port C */
#  define LM_GPIOD_BASE     (LM_PERIPH_BASE + 0x07000) /* -0x07fff: GPIO Port D */
#  define LM_SSI0_BASE      (LM_PERIPH_BASE + 0x08000) /* -0x08fff: SSI0 */
#  define LM_SSI1_BASE      (LM_PERIPH_BASE + 0x09000) /* -0x09fff: SSI1 */
#  define LM_SSI2_BASE      (LM_PERIPH_BASE + 0x0a000) /* -0x0afff: SSI2 */
#  define LM_SSI3_BASE      (LM_PERIPH_BASE + 0x0b000) /* -0x0bfff: SSI3 */
#  define LM_UART0_BASE     (LM_PERIPH_BASE + 0x0c000) /* -0x0cfff: UART0 */
#  define LM_UART1_BASE     (LM_PERIPH_BASE + 0x0d000) /* -0x0dfff: UART1 */
#  define LM_UART2_BASE     (LM_PERIPH_BASE + 0x0e000) /* -0x0efff: UART2 */
#  define LM_UART3_BASE     (LM_PERIPH_BASE + 0x0f000) /* -0x0ffff: UART3 */
#  define LM_UART4_BASE     (LM_PERIPH_BASE + 0x10000) /* -0x10fff: UART4 */
#  define LM_UART5_BASE     (LM_PERIPH_BASE + 0x11000) /* -0x11fff: UART5 */
#  define LM_UART6_BASE     (LM_PERIPH_BASE + 0x12000) /* -0x12fff: UART6 */
#  define LM_UART7_BASE     (LM_PERIPH_BASE + 0x13000) /* -0x13fff: UART7 */
                                                       /* -0x1ffff: Reserved */
/* Peripheral Base Addresses */

#  define LM_I2CM0_BASE     (LM_PERIPH_BASE + 0x20000)  /* -0x207ff: I2C Master 0 */
#  define LM_I2CS0_BASE     (LM_PERIPH_BASE + 0x20800)  /* -0x20fbf: I2C Slave 0 */
#  define LM_I2CSC0_BASE    (LM_PERIPH_BASE + 0x20fc0)  /* -0x20fff: I2C Status and Control 0 */
#  define LM_I2CM1_BASE     (LM_PERIPH_BASE + 0x21000)  /* -0x217ff: I2C Master 1 */
#  define LM_I2CS1_BASE     (LM_PERIPH_BASE + 0x21800)  /* -0x21fbf: I2C Slave 1 */
#  define LM_I2CSC1_BASE    (LM_PERIPH_BASE + 0x21fc0)  /* -0x21fff: I2C Status and Control 1 */
#  define LM_I2CM2_BASE     (LM_PERIPH_BASE + 0x22000)  /* -0x227ff: I2C Master 2 */
#  define LM_I2CS2_BASE     (LM_PERIPH_BASE + 0x22800)  /* -0x22fbf: I2C Slave 2 */
#  define LM_I2CSC2_BASE    (LM_PERIPH_BASE + 0x22fc0)  /* -0x22fff: I2C Status and Control 2 */
#  define LM_I2CM3_BASE     (LM_PERIPH_BASE + 0x23000)  /* -0x237ff: I2C Master 3 */
#  define LM_I2CS3_BASE     (LM_PERIPH_BASE + 0x23800)  /* -0x23fbf: I2C Slave 3 */
#  define LM_I2CSC3_BASE    (LM_PERIPH_BASE + 0x23fc0)  /* -0x23fff: I2C Status and Control 3 */
#  define LM_GPIOE_BASE     (LM_PERIPH_BASE + 0x24000)  /* -0x24fff: GPIO Port E */
#  define LM_GPIOF_BASE     (LM_PERIPH_BASE + 0x25000)  /* -0x25fff: GPIO Port F */
                                                        /* -0x2ffff: Reserved */
#  define LM_TIMER0_BASE    (LM_PERIPH_BASE + 0x30000)  /* -0x30fff: 16/32 Timer 0 */
#  define LM_TIMER1_BASE    (LM_PERIPH_BASE + 0x31000)  /* -0x31fff: 16/32 Timer 1 */
#  define LM_TIMER2_BASE    (LM_PERIPH_BASE + 0x32000)  /* -0x32fff: 16/32 Timer 2 */
#  define LM_TIMER3_BASE    (LM_PERIPH_BASE + 0x33000)  /* -0x33fff: 16/32 Timer 3 */
#  define LM_TIMER4_BASE    (LM_PERIPH_BASE + 0x34000)  /* -0x34fff: 16/32 Timer 4 */
#  define LM_TIMER5_BASE    (LM_PERIPH_BASE + 0x35000)  /* -0x35fff: 16/32 Timer 5 */
#  define LM_WTIMER0_BASE   (LM_PERIPH_BASE + 0x36000)  /* -0x36fff: 32/64 Wide Timer 0 */
#  define LM_WTIMER1_BASE   (LM_PERIPH_BASE + 0x37000)  /* -0x37fff: 32/64 Wide Timer 1 */
#  define LM_ADC0_BASE      (LM_PERIPH_BASE + 0x38000)  /* -0x38fff: ADC 0 */
#  define LM_ADC1_BASE      (LM_PERIPH_BASE + 0x39000)  /* -0x39fff: ADC 1 */
                                                        /* -0x3bfff: Reserved */
#  define LM_COMPARE_BASE   (LM_PERIPH_BASE + 0x3c000)  /* -0x3cfff: Analog Comparators */
                                                        /* -0x43fff: Reserved */
#  define LM_CANCON_BASE    (LM_PERIPH_BASE + 0x40000)  /* -0x40fff: CAN Controller */
                                                        /* -0x4bfff: Reserved */
#  define LM_WTIMER2_BASE   (LM_PERIPH_BASE + 0x4c000)  /* -0x4cfff: 32/64 Wide Timer 2 */
#  define LM_WTIMER3_BASE   (LM_PERIPH_BASE + 0x4d000)  /* -0x4dfff: 32/64 Wide Timer 3 */
#  define LM_WTIMER4_BASE   (LM_PERIPH_BASE + 0x4e000)  /* -0x4efff: 32/64 Wide Timer 4 */
#  define LM_WTIMER5_BASE   (LM_PERIPH_BASE + 0x4f000)  /* -0x4ffff: 32/64 Wide Timer 5 */
#  define LM_USB_BASE       (LM_PERIPH_BASE + 0x50000)  /* -0x50fff: USB */
                                                        /* -0x57fff: Reserved */
#  define LM_GPIOAAHB_BASE  (LM_PERIPH_BASE + 0x58000)  /* -0x58fff: GPIO Port A (AHB aperture) */
#  define LM_GPIOBAHB_BASE  (LM_PERIPH_BASE + 0x59000)  /* -0x59fff: GPIO Port B (AHB aperture) */
#  define LM_GPIOCAHB_BASE  (LM_PERIPH_BASE + 0x5A000)  /* -0x5afff: GPIO Port C (AHB aperture) */
#  define LM_GPIODAHB_BASE  (LM_PERIPH_BASE + 0x5B000)  /* -0x5bfff: GPIO Port D (AHB aperture) */
#  define LM_GPIOEAHB_BASE  (LM_PERIPH_BASE + 0x5C000)  /* -0x5cfff: GPIO Port E (AHB aperture) */
#  define LM_GPIOFAHB_BASE  (LM_PERIPH_BASE + 0x5D000)  /* -0x5dfff: GPIO Port F (AHB aperture) */
                                                        /* -0xaefff: Reserved */
#  define LM_EEPROM_BASE    (LM_PERIPH_BASE + 0xaf000)  /* -0xaffff: EEPROM and Key Locker */
                                                        /* -0xf8fff: Reserved */
#  define LM_SYSEXC_BASE    (LM_PERIPH_BASE + 0xf9000)  /* -0xf9fff: System Exception Control */
                                                        /* -0xfbfff: Reserved */
#  define LM_HIBERNATE_BASE (LM_PERIPH_BASE + 0xfc000)  /* -0xfcfff: Hibernation Controller */
#  define LM_FLASHCON_BASE  (LM_PERIPH_BASE + 0xfd000)  /* -0xfdfff: FLASH Control */
#  define LM_SYSCON_BASE    (LM_PERIPH_BASE + 0xfe000)  /* -0xfefff: System Control */
#  define LM_UDMA_BASE      (LM_PERIPH_BASE + 0xff000)  /* -0xfffff: Micro Direct Memory Access */
#else
#  error "Peripheral base addresses not specified for this Stellaris chip"
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM4F_MEMORYMAP_H */
