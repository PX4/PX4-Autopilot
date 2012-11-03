/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f10xxx_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* FLASH and SRAM *******************************************************************/

#define STM32_FLASH_BASE     0x08000000     /* 0x08000000 - Up to 512Kb */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000 - 64Kb SRAM */
#define STM32_SRAMBB_BASE    0x22000000
#define STM32_PERIPH_BASE    0x40000000

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)

/* Register Base Address ************************************************************/

/* APB1 bus */

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000 - 0x400003ff: TIM2 timer */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400 - 0x400007ff: TIM3 timer */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800 - 0x40000bff: TIM4 timer */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00 - 0x40000fff: TIM5 timer */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000 - 0x400013ff: TIM6 timer */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400 - 0x400007ff: TIM7 timer */
#define STM32_TIM12_BASE     0x40001800     /* 0x40001800 - 0x40001bff: TIM12 timer */
#define STM32_TIM13_BASE     0x40001c00     /* 0x40001c00 - 0x40001fff: TIM13 timer */
#define STM32_TIM14_BASE     0x40002000     /* 0x40002000 - 0x400023ff: TIM14 timer */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800 - 0x40002bff: RTC */
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002C00 - 0x40002fff: Window watchdog (WWDG) */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000 - 0x400033ff: Independent watchdog (IWDG) */
                                            /* 0x40003400 - 0x400037ff: Reserved */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800 - 0x40003bff: SPI2/I2S2 */
#define STM32_I2S2_BASE      0x40003800
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00 - 0x40003fff: SPI3/I2S3 */
#define STM32_I2S3_BASE      0x40003c00
                                            /* 0x40004000 - 0x400043ff: Reserved */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400 - 0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800 - 0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00 - 0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000 - 0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400 - 0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800 - 0x40005Bff: I2C2 */
#define STM32_USB_BASE       0x40005c00     /* 0x40005c00 - 0x40005fff: USB device FS registers */
#define STM32_USBCANRAM_BASE 0x40006000     /* 0x40006000 - 0x400063ff: Shared USB/CAN SRAM 512 bytes */
#define STM32_CAN1_BASE      0x40006400     /* 0x40006400 - 0x400067ff: bxCAN1 */
#define STM32_CAN2_BASE      0x40006800     /* 0x40006800 - 0x40006bff: bxCAN2 */
#define STM32_BKP_BASE       0x40006c00     /* 0x40006c00 - 0x40006fff: Backup registers (BKP) */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000 - 0x400073ff: Power control PWR */
#define STM32_DAC_BASE       0x40007400     /* 0x40007400 - 0x400077ff: DAC */
#define STM32_CEC_BASE       0x40007800     /* 0x40007800 - 0x40007bff: CEC */
                                            /* 0x40007c00 - 0x4000ffff: Reserved */
/* APB2 bus */

#define STM32_AFIO_BASE      0x40010000     /* 0x40010000 - 0x400103ff: AFIO */
#define STM32_EXTI_BASE      0x40010400     /* 0x40010400 - 0x400107ff: EXTI */
#define STM32_GPIOA_BASE     0x40010800     /* 0x40010800 - 0x40010bff: GPIO Port A */
#define STM32_GPIOB_BASE     0X40010c00     /* 0X40010c00 - 0x40010fff: GPIO Port B */
#define STM32_GPIOC_BASE     0x40011000     /* 0x40011000 - 0x400113ff: GPIO Port C */
#define STM32_GPIOD_BASE     0x40011400     /* 0x40011400 - 0x400117ff: GPIO Port D */
#define STM32_GPIOE_BASE     0x40011800     /* 0x40011800 - 0x40011bff: GPIO Port E */
#define STM32_GPIOF_BASE     0x40011c00     /* 0x4001c000 - 0x400111ff: GPIO Port F */
#define STM32_GPIOG_BASE     0x40012000     /* 0x40012000 - 0x400123ff: GPIO Port G */
#define STM32_ADC1_BASE      0x40012400     /* 0x40012400 - 0x400127ff: ADC1 */
#define STM32_ADC2_BASE      0x40012800     /* 0x40012800 - 0x40012bff: ADC2 */
#define STM32_TIM1_BASE      0x40012c00     /* 0x40012c00 - 0x40012fff: TIM1 timer */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000 - 0x400133ff: SPI1 */
#define STM32_TIM8_BASE      0x40013400     /* 0x40013400 - 0x400137ff: TIM8 timer */
#define STM32_USART1_BASE    0x40013800     /* 0x40013800 - 0x40013bff: USART1 */
#define STM32_ADC3_BASE      0x40012800     /* 0x40012800 - 0x40013c00: ADC3 */
                                            /* 0x40013c00 - 0x40013fff: Reserved */
#define STM32_TIM15_BASE     0x40014400     /* 0x40014400 - 0x400147ff: TIM15 */
#define STM32_TIM16_BASE     0x40014400     /* 0x40014400 - 0x400147ff: TIM16 */
#define STM32_TIM17_BASE     0x40014800     /* 0x40014800 - 0x40014bff: TIM17 */
                                            /* 0x40014c00 - 0x4001ffff: Reserved */

/* AHB bus */

#define STM32_SDIO_BASE      0x40018000     /* 0x40018000 - 0x400183ff: SDIO  */
                                            /* 0x40018400 - 0x40017fff: Reserved */
#define STM32_DMA1_BASE      0x40020000     /* 0x40020000 - 0x400203ff: DMA1  */
#define STM32_DMA2_BASE      0x40020400     /* 0x40020000 - 0x400207ff: DMA2  */
                                            /* 0x40020800 - 0x40020fff: Reserved */
#define STM32_RCC_BASE       0x40021000     /* 0x40021000 - 0x400213ff: Reset and Clock control RCC */
                                            /* 0x40021400 - 0x40021fff:  Reserved */
#define STM32_OTGFS_BASE     0x50000000     /* 0x50000000 - 0x500003ff: USB OTG FS */
#define STM32_FLASHIF_BASE   0x40022000     /* 0x40022000 - 0x400223ff: Flash memory interface */
#define STM32_CRC_BASE       0x40028000     /* 0x40023000 - 0x400233ff: CRC */
                                            /* 0x40023400 - 0x40027fff: Reserved */
#define STM32_ETHERNET_BASE  0x40028000     /* 0x40028000 - 0x40029fff: Ethernet */
                                            /* 0x40030000 - 0x4fffffff: Reserved */

/* Peripheral BB base */

#define STM32_PERIPHBB_BASE  0x42000000

/* Flexible SRAM controller (FSMC) */

#define STM32_FSMC_BANK1     0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/SRAM */
#define STM32_FSMC_BANK2     0x70000000     /* 0x70000000-0x7fffffff: 256Mb NAND FLASH */
#define STM32_FSMC_BANK3     0x80000000     /* 0x80000000-0x8fffffff: 256Mb  NAND FLASH */
#define STM32_FSMC_BANK4     0x90000000     /* 0x90000000-0x9fffffff: 256Mb PC CARD*/
#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FSMC_BANK1)

#define STM32_FSMC_BASE      0xa0000000     /* 0xa0000000-0xbfffffff: 512Mb FSMC register block */

/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range
 */

#define STM32_SCS_BASE       0xe000e000
#define STM32_DEBUGMCU_BASE  0xe0042000

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_MEMORYMAP_H */

