/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f20xxx_memorymap.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32F20XXX_MEMORYMAP_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32F20XXX_MEMORYMAP_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* STM32F20XXX Address Blocks *******************************************************/

#define STM32_CODE_BASE      0x00000000     /* 0x00000000-0x1fffffff: 512Mb code block */
#define STM32_SRAM_BASE      0x20000000     /* 0x20000000-0x3fffffff: 512Mb sram block */
#define STM32_PERIPH_BASE    0x40000000     /* 0x40000000-0x5fffffff: 512Mb peripheral block */
#define STM32_FSMC_BASE12    0x60000000     /* 0x60000000-0x7fffffff: 512Mb FSMC bank1&2 block */
#  define STM32_FSMC_BANK1   0x60000000     /* 0x60000000-0x6fffffff: 256Mb NOR/SRAM */
#  define STM32_FSMC_BANK2   0x70000000     /* 0x70000000-0x7fffffff: 256Mb NAND FLASH */
#define STM32_FSMC_BASE34    0x80000000     /* 0x80000000-0x8fffffff: 512Mb FSMC bank3&4 block */
#  define STM32_FSMC_BANK3   0x80000000     /* 0x80000000-0x8fffffff: 256Mb NAND FLASH */
#  define STM32_FSMC_BANK4   0x90000000     /* 0x90000000-0x9fffffff: 256Mb PC CARD*/
#define STM32_FSMC_BASE      0xa0000000     /* 0xa0000000-0xbfffffff: 512Mb FSMC register block */
                                            /* 0xc0000000-0xdfffffff: 512Mb (not used) */
#define STM32_CORTEX_BASE    0xe0000000     /* 0xe0000000-0xffffffff: 512Mb Cortex-M4 block */

#define STM32_REGION_MASK    0xf0000000
#define STM32_IS_SRAM(a)     ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_SRAM_BASE)
#define STM32_IS_EXTSRAM(a)  ((((uint32_t)(a)) & STM32_REGION_MASK) == STM32_FSMC_BANK1)

/* Code Base Addresses **************************************************************/

#define STM32_BOOT_BASE      0x00000000     /* 0x00000000-0x000fffff: Aliased boot memory */
                                            /* 0x00100000-0x07ffffff: Reserved */
#define STM32_FLASH_BASE     0x08000000     /* 0x08000000-0x080fffff: FLASH memory */
                                            /* 0x08100000-0x0fffffff: Reserved */
#define STM32_CCMRAM_BASE    0x10000000     /* 0x10000000-0x1000ffff: 64Kb CCM data RAM */
                                            /* 0x10010000-0x1ffeffff: Reserved */
#define STM32_SYSMEM_BASE    0x1fff0000     /* 0x1fff0000-0x1fff7a0f: System memory */
                                            /* 0x1fff7a10-0x1fff7fff: Reserved */
#define STM32_OPTION_BASE    0x1fffc000     /* 0x1fffc000-0x1fffc007: Option bytes */
                                            /* 0x1fffc008-0x1fffffff: Reserved */

/* SRAM Base Addresses **************************************************************/

                                            /* 0x20000000-0x2001bfff: 112Kb aliased by bit-banding */
                                            /* 0x2001c000-0x2001ffff: 16Kb aliased by bit-banding */
#define STM32_SRAMBB_BASE    0x22000000     /* 0x22000000-          : SRAM bit-band region */

/* Peripheral Base Addresses ********************************************************/

#define STM32_APB1_BASE      0x40000000     /* 0x40000000-0x400023ff: APB1 */
                                            /* 0x40002400-0x400027ff: Reserved */
                                            /* 0x40002800-0x400077ff: APB1 */
                                            /* 0x40007800-0x4000ffff: Reserved */
#define STM32_APB2_BASE      0x40010000     /* 0x40010000-0x400023ff: APB2 */
                                            /* 0x40013400-0x400137ff: Reserved */
                                            /* 0x40013800-0x40013bff: SYSCFG */
#define STM32_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
                                            /* 0x40014000-0x40014bff: APB2 */
                                            /* 0x40014c00-0x4001ffff: Reserved */
#define STM32_AHB1_BASE      0x40020000     /* 0x40020000-0x400223ff: APB1 */
                                            /* 0x40022400-0x40022fff: Reserved */
                                            /* 0x40023000-0x400233ff: CRC */
                                            /* 0x40023400-0x400237ff: Reserved */
                                            /* 0x40023800-0x40023bff: Reset and Clock control RCC */
                                            /* 0x40023c00-0x400293ff: AHB1 (?) */
                                            /* 0x40029400-0x4fffffff: Reserved (?) */
#define STM32_AHB2_BASE      0x50000000     /* 0x50000000-0x5003ffff: AHB2 */
                                            /* 0x50040000-0x5004ffff: Reserved */
                                            /* 0x50050000-0x500503ff: AHB2 */
                                            /* 0x50050400-0x500607ff: Reserved */
                                            /* 0x50060800-0x50060bff: AHB2 */
                                            /* 0x50060c00-0x5fffffff: Reserved */

/* FSMC Base Addresses **************************************************************/

#define STM32_AHB3_BASE      0x60000000     /* 0x60000000-0xa0000fff: AHB3 */

/* APB1 Base Addresses **************************************************************/

#define STM32_TIM2_BASE      0x40000000     /* 0x40000000-0x400003ff: TIM2 timer */
#define STM32_TIM3_BASE      0x40000400     /* 0x40000400-0x400007ff: TIM3 timer */
#define STM32_TIM4_BASE      0x40000800     /* 0x40000800-0x40000bff: TIM4 timer */
#define STM32_TIM5_BASE      0x40000c00     /* 0x40000c00-0x40000fff: TIM5 timer */
#define STM32_TIM6_BASE      0x40001000     /* 0x40001000-0x400013ff: TIM6 timer */
#define STM32_TIM7_BASE      0x40001400     /* 0x40001400-0x400017ff: TIM7 timer */
#define STM32_TIM12_BASE     0x40001800     /* 0x40001800-0x40001bff: TIM12 timer */
#define STM32_TIM13_BASE     0x40001c00     /* 0x40001c00-0x40001fff: TIM13 timer */
#define STM32_TIM14_BASE     0x40002000     /* 0x40002000-0x400023ff: TIM14 timer */
#define STM32_RTC_BASE       0x40002800     /* 0x40002800-0x40002bff: RTC & BKP registers */
#define STM32_BKP_BASE       0x40002850
#define STM32_WWDG_BASE      0x40002c00     /* 0x40002c00-0x40002fff: Window watchdog (WWDG) */
#define STM32_IWDG_BASE      0x40003000     /* 0x40003000-0x400033ff: Independent watchdog (IWDG) */
#define STM32_I2S2EXT_BASE   0x40003400     /* 0x40003400-0x400037ff: I2S2ext */
#define STM32_SPI2_BASE      0x40003800     /* 0x40003800-0x40003bff: SPI2/I2S2 */
#define STM32_I2S2_BASE      0x40003800
#define STM32_SPI3_BASE      0x40003c00     /* 0x40003c00-0x40003fff: SPI3/I2S3 */
#define STM32_I2S3_BASE      0x40003c00
#define STM32_I2S3EXT_BASE   0x40004000     /* 0x40003400-0x400043ff: I2S3ext */
#define STM32_USART2_BASE    0x40004400     /* 0x40004400-0x400047ff: USART2 */
#define STM32_USART3_BASE    0x40004800     /* 0x40004800-0x40004bff: USART3 */
#define STM32_UART4_BASE     0x40004c00     /* 0x40004c00-0x40004fff: UART4 */
#define STM32_UART5_BASE     0x40005000     /* 0x40005000-0x400053ff: UART5 */
#define STM32_I2C1_BASE      0x40005400     /* 0x40005400-0x400057ff: I2C1 */
#define STM32_I2C2_BASE      0x40005800     /* 0x40005800-0x40005Bff: I2C2 */
#define STM32_I2C3_BASE      0x40005c00     /* 0x40005c00-0x40005fff: I2C3 */
#define STM32_CAN1_BASE      0x40006400     /* 0x40006400-0x400067ff: bxCAN1 */
#define STM32_CAN2_BASE      0x40006800     /* 0x40006800-0x40006bff: bxCAN2 */
#define STM32_PWR_BASE       0x40007000     /* 0x40007000-0x400073ff: Power control PWR */
#define STM32_DAC_BASE       0x40007400     /* 0x40007400-0x400077ff: DAC */

/* APB2 Base Addresses **************************************************************/

#define STM32_TIM1_BASE      0x40010000     /* 0x40010000-0x400103ff: TIM1 timer */
#define STM32_TIM8_BASE      0x40010400     /* 0x40010400-0x400107ff: TIM8 timer */
#define STM32_USART1_BASE    0x40011000     /* 0x40011000-0x400113ff: USART1 */
#define STM32_USART6_BASE    0x40011400     /* 0x40011400-0x400117ff: USART6 */
#define STM32_ADC_BASE       0x40012000     /* 0x40012000-0x400123ff: ADC1-3 */
#  define STM32_ADC1_BASE    0x40012000     /*                        ADC1 */
#  define STM32_ADC2_BASE    0x40012100     /*                        ADC2 */
#  define STM32_ADC3_BASE    0x40012200     /*                        ADC3 */
#  define STM32_ADCCMN_BASE  0x40012300     /*                        Common */
#define STM32_SDIO_BASE      0x40012c00     /* 0x40012c00-0x40012fff: SDIO  */
#define STM32_SPI1_BASE      0x40013000     /* 0x40013000-0x400133ff: SPI1 */
#define STM32_SYSCFG_BASE    0x40013800     /* 0x40013800-0x40013bff: SYSCFG */
#define STM32_EXTI_BASE      0x40013c00     /* 0x40013c00-0x40013fff: EXTI */
#define STM32_TIM9_BASE      0x40014000     /* 0x40014000-0x400143ff: TIM9 timer */
#define STM32_TIM10_BASE     0x40014400     /* 0x40014400-0x400147ff: TIM10 timer */
#define STM32_TIM11_BASE     0x40014800     /* 0x40014800-0x40014bff: TIM11 timer */

/* AHB1 Base Addresses **************************************************************/

#define STM32_GPIOA_BASE     0x40020000     /* 0x40020000-0x400203ff: GPIO Port A */
#define STM32_GPIOB_BASE     0x40020400     /* 0x40020400-0x400207ff: GPIO Port B */
#define STM32_GPIOC_BASE     0x40020800     /* 0x40020800-0x40020bff: GPIO Port C */
#define STM32_GPIOD_BASE     0X40020C00     /* 0x40020c00-0x40020fff: GPIO Port D */
#define STM32_GPIOE_BASE     0x40021000     /* 0x40021000-0x400213ff: GPIO Port E */
#define STM32_GPIOF_BASE     0x40021400     /* 0x40021400-0x400217ff: GPIO Port F */
#define STM32_GPIOG_BASE     0x40021800     /* 0x40021800-0x40021bff: GPIO Port G */
#define STM32_GPIOH_BASE     0x40021C00     /* 0x40021C00-0x40021fff: GPIO Port H */
#define STM32_GPIOI_BASE     0x40022000     /* 0x40022000-0x400223ff: GPIO Port I */
#define STM32_CRC_BASE       0x40023000     /* 0x40023000-0x400233ff: CRC */
#define STM32_RCC_BASE       0x40023800     /* 0x40023800-0x40023bff: Reset and Clock control RCC */
#define STM32_FLASHIF_BASE   0x40023c00     /* 0x40023c00-0x40023fff: Flash memory interface */
#define STM32_BKPSRAM_BASE   0x40024000     /* 0x40024000-0x40024fff: Backup SRAM (BKPSRAM) */
#define STM32_DMA1_BASE      0x40026000     /* 0x40026000-0x400263ff: DMA1  */
#define STM32_DMA2_BASE      0x40026400     /* 0x40026400-0x400267ff: DMA2  */
#define STM32_ETHERNET_BASE  0x40028000     /* 0x40028000-0x400283ff: Ethernet MAC */
                                            /* 0x40028400-0x400287ff: Ethernet MAC */
                                            /* 0x40028800-0x40028bff: Ethernet MAC */
                                            /* 0x40028c00-0x40028fff: Ethernet MAC */
                                            /* 0x40029000-0x400293ff: Ethernet MAC */
#define STM32_OTGHS_BASE     0x40040000     /* 0x40040000-0x4007ffff: USB OTG HS */
#define STM32_PERIPHBB_BASE  0x42000000     /* Peripheral bit-band region */

/* AHB2 Base Addresses **************************************************************/

#define STM32_OTGFS_BASE     0x50000000     /* 0x50000000-0x5003ffff: USB OTG FS */
#define STM32_DCMI_BASE      0x50050000     /* 0x50050000-0x500503ff: DCMI */
#define STM32_CRYP_BASE      0x50060000     /* 0x50060000-0x500603ff: CRYP */
#define STM32_HASH_BASE      0x50060400     /* 0x50060400-0x500607ff: HASH */
#define STM32_RNG_BASE       0x50060800     /* 0x50060800-0x50060bff: RNG */

/* Cortex-M3 Base Addresses *********************************************************/
/* Other registers -- see armv7-m/nvic.h for standard Cortex-M3 registers in this
 * address range
 */

#define STM32_SCS_BASE      0xe000e000
#define STM32_DEBUGMCU_BASE 0xe0042000

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32F20XXX_MEMORYMAP_H */

