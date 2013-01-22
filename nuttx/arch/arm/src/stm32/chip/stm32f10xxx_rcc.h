/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f10xx_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_RCC_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_RCC_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register Offsets *****************************************************************/

#define STM32_RCC_CR_OFFSET         0x0000  /* Clock control register */
#define STM32_RCC_CFGR_OFFSET       0x0004  /* Clock configuration register */
#define STM32_RCC_CIR_OFFSET        0x0008  /* Clock interrupt register */
#define STM32_RCC_APB2RSTR_OFFSET   0x000c  /* APB2 Peripheral reset register */
#define STM32_RCC_APB1RSTR_OFFSET   0x0010  /* APB1 Peripheral reset register */
#define STM32_RCC_AHBENR_OFFSET     0x0014  /* AHB Peripheral Clock enable register */
#define STM32_RCC_APB2ENR_OFFSET    0x0018  /* APB2 Peripheral Clock enable register */
#define STM32_RCC_APB1ENR_OFFSET    0x001c  /* APB1 Peripheral Clock enable register */
#define STM32_RCC_BDCR_OFFSET       0x0020  /* Backup domain control register */
#define STM32_RCC_CSR_OFFSET        0x0024  /* Control/status register */
#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define STM32_RCC_AHBRSTR_OFFSET  0x0028  /* AHB Reset register */
#endif
#if defined(CONFIG_STM32_VALUELINE) || defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define STM32_RCC_CFGR2_OFFSET    0x002c  /* Clock configuration register 2 */
#endif

/* Register Addresses ***************************************************************/

#define STM32_RCC_CR                (STM32_RCC_BASE+STM32_RCC_CR_OFFSET)
#define STM32_RCC_CFGR              (STM32_RCC_BASE+STM32_RCC_CFGR_OFFSET)
#define STM32_RCC_CIR               (STM32_RCC_BASE+STM32_RCC_CIR_OFFSET)
#define STM32_RCC_APB2RSTR          (STM32_RCC_BASE+STM32_RCC_APB2RSTR_OFFSET)
#define STM32_RCC_APB1RSTR          (STM32_RCC_BASE+STM32_RCC_APB1RSTR_OFFSET)
#define STM32_RCC_AHBENR            (STM32_RCC_BASE+STM32_RCC_AHBENR_OFFSET)
#define STM32_RCC_APB2ENR           (STM32_RCC_BASE+STM32_RCC_APB2ENR_OFFSET)
#define STM32_RCC_APB1ENR           (STM32_RCC_BASE+STM32_RCC_APB1ENR_OFFSET)
#define STM32_RCC_BDCR              (STM32_RCC_BASE+STM32_RCC_BDCR_OFFSET)
#define STM32_RCC_CSR               (STM32_RCC_BASE+STM32_RCC_CSR_OFFSET)
#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define STM32_RCC_AHBRSTR         (STM32_RCC_BASE+STM32_RCC_AHBRSTR_OFFSET)
#endif
#if defined(CONFIG_STM32_VALUELINE) || defined(CONFIG_STM32_CONNECTIVITYLINE)
#  define STM32_RCC_CFGR2           (STM32_RCC_BASE+STM32_RCC_CFGR2_OFFSET)
#endif

/* Register Bitfield Definitions ****************************************************/

/* Clock control register */

#define RCC_CR_HSION                (1 << 0)  /* Bit 0: Internal High Speed clock enable */
#define RCC_CR_HSIRDY               (1 << 1)  /* Bit 1: Internal High Speed clock ready flag */
#define RCC_CR_HSITRIM_SHIFT        (3)       /* Bits 7-3: Internal High Speed clock trimming */
#define RCC_CR_HSITRIM_MASK         (0x1f << RCC_CR_HSITRIM_SHIFT)
#define RCC_CR_HSICAL_SHIFT         (8)       /* Bits 15-8: Internal High Speed clock Calibration */
#define RCC_CR_HSICAL_MASK          (0xff << RCC_CR_HSICAL_SHIFT)
#define RCC_CR_HSEON                (1 << 16) /* Bit 16: External High Speed clock enable */
#define RCC_CR_HSERDY               (1 << 17) /* Bit 17: External High Speed clock ready flag */
#define RCC_CR_HSEBYP               (1 << 18) /* Bit 18: External High Speed clock Bypass */
#define RCC_CR_CSSON                (1 << 19) /* Bit 19: Clock Security System enable */
#define RCC_CR_PLLON                (1 << 24) /* Bit 24: PLL enable */
#define RCC_CR_PLLRDY               (1 << 25) /* Bit 25: PLL clock ready flag */
#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define RCC_CR_PLL2ON             (1 << 26) /* Bit 26: PLL2 enable */
#  define RCC_CR_PLL2RDY            (1 << 27) /* Bit 27: PLL2 clock ready flag */
#  define RCC_CR_PLL3ON             (1 << 28) /* Bit 28: PLL3 enable */
#  define RCC_CR_PLL3RDY            (1 << 29) /* Bit 29: PLL3 ready flag */
#endif

/* Clock configuration register */

#define RCC_CFGR_SW_SHIFT           (0)       /* Bits 1-0: System clock Switch */
#define RCC_CFGR_SW_MASK            (3 << RCC_CFGR_SW_SHIFT)
#  define RCC_CFGR_SW_HSI           (0 << RCC_CFGR_SW_SHIFT) /* 00: HSI selected as system clock */
#  define RCC_CFGR_SW_HSE           (1 << RCC_CFGR_SW_SHIFT) /* 01: HSE selected as system clock */
#  define RCC_CFGR_SW_PLL           (2 << RCC_CFGR_SW_SHIFT) /* 10: PLL selected as system clock */
#define RCC_CFGR_SWS_SHIFT          (2)       /* Bits 3-2: System Clock Switch Status */
#define RCC_CFGR_SWS_MASK           (3 << RCC_CFGR_SWS_SHIFT)
#  define RCC_CFGR_SWS_HSI          (0 << RCC_CFGR_SWS_SHIFT) /* 00: HSI oscillator used as system clock */
#  define RCC_CFGR_SWS_HSE          (1 << RCC_CFGR_SWS_SHIFT) /* 01: HSE oscillator used as system clock */
#  define RCC_CFGR_SWS_PLL          (2 << RCC_CFGR_SWS_SHIFT) /* 10: PLL used as system clock */
#define RCC_CFGR_HPRE_SHIFT         (4)       /* Bits 7-4: AHB prescaler */
#define RCC_CFGR_HPRE_MASK          (0x0f << RCC_CFGR_HPRE_SHIFT)
#  define RCC_CFGR_HPRE_SYSCLK      (0 << RCC_CFGR_HPRE_SHIFT) /* 0xxx: SYSCLK not divided */
#  define RCC_CFGR_HPRE_SYSCLKd2    (8 << RCC_CFGR_HPRE_SHIFT) /* 1000: SYSCLK divided by 2 */
#  define RCC_CFGR_HPRE_SYSCLKd4    (9 << RCC_CFGR_HPRE_SHIFT) /* 1001: SYSCLK divided by 4 */
#  define RCC_CFGR_HPRE_SYSCLKd8    (10 << RCC_CFGR_HPRE_SHIFT) /* 1010: SYSCLK divided by 8 */
#  define RCC_CFGR_HPRE_SYSCLKd16   (11 << RCC_CFGR_HPRE_SHIFT) /* 1011: SYSCLK divided by 16 */
#  define RCC_CFGR_HPRE_SYSCLKd64   (12 << RCC_CFGR_HPRE_SHIFT) /* 1100: SYSCLK divided by 64 */
#  define RCC_CFGR_HPRE_SYSCLKd128  (13 << RCC_CFGR_HPRE_SHIFT) /* 1101: SYSCLK divided by 128 */
#  define RCC_CFGR_HPRE_SYSCLKd256  (14 << RCC_CFGR_HPRE_SHIFT) /* 1110: SYSCLK divided by 256 */
#  define RCC_CFGR_HPRE_SYSCLKd512  (15 << RCC_CFGR_HPRE_SHIFT) /* 1111: SYSCLK divided by 512 */
#define RCC_CFGR_PPRE1_SHIFT        (8)       /* Bits 10-8: APB Low speed prescaler (APB1) */
#define RCC_CFGR_PPRE1_MASK         (7 << RCC_CFGR_PPRE1_SHIFT)
#  define RCC_CFGR_PPRE1_HCLK       (0 << RCC_CFGR_PPRE1_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE1_HCLKd2     (4 << RCC_CFGR_PPRE1_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE1_HCLKd4     (5 << RCC_CFGR_PPRE1_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE1_HCLKd8     (6 << RCC_CFGR_PPRE1_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE1_HCLKd16    (7 << RCC_CFGR_PPRE1_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_PPRE2_SHIFT        (11)      /* Bits 13-11: APB High speed prescaler (APB2) */
#define RCC_CFGR_PPRE2_MASK         (7 << RCC_CFGR_PPRE2_SHIFT)
#  define RCC_CFGR_PPRE2_HCLK       (0 << RCC_CFGR_PPRE2_SHIFT) /* 0xx: HCLK not divided */
#  define RCC_CFGR_PPRE2_HCLKd2     (4 << RCC_CFGR_PPRE2_SHIFT) /* 100: HCLK divided by 2 */
#  define RCC_CFGR_PPRE2_HCLKd4     (5 << RCC_CFGR_PPRE2_SHIFT) /* 101: HCLK divided by 4 */
#  define RCC_CFGR_PPRE2_HCLKd8     (6 << RCC_CFGR_PPRE2_SHIFT) /* 110: HCLK divided by 8 */
#  define RCC_CFGR_PPRE2_HCLKd16    (7 << RCC_CFGR_PPRE2_SHIFT) /* 111: HCLK divided by 16 */
#define RCC_CFGR_ADCPRE_SHIFT       (14)      /* Bits 15-14: ADC prescaler */
#define RCC_CFGR_ADCPRE_MASK        (3 << RCC_CFGR_ADCPRE_SHIFT)
#  define RCC_CFGR_PLCK2d2          (0 << RCC_CFGR_ADCPRE_SHIFT) /* 00: PLCK2 divided by 2 */
#  define RCC_CFGR_PLCK2d4          (1 << RCC_CFGR_ADCPRE_SHIFT) /* 01: PLCK2 divided by 4 */
#  define RCC_CFGR_PLCK2d6          (2 << RCC_CFGR_ADCPRE_SHIFT) /* 10: PLCK2 divided by 6 */
#  define RCC_CFGR_PLCK2d8          (3 << RCC_CFGR_ADCPRE_SHIFT) /* 11: PLCK2 divided by 8 */
#define RCC_CFGR_PLLSRC             (1 << 16) /* Bit 16: PLL entry clock source */
#define RCC_CFGR_PLLXTPRE           (1 << 17) /* Bit 17: HSE divider for PLL entry */
#define RCC_CFGR_PLLMUL_SHIFT       (18)      /* Bits 21-18: PLL Multiplication Factor */
#define RCC_CFGR_PLLMUL_MASK        (0x0f << RCC_CFGR_PLLMUL_SHIFT)
#  define RCC_CFGR_PLLMUL_CLKx2     (0 << RCC_CFGR_PLLMUL_SHIFT)  /* 0000: PLL input clock x 2 */
#  define RCC_CFGR_PLLMUL_CLKx3     (1 << RCC_CFGR_PLLMUL_SHIFT)  /* 0001: PLL input clock x 3 */
#  define RCC_CFGR_PLLMUL_CLKx4     (2 << RCC_CFGR_PLLMUL_SHIFT)  /* 0010: PLL input clock x 4 */
#  define RCC_CFGR_PLLMUL_CLKx5     (3 << RCC_CFGR_PLLMUL_SHIFT)  /* 0011: PLL input clock x 5 */
#  define RCC_CFGR_PLLMUL_CLKx6     (4 << RCC_CFGR_PLLMUL_SHIFT)  /* 0100: PLL input clock x 6 */
#  define RCC_CFGR_PLLMUL_CLKx7     (5 << RCC_CFGR_PLLMUL_SHIFT)  /* 0101: PLL input clock x 7 */
#  define RCC_CFGR_PLLMUL_CLKx8     (6 << RCC_CFGR_PLLMUL_SHIFT)  /* 0110: PLL input clock x 8 */
#  define RCC_CFGR_PLLMUL_CLKx9     (7 << RCC_CFGR_PLLMUL_SHIFT)  /* 0111: PLL input clock x 9 */
#  define RCC_CFGR_PLLMUL_CLKx10    (8 << RCC_CFGR_PLLMUL_SHIFT)  /* 1000: PLL input clock x 10 */
#  define RCC_CFGR_PLLMUL_CLKx11    (9 << RCC_CFGR_PLLMUL_SHIFT)  /* 1001: PLL input clock x 11 */
#  define RCC_CFGR_PLLMUL_CLKx12    (10 << RCC_CFGR_PLLMUL_SHIFT) /* 1010: PLL input clock x 12 */
#  define RCC_CFGR_PLLMUL_CLKx13    (11 << RCC_CFGR_PLLMUL_SHIFT) /* 1011: PLL input clock x 13 */
#  define RCC_CFGR_PLLMUL_CLKx14    (12 << RCC_CFGR_PLLMUL_SHIFT) /* 1100: PLL input clock x 14 */
#  define RCC_CFGR_PLLMUL_CLKx15    (13 << RCC_CFGR_PLLMUL_SHIFT) /* 1101: PLL input clock x 15 */
#  define RCC_CFGR_PLLMUL_CLKx16    (14 << RCC_CFGR_PLLMUL_SHIFT) /* 111x: PLL input clock x 16 */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_CFGR_USBPRE           (1 << 22) /* Bit 22: USB prescaler */
#endif
#define RCC_CFGR_MCO_SHIFT          (24)      /* Bits 26-24: Microcontroller Clock Output */
#define RCC_CFGR_MCO_MASK           (0x0f << RCC_CFGR_MCO_SHIFT)
#  define RCC_CFGR_NOCLK            (0 << RCC_CFGR_MCO_SHIFT)  /* 0xx: No clock */
#  define RCC_CFGR_SYSCLK           (4 << RCC_CFGR_MCO_SHIFT)  /* 100: System clock selected */
#  define RCC_CFGR_INTCLK           (5 << RCC_CFGR_MCO_SHIFT)  /* 101: Internal 8 MHz RC oscillator clock selected */
#  define RCC_CFGR_EXTCLK           (6 << RCC_CFGR_MCO_SHIFT)  /* 110: External 1-25 MHz oscillator clock selected */
#  define RCC_CFGR_PLLCLKd2         (7 << RCC_CFGR_MCO_SHIFT)  /* 111: PLL clock divided by 2 selected */
#  define RCC_CFGR_PLL2CLK          (8 << RCC_CFGR_MCO_SHIFT)  /* 1000: PLL2 clock selected */
#  define RCC_CFGR_PLL3CLKd2        (9 << RCC_CFGR_MCO_SHIFT)  /* 1001: PLL3 clock devided by 2 selected */
#  define RCC_CFGR_XT1              (10 << RCC_CFGR_MCO_SHIFT) /* 1010: external 3-25 MHz oscillator clock selected (for Ethernet) */
#  define RCC_CFGR_PLL3CLK          (11 << RCC_CFGR_MCO_SHIFT) /* 1011: PLL3 clock selected (for Ethernet) */

/* Clock interrupt register */

#define RCC_CIR_LSIRDYF             (1 << 0)  /* Bit 0: LSI Ready Interrupt flag */
#define RCC_CIR_LSERDYF             (1 << 1)  /* Bit 1: LSE Ready Interrupt flag */
#define RCC_CIR_HSIRDYF             (1 << 2)  /* Bit 2: HSI Ready Interrupt flag */
#define RCC_CIR_HSERDYF             (1 << 3)  /* Bit 3: HSE Ready Interrupt flag */
#define RCC_CIR_PLLRDYF             (1 << 4)  /* Bit 4: PLL Ready Interrupt flag */
#define RCC_CIR_CSSF                (1 << 7)  /* Bit 7: Clock Security System Interrupt flag */
#define RCC_CIR_LSIRDYIE            (1 << 8)  /* Bit 8: LSI Ready Interrupt Enable */
#define RCC_CIR_LSERDYIE            (1 << 9)  /* Bit 9: LSE Ready Interrupt Enable */
#define RCC_CIR_HSIRDYIE            (1 << 10) /* Bit 10: HSI Ready Interrupt Enable */
#define RCC_CIR_HSERDYIE            (1 << 11) /* Bit 11: HSE Ready Interrupt Enable */
#define RCC_CIR_PLLRDYIE            (1 << 12) /* Bit 12: PLL Ready Interrupt Enable */
#define RCC_CIR_LSIRDYC             (1 << 16) /* Bit 16: LSI Ready Interrupt Clear */
#define RCC_CIR_LSERDYC             (1 << 17) /* Bit 17: LSE Ready Interrupt Clear */
#define RCC_CIR_HSIRDYC             (1 << 18) /* Bit 18: HSI Ready Interrupt Clear */
#define RCC_CIR_HSERDYC             (1 << 19) /* Bit 19: HSE Ready Interrupt Clear */
#define RCC_CIR_PLLRDYC             (1 << 20) /* Bit 20: PLL Ready Interrupt Clear */
#define RCC_CIR_CSSC                (1 << 23) /* Bit 23: Clock Security System Interrupt Clear */

/* APB2 Peripheral reset register */

#define RCC_APB2RSTR_AFIORST        (1 << 0)  /* Bit 0: Alternate Function I/O reset */
#define RCC_APB2RSTR_IOPARST        (1 << 2)  /* Bit 2: I/O port A reset */
#define RCC_APB2RSTR_IOPBRST        (1 << 3)  /* Bit 3: IO port B reset */
#define RCC_APB2RSTR_IOPCRST        (1 << 4)  /* Bit 4: IO port C reset */
#define RCC_APB2RSTR_IOPDRST        (1 << 5)  /* Bit 5: IO port D reset */
#define RCC_APB2RSTR_IOPERST        (1 << 6)  /* Bit 6: IO port E reset */
#define TCC_APB2RSTR_IOPFRST        (1 << 7)  /* Bit 7: IO port F reset */
#define TCC_APB2RSTR_IOPGRST        (1 << 8)  /* Bit 8: IO port G reset */
#define RCC_APB2RSTR_ADC1RST        (1 << 9)  /* Bit 9: ADC 1 interface reset */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB2RSTR_ADC2RST      (1 << 10) /* Bit 10: ADC 2 interface reset */
#endif
#define RCC_APB2RSTR_TIM1RST        (1 << 11) /* Bit 11: TIM1 Timer reset */
#define RCC_APB2RSTR_SPI1RST        (1 << 12) /* Bit 12: SPI 1 reset */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB2RSTR_TIM8RST      (1 << 13) /* Bit 13: TIM8 Timer reset */
#endif
#define RCC_APB2RSTR_USART1RST      (1 << 14) /* Bit 14: USART1 reset */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB2RSTR_ADC3RST      (1 << 15) /* Bit 15: ADC3 interface reset */
#else
#  define RCC_APB2RSTR_TIM15RST     (1 << 16) /* Bit 16: TIM15 reset */
#  define RCC_APB2RSTR_TIM16RST     (1 << 17) /* Bit 17: TIM16 reset */
#  define RCC_APB2RSTR_TIM17RST     (1 << 18) /* Bit 18: TIM17 reset */
#endif

/* APB1 Peripheral reset register */

#define RCC_APB1RSTR_TIM2RST        (1 << 0)  /* Bit 0: Timer 2 reset */
#define RCC_APB1RSTR_TIM3RST        (1 << 1)  /* Bit 1: Timer 3 reset */
#define RCC_APB1RSTR_TIM4RST        (1 << 2)  /* Bit 2: Timer 4 reset */
#define RCC_APB1RSTR_TIM5RST        (1 << 3)  /* Bit 3: Timer 5 reset */
#define RCC_APB1RSTR_TIM6RST        (1 << 4)  /* Bit 4: Timer 6 reset */
#define RCC_APB1RSTR_TIM7RST        (1 << 5)  /* Bit 5: Timer 7 reset */
#ifdef CONFIG_STM32_VALUELINE
#  define RCC_APB1RSTR_TIM12RST     (1 << 6) /* Bit 6: TIM12 reset */
#  define RCC_APB1RSTR_TIM13RST     (1 << 7) /* Bit 7: TIM13 reset */
#  define RCC_APB1RSTR_TIM14RST     (1 << 8) /* Bit 8: TIM14 reset */
#endif
#define RCC_APB1RSTR_WWDGRST        (1 << 11) /* Bit 11: Window Watchdog reset */
#define RCC_APB1RSTR_SPI2RST        (1 << 14) /* Bit 14: SPI 2 reset */
#define RCC_APB1RSTR_SPI3RST        (1 << 15) /* Bit 15: SPI 3 reset */
#define RCC_APB1RSTR_USART2RST      (1 << 17) /* Bit 17: USART 2 reset */
#define RCC_APB1RSTR_USART3RST      (1 << 18) /* Bit 18: USART 3 reset */
#define RCC_APB1RSTR_UART4RST       (1 << 19) /* Bit 19: UART 4 reset */
#define RCC_APB1RSTR_UART5RST       (1 << 20) /* Bit 18: UART 5 reset */
#define RCC_APB1RSTR_I2C1RST        (1 << 21) /* Bit 21: I2C 1 reset */
#define RCC_APB1RSTR_I2C2RST        (1 << 22) /* Bit 22: I2C 2 reset */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB1RSTR_USBRST       (1 << 23) /* Bit 23: USB reset */
#  define RCC_APB1RSTR_CAN1RST      (1 << 25) /* Bit 25: CAN1 reset */
#  define RCC_APB1RSTR_CAN2RST      (1 << 26) /* Bit 26: CAN2 reset */
#endif
#define RCC_APB1RSTR_BKPRST         (1 << 27) /* Bit 27: Backup interface reset */
#define RCC_APB1RSTR_PWRRST         (1 << 28) /* Bit 28: Power interface reset */
#define RCC_APB1RSTR_DACRST         (1 << 29) /* Bit 29: DAC interface reset */
#ifdef CONFIG_STM32_VALUELINE
#  define RCC_APB1RSTR_CECRST       (1 << 30) /* Bit 30: CEC reset */
#endif

/* AHB Peripheral Clock enable register */

#define RCC_AHBENR_DMA1EN           (1 << 0)  /* Bit 0: DMA1 clock enable */
#define RCC_AHBENR_DMA2EN           (1 << 1)  /* Bit 1: DMA2 clock enable */
#define RCC_AHBENR_SRAMEN           (1 << 2)  /* Bit 2: SRAM interface clock enable */
#define RCC_AHBENR_FLITFEN          (1 << 4)  /* Bit 4: FLITF clock enable */
#define RCC_AHBENR_CRCEN            (1 << 6)  /* Bit 6: CRC clock enable */
#define RCC_AHBENR_FSMCEN           (1 << 8)  /* Bit 8: FSMC clock enable */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_AHBENR_SDIOEN         (1 << 10) /* Bit 10: SDIO clock enable */
#endif
#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define RCC_AHBENR_ETHMACEN       (1 << 14) /* Bit 14: Ethernet MAC clock enable */
#  define RCC_AHBENR_ETHMACTXEN     (1 << 15) /* Bit 15: Ethernet MAC TX clock enable */
#  define RCC_AHBENR_ETHMACRXEN     (1 << 16) /* Bit 16: Ethernet MAC RX clock enable */
#endif

/* AHB peripheral clock reset register (RCC_AHBRSTR) */

#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define RCC_AHBRSTR_OTGFSRST      (1 << 12) /* USB OTG FS reset */
#  define RCC_AHBRSTR_ETHMACRST     (1 << 14) /* Ethernet MAC reset */
#endif

/* APB2 Peripheral Clock enable register */

#define RCC_APB2ENR_AFIOEN          (1 << 0)  /* Bit 0: Alternate Function I/O clock enable */
#define RCC_APB2ENR_IOPEN(n)        (1 << ((n)+2))
#define RCC_APB2ENR_IOPAEN          (1 << 2)  /* Bit 2: I/O port A clock enable */
#define RCC_APB2ENR_IOPBEN          (1 << 3)  /* Bit 3: I/O port B clock enable */
#define RCC_APB2ENR_IOPCEN          (1 << 4)  /* Bit 4: I/O port C clock enable */
#define RCC_APB2ENR_IOPDEN          (1 << 5)  /* Bit 5: I/O port D clock enable */
#define RCC_APB2ENR_IOPEEN          (1 << 6)  /* Bit 6: I/O port E clock enable */
#define RCC_APB2ENR_IOPFEN          (1 << 7)  /* Bit 7: I/O port F clock enable */
#define RCC_APB2ENR_IOPGEN          (1 << 8)  /* Bit 8: I/O port G clock enable */
#define RCC_APB2ENR_ADC1EN          (1 << 9)  /* Bit 9: ADC 1 interface clock enable */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB2ENR_ADC2EN        (1 << 10) /* Bit 10: ADC 2 interface clock enable */
#endif
#define RCC_APB2ENR_TIM1EN          (1 << 11) /* Bit 11: TIM1 Timer clock enable */
#define RCC_APB2ENR_SPI1EN          (1 << 12) /* Bit 12: SPI 1 clock enable */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB2ENR_TIM8EN        (1 << 13) /* Bit 13: TIM8 Timer clock enable */
#endif
#define RCC_APB2ENR_USART1EN        (1 << 14) /* Bit 14: USART1 clock enable */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB2ENR_ADC3EN        (1 << 15) /* Bit 14: ADC3 interface clock enable */
#else
#  define RCC_APB2ENR_TIM15EN       (1 << 16) /* Bit 16: TIM15 clock enable */
#  define RCC_APB2ENR_TIM16EN       (1 << 17) /* Bit 17: TIM16 clock enable */
#  define RCC_APB2ENR_TIM17EN       (1 << 18) /* Bit 18: TIM17 clock enable */
#endif

/* APB1 Peripheral Clock enable register */

#define RCC_APB1ENR_TIM2EN          (1 << 0)  /* Bit 0: Timer 2 clock enable */
#define RCC_APB1ENR_TIM3EN          (1 << 1)  /* Bit 1: Timer 3 clock enable */
#define RCC_APB1ENR_TIM4EN          (1 << 2)  /* Bit 2: Timer 4 clock enable */
#define RCC_APB1ENR_TIM5EN          (1 << 3)  /* Bit 3: Timer 5 clock enable */
#define RCC_APB1ENR_TIM6EN          (1 << 4)  /* Bit 4: Timer 6 clock enable */
#define RCC_APB1ENR_TIM7EN          (1 << 5)  /* Bit 5: Timer 7 clock enable */
#ifdef CONFIG_STM32_VALUELINE
#  define RCC_APB1ENR_TIM12EN       (1 << 6)  /* Bit 6: Timer 12 clock enable */
#  define RCC_APB1ENR_TIM13EN       (1 << 7)  /* Bit 7: Timer 13 clock enable */
#  define RCC_APB1ENR_TIM14EN       (1 << 8)  /* Bit 8: Timer 14 clock enable */
#endif
#define RCC_APB1ENR_WWDGEN          (1 << 11) /* Bit 11: Window Watchdog clock enable */
#define RCC_APB1ENR_SPI2EN          (1 << 14) /* Bit 14: SPI 2 clock enable */
#define RCC_APB1ENR_SPI3EN          (1 << 15) /* Bit 15: SPI 3 clock enable */
#define RCC_APB1ENR_USART2EN        (1 << 17) /* Bit 17: USART 2 clock enable */
#define RCC_APB1ENR_USART3EN        (1 << 18) /* Bit 18: USART 3 clock enable */
#define RCC_APB1ENR_UART4EN         (1 << 19) /* Bit 19: UART 4 clock enable */
#define RCC_APB1ENR_UART5EN         (1 << 20) /* Bit 20: UART 5 clock enable */
#define RCC_APB1ENR_I2C1EN          (1 << 21) /* Bit 21: I2C 1 clock enable */
#define RCC_APB1ENR_I2C2EN          (1 << 22) /* Bit 22: I2C 2 clock enable */
#ifndef CONFIG_STM32_VALUELINE
#  define RCC_APB1ENR_USBEN         (1 << 23) /* Bit 23: USB clock enable */
#  define RCC_APB1ENR_CAN1EN        (1 << 25) /* Bit 25: CAN1 clock enable */
#  define RCC_APB1ENR_CAN2EN        (1 << 26) /* Bit 25: CAN2 clock enable */
#endif
#define RCC_APB1ENR_BKPEN           (1 << 27) /* Bit 27: Backup interface clock enable */
#define RCC_APB1ENR_PWREN           (1 << 28) /* Bit 28: Power interface clock enable */
#define RCC_APB1ENR_DACEN           (1 << 29) /* Bit 29: DAC interface clock enable */
#ifdef CONFIG_STM32_VALUELINE
#  define RCC_APB1ENR_CECEN         (1 << 30) /* Bit 30: CEC clock enable */
#endif

/* Backup domain control register */

#define RCC_BDCR_BDRST              (1 << 16) /* Bit 16: Backup domain software reset */
#define RCC_BDCR_RTCEN              (1 << 15) /* Bit 15: RTC clock enable */
#define RCC_BDCR_RTCSEL_SHIFT       (8)       /* Bits 9:8: RTC clock source selection */
#define RCC_BDCR_RTCSEL_MASK        (3 << RCC_BDCR_RTCSEL_SHIFT)
#  define RCC_BDCR_RTCSEL_NOCLK     (0 << RCC_BDCR_RTCSEL_SHIFT) /* 00: No clock */
#  define RCC_BDCR_RTCSEL_LSE       (1 << RCC_BDCR_RTCSEL_SHIFT) /* 01: LSE oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_LSI       (2 << RCC_BDCR_RTCSEL_SHIFT) /* 10: LSI oscillator clock used as RTC clock */
#  define RCC_BDCR_RTCSEL_HSE       (3 << RCC_BDCR_RTCSEL_SHIFT) /* 11: HSE oscillator clock divided by 128 used as RTC clock */
#define RCC_BDCR_LSEBYP             (1 << 2)  /* Bit 2: External Low Speed oscillator Bypass */
#define RCC_BDCR_LSERDY             (1 << 1)  /* Bit 1: External Low Speed oscillator Ready */
#define RCC_BDCR_LSEON              (1 << 0)  /* Bit 0: External Low Speed oscillator enable */

/* Control/status register */

#define RCC_CSR_LSION               (1 << 0)  /* Bit 0: Internal Low Speed oscillator enable */
#define RCC_CSR_LSIRDY              (1 << 1)  /* Bit 1: Internal Low Speed oscillator Ready */
#define RCC_CSR_RMVF                (1 << 24) /* Bit 24: Remove reset flag */
#define RCC_CSR_PINRSTF             (1 << 26) /* Bit 26: PIN reset flag */
#define RCC_CSR_PORRSTF             (1 << 27) /* Bit 27: POR/PDR reset flag */
#define RCC_CSR_SFTRSTF             (1 << 28) /* Bit 28: Software Reset flag */
#define RCC_CSR_IWDGRSTF            (1 << 29) /* Bit 29: Independent Watchdog reset flag */
#define RCC_CSR_WWDGRSTF            (1 << 30) /* Bit 30: Window watchdog reset flag */
#define RCC_CSR_LPWRRSTF            (1 << 31) /* Bit 31: Low-Power reset flag */

#if defined(CONFIG_STM32_VALUELINE) || defined(CONFIG_STM32_CONNECTIVITYLINE)

/* Clock configuration register 2 (For value line and connectivity line only) */

#define RCC_CFGR2_PREDIV1_SHIFT     (0)
#define RCC_CFGR2_PREDIV1_MASK      (0x0f << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d1       (0 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d2       (1 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d3       (2 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d4       (3 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d5       (4 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d6       (5 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d7       (6 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d8       (7 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d9       (8 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d10      (9 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d11      (10 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d12      (11 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d13      (12 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d14      (13 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d15      (14 << RCC_CFGR2_PREDIV1_SHIFT)
#  define RCC_CFGR2_PREDIV1d16      (15 << RCC_CFGR2_PREDIV1_SHIFT)

#endif

#ifdef CONFIG_STM32_CONNECTIVITYLINE

#define RCC_CFGR2_PREDIV2_SHIFT     (4)
#define RCC_CFGR2_PREDIV2_MASK      (0x0f << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d1       (0 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d2       (1 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d3       (2 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d4       (3 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d5       (4 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d6       (5 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d7       (6 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d8       (7 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d9       (8 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d10      (9 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d11      (10 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d12      (11 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d13      (12 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d14      (13 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d15      (14 << RCC_CFGR2_PREDIV2_SHIFT)
#  define RCC_CFGR2_PREDIV2d16      (15 << RCC_CFGR2_PREDIV2_SHIFT)

#define RCC_CFGR2_PLL2MUL_SHIFT     (8)
#define RCC_CFGR2_PLL2MUL_MASK      (0x0f << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx8       (6 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx9       (7 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx10      (8 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx11      (9 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx12      (10 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx13      (11 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx14      (12 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx16      (14 << RCC_CFGR2_PLL2MUL_SHIFT)
#  define RCC_CFGR2_PLL2MULx20      (15 << RCC_CFGR2_PLL2MUL_SHIFT)

#define RCC_CFGR2_PLL3MUL_SHIFT     (12)
#define RCC_CFGR2_PLL3MUL_MASK      (0x0f << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx8       (6 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx9       (7 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx10      (8 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx11      (9 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx12      (10 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx13      (11 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx14      (12 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx16      (14 << RCC_CFGR2_PLL3MUL_SHIFT)
#  define RCC_CFGR2_PLL3MULx20      (15 << RCC_CFGR2_PLL3MUL_SHIFT)

#define RCC_CFGR2_PREDIV1SRC_SHIFT  (16)
#define RCC_CFGR2_PREDIV1SRC_MASK   (0x01 << RCC_CFGR2_PREDIV1SRC_SHIFT)
#  define RCC_CFGR2_PREDIV1SRC_HSE  (0 << RCC_CFGR2_PREDIV1SRC_SHIFT)
#  define RCC_CFGR2_PREDIV1SRC_PLL2 (1 << RCC_CFGR2_PREDIV1SRC_SHIFT)

#define RCC_CFGR2_I2S2SRC_SHIFT     (17)
#define RCC_CFGR2_I2S2SRC_MASK      (0x01 << RCC_CFGR2_I2S2SRC_SHIFT)
#  define RCC_CFGR2_I2S2SRC_SYSCLK  (0 << RCC_CFGR2_I2S2SRC_SHIFT)
#  define RCC_CFGR2_I2S2SRC_PLL3    (1 << RCC_CFGR2_I2S2SRC_SHIFT)

#define RCC_CFGR2_I2S3SRC_SHIFT     (17)
#define RCC_CFGR2_I2S3SRC_MASK      (0x01 << RCC_CFGR2_I2S3SRC_SHIFT)
#  define RCC_CFGR2_I2S3SRC_SYSCLK  (0 << RCC_CFGR2_I2S3SRC_SHIFT)
#  define RCC_CFGR2_I2S3SRC_PLL3    (1 << RCC_CFGR2_I2S3SRC_SHIFT)

#endif

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_RCC_H */

