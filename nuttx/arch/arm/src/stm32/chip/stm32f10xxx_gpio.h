/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f10xxx_gpio.h
 *
 *   Copyright (C) 2009, 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_GPIO_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_GPIO_H

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#define STM32_NGPIO_PORTS            ((STM32_NGPIO + 15) >> 4)

/* Register Offsets *****************************************************************/

#define STM32_GPIO_CRL_OFFSET        0x0000  /* Port configuration register low */
#define STM32_GPIO_CRH_OFFSET        0x0004  /* Port configuration register high */
#define STM32_GPIO_IDR_OFFSET        0x0008  /* Port input data register */
#define STM32_GPIO_ODR_OFFSET        0x000c  /* Port output data register */
#define STM32_GPIO_BSRR_OFFSET       0x0010  /* Port bit set/reset register */
#define STM32_GPIO_BRR_OFFSET        0x0014  /* Port bit reset register */
#define STM32_GPIO_LCKR_OFFSET       0x0018  /* Port configuration lock register */

#define STM32_AFIO_EVCR_OFFSET       0x0000  /* Event control register */
#define STM32_AFIO_MAPR_OFFSET       0x0004  /* AF remap and debug I/O configuration register */
#define STM32_AFIO_EXTICR_OFFSET(p)  (0x0008 + ((p) & 0x000c)) /* Registers are displaced by 4! */
#define STM32_AFIO_EXTICR1_OFFSET    0x0008  /* External interrupt configuration register 1 */
#define STM32_AFIO_EXTICR2_OFFSET    0x000c  /* External interrupt configuration register 2 */
#define STM32_AFIO_EXTICR3_OFFSET    0x0010  /* External interrupt configuration register 3 */
#define STM32_AFIO_EXTICR4_OFFSET    0x0014  /* External interrupt configuration register 4 */
#define STM32_AFIO_MAPR2_OFFSET      0x001c  /* AF remap and debug I/O configuration register 2 */

/* Register Addresses ***************************************************************/

#if STM32_NGPIO_PORTS > 0
#  define STM32_GPIOA_CRL            (STM32_GPIOA_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOA_CRH            (STM32_GPIOA_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOA_IDR            (STM32_GPIOA_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOA_ODR            (STM32_GPIOA_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOA_BSRR           (STM32_GPIOA_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOA_BRR            (STM32_GPIOA_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOA_LCKR           (STM32_GPIOA_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 1
#  define STM32_GPIOB_CRL            (STM32_GPIOB_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOB_CRH            (STM32_GPIOB_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOB_IDR            (STM32_GPIOB_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOB_ODR            (STM32_GPIOB_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOB_BSRR           (STM32_GPIOB_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOB_BRR            (STM32_GPIOB_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOB_LCKR           (STM32_GPIOB_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 2
#  define STM32_GPIOC_CRL            (STM32_GPIOC_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOC_CRH            (STM32_GPIOC_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOC_IDR            (STM32_GPIOC_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOC_ODR            (STM32_GPIOC_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOC_BSRR           (STM32_GPIOC_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOC_BRR            (STM32_GPIOC_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOC_LCKR           (STM32_GPIOC_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 3
#  define STM32_GPIOD_CRL            (STM32_GPIOD_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOD_CRH            (STM32_GPIOD_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOD_IDR            (STM32_GPIOD_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOD_ODR            (STM32_GPIOD_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOD_BSRR           (STM32_GPIOD_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOD_BRR            (STM32_GPIOD_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOD_LCKR           (STM32_GPIOD_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 4
#  define STM32_GPIOE_CRL            (STM32_GPIOE_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOE_CRH            (STM32_GPIOE_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOE_IDR            (STM32_GPIOE_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOE_ODR            (STM32_GPIOE_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOE_BSRR           (STM32_GPIOE_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOE_BRR            (STM32_GPIOE_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOE_LCKR           (STM32_GPIOE_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 5
#  define STM32_GPIOF_CRL            (STM32_GPIOF_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOF_CRH            (STM32_GPIOF_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOF_IDR            (STM32_GPIOF_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOF_ODR            (STM32_GPIOF_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOF_BSRR           (STM32_GPIOF_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOF_BRR            (STM32_GPIOF_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOF_LCKR           (STM32_GPIOF_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#if STM32_NGPIO_PORTS > 6
#  define STM32_GPIOG_CRL            (STM32_GPIOG_BASE+STM32_GPIO_CRL_OFFSET)
#  define STM32_GPIOG_CRH            (STM32_GPIOG_BASE+STM32_GPIO_CRH_OFFSET)
#  define STM32_GPIOG_IDR            (STM32_GPIOG_BASE+STM32_GPIO_IDR_OFFSET)
#  define STM32_GPIOG_ODR            (STM32_GPIOG_BASE+STM32_GPIO_ODR_OFFSET)
#  define STM32_GPIOG_BSRR           (STM32_GPIOG_BASE+STM32_GPIO_BSRR_OFFSET)
#  define STM32_GPIOG_BRR            (STM32_GPIOG_BASE+STM32_GPIO_BRR_OFFSET)
#  define STM32_GPIOG_LCKR           (STM32_GPIOG_BASE+STM32_GPIO_LCKR_OFFSET)
#endif

#define STM32_AFIO_EVCR              (STM32_AFIO_BASE+STM32_AFIO_EVCR_OFFSET)
#define STM32_AFIO_MAPR              (STM32_AFIO_BASE+STM32_AFIO_MAPR_OFFSET)
#define STM32_AFIO_EXTICR(p)         (STM32_AFIO_BASE+STM32_AFIO_EXTICR_OFFSET(p))
#define STM32_AFIO_EXTICR1           (STM32_AFIO_BASE+STM32_AFIO_EXTICR1_OFFSET)
#define STM32_AFIO_EXTICR2           (STM32_AFIO_BASE+STM32_AFIO_EXTICR2_OFFSET)
#define STM32_AFIO_EXTICR3           (STM32_AFIO_BASE+STM32_AFIO_EXTICR3_OFFSET)
#define STM32_AFIO_EXTICR4           (STM32_AFIO_BASE+STM32_AFIO_EXTICR4_OFFSET)

/* Register Bitfield Definitions ****************************************************/

/* Port configuration register low */

#define GPIO_CR_MODE_SHIFT(n)        ((n) << 2)
#define GPIO_CR_MODE_MASK(n)         (3 << GPIO_CR_MODE_SHIFT(n))
#define GPIO_CR_CNF_SHIFT(n)         (2 + ((n) << 2))
#define GPIO_CR_CNF_MASK(n)          (3 << GPIO_CR_CNF_SHIFT(n))

#define GPIO_CR_MODECNF_SHIFT(n)     ((n) << 2)
#define GPIO_CR_MODECNF_MASK(n)      (0x0f << GPIO_CR_MODECNF_SHIFT(n))

#define GPIO_CRL_MODE0_SHIFT         (0)     /* Bits 1:0: Port mode bits */
#define GPIO_CRL_MODE0_MASK          (3 << GPIO_CRL_MODE0_SHIFT)
#define GPIO_CRL_CNF0_SHIFT          (2)     /* Bits 3:2: Port configuration bits */
#define GPIO_CRL_CNF0_MASK           (3 << GPIO_CRL_CNF0_SHIFT)
#define GPIO_CRL_MODE1_SHIFT         (4)     /* Bits 5:4: Port mode bits */
#define GPIO_CRL_MODE1_MASK          (3 << GPIO_CRL_MODE1_SHIFT)
#define GPIO_CRL_CNF1_SHIFT          (6)     /* Bits 7:6: Port configuration bits */
#define GPIO_CRL_CNF1_MASK           (3 << GPIO_CRL_CNF1_SHIFT)
#define GPIO_CRL_MODE2_SHIFT         (8)     /* Bits 9:8: Port mode bits */
#define GPIO_CRL_MODE2_MASK          (3 << GPIO_CRL_MODE2_SHIFT)
#define GPIO_CRL_CNF2_SHIFT          (10)     /* Bits 11:10: Port configuration bits */
#define GPIO_CRL_CNF2_MASK           (3 << GPIO_CRL_CNF2_SHIFT)
#define GPIO_CRL_MODE3_SHIFT         (12)     /* Bits 13:12: Port mode bits */
#define GPIO_CRL_MODE3_MASK          (3 << GPIO_CRL_MODE3_SHIFT)
#define GPIO_CRL_CNF3_SHIFT          (14)     /* Bits 15:14: Port configuration bits */
#define GPIO_CRL_CNF3_MASK           (3 << GPIO_CRL_CNF3_SHIFT)
#define GPIO_CRL_MODE4_SHIFT         (16)     /* Bits 17:16: Port mode bits */
#define GPIO_CRL_MODE4_MASK          (3 << GPIO_CRL_MODE4_SHIFT)
#define GPIO_CRL_CNF4_SHIFT          (18)     /* Bits 19:18: Port configuration bits */
#define GPIO_CRL_CNF4_MASK           (3 << GPIO_CRL_CNF4_SHIFT)
#define GPIO_CRL_MODE5_SHIFT         (20)     /* Bits 21:20: Port mode bits */
#define GPIO_CRL_MODE5_MASK          (3 << GPIO_CRL_MODE5_SHIFT)
#define GPIO_CRL_CNF5_SHIFT          (22)     /* Bits 23:22: Port configuration bits */
#define GPIO_CRL_CNF5_MASK           (3 << GPIO_CRL_CNF5_SHIFT)
#define GPIO_CRL_MODE6_SHIFT         (24)     /* Bits 25:24: Port mode bits */
#define GPIO_CRL_MODE6_MASK          (3 << GPIO_CRL_MODE6_SHIFT)
#define GPIO_CRL_CNF6_SHIFT          (26)     /* Bits 27:26: Port configuration bits */
#define GPIO_CRL_CNF6_MASK           (3 << GPIO_CRL_CNF6_SHIFT)
#define GPIO_CRL_MODE7_SHIFT         (28)     /* Bits 29:28: Port mode bits */
#define GPIO_CRL_MODE7_MASK          (3 << GPIO_CRL_MODE7_SHIFT)
#define GPIO_CRL_CNF7_SHIFT          (30)     /* Bits 31:30: Port configuration bits */
#define GPIO_CRL_CNF7_MASK           (3 << GPIO_CRL_CNF7_SHIFT)

#define GPIO_CR_CNF_INANALOG         (0)       /* 00: Analog input mode */
#define GPIO_CR_CNF_INFLOAT          (1)       /* 01: Floating input (reset state) */
#define GPIO_CR_CNF_INPULLUD         (2)       /* 10: Input with pull-up / pull-down */

#define GPIO_CR_CNF_OUTPP            (0)       /* 00: General purpose output push-pull */
#define GPIO_CR_CNF_OUTOD            (1)       /* 01: General purpose output Open-drain */
#define GPIO_CR_CNF_ALTPP            (2)       /* 10: Alternate function output Push-pull */
#define GPIO_CR_CNF_ALTOD            (3)       /* 11: Alternate function output Open-drain */

#define GPIO_CR_MODE_INRST           (0)       /* 00: Input mode (reset state) */
#define GPIO_CR_MODE_OUT10MHz        (1)       /* 01: Output mode, max speed 10 MHz */
#define GPIO_CR_MODE_OUT2MHz         (2)       /* 10: Output mode, max speed 2 MHz */
#define GPIO_CR_MODE_OUT50MHz        (3)       /* 11: Output mode, max speed 50 MHz */

/* Port configuration register high */

#define GPIO_CRH_MODE8_SHIFT         (0)     /* Bits 1:0: Port mode bits */
#define GPIO_CRH_MODE8_MASK          (3 << GPIO_CRH_MODE8_SHIFT)
#define GPIO_CRH_CNF8_SHIFT          (2)     /* Bits 3:2: Port configuration bits */
#define GPIO_CRH_CNF8_MASK           (3 << GPIO_CRH_CNF8_SHIFT)
#define GPIO_CRH_MODE9_SHIFT         (4)     /* Bits 5:4: Port mode bits */
#define GPIO_CRH_MODE9_MASK          (3 << GPIO_CRH_MODE9_SHIFT)
#define GPIO_CRH_CNF9_SHIFT          (6)     /* Bits 7:6: Port configuration bits */
#define GPIO_CRH_CNF9_MASK           (3 << GPIO_CRH_CNF9_SHIFT)
#define GPIO_CRH_MODE10_SHIFT        (8)     /* Bits 9:8: Port mode bits */
#define GPIO_CRH_MODE10_MASK         (3 << GPIO_CRH_MODE10_SHIFT)
#define GPIO_CRH_CNF10_SHIFT         (10)     /* Bits 11:10: Port configuration bits */
#define GPIO_CRH_CNF10_MASK          (3 << GPIO_CRH_CNF10_SHIFT)
#define GPIO_CRH_MODE11_SHIFT        (12)     /* Bits 13:12: Port mode bits */
#define GPIO_CRH_MODE11_MASK         (3 << GPIO_CRH_MODE11_SHIFT)
#define GPIO_CRH_CNF11_SHIFT         (14)     /* Bits 15:14: Port configuration bits */
#define GPIO_CRH_CNF11_MASK          (3 << GPIO_CRH_CNF11_SHIFT)
#define GPIO_CRH_MODE12_SHIFT        (16)     /* Bits 17:16: Port mode bits */
#define GPIO_CRH_MODE12_MASK         (3 << GPIO_CRH_MODE12_SHIFT)
#define GPIO_CRH_CNF12_SHIFT         (18)     /* Bits 19:18: Port configuration bits */
#define GPIO_CRH_CNF12_MASK          (3 << GPIO_CRH_CNF12_SHIFT)
#define GPIO_CRH_MODE13_SHIFT        (20)     /* Bits 21:20: Port mode bits */
#define GPIO_CRH_MODE13_MASK         (3 << GPIO_CRH_MODE13_SHIFT)
#define GPIO_CRH_CNF13_SHIFT         (22)     /* Bits 23:22: Port configuration bits */
#define GPIO_CRH_CNF13_MASK          (3 << GPIO_CRH_CNF13_SHIFT)
#define GPIO_CRH_MODE14_SHIFT        (24)     /* Bits 25:24: Port mode bits */
#define GPIO_CRH_MODE14_MASK         (3 << GPIO_CRH_MODE14_SHIFT)
#define GPIO_CRH_CNF14_SHIFT         (26)     /* Bits 27:26: Port configuration bits */
#define GPIO_CRH_CNF14_MASK          (3 << GPIO_CRH_CNF14_SHIFT)
#define GPIO_CRH_MODE15_SHIFT        (28)     /* Bits 29:28: Port mode bits */
#define GPIO_CRH_MODE15_MASK         (3 << GPIO_CRH_MODE15_SHIFT)
#define GPIO_CRH_CNF15_SHIFT         (30)     /* Bits 31:30: Port configuration bits */
#define GPIO_CRL_CNF15_MASK          (3 << GPIO_CRL_CNF15_SHIFT)

/* Port input/ouput data register */

#define GPIO_IDR(n)                  (1 << (n))
#define GPIO_ODR(n)                  (1 << (n))

/* Port bit set/reset register */

#define GPIO_BSRR_RESET(n)           (1 << ((n)+16))
#define GPIO_BSRR_SET(n)             (1 << (n))
#define GPIO_BRR(n)                  (1 << (n))

/* Port configuration lock register */

#define GPIO_LCKR_LCKK               (1 << 16) /* Bit 16: Lock key */
#define GPIO_LCKR_LCK(n)             (1 << (n))

/* Event control register */

#define AFIO_EVCR_PIN_SHIFT          (0)       /* Bits 3-0: Pin selection */
#define AFIO_EVCR_PIN_MASK           (0x0f << AFIO_EVCR_PIN_SHIFT)
#define AFIO_EVCR_PORT_SHIFT         (4)       /* Bits 6-4: Port selection */
#define AFIO_EVCR_PORT_MASK          (7 << AFIO_EVCR_PORT_SHIFT)
#  define AFIO_EVCR_PORTA            (0 << AFIO_EVCR_PORT_SHIFT) /* 000: PA selected */
#  define AFIO_EVCR_PORTB            (1 << AFIO_EVCR_PORT_SHIFT) /* 001: PB selected */
#  define AFIO_EVCR_PORTC            (2 << AFIO_EVCR_PORT_SHIFT) /* 010: PC selected */
#  define AFIO_EVCR_PORTD            (3 << AFIO_EVCR_PORT_SHIFT) /* 011: PD selected */
#  define AFIO_EVCR_PORTE            (4 << AFIO_EVCR_PORT_SHIFT) /* 100: PE selected */
#define AFIO_EVCR_EVOE               (1 << 7)  /* Bit 7: Event Output Enable */

/* AF remap and debug I/O configuration register */

#define AFIO_MAPR_SPI1_REMAP         (1 << 0)  /* Bit 0: SPI1 remapping */
#define AFIO_MAPR_I2C1_REMAP         (1 << 1)  /* Bit 1: I2C1 remapping */
#define AFIO_MAPR_USART1_REMAP       (1 << 2)  /* Bit 2: USART1 remapping */
#define AFIO_MAPR_USART2_REMAP       (1 << 3)  /* Bit 3: USART2 remapping */
#define AFIO_MAPR_USART3_REMAP_SHIFT (4)       /* Bits 5-4: USART3 remapping */
#define AFIO_MAPR_USART3_REMAP_MASK  (3 << AFIO_MAPR_USART3_REMAP_SHIFT) 
#  define AFIO_MAPR_USART3_NOREMAP   (0 << AFIO_MAPR_USART3_REMAP_SHIFT) /* 00: No remap */
#  define AFIO_MAPR_USART3_PARTREMAP (1 << AFIO_MAPR_USART3_REMAP_SHIFT) /* 01: Partial remap */
#  define AFIO_MAPR_USART3_FULLREMAP (3 << AFIO_MAPR_USART3_REMAP_SHIFT) /* 11: Full remap */
#define AFIO_MAPR_TIM1_REMAP_SHIFT   (6)       /* Bits 7-6: TIM1 remapping */
#define AFIO_MAPR_TIM1_REMAP_MASK    (3 << AFIO_MAPR_TIM1_REMAP_SHIFT) 
#  define AFIO_MAPR_TIM1_NOREMAP     (0 << AFIO_MAPR_TIM1_REMAP_SHIFT) /* 00: No remap */
#  define AFIO_MAPR_TIM1_PARTREMAP   (1 << AFIO_MAPR_TIM1_REMAP_SHIFT) /* 01: Partial remap */
#  define AFIO_MAPR_TIM1_FULLREMAP   (3 << AFIO_MAPR_TIM1_REMAP_SHIFT) /* 11: Full remap */
#define AFIO_MAPR_TIM2_REMAP_SHIFT   (8)       /* Bits 9-8: TIM2 remapping */
#define AFIO_MAPR_TIM2_REMAP_MASK    (3 << AFIO_MAPR_TIM2_REMAP_SHIFT)
#  define AFIO_MAPR_TIM2_NOREMAP     (0 << AFIO_MAPR_TIM2_REMAP_SHIFT) /* 00: No remap */
#  define AFIO_MAPR_TIM2_PARTREMAP1  (1 << AFIO_MAPR_TIM2_REMAP_SHIFT) /* 01: Partial remap */
#  define AFIO_MAPR_TIM2_PARTREMAP2  (2 << AFIO_MAPR_TIM2_REMAP_SHIFT) /* 10: Partial remap */
#  define AFIO_MAPR_TIM2_FULLREMAP   (3 << AFIO_MAPR_TIM2_REMAP_SHIFT) /* 11: Full remap */
#define AFIO_MAPR_TIM3_REMAP_SHIFT   (10)      /* Bits 11-10: TIM3 remapping */
#define AFIO_MAPR_TIM3_REMAP_MASK    (3 << AFIO_MAPR_TIM3_REMAP_SHIFT)
#  define AFIO_MAPR_TIM3_NOREMAP     (0 << AFIO_MAPR_TIM3_REMAP_SHIFT) /* 00: No remap */
#  define AFIO_MAPR_TIM3_PARTREMAP   (2 << AFIO_MAPR_TIM3_REMAP_SHIFT) /* 10: Partial remap */
#  define AFIO_MAPR_TIM3_FULLREMAP   (3 << AFIO_MAPR_TIM3_REMAP_SHIFT) /* 11: Full remap */
#define AFIO_MAPR_TIM4_REMAP         (1 << 12) /* Bit 12: TIM4 remapping */
#define AFIO_MAPR_CAN1_REMAP_SHIFT   (13)      /* Bits 14-13: CAN Alternate function remapping */
#define AFIO_MAPR_CAN1_REMAP_MASK    (3 << AFIO_MAPR_CAN1_REMAP_SHIFT)
#  define AFIO_MAPR_PA1112           (0 << AFIO_MAPR_CAN1_REMAP_SHIFT) /* 00: CANRX mapped to PA11, CANTX mapped to PA12 */
#  define AFIO_MAPR_PB89             (2 << AFIO_MAPR_CAN1_REMAP_SHIFT) /* 10: CANRX mapped to PB8, CANTX mapped to PB9 */
#  define AFIO_MAPR_PD01             (3 << AFIO_MAPR_CAN1_REMAP_SHIFT) /* 11: CANRX mapped to PD0, CANTX mapped to PD1 */
#define AFIO_MAPR_PD01_REMAP         (1 << 15) /* Bit 15 : Port D0/Port D1 mapping on OSC_IN/OSC_OUT */
#define AFIO_MAPR_TIM5CH4_IREMAP     (1 << 16) /* Bit 16: TIM5 channel4 internal remap */
                                               /* Bits 17-20: Reserved */
#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define AFIO_MAPR_ETH_REMAP        (1 << 21) /* Bit 21: Ethernet MAC I/O remapping */
#  define AFIO_MAPR_CAN2_REMAP       (1 << 22) /* Bit 22: CAN2 I/O remapping */
#  define AFIO_MAPR_MII_RMII_SEL     (1 << 23) /* Bit 23: MII or RMII selection */
#endif
#define AFIO_MAPR_SWJ_CFG_SHIFT      (24)      /* Bits 26-24: Serial Wire JTAG configuration */
#define AFIO_MAPR_SWJ_CFG_MASK       (7 << AFIO_MAPR_SWJ_CFG_SHIFT)
#  define AFIO_MAPR_SWJRST           (0 << AFIO_MAPR_SWJ_CFG_SHIFT) /* 000: Full SWJ (JTAG-DP + SW-DP): Reset State */
#  define AFIO_MAPR_SWJ              (1 << AFIO_MAPR_SWJ_CFG_SHIFT) /* 001: Full SWJ (JTAG-DP + SW-DP) but without JNTRST */
#  define AFIO_MAPR_SWDP             (2 << AFIO_MAPR_SWJ_CFG_SHIFT) /* 010: JTAG-DP Disabled and SW-DP Enabled */
#  define AFIO_MAPR_DISAB            (4 << AFIO_MAPR_SWJ_CFG_SHIFT) /* 100: JTAG-DP Disabled and SW-DP Disabled */
                                               /* Bit 27: Reserved */
#ifdef CONFIG_STM32_CONNECTIVITYLINE
#  define AFIO_MAPR_SPI3_REMAP       (1 << 28) /* Bit 28: SPI3 remapping */
#  define AFIO_MAPR_TIM2ITR1_IREMAP  (1 << 29) /* Bit 29: TIM2 internal trigger 1 remapping */
#  define AFIO_MAPR_PTP_PPS_REMAP    (1 << 30) /* Bit 30: Ethernet PTP PPS remapping */
#endif
                                               /* Bit 31: Reserved */
/* External interrupt configuration register 1 */

#define AFIO_EXTICR_PORT_MASK        (0x0f)
#define AFIO_EXTICR_EXTI_SHIFT(g)    (((g) & 3) << 2)
#define AFIO_EXTICR_EXTI_MASK(g)     (AFIO_EXTICR_PORT_MASK << (AFIO_EXTICR_EXTI_SHIFT(g)))

#define AFIO_EXTICR1_EXTI0_SHIFT     (0)       /* Bits 3-0: EXTI 0 configuration */
#define AFIO_EXTICR1_EXTI0_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR1_EXTI0_SHIFT)
#define AFIO_EXTICR1_EXTI1_SHIFT     (4)       /* Bits 7-4: EXTI 1 configuration */
#define AFIO_EXTICR1_EXTI1_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR1_EXTI1_SHIFT)
#define AFIO_EXTICR1_EXTI2_SHIFT     (8)       /* Bits 11-8: EXTI 2 configuration */
#define AFIO_EXTICR1_EXTI2_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR1_EXTI2_SHIFT)
#define AFIO_EXTICR1_EXTI3_SHIFT     (12)      /* Bits 15-12: EXTI 3 configuration */
#define AFIO_EXTICR1_EXTI3_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR1_EXTI3_SHIFT)

#define AFIO_EXTICR_PORTA            (0)       /* 0000: PA[x] pin */
#define AFIO_EXTICR_PORTB            (1)       /* 0001: PB[x] pin */
#define AFIO_EXTICR_PORTC            (2)       /* 0010: PC[x] pin */
#define AFIO_EXTICR_PORTD            (3)       /* 0011: PD[x] pin */
#define AFIO_EXTICR_PORTE            (4)       /* 0100: PE[x] pin */
#define AFIO_EXTICR_PORTF            (5)       /* 0101: PF[x] pin */
#define AFIO_EXTICR_PORTG            (6)       /* 0110: PG[x] pin */

/* External interrupt configuration register 2 */

#define AFIO_EXTICR2_EXTI4_SHIFT     (0)       /* Bits 3-0: EXTI 4 configuration */
#define AFIO_EXTICR2_EXTI4_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR2_EXTI4_SHIFT)
#define AFIO_EXTICR2_EXTI5_SHIFT     (4)       /* Bits 7-4: EXTI 5 configuration */
#define AFIO_EXTICR2_EXTI5_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR2_EXTI5_SHIFT)
#define AFIO_EXTICR2_EXTI6_SHIFT     (8)       /* Bits 11-8: EXTI 6 configuration */
#define AFIO_EXTICR2_EXTI6_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR2_EXTI6_SHIFT)
#define AFIO_EXTICR2_EXTI7_SHIFT     (12)      /* Bits 15-12: EXTI 7 configuration */
#define AFIO_EXTICR2_EXTI7_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR2_EXTI7_SHIFT)

/* External interrupt configuration register 3 */

#define AFIO_EXTICR3_EXTI8_SHIFT     (0)       /* Bits 3-0: EXTI 8 configuration */
#define AFIO_EXTICR3_EXTI8_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR3_EXTI8_SHIFT)
#define AFIO_EXTICR3_EXTI9_SHIFT     (4)       /* Bits 7-4: EXTI 9 configuration */
#define AFIO_EXTICR3_EXTI9_MASK      (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR3_EXTI9_SHIFT)
#define AFIO_EXTICR3_EXTI10_SHIFT    (8)       /* Bits 11-8: EXTI 10 configuration */
#define AFIO_EXTICR3_EXTI10_MASK     (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR3_EXTI10_SHIFT)
#define AFIO_EXTICR3_EXTI11_SHIFT    (12)      /* Bits 15-12: EXTI 11 configuration */
#define AFIO_EXTICR3_EXTI11_MASK     (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR3_EXTI11_SHIFT)

/* External interrupt configuration register 4 */

#define AFIO_EXTICR4_EXTI12_SHIFT    (0)       /* Bits 3-0: EXTI 12 configuration */
#define AFIO_EXTICR4_EXTI12_MASK     (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR4_EXTI12_SHIFT)
#define AFIO_EXTICR4_EXTI13_SHIFT    (4)       /* Bits 7-4: EXTI 13 configuration */
#define AFIO_EXTICR4_EXTI13_MASK     (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR4_EXTI13_SHIFT)
#define AFIO_EXTICR4_EXTI14_SHIFT    (8)       /* Bits 11-8: EXTI 14 configuration */
#define AFIO_EXTICR4_EXTI14_MASK     (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR4_EXTI14_SHIFT)
#define AFIO_EXTICR4_EXTI15_SHIFT    (12)      /* Bits 15-12: EXTI 15 configuration */
#define AFIO_EXTICR4_EXTI15_MASK     (AFIO_EXTICR_PORT_MASK << AFIO_EXTICR4_EXTI15_SHIFT)

/* AF remap and debug I/O configuration register 2 */

#ifdef CONFIG_STM32_VALUELINE
#  define AFIO_MAPR2_TIM15_REMAP         (1 << 0)   /* Bit 0: TIM15 remapping */
#  define AFIO_MAPR2_TIM16_REMAP         (1 << 1)   /* Bit 1: TIM16 remapping */
#  define AFIO_MAPR2_TIM17_REMAP         (1 << 2)   /* Bit 2: TIM17 remapping */
#  define AFIO_MAPR2_CEC_REMAP           (1 << 3)   /* Bit 3: CEC remapping */
#  define AFIO_MAPR2_TIM1_DMA_REMAP      (1 << 4)   /* Bit 4: TIM1 DMA remapping */
#else
#  define AFIO_MAPR2_TIM9_REMAP          (1 << 5)   /* Bit 5: TIM9 remapping */
#  define AFIO_MAPR2_TIM10_REMAP         (1 << 6)   /* Bit 6: TIM10 remapping */
#  define AFIO_MAPR2_TIM11_REMAP         (1 << 7)   /* Bit 7: TIM11 remapping */
#endif
#define AFIO_MAPR2_TIM13_REMAP           (1 << 8)   /* Bit 8: TIM13 remapping */
#define AFIO_MAPR2_TIM14_REMAP           (1 << 9)   /* Bit 9: TIM14 remapping */
#define AFIO_MAPR2_FSMC_NADV             (1 << 10)  /* Bit 10: NADV connect/disconnect */
#ifdef CONFIG_STM32_VALUELINE
#  define AFIO_MAPR2_TIM67_DAC_DMA_REMAP (1 << 11)  /* Bit 11: TIM67_DAC DMA remapping */
#  define AFIO_MAPR2_TIM12_REMAP         (1 << 12)  /* Bit 12: TIM12 remapping */
#  define AFIO_MAPR2_MISC_REMAP          (1 << 13)  /* Bit 13: Miscellaneous features remapping */
#endif

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32F10XXX_GPIO_H */

