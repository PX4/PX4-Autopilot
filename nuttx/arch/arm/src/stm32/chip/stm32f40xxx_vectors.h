/************************************************************************************
 * arch/arm/src/stm32/chip/stm32f40xxx_vectors.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/

/* This file is included by stm32_vectors.S.  It provides the macro VECTOR that
 * supplies ach STM32F40xxx vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/stm32/stm32f40xxx_irq.h.
 * stm32_vectors.S will define the VECTOR macro in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve interrupt table entries for I/O interrupts. */

#  ifdef CONFIG_STM32_STM32F427
#    define ARMV7M_PERIPHERAL_INTERRUPTS 87
#  else
#    define ARMV7M_PERIPHERAL_INTERRUPTS 82
#  endif

#else

VECTOR(stm32_wwdg, STM32_IRQ_WWDG)               /* Vector 16+0:  Window Watchdog interrupt */
VECTOR(stm32_pvd, STM32_IRQ_PVD)                 /* Vector 16+1:  PVD through EXTI Line detection interrupt */
VECTOR(stm32_tamper, STM32_IRQ_TAMPER)           /* Vector 16+2:  Tamper and time stamp interrupts */
VECTOR(stm32_rtc_wkup, STM32_IRQ_RTC_WKUP)       /* Vector 16+3:  RTC global interrupt */
VECTOR(stm32_flash, STM32_IRQ_FLASH)             /* Vector 16+4:  Flash global interrupt */
VECTOR(stm32_rcc, STM32_IRQ_RCC)                 /* Vector 16+5:  RCC global interrupt */
VECTOR(stm32_exti0, STM32_IRQ_EXTI0)             /* Vector 16+6:  EXTI Line 0 interrupt */
VECTOR(stm32_exti1, STM32_IRQ_EXTI1)             /* Vector 16+7:  EXTI Line 1 interrupt */
VECTOR(stm32_exti2, STM32_IRQ_EXTI2)             /* Vector 16+8:  EXTI Line 2 interrupt */
VECTOR(stm32_exti3, STM32_IRQ_EXTI3)             /* Vector 16+9:  EXTI Line 3 interrupt */
VECTOR(stm32_exti4, STM32_IRQ_EXTI4)             /* Vector 16+10: EXTI Line 4 interrupt */
VECTOR(stm32_dma1s0, STM32_IRQ_DMA1S0)           /* Vector 16+11: DMA1 Stream 0 global interrupt */
VECTOR(stm32_dma1s1, STM32_IRQ_DMA1S1)           /* Vector 16+12: DMA1 Stream 1 global interrupt */
VECTOR(stm32_dma1s2, STM32_IRQ_DMA1S2)           /* Vector 16+13: DMA1 Stream 2 global interrupt */
VECTOR(stm32_dma1s3, STM32_IRQ_DMA1S3)           /* Vector 16+14: DMA1 Stream 3 global interrupt */
VECTOR(stm32_dma1s4, STM32_IRQ_DMA1S4)           /* Vector 16+15: DMA1 Stream 4 global interrupt */
VECTOR(stm32_dma1s5, STM32_IRQ_DMA1S5)           /* Vector 16+16: DMA1 Stream 5 global interrupt */
VECTOR(stm32_dma1s6, STM32_IRQ_DMA1S6)           /* Vector 16+17: DMA1 Stream 6 global interrupt */
VECTOR(stm32_adc, STM32_IRQ_ADC)                 /* Vector 16+18: ADC1, ADC2, and ADC3 global interrupt */
VECTOR(stm32_can1tx, STM32_IRQ_CAN1TX)           /* Vector 16+19: CAN1 TX interrupts */
VECTOR(stm32_can1rx0, STM32_IRQ_CAN1RX0)         /* Vector 16+20: CAN1 RX0 interrupts */
VECTOR(stm32_can1rx1, STM32_IRQ_CAN1RX1)         /* Vector 16+21: CAN1 RX1 interrupt */
VECTOR(stm32_can1sce, STM32_IRQ_CAN1SCE)         /* Vector 16+22: CAN1 SCE interrupt */
VECTOR(stm32_exti95, STM32_IRQ_EXTI95)           /* Vector 16+23: EXTI Line[9:5] interrupts */
VECTOR(stm32_tim1brk, STM32_IRQ_TIM1BRK)         /* Vector 16+24: TIM1 Break interrupt/TIM9 global interrupt */
VECTOR(stm32_tim1up, STM32_IRQ_TIM1UP)           /* Vector 16+25: TIM1 Update interrupt/TIM10 global interrupt */
VECTOR(stm32_tim1trgcom, STM32_IRQ_TIM1TRGCOM)   /* Vector 16+26: TIM1 Trigger and Commutation interrupts/TIM11 global interrupt */
VECTOR(stm32_tim1cc, STM32_IRQ_TIM1CC)           /* Vector 16+27: TIM1 Capture Compare interrupt */
VECTOR(stm32_tim2, STM32_IRQ_TIM2)               /* Vector 16+28: TIM2 global interrupt */
VECTOR(stm32_tim3, STM32_IRQ_TIM3)               /* Vector 16+29: TIM3 global interrupt */
VECTOR(stm32_tim4, STM32_IRQ_TIM4)               /* Vector 16+30: TIM4 global interrupt */
VECTOR(stm32_i2c1ev, STM32_IRQ_I2C1EV)           /* Vector 16+31: I2C1 event interrupt */
VECTOR(stm32_i2c1er, STM32_IRQ_I2C1ER)           /* Vector 16+32: I2C1 error interrupt */
VECTOR(stm32_i2c2ev, STM32_IRQ_I2C2EV)           /* Vector 16+33: I2C2 event interrupt */
VECTOR(stm32_i2c2er, STM32_IRQ_I2C2ER)           /* Vector 16+34: I2C2 error interrupt */
VECTOR(stm32_spi1, STM32_IRQ_SPI1)               /* Vector 16+35: SPI1 global interrupt */
VECTOR(stm32_spi2, STM32_IRQ_SPI2)               /* Vector 16+36: SPI2 global interrupt */
VECTOR(stm32_usart1, STM32_IRQ_USART1)           /* Vector 16+37: USART1 global interrupt */
VECTOR(stm32_usart2, STM32_IRQ_USART2)           /* Vector 16+38: USART2 global interrupt */
VECTOR(stm32_usart3, STM32_IRQ_USART3)           /* Vector 16+39: USART3 global interrupt */
VECTOR(stm32_exti1510, STM32_IRQ_EXTI1510)       /* Vector 16+40: EXTI Line[15:10] interrupts */
VECTOR(stm32_rtcalrm, STM32_IRQ_RTCALRM)         /* Vector 16+41: RTC alarm through EXTI line interrupt */
VECTOR(stm32_otgfswkup, STM32_IRQ_OTGFSWKUP)     /* Vector 16+42: USB On-The-Go FS Wakeup through EXTI line interrupt */
VECTOR(stm32_tim8brk, STM32_IRQ_TIM8BRK)         /* Vector 16+43: TIM8 Break interrupt/TIM12 global interrupt */
VECTOR(stm32_tim8up, STM32_IRQ_TIM8UP)           /* Vector 16+44: TIM8 Update interrup/TIM13 global interrupt */
VECTOR(stm32_tim8trgcom, STM32_IRQ_TIM8TRGCOM)   /* Vector 16+45: TIM8 Trigger and Commutation interrupts/TIM14 global interrupt */
VECTOR(stm32_tim8cc, STM32_IRQ_TIM8CC)           /* Vector 16+46: TIM8 Capture Compare interrupt */
VECTOR(stm32_dma1s7, STM32_IRQ_DMA1S7)           /* Vector 16+47: DMA1 Stream 7 global interrupt */
VECTOR(stm32_fsmc, STM32_IRQ_FSMC)               /* Vector 16+48: FSMC global interrupt */
VECTOR(stm32_sdio, STM32_IRQ_SDIO)               /* Vector 16+49: SDIO global interrupt */
VECTOR(stm32_tim5, STM32_IRQ_TIM5)               /* Vector 16+50: TIM5 global interrupt */
VECTOR(stm32_spi3, STM32_IRQ_SPI3)               /* Vector 16+51: SPI3 global interrupt */
VECTOR(stm32_uart4, STM32_IRQ_UART4)             /* Vector 16+52: UART4 global interrupt */
VECTOR(stm32_uart5, STM32_IRQ_UART5)             /* Vector 16+53: UART5 global interrupt */
VECTOR(stm32_tim6, STM32_IRQ_TIM6)               /* Vector 16+54: TIM6 global interrupt/DAC1 and DAC2 underrun error interrupts */
VECTOR(stm32_tim7, STM32_IRQ_TIM7)               /* Vector 16+55: TIM7 global interrupt */
VECTOR(stm32_dma2s0, STM32_IRQ_DMA2S0)           /* Vector 16+56: DMA2 Stream 0 global interrupt */
VECTOR(stm32_dma2s1, STM32_IRQ_DMA2S1)           /* Vector 16+57: DMA2 Stream 1 global interrupt */
VECTOR(stm32_dma2s2, STM32_IRQ_DMA2S2)           /* Vector 16+58: DMA2 Stream 2 global interrupt */
VECTOR(stm32_dma2s3, STM32_IRQ_DMA2S3)           /* Vector 16+59: DMA2 Stream 3 global interrupt */
VECTOR(stm32_dma2s4, STM32_IRQ_DMA2S4)           /* Vector 16+60: DMA2 Stream 4 global interrupt */
VECTOR(stm32_eth, STM32_IRQ_ETH)                 /* Vector 16+61: Ethernet global interrupt */
VECTOR(stm32_ethwkup, STM32_IRQ_ETHWKUP)         /* Vector 16+62: Ethernet Wakeup through EXTI line interrupt */
VECTOR(stm32_can2tx, STM32_IRQ_CAN2TX)           /* Vector 16+63: CAN2 TX interrupts */
VECTOR(stm32_can2rx0, STM32_IRQ_CAN2RX0)         /* Vector 16+64: CAN2 RX0 interrupts */
VECTOR(stm32_can2rx1, STM32_IRQ_CAN2RX1)         /* Vector 16+65: CAN2 RX1 interrupt */
VECTOR(stm32_can2sce, STM32_IRQ_CAN2SCE)         /* Vector 16+66: CAN2 SCE interrupt */
VECTOR(stm32_otgfs, STM32_IRQ_OTGFS)             /* Vector 16+67: USB On The Go FS global interrupt */
VECTOR(stm32_dma2s5, STM32_IRQ_DMA2S5)           /* Vector 16+68: DMA2 Stream 5 global interrupt */
VECTOR(stm32_dma2s6, STM32_IRQ_DMA2S6)           /* Vector 16+69: DMA2 Stream 6 global interrupt */
VECTOR(stm32_dma2s7, STM32_IRQ_DMA2S7)           /* Vector 16+70: DMA2 Stream 7 global interrupt */
VECTOR(stm32_usart6, STM32_IRQ_USART6)           /* Vector 16+71: USART6 global interrupt */
VECTOR(stm32_i2c3ev, STM32_IRQ_I2C3EV)           /* Vector 16+72: I2C3 event interrupt */
VECTOR(stm32_i2c3er, STM32_IRQ_I2C3ER)           /* Vector 16+73: I2C3 error interrupt */
VECTOR(stm32_otghsep1out, STM32_IRQ_OTGHSEP1OUT) /* Vector 16+74: USB On The Go HS End Point 1 Out global interrupt */
VECTOR(stm32_otghsep1in, STM32_IRQ_OTGHSEP1IN)   /* Vector 16+75: USB On The Go HS End Point 1 In global interrupt */
VECTOR(stm32_otghswkup, STM32_IRQ_OTGHSWKUP)     /* Vector 16+76: USB On The Go HS Wakeup through EXTI interrupt */
VECTOR(stm32_otghs, STM32_IRQ_OTGHS)             /* Vector 16+77: USB On The Go HS global interrupt */
VECTOR(stm32_dcmi, STM32_IRQ_DCMI)               /* Vector 16+78: DCMI global interrupt */
VECTOR(stm32_cryp, STM32_IRQ_CRYP)               /* Vector 16+79: CRYP crypto global interrupt */
VECTOR(stm32_hash, STM32_IRQ_HASH)               /* Vector 16+80: Hash and Rng global interrupt */
VECTOR(stm32_fpu, STM32_IRQ_FPU)                 /* Vector 16+81: FPU global interrupt */

#ifdef CONFIG_STM32_STM32F427
VECTOR(stm32_uart7, STM32_IRQ_UART7)             /* Vector 16+82: UART7 interrupt */
VECTOR(stm32_uart8, STM32_IRQ_UART8)             /* Vector 16+83: UART8 interrupt */
VECTOR(stm32_spi4, STM32_IRQ_SPI4)               /* Vector 16+84: SPI4 interrupt */
VECTOR(stm32_spi5, STM32_IRQ_SPI5)               /* Vector 16+85: SPI5 interrupt */
VECTOR(stm32_spi6, STM32_IRQ_SPI6)               /* Vector 16+86: SPI6 interrupt */
#endif

#endif /* CONFIG_ARMV7M_CMNVECTOR */
