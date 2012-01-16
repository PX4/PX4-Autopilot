/****************************************************************************************************
 * arch/arm/src/stm32/chip/stm32_tim.h
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
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_STM32_CHIP_STM32_TIM_H
#define __ARCH_ARM_SRC_STM32_CHIP_STM32_TIM_H

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* Register Offsets *********************************************************************************/

/* Basic Timers - TIM6 and TIM7 */

#define STM32_BTIM_CR1_OFFSET     0x0000  /* Control register 1 (16-bit) */
#define STM32_BTIM_CR2_OFFSET     0x0004  /* Control register 2 (16-bit) */
#define STM32_BTIM_DIER_OFFSET    0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_BTIM_SR_OFFSET      0x0010  /* Status register (16-bit) */
#define STM32_BTIM_EGR_OFFSET     0x0014  /* Event generation register (16-bit) */
#define STM32_BTIM_CNT_OFFSET     0x0024  /* Counter (16-bit) */
#define STM32_BTIM_PSC_OFFSET     0x0028  /* Prescaler (16-bit) */
#define STM32_BTIM_ARR_OFFSET     0x002c  /* Auto-reload register (16-bit) */

/* 16-/32-bit General Timers with DMA: TIM2, TM3, TIM4, and TIM5
 * 16-bit General Timers without DMA: TIM9, TIM10, TIM11, TIM12, TIM13, and TIM14
 * For the STM32F10xx all timers are 16-bit.
 * For the STM32F40xx, TIM2 and 5 are 32-bit
 */

#define STM32_GTIM_CR1_OFFSET     0x0000  /* Control register 1 (16-bit) */
#define STM32_GTIM_CR2_OFFSET     0x0004  /* Control register 2 (16-bit, TIM2-5 only) */
#define STM32_GTIM_SMCR_OFFSET    0x0008  /* Slave mode control register (16-bit, TIM2-5 only) */
#define STM32_GTIM_DIER_OFFSET    0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_GTIM_SR_OFFSET      0x0010  /* Status register (16-bit) */
#define STM32_GTIM_EGR_OFFSET     0x0014  /* Event generation register (16-bit) */
#define STM32_GTIM_CCMR1_OFFSET   0x0018  /* Capture/compare mode register 1 (16-bit) */
#define STM32_GTIM_CCMR2_OFFSET   0x001c  /* Capture/compare mode register 2 (16-bit, TIM2-5 only) */
#define STM32_GTIM_CCER_OFFSET    0x0020  /* Capture/compare enable register (16-bit) */
#define STM32_GTIM_CNT_OFFSET     0x0024  /* Counter (16-bit or 32-bit STM3240 TIM2 and 5 only) */
#define STM32_GTIM_PSC_OFFSET     0x0028  /* Prescaler (16-bit) */
#define STM32_GTIM_ARR_OFFSET     0x002c  /* Auto-reload register (16-bit) */
#define STM32_GTIM_CCR1_OFFSET    0x0034  /* Capture/compare register 1 (16-bit or 32-bit STM3240 TIM2/5 only) */
#define STM32_GTIM_CCR2_OFFSET    0x0038  /* Capture/compare register 2 (16-bit TIM2-5 only or 32-bit STM3240 TIM2/5 only) */
#define STM32_GTIM_CCR3_OFFSET    0x003c  /* Capture/compare register 3 (16-bit TIM2-5 only or 32-bit STM3240 TIM2/5 only) */
#define STM32_GTIM_CCR4_OFFSET    0x0040  /* Capture/compare register 4 (16-bit TIM2-5 only or 32-bit STM3240 TIM2/5 only) */
#define STM32_GTIM_DCR_OFFSET     0x0048  /* DMA control register (16-bit, TIM2-5 only) */
#define STM32_GTIM_DMAR_OFFSET    0x004c  /* DMA address for burst mode (16-bit, TIM2-5 only) */

#ifdef CONFIG_STM32_STM32F40XX
#  define STM32_GTIM_OR_OFFSET    0x0050  /* Timer 2/5/11 option register */
#endif

/* Advanced Timers - TIM1 and TIM8 */

#define STM32_ATIM_CR1_OFFSET     0x0000  /* Control register 1 (16-bit) */
#define STM32_ATIM_CR2_OFFSET     0x0004  /* Control register 2 *(16-bit) */
#define STM32_ATIM_SMCR_OFFSET    0x0008  /* Slave mode control register (16-bit) */
#define STM32_ATIM_DIER_OFFSET    0x000c  /* DMA/Interrupt enable register (16-bit) */
#define STM32_ATIM_SR_OFFSET      0x0010  /* Status register (16-bit) */
#define STM32_ATIM_EGR_OFFSET     0x0014  /* Event generation register (16-bit) */
#define STM32_ATIM_CCMR1_OFFSET   0x0018  /* Capture/compare mode register 1 (16-bit) */
#define STM32_ATIM_CCMR2_OFFSET   0x001c  /* Capture/compare mode register 2 (16-bit) */
#define STM32_ATIM_CCER_OFFSET    0x0020  /* Capture/compare enable register (16-bit) */
#define STM32_ATIM_CNT_OFFSET     0x0024  /* Counter (16-bit) */
#define STM32_ATIM_PSC_OFFSET     0x0028  /* Prescaler (16-bit) */
#define STM32_ATIM_ARR_OFFSET     0x002c  /* Auto-reload register (16-bit) */
#define STM32_ATIM_RCR_OFFSET     0x0030  /* Repetition counter register (16-bit) */
#define STM32_ATIM_CCR1_OFFSET    0x0034  /* Capture/compare register 1 (16-bit) */
#define STM32_ATIM_CCR2_OFFSET    0x0038  /* Capture/compare register 2 (16-bit) */
#define STM32_ATIM_CCR3_OFFSET    0x003c  /* Capture/compare register 3 (16-bit) */
#define STM32_ATIM_CCR4_OFFSET    0x0040  /* Capture/compare register 4 (16-bit) */
#define STM32_ATIM_BDTR_OFFSET    0x0044  /* Break and dead-time register (16-bit) */
#define STM32_ATIM_DCR_OFFSET     0x0048  /* DMA control register (16-bit) */
#define STM32_ATIM_DMAR_OFFSET    0x004c  /* DMA address for burst mode (16-bit) */

/* Register Addresses *******************************************************************************/

/* Advanced Timers - TIM1 and TIM8 */

#if STM32_NATIM > 0
#  define STM32_TIM1_CR1         (STM32_TIM1_BASE+STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM1_CR2         (STM32_TIM1_BASE+STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM1_SMCR        (STM32_TIM1_BASE+STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM1_DIER        (STM32_TIM1_BASE+STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM1_SR          (STM32_TIM1_BASE+STM32_ATIM_SR_OFFSET)
#  define STM32_TIM1_EGR         (STM32_TIM1_BASE+STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM1_CCMR1       (STM32_TIM1_BASE+STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM1_CCMR2       (STM32_TIM1_BASE+STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM1_CCER        (STM32_TIM1_BASE+STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM1_CNT         (STM32_TIM1_BASE+STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM1_PSC         (STM32_TIM1_BASE+STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM1_ARR         (STM32_TIM1_BASE+STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM1_RCR         (STM32_TIM1_BASE+STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM1_CCR1        (STM32_TIM1_BASE+STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM1_CCR2        (STM32_TIM1_BASE+STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM1_CCR3        (STM32_TIM1_BASE+STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM1_CCR4        (STM32_TIM1_BASE+STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM1_BDTR        (STM32_TIM1_BASE+STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM1_DCR         (STM32_TIM1_BASE+STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM1_DMAR        (STM32_TIM1_BASE+STM32_ATIM_DMAR_OFFSET)
#endif

#if STM32_NATIM > 1
#  define STM32_TIM8_CR1         (STM32_TIM8_BASE+STM32_ATIM_CR1_OFFSET)
#  define STM32_TIM8_CR2         (STM32_TIM8_BASE+STM32_ATIM_CR2_OFFSET)
#  define STM32_TIM8_SMCR        (STM32_TIM8_BASE+STM32_ATIM_SMCR_OFFSET)
#  define STM32_TIM8_DIER        (STM32_TIM8_BASE+STM32_ATIM_DIER_OFFSET)
#  define STM32_TIM8_SR          (STM32_TIM8_BASE+STM32_ATIM_SR_OFFSET)
#  define STM32_TIM8_EGR         (STM32_TIM8_BASE+STM32_ATIM_EGR_OFFSET)
#  define STM32_TIM8_CCMR1       (STM32_TIM8_BASE+STM32_ATIM_CCMR1_OFFSET)
#  define STM32_TIM8_CCMR2       (STM32_TIM8_BASE+STM32_ATIM_CCMR2_OFFSET)
#  define STM32_TIM8_CCER        (STM32_TIM8_BASE+STM32_ATIM_CCER_OFFSET)
#  define STM32_TIM8_CNT         (STM32_TIM8_BASE+STM32_ATIM_CNT_OFFSET)
#  define STM32_TIM8_PSC         (STM32_TIM8_BASE+STM32_ATIM_PSC_OFFSET)
#  define STM32_TIM8_ARR         (STM32_TIM8_BASE+STM32_ATIM_ARR_OFFSET)
#  define STM32_TIM8_RCR         (STM32_TIM8_BASE+STM32_ATIM_RCR_OFFSET)
#  define STM32_TIM8_CCR1        (STM32_TIM8_BASE+STM32_ATIM_CCR1_OFFSET)
#  define STM32_TIM8_CCR2        (STM32_TIM8_BASE+STM32_ATIM_CCR2_OFFSET)
#  define STM32_TIM8_CCR3        (STM32_TIM8_BASE+STM32_ATIM_CCR3_OFFSET)
#  define STM32_TIM8_CCR4        (STM32_TIM8_BASE+STM32_ATIM_CCR4_OFFSET)
#  define STM32_TIM8_BDTR        (STM32_TIM8_BASE+STM32_ATIM_BDTR_OFFSET)
#  define STM32_TIM8_DCR         (STM32_TIM8_BASE+STM32_ATIM_DCR_OFFSET)
#  define STM32_TIM8_DMAR        (STM32_TIM8_BASE+STM32_ATIM_DMAR_OFFSET)
#endif

/* 16-/32-bit General Timers - TIM2, TIM3, TIM4, and TIM5 with DMA.
 * For the STM32F10xx all timers are 16-bit.
 * For the STM32F40xx, TIM2 and 5 are 32-bit
 */

#if STM32_NGTIM > 0
#  define STM32_TIM2_CR1         (STM32_TIM2_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM2_CR2         (STM32_TIM2_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM2_SMCR        (STM32_TIM2_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM2_DIER        (STM32_TIM2_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM2_SR          (STM32_TIM2_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM2_EGR         (STM32_TIM2_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM2_CCMR1       (STM32_TIM2_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM2_CCMR2       (STM32_TIM2_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM2_CCER        (STM32_TIM2_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM2_CNT         (STM32_TIM2_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM2_PSC         (STM32_TIM2_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM2_ARR         (STM32_TIM2_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM2_CCR1        (STM32_TIM2_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM2_CCR2        (STM32_TIM2_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM2_CCR3        (STM32_TIM2_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM2_CCR4        (STM32_TIM2_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM2_DCR         (STM32_TIM2_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM2_DMAR        (STM32_TIM2_BASE+STM32_GTIM_DMAR_OFFSET)
#  ifdef CONFIG_STM32_STM32F40XX
#    define STM32_TIM2_OR        (STM32_TIM2_BASE+STM32_GTIM_OR_OFFSET)
#  endif
#endif

#if STM32_NGTIM > 1
#  define STM32_TIM3_CR1         (STM32_TIM3_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM3_CR2         (STM32_TIM3_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM3_SMCR        (STM32_TIM3_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM3_DIER        (STM32_TIM3_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM3_SR          (STM32_TIM3_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM3_EGR         (STM32_TIM3_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM3_CCMR1       (STM32_TIM3_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM3_CCMR2       (STM32_TIM3_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM3_CCER        (STM32_TIM3_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM3_CNT         (STM32_TIM3_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM3_PSC         (STM32_TIM3_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM3_ARR         (STM32_TIM3_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM3_CCR1        (STM32_TIM3_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM3_CCR2        (STM32_TIM3_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM3_CCR3        (STM32_TIM3_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM3_CCR4        (STM32_TIM3_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM3_DCR         (STM32_TIM3_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM3_DMAR        (STM32_TIM3_BASE+STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 2
#  define STM32_TIM4_CR1         (STM32_TIM4_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM4_CR2         (STM32_TIM4_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM4_SMCR        (STM32_TIM4_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM4_DIER        (STM32_TIM4_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM4_SR          (STM32_TIM4_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM4_EGR         (STM32_TIM4_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM4_CCMR1       (STM32_TIM4_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM4_CCMR2       (STM32_TIM4_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM4_CCER        (STM32_TIM4_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM4_CNT         (STM32_TIM4_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM4_PSC         (STM32_TIM4_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM4_ARR         (STM32_TIM4_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM4_CCR1        (STM32_TIM4_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM4_CCR2        (STM32_TIM4_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM4_CCR3        (STM32_TIM4_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM4_CCR4        (STM32_TIM4_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM4_DCR         (STM32_TIM4_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM4_DMAR        (STM32_TIM4_BASE+STM32_GTIM_DMAR_OFFSET)
#endif

#if STM32_NGTIM > 3
#  define STM32_TIM5_CR1         (STM32_TIM5_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM5_CR2         (STM32_TIM5_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM5_SMCR        (STM32_TIM5_BASE+STM32_GTIM_SMCR_OFFSET)
#  define STM32_TIM5_DIER        (STM32_TIM5_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM5_SR          (STM32_TIM5_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM5_EGR         (STM32_TIM5_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM5_CCMR1       (STM32_TIM5_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM5_CCMR2       (STM32_TIM5_BASE+STM32_GTIM_CCMR2_OFFSET)
#  define STM32_TIM5_CCER        (STM32_TIM5_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM5_CNT         (STM32_TIM5_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM5_PSC         (STM32_TIM5_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM5_ARR         (STM32_TIM5_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM5_CCR1        (STM32_TIM5_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM5_CCR2        (STM32_TIM5_BASE+STM32_GTIM_CCR2_OFFSET)
#  define STM32_TIM5_CCR3        (STM32_TIM5_BASE+STM32_GTIM_CCR3_OFFSET)
#  define STM32_TIM5_CCR4        (STM32_TIM5_BASE+STM32_GTIM_CCR4_OFFSET)
#  define STM32_TIM5_DCR         (STM32_TIM5_BASE+STM32_GTIM_DCR_OFFSET)
#  define STM32_TIM5_DMAR        (STM32_TIM5_BASE+STM32_GTIM_DMAR_OFFSET)
#  ifdef CONFIG_STM32_STM32F40XX
#    define STM32_TIM5_OR        (STM32_TIM5_BASE+STM32_GTIM_OR_OFFSET)
#  endif
#endif

/* 16-bit General Timers - TIM9-14 without DMA.  Note that (1) these timers
 * support only a subset of the general timer registers are supported, and
 * (2) TIM9 and TIM12 differ from the others.
 */

#if STM32_NGTIMNDMA > 0
#  define STM32_TIM9_CR1         (STM32_TIM9_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM9_CR2         (STM32_TIM9_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM9_DIER        (STM32_TIM9_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM9_SR          (STM32_TIM9_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM9_EGR         (STM32_TIM9_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM9_CCMR1       (STM32_TIM9_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM9_CCER        (STM32_TIM9_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM9_CNT         (STM32_TIM9_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM9_PSC         (STM32_TIM9_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM9_ARR         (STM32_TIM9_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM9_CCR1        (STM32_TIM9_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM9_CCR2        (STM32_TIM9_BASE+STM32_GTIM_CCR2_OFFSET)
#endif

#if STM32_NGTIMNDMA > 1
#  define STM32_TIM10_CR1         (STM32_TIM10_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM10_DIER        (STM32_TIM10_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM10_SR          (STM32_TIM10_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM10_EGR         (STM32_TIM10_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM10_CCMR1       (STM32_TIM10_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM10_CCER        (STM32_TIM10_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM10_CNT         (STM32_TIM10_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM10_PSC         (STM32_TIM10_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM10_ARR         (STM32_TIM10_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM10_CCR1        (STM32_TIM10_BASE+STM32_GTIM_CCR1_OFFSET)
#endif

#if STM32_NGTIMNDMA > 2
#  define STM32_TIM11_CR1         (STM32_TIM11_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM11_DIER        (STM32_TIM11_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM11_SR          (STM32_TIM11_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM11_EGR         (STM32_TIM11_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM11_CCMR1       (STM32_TIM11_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM11_CCER        (STM32_TIM11_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM11_CNT         (STM32_TIM11_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM11_PSC         (STM32_TIM11_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM11_ARR         (STM32_TIM11_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM11_CCR1        (STM32_TIM11_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM11_OR        (STM32_TIM11_BASE+STM32_GTIM_OR_OFFSET)
#endif

#if STM32_NGTIMNDMA > 3
#  define STM32_TIM12_CR1         (STM32_TIM12_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM12_CR2         (STM32_TIM9_BASE+STM32_GTIM_CR2_OFFSET)
#  define STM32_TIM12_DIER        (STM32_TIM12_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM12_SR          (STM32_TIM12_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM12_EGR         (STM32_TIM12_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM12_CCMR1       (STM32_TIM12_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM12_CCER        (STM32_TIM12_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM12_CNT         (STM32_TIM12_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM12_PSC         (STM32_TIM12_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM12_ARR         (STM32_TIM12_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM12_CCR1        (STM32_TIM12_BASE+STM32_GTIM_CCR1_OFFSET)
#  define STM32_TIM12_CCR2        (STM32_TIM12_BASE+STM32_GTIM_CCR2_OFFSET)
#endif

#if STM32_NGTIMNDMA > 4
#  define STM32_TIM13_CR1         (STM32_TIM13_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM13_DIER        (STM32_TIM13_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM13_SR          (STM32_TIM13_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM13_EGR         (STM32_TIM13_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM13_CCMR1       (STM32_TIM13_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM13_CCER        (STM32_TIM13_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM13_CNT         (STM32_TIM13_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM13_PSC         (STM32_TIM13_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM13_ARR         (STM32_TIM13_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM13_CCR1        (STM32_TIM13_BASE+STM32_GTIM_CCR1_OFFSET)
#endif

#if STM32_NGTIMNDMA > 5
#  define STM32_TIM14_CR1         (STM32_TIM14_BASE+STM32_GTIM_CR1_OFFSET)
#  define STM32_TIM14_DIER        (STM32_TIM14_BASE+STM32_GTIM_DIER_OFFSET)
#  define STM32_TIM14_SR          (STM32_TIM14_BASE+STM32_GTIM_SR_OFFSET)
#  define STM32_TIM14_EGR         (STM32_TIM14_BASE+STM32_GTIM_EGR_OFFSET)
#  define STM32_TIM14_CCMR1       (STM32_TIM14_BASE+STM32_GTIM_CCMR1_OFFSET)
#  define STM32_TIM14_CCER        (STM32_TIM14_BASE+STM32_GTIM_CCER_OFFSET)
#  define STM32_TIM14_CNT         (STM32_TIM14_BASE+STM32_GTIM_CNT_OFFSET)
#  define STM32_TIM14_PSC         (STM32_TIM14_BASE+STM32_GTIM_PSC_OFFSET)
#  define STM32_TIM14_ARR         (STM32_TIM14_BASE+STM32_GTIM_ARR_OFFSET)
#  define STM32_TIM14_CCR1        (STM32_TIM14_BASE+STM32_GTIM_CCR1_OFFSET)
#endif

/* Basic Timers - TIM6 and TIM7 */

#if STM32_NBTIM > 0
#  define STM32_TIM6_CR1         (STM32_TIM6_BASE+STM32_BTIM_CR1_OFFSET)
#  define STM32_TIM6_CR2         (STM32_TIM6_BASE+STM32_BTIM_CR2_OFFSET)
#  define STM32_TIM6_DIER        (STM32_TIM6_BASE+STM32_BTIM_DIER_OFFSET)
#  define STM32_TIM6_SR          (STM32_TIM6_BASE+STM32_BTIM_SR_OFFSET)
#  define STM32_TIM6_EGR         (STM32_TIM6_BASE+STM32_BTIM_EGR_OFFSET)
#  define STM32_TIM6_CNT         (STM32_TIM6_BASE+STM32_BTIM_CNT_OFFSET)
#  define STM32_TIM6_PSC         (STM32_TIM6_BASE+STM32_BTIM_PSC_OFFSET)
#  define STM32_TIM6_ARR         (STM32_TIM6_BASE+STM32_BTIM_ARR_OFFSET)
#endif

#if STM32_NBTIM > 1
#  define STM32_TIM7_CR1         (STM32_TIM7_BASE+STM32_BTIM_CR1_OFFSET)
#  define STM32_TIM7_CR2         (STM32_TIM7_BASE+STM32_BTIM_CR2_OFFSET)
#  define STM32_TIM7_DIER        (STM32_TIM7_BASE+STM32_BTIM_DIER_OFFSET)
#  define STM32_TIM7_SR          (STM32_TIM7_BASE+STM32_BTIM_SR_OFFSET)
#  define STM32_TIM7_EGR         (STM32_TIM7_BASE+STM32_BTIM_EGR_OFFSET)
#  define STM32_TIM7_CNT         (STM32_TIM7_BASE+STM32_BTIM_CNT_OFFSET)
#  define STM32_TIM7_PSC         (STM32_TIM7_BASE+STM32_BTIM_PSC_OFFSET)
#  define STM32_TIM7_ARR         (STM32_TIM7_BASE+STM32_BTIM_ARR_OFFSET)
#endif

/* Register Bitfield Definitions ********************************************************************/

/* Control register 1 */

#define ATIM_CR1_CEN               (1 << 0)  /* Bit 0: Counter enable */
#define ATIM_CR1_UDIS              (1 << 1)  /* Bit 1: Update disable */
#define ATIM_CR1_URS               (1 << 2)  /* Bit 2: Update request source */
#define ATIM_CR1_OPM               (1 << 3)  /* Bit 3: One pulse mode */
#define ATIM_CR1_DIR               (1 << 4)  /* Bit 4: Direction */
#define ATIM_CR1_CMS_SHIFT         (5)       /* Bits 6-5: Center-aligned mode selection */
#define ATIM_CR1_CMS_MASK          (3 << ATIM_CR1_CMS_SHIFT)
#  define ATIM_CR1_EDGE            (0 << ATIM_CR1_CMS_SHIFT) /* 00: Edge-aligned mode */
#  define ATIM_CR1_CENTER1         (1 << ATIM_CR1_CMS_SHIFT) /* 01: Center-aligned mode 1 */
#  define ATIM_CR1_CENTER2         (2 << ATIM_CR1_CMS_SHIFT) /* 10: Center-aligned mode 2 */
#  define ATIM_CR1_CENTER3         (3 << ATIM_CR1_CMS_SHIFT) /* 11: Center-aligned mode 3 */
#define ATIM_CR1_ARPE              (1 << 7)  /* Bit 7: Auto-reload preload enable */
#define ATIM_CR1_CKD_SHIFT         (8)       /* Bits 9-8: Clock division */
#define ATIM_CR1_CKD_MASK          (3 << ATIM_CR1_CKD_SHIFT)
#  define ATIM_CR1_TCKINT          (0 << ATIM_CR1_CKD_SHIFT) /* 00: tDTS=tCK_INT */
#  define ATIM_CR1_2TCKINT         (1 << ATIM_CR1_CKD_SHIFT) /* 01: tDTS=2*tCK_INT */
#  define ATIM_CR1_4TCKINT         (2 << ATIM_CR1_CKD_SHIFT) /* 10: tDTS=4*tCK_INT */

/* Control register 2 */

#define ATIM_CR2_CCPC             (1 << 0)  /* Bit 0:  Capture/Compare Preloaded Control */
#define ATIM_CR2_CCUS             (1 << 2)  /* Bit 2:  Capture/Compare Control Update Selection */
#define ATIM_CR2_CCDS             (1 << 3)  /* Bit 3:  Capture/Compare DMA Selection */
#define ATIM_CR2_MMS_SHIFT        (4)       /* Bits 6-4: Master Mode Selection */
#define ATIM_CR2_MMS_MASK         (7 << ATIM_CR2_MMS_SHIFT)
#define ATIM_CR2_OC1REF           (4 << ATIM_CR2_MMS_SHIFT) /* 100: Compare OC1REF is TRGO */
#define ATIM_CR2_OC2REF           (5 << ATIM_CR2_MMS_SHIFT) /* 101: Compare OC2REF is TRGO */
#define ATIM_CR2_OC3REF           (6 << ATIM_CR2_MMS_SHIFT) /* 110: Compare OC3REF is TRGO */
#define ATIM_CR2_OC4REF           (7 << ATIM_CR2_MMS_SHIFT) /* 111: Compare OC4REF is TRGO */
#define ATIM_CR2_TI1S             (1 << 7)  /* Bit 7: TI1 Selection */
#define ATIM_CR2_OIS1             (1 << 8)  /* Bit 8:  Output Idle state 1 (OC1 output) */
#define ATIM_CR2_OIS1N            (1 << 9)  /* Bit 9:  Output Idle state 1 (OC1N output) */
#define ATIM_CR2_OIS2             (1 << 10) /* Bit 10: Output Idle state 2 (OC2 output) */
#define ATIM_CR2_OIS2N            (1 << 11) /* Bit 11: Output Idle state 2 (OC2N output) */
#define ATIM_CR2_OIS3             (1 << 12) /* Bit 12: Output Idle state 3 (OC3 output) */
#define ATIM_CR2_OIS3N            (1 << 13) /* Bit 13: Output Idle state 3 (OC3N output) */
#define ATIM_CR2_OIS4             (1 << 14) /* Bit 14: Output Idle state 4 (OC4 output) */

/* Slave mode control register */

#define ATIM_SMCR_SMS_SHIFT       (0)       /* Bits 0-2: Slave mode selection */
#define ATIM_SMCR_SMS_MASK        (7 << ATIM_SMCR_SMS_SHIFT)
#  define ATIM_SMCR_DISAB         (0 << ATIM_SMCR_SMS_SHIFT) /* 000: Slave mode disabled */
#  define ATIM_SMCR_ENCMD1        (1 << ATIM_SMCR_SMS_SHIFT) /* 001: Encoder mode 1 */
#  define ATIM_SMCR_ENCMD2        (2 << ATIM_SMCR_SMS_SHIFT) /* 010: Encoder mode 2 */
#  define ATIM_SMCR_ENCMD3        (3 << ATIM_SMCR_SMS_SHIFT) /* 011: Encoder mode 3 */
#  define ATIM_SMCR_RESET         (4 << ATIM_SMCR_SMS_SHIFT) /* 100: Reset Mode */
#  define ATIM_SMCR_GATED         (5 << ATIM_SMCR_SMS_SHIFT) /* 101: Gated Mode */
#  define ATIM_SMCR_TRIGGER       (6 << ATIM_SMCR_SMS_SHIFT) /* 110: Trigger Mode */
#  define ATIM_SMCR_EXTCLK1       (7 << ATIM_SMCR_SMS_SHIFT) /* 111: External Clock Mode 1 */
#define ATIM_SMCR_TS_SHIFT        (4)       /* Bits 4-6: Trigger selection */
#define ATIM_SMCR_TS_MASK         (7 << ATIM_SMCR_TS_SHIFT)
#  define ATIM_SMCR_ITR0          (0 << ATIM_SMCR_TS_SHIFT) /* 000: Internal trigger 0 (ITR0) */
#  define ATIM_SMCR_ITR1          (1 << ATIM_SMCR_TS_SHIFT) /* 001: Internal trigger 1 (ITR1) */
#  define ATIM_SMCR_ITR2          (2 << ATIM_SMCR_TS_SHIFT) /* 010: Internal trigger 2 (ITR2) */
#  define ATIM_SMCR_ITR3          (3 << ATIM_SMCR_TS_SHIFT) /* 011: Internal trigger 3 (ITR3) */
#  define ATIM_SMCR_T1FED         (4 << ATIM_SMCR_TS_SHIFT) /* 100: TI1 Edge Detector (TI1F_ED) */
#  define ATIM_SMCR_TI1FP1        (5 << ATIM_SMCR_TS_SHIFT) /* 101: Filtered Timer Input 1 (TI1FP1) */
#  define ATIM_SMCR_T12FP2        (6 << ATIM_SMCR_TS_SHIFT) /* 110: Filtered Timer Input 2 (TI2FP2) */
#  define ATIM_SMCR_ETRF          (7 << ATIM_SMCR_TS_SHIFT) /* 111: External Trigger input (ETRF) */
#define ATIM_SMCR_MSM             (1 << 7)  /* Bit 7: Master/slave mode */
#define ATIM_SMCR_ETF_SHIFT       (8)       /* Bits 8-11: External trigger filter */
#define ATIM_SMCR_ETF_MASK        (0x0f << ATIM_SMCR_ETF_SHIFT)
#  define ATIM_SMCR_NOFILT        (0 << ATIM_SMCR_ETF_SHIFT) /* 0000: No filter, sampling is done at fDTS */
#  define ATIM_SMCR_FCKINT2       (1 << ATIM_SMCR_ETF_SHIFT) /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define ATIM_SMCR_FCKINT4       (2 << ATIM_SMCR_ETF_SHIFT) /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define ATIM_SMCR_FCKINT8       (3 << ATIM_SMCR_ETF_SHIFT) /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define ATIM_SMCR_FDTSd26       (4 << ATIM_SMCR_ETF_SHIFT) /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define ATIM_SMCR_FDTSd28       (5 << ATIM_SMCR_ETF_SHIFT) /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define ATIM_SMCR_FDTSd46       (6 << ATIM_SMCR_ETF_SHIFT) /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define ATIM_SMCR_FDTSd48       (7 << ATIM_SMCR_ETF_SHIFT) /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define ATIM_SMCR_FDTSd86       (8 << ATIM_SMCR_ETF_SHIFT) /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define ATIM_SMCR_FDTSd88       (9 << ATIM_SMCR_ETF_SHIFT) /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define ATIM_SMCR_FDTSd165      (10 << ATIM_SMCR_ETF_SHIFT) /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define ATIM_SMCR_FDTSd166      (11 << ATIM_SMCR_ETF_SHIFT) /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define ATIM_SMCR_FDTSd168      (12 << ATIM_SMCR_ETF_SHIFT) /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define ATIM_SMCR_FDTSd325      (13 << ATIM_SMCR_ETF_SHIFT) /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define ATIM_SMCR_FDTSd326      (14 << ATIM_SMCR_ETF_SHIFT) /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define ATIM_SMCR_FDTSd328      (15 << ATIM_SMCR_ETF_SHIFT) /* 1111: fSAMPLING=fDTS/32, N=8 */
#define ATIM_SMCR_ETPS_SHIFT      (12)      /* Bits 12-13: External trigger prescaler */
#define ATIM_SMCR_ETPS_MASK       (3 << ATIM_SMCR_ETPS_SHIFT)
#  define ATIM_SMCR_PSCOFF        (0 << ATIM_SMCR_ETPS_SHIFT) /* 00: Prescaler OFF */
#  define ATIM_SMCR_ETRPd2        (1 << ATIM_SMCR_ETPS_SHIFT) /* 01: ETRP frequency divided by 2 */
#  define ATIM_SMCR_ETRPd4        (2 << ATIM_SMCR_ETPS_SHIFT) /* 10: ETRP frequency divided by 4 */
#  define ATIM_SMCR_ETRPd8        (3 << ATIM_SMCR_ETPS_SHIFT) /* 11: ETRP frequency divided by 8 */
#define ATIM_SMCR_ECE             (1 << 14) /* Bit 14: External clock enable */
#define ATIM_SMCR_ETP             (1 << 15) /* Bit 15: External trigger polarity */

/* DMA/Interrupt enable register */

#define ATIM_DIER_UIE             (1 << 0)  /* Bit 0: Update interrupt enable */
#define ATIM_DIER_CC1IE           (1 << 1)  /* Bit 1: Capture/Compare 1 interrupt enable */
#define ATIM_DIER_CC2IE           (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt enable */
#define ATIM_DIER_CC3IE           (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt enable */
#define ATIM_DIER_CC4IE           (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt enable */

#ifdef CONFIG_STM32_STM32F10XX
#  define ATIM_DIER_COMIE         (1 << 5)  /* Bit 5: COM interrupt enable */
#endif

#define ATIM_DIER_TIE             (1 << 6)  /* Bit 6: Trigger interrupt enable */

#ifdef CONFIG_STM32_STM32F10XX
#  define ATIM_DIER_BIE           (1 << 7)  /* Bit 7: Break interrupt enable */
#endif

#define ATIM_DIER_UDE             (1 << 8)  /* Bit 8: Update DMA request enable */
#define ATIM_DIER_CC1DE           (1 << 9)  /* Bit 9: Capture/Compare 1 DMA request enable */
#define ATIM_DIER_CC2DE           (1 << 10) /* Bit 10: Capture/Compare 2 DMA request enable */
#define ATIM_DIER_CC3DE           (1 << 11) /* Bit 11: Capture/Compare 3 DMA request enable */
#define ATIM_DIER_CC4DE           (1 << 12) /* Bit 12: Capture/Compare 4 DMA request enable */

#ifdef CONFIG_STM32_STM32F10XX
#  define ATIM_DIER_COMDE         (1 << 13) /* Bit 13: COM DMA request enable */
#endif

#define ATIM_DIER_TDE             (1 << 14) /* Bit 14: Trigger DMA request enable */

/* Status register */

#define ATIM_SR_UIF               (1 << 0)  /* Bit 0: Update interrupt Flag */
#define ATIM_SR_CC1IF             (1 << 1)  /* Bit 1: Capture/Compare 1 interrupt Flag */
#define ATIM_SR_CC2IF             (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt Flag */
#define ATIM_SR_CC3IF             (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt Flag */
#define ATIM_SR_CC4IF             (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt Flag */
#define ATIM_SR_COMIF             (1 << 5)  /* Bit 5: COM interrupt Flag */
#define ATIM_SR_TIF               (1 << 6)  /* Bit 6: Trigger interrupt Flag */

#ifdef CONFIG_STM32_STM32F10XX
#  define ATIM_SR_BIF             (1 << 7)  /* Bit 7: Break interrupt Flag */
#endif

#define ATIM_SR_CC1OF             (1 << 9)  /* Bit 9: Capture/Compare 1 Overcapture Flag */
#define ATIM_SR_CC2OF             (1 << 10) /* Bit 10: Capture/Compare 2 Overcapture Flag */
#define ATIM_SR_CC3OF             (1 << 11) /* Bit 11: Capture/Compare 3 Overcapture Flag */
#define ATIM_SR_CC4OF             (1 << 12) /* Bit 12: Capture/Compare 4 Overcapture Flag */

/* Event generation register */

#define ATIM_EGR_UG               (1 << 0)  /* Bit 0: Update Generation */
#define ATIM_EGR_CC1G             (1 << 1)  /* Bit 1: Capture/Compare 1 Generation */
#define ATIM_EGR_CC2G             (1 << 2)  /* Bit 2: Capture/Compare 2 Generation */
#define ATIM_EGR_CC3G             (1 << 3)  /* Bit 3: Capture/Compare 3 Generation */
#define ATIM_EGR_CC4G             (1 << 4)  /* Bit 4: Capture/Compare 4 Generation */

#ifdef CONFIG_STM32_STM32F10XX
#  define ATIM_EGR_COMG           (1 << 5)  /* Bit 5: Capture/Compare Control Update Generation */
#endif

#define ATIM_EGR_TG               (1 << 6)  /* Bit 6: Trigger Generation */

#ifdef CONFIG_STM32_STM32F10XX
#  define ATIM_EGR_BG             (1 << 7)  /* Bit 7: Break Generation */
#endif

/* Capture/compare mode register 1 -- Output compare mode */

#define ATIM_CCMR1_CC1S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 1 Selection */
#define ATIM_CCMR1_CC1S_MASK      (3 << ATIM_CCMR1_CC1S_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC1FE          (1 << 2)  /* Bit 2: Output Compare 1 Fast enable */
#define ATIM_CCMR1_OC1PE          (1 << 3)  /* Bit 3: Output Compare 1 Preload enable */
#define ATIM_CCMR1_OC1M_SHIFT     (4)       /* Bits 6-4: Output Compare 1 Mode */
#define ATIM_CCMR1_OC1M_MASK      (7 << ATIM_CCMR1_OC1M_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC1CE          (1 << 7)  /* Bit 7: Output Compare 1Clear Enable */
#define ATIM_CCMR1_CC2S_SHIFT     (8)       /* Bits 8-9: Capture/Compare 2 Selection */
#define ATIM_CCMR1_CC2S_MASK      (3 << ATIM_CCMR1_CC2S_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC2FE          (1 << 10) /* Bit 10: Output Compare 2 Fast enable */
#define ATIM_CCMR1_OC2PE          (1 << 11) /* Bit 11: Output Compare 2 Preload enable */
#define ATIM_CCMR1_OC2M_SHIFT     (12)      /* Bits 14-12: Output Compare 2 Mode */
#define ATIM_CCMR1_OC2M_MASK      (7 << ATIM_CCMR1_OC2M_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_OC2CE          (1 << 15) /* Bit 15: Output Compare 2 Clear Enable */

/* Common CCMR (unshifted) Capture/Compare Selection bit-field definitions */

#define ATIM_CCMR_CCS_CCOUT       (0)       /* 00: CCx channel  output */
#define ATIM_CCMR_CCS_CCIN1       (1)       /* 01: CCx channel input, ICx is TIx */
#define ATIM_CCMR_CCS_CCIN2       (2)       /* 10: CCx channel input, ICx is TIy */
#define ATIM_CCMR_CCS_CCINTRC     (3)       /* 11: CCx channel input, ICx is TRC */

/* Common CCMR (unshifted) Compare Mode bit field definitions */

#define ATIM_CCMR_MODE_FRZN       (0)       /* 000: Frozen */
#define ATIM_CCMR_MODE_CHACT      (1)       /* 001: Channel x active on match */
#define ATIM_CCMR_MODE_CHINACT    (2)       /* 010: Channel x inactive on match */
#define ATIM_CCMR_MODE_OCREFTOG   (3)       /* 011: OCxREF toggle ATIM_CNT=ATIM_CCRx */
#define ATIM_CCMR_MODE_OCREFLO    (4)       /* 100: OCxREF forced low */
#define ATIM_CCMR_MODE_OCREFHI    (5)       /* 101: OCxREF forced high */
#define ATIM_CCMR_MODE_PWM1       (6)       /* 110: PWM mode 1 */
#define ATIM_CCMR_MODE_PWM2       (7)       /* 111: PWM mode 2 */

/* Capture/compare mode register 1 -- Input capture mode */

                                            /* Bits 1-0:(same as output compare mode) */
#define ATIM_CCMR1_IC1PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 1 Prescaler */
#define ATIM_CCMR1_IC1PSC_MASK    (3 << ATIM_CCMR1_IC1PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC1F_SHIFT     (4)       /* Bits 7-4: Input Capture 1 Filter */
#define ATIM_CCMR1_IC1F_MASK      (0x0f << ATIM_CCMR1_IC1F_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
                                            /* Bits 9:8 (same as output compare mode) */
#define ATIM_CCMR1_IC2PSC_SHIFT   (10)      /* Bits 11:10: Input Capture 2 Prescaler */
#define ATIM_CCMR1_IC2PSC_MASK    (3 << ATIM_CCMR1_IC2PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */
#define ATIM_CCMR1_IC2F_SHIFT     (12)      /* Bits 15-12: Input Capture 2 Filter */
#define ATIM_CCMR1_IC2F_MASK      (0x0f << ATIM_CCMR1_IC2F_SHIFT)
                                            /* (See common (unshifted) bit field definitions below) */

/* Common CCMR (unshifted) Input Capture Prescaler bit-field definitions */

#define ATIM_CCMR_ICPSC_NOPSC     (0)       /* 00: no prescaler, capture each edge */
#define ATIM_CCMR_ICPSC_EVENTS2   (1)       /* 01: capture once every 2 events */
#define ATIM_CCMR_ICPSC_EVENTS4   (2)       /* 10: capture once every 4 events */
#define ATIM_CCMR_ICPSC_EVENTS8   (3)       /* 11: capture once every 8 events */

/* Common CCMR (unshifted) Input Capture Filter bit-field definitions */

#define ATIM_CCMR_ICF_NOFILT      (0)       /* 0000: No filter, sampling at fDTS */
#define ATIM_CCMR_ICF_FCKINT2     (1)       /* 0001: fSAMPLING=fCK_INT, N=2 */
#define ATIM_CCMR_ICF_FCKINT4     (2)       /* 0010: fSAMPLING=fCK_INT, N=4 */
#define ATIM_CCMR_ICF_FCKINT8     (3)       /* 0011: fSAMPLING=fCK_INT, N=8 */
#define ATIM_CCMR_ICF_FDTSd26     (4)       /* 0100: fSAMPLING=fDTS/2, N=6 */
#define ATIM_CCMR_ICF_FDTSd28     (5)       /* 0101: fSAMPLING=fDTS/2, N=8 */
#define ATIM_CCMR_ICF_FDTSd46     (6)       /* 0110: fSAMPLING=fDTS/4, N=6 */
#define ATIM_CCMR_ICF_FDTSd48     (7)       /* 0111: fSAMPLING=fDTS/4, N=8 */
#define ATIM_CCMR_ICF_FDTSd86     (8)       /* 1000: fSAMPLING=fDTS/8, N=6 */
#define ATIM_CCMR_ICF_FDTSd88     (9)       /* 1001: fSAMPLING=fDTS/8, N=8 */
#define ATIM_CCMR_ICF_FDTSd165    (10)      /* 1010: fSAMPLING=fDTS/16, N=5 */
#define ATIM_CCMR_ICF_FDTSd166    (11)      /* 1011: fSAMPLING=fDTS/16, N=6 */
#define ATIM_CCMR_ICF_FDTSd168    (12)      /* 1100: fSAMPLING=fDTS/16, N=8 */
#define ATIM_CCMR_ICF_FDTSd325    (13)      /* 1101: fSAMPLING=fDTS/32, N=5 */
#define ATIM_CCMR_ICF_FDTSd326    (14)      /* 1110: fSAMPLING=fDTS/32, N=6 */
#define ATIM_CCMR_ICF_FDTSd328    (15)      /* 1111: fSAMPLING=fDTS/32, N=8 */

/* Capture/compare mode register 2 - Output Compare mode */

#define ATIM_CCMR2_CC3S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 3 Selection */
#define ATIM_CCMR2_CC3S_MASK      (3 << ATIM_CCMR2_CC3S_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC3FE          (1 << 2)  /* Bit 2: Output Compare 3 Fast enable */
#define ATIM_CCMR2_OC3PE          (1 << 3)  /* Bit 3: Output Compare 3 Preload enable */
#define ATIM_CCMR2_OC3M_SHIFT     (4)       /* Bits 6-4: Output Compare 3 Mode */
#define ATIM_CCMR2_OC3M_MASK      (7 << ATIM_CCMR2_OC3M_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC3CE          (1 << 7)  /* Bit 7: Output Compare 3 Clear Enable */
#define ATIM_CCMR2_CC4S_SHIFT     (8)       /* Bits 9-8: Capture/Compare 4 Selection */
#define ATIM_CCMR2_CC4S_MASK      (3 << ATIM_CCMR2_CC4S_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC4FE          (1 << 10) /* Bit 10: Output Compare 4 Fast enable */
#define ATIM_CCMR2_OC4PE          (1 << 11) /* Bit 11: Output Compare 4 Preload enable */
#define ATIM_CCMR2_OC4M_SHIFT     (12)      /* Bits 14-12: Output Compare 4 Mode */
#define ATIM_CCMR2_OC4M_MASK      (7 << ATIM_CCMR2_OC4M_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_OC4CE          (1 << 15) /* Bit 15: Output Compare 4 Clear Enable */

/* Capture/compare mode register 2 - Input Capture Mode */

                                            /* Bits 1-0:(same as output compare mode) */
#define ATIM_CCMR2_IC3PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 3 Prescaler */
#define ATIM_CCMR1_IC3PSC_MASK    (3 << ATIM_CCMR2_IC3PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC3F_SHIFT     (4)       /* Bits 7-4: Input Capture 3 Filter */
#define ATIM_CCMR2_IC3F_MASK      (0x0f << ATIM_CCMR2_IC3F_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
                                            /* Bits 9:8 (same as output compare mode) */
#define ATIM_CCMR2_IC4PSC_SHIFT   (10)      /* Bits 11:10: Input Capture 4 Prescaler */
#define ATIM_CCMR2_IC4PSC_MASK    (3 << ATIM_CCMR2_IC4PSC_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */
#define ATIM_CCMR2_IC4F_SHIFT     (12)      /* Bits 15-12: Input Capture 4 Filter */
#define ATIM_CCMR2_IC4F_MASK      (0x0f << ATIM_CCMR2_IC4F_SHIFT)
                                            /* (See common (unshifted) bit field definitions above) */

/* Capture/compare enable register */

#define ATIM_CCER_CC1E            (1 << 0)  /* Bit 0: Capture/Compare 1 output enable */
#define ATIM_CCER_CC1P            (1 << 1)  /* Bit 1: Capture/Compare 1 output Polarity */
#define ATIM_CCER_CC1NE           (1 << 2)  /* Bit 2: Capture/Compare 1 Complementary output enable */
#define ATIM_CCER_CC1NP           (1 << 3)  /* Bit 3: Capture/Compare 1 Complementary output Polarity */
#define ATIM_CCER_CC2E            (1 << 4)  /* Bit 4: Capture/Compare 2 output enable */
#define ATIM_CCER_CC2P            (1 << 5)  /* Bit 5: Capture/Compare 2 output Polarity */
#define ATIM_CCER_CC2NE           (1 << 6)  /* Bit 6: Capture/Compare 2 Complementary output enable */
#define ATIM_CCER_CC2NP           (1 << 7)  /* Bit 7: Capture/Compare 2 Complementary output Polarity */
#define ATIM_CCER_CC3E            (1 << 8)  /* Bit 8: Capture/Compare 3 output enable */
#define ATIM_CCER_CC3P            (1 << 9)  /* Bit 9: Capture/Compare 3 output Polarity */
#define ATIM_CCER_CC3NE           (1 << 10) /* Bit 10: Capture/Compare 3 Complementary output enable */
#define ATIM_CCER_CC3NP           (1 << 11) /* Bit 11: Capture/Compare 3 Complementary output Polarity */
#define ATIM_CCER_CC4E            (1 << 12) /* Bit 12: Capture/Compare 4 output enable */
#define ATIM_CCER_CC4P            (1 << 13) /* Bit 13: Capture/Compare 4 output Polarity */

#ifdef CONFIG_STM32_STM32F40XX
#  define ATIM_CCER_CC4NP         (1 << 15) /* Bit 15: Capture/Compare 4 output Polarity */
#endif

/* Repetition counter register */

#define ATIM_RCR_REP_SHIFT        (0)       /* Bits 7-0: Repetition Counter Value */
#define ATIM_RCR_REP_MASK         (0xff << ATIM_RCR_REP_SHIFT)

#define ATIM_RCR_REP_MAX           128

/* Break and dead-time register */

#define ATIM_BDTR_DTG_SHIFT       (0)       /* Bits 7:0 [7:0]: Dead-Time Generator set-up */
#define ATIM_BDTR_DTG_MASK        (0xff << ATIM_BDTR_DTG_SHIFT)
#define ATIM_BDTR_LOCK_SHIFT      (8)       /* Bits 9:8 [1:0]: Lock Configuration */
#define ATIM_BDTR_LOCK_MASK       (3 << ATIM_BDTR_LOCK_SHIFT)
#  define ATIM_BDTR_LOCKOFF       (0 << ATIM_BDTR_LOCK_SHIFT) /* 00: LOCK OFF - No bit is write protected */
#  define ATIM_BDTR_LOCK1         (1 << ATIM_BDTR_LOCK_SHIFT) /* 01: LOCK Level 1 protection */
#  define ATIM_BDTR_LOCK2         (2 << ATIM_BDTR_LOCK_SHIFT) /* 10: LOCK Level 2 protection */
#  define ATIM_BDTR_LOCK3         (3 << ATIM_BDTR_LOCK_SHIFT) /* 11: LOCK Level 3 protection */ */
#define ATIM_BDTR_OSSI            (1 << 10) /* Bit 10: Off-State Selection for Idle mode */
#define ATIM_BDTR_OSSR            (1 << 11) /* Bit 11: Off-State Selection for Run mode */
#define ATIM_BDTR_BKE             (1 << 12) /* Bit 12: Break enable */
#define ATIM_BDTR_BKP             (1 << 13) /* Bit 13: Break Polarity */
#define ATIM_BDTR_AOE             (1 << 14) /* Bit 14: Automatic Output enable */
#define ATIM_BDTR_MOE             (1 << 15) /* Bit 15: Main Output enable */

/* DMA control register */

#define ATIM_DCR_DBL_SHIFT        (8)       /* Bits 12-8: DMA Burst Length */
#define ATIM_DCR_DBL_MASK         (0x1f << ATIM_DCR_DBL_SHIFT)
#  define ATIM_DCR_DBL(n)         (((n)-1) << ATIM_DCR_DBL_SHIFT) /* n transfers, n = 1..18 */
#define ATIM_DCR_DBA_SHIFT        (0)       /* Bits 4-0: DMA Base Address */
#define ATIM_DCR_DBA_MASK         (0x1f << ATIM_DCR_DBA_SHIFT)

/* Control register 1 (TIM2-5 and TIM9-14) */

#define GTIM_CR1_CEN              (1 << 0)  /* Bit 0: Counter enable */
#define GTIM_CR1_UDIS             (1 << 1)  /* Bit 1: Update Disable */
#define GTIM_CR1_URS              (1 << 2)  /* Bit 2: Update Request Source */
#define GTIM_CR1_OPM              (1 << 3)  /* Bit 3: One Pulse Mode (TIM2-5, 9, and 12 only) */
#define GTIM_CR1_DIR              (1 << 4)  /* Bit 4: Direction (TIM2-5 only) */
#define GTIM_CR1_CMS_SHIFT        (5)       /* Bits 6-5: Center-aligned Mode Selection (TIM2-5 only) */
#define GTIM_CR1_CMS_MASK         (3 << GTIM_CR1_CMS_SHIFT)
#  define GTIM_CR1_EDGE           (0 << GTIM_CR1_CMS_SHIFT) /* 00: Edge-aligned mode.  */
#  define GTIM_CR1_CENTER1        (1 << GTIM_CR1_CMS_SHIFT) /* 01: Center-aligned mode 1 */
#  define GTIM_CR1_CENTER2        (2 << GTIM_CR1_CMS_SHIFT) /* 10: Center-aligned mode 2 */
#  define GTIM_CR1_CENTER3        (3 << GTIM_CR1_CMS_SHIFT) /* 11: Center-aligned mode 3 */
#define GTIM_CR1_ARPE             (1 << 7)  /* Bit 7: Auto-Reload Preload enable */
#define GTIM_CR1_CKD_SHIFT        (8)       /* Bits 9-8: Clock Division */
#define GTIM_CR1_CKD_MASK         (3 << GTIM_CR1_CKD_SHIFT)
#  define GTIM_CR1_TCKINT         (0 << GTIM_CR1_CKD_SHIFT) /* 00: tDTS = tCK_INT */
#  define GTIM_CR1_2TCKINT        (1 << GTIM_CR1_CKD_SHIFT) /* 01: tDTS = 2 x tCK_INT */
#  define GTIM_CR1_4TCKINT        (2 << GTIM_CR1_CKD_SHIFT) /* 10: tDTS = 4 x tCK_INT */

/* Control register 2 (TIM2-5 and TIM9/12 only) */

#define GTIM_CR2_CCDS             (1 << 3)  /* Bit 3: Capture/Compare DMA Selection (TIM2-5 only) */
#define GTIM_CR2_MMS_SHIFT        (4)       /* Bits 6-4: Master Mode Selection */
#define GTIM_CR2_MMS_MASK         (7 << GTIM_CR2_MMS_SHIFT)
#  define GTIM_CR2_RESET          (0 << GTIM_CR2_MMS_SHIFT) /* 000: Reset */
#  define GTIM_CR2_ENAB           (1 << GTIM_CR2_MMS_SHIFT) /* 001: Enable */
#  define GTIM_CR2_UPDT           (2 << GTIM_CR2_MMS_SHIFT) /* 010: Update */
#  define GTIM_CR2_CMPP           (3 << GTIM_CR2_MMS_SHIFT) /* 011: Compare Pulse */
#  define GTIM_CR2_CMP1           (4 << GTIM_CR2_MMS_SHIFT) /* 100: Compare - OC1REF signal is used as trigger output (TRGO) */
#  define GTIM_CR2_CMP2           (5 << GTIM_CR2_MMS_SHIFT) /* 101: Compare - OC2REF signal is used as trigger output (TRGO) */
#  define GTIM_CR2_CMP3           (6 << GTIM_CR2_MMS_SHIFT) /* 110: Compare - OC3REF signal is used as trigger output (TRGO, TIM2-5 only) */
#  define GTIM_CR2_CMP4           (7 << GTIM_CR2_MMS_SHIFT) /* 111: Compare - OC4REF signal is used as trigger output (TRGO, TIM2-5 only) */
#define GTIM_CR2_TI1S             (1 << 7)  /* Bit 7: TI1 Selection */

/* Slave mode control register (TIM2-5 only) */

#define GTIM_SMCR_SMS_SHIFT       (0)       /* Bits 2-0: Slave Mode Selection */
#define GTIM_SMCR_SMS_MASK        (7 << GTIM_SMCR_SMS_SHIFT)
#  define GTIM_SMCR_DISAB         (0 << GTIM_SMCR_SMS_SHIFT) /* 000: Slave mode disabled */
#  define GTIM_SMCR_ENCMD1        (1 << GTIM_SMCR_SMS_SHIFT) /* 001: Encoder mode 1 */
#  define GTIM_SMCR_ENCMD2        (2 << GTIM_SMCR_SMS_SHIFT) /* 010: Encoder mode 2 */
#  define GTIM_SMCR_ENCMD3        (3 << GTIM_SMCR_SMS_SHIFT) /* 011: Encoder mode 3 */
#  define GTIM_SMCR_RESET         (4 << GTIM_SMCR_SMS_SHIFT) /* 100: Reset Mode  */
#  define GTIM_SMCR_GATED         (5 << GTIM_SMCR_SMS_SHIFT) /* 101: Gated Mode  */
#  define GTIM_SMCR_TRIGGER       (6 << GTIM_SMCR_SMS_SHIFT) /* 110: Trigger Mode */
#  define GTIM_SMCR_EXTCLK1       (7 << GTIM_SMCR_SMS_SHIFT) /* 111: External Clock Mode 1 */
#define GTIM_SMCR_TS_SHIFT        (4)       /* Bits 6-4: Trigger Selection */
#define GTIM_SMCR_TS_MASK         (7 << GTIM_SMCR_TS_SHIFT)
#  define GTIM_SMCR_ITR0          (0 << GTIM_SMCR_TS_SHIFT) /* 000: Internal Trigger 0 (ITR0). TIM1 */
#  define GTIM_SMCR_ITR1          (1 << GTIM_SMCR_TS_SHIFT) /* 001: Internal Trigger 1 (ITR1). TIM2 */
#  define GTIM_SMCR_ITR2          (2 << GTIM_SMCR_TS_SHIFT) /* 010: Internal Trigger 2 (ITR2). TIM3 */
#  define GTIM_SMCR_ITR3          (3 << GTIM_SMCR_TS_SHIFT) /* 011: Internal Trigger 3 (ITR3). TIM4 */
#  define GTIM_SMCR_TI1FED        (4 << GTIM_SMCR_TS_SHIFT) /* 100: TI1 Edge Detector (TI1F_ED) */
#  define GTIM_SMCR_TI1FP1        (5 << GTIM_SMCR_TS_SHIFT) /* 101: Filtered Timer Input 1 (TI1FP1) */
#  define GTIM_SMCR_TI2FP2        (6 << GTIM_SMCR_TS_SHIFT) /* 110: Filtered Timer Input 2 (TI2FP2) */
#  define GTIM_SMCR_ETRF          (7 << GTIM_SMCR_TS_SHIFT) /* 111: External Trigger input (ETRF) */
#define GTIM_SMCR_MSM             (1 << 7)  /* Bit 7: Master/Slave mode */
#define GTIM_SMCR_ETF_SHIFT       (8)       /* Bits 11-8: External Trigger Filter */
#define GTIM_SMCR_ETF_MASK        (0x0f << GTIM_SMCR_ETF_SHIFT)
#  define GTIM_SMCR_NOFILT        (0 << GTIM_SMCR_ETF_SHIFT) /* 0000: No filter, sampling is done at fDTS */
#  define GTIM_SMCR_FCKINT2       (1 << GTIM_SMCR_ETF_SHIFT) /* 0001: fSAMPLING=fCK_INT, N=2 */
#  define GTIM_SMCR_FCKINT4       (2 << GTIM_SMCR_ETF_SHIFT) /* 0010: fSAMPLING=fCK_INT, N=4 */
#  define GTIM_SMCR_FCKINT8       (3 << GTIM_SMCR_ETF_SHIFT) /* 0011: fSAMPLING=fCK_INT, N=8 */
#  define GTIM_SMCR_FDTSd26       (4 << GTIM_SMCR_ETF_SHIFT) /* 0100: fSAMPLING=fDTS/2, N=6 */
#  define GTIM_SMCR_FDTSd28       (5 << GTIM_SMCR_ETF_SHIFT) /* 0101: fSAMPLING=fDTS/2, N=8 */
#  define GTIM_SMCR_FDTSd36       (6 << GTIM_SMCR_ETF_SHIFT) /* 0110: fSAMPLING=fDTS/4, N=6 */
#  define GTIM_SMCR_FDTSd38       (7 << GTIM_SMCR_ETF_SHIFT) /* 0111: fSAMPLING=fDTS/4, N=8 */
#  define GTIM_SMCR_FDTSd86       (8 << GTIM_SMCR_ETF_SHIFT) /* 1000: fSAMPLING=fDTS/8, N=6 */
#  define GTIM_SMCR_FDTSd88       (9 << GTIM_SMCR_ETF_SHIFT) /* 1001: fSAMPLING=fDTS/8, N=8 */
#  define GTIM_SMCR_FDTSd165      (10 << GTIM_SMCR_ETF_SHIFT) /* 1010: fSAMPLING=fDTS/16, N=5 */
#  define GTIM_SMCR_FDTSd166      (11 << GTIM_SMCR_ETF_SHIFT) /* 1011: fSAMPLING=fDTS/16, N=6 */
#  define GTIM_SMCR_FDTSd168      (12 << GTIM_SMCR_ETF_SHIFT) /* 1100: fSAMPLING=fDTS/16, N=8 */
#  define GTIM_SMCR_FDTSd325      (13 << GTIM_SMCR_ETF_SHIFT) /* 1101: fSAMPLING=fDTS/32, N=5 */
#  define GTIM_SMCR_FDTSd326      (14 << GTIM_SMCR_ETF_SHIFT) /* 1110: fSAMPLING=fDTS/32, N=6 */
#  define GTIM_SMCR_FDTSd328      (15 << GTIM_SMCR_ETF_SHIFT) /* 1111: fSAMPLING=fDTS/32, N=8 */
#define GTIM_SMCR_ETPS_SHIFT      (12)      /* Bits 13-12: External Trigger Prescaler */
#define GTIM_SMCR_ETPS_MASK       (3 << GTIM_SMCR_ETPS_SHIFT)
#  define GTIM_SMCR_PSCOFF        (0 << GTIM_SMCR_ETPS_SHIFT) /* 00: Prescaler OFF */
#  define GTIM_SMCR_ETRPd2        (1 << GTIM_SMCR_ETPS_SHIFT) /* 01: ETRP frequency divided by 2 */
#  define GTIM_SMCR_ETRPd4        (2 << GTIM_SMCR_ETPS_SHIFT) /* 10: ETRP frequency divided by 4 */
#  define GTIM_SMCR_ETRPd8        (3 << GTIM_SMCR_ETPS_SHIFT) /* 11: ETRP frequency divided by 8 */
#define GTIM_SMCR_ECE             (1 << 14) /* Bit 14: External Clock enable */
#define GTIM_SMCR_ETP             (1 << 15) /* Bit 15: External Trigger Polarity */

/* DMA/Interrupt enable register (TIM2-5 and TIM9-14) */

#define GTIM_DIER_UIE             (1 << 0)  /* Bit 0: Update interrupt enable */
#define GTIM_DIER_CC1IE           (1 << 1)  /* Bit 1: Capture/Compare 1 interrupt enable */
#define GTIM_DIER_CC2IE           (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt enable (TIM2-5,9,&12 only) */
#define GTIM_DIER_CC3IE           (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt enable (TIM2-5 only) */
#define GTIM_DIER_CC4IE           (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt enable (TIM2-5 only) */
#define GTIM_DIER_TIE             (1 << 6)  /* Bit 6: Trigger interrupt enable (TIM2-5,9,&12 only) */
#define GTIM_DIER_UDE             (1 << 8)  /* Bit 8: Update DMA request enable (TIM2-5 only) */
#define GTIM_DIER_CC1DE           (1 << 9)  /* Bit 9: Capture/Compare 1 DMA request enable (TIM2-5 only) */
#define GTIM_DIER_CC2DE           (1 << 10) /* Bit 10: Capture/Compare 2 DMA request enable (TIM2-5 only) */
#define GTIM_DIER_CC3DE           (1 << 11) /* Bit 11: Capture/Compare 3 DMA request enable (TIM2-5 only) */
#define GTIM_DIER_CC4DE           (1 << 12) /* Bit 12: Capture/Compare 4 DMA request enable (TIM2-5 only) */
#define GTIM_DIER_TDE             (1 << 14) /* Bit 14: Trigger DMA request enable (TIM2-5 only) */

/* Status register */

#define GTIM_SR_UIF               (1 << 0)  /* Bit 0: Update interrupt flag */
#define GTIM_SR_CC1IF             (1 << 1)  /* Bit 1: Capture/compare 1 interrupt Flag */
#define GTIM_SR_CC2IF             (1 << 2)  /* Bit 2: Capture/Compare 2 interrupt Flag (TIM2-5,9,&12 only) */
#define GTIM_SR_CC3IF             (1 << 3)  /* Bit 3: Capture/Compare 3 interrupt Flag (TIM2-5 only) */
#define GTIM_SR_CC4IF             (1 << 4)  /* Bit 4: Capture/Compare 4 interrupt Flag (TIM2-5 only) */
#define GTIM_SR_TIF               (1 << 6)  /* Bit 6: Trigger interrupt Flag (TIM2-5,9,&12 only) */
#define GTIM_SR_CC1OF             (1 << 9)  /* Bit 9: Capture/Compare 1 Overcapture Flag */
#define GTIM_SR_CC2OF             (1 << 10) /* Bit 10: Capture/Compare 2 Overcapture Flag (TIM2-5,9,&12 only) */
#define GTIM_SR_CC3OF             (1 << 11) /* Bit 11: Capture/Compare 3 Overcapture Flag (TIM2-5 only) */
#define GTIM_SR_CC4OF             (1 << 12) /* Bit 12: Capture/Compare 4 Overcapture Flag (TIM2-5 only) */

/* Event generation register (TIM2-5 and TIM9-14) */

#define GTIM_EGR_UG               (1 << 0)  /* Bit 0: Update generation */
#define GTIM_EGR_CC1G             (1 << 1)  /* Bit 1: Capture/compare 1 generation */
#define GTIM_EGR_CC2G             (1 << 2)  /* Bit 2: Capture/compare 2 generation (TIM2-5,9,&12 only) */
#define GTIM_EGR_CC3G             (1 << 3)  /* Bit 3: Capture/compare 3 generation (TIM2-5 only) */
#define GTIM_EGR_CC4G             (1 << 4)  /* Bit 4: Capture/compare 4 generation (TIM2-5 only) */
#define GTIM_EGR_TG               (1 << 6)  /* Bit 6: Trigger generation (TIM2-5,9,&12 only) */

/* Capture/compare mode register 1 - Output compare mode (TIM2-5 and TIM9-14) */

#define GTIM_CCMR1_CC1S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 1 Selection */
#define GTIM_CCMR1_CC1S_MASK      (3 << GTIM_CCMR1_CC1S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions below) */
#define GTIM_CCMR1_OC1FE          (1 << 2)  /* Bit 2: Output Compare 1 Fast enable */
#define GTIM_CCMR1_OC1PE          (1 << 3)  /* Bit 3: Output Compare 1 Preload enable */
#define GTIM_CCMR1_OC1M_SHIFT     (4)       /* Bits 6-4: Output Compare 1 Mode */
#define GTIM_CCMR1_OC1M_MASK      (7 << GTIM_CCMR1_OC1M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions below) */
#define GTIM_CCMR1_OC1CE          (1 << 7)  /* Bit 7: Output Compare 1Clear Enable */
#define GTIM_CCMR1_CC2S_SHIFT     (8)       /* Bits 9-8: Capture/Compare 2 Selection */
#define GTIM_CCMR1_CC2S_MASK      (3 << GTIM_CCMR1_CC2S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions below) */
#define GTIM_CCMR1_OC2FE          (1 << 10) /* Bit 10: Output Compare 2 Fast enable */
#define GTIM_CCMR1_OC2PE          (1 << 11) /* Bit 11: Output Compare 2 Preload enable */
#define GTIM_CCMR1_OC2M_SHIFT     (12)      /* Bits 14-12: Output Compare 2 Mode */
#define GTIM_CCMR1_OC2M_MASK      (7 << GTIM_CCMR1_OC2M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions below) */
#define GTIM_CCMR1_OC2CE          (1 << 15) /* Bit 15: Output Compare 2 Clear Enable */

/* Common CCMR (unshifted) Capture/Compare Selection bit-field definitions */

#define GTIM_CCMR_CCS_CCOUT       (0)       /* 00: CCx channel output */
#define GTIM_CCMR_CCS_CCIN1       (1)       /* 01: CCx channel input, ICx is TIx */
#define GTIM_CCMR_CCS_CCIN2       (2)       /* 10: CCx channel input, ICx is TIy */
#define GTIM_CCMR_CCS_CCINTRC     (3)       /* 11: CCx channel input, ICx is TRC */

/* Common CCMR (unshifted) Compare Mode bit field definitions */

#define GTIM_CCMR_MODE_FRZN       (0)       /* 000: Frozen */
#define GTIM_CCMR_MODE_CHACT      (1)       /* 001: Channel x active on match */
#define GTIM_CCMR_MODE_CHINACT    (2)       /* 010: Channel x inactive on match */
#define GTIM_CCMR_MODE_OCREFTOG   (3)       /* 011: OCxREF toggle ATIM_CNT=ATIM_CCRx */
#define GTIM_CCMR_MODE_OCREFLO    (4)       /* 100: OCxREF forced low */
#define GTIM_CCMR_MODE_OCREFHI    (5)       /* 101: OCxREF forced high */
#define GTIM_CCMR_MODE_PWM1       (6)       /* 110: PWM mode 1 */
#define GTIM_CCMR_MODE_PWM2       (7)       /* 111: PWM mode 2 */

/* Capture/compare mode register 1 - Input capture mode (TIM2-5 and TIM9-14) */

                                            /* Bits 1-0 (Same as Output Compare Mode) */
#define GTIM_CCMR1_IC1PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 1 Prescaler */
#define GTIM_CCMR1_IC1PSC_MASK    (3 << GTIM_CCMR1_IC1PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR1_IC1F_SHIFT     (4)       /* Bits 7-4: Input Capture 1 Filter */
#define GTIM_CCMR1_IC1F_MASK      (0x0f << GTIM_CCMR1_IC1F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */
                                            /* Bits 9-8: (Same as Output Compare Mode) */
#define GTIM_CCMR1_IC2PSC_SHIFT   (10)      /* Bits 11-10: Input Capture 2 Prescaler */
#define GTIM_CCMR1_IC2PSC_MASK    (3 << GTIM_CCMR1_IC2PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR1_IC2F_SHIFT     (12)      /* Bits 15-12: Input Capture 2 Filter */
#define GTIM_CCMR1_IC2F_MASK      (0x0f << GTIM_CCMR1_IC2F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */

/* Common CCMR (unshifted) Input Capture Prescaler bit-field definitions */

#define GTIM_CCMR_ICPSC_NOPSC     (0)       /* 00: no prescaler, capture each edge */
#define GTIM_CCMR_ICPSC_EVENTS2   (1)       /* 01: capture once every 2 events */
#define GTIM_CCMR_ICPSC_EVENTS4   (2)       /* 10: capture once every 4 events */
#define GTIM_CCMR_ICPSC_EVENTS8   (3)       /* 11: capture once every 8 events */

/* Common CCMR (unshifted) Input Capture Filter bit-field definitions */

#define GTIM_CCMR_ICF_NOFILT      (0)       /* 0000: No filter, sampling at fDTS */
#define GTIM_CCMR_ICF_FCKINT2     (1)       /* 0001: fSAMPLING=fCK_INT, N=2 */
#define GTIM_CCMR_ICF_FCKINT4     (2)       /* 0010: fSAMPLING=fCK_INT, N=4 */
#define GTIM_CCMR_ICF_FCKINT8     (3)       /* 0011: fSAMPLING=fCK_INT, N=8 */
#define GTIM_CCMR_ICF_FDTSd26     (4)       /* 0100: fSAMPLING=fDTS/2, N=6 */
#define GTIM_CCMR_ICF_FDTSd28     (5)       /* 0101: fSAMPLING=fDTS/2, N=8 */
#define GTIM_CCMR_ICF_FDTSd46     (6)       /* 0110: fSAMPLING=fDTS/4, N=6 */
#define GTIM_CCMR_ICF_FDTSd48     (7)       /* 0111: fSAMPLING=fDTS/4, N=8 */
#define GTIM_CCMR_ICF_FDTSd86     (8)       /* 1000: fSAMPLING=fDTS/8, N=6 */
#define GTIM_CCMR_ICF_FDTSd88     (9)       /* 1001: fSAMPLING=fDTS/8, N=8 */
#define GTIM_CCMR_ICF_FDTSd165    (10)      /* 1010: fSAMPLING=fDTS/16, N=5 */
#define GTIM_CCMR_ICF_FDTSd166    (11)      /* 1011: fSAMPLING=fDTS/16, N=6 */
#define GTIM_CCMR_ICF_FDTSd168    (12)      /* 1100: fSAMPLING=fDTS/16, N=8 */
#define GTIM_CCMR_ICF_FDTSd325    (13)      /* 1101: fSAMPLING=fDTS/32, N=5 */
#define GTIM_CCMR_ICF_FDTSd326    (14)      /* 1110: fSAMPLING=fDTS/32, N=6 */
#define GTIM_CCMR_ICF_FDTSd328    (15)      /* 1111: fSAMPLING=fDTS/32, N=8 */

/* Capture/compare mode register 2 - Output Compare mode (TIM2-5 only) */

#define GTIM_CCMR2_CC3S_SHIFT     (0)       /* Bits 1-0: Capture/Compare 3 Selection */
#define GTIM_CCMR2_CC3S_MASK      (3 << GTIM_CCMR2_CC3S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions above) */
#define GTIM_CCMR2_OC3FE          (1 << 2)  /* Bit 2: Output Compare 3 Fast enable */
#define GTIM_CCMR2_OC3PE          (1 << 3)  /* Bit 3: Output Compare 3 Preload enable */
#define GTIM_CCMR2_OC3M_SHIFT     (4)       /* Bits 6-4: Output Compare 3 Mode */
#define GTIM_CCMR2_OC3M_MASK      (7 << GTIM_CCMR2_OC3M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions above) */
#define GTIM_CCMR2_OC3CE          (1 << 7)  /* Bit 7: Output Compare 3 Clear Enable */
#define GTIM_CCMR2_CC4S_SHIFT     (8)       /* Bits 9-8: Capture/Compare 4 Selection */
#define GTIM_CCMR2_CC4S_MASK      (3 << GTIM_CCMR2_CC4S_SHIFT)
                                            /* (See common CCMR Capture/Compare Selection definitions above) */
#define GTIM_CCMR2_OC4FE          (1 << 10) /* Bit 10: Output Compare 4 Fast enable */
#define GTIM_CCMR2_OC4PE          (1 << 11) /* Bit 11: Output Compare 4 Preload enable */
#define GTIM_CCMR2_OC4M_SHIFT     (12)      /* Bits 14-12: Output Compare 4 Mode */
#define GTIM_CCMR2_OC4M_MASK      (7 << GTIM_CCMR2_OC4M_SHIFT)
                                            /* (See common CCMR Output Compare Mode definitions above) */
#define GTIM_CCMR2_OC4CE          (1 << 15) /* Bit 15: Output Compare 4 Clear Enable */

/* Capture/compare mode register 2 - Input capture mode (TIM2-5 only) */

                                            /* Bits 1-0 (Same as Output Compare Mode) */
#define GTIM_CCMR2_IC3PSC_SHIFT   (2)       /* Bits 3-2: Input Capture 3 Prescaler */
#define GTIM_CCMR2_IC3PSC_MASK    (3 << GTIM_CCMR2_IC3PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR2_IC3F_SHIFT     (4)       /* Bits 7-4: Input Capture 3 Filter */
#define GTIM_CCMR2_IC3F_MASK      (0x0f << GTIM_CCMR2_IC3F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */
                                            /* Bits 9-8: (Same as Output Compare Mode) */
#define GTIM_CCMR2_IC4PSC_SHIFT   (10)      /* Bits 11-10: Input Capture 4 Prescaler */
#define GTIM_CCMR2_IC4PSC_MASK    (3 << GTIM_CCMR2_IC4PSC_SHIFT)
                                            /* (See common CCMR Input Capture Prescaler definitions below) */
#define GTIM_CCMR2_IC4F_SHIFT     (12)      /* Bits 15-12: Input Capture 4 Filter */
#define GTIM_CCMR2_IC4F_MASK      (0x0f << GTIM_CCMR2_IC4F_SHIFT)
                                            /* (See common CCMR Input Capture Filter definitions below) */

/* Capture/compare enable register (TIM2-5 and TIM9-14) */

#define GTIM_CCER_CC1E            (1 << 0)  /* Bit 0: Capture/Compare 1 output enable */
#define GTIM_CCER_CC1P            (1 << 1)  /* Bit 1: Capture/Compare 1 output Polarity */

#ifdef CONFIG_STM32_STM32F40XX
#  define GTIM_CCER_CC1NP         (1 << 3)  /* Bit 3: Capture/Compare 1 output Polarity */
#endif

#define GTIM_CCER_CC2E            (1 << 4)  /* Bit 4: Capture/Compare 2 output enable (TIM2-5,9&12 only) */
#define GTIM_CCER_CC2P            (1 << 5)  /* Bit 5: Capture/Compare 2 output Polarity (TIM2-5,9&12 only) */

#ifdef CONFIG_STM32_STM32F40XX
#  define GTIM_CCER_CC2NP         (1 << 7)  /* Bit 7: Capture/Compare 2 output Polarity (TIM2-5,9&12 only) */
#endif

#define GTIM_CCER_CC3E            (1 << 8)  /* Bit 8: Capture/Compare 3 output enable (TIM2-5 only) */
#define GTIM_CCER_CC3P            (1 << 9)  /* Bit 9: Capture/Compare 3 output Polarity (TIM2-5 only) */

#ifdef CONFIG_STM32_STM32F40XX
#  define GTIM_CCER_CC3NP         (1 << 11) /* Bit 11: Capture/Compare 3 output Polarity (TIM2-5 only) */
#endif

#define GTIM_CCER_CC4E            (1 << 12) /* Bit 12: Capture/Compare 4 output enable (TIM2-5 only) */
#define GTIM_CCER_CC4P            (1 << 13) /* Bit 13: Capture/Compare 4 output Polarity (TIM2-5 only) */

#ifdef CONFIG_STM32_STM32F40XX
#  define GTIM_CCER_CC4NP         (1 << 15) /* Bit 15: Capture/Compare 4 output Polarity (TIM2-5 only) */
#endif

/* DMA control register */

#define GTIM_DCR_DBL_SHIFT        (8)       /* Bits 12-8: DMA Burst Length */
#define GTIM_DCR_DBL_MASK         (0x1f << GTIM_DCR_DBL_SHIFT)
#define GTIM_DCR_DBA_SHIFT        (0)       /* Bits 4-0: DMA Base Address */
#define GTIM_DCR_DBA_MASK         (0x1f << GTIM_DCR_DBA_SHIFT)

/* Timer 2/5 option register */

#ifdef CONFIG_STM32_STM32F40XX
#  define TIM2_OR_ITR1_RMP_SHIFT  (10)     /* Bits 10-11: Internal trigger 1 remap */
#  define TIM2_OR_ITR1_RMP_MASK   (3 << TIM2_OR_ITR1_RMP_SHIFT)
#    define TIM2_OR_ITR1_TIM8_TRGOUT (0 << TIM2_OR_ITR1_RMP_SHIFT) /* 00: TIM2_ITR1 input connected to TIM8_TRGOUT */
#    define TIM2_OR_ITR1_PTP      (1 << TIM2_OR_ITR1_RMP_SHIFT) /* 01: TIM2_ITR1 input connected to PTP trigger output */
#    define TIM2_OR_ITR1_OTGFSSOF (2 << TIM2_OR_ITR1_RMP_SHIFT) /* 10: TIM2_ITR1 input connected to OTG FS SOF */
#    define TIM2_OR_ITR1_OTGHSSOF (3 << TIM2_OR_ITR1_RMP_SHIFT) /* 11: TIM2_ITR1 input connected to OTG HS SOF */

#  define TIM5_OR_TI4_RMP_SHIFT   (6)     /* Bits 6-7: Internal trigger 4 remap */
#  define TIM5_OR_TI4_RMP_MASK    (3 << TIM5_OR_TI4_RMP_SHIFT)
#    define TIM5_OR_TI4_GPIO      (0 << TIM5_OR_TI4_RMP_SHIFT) /* 00: TIM5_CH4 input connected to GPIO */
#    define TIM5_OR_TI4_LSI       (1 << TIM5_OR_TI4_RMP_SHIFT) /* 01: TIM5_CH4 input connected to LSI internal clock */
#    define TIM5_OR_TI4_LSE       (2 << TIM5_OR_TI4_RMP_SHIFT) /* 10: TIM5_CH4 input connected to LSE internal clock */
#    define TIM5_OR_TI4_RTC       (3 << TIM5_OR_TI4_RMP_SHIFT) /* 11: TIM5_CH4 input connected to RTC output event */

#  define TIM11_OR_TI1_RMP_SHIFT  (6)     /* Bits 6-7: Internal trigger 4 remap */
#  define TIM11_OR_TI1_RMP_MASK   (3 << TIM11_OR_TI1_RMP_SHIFT)
#    define TIM11_OR_TI1_GPIO     (0 << TIM11_OR_TI1_RMP_SHIFT) /* 00-11: TIM11_CH1 input connected to GPIO */
#    define TIM11_OR_TI1_HSERTC   (3 << TIM11_OR_TI1_RMP_SHIFT) /* 11: TIM11_CH1 input connected to HSE_RTC clock */
#endif

/* Control register 1 */

#define BTIM_CR1_CEN              (1 << 0)  /* Bit 0: Counter enable */
#define BTIM_CR1_UDIS             (1 << 1)  /* Bit 1: Update Disable */
#define BTIM_CR1_URS              (1 << 2)  /* Bit 2: Update Request Source */
#define BTIM_CR1_OPM              (1 << 3)  /* Bit 3: One Pulse Mode */
#define BTIM_CR1_ARPE             (1 << 7)  /* Bit 7: Auto-Reload Preload enable */

/* Control register 2 */

#define BTIM_CR2_MMS_SHIFT        (4)       /* Bits 6-4: Master Mode Selection */
#define BTIM_CR2_MMS_MASK         (7 << BTIM_CR2_MMS_SHIFT)
#  define BTIM_CR2_RESET          (0 << BTIM_CR2_MMS_SHIFT) /* 000: Reset */
#  define BTIM_CR2_ENAB           (1 << BTIM_CR2_MMS_SHIFT) /* 001: Enable */
#  define BTIM_CR2_UPDT           (2 << BTIM_CR2_MMS_SHIFT) /* 010: Update */

/* DMA/Interrupt enable register */

#define BTIM_DIER_UIE             (1 << 0)  /* Bit 0: Update interrupt enable */
#define BTIM_DIER_UDE             (1 << 8)  /* Bit 8: Update DMA request enable */

/* Status register */

#define BTIM_SR_UIF               (1 << 0)  /* Bit 0: Update interrupt flag */

/* Event generation register */

#define BTIM_EGR_UG               (1 << 0)  /* Bit 0: Update generation */

#endif /* __ARCH_ARM_SRC_STM32_CHIP_STM32_TIM_H */

