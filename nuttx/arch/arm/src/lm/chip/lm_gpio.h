/************************************************************************************
 * arch/arm/src/lm/chip/lm_gpio.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LM_CHIP_LM_GPIO_H
#define __ARCH_ARM_SRC_LM_CHIP_LM_GPIO_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* GPIO Register Offsets ************************************************************/

#define LM_GPIO_DATA_OFFSET         0x000 /* GPIO Data */
#define LM_GPIO_DIR_OFFSET          0x400 /* GPIO Direction */
#define LM_GPIO_IS_OFFSET           0x404 /* GPIO Interrupt Sense */
#define LM_GPIO_IBE_OFFSET          0x408 /* GPIO Interrupt Both Edges */
#define LM_GPIO_IEV_OFFSET          0x40c /* GPIO Interrupt Event */
#define LM_GPIO_IM_OFFSET           0x410 /* GPIO Interrupt Mask */
#define LM_GPIO_RIS_OFFSET          0x414 /* GPIO Raw Interrupt Status */
#define LM_GPIO_MIS_OFFSET          0x418 /* GPIO Masked Interrupt Status */
#define LM_GPIO_ICR_OFFSET          0x41c /* GPIO Interrupt Clear */
#define LM_GPIO_AFSEL_OFFSET        0x420 /* GPIO Alternate Function */
#define LM_GPIO_DR2R_OFFSET         0x500 /* Select GPIO 2-mA Drive Select */
#define LM_GPIO_DR4R_OFFSET         0x504 /* GPIO 4-mA Drive Select */
#define LM_GPIO_DR8R_OFFSET         0x508 /* GPIO 8-mA Drive Select */
#define LM_GPIO_ODR_OFFSET          0x50c /* GPIO Open Drain Select */
#define LM_GPIO_PUR_OFFSET          0x510 /* GPIO Pull-Up Select */
#define LM_GPIO_PDR_OFFSET          0x514 /* GPIO Pull-Down Select */
#define LM_GPIO_SLR_OFFSET          0x518 /* GPIO Slew Rate Control Select */
#define LM_GPIO_DEN_OFFSET          0x51C /* GPIO Digital Enable */
#define LM_GPIO_LOCK_OFFSET         0x520 /* GPIO Lock */
#define LM_GPIO_CR_OFFSET           0x524 /* GPIO Commit */

#ifdef LM4F
#  define LM_GPIO_AMSEL_OFFSET      0x528 /* GPIO Analog Mode Select */
#  define LM_GPIO_PCTL_OFFSET       0x52c /* GPIO Port Control */
#  define LM_GPIO_ADCCTL_OFFSET     0x530 /* GPIO ADC Control */
#  define LM_GPIO_DMACTL_OFFSET     0x534 /* GPIO DMA Control */
#endif

#define LM_GPIO_PERIPHID4_OFFSET    0xfd0 /* GPIO Peripheral Identification 4 */
#define LM_GPIO_PERIPHID5_OFFSET    0xfd4 /* GPIO Peripheral Identification 5 */
#define LM_GPIO_PERIPHID6_OFFSET    0xfd8 /* GPIO Peripheral Identification 6 */
#define LM_GPIO_PERIPHID7_OFFSET    0xfdc /* GPIO Peripheral Identification 7 */
#define LM_GPIO_PERIPHID0_OFFSET    0xfe0 /* GPIO Peripheral Identification 0 */
#define LM_GPIO_PERIPHID1_OFFSET    0xfe4 /* GPIO Peripheral Identification 1 */
#define LM_GPIO_PERIPHID2_OFFSET    0xfe8 /* GPIO Peripheral Identification 2 */
#define LM_GPIO_PERIPHID3_OFFSET    0xfec /* GPIO Peripheral Identification 3 */
#define LM_GPIO_PCELLID0_OFFSET     0xff0 /* GPIO PrimeCell Identification 0 */
#define LM_GPIO_PCELLID1_OFFSET     0xff4 /* GPIO PrimeCell Identification 1 */
#define LM_GPIO_PCELLID2_OFFSET     0xff8 /* GPIO PrimeCell Identification 2 */
#define LM_GPIO_PCELLID3_OFFSET     0xffc /* GPIO PrimeCell Identification 3*/

/* GPIO Register Addresses **********************************************************/

#if LM_NPORTS > 0

#  define LM_GPIOA_DATA             (LM_GPIOA_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOA_DIR              (LM_GPIOA_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOA_IS               (LM_GPIOA_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOA_IBE              (LM_GPIOA_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOA_IEV              (LM_GPIOA_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOA_IM               (LM_GPIOA_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOA_RIS              (LM_GPIOA_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOA_MIS              (LM_GPIOA_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOA_ICR              (LM_GPIOA_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOA_AFSEL            (LM_GPIOA_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOA_DR2R             (LM_GPIOA_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOA_DR4R             (LM_GPIOA_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOA_DR8R             (LM_GPIOA_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOA_ODR              (LM_GPIOA_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOA_PUR              (LM_GPIOA_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOA_PDR              (LM_GPIOA_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOA_SLR              (LM_GPIOA_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOA_DEN              (LM_GPIOA_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOA_LOCK             (LM_GPIOA_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOA_CR               (LM_GPIOA_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOA_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOA_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOA_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOA_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOA_PERIPHID4        (LM_GPIOA_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOA_PERIPHID5        (LM_GPIOA_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOA_PERIPHID6        (LM_GPIOA_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOA_PERIPHID7        (LM_GPIOA_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOA_PERIPHID0        (LM_GPIOA_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOA_PERIPHID1        (LM_GPIOA_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOA_PERIPHID2        (LM_GPIOA_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOA_PERIPHID3        (LM_GPIOA_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOA_PCELLID0         (LM_GPIOA_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOA_PCELLID1         (LM_GPIOA_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOA_PCELLID2         (LM_GPIOA_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOA_PCELLID3         (LM_GPIOA_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 1

#  define LM_GPIOB_DATA             (LM_GPIOB_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOB_DIR              (LM_GPIOB_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOB_IS               (LM_GPIOB_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOB_IBE              (LM_GPIOB_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOB_IEV              (LM_GPIOB_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOB_IM               (LM_GPIOB_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOB_RIS              (LM_GPIOB_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOB_MIS              (LM_GPIOB_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOB_ICR              (LM_GPIOB_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOB_AFSEL            (LM_GPIOB_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOB_DR2R             (LM_GPIOB_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOB_DR4R             (LM_GPIOB_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOB_DR8R             (LM_GPIOB_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOB_ODR              (LM_GPIOB_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOB_PUR              (LM_GPIOB_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOB_PDR              (LM_GPIOB_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOB_SLR              (LM_GPIOB_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOB_DEN              (LM_GPIOB_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOB_LOCK             (LM_GPIOB_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOB_CR               (LM_GPIOB_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOB_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOB_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOB_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOB_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOB_PERIPHID4        (LM_GPIOB_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOB_PERIPHID5        (LM_GPIOB_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOB_PERIPHID6        (LM_GPIOB_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOB_PERIPHID7        (LM_GPIOB_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOB_PERIPHID0        (LM_GPIOB_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOB_PERIPHID1        (LM_GPIOB_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOB_PERIPHID2        (LM_GPIOB_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOB_PERIPHID3        (LM_GPIOB_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOB_PCELLID0         (LM_GPIOB_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOB_PCELLID1         (LM_GPIOB_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOB_PCELLID2         (LM_GPIOB_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOB_PCELLID3         (LM_GPIOB_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 2

#  define LM_GPIOC_DATA             (LM_GPIOC_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOC_DIR              (LM_GPIOC_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOC_IS               (LM_GPIOC_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOC_IBE              (LM_GPIOC_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOC_IEV              (LM_GPIOC_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOC_IM               (LM_GPIOC_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOC_RIS              (LM_GPIOC_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOC_MIS              (LM_GPIOC_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOC_ICR              (LM_GPIOC_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOC_AFSEL            (LM_GPIOC_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOC_DR2R             (LM_GPIOC_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOC_DR4R             (LM_GPIOC_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOC_DR8R             (LM_GPIOC_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOC_ODR              (LM_GPIOC_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOC_PUR              (LM_GPIOC_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOC_PDR              (LM_GPIOC_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOC_SLR              (LM_GPIOC_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOC_DEN              (LM_GPIOC_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOC_LOCK             (LM_GPIOC_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOC_CR               (LM_GPIOC_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOC_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOC_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOC_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOC_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOC_PERIPHID4        (LM_GPIOC_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOC_PERIPHID5        (LM_GPIOC_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOC_PERIPHID6        (LM_GPIOC_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOC_PERIPHID7        (LM_GPIOC_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOC_PERIPHID0        (LM_GPIOC_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOC_PERIPHID1        (LM_GPIOC_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOC_PERIPHID2        (LM_GPIOC_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOC_PERIPHID3        (LM_GPIOC_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOC_PCELLID0         (LM_GPIOC_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOC_PCELLID1         (LM_GPIOC_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOC_PCELLID2         (LM_GPIOC_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOC_PCELLID3         (LM_GPIOC_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 3

#  define LM_GPIOD_DATA             (LM_GPIOD_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOD_DIR              (LM_GPIOD_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOD_IS               (LM_GPIOD_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOD_IBE              (LM_GPIOD_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOD_IEV              (LM_GPIOD_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOD_IM               (LM_GPIOD_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOD_RIS              (LM_GPIOD_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOD_MIS              (LM_GPIOD_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOD_ICR              (LM_GPIOD_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOD_AFSEL            (LM_GPIOD_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOD_DR2R             (LM_GPIOD_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOD_DR4R             (LM_GPIOD_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOD_DR8R             (LM_GPIOD_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOD_ODR              (LM_GPIOD_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOD_PUR              (LM_GPIOD_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOD_PDR              (LM_GPIOD_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOD_SLR              (LM_GPIOD_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOD_DEN              (LM_GPIOD_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOD_LOCK             (LM_GPIOD_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOD_CR               (LM_GPIOD_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOD_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOD_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOD_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOD_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOD_PERIPHID4        (LM_GPIOD_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOD_PERIPHID5        (LM_GPIOD_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOD_PERIPHID6        (LM_GPIOD_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOD_PERIPHID7        (LM_GPIOD_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOD_PERIPHID0        (LM_GPIOD_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOD_PERIPHID1        (LM_GPIOD_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOD_PERIPHID2        (LM_GPIOD_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOD_PERIPHID3        (LM_GPIOD_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOD_PCELLID0         (LM_GPIOD_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOD_PCELLID1         (LM_GPIOD_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOD_PCELLID2         (LM_GPIOD_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOD_PCELLID3         (LM_GPIOD_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 4

#  define LM_GPIOE_DATA             (LM_GPIOE_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOE_DIR              (LM_GPIOE_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOE_IS               (LM_GPIOE_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOE_IBE              (LM_GPIOE_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOE_IEV              (LM_GPIOE_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOE_IM               (LM_GPIOE_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOE_RIS              (LM_GPIOE_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOE_MIS              (LM_GPIOE_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOE_ICR              (LM_GPIOE_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOE_AFSEL            (LM_GPIOE_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOE_DR2R             (LM_GPIOE_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOE_DR4R             (LM_GPIOE_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOE_DR8R             (LM_GPIOE_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOE_ODR              (LM_GPIOE_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOE_PUR              (LM_GPIOE_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOE_PDR              (LM_GPIOE_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOE_SLR              (LM_GPIOE_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOE_DEN              (LM_GPIOE_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOE_LOCK             (LM_GPIOE_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOE_CR               (LM_GPIOE_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOE_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOE_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOE_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOE_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOE_PERIPHID4        (LM_GPIOE_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOE_PERIPHID5        (LM_GPIOE_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOE_PERIPHID6        (LM_GPIOE_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOE_PERIPHID7        (LM_GPIOE_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOE_PERIPHID0        (LM_GPIOE_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOE_PERIPHID1        (LM_GPIOE_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOE_PERIPHID2        (LM_GPIOE_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOE_PERIPHID3        (LM_GPIOE_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOE_PCELLID0         (LM_GPIOE_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOE_PCELLID1         (LM_GPIOE_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOE_PCELLID2         (LM_GPIOE_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOE_PCELLID3         (LM_GPIOE_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 5

#  define LM_GPIOF_DATA             (LM_GPIOF_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOF_DIR              (LM_GPIOF_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOF_IS               (LM_GPIOF_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOF_IBE              (LM_GPIOF_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOF_IEV              (LM_GPIOF_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOF_IM               (LM_GPIOF_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOF_RIS              (LM_GPIOF_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOF_MIS              (LM_GPIOF_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOF_ICR              (LM_GPIOF_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOF_AFSEL            (LM_GPIOF_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOF_DR2R             (LM_GPIOF_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOF_DR4R             (LM_GPIOF_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOF_DR8R             (LM_GPIOF_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOF_ODR              (LM_GPIOF_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOF_PUR              (LM_GPIOF_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOF_PDR              (LM_GPIOF_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOF_SLR              (LM_GPIOF_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOF_DEN              (LM_GPIOF_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOF_LOCK             (LM_GPIOF_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOF_CR               (LM_GPIOF_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOF_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOF_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOF_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOF_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOF_PERIPHID4        (LM_GPIOF_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOF_PERIPHID5        (LM_GPIOF_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOF_PERIPHID6        (LM_GPIOF_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOF_PERIPHID7        (LM_GPIOF_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOF_PERIPHID0        (LM_GPIOF_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOF_PERIPHID1        (LM_GPIOF_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOF_PERIPHID2        (LM_GPIOF_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOF_PERIPHID3        (LM_GPIOF_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOF_PCELLID0         (LM_GPIOF_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOF_PCELLID1         (LM_GPIOF_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOF_PCELLID2         (LM_GPIOF_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOF_PCELLID3         (LM_GPIOF_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 6

#  define LM_GPIOG_DATA             (LM_GPIOG_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOG_DIR              (LM_GPIOG_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOG_IS               (LM_GPIOG_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOG_IBE              (LM_GPIOG_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOG_IEV              (LM_GPIOG_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOG_IM               (LM_GPIOG_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOG_RIS              (LM_GPIOG_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOG_MIS              (LM_GPIOG_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOG_ICR              (LM_GPIOG_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOG_AFSEL            (LM_GPIOG_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOG_DR2R             (LM_GPIOG_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOG_DR4R             (LM_GPIOG_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOG_DR8R             (LM_GPIOG_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOG_ODR              (LM_GPIOG_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOG_PUR              (LM_GPIOG_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOG_PDR              (LM_GPIOG_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOG_SLR              (LM_GPIOG_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOG_DEN              (LM_GPIOG_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOG_LOCK             (LM_GPIOG_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOG_CR               (LM_GPIOG_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOG_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOG_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOG_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOG_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOG_PERIPHID4        (LM_GPIOG_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOG_PERIPHID5        (LM_GPIOG_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOG_PERIPHID6        (LM_GPIOG_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOG_PERIPHID7        (LM_GPIOG_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOG_PERIPHID0        (LM_GPIOG_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOG_PERIPHID1        (LM_GPIOG_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOG_PERIPHID2        (LM_GPIOG_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOG_PERIPHID3        (LM_GPIOG_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOG_PCELLID0         (LM_GPIOG_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOG_PCELLID1         (LM_GPIOG_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOG_PCELLID2         (LM_GPIOG_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOG_PCELLID3         (LM_GPIOG_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 7

#  define LM_GPIOH_DATA             (LM_GPIOH_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOH_DIR              (LM_GPIOH_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOH_IS               (LM_GPIOH_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOH_IBE              (LM_GPIOH_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOH_IEV              (LM_GPIOH_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOH_IM               (LM_GPIOH_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOH_RIS              (LM_GPIOH_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOH_MIS              (LM_GPIOH_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOH_ICR              (LM_GPIOH_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOH_AFSEL            (LM_GPIOH_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOH_DR2R             (LM_GPIOH_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOH_DR4R             (LM_GPIOH_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOH_DR8R             (LM_GPIOH_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOH_ODR              (LM_GPIOH_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOH_PUR              (LM_GPIOH_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOH_PDR              (LM_GPIOH_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOH_SLR              (LM_GPIOH_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOH_DEN              (LM_GPIOH_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOH_LOCK             (LM_GPIOH_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOH_CR               (LM_GPIOH_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOH_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOH_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOH_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOH_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOH_PERIPHID4        (LM_GPIOH_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOH_PERIPHID5        (LM_GPIOH_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOH_PERIPHID6        (LM_GPIOH_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOH_PERIPHID7        (LM_GPIOH_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOH_PERIPHID0        (LM_GPIOH_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOH_PERIPHID1        (LM_GPIOH_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOH_PERIPHID2        (LM_GPIOH_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOH_PERIPHID3        (LM_GPIOH_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOH_PCELLID0         (LM_GPIOH_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOH_PCELLID1         (LM_GPIOH_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOH_PCELLID2         (LM_GPIOH_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOH_PCELLID3         (LM_GPIOH_BASE + LM_GPIO_PCELLID3_OFFSET)

#elif LM_NPORTS > 8

#  define LM_GPIOJ_DATA             (LM_GPIOJ_BASE + LM_GPIO_DATA_OFFSET)
#  define LM_GPIOJ_DIR              (LM_GPIOJ_BASE + LM_GPIO_DIR_OFFSET)
#  define LM_GPIOJ_IS               (LM_GPIOJ_BASE + LM_GPIO_IS_OFFSET)
#  define LM_GPIOJ_IBE              (LM_GPIOJ_BASE + LM_GPIO_IBE_OFFSET)
#  define LM_GPIOJ_IEV              (LM_GPIOJ_BASE + LM_GPIO_IEV_OFFSET)
#  define LM_GPIOJ_IM               (LM_GPIOJ_BASE + LM_GPIO_IM_OFFSET)
#  define LM_GPIOJ_RIS              (LM_GPIOJ_BASE + LM_GPIO_RIS_OFFSET)
#  define LM_GPIOJ_MIS              (LM_GPIOJ_BASE + LM_GPIO_MIS_OFFSET)
#  define LM_GPIOJ_ICR              (LM_GPIOJ_BASE + LM_GPIO_ICR_OFFSET)
#  define LM_GPIOJ_AFSEL            (LM_GPIOJ_BASE + LM_GPIO_AFSEL_OFFSET)
#  define LM_GPIOJ_DR2R             (LM_GPIOJ_BASE + LM_GPIO_DR2R_OFFSET)
#  define LM_GPIOJ_DR4R             (LM_GPIOJ_BASE + LM_GPIO_DR4R_OFFSET)
#  define LM_GPIOJ_DR8R             (LM_GPIOJ_BASE + LM_GPIO_DR8R_OFFSET)
#  define LM_GPIOJ_ODR              (LM_GPIOJ_BASE + LM_GPIO_ODR_OFFSET)
#  define LM_GPIOJ_PUR              (LM_GPIOJ_BASE + LM_GPIO_PUR_OFFSET)
#  define LM_GPIOJ_PDR              (LM_GPIOJ_BASE + LM_GPIO_PDR_OFFSET)
#  define LM_GPIOJ_SLR              (LM_GPIOJ_BASE + LM_GPIO_SLR_OFFSET)
#  define LM_GPIOJ_DEN              (LM_GPIOJ_BASE + LM_GPIO_DEN_OFFSET)
#  define LM_GPIOJ_LOCK             (LM_GPIOJ_BASE + LM_GPIO_LOCK_OFFSET)
#  define LM_GPIOJ_CR               (LM_GPIOJ_BASE + LM_GPIO_CR_OFFSET)

#  ifdef LM4F
#    define LM_GPIOJ_AMSEL          (LM_GPIOA_BASE + LM_GPIO_AMSEL_OFFSET)
#    define LM_GPIOJ_PCTL           (LM_GPIOA_BASE + LM_GPIO_PCTL_OFFSET)
#    define LM_GPIOJ_ADCCTL         (LM_GPIOA_BASE + LM_GPIO_ADCCTL_OFFSET)
#    define LM_GPIOJ_DMACTL         (LM_GPIOA_BASE + LM_GPIO_DMACTL_OFFSET)
#  endif

#  define LM_GPIOJ_PERIPHID4        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID4_OFFSET)
#  define LM_GPIOJ_PERIPHID5        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID5_OFFSET)
#  define LM_GPIOJ_PERIPHID6        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID6_OFFSET)
#  define LM_GPIOJ_PERIPHID7        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID7_OFFSET)
#  define LM_GPIOJ_PERIPHID0        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID0_OFFSET)
#  define LM_GPIOJ_PERIPHID1        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID1_OFFSET)
#  define LM_GPIOJ_PERIPHID2        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID2_OFFSET)
#  define LM_GPIOJ_PERIPHID3        (LM_GPIOJ_BASE + LM_GPIO_PERIPHID3_OFFSET)
#  define LM_GPIOJ_PCELLID0         (LM_GPIOJ_BASE + LM_GPIO_PCELLID0_OFFSET)
#  define LM_GPIOJ_PCELLID1         (LM_GPIOJ_BASE + LM_GPIO_PCELLID1_OFFSET)
#  define LM_GPIOJ_PCELLID2         (LM_GPIOJ_BASE + LM_GPIO_PCELLID2_OFFSET)
#  define LM_GPIOJ_PCELLID3         (LM_GPIOJ_BASE + LM_GPIO_PCELLID3_OFFSET)

#endif /* LM_NPORTS */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LM_CHIP_LM_GPIO_H */
