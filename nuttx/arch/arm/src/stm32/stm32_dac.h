/************************************************************************************
 * arch/arm/src/stm32/stm32_dac.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_DAC_H
#define __ARCH_ARM_SRC_STM32_STM32_DAC_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/stm32_dac.h"

#include <nuttx/analog/dac.h>

/************************************************************************************
 * Pre-processor definitions
 ************************************************************************************/
/* Configuration ********************************************************************/
/* Timer devices may be used for different purposes.  One special purpose is to
 * control periodic DAC outputs.  If CONFIG_STM32_TIMn is defined then 
 * CONFIG_STM32_TIMn_DAC must also be defined to indicate that timer "n" is intended
 * to be used for that purpose.
 */

#ifndef CONFIG_STM32_TIM1
#  undef CONFIG_STM32_TIM1_DAC
#endif
#ifndef CONFIG_STM32_TIM2
#  undef CONFIG_STM32_TIM2_DAC
#endif
#ifndef CONFIG_STM32_TIM3
#  undef CONFIG_STM32_TIM3_DAC
#endif
#ifndef CONFIG_STM32_TIM4
#  undef CONFIG_STM32_TIM4_DAC
#endif
#ifndef CONFIG_STM32_TIM5
#  undef CONFIG_STM32_TIM5_DAC
#endif
#ifndef CONFIG_STM32_TIM6
#  undef CONFIG_STM32_TIM6_DAC
#endif
#ifndef CONFIG_STM32_TIM7
#  undef CONFIG_STM32_TIM7_DAC
#endif
#ifndef CONFIG_STM32_TIM8
#  undef CONFIG_STM32_TIM8_DAC
#endif
#ifndef CONFIG_STM32_TIM9
#  undef CONFIG_STM32_TIM9_DAC
#endif
#ifndef CONFIG_STM32_TIM10
#  undef CONFIG_STM32_TIM10_DAC
#endif
#ifndef CONFIG_STM32_TIM11
#  undef CONFIG_STM32_TIM11_DAC
#endif
#ifndef CONFIG_STM32_TIM12
#  undef CONFIG_STM32_TIM12_DAC
#endif
#ifndef CONFIG_STM32_TIM13
#  undef CONFIG_STM32_TIM13_DAC
#endif
#ifndef CONFIG_STM32_TIM14
#  undef CONFIG_STM32_TIM14_DAC
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: stm32_dacinitialize
 *
 * Description:
 *   Initialize the DAC
 *
 * Input Parameters:
 *   intf - The DAC interface number.
 *
 * Returned Value:
 *   Valid dac device structure reference on succcess; a NULL on failure
 *
 ****************************************************************************/

struct dac_dev_s;
EXTERN FAR struct dac_dev_s *stm32_dacinitialize(int intf);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_ARM_SRC_STM32_STM32_DAC_H */

