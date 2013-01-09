/************************************************************************************
 * configs/eagle100/src/eagle100_internal.h
 * arch/arm/src/board/eagle100_internal.n
 *
 *   Copyright (C) 2009-2010 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_EAGLE100_SRC_EAGLE100_INTERNAL_H
#define __CONFIGS_EAGLE100_SRC_EAGLE100_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include "chip.h"
#include "lm_gpio.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* How many SSI modules does this chip support? The LM3S6918 supports 2 SSI
 * modules (others may support more -- in such case, the following must be
 * expanded).
 */

#if LM_NSSI == 0
#  undef CONFIG_SSI0_DISABLE
#  define CONFIG_SSI0_DISABLE 1
#  undef CONFIG_SSI1_DISABLE
#  define CONFIG_SSI1_DISABLE 1
#elif LM_NSSI == 1
#  undef CONFIG_SSI1_DISABLE
#  define CONFIG_SSI1_DISABLE 1
#endif

/* Eagle-100 GPIOs ******************************************************************/

/* GPIO for microSD card chip select */

#define SDCCS_GPIO (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                    GPIO_VALUE_ONE | GPIO_PORTG | 1)
#define LED_GPIO   (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTE | 1)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: lm_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Eagle100 board.
 *
 ************************************************************************************/

extern void weak_function lm_ssiinitialize(void);

/****************************************************************************
 * Name: up_ledinit
 *
 * Description:
 *   Initialize on-board LEDs.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinit(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_EAGLE100_SRC_EAGLE100_INTERNAL_H */

