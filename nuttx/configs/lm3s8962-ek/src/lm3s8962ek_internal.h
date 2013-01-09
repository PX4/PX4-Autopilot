/************************************************************************************
 * configs/lm3s8962-ek/src/lm3s8962ek_internal.h
 * arch/arm/src/board/lm3s8962ek_internal.n
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_LM3S8962_EK_SRC_LM3S8962EK_INTERNAL_H
#define __CONFIGS_LM3S8962_EK_SRC_LM3S8962EK_INTERNAL_H

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

/* How many SSI modules does this chip support? The LM3S8962 supports 1 SSI
 * module (others may support more than 2 -- in such case, the following must be
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

/* LM3S8962 Eval Kit ***************************************************************/

/* GPIO Usage
 *
 * PIN SIGNAL      EVB Function
 * --- ----------- ---------------------------------------
 *  26 PA0/U0RX    Virtual COM port receive
 *  27 PA1/U0TX    Virtual COM port transmit
 *  10 PD0/IDX0    SD card chip select
 *  11 PD1/PWM1    Sound
 *  30 PA4/SSI0RX  SD card data out
 *  31 PA5/SSI0TX  SD card and OLED display data in
 *  28 PA2/SSI0CLK SD card and OLED display clock
 *  22 PC7/PHB0    OLED display data/control select
 *  29 PA3/SSI0FSS OLED display chip select
 *  73 PE1/PWM5    Down switch
 *  74 PE2/PHB1    Left switch
 *  72 PE0/PWM4    Up switch
 *  75 PE3/PHA1    Right switch
 *  61 PF1/IDX1    Select switch
 *  47 PF0/PWM0    User LED
 *  23 PC6/CCP3    Enable +15 V
 */

/* GPIO for microSD card chip select:
 * - PD0: SD card chip select (CARDCSn)
 */

#define SDCCS_GPIO  (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                     GPIO_VALUE_ONE | GPIO_PORTD | 0)

/* GPIO for single LED:
 * - PF0: User LED
 */

#define LED_GPIO    (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTF | 0)

/* GPIOs for OLED:
 *  - PC7: OLED display data/control select (D/Cn)
 *  - PA3: OLED display chip select (CSn)
 *  - PC6: Enable +15V needed by OLED (EN+15V)
 */

#define OLEDDC_GPIO (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STD | GPIO_STRENGTH_8MA | \
                     GPIO_VALUE_ONE | GPIO_PORTC | 7)
#define OLEDCS_GPIO (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STDWPU | GPIO_STRENGTH_4MA | \
                     GPIO_VALUE_ONE | GPIO_PORTA | 3)
#define OLEDEN_GPIO (GPIO_FUNC_OUTPUT | GPIO_PADTYPE_STD | GPIO_STRENGTH_8MA | \
                     GPIO_VALUE_ONE | GPIO_PORTC | 6)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: lm_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LM3S8962 Eval Kit.
 *
 ************************************************************************************/

extern void weak_function lm_ssiinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_LM3S8962_EK_SRC_LM3S8962EK_INTERNAL_H */

