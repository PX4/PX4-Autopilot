/************************************************************************************
 * configs/avr32dev1/src/avr32dev1_internal.h
 * arch/avr/src/board/avr32dev1_internal.n
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

#ifndef _CONFIGS_AVR32DEV1_SRC_AVR32DEV1_INTERNAL_H
#define _CONFIGS_AVR32DEV1_SRC_AVR32DEV1_INTERNAL_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include "at32uc3_config.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Configuration ********************************************************************/

#if (CONFIG_AVR32_GPIOIRQSETB & 4) == 1
#  define CONFIG_AVR32DEV_BUTTON1_IRQ 1
#endif 

#if (CONFIG_AVR32_GPIOIRQSETB & 8) == 1
#  define CONFIG_AVR32DEV_BUTTON2_IRQ 1
#endif 

/* AVRDEV1 GPIO Pin Definitions *****************************************************/
/* LEDs
 * 
 * The AVR32DEV1 board has 3 LEDs, two of which can be controlled through GPIO pins.
 *
 * PIN 13  PA7  LED1
 * PIN 14  PA8  LED2
 */

#define PINMUX_GPIO_LED1 (GPIO_ENABLE | GPIO_OUTPUT | GPIO_LOW | GPIO_PORTA | 7)
#define PINMUX_GPIO_LED2 (GPIO_ENABLE | GPIO_OUTPUT | GPIO_LOW | GPIO_PORTA | 8)

/* BUTTONs
 *
 * The AVR32DEV1 board has 3 BUTTONs, two of which can be sensed through GPIO pins.
 *
 * PIN 24  PB2  KEY1
 * PIN 25  PB3  KEY2
 */

#if CONFIG_AVR32DEV_BUTTON1_IRQ
#  define PINMUX_GPIO_BUTTON1 (GPIO_ENABLE | GPIO_INPUT | GPIO_INTR | \
                               GPIO_INTMODE_BOTH | GPIO_GLITCH | GPIO_PORTB | 2)
#  define GPIO_BUTTON1_IRQ    AVR32_IRQ_GPIO_PB2
#else
#  define PINMUX_GPIO_BUTTON1 (GPIO_ENABLE | GPIO_INPUT | GPIO_GLITCH | \
                               GPIO_PORTB | 2)
#endif

#if CONFIG_AVR32DEV_BUTTON2_IRQ
#  define PINMUX_GPIO_BUTTON2 (GPIO_ENABLE | GPIO_INPUT | GPIO_INTR | \
                               GPIO_INTMODE_BOTH | GPIO_GLITCH | GPIO_PORTB | 3)
#  define GPIO_BUTTON2_IRQ    AVR32_IRQ_GPIO_PB3
#else
#  define PINMUX_GPIO_BUTTON2 (GPIO_ENABLE | GPIO_INPUT | GPIO_GLITCH | \
                               GPIO_PORTB | 3)
#endif

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: up_ledinitialize
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
extern void up_ledinitialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* _CONFIGS_AVR32DEV1_SRC_AVR32DEV1_INTERNAL_H */

