/************************************************************************************
 * configs/stm32f100rc_generic/src/stm32f100rc_internal.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Freddie Chopin <freddie_chopin@op.pl>
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

#ifndef __CONFIGS_STM32F100RC_GENERIC_SRC_STM32F100RC_INTERNAL_H
#define __CONFIGS_STM32F100RC_GENERIC_SRC_STM32F100RC_INTERNAL_H

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* LED - assume it is on PA0 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN0)

/* BUTTON - assume it is on PA1 */

#define MIN_IRQBUTTON   BUTTON_0
#define MAX_IRQBUTTON   BUTTON_0
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_0      (GPIO_INPUT | GPIO_CNF_INFLOAT | GPIO_MODE_INPUT | GPIO_PORTA | GPIO_PIN1)

#endif /* __CONFIGS_STM32F100RC_GENERIC_SRC_STM32F100RC_INTERNAL_H */

