/************************************************************************************
 * configs/lm3s6432-s2e/src/lm3s6432s2e_internal.h
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

#ifndef __CONFIGS_LM3S6432_S2E_SRC_LM3S6432S2E_INTERNAL_H
#define __CONFIGS_LM3S6432_S2E_SRC_LM3S6432S2E_INTERNAL_H

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

/* How many SSI modules does this chip support? The LM3S6432 supports 1 SSI
 * module (others may support more than 2 -- in such case, the following must be
 * expanded).
 */

#if LM_NSSI == 0
#  undef CONFIG_SSI0_DISABLE
#  define CONFIG_SSI0_DISABLE 1
#endif
#undef CONFIG_SSI1_DISABLE
#define CONFIG_SSI1_DISABLE 1

/* LM3S6432 MDL-S2E *****************************************************************/

/* GPIO Usage
 *
 * PIN SIGNAL            S2E Function
 * --- ----------------- ---------------------------------------
 *  L3 PA0/U0RX          Virtual COM port receive
 *  M3 PA1/U0TX          Virtual COM port transmit
 * E12 PB0/U0CTS         Virtual COM port CTS
 * D12 PB1/U0RTS         Virtual COM port RTS
 *  L5 PA4/SPIRX         SPI receive
 *  M5 PA5/SPITX         SPI transmit
 *  H2 PD2/U1RX          Virtual COM port receive
 *  H1 PD3/U1TX          Virtual COM port transmit
 *  L4 PA3/U1CTS/SPICLK  Virtual COM port CTS
 *  M4 PA2/U1RTS/SPISEL  Virtual COM port RTS
 * J11 PF0/LED1          Ethernet LED1 (green)
 * J12 PF1/LED0          Ethernet LED0 (yellow)
 * C11 PB2               Transciever #INVALID
 * C12 PB3               Transciever #ENABLE
 *  A6 PB4               Transciever ON
 *  B7 PB5               Transciever #OFF
 */

/* GPIO for LEDs:
 * - PF0: User LED
 */

#define LED1_GPIO     (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORTF | 2)
#define LED0_GPIO     (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTF | 3)

/* GPIO for SSI0 select
 */
#define SSICS_GPIO    (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE | GPIO_PORTA | 3)

/* GPIOs for the RS-232 transciever enable/disable.
 * Default state for these enables the transciever.
 */
#define XCVR_INV_GPIO (GPIO_FUNC_INPUT | GPIO_PORTB | 2)
#define XCVR_ENA_GPIO (GPIO_FUNC_OUTPUT | GPIO_VALUE_ZERO | GPIO_PORTB | 3)
#define XCVR_ON_GPIO  (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORTB | 4)
#define XCVR_OFF_GPIO (GPIO_FUNC_OUTPUT | GPIO_VALUE_ONE  | GPIO_PORTB | 5)

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Name: lm_ssiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the MDL-S2E.
 *
 ************************************************************************************/

extern void weak_function lm_ssiinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_LM3S6432_S2E_SRC_LM3S6432S2E_INTERNAL_H */

