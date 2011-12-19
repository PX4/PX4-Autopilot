/****************************************************************************************************
 * configs/stm3240g_eval/src/stm3240g_internal.h
 * arch/arm/src/board/stm3240g_internal.n
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
 ****************************************************************************************************/

#ifndef __CONFIGS_STM3240G_EVAL_SRC_STM3240G_INTERNAL_H
#define __CONFIGS_STM3240G_EVAL_SRC_STM3240G_INTERNAL_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

/* How many SPI modules does this chip support? */

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#  undef CONFIG_STM32_SPI3
#elif STM32_NSPI < 3
#  undef CONFIG_STM32_SPI3
#endif

/* STM3240G-EVAL GPIOs ******************************************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN6)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTG|GPIO_PIN8)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTI|GPIO_PIN9)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN7)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_WAKEUP
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  (BUTTON_USER - BUTTON_WAKEUP + 1)

#define GPIO_BTN_WAKEUP (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)
#define GPIO_BTN_TAMPER (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)
#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTG|GPIO_PIN15)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/* GPIO settings that will be altered when external memory is selected:
 *
 * ----- ------- -------- ------------ ------- ------------ -------- ----------- -------- -------------
 * PB7:  FSMC NL PD0-1:   FSMC D2-D3   PE0:    FSMC NBL0    PF0-5:   FSMC A0-A5  PG0-5:   FSMC A10-A15
 *               PD3:     FSMC CLK     PE1:    FSMC BLN1    PF6:     FSMC NIORD  PG6-7:   FSMC INT2-3
 *               PD4:     FSMC NOE     PE2:    FSMC A23     PF7:     FSMC NREG   PG9:     FSMC NCE3
 *               PD5:     FSMC NWE     PE3-6:  FSMC A19-A22 PF8:     FSMC NIOWR  PG9-10:  FSMC NE2-3
 *               PD6:     FSMC NWAIT   PE7-15: FSMC D4-D12  PF9:     FSMC CD     PG10:    FSMC NCE4 (1)
 *               PD7:     FSMC NE1                          PF10:    FSMC INTR   PG11:    FSMC NCE4 (2)
 *               PD7:     FSMC NCE2                         PF12-15: FSMC A6-A9  PG12:    FSMC NE4
 *               PD8-10:  FSMC D13-D15                                           PG13-14: FSMC A24-A25
 *               PD11-13: FSMC_A16-A18
 *               PD14-15: FSMC D0-D1
 */

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the STM3240G-EVAL board.
 *
 ****************************************************************************************************/

extern void weak_function stm32_spiinitialize(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM3240G_EVAL_SRC_STM3240G_INTERNAL_H */

