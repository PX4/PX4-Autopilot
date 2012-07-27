/****************************************************************************************************
 * configs/stm3240g_eval/src/stm3240g_internal.h
 * arch/arm/src/board/stm3240g_internal.n
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
/* Configuration ************************************************************************************/
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

/* STM32F4 Discovery GPIOs **************************************************************************/
/* LEDs */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN12)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN13)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN15)

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* PWM
 *
 * The STM32F4 Discovery has no real on-board PWM devices, but the board can be
 * configured to output a pulse train using TIM4 CH2 on PD13.
 */

#define STM3240G_EVAL_PWMTIMER   4
#define STM3240G_EVAL_PWMCHANNEL 2

/* SPI chip selects */

#define GPIO_CS_MEMS    (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)

/* USB OTG FS 
 *
 * PA9  OTG_FS_VBUS VBUS sensing (also connected to the green LED)
 * PC0  OTG_FS_PowerSwitchOn
 * PD5  OTG_FS_Overcurrent
 */

#define GPIO_OTGFS_VBUS (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_OPENDRAIN|GPIO_PORTA|GPIO_PIN9)
#define GPIO_OTGFS_PWRON (GPIO_OUTPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTC|GPIO_PIN0)
#define GPIO_OTGFS_OVER  (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_100MHz|GPIO_PUSHPULL|GPIO_PORTD|GPIO_PIN5)

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

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
 *   Called to configure SPI chip select GPIO pins for the stm32f4discovery board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the STM3210E-EVAL board.
 *
 ************************************************************************************/

void weak_function stm32_usbinitialize(void);

/************************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for external memory usage
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemgpios(const uint32_t *gpios, int ngpios);
#endif

/************************************************************************************
 * Name: stm32_extmemaddr
 *
 * Description:
 *   Initialize adress line GPIOs for external memory access
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemaddr(int naddrs);
#endif

/************************************************************************************
 * Name: stm32_extmemdata
 *
 * Description:
 *   Initialize data line GPIOs for external memory access
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_extmemdata(int ndata);
#endif

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_enablefsmc(void);
#endif

/************************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
void stm32_disablefsmc(void);
#endif

/****************************************************************************
 * Name: up_ledpminitialize
 ****************************************************************************/

#ifdef CONFIG_PM
void up_ledpminitialize(void);
#endif

/****************************************************************************
 * Name: up_pmbuttons
 *
 * Description:
 *   Configure the user button of the STM32f4discovery board as EXTI,
 *   so it is able to wakeup the MCU from the PM_STANDBY mode
 *
 ****************************************************************************/

#if defined(CONFIG_PM) && defined(CONFIG_IDLE_CUSTOM) && defined(CONFIG_PM_BUTTONS)
void up_pmbuttons(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_STM3240G_EVAL_SRC_STM3240G_INTERNAL_H */

