/************************************************************************************
 * configs/vsn/src/vsn.h
 * arch/arm/src/board/vsn.n
 *
 *   Copyright (c) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
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

#ifndef __CONFIGS_VSN_SRC_VSN_INTERNAL_H
#define __CONFIGS_VSN_SRC_VSN_INTERNAL_H

#include <nuttx/config.h>
#include <arch/board/board.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include "stm32.h"
#include "up_internal.h"
#include "up_arch.h"


/************************************************************************************
 * PIN Definitions
 ************************************************************************************/
 
 /* Board Peripheral Assignment
 * 
 * RS232/Power connector:
 *  - USART1, is the default bootloader and console
 * 
 * Sensor Connector:
 *  Digital:
 *  - GPIOs: PB10, PB11 (or even TIM2 CH3 and CH4)
 *  - USART3
 *  - I2C2
 *  Analog:
 *  - ADC1
 *  Supporting Analog Circuitry (not seen outside)
 *  - RefTap (TIM3_CH3)
 *  - Power PWM Out (TIM8_CH1 / TIM3_CH1)
 *  - Filtered Out (TIM3_CH4)
 *    (TIM8 could run at lower frequency, while TIM3 must run at highest possible)
 *  - Gain selection muxed with SDcard I/Os.
 * 
 * Radio connector:
 *  - UART3 / UART4
 *  - SPI2
 *  - I2C1 (remapped pins vs. Expansion connector)
 *  - CAN
 *  - TIM4 CH[3:4]
 * 
 * Expansion connector:
 *  - WakeUp Pin
 *  - System Wide Reset
 *  - SPI1 is wired to expansion port
 *  - I2C1
 *  - USART2 [Rx, Tx, CTS, RTS]
 *  - DAC [0:1]
 *  - ADC2 on pins [0:7]
 *  - TIM2 Channels [1:4]
 *  - TIM5 Channels [1:4]
 * 
 * Onboard Components:
 *  - SPI3 has direct connection with FRAM
 *  - SDCard, conencts the microSD and shares the control lines with Sensor Interface
 *    to select Amplifier Gain
 *  - ADC3 is used also for power management (can be shared with ADC1 on sensor connector
 *    if not used)
 */

/* LED */

#define GPIO_LED		(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN2 |GPIO_OUTPUT_CLEAR)
                         
/* BUTTON - Note that after a good second button causes hardware reset */

#define GPIO_PUSHBUTTON	(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN5 )

/* Power Management Pins */

#define GPIO_PVS		(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN7 |GPIO_OUTPUT_CLEAR)
#define GPIO_PST		(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN13)
#define GPIO_SCTC		(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTA|GPIO_PIN8 )
#define GPIO_PCLR		(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN1 )	// by default this pin is OSCOUT, requires REMAP
#define GPIO_XPWR		(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN4 )

/* FRAM (alt pins are not listed here and are a part of SPI) */

#define GPIO_FRAM_CS	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_50MHz|GPIO_PORTA|GPIO_PIN15|GPIO_OUTPUT_SET)

/* Sensor Interface */

#define GPIO_GP1_HIZ	(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#define GPIO_GP1_PUP	(GPIO_INPUT |GPIO_CNF_INPULLUP |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#define GPIO_GP1_PDN	(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN10)
#define GPIO_GP1_LOW	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN10|GPIO_OUTPUT_CLEAR)
#define GPIO_GP1_HIGH	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN10|GPIO_OUTPUT_SET)

#define GPIO_GP2_HIZ	(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#define GPIO_GP2_PUP	(GPIO_INPUT |GPIO_CNF_INPULLUP |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#define GPIO_GP2_PDN	(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN11)
#define GPIO_GP2_LOW	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN11|GPIO_OUTPUT_CLEAR)
#define GPIO_GP2_HIGH	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN11|GPIO_OUTPUT_SET)

#define GPIO_OPA_INPUT	(GPIO_INPUT |GPIO_CNF_ANALOGIN |GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN0 )
#define GPIO_OPA_ENABLE	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN1 |GPIO_OUTPUT_CLEAR)
#define GPIO_OPA_REFAIN	(GPIO_INPUT |GPIO_CNF_ANALOGIN |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN0 )
#define GPIO_OPA_REFPWM	(GPIO_ALT   |GPIO_CNF_AFPP     |GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN0 )

#define GPIO_OUT_PWRON  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN6 |GPIO_OUTPUT_CLEAR)
#define GPIO_OUT_PWROFF	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN6 |GPIO_OUTPUT_SET)
#define GPIO_OUT_PWRPWM	(GPIO_ALT   |GPIO_CNF_AFPP     |GPIO_MODE_10MHz|GPIO_PORTC|GPIO_PIN6 )
#define GPIO_OUT_PWRPWM_TIM8_CH     1   /* TIM8.CH1 */

#define GPIO_OUT_HIZ	(GPIO_INPUT |GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1 )
#define GPIO_OUT_PUP	(GPIO_INPUT |GPIO_CNF_INPULLUP |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1 )
#define GPIO_OUT_PDN	(GPIO_INPUT |GPIO_CNF_INPULLDWN|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1 )
#define GPIO_OUT_LOW	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN1 |GPIO_OUTPUT_CLEAR)
#define GPIO_OUT_HIGH	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTB|GPIO_PIN1 |GPIO_OUTPUT_SET)
#define GPIO_OUT_AIN	(GPIO_INPUT |GPIO_CNF_ANALOGIN |GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN1 )
#define GPIO_OUT_PWM	(GPIO_ALT   |GPIO_CNF_AFPP     |GPIO_MODE_10MHz|GPIO_PORTB|GPIO_PIN1 )
#define GPIO_OUT_PWM_TIM3_CH        4   /* TIM3.CH4 */ 

#define GPIO_PGIA_A0_H  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN8 |GPIO_OUTPUT_SET)
#define GPIO_PGIA_A0_L  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN8 |GPIO_OUTPUT_CLEAR)
#define GPIO_PGIA_A1_L  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN12|GPIO_OUTPUT_CLEAR)
#define GPIO_PGIA_A1_H  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN12|GPIO_OUTPUT_SET)
#define GPIO_PGIA_A2_H  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTD|GPIO_PIN2 |GPIO_OUTPUT_SET)
#define GPIO_PGIA_A2_L  (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTD|GPIO_PIN2 |GPIO_OUTPUT_CLEAR)
#define GPIO_PGIA_AEN   (GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_2MHz |GPIO_PORTC|GPIO_PIN1 |GPIO_OUTPUT_CLEAR)


/* Radio Connector */

#define GPIO_CC1101_CS	(GPIO_OUTPUT|GPIO_CNF_OUTPP    |GPIO_MODE_50MHz|GPIO_PORTB|GPIO_PIN12|GPIO_OUTPUT_SET)
#define GPIO_CC1101_GDO0 (GPIO_INPUT|GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTC|GPIO_PIN9 )
#define GPIO_CC1101_GDO2 (GPIO_INPUT|GPIO_CNF_INFLOAT  |GPIO_MODE_INPUT|GPIO_PORTD|GPIO_PIN0 )


/* Expansion Connector */


/************************************************************************************
 * Debugging
 ************************************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif


/************************************************************************************
 * Public data
 ************************************************************************************/

#ifndef __ASSEMBLY__

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the VSN board.
 *
 ************************************************************************************/

extern void weak_function stm32_spiinitialize(void);

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the VSN board.
 *
 ************************************************************************************/

extern void weak_function stm32_usbinitialize(void);

/************************************************************************************
 * Init Power Module and set board system voltage
 ************************************************************************************/

extern void board_power_init(void);

/************************************************************************************
 * Name: sysclock_select_hsi
 *
 * Description:
 *   Selects internal HSI Clock, SYSCLK = 36 MHz, HCLK = 36 MHz.
 *
 ************************************************************************************/

extern void sysclock_select_hsi(void);

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_VSN_SRC_VSN_INTERNAL_H */

