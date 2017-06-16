/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * PX4ESCv1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

#if STM32_NSPI < 1
#  undef CONFIG_STM32_SPI1
#  undef CONFIG_STM32_SPI2
#elif STM32_NSPI < 2
#  undef CONFIG_STM32_SPI2
#endif

/* High-resolution timer
 */
#define HRT_TIMER               8       /* use timer8 for the HRT */
#define HRT_TIMER_CHANNEL       1       /* use capture/compare channel */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PA[03] PA3/TIM2_CH4/TIM5_CH4/TIM9_CH2/USART2_RX           17       RC_PWM
 *  PA[04] PA4/SPI1_NSS                                       20       OC_ADJ
 *  PA[05] PA5/TIM2_CH1/TIM2_ETR/TIM8_CH1                     21       EN_GATE
 *  PA[06] PA6/TIM1_BKIN/TIM3_CH1/TIM8_BKIN/SPI1_MISO         22       DC_CAL
 *
 */

#define GPIO_RC_PWM   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTA | GPIO_PIN3)
#define GPIO_OC_ADJ   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
		       GPIO_PORTA | GPIO_PIN4 | GPIO_OUTPUT_CLEAR)
#define GPIO_EN_GATE  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
		       GPIO_PORTA | GPIO_PIN5 | GPIO_OUTPUT_CLEAR)
#define GPIO_DC_CAL   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
		       GPIO_PORTA | GPIO_PIN6 | GPIO_OUTPUT_CLEAR)


/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PB[02] PB2/TIM2_CH4/SPI3_MOSI                             28       GAIN
 *  PB[03] PB3/TIM2_CH2/I2C2_SDA/SPI1_SCK                     55       TEST2
 *  PB[04] PB4/TIM3_CH1/I2C3_SDA/SPI1_MISO/SPI3_MISO          56       TEST3
 *
 */

#define GPIO_GAIN     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
		       GPIO_PORTB | GPIO_PIN2 | GPIO_OUTPUT_CLEAR)
#define GPIO_TEST2     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
			GPIO_PORTB | GPIO_PIN3 | GPIO_OUTPUT_CLEAR)
#define GPIO_TEST3     (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
			GPIO_PORTB | GPIO_PIN4 | GPIO_OUTPUT_CLEAR)

/* CAN ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PB[05] PB5/TIM3_CH2/SPI1_MOSI/CAN2_RX                     57       CAN2_RX
 *  PB[06] PB6/TIM4_CH1/USART1_TX/CAN2_TX                     58       CAN2_TX
 *  PB[07] PB7/TIM2_CH2/TIM4_CH4/TIM11_CH1/I2C1_SDA           59       WAIT_GETNODEINFO
 *  PB[08] PB8/TIM4_CH3/I2C1_SCL/CAN1_RX                      61       CAN1_RX
 *  PB[09] PB9/TIM4_CH4/I2C1_SDA/CAN1_TX                      62       CAN1_TX
 *
 */

#define GPIO_WAIT_GETNODEINFO   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTB | GPIO_PIN7)

/* UART3 ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PB[10] PB10/TIM2_CH3/I2C2_SCL/SPI2_SCK/I2S2_CK/USART3_TX  29       DBG_TX
 *  PC[05] PC5/USART3_RX/SPDIFRX_IN3/FMC_SDCKE0               25       DBG_RX
 *
 */


/* Analog ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[00] PC0/ADC123_IN10                                   8       TEMP_SENS
 *  PC[01] PC1/ADC123_IN11/SPI3_MOSI/SPI2_MOSI               9       VBAT_SENS
 *  PC[02] PC2/ADC123_IN12/SPI2_MISO,                       10       CURR_SENS2
 *  PC[03] PC3/ADC123_IN13/SPI2_MOSI                        11       CURR_SENS1
 *
 */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[06] TIM3_CH1/TIM8_CH1/USART6_TX                      37         RPM
 *
 */

#define GPIO_RPM   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN6)

/* LEDs ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[07] PC7/TIM3_CH2/TIM8_CH2                            38       LED_RED
 *  PC[08] PC8/TIM3_CH3/TIM8_CH3                            39       LED_GREEN
 *  PC[09] PC9/TIM3_CH4/TIM8_CH4                            40       LED_BLUE
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
			 GPIO_PORTC | GPIO_PIN7 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_RED    GPIO_LED1

#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
			 GPIO_PORTC | GPIO_PIN8 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_GREEN  GPIO_LED2

#define GPIO_LED3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
			 GPIO_PORTC | GPIO_PIN9 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_BLUE  GPIO_LED3


/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC[10] PC10/SPI3_SCK/USART3_TX/UART4_TX/                51       HWID0
 *  PC[11] PC11/SPI3_MISO/USART3_RX/UART4_RX                52       HWID1
 *  PC[12] PC12/I2C2_SDA/SPI3_MOSI/USART3_CK/UART5_TX       53       TEST4
 *  PC[13] PC13/TAMP_1/WKUP1                                 2       PWRGD
 *  PC[14] PC14/OSC32_IN                                     3       OCTW
 *  PC[15] PC15/OSC32_OUT                                    4       FAULT
 *
 */

#define GPIO_HWID0   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN10)
#define GPIO_HWID1   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN11)
#define GPIO_TEST4   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
		      GPIO_PORTC | GPIO_PIN12 | GPIO_OUTPUT_CLEAR)

#define GPIO_PWRGD   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN13)
#define GPIO_OCTW    (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN14)
#define GPIO_FAULT   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTC | GPIO_PIN15)

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PD[02] PD2/TIM3_ETR/UART5_RX/SDIO_CMD                   54       TEST1
 */
#define GPIO_TEST1   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
		      GPIO_PORTD | GPIO_PIN2 | GPIO_OUTPUT_CLEAR)

#define BOARD_NAME "PX4ESC_V1"

__BEGIN_DECLS

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
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 ************************************************************************************/

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2) || \
    defined(CONFIG_STM32_SPI3)
void weak_function board_spiinitialize(void);
#endif

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins.
 *
 ************************************************************************************/

void stm32_usbinitialize(void);

/************************************************************************************
 * Name: stm32_usb_set_pwr_callback()
 *
 * Description:
 *   Called to setup set a call back for USB power state changes.
 *
 * Inputs:
 *   pwr_changed_handler: An interrupt handler that will be called on VBUS power
 *   state changes.
 *
 ************************************************************************************/

void stm32_usb_set_pwr_callback(xcpt_t pwr_changed_handler);

/****************************************************************************
 * Name: stm32_led_initialize
 *
 * Description:
 *   This functions is called very early in initialization to perform board-
 *   specific initialization of LED-related resources.  This includes such
 *   things as, for example, configure GPIO pins to drive the LEDs and also
 *   putting the LEDs in their correct initial state.
 *
 *   NOTE: In most architectures, LED initialization() is called from
 *   board-specific initialization and should, therefore, have the name
 *   <arch>_led_intialize().  But there are a few architectures where the
 *   LED initialization function is still called from common chip
 *   architecture logic.  This interface is not, however, a common board
 *   interface in any event and the name board_autoled_initialization is
 *   deprecated.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_LEDS
void board_autoled_initialize(void);
#endif

/************************************************************************************
 * Name: stm32_can_initialize
 *
 * Description:
 *   Called at application startup time to initialize the CAN functionality.
 *
 ************************************************************************************/

#if defined(CONFIG_CAN) && (defined(CONFIG_STM32_CAN1) || defined(CONFIG_STM32_CAN2))
int board_can_initialize(void);
#endif

/************************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   Called at application startup time to initialize the Buttons functionality.
 *
 ************************************************************************************/

#if defined(CONFIG_ARCH_BUTTONS)
void board_button_initialize(void);
#endif

/****************************************************************************
 * Name: usbmsc_archinitialize
 *
 * Description:
 *   Called from the application system/usbmc or the boards_nsh if the
 *   application is not included.
 *   Perform architecture specific initialization.  This function must
 *   configure the block device to export via USB.  This function must be
 *   provided by architecture-specific logic in order to use this add-on.
 *
 ****************************************************************************/

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_SYSTEM_USBMSC)
int usbmsc_archinitialize(void);
#endif

/****************************************************************************
 * Name: composite_archinitialize
 *
 * Description:
 *   Called from the application system/composite or the boards_nsh if the
 *   application is not included.
 *   Perform architecture specific initialization.  This function must
 *   configure the block device to export via USB.  This function must be
 *   provided by architecture-specific logic in order to use this add-on.
 *
 ****************************************************************************/

#if !defined(CONFIG_NSH_BUILTIN_APPS) && !defined(CONFIG_SYSTEM_COMPOSITE)
extern int composite_archinitialize(void);
#endif

#include "../common/board_common.h"

#endif /* __ASSEMBLY__ */

__END_DECLS
