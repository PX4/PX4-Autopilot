/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * ESC35v1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/config.h>
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
#define HRT_TIMER               3       /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       1       /* use capture/compare channel */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PB[6] PB06                                                42       nCAN_SILENT
 */

#define GPIO_CAN_SILENT (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN6 | GPIO_OUTPUT_CLEAR)

#define GPIO_RC_PWM   (GPIO_INPUT | GPIO_PULLUP | GPIO_PORTA | GPIO_PIN0)

#define GPIO_OC_ADJ   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN4 | GPIO_OUTPUT_CLEAR)
#define GPIO_EN_GATE  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN5 | GPIO_OUTPUT_CLEAR)
#define GPIO_DC_CAL   (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN6 | GPIO_OUTPUT_CLEAR)


/* CAN ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PB[8] PB8/CAN_RX                                          45       CAN_RX
 *  PB[9] PB9/CAN_TX                                          46       CAN_TX
 *
 */
#define GPIO_CAN1_RX		GPIO_CAN_RX_3
#define GPIO_CAN1_TX		GPIO_CAN_TX_3

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

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN2 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_RED    GPIO_LED1

#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN3 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_GREEN  GPIO_LED2

#define GPIO_LED3       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN1 | GPIO_OUTPUT_CLEAR)
#define GPIO_LED_BLUE  GPIO_LED3

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

#include <drivers/boards/common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
