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
 * PX4FLOWv1 internal definitions
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

__BEGIN_DECLS

/* these headers are not C++ safe */
#include <stm32.h>
#include <arch/board/board.h>

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
#define HRT_TIMER               1       /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       1       /* use capture/compare channel */
#define HRT_PPM_CHANNEL         3       /* use capture/compare channel 3 */
#define GPIO_PPM_IN             (GPIO_ALT|GPIO_CNF_INPULLUP|GPIO_PORTB|GPIO_PIN12)

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PA[00] PA0/TIM2_CH1/TIM5_CH1/USART2_CTS/UART4_TX         23       TIM5_CH1
 *  PA[01] PA1/TIM2_CH2/TIM5_CH2/USART2_RTS/UART4_RX         24       TIM5_CH2
 *  PA[02] PA2/TIM2_CH3/TIM5_CH3/TIM9_CH1/USART2_TX          25       TIM5_CH3_EXPOSURE
 *  PA[03] PA3/TIM2_CH4/TIM5_CH4/TIM9_CH2/USART2_RX          26       TIM5_CH4_STANDBY
 *  PA[04] PA4/SPI1_NSS/SPI3_NSS/DCMI_HSYNC/LCD_VSYNC        29       DCMI_HSYNC
 *  PA[05] PA5/TIM2_CH1/TIM2_ETR/TIM8_CH1N/SPI1_SCK          30       CAM_NRESET
 *  PA[06] PA6/TIM3_CH1/SPI1_MISO/TIM13_CH1/DCMI_PIXCLK      31       DCMI_PIXCK
 *  PA[07] PA7/TIM1,8_CH1N/TIM3_CH2/SPI1_MOSI/TIM14_CH1      32       SPI1_MOSI---
 *  PA[08] PA8/MCO1/TIM1_CH1/I2C3_SCL/USART1_CK/LCD_R6/      67       I2C3_SCL---
 *  PA[09] PA9/TIM1_CH2/USART1_TX/DCMI_D0/OTG_FS_VBUS        68       VBUS
 *  PA[10] PA10/TIM1_CH3/USART1_RX/OTG_FS_ID/DCMI_D1         69       OTG_FS_ID
 *  PA[11] PA11/TIM1_CH4/USART1_CTS/CAN1_RX/LCD_R4/OTG_FS_DM 70       OTG_FS_N
 *  PA[12] PA12/TIM1_ETR/USART1_RTS/CAN1_TX/LCD_R5/OTG_FS_DP 71       OTG_FS_P
 *  PA[13] PA13/JTMS-SWDIO                                   72       JTMS-SWDIO
 *  PA[14] PA14/JTCK-SWCLK                                   76       JTCK-SWCLK
 *  PA[15] PA15/JTDI/TIM2_CH1/TIM2_ETR/SPI1_NSS              77       JTDI
 *
 */


/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PB0
 */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 *  PC0
 */

/* CAN ************************************************************************ *
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ------------------------
 *
 *  PD[00] PD0/TIM4_CH3/I2C1_SCL/CAN1_RX                     81       CAN1_RX
 *  PD[01] PD1/TIM4_CH4/I2C1_SDA/CAN1_TX                     82       CAN1_TX
 *
 */

/* GPIO ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 */

#define GPIO_WAIT_GETNODEINFO   (GPIO_INPUT | GPIO_CNF_INPULLUP | GPIO_PORTC | GPIO_PIN13)


/* LEDs ***********************************************************************
 *
 *   GPIO      Function                                     MPU        Board
 *                                                          Pin #      Name
 * -- ----- --------------------------------             ----------------------
 *
 */

#define GPIO_LED2       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_SPEED_2MHz | \
                         GPIO_PORTE | GPIO_PIN2 | GPIO_OUTPUT_SET)
#define GPIO_LED_AMBER  GPIO_LED2

#define GPIO_LED3       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_SPEED_2MHz | \
                         GPIO_PORTE | GPIO_PIN3 | GPIO_OUTPUT_SET)
#define GPIO_LED_BLUE   GPIO_LED2

#define GPIO_LED4       (GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_SPEED_2MHz | \
                         GPIO_PORTE | GPIO_PIN7 | GPIO_OUTPUT_SET)
#define GPIO_LED_RED    GPIO_LED4


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

/************************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization for NSH.
 *
 *   CONFIG_NSH_ARCHINIT=y :
 *     Called from the NSH library
 *
 *   CONFIG_BOARD_INITIALIZE=y, CONFIG_NSH_LIBRARY=y, &&
 *   CONFIG_NSH_ARCHINIT=n :
 *     Called from board_initialize().
 *
 ************************************************************************************/

#ifdef CONFIG_NSH_LIBRARY
int nsh_archinitialize(void);
#endif

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
 *   interface in any event and the name board_led_initialization is
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
void board_led_initialize(void);
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
///
#endif /* __ASSEMBLY__ */

__END_DECLS
