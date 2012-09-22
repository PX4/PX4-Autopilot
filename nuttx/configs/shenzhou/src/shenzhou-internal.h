/****************************************************************************************************
 * configs/shenzhou/src/shenzhou-internal.h
 * arch/arm/src/board/shenzhou-internal.n
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SHENZHOUL_SRC_SHENZHOU_INTERNAL_H
#define __CONFIGS_SHENZHOUL_SRC_SHENZHOU_INTERNAL_H

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

/* Shenzhou GPIO Configuration **********************************************************************/

/* STM3240G-EVAL GPIOs ******************************************************************************/
/* Wireless
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 26 PA3  315M_VT
 * 17 PC2  WIRELESS_INT
 * 18 PC3  WIRELESS_CE    To the NRF24L01 2.4G wireless module
 * 59 PD12 WIRELESS_CS    To the NRF24L01 2.4G wireless module
 */

#define GPIO_WIRELESS_CS  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                           GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN12)

/* Buttons
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 23 PA0  WAKEUP         Connected to KEY4.  Active low: Closing KEY4 pulls WAKEUP to ground.
 * 47 PB10 USERKEY        Connected to KEY2
 * 33 PC4  USERKEY2       Connected to KEY1
 * 7  PC13 TAMPER         Connected to KEY3
 */

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON     BUTTON_KEY1
#define MAX_IRQBUTTON     BUTTON_KEY4
#define NUM_IRQBUTTONS    (BUTTON_KEY4 - BUTTON_KEY1 + 1)

#define GPIO_BTN_WAKEUP   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)
#define GPIO_BTN_USERKEY  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN10)
#define GPIO_BTN_USERKEY2 (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)
#define GPIO_BTN_TAMPER   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN13)

/* LEDs
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 83 PD2  LED1           Active low: Pulled high
 * 84 PD3  LED2           Active low: Pulled high
 * 85 PD4  LED3           Active low: Pulled high
 * 88 PD7  LED4           Active low: Pulled high
 */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN2)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN3)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN4)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN7)

/* TFT LCD
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 37 PB2  DATA_LE        To TFT LCD (CN13)
 * 96 PB9  F_CS           To both the TFT LCD (CN13) and to the W25X16 SPI FLASH
 * 34 PC5  TP_INT         JP6.  To TFT LCD (CN13) module
 * 65 PC8  LCD_CS         Active low: Pulled high
 * 66 PC9  TP_CS          Active low: Pulled high
 * 60 PD13 LCD_RS         To TFT LCD (CN13)
 * 61 PD14 LCD_WR         To TFT LCD (CN13)
 * 62 PD15 LCD_RD         To TFT LCD (CN13)
 * 97 PE0  DB00           To TFT LCD (CN13)
 * 98 PE1  DB01           To TFT LCD (CN13)
 * 1  PE2  DB02           To TFT LCD (CN13)
 * 2  PE3  DB03           To TFT LCD (CN13)
 * 3  PE4  DB04           To TFT LCD (CN13)
 * 4  PE5  DB05           To TFT LCD (CN13)
 * 5  PE6  DB06           To TFT LCD (CN13)
 * 38 PE7  DB07           To TFT LCD (CN13)
 * 39 PE8  DB08           To TFT LCD (CN13)
 * 40 PE9  DB09           To TFT LCD (CN13)
 * 41 PE10 DB10           To TFT LCD (CN13)
 * 42 PE11 DB11           To TFT LCD (CN13)
 * 43 PE12 DB12           To TFT LCD (CN13)
 * 44 PE13 DB13           To TFT LCD (CN13)
 * 45 PE14 DB14           To TFT LCD (CN13)
 * 46 PE15 DB15           To TFT LCD (CN13)
 */

#define GPIO_LCD_CS    (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

/* RS-485
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 88 PD7  485_DIR        SP3485 read enable (not)
 */

/* To be provided */

/* USB
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 95 PB8  USB_PWR        Drives USB VBUS
 */

#define GPIO_OTGFS_PWRON  (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_SPEED_100MHz|\
                           GPIO_PUSHPULL|GPIO_PORTB|GPIO_PIN8)

/* Audio DAC
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 */

/* To be provided */

/* SPI FLASH
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 96 PB9  F_CS           To both the TFT LCD (CN13) and to the W25X16 SPI FLASH
 */

#define GPIO_FLASH_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN9)

/* SD Card
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 53 PB14 SD_CD          Active low: Pulled high
 * 58 PD11 SD_CS
 */

#define GPIO_SD_CD   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN14)
#define GPIO_SD_CS   (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)

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
 *   Called to configure SPI chip select GPIO pins for the STM3240G-EVAL board.
 *
 ****************************************************************************************************/

void weak_function stm32_spiinitialize(void);

/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called from stm32_usbinitialize very early in inialization to setup USB-related GPIO pins for
 *   the STM3240G-EVAL board.
 *
 ****************************************************************************************************/

#ifdef CONFIG_STM32_OTGFS
void weak_function stm32_usbinitialize(void);
#endif

/****************************************************************************************************
 * Name: stm32_usbhost_initialize
 *
 * Description:
 *   Called at application startup time to initialize the USB host functionality. This function will
 *   start a thread that will monitor for device connection/disconnection events.
 *
 ****************************************************************************************************/

#if defined(CONFIG_STM32_OTGFS) && defined(CONFIG_USBHOST)
int stm32_usbhost_initialize(void);
#endif

/****************************************************************************
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SPI1=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor);

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_W25
int stm32_w25initialize(int minor);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SHENZHOUL_SRC_SHENZHOU_INTERNAL_H */
