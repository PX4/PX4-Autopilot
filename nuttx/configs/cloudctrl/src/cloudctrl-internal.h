/****************************************************************************************************
 * configs/cloudctrl/src/cloudctrl-internal.h
 * arch/arm/src/board/cloudctrl-internal.n
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Darcy Gong <darcy.gong@gmail.com>
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

#ifndef __CONFIGS_CLOUDCTRLL_SRC_CLOUDCTRL_INTERNAL_H
#define __CONFIGS_CLOUDCTRLL_SRC_CLOUDCTRL_INTERNAL_H

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

/* cloudctrl GPIO Configuration **********************************************************************/

/* STM3240G-EVAL GPIOs ******************************************************************************/
/* Ethernet
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 24 PA1  MII_RX_CLK     Ethernet PHY   NOTE:  Despite the MII labeling of these
 *         RMII_REF_CLK   Ethernet PHY   signals, the DM916AEP is actually configured
 * 25 PA2  MII_MDIO       Ethernet PHY   to work in RMII mode.
 * 48 PB11 MII_TX_EN      Ethernet PHY
 * 51 PB12 MII_TXD0       Ethernet PHY
 * 52 PB13 MII_TXD1       Ethernet PHY
 * 16 PC1  MII_MDC        Ethernet PHY
 * 34 PC5  MII_INT        Ethernet PHY
 * 55 PD8  MII_RX_DV      Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 * 55 PD8  RMII_CRSDV     Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 * 56 PD9  MII_RXD0       Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 * 57 PD10 MII_RXD1       Ethernet PHY.  Requires CONFIG_STM32_ETH_REMAP
 *
 * The board desdign can support a 50MHz external clock to drive the PHY 
 * (U9).  However, on my board, U9 is not present.
 *
 * 67 PA8  MCO            DM9161AEP
 */

#ifdef CONFIG_STM32_ETHMAC
#  define GPIO_MII_INT   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN5)
#endif

/* Use MCU Pin Reset DM9161 PHY Chip */

#ifdef CONFIG_PHY_DM9161
#  define GPIO_DM9161_RET (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|\
                           GPIO_PORTB|GPIO_PIN15)
#endif

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
 * 23 PA0  WAKEUP         Connected to KEY3.  Active low: Closing KEY4 pulls WAKEUP to ground.
 * 47 PB10 USERKEY        Connected to KEY1
 * 33 PC4  TAMPER         Connected to KEY2
 */

/* BUTTONS -- NOTE that all have EXTI interrupts configured */

#define MIN_IRQBUTTON     BUTTON_KEY1
#define MAX_IRQBUTTON     BUTTON_KEY3
#define NUM_IRQBUTTONS    (BUTTON_KEY3 - BUTTON_KEY1 + 1)

#define GPIO_BTN_WAKEUP   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTC|GPIO_PIN4)
#define GPIO_BTN_USERKEY  (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTB|GPIO_PIN10)
#define GPIO_BTN_TAMPER   (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_EXTI|GPIO_PORTA|GPIO_PIN0)

/* LEDs
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 1  PE2  LED1           Active low: Pulled high
 * 2  PE3  LED2           Active low: Pulled high
 * 3  PE4  LED3           Active low: Pulled high
 * 4  PE5  LED4           Active low: Pulled high
 */

#define GPIO_LED1       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN2)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN3)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN4)
#define GPIO_LED4       (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN5)

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


/* Relays */

#define NUM_RELAYS      2
#define GPIO_RELAYS_R00 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN0)
#define GPIO_RELAYS_R01 (GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|\
                         GPIO_OUTPUT_SET|GPIO_PORTE|GPIO_PIN1)

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
#endif /* __CONFIGS_CLOUDCTRLL_SRC_CLOUDCTRL_INTERNAL_H */
