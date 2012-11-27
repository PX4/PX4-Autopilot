/************************************************************************************
 * configs/shenzhou/include/board.h
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
 ************************************************************************************/

#ifndef __CONFIGS_SHENZHOU_INCLUDE_BOARD_H
#define __CONFIGS_SHENZHOU_INCLUDE_BOARD_H 1

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32_sdio.h"
#include "stm32_internal.h"

#include <nuttx/arch.h>

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 25MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL ouput is 72MHz */

#define STM32_PLL_PREDIV2       RCC_CFGR2_PREDIV2d5   /* 25MHz / 5 => 5MHz */
#define STM32_PLL_PLL2MUL       RCC_CFGR2_PLL2MULx8   /* 5MHz * 8  => 40MHz */
#define STM32_PLL_PREDIV1       RCC_CFGR2_PREDIV1d5   /* 40MHz / 5 => 8MHz */
#define STM32_PLL_PLLMUL        RCC_CFGR_PLLMUL_CLKx9 /* 8MHz * 9  => 72Mhz */
#define STM32_PLL_FREQUENCY     (72000000)

/* SYCLLK and HCLK are the PLL frequency */

#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY  /* same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 2-7, 12-14 */

/* APB2 timers 1 and 8 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-4 will be twice PCLK1 (I presume the remaining will receive PCLK1) */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* MCO output driven by PLL3. From above, we already have PLL3 input frequency as:
 *
 *  STM32_PLL_PREDIV2 = 5, 25MHz / 5 => 5MHz 
 */

#if defined(CONFIG_STM32_MII_MCO) || defined(CONFIG_STM32_RMII_MCO)
#  define BOARD_CFGR_MCO_SOURCE  RCC_CFGR_PLL3CLK      /* Source: PLL3 */
#  define STM32_PLL_PLL3MUL      RCC_CFGR2_PLL3MULx10  /* MCO 5MHz * 10 = 50MHz */
#endif

/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with stm32_setled() */

#define BOARD_LED1          0
#define BOARD_LED2          1
#define BOARD_LED3          2
#define BOARD_LED4          3
#define BOARD_NLEDS         4

/* LED bits for use with stm32_setleds() */

#define BOARD_LED1_BIT      (1 << BOARD_LED1)
#define BOARD_LED2_BIT      (1 << BOARD_LED2)
#define BOARD_LED3_BIT      (1 << BOARD_LED3)
#define BOARD_LED4_BIT      (1 << BOARD_LED4)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 4 LEDs on board the
 * STM3240G-EVAL.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED         0  /* LED1 */
#define LED_HEAPALLOCATE    1  /* LED2 */
#define LED_IRQSENABLED     2  /* LED1 + LED2 */
#define LED_STACKCREATED    3  /* LED3 */
#define LED_INIRQ           4  /* LED1 + LED3 */
#define LED_SIGNAL          5  /* LED2 + LED3 */
#define LED_ASSERTION       6  /* LED1 + LED2 + LED3 */
#define LED_PANIC           7  /* N/C  + N/C  + N/C + LED4 */

/* Button definitions ***************************************************************/
/* The STM3240G-EVAL supports three buttons: */

#define BUTTON_KEY1         0  /* Name printed on board */
#define BUTTON_KEY2         1
#define BUTTON_KEY3         2
#define BUTTON_KEY4         3
#define NUM_BUTTONS         4

#define BUTTON_USERKEY2     BUTTON_KEY1 /* Names in schematic */
#define BUTTON_USERKEY      BUTTON_KEY2
#define BUTTON_TAMPER       BUTTON_KEY3
#define BUTTON_WAKEUP       BUTTON_KEY4

#define BUTTON_KEY1_BIT     (1 << BUTTON_KEY1)
#define BUTTON_KEY2_BIT     (1 << BUTTON_KEY2)
#define BUTTON_KEY3_BIT     (1 << BUTTON_KEY3)
#define BUTTON_KEY4_BIT     (1 << BUTTON_KEY4)

#define BUTTON_USERKEY2_BIT BUTTON_KEY1_BIT
#define BUTTON_USERKEY_BIT  BUTTON_KEY2_BIT
#define BUTTON_TAMPER_BIT   BUTTON_KEY3_BIT
#define BUTTON_WAKEUP_BIT   BUTTON_KEY4_BIT

/* Relays */

#define NUM_RELAYS          2

/* Pin selections ******************************************************************/
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
#  ifndef CONFIG_STM32_ETH_REMAP
#    error "STM32 Ethernet requires CONFIG_STM32_ETH_REMAP"
#  endif
#  ifndef CONFIG_STM32_RMII
#    error "STM32 Ethernet requires CONFIG_STM32_RMII"
#  endif
#  ifndef CONFIG_STM32_RMII_MCO
#    error "STM32 Ethernet requires CONFIG_STM32_RMII_MCO"
#  endif
#endif

/* USB
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 68 PA9  USB_VBUS       MINI-USB-AB. JP3
 * 69 PA10 USB_ID         MINI-USB-AB. JP5
 * 70 PA11 USB_DM         MINI-USB-AB
 * 71 PA12 USB_DP         MINI-USB-AB
 * 95 PB8  USB_PWR        Drives USB VBUS
 */

/* UARTS/USARTS
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 68 PA9  USART1_TX      MAX3232 to CN5.  Requires CONFIG_STM32_USART1_REMAP
 * 69 PA10 USART1_RX      MAX3232 to CN5.  Requires CONFIG_STM32_USART1_REMAP
 * 86 PD5  USART2_TX      MAX3232 to CN6.  Requires CONFIG_STM32_USART2_REMAP
 * 87 PD6  USART2_RX      MAX3232 to CN6.  Requires CONFIG_STM32_USART2_REMAP
 * 86 PD5  485_TX         Same as USART2_TX but goes to SP3485
 * 87 PD6  485_RX         Save as USART2_RX but goes to SP3485 (see JP4)
 */

#if defined(CONFIG_STM32_USART1) && !defined(CONFIG_STM32_USART1_REMAP)
#  error "CONFIG_STM32_USART1 requires CONFIG_STM32_USART1_REMAP"
#endif

#if defined(CONFIG_STM32_USART2) && !defined(CONFIG_STM32_USART2_REMAP)
#  error "CONFIG_STM32_USART2 requires CONFIG_STM32_USART2_REMAP"
#endif

/* SPI
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 30 PA5  SPI1_SCK       To the SD card, SPI FLASH.
 *                        Requires !CONFIG_STM32_SPI1_REMAP
 * 31 PA6  SPI1_MISO      To the SD card, SPI FLASH.
 *                        Requires !CONFIG_STM32_SPI1_REMAP
 * 32 PA7  SPI1_MOSI      To the SD card, SPI FLASH.
 *                        Requires !CONFIG_STM32_SPI1_REMAP
 * 78 PC10 SPI3_SCK       To TFT LCD (CN13), the NRF24L01 2.4G wireless module.
 *                        Requires CONFIG_STM32_SPI3_REMAP.
 * 79 PC11 SPI3_MISO      To TFT LCD (CN13), the NRF24L01 2.4G wireless module.
 *                        Requires CONFIG_STM32_SPI3_REMAP.
 * 80 PC12 SPI3_MOSI      To TFT LCD (CN13), the NRF24L01 2.4G wireless module.
 *                        Requires CONFIG_STM32_SPI3_REMAP.
 */

#if defined(CONFIG_STM32_SPI1) && defined(CONFIG_STM32_SPI1_REMAP)
#  error "CONFIG_STM32_SPI1 must not have CONFIG_STM32_SPI1_REMAP"
#endif

#if defined(CONFIG_STM32_SPI3) && !defined(CONFIG_STM32_SPI3_REMAP)
#  error "CONFIG_STM32_SPI3 requires CONFIG_STM32_SPI3_REMAP"
#endif

/* DAC
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 29 PA4  DAC_OUT1       To CON5(CN14)
 * 30 PA5  DAC_OUT2       To CON5(CN14). JP10
 */

/* ADC
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 35 PB0  ADC_IN1        GPIO_ADC12_IN8. To CON5(CN14)
 * 36 PB1  ADC_IN2        GPIO_ADC12_IN9. To CON5(CN14)
 * 15 PC0  POTENTIO_METER GPIO_ADC12_IN10
 */

/* CAN
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 91 PB5  CAN2_RX        Requires CONFIG_STM32_CAN2_REMAP.
 * 92 PB6  CAN2_TX        Requires CONFIG_STM32_CAN2_REMAP.  See also JP11
 * 81 PD0  CAN1_RX        Requires CONFIG_STM32_CAN1_REMAP2.
 * 82 PD1  CAN1_TX        Requires CONFIG_STM32_CAN1_REMAP2.
 */

#if defined(CONFIG_STM32_CAN1) && !defined(CONFIG_STM32_CAN1_REMAP2)
#  error "CONFIG_STM32_CAN1 requires CONFIG_STM32_CAN1_REMAP2"
#endif

#if defined(CONFIG_STM32_CAN2) && !defined(CONFIG_STM32_CAN2_REMAP)
#  error "CONFIG_STM32_CAN2 requires CONFIG_STM32_CAN2_REMAP"
#endif

/* I2C
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 92 PB6  I2C1_SCL       Requires !CONFIG_STM32_I2C1_REMAP
 * 93 PB7  I2C1_SDA
 */

#if defined(CONFIG_STM32_I2C1) && defined(CONFIG_STM32_I2C1_REMAP)
#  error "CONFIG_STM32_I2C1 must not have CONFIG_STM32_I2C1_REMAP"
#endif

/* I2S
 *
 * -- ---- -------------- ----------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- ----------------------------------------------------------
 * 51 PB12 I2S_WS         GPIO_I2S2_WS. Audio DAC
 * 52 PB13 I2S_CK         GPIO_I2S2_CK. Audio DAC
 * 54 PB15 I2S_DIN        ??? Audio DAC data in.
 * 63 PC6  I2S_MCK        GPIO_I2S2_MCK. Audio DAC. Active low: Pulled high
 */

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void stm32_boardinitialize(void);

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 *   After up_buttoninit() has been called, up_buttons() may be called to
 *   collect the state of all buttons.  up_buttons() returns an 8-bit bit set
 *   with each bit associated with a button.  See the BUTTON_*_BIT and JOYSTICK_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See the
 *   BUTTON_* and JOYSTICK_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it may
 *   restored, if so desired).
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#ifdef CONFIG_ARCH_IRQBUTTONS
EXTERN xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif

/************************************************************************************
 * Name:  stm32_ledinit, stm32_setled, and stm32_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfacesare available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void stm32_ledinit(void);
EXTERN void stm32_setled(int led, bool ledon);
EXTERN void stm32_setleds(uint8_t ledset);
#endif

/************************************************************************************
 * Name:  stm32_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the Shenzhou board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 ************************************************************************************/

EXTERN void stm32_lcdclear(uint16_t color);

/************************************************************************************
 * Relay control functions
 *
 * Description:
 *   Non-standard functions for relay control from the Shenzhou board.
 *
 *   NOTE:  These must match the prototypes in include/nuttx/arch.h
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_RELAYS
EXTERN void up_relaysinit(void);
EXTERN void relays_setstat(int relays, bool stat);
EXTERN bool relays_getstat(int relays);
EXTERN void relays_setstats(uint32_t relays_stat);
EXTERN uint32_t relays_getstats(void);
EXTERN void relays_onoff(int relays, uint32_t mdelay);
EXTERN void relays_onoffs(uint32_t relays_stat, uint32_t mdelay);
EXTERN void relays_resetmode(int relays);
EXTERN void relays_powermode(int relays);
EXTERN void relays_resetmodes(uint32_t relays_stat);
EXTERN void relays_powermodes(uint32_t relays_stat);
#endif

/************************************************************************************
 * Chip ID functions
 *
 * Description:
 *   Non-standard functions to obtain chip ID information.
 *
 ************************************************************************************/

EXTERN const char *stm32_getchipid(void);
EXTERN const char *stm32_getchipid_string(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SHENZHOU_INCLUDE_BOARD_H */
