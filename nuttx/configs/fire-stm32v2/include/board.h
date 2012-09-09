/************************************************************************************
 * configs/fire-stm32v2/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_FIRE_STM32V2_INCLUDE_BOARD_H
#define __CONFIGS_FIRE_STM32V2_INCLUDE_BOARD_H

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

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* On-board crystal frequency is 8MHz (HSE) */

#define STM32_BOARD_XTAL        8000000ul

/* PLL source is HSE/1, PLL multipler is 9: PLL frequency is 8MHz (XTAL) x 9 = 72MHz */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY    /* same as above, to satisfy compiler */

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

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx. 
 * Note: TIM1,8 are on APB2, others on APB1 */

#define STM32_TIM18_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_TIM27_FREQUENCY   STM32_HCLK_FREQUENCY

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled 
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 */
  
#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT) 
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT) 
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions ******************************************************************/
/* The M3 Wildfire has 3 LEDs labeled LED1, LED2 and LED3.  These LEDs are not
 * used by the NuttX port unless CONFIG_ARCH_LEDS is defined.  In that case, the
 * usage by the board port is defined in include/board.h and src/up_autoleds.c.
 * The LEDs are used to encode OS-related events as follows:
 *.
                                      /* LED1   LED2   LED3 */
#define LED_STARTED                0  /* OFF    OFF    OFF */
#define LED_HEAPALLOCATE           1  /* ON     OFF    OFF */
#define LED_IRQSENABLED            2  /* OFF    ON     OFF */
#define LED_STACKCREATED           3  /* OFF    OFF    OFF */

#define LED_INIRQ                  4  /*  NC     NC    ON  (momentary) */
#define LED_SIGNAL                 5  /*  NC     NC    ON  (momentary) */
#define LED_ASSERTION              6  /*  NC     NC    ON  (momentary) */
#define LED_PANIC                  7  /*  NC     NC    ON  (2Hz flashing) */
#undef  LED_IDLE                      /* Sleep mode indication not supported */

/* The M3 Wildfire supports several two user buttons:  KEY1 and KEY2 */

#define BUTTON_KEY1                0
#define BUTTON_KEY2                1
#define NUM_BUTTONS                2

#define BUTTON_KEY1_BIT            (1 << BUTTON_KEY1)
#define BUTTON_KEY2_BIT            (1 << BUTTON_KEY2)

/* Pin Remapping ********************************************************************/
/* USB 2.0
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 70  PA11   PA11-USBDM     USB2.0
 * 71  PA12   PA12-USBDP     USB2.0
 * 2   PE3    PE3-USB-M      USB2.0
 */

/* 2.4" TFT + Touchscreen
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 30  PA5    PA5-SPI1-SCK   2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 31  PA6    PA6-SPI1-MISO  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 32  PA7    PA7-SPI1-MOSI  2.4" TFT + Touchscreen, 10Mbit ENC28J60, SPI 2M FLASH
 * 92  PB6    PB6-I2C1-SCL   2.4" TFT + Touchscreen, AT24C02
 * 93  PB7    PB7-I2C1-SDA   2.4" TFT + Touchscreen, AT24C02
 * 7   PC13   PD13_LCD_LIGHT 2.4" TFT + Touchscreen
 * 81  PD0    PD0-FSMC_D2    2.4" TFT + Touchscreen
 * 82  PD1    PD1-FSMC_D3    2.4" TFT + Touchscreen
 * 85  PD4    PD4-FSMC_NOE   2.4" TFT + Touchscreen
 * 86  PD5    PD5-FSMC_NWE   2.4" TFT + Touchscreen
 * 88  PD7    PD7-FSMC_NE1   2.4" TFT + Touchscreen
 * 55  PD8    PD8-FSMC_D13   2.4" TFT + Touchscreen
 * 56  PD9    PD9-FSMC_D14   2.4" TFT + Touchscreen
 * 57  PD10   PD10-FSMC_D15  2.4" TFT + Touchscreen
 * 58  PD11   PD11-FSMC_A16  2.4" TFT + Touchscreen
 * 60  PD13   PD13-LCD/LIGHT 2.4" TFT + Touchscreen
 * 61  PD14   PD14-FSMC_D0   2.4" TFT + Touchscreen
 * 62  PD15   PD15-FSMC_D1   2.4" TFT + Touchscreen
 * 98  PE1    PE1-FSMC_NBL1  2.4" TFT + Touchscreen
 * 38  PE7    PE7-FSMC_D4    2.4" TFT + Touchscreen
 * 39  PE8    PE8-FSMC_D5    2.4" TFT + Touchscreen
 * 40  PE9    PE9-FSMC_D6    2.4" TFT + Touchscreen
 * 41  PE10   PE10-FSMC_D7   2.4" TFT + Touchscreen
 * 42  PE11   PE11-FSMC_D8   2.4" TFT + Touchscreen
 * 43  PE12   PE12-FSMC_D9   2.4" TFT + Touchscreen
 * 44  PE13   PE13-FSMC_D10  2.4" TFT + Touchscreen
 * 45  PE14   PE14-FSMC_D11  2.4" TFT + Touchscreen
 * 46  PE15   PE15-FSMC_D12  2.4" TFT + Touchscreen
 */

#if defined(CONFIG_STM32_SPI1) && defined(CONFIG_STM32_SPI1_REMAP)
#  error "SPI1 requires CONFIG_STM32_SPI1_REMAP=n"
#endif

#if defined(CONFIG_STM32_I2C1) && defined(CONFIG_STM32_I2C1_REMAP)
#  error "SPI1 requires CONFIG_STM32_I2C1_REMAP=n"
#endif

/* AT24C02
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 92  PB6    PB6-I2C1-SCL   2.4" TFT + Touchscreen, AT24C02
 * 93  PB7    PB7-I2C1-SDA   2.4" TFT + Touchscreen, AT24C02
 */

#if defined(CONFIG_STM32_I2C1) && defined(CONFIG_STM32_I2C1_REMAP)
#  error "SPI1 requires CONFIG_STM32_I2C1_REMAP=n"
#endif

/* Potentiometer/ADC
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 16  PC1    PC1/ADC123-IN11 Potentiometer (R16)
 * 24  PA1    PC1/ADC123-IN1
 */

/* USARTs
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 68  PA9    PA9-US1-TX     MAX3232, DB9 D8, Requires CONFIG_STM32_USART1_REMAP
 * 69  PA10   PA10-US1-RX    MAX3232, DB9 D8, Requires CONFIG_STM32_USART1_REMAP
 * 25  PA2    PA2-US2-TX     MAX3232, DB9 D7, Requires !CONFIG_STM32_USART2_REMAP
 * 26  PA3    PA3-US2-RX     MAX3232, DB9 D7, Requires !CONFIG_STM32_USART2_REMAP
 */

#if defined(CONFIG_STM32_USART1) && !defined(CONFIG_STM32_USART1_REMAP)
#  errror "USART1 requires CONFIG_STM32_USART1_REMAP=y"
#endif

#if defined(CONFIG_STM32_USART2) && defined()
#  errror "USART2 requires CONFIG_STM32_USART2_REMAP=n"
#endif

/* ENC28J60
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 29  PA4    PA4-SPI1-NSS   10Mbit ENC28J60, SPI 2M FLASH
 */

#if defined(CONFIG_STM32_SPI1) && defined(CONFIG_STM32_SPI1_REMAP)
#  error "SPI1 requires CONFIG_STM32_SPI1_REMAP=n"
#endif

/* MP3
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 48  PB11   PB11-MP3-RST   MP3
 * 51  PB12   PB12-SPI2-NSS  MP3
 * 52  PB13   PB13-SPI2-SCK  MP3
 * 53  PB14   PB14-SPI2-MISO MP3
 * 54  PB15   PB15-SPI2-MOSI MP3
 * 63  PC6    PC6-MP3-XDCS   MP3
 * 64  PC7    PC7-MP3-DREQ   MP3
 */

/* SD Card
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 65  PC8    PC8-SDIO-D0    SD card, pulled high
 * 66  PC9    PC9-SDIO-D1    SD card, pulled high
 * 78  PC10   PC10-SDIO-D2   SD card, pulled high
 * 79  PC11   PC10-SDIO-D3   SD card, pulled high
 * 80  PC12   PC12-SDIO-CLK  SD card
 * 83  PD2    PD2-SDIO-CMD   SD card, pulled high
 */

/* CAN
 *
 * --- ------ -------------- -------------------------------------------------------------------
 * PIN NAME   SIGNAL         NOTES
 * --- ------ -------------- -------------------------------------------------------------------
 *
 * 95  PB8    PB8-CAN-RX     CAN tranceiver, Header 2H
 * 96  PB9    PB9-CAN-TX     CAN tranceiver, Header 2H
 */

#if defined(CONFIG_STM32_CAN1) && !defined(CONFIG_STM32_CAN1_REMAP1)
#  error "SPI1 requires CONFIG_STM32_CAN1_REMAP1=y"
#endif

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

EXTERN void stm32_boardinitialize(void);

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
 * Name:  fire_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the M3 Wildfire board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   dispaly is cleared by simply setting all GRAM memory to the specified color.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_FSMC
EXTERN void fire_lcdclear(uint16_t color);
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_FIRE_STM32V2_INCLUDE_BOARD_H */
