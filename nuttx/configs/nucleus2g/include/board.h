/************************************************************************************
 * configs/nucleus2g/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* NOTE:  The following definitions require lpc17_syscon.h.  It is not included here
 * because the including C file may not have that file in its include path.
 */

#define BOARD_XTAL_FREQUENCY        (12000000)            /* XTAL oscillator frequency */
#define BOARD_OSCCLK_FREQUENCY      BOARD_XTAL_FREQUENCY  /* Main oscillator frequency */
#define BOARD_RTCCLK_FREQUENCY      (32000)               /* RTC oscillator frequency */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)             /* Internal RC oscillator frequency */

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> PLL0 multipler=20, pre-divider=1
 *   CCLCK = 480MHz / 6 = 80MHz               -> CCLK divider = 6
 */

#define LPC17_CCLK                 80000000 /* 80Mhz */

/* Select the main oscillator as the frequency source.  SYSCLK is then the frequency
 * of the main oscillator.
 */

#undef CONFIG_LPC17_MAINOSC
#define CONFIG_LPC17_MAINOSC       1
#define BOARD_SCS_VALUE            SYSCON_SCS_OSCEN

/* Select the main oscillator and CCLK divider. The output of the divider is CCLK.
 * The input to the divider (PLLCLK) will be determined by the PLL output.
 */

#define BOARD_CCLKCFG_DIVIDER      6
#define BOARD_CCLKCFG_VALUE        ((BOARD_CCLKCFG_DIVIDER-1) << SYSCON_CCLKCFG_SHIFT)

/* PLL0.  PLL0 is used to generate the CPU clock divider input (PLLCLK).
 *
 *  Source clock:               Main oscillator
 *  PLL0 Multiplier value (M):  20 
 *  PLL0 Pre-divider value (N): 1
 *
 *  PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz
 */

#undef CONFIG_LPC17_PLL0
#define CONFIG_LPC17_PLL0          1
#define BOARD_CLKSRCSEL_VALUE      SYSCON_CLKSRCSEL_MAIN

#define BOARD_PLL0CFG_MSEL         20
#define BOARD_PLL0CFG_NSEL         1
#define BOARD_PLL0CFG_VALUE \
  (((BOARD_PLL0CFG_MSEL-1) << SYSCON_PLL0CFG_MSEL_SHIFT) | \
   ((BOARD_PLL0CFG_NSEL-1) << SYSCON_PLL0CFG_NSEL_SHIFT))

/* PLL1 -- Not used. */

#undef CONFIG_LPC17_PLL1
#define BOARD_PLL1CFG_MSEL         36
#define BOARD_PLL1CFG_NSEL         1
#define BOARD_PLL1CFG_VALUE \
  (((BOARD_PLL1CFG_MSEL-1) << SYSCON_PLL1CFG_MSEL_SHIFT) | \
   ((BOARD_PLL1CFG_NSEL-1) << SYSCON_PLL1CFG_NSEL_SHIFT))

/* USB divider.  This divider is used when PLL1 is not enabled to get the
 * USB clock from PLL0:
 *
 *  USBCLK = PLL0CLK / 10 = 48MHz
 */

#define BOARD_USBCLKCFG_VALUE      SYSCON_USBCLKCFG_DIV10

/* FLASH Configuration */

#undef  CONFIG_LP17_FLASH
#define CONFIG_LP17_FLASH          1
#define BOARD_FLASHCFG_VALUE       0x0000303a

/* LED definitions ******************************************************************/
/* The Nucleus2G has 3 LEDs... two on the Babel CAN board and a "heartbeat" LED."
 * The LEDs on the Babel CAN board are capabl of OFF/GREEN/RED/AMBER status.
 * In normal usage, the two LEDs on the Babel CAN board would show CAN status, but if
 * CONFIG_ARCH_LEDS is defined, these LEDs will be controlled as follows for NuttX
 * debug functionality (where NC means "No Change").
 *
 * During the boot phases.  LED1 and LED2 will show boot status.
 */
                                      /* LED1   LED2   HEARTBEAT */
#define LED_STARTED                0  /* OFF    OFF    OFF */
#define LED_HEAPALLOCATE           1  /* GREEN  OFF    OFF */
#define LED_IRQSENABLED            2  /* OFF    GREEN  OFF */
#define LED_STACKCREATED           3  /* OFF    OFF    OFF */

/* After the system is booted, this logic will no longer use LEDs 1 and 2.  They
 * are available for use the application software using lpc17_led1() and lpc17_led2()
 * (prototyped below)
 */
                                      /* LED1   LED2   HEARTBEAT */
#define LED_INIRQ                  4  /*  NC     NC    ON  (momentary) */
#define LED_SIGNAL                 5  /*  NC     NC    ON  (momentary) */
#define LED_ASSERTION              6  /*  NC     NC    ON  (momentary) */
#define LED_PANIC                  7  /*  NC     NC    ON  (1Hz flashing) */

/* Alternate pin selections *********************************************************/
/* UART1 -- Not connected */

#define GPIO_UART1_TXD             GPIO_UART1_TXD_1
#define GPIO_UART1_RXD             GPIO_UART1_RXD_1
#define GPIO_UART1_CTS             GPIO_UART1_CTS_1
#define GPIO_UART1_DCD             GPIO_UART1_DCD_1
#define GPIO_UART1_DSR             GPIO_UART1_DSR_1
#define GPIO_UART1_DTR             GPIO_UART1_DTR_1
#define GPIO_UART1_RI              GPIO_UART1_RI_1
#define GPIO_UART1_RTS             GPIO_UART1_RTS_1

/* UART2 -- Not connected */

#define GPIO_UART2_TXD             GPIO_UART2_TXD_1
#define GPIO_UART2_RXD             GPIO_UART2_RXD_1

/* UART3 -- Not connected */

#define GPIO_UART3_TXD             GPIO_UART3_TXD_1
#define GPIO_UART3_RXD             GPIO_UART3_RXD_1

/* Either SPI or SSP0 can drive the MMC/SD slot (SSP0 alternate pin settings are
 * not connected)
 */

#define GPIO_SSP0_SCK              GPIO_SSP0_SCK_1
#define GPIO_SSP0_SSEL             GPIO_SSP0_SSEL_1
#define GPIO_SSP0_MISO             GPIO_SSP0_MISO_1
#define GPIO_SSP0_MOSI             GPIO_SSP0_MOSI_1

/* SSP1 */

#define GPIO_SSP1_SCK              GPIO_SSP1_SCK_1

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__
#ifdef CONFIG_ARCH_LEDS
enum lpc17_ledstate_e
{
  LPC17_LEDSTATE_OFF   = 0,
  LPC17_LEDSTATE_GREEN = 1,
  LPC17_LEDSTATE_RED   = 2,
  LPC17_LEDSTATE_AMBER = (LPC17_LEDSTATE_GREEN|LPC17_LEDSTATE_RED),
};
#endif

enum output_state
{
  RELAY_OPEN   = 0,
  RELAY_CLOSED = 1,
  RELAY_TOGGLE = 2,
};

/************************************************************************************
 * Public Data
 ************************************************************************************/

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
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void lpc17_boardinitialize(void);

/************************************************************************************
 * Name: lpc17_led1 and 2
 *
 * Description:
 *   Once the system has booted, these functions can be used to control LEDs 1 and 2
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
EXTERN void lpc17_led1(enum lpc17_ledstate_e state);
EXTERN void lpc17_led2(enum lpc17_ledstate_e state);
#endif

/************************************************************************************
 * Name: nucleus_bms_relay 1-4
 *
 * Description:
 * once booted these functions control the 4 isolated FET outputs from the
 * master BMS controller
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BOARD_NUCLEUS2G_BMS
EXTERN void nucleus_bms_relay1(enum output_state state);
EXTERN void nucleus_bms_relay2(enum output_state state);
EXTERN void nucleus_bms_relay3(enum output_state state);
EXTERN void nucleus_bms_relay4(enum output_state state);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
