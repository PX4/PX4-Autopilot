/****************************************************************************
 * configs/lpc4330-xplorer/include/board.h
 * include/arch/board/board.h
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
 ****************************************************************************/

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#if defined(CONFIG_ARCH_IRQBUTTONS) && defined(CONFIG_GPIO_IRQ)
#  include <nuttx/irq.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Clocking ****************************************************************/
/* NOTE:  The following definitions require lpc43_cgu.h.  It is not included
 * here because the including C file may not have that file in its include
 * path.
 *
 * The Xplorer board has four crystals on board:
 *
 *     Y1 - RTC 32.768 MHz oscillator input,
 *     Y2 - 24.576 MHz input to the UDA 1380 audio codec,
 *     Y3 - 12.000 MHz LPC43xx crystal oscillator input
 *     Y4 - 50 MHz input for Ethernet
 */

#define BOARD_XTAL_FREQUENCY        (12000000)  /* XTAL oscillator frequency (Y3) */
#define BOARD_RTCCLK_FREQUENCY      (32768)     /* RTC oscillator frequency (Y1) */
#define BOARD_INTRCOSC_FREQUENCY    (4000000)   /* Internal RC oscillator frequency */

/* Integer and direct modes are supported:
 *
 * In integer mode (Fclkout < 156000000):
 *    Fclkin  = BOARD_XTAL_FREQUENCY
 *    Fclkout = Msel * FClkin / Nsel
 *    Fcco    = 2 * Psel * Fclkout
 * In direct mode (Fclkout > 156000000):
 *    Fclkin  = BOARD_XTAL_FREQUENCY
 *    Fclkout = Msel * FClkin / Nsel
 *    Fcco    = Fclkout
 */

#ifdef CONFIG_LPC43_72MHz

/* NOTE:  At 72MHz, the calibrated value of CONFIG_BOARD_LOOPSPERMSEC was
 * determined to be:
 *
 *    CONFIG_BOARD_LOOPSPERMSEC=7191
 *
 * executing from SRAM.
 */

/* Final clocking (Integer mode with no ramp-up)
 *
 *    Fclkout = 6 * 12MHz / 1  = 72MHz
 *    Fcco    = 2 * 2 * 72MHz = 216MHz
 */

#  define BOARD_PLL_MSEL            (6)         /* Msel = 6 */
#  define BOARD_PLL_NSEL            (1)         /* Nsel = 1 */
#  define BOARD_PLL_PSEL            (2)         /* Psel = 2 */

#  define BOARD_FCLKOUT_FREQUENCY   (72000000)  /* 6 * 12,000,000 / 1 */
#  define BOARD_FCCO_FREQUENCY      (244000000) /* 2 * 2 * Fclkout */
 
#else

/* NOTE:  At 72MHz, the calibrated value of CONFIG_BOARD_LOOPSPERMSEC was
 * determined to be:
 *
 *    CONFIG_BOARD_LOOPSPERMSEC=18535
 *
 * executing from SRAM.
 */

/* Intermediate ramp-up clocking (Integer mode). If BOARD_PLL_RAMP_MSEL
 * is not defined, there will be no ramp-up.  
 *
 *    Fclkout = 9 * 12MHz / 1  = 108MHz
 *    Fcco    = 2 * 1 * 108MHz = 216MHz
 */

#  define BOARD_PLL_RAMP_MSEL       (9)         /* Msel = 9 */
#  define BOARD_PLL_RAMP_NSEL       (1)         /* Nsel = 1 */
#  define BOARD_PLL_RAMP_PSEL       (1)         /* Psel = 1 */

#  define BOARD_RAMP_FCLKOUT        (108000000) /* 9 * 12,000,000 / 1 */
#  define BOARD_RAMP_FCCO           (216000000) /* 2 * 1 * Fclkout */

/* Final clocking (Direct mode). 
 *
 *    Fclkout = 17 * 12MHz / 1 = 204MHz
 *    Fcco    = Fclockout      = 204MHz
 */

#  define BOARD_PLL_MSEL            (17)        /* Msel = 17 */
#  define BOARD_PLL_NSEL            (1)         /* Nsel = 1 */

#  define BOARD_FCLKOUT_FREQUENCY   (204000000) /* 17 * 12,000,000 / 1 */
#  define BOARD_FCCO_FREQUENCY      (204000000) /* Fclockout */

#endif

/* This is the clock setup we configure for:
 *
 *   SYSCLK = BOARD_OSCCLK_FREQUENCY = 12MHz  -> Select Main oscillator for source
 *   PLL0CLK = (2 * 20 * SYSCLK) / 1 = 480MHz -> PLL0 multipler=20, pre-divider=1
 *   CCLCK = 480MHz / 6 = 80MHz               -> CCLK divider = 6
 */

#define LPC43_CCLK                  BOARD_FCLKOUT_FREQUENCY

/* SPIFI clocking **********************************************************/
/* The SPIFI will receive clocking from a divider per the settings provided
 * in this file.  The NuttX code will configure PLL1 as the input clock
 * for the selected divider
 */

#undef  BOARD_SPIFI_PLL1                        /* No division */
#undef  BOARD_SPIFI_DIVA                        /* Supports division by 1-4 */
#undef  BOARD_SPIFI_DIVB                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVC                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVD                        /* Supports division by 1-16 */
#undef  BOARD_SPIFI_DIVE                        /* Supports division by 1-256 */

#if BOARD_FCLKOUT_FREQUENCY < 20000000
#  define BOARD_SPIFI_PLL1          1           /* Use PLL1 directly */
#else
#  define BOARD_SPIFI_DIVB          1           /* Use IDIVB */
#endif


/* We need to configure the divider so that its output is as close to the
 * desired SCLK value.  The peak data transfer rate will be about half of
 * this frequency in bytes per second.
 */

#if BOARD_FCLKOUT_FREQUENCY < 20000000
#  define BOARD_SPIFI_FREQUENCY     BOARD_FCLKOUT_FREQUENCY  /* 72Mhz? */
#else
#  define BOARD_SPIFI_DIVIDER       (14)        /* 204MHz / 14 = 14.57MHz */
#  define BOARD_SPIFI_FREQUENCY     (102000000) /* 204MHz / 14 = 14.57MHz */
#endif

/* UART clocking ***********************************************************/
/* Configure all U[S]ARTs to use the XTAL input frequency */

#define BOARD_USART0_CLKSRC         BASE_USART0_CLKSEL_XTAL
#define BOARD_USART0_BASEFREQ       BOARD_XTAL_FREQUENCY

#define BOARD_UART1_CLKSRC          BASE_UART1_CLKSEL_XTAL
#define BOARD_UART1_BASEFREQ        BOARD_XTAL_FREQUENCY

#define BOARD_USART2_CLKSRC         BASE_USART2_CLKSEL_XTAL
#define BOARD_USART2_BASEFREQ       BOARD_XTAL_FREQUENCY

#define BOARD_USART3_CLKSRC         BASE_USART3_CLKSEL_XTAL
#define BOARD_USART3_BASEFREQ       BOARD_XTAL_FREQUENCY

/* LED definitions *********************************************************/
/* The LPC4330-Xplorer has 2 user-controllable LEDs labeled D2 an D3 in the
 * schematic and on but referred to has LED1 and LED2 here, respectively.
 *
 *  LED1   D2  GPIO1[12]
 *  LED2   D3  GPIO1[11]
 *
 * LEDs are pulled high to a low output illuminates the LED.
 * 
 * LED index values for use with lpc43_setled()
 */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

/* LED bits for use with lpc43_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, the LEDs will be controlled as follows
 * for NuttX debug functionality (where NC means "No Change"). If
 * CONFIG_ARCH_LEDS is not defined, then the LEDs are completely under
 * control of the application.  The following interfaces are then available
 * for application control of the LEDs:
 *
 *  void lpc43_ledinit(void);
 *  void lpc43_setled(int led, bool ledon);
 *  void lpc43_setleds(uint8_t ledset);
 */
                                      /*     ON            OFF     */
                                      /* LED1   LED2   LED1   LED2 */
#define LED_STARTED                0  /* OFF    OFF     -      -   */
#define LED_HEAPALLOCATE           1  /* ON     OFF     -      -   */
#define LED_IRQSENABLED            1  /* ON     OFF     -      -   */
#define LED_STACKCREATED           1  /* ON     OFF     -      -   */
#define LED_INIRQ                  2  /* NC     ON      NC     OFF */
#define LED_SIGNAL                 2  /* NC     ON      NC     OFF */
#define LED_ASSERTION              2  /* NC     ON      NC     OFF */
#define LED_PANIC                  2  /* NC     ON      NC     OFF */

/* UART Pins ****************************************************************/
/* The LPC4330 Xplorer does not have RS-232 drivers or serial connectors on
 * board. USART0 and UART1 are available on J8 as follows:
 *
 *   ------ ------ -----------------------
 *   SIGNAL J8 PIN   LPC4330FET100 PIN
 *                   (TFBGA100 package)
 *   ------ ------ -----------------------
 *   U0_TXD pin  9  F6  P6_4  U0_TXD=Alt 2
 *   U0_RXD pin 10  F9  P6_5  U0_RXD=Alt 2
 *   U1_TXD pin 13  H8  P1_13 U1_TXD=Alt 1
 *   U1_RXD pin 14  J8  P1_14 U1_RXD=Alt 1
 *   ------ ------ -----------------------
 *
 * The following definitions must be provided so that the LPC43 serial
 * driver can set up the U[S]ART for the serial console properly (see the
 * file arch/arc/src/lpc43xx/lpc4310203050_pinconf.h for more info).
 */

#define PINCONF_U0_TXD  PINCONF_U0_TXD_3
#define PINCONF_U0_RXD  PINCONF_U0_RXD_3
#define PINCONF_U0_DIR  PINCONF_U0_DIR_3

#define PINCONF_U1_TXD  PINCONF_U1_TXD_1
#define PINCONF_U1_RXD  PINCONF_U1_RXD_1

#define PINCONF_U2_TXD  PINCONF_U2_TXD_1
#define PINCONF_U2_RXD  PINCONF_U2_RXD_1
#define PINCONF_U2_DIR  PINCONF_U2_DIR_1

#define PINCONF_U3_TXD  PINCONF_U3_TXD_2
#define PINCONF_U3_RXD  PINCONF_U3_RXD_2
#define PINCONF_U3_DIR  PINCONF_U3_DIR_2

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_boardinitialize
 *
 * Description:
 *   All LPC43xx architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ****************************************************************************/

EXTERN void lpc43_boardinitialize(void);

/*****************************************************************************
 * Name:  lpc43_ledinit, lpc43_setled, and lpc43_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board
 *   LEDs.  If CONFIG_ARCH_LEDS is not defined, then the following interfaces
 *   are available to control the LEDs from user applications.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_LEDS
EXTERN void lpc43_ledinit(void);
EXTERN void lpc43_setled(int led, bool ledon);
EXTERN void lpc43_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
