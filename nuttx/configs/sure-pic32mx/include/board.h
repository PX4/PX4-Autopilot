/****************************************************************************
 * configs/sure-pic32mx/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SURE_PIC32MX_INCLUDE_BOARD_H
#define __CONFIGS_SURE_PIC32MX_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Clocking *****************************************************************/
/* Crystal frequencies */

#define BOARD_POSC_FREQ        20000000  /* Primary OSC XTAL frequency (20MHz) */
#define BOARD_SOSC_FREQ        32768     /* Secondary OSC XTAL frequency (32.768KHz) */

/* Oscillator modes */

#define BOARD_FNOSC_POSCPLL    1        /* Use primary oscillator w/PLL */
#define BOARD_POSC_HSMODE      1        /* High-speed crystal (HS) mode */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_INPUT        BOARD_POSC_FREQ
#define BOARD_PLL_IDIV         5         /* PLL input divider */
#define BOARD_PLL_MULT         15        /* PLL multiplier */
#define BOARD_PLL_ODIV         1         /* PLL output divider */

#define BOARD_CPU_CLOCK        60000000 /* CPU clock (60MHz = (20MHz / 5) * 15 / 1) */

/* USB PLL configuration.
 * USB_CLOCK = ((POSC_XTAL / IDIV) * 24) / 2
 */

#define BOARD_UPLL_IDIV        5         /* USB PLL divider */
#define BOARD_USB_CLOCK        48000000  /* USB clock ((20MHz / 5) * 24) / 2 */

/* Peripheral clock is *not* divided down from CPU clock.
 * PBCLOCK = CPU_CLOCK / PBDIV
 */

#define BOARD_PBDIV            1        /* Peripheral clock divisor (PBDIV) */
#define BOARD_PBCLOCK          60000000 /* Peripheral clock (PBCLK = 60MHz/1) */

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_ENABLE        0        /* Watchdog is disabled */
#define BOARD_WD_PRESCALER     8        /* Watchdog pre-scaler */

/* Timer 1 is a type A timer and is used as system clock.  It can be clocked
 * with either the SOSC or the PBCLOCK.  We will use the PBCLOCK because it
 * is much more accurate.
 */

#undef BOARD_TIMER1_SOSC

/* LED definitions **********************************************************/
/* The Sure DB_DP11215 PIC32 Storage Demo Board board has five LEDs.  One
 * (D4, lablel "Power") is not controllable by software.  Four are
 * controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 */
                                  /* ON                  OFF                 */
                                  /* USB SD  FLASH ERROR USB SD  FLASH ERROR */
#define LED_STARTED            0  /* OFF OFF OFF   OFF   --- --- ---   ---   */
#define LED_HEAPALLOCATE       1  /* ON  OFF N/C   N/C   --- --- ---   ---   */
#define LED_IRQSENABLED        2  /* OFF ON  N/C   N/C   --- --- ---   ---   */
#define LED_STACKCREATED       3  /* ON  ON  N/C   N/C   --- --- ---   ---   */
#define LED_INIRQ              4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_SIGNAL             4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_ASSERTION          4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_PANIC              5  /* N/C N/C N/C   ON    N/C N/C N/C   OFF   */
#define LED_NVALUES            6

/* The Sure DB-DP11212 PIC32 General Purpose Demo Board does not have any user
 * controllable LEDs, but does does have a segment LED display.  That display is
 * however, obscured by the larger segment display attached to the board and, so,
 * is not supported.
 */

/* For distinguishing individual LEDs */

#define LED_USB                0
#define LED_SD                 1
#define LED_FLASH              2
#define LED_ERROR              3

/* Button Definitions *******************************************************/
/* The Sure PIC32MX board has three buttons.
 *
 * SW1  (SW_UP, left arrow)          RB3 Pulled high, Grounded/low when depressed
 * SW2  (SW_DOWN, down/right arrow)  RB2 Pulled high, Grounded/low when depressed
 * SW3  (SW_OK, right arrow)         RB4 Pulled high, Grounded/low when depressed
 */

#define BUTTON_SW1             0
#define BUTTON_SW2             1
#define BUTTON_SW3             2
#define NUM_BUTTONS            3

#define BUTTON_SW1_BIT         (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT         (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT         (1 << BUTTON_SW3)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
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
 *   with each bit associated with a button.  See the BUTTON_*_BIT
 *   definitions in board.h for the meaning of each bit.
 *
 *   up_irqbutton() may be called to register an interrupt handler that will
 *   be called when a button is depressed or released.  The ID value is a
 *   button enumeration value that uniquely identifies a button resource.
 *   See the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.  The previous interrupt handler address is returned (so that it
 *   may restored, if so desired).
 *
 *   When an interrupt occurs, it is due to a change on the GPIO input pin
 *   associated with the button.  In that case, all attached change
 *   notification handlers will be called.  Each handler must maintain state
 *   and determine if the unlying GPIO button input value changed.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#ifdef CONFIG_ARCH_IRQBUTTONS
EXTERN xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SURE_PIC32MX_INCLUDE_BOARD_H */
