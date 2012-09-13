/************************************************************************************
 * configs/avr32dev1/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_BOARD_BOARD_H
#define __ARCH_BOARD_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Oscillator setup:  RCOSC, OSC32, OSC0, OSC1.  Only RCOSC, OSC0, or PLL0 can drive
 * the main clock.
 */

/* The RCOSC frequency needs to be calibrated! */

#define AVR32_FRCOSC         115000     /* RCOSC frequency in Hz. 115KHz nominal */

#define AVR32_FOSC32         32768      /* OSC32 frequency in Hz */
#define AVR32_OSC32STARTUP   3          /* OSC32 startup time in RCOSC periods */

#define AVR32_FOSC0          12000000   /* OSC0 frequency in Hz */
#define AVR32_OSC0STARTUP    3          /* OSC0 startup time in RCOSC periods.

/* #define AVR32_FOSC1       12000000      OSC1 frequency: Hz.
 * #define AVR32_OSC1STARTUP 3             OSC1 startup time in RCOSC periods.
 */

/* PLL setup
 *
 *   FOSC0 MUL DIV PLL DIV2_EN CPU_CLOCK PBA_CLOCK   COMMENT
 *   (MHz)         (MHz)        (MHz)    (MHz)
 *    12   15   1  192     1     12       12
 *    12    9   3   40     1     20       20    PLL out of spec
 *    12   15   1  192     1     24       12
 *    12    9   1  120     1     30       15
 *    12    9   3   40     0     40       20    PLL out of spec
 *    12   15   1  192     1     48       12
 *    12   15   1  192     1     48       24
 *    12    8   1  108     1     54       27
 *    12    9   1  120     1     60       15
 *    12    9   1  120     1     60       30
 *    12   10   1  132     1     66       16.5
 */

#define AVR32_CLOCK_PLL0_OSC0 1
#undef AVR32_CLOCK_PLL0_OSC1
#define AVR32_PLL0_MUL        15
#define AVR32_PLL0_DIV        1
#define AVR32_PLL0_DIV2       1
#define AVR32_PLL0_WBWM       0
#define AVR32_PLL0_FREQ       192000000

/* Set PLL1 @ 96 MHz from OSC0: 12MHz*(7+1)/1 = 96MHz */

#define AVR32_CLOCK_PLL1_OSC0 1
#undef AVR32_CLOCK_PLL1_OSC1
#define AVR32_PLL1_MUL        7
#define AVR32_PLL1_DIV        1
#define AVR32_PLL1_DIV2       1
#define AVR32_PLL1_WBWM       0
#define AVR32_PLL1_FREQ       96000000

/* Clock divider setup */

#define AVR32_CKSEL_CPUDIV    0
#define AVR32_CKSEL_HSBDIV    0
#define AVR32_CKSEL_PBADIV    0
#define AVR32_CKSEL_PBBDIV    0

/* GCLK_USBB */

#undef  AVR32_CLOCK_USB_PLL0
#define AVR32_CLOCK_USB_PLL1  1
#undef  AVR32_CLOCK_USB_OSC0
#undef  AVR32_CLOCK_USB_OSC1
#define AVR32_CLOCK_USB_DIV   0

/* Main Clock settup: Select OSC0 as the main clock.
 *
 * - A 12Mhz crystal is provided on he board for OSC0.
 * - The AVR32DEV1 board has no support for OSC1.
 * - There are pads for he 32Khz OSC32, but it is not populated on the
 *   board.
 */

#define AVR32_CLOCK_OSC0      1
#undef  AVR32_CLOCK_OSC1          /* Not supported */
#undef  AVR32_CLOCK_OSC32         /* Not populated */
#undef  AVR32_CLOCK_PLL0
#undef  AVR32_CLOCK_PLL1

#define AVR32_CPU_CLOCK       AVR32_FOSC0
#define AVR32_PBA_CLOCK       AVR32_FOSC0

/* Pin muliplexing selecion *********************************************************/

#define PINMUX_USART1_RXD     PINMUX_USART1_RXD_1
#define PINMUX_USART1_TXD     PINMUX_USART1_TXD_1

/* LED definitions ******************************************************************/
/* The AVR32DEV1 board has 3 LEDs, two of which can be controlled through GPIO pins */

                                 /*    ON        OFF    */
                                 /* LED1 LED2 LED1 LED2 */
#define LED_STARTED           0  /* OFF  OFF  OFF  OFF  */
#define LED_HEAPALLOCATE      0  /* OFF  OFF  OFF  OFF  */
#define LED_IRQSENABLED       0  /* OFF  OFF  OFF  OFF  */
#define LED_STACKCREATED      1  /* ON   OFF  OFF  OFF  */
#define LED_INIRQ             2  /* ON   ON   ON   OFF  */
#define LED_SIGNAL            2  /* ON   ON   ON   OFF  */
#define LED_ASSERTION         2  /* ON   ON   ON   OFF  */
#define LED_PANIC             2  /* ON   ON   ON   OFF  */

/* Button definitions ***************************************************************/
/* The AVR32DEV1 board has 3 BUTTONs, two of which can be sensed through GPIO pins. */

#define BUTTON1               1 /* Bit 0: Button 1 */
#define BUTTON2               2 /* Bit 1: Button 2 */

/************************************************************************************
 * Public Types
 ************************************************************************************/

#ifndef __ASSEMBLY__

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
 * Name: avr32_boardinitialize
 *
 * Description:
 *   All AVR32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void avr32_boardinitialize(void);

/************************************************************************************
 * Name: up_buttoninit
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After
 *   that, up_buttons() may be called to collect the current state of all
 *   buttons or up_irqbutton() may be called to register button interrupt
 *   handlers.
 *
 * NOTE: Nothing in the "base" NuttX code calls up_buttoninit().  If you want button
 * support in an application, your application startup code must call up_buttoninit()
 * prior to calling any of the other button interfaces.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);

/************************************************************************************
 * Name: up_buttons
 *
 * Description:
 *   After up_buttoninit() has been called, up_buttons() may be called to collect
 *   the state of all buttons.  up_buttons() returns an 8-bit bit set with each bit
 *   associated with a button.  See the BUTTON* definitions above for the meaning of
 *   each bit in the returned value.
 *
 ************************************************************************************/

EXTERN uint8_t up_buttons(void);

/************************************************************************************
 * Name: up_irqbutton
 *
 * Description:
 *   This function may be called to register an interrupt handler that will be
 *   called when a button is depressed or released.  The ID value is one of the
 *   BUTTON* definitions provided above. The previous interrupt handler address is
 *   returned (so that it may restored, if so desired).
 *
 * Configuration Notes:
 *   Configuration CONFIG_AVR32_GPIOIRQ must be selected to enable the overall GPIO
 *   IRQ feature and CONFIG_AVR32_GPIOIRQSETA and/or CONFIG_AVR32_GPIOIRQSETB must
 *   be enabled to select GPIOs to support interrupts on.  For button support, bits
 *   2 and 3 must be set in CONFIG_AVR32_GPIOIRQSETB (PB2 and PB3).
 *
 ************************************************************************************/

#ifdef CONFIG_AVR32_GPIOIRQ
EXTERN xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif /* CONFIG_ARCH_BUTTONS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */

