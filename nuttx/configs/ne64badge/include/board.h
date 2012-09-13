/************************************************************************************
 * configs/ne64badge/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Frequency of the crystal oscillator */

#define HCS12_OSCCLK      16000000 /* 16MHz */

/* PLL Settings
 *
 * SYNR register controls the multiplication factor of the PLL. If the PLL is on, the
 * count in the loop divider (SYNR) register effectively multiplies up the PLL clock
 * (PLLCLK) from the reference frequency by 2 x (SYNR+1). PLLCLK will not be below
 * the minimum VCO frequency (fSCM).
 *
 * The REFDV register provides a finer granularity for the PLL multiplier steps. The
 * count in the reference divider divides OSCCLK frequency by REFDV + 1.
 *
 *   PLLCLK = 2 * OSCCLK * (SYNR + 1) / (REFDV + 1)
 *
 * If (PLLSEL = 1), Bus Clock = PLLCLK / 2
 */

#define HCS12_SYNR_VALUE  0x15
#define HCS12_REFDV_VALUE 0x15
#define HCS12_PLLCLK      (2*HCS12_OSCCLK*(HCS12_SYNR+1)/(HCS12_REFDV+1))
#define HCS12_BUSCLK      (HSC12_PLLCLK/2)

/* LED definitions ******************************************************************/

/* The NE64 Badge board has 2 red LEDs that we will encode as: */

                             /* LED1 LED2 */
#define LED_STARTED       0  /*  OFF  OFF = Still initializing */
#define LED_HEAPALLOCATE  0  /*  OFF  OFF = Still initializing */
#define LED_IRQSENABLED   0  /*  OFF  OFF = Still initializing */
#define LED_STACKCREATED  1  /*  ON   OFF = Initialization complete */
#define LED_INIRQ         2  /*  NC   ON  = In an interrupt handler */
#define LED_SIGNAL        2  /*  NC   ON  = In a signal handler */
#define LED_ASSERTION     2  /*  NC   ON  = In an assertion */
#define LED_PANIC         2  /*  NC   ON  = Oops! We crashed. (flashing) */

/* Button definitions ***************************************************************/

/* The NE64 badge has 2 switches, one labed SW1 and the other SW2 on the board (but
 * referred to as button1 and button2 in some documentation)
 */

#define BUTTON1           1 /* Bit 0: SW1 button is depressed */
#define BUTTON2           2 /* Bit 1: SW2 button is depressed */

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
 * Name: hcs12_boardinitialize
 *
 * Description:
 *   All HCS12 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void hcs12_boardinitialize(void);

/************************************************************************************
 * Button support.
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After that,
 *   up_buttons() may be called to collect the state of all buttons.  up_buttons()
 *   returns an bit set with bits corresponding to the state of each button.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_BUTTONS
EXTERN void up_buttoninit(void);
EXTERN uint8_t up_buttons(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
