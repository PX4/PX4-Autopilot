/************************************************************************************
 * configs/detron/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Authors:
 *      Decio Renno <http://www.detroneletronica.com.br>
 *      Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <arch/irq.h>          //GPIO IRQ
#include <arch/lpc17xx/irq.h>

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


/* Button definitions ***************************************************************/

#define panel_pulse_up                   1 /* Bit 0: painel_pulsada_up */
#define panel_pulse_down                 2 /* Bit 1: painel_pulsada_down */
#define panel_center                     4 /* Bit 2: painel_pulsada_centro */

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
 * Name: lpc17_boardinitialize
 *
 * Description:
 *   All LPC17xx architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void lpc17_boardinitialize(void);


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
 * Name: up_up_irq_pin_panel_pulse_up/down/center
 *
 * Description:
 *   These functions may be called to register an interrupt handler that will be
 *   called when BUTTON UP/DOWN/CENTER is depressed.  The previous interrupt handler value is
 *   returned (so that it may restored, if so desired).
 *
 ************************************************************************************/


EXTERN xcpt_t up_irq_pin_panel_pulse_up(xcpt_t irqhandler);
EXTERN xcpt_t up_irq_pin_panel_pulse_down(xcpt_t irqhandler);
EXTERN xcpt_t up_irq_pin_panel_center(xcpt_t irqhandler);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
