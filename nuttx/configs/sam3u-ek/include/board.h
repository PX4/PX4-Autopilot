/************************************************************************************
 * configs/sam3u-ek/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2009-2011 Gregory Nutt. All rights reserved.
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
#include "sam3u_internal.h"

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the sam3u device is running on a 4MHz internal RC.  These
 * definitions will configure clocking with MCK = 48MHz, PLLA = 96, and CPU=48MHz.
 */

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST    (63 << CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration */

#define BOARD_CKGR_PLLAR_MULA      (7 << CKGR_PLLAR_MULA_SHIFT)
#define BOARD_CKGR_PLLAR_STMODE    CKGR_PLLAR_STMODE_FAST
#define BOARD_CKGR_PLLAR_PLLACOUNT (63 << CKGR_PLLAR_PLLACOUNT_SHIFT)
#define BOARD_CKGR_PLLAR_DIVA      CKGR_PLLAR_DIVA_BYPASS

/* PMC master clock register settings */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV2

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT (3 << CKGR_UCKR_UPLLCOUNT_SHIFT)

/* Resulting frequencies */

#define SAM3U_MAINOSC_FREQUENCY    (12000000)
#define SAM3U_MCK_FREQUENCY        (48000000)
#define SAM3U_PLLA_FREQUENCY       (96000000)
#define SAM3U_CPU_FREQUENCY        (48000000)

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCI / MCI_SPEED / 2 - 1
 */

/* MCK = 48MHz, CLKDIV = 59, MCI_SPEED = 48MHz / 2 * (59+1) = 400 KHz */
  
#define HSMCI_INIT_CLKDIV      (59 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 48MHz, CLKDIV = 1, MCI_SPEED = 48MHz / 2 * (1+1) = 12 MHz */

#define HSMCI_MMCXFR_CLKDIV    (3 << HSMCI_MR_CLKDIV_SHIFT) 

/* MCK = 48MHz, CLKDIV = 0, MCI_SPEED = 48MHz / 2 * (0+1) = 24 MHz */

#define HSMCI_SDXFR_CLKDIV     (0 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV HSMCI_SDXFR_CLKDIV

/* LED definitions ******************************************************************/

#define LED_STARTED                0 /* LED0=OFF LED1=OFF LED2=OFF */
#define LED_HEAPALLOCATE           1 /* LED0=OFF LED1=OFF LED2=ON */
#define LED_IRQSENABLED            2 /* LED0=OFF LED1=ON  LED2=OFF */
#define LED_STACKCREATED           3 /* LED0=OFF LED1=ON  LED2=ON */

#define LED_INIRQ                  4 /* LED0=XXX LED1=TOG LED2=XXX */
#define LED_SIGNAL                 5 /* LED0=XXX LED1=XXX LED2=TOG */
#define LED_ASSERTION              6 /* LED0=TOG LED1=XXX LED2=XXX */
#define LED_PANIC                  7 /* LED0=TOG LED1=XXX LED2=XXX */

/* Button definitions ***************************************************************/

#define BUTTON1                    1 /* Bit 0: Button 1 */
#define BUTTON2                    2 /* Bit 1: Button 2 */

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
 * Name: sam3u_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

EXTERN void sam3u_boardinitialize(void);

/************************************************************************************
 * Name: up_buttoninit
 *
 * Description:
 *   up_buttoninit() must be called to initialize button resources.  After that,
 *   up_buttons() may be called to collect the current state of all buttons or
 *   up_irqbutton() may be called to register button interrupt handlers.
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
 ************************************************************************************/

#ifdef CONFIG_GPIOA_IRQ
EXTERN xcpt_t up_irqbutton(int id, xcpt_t irqhandler);
#endif
#endif /* CONFIG_ARCH_BUTTONS */

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __ARCH_BOARD_BOARD_H */
