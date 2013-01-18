/****************************************************************************
 * arch/arm/src/lpc17xx/lpc17_clockconfig.c
 * arch/arm/src/chip/lpc17_clockconfig.c
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "up_internal.h"
#include "lpc17_clockconfig.h"
#include "chip/lpc17_syscon.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: lpc17_clockconfig
 *
 * Description:
 *   Called to initialize the LPC17xx.  This does whatever setup is needed to put the
 *   SoC in a usable state.  This includes the initialization of clocking using the
 *   settings in board.h.
 *
 ************************************************************************************/

void lpc17_clockconfig(void)
{
  /* Enable the main oscillator (or not) and the frequency range of the main oscillator */

  putreg32(BOARD_SCS_VALUE, LPC17_SYSCON_SCS);

  /* Wait for the main oscillator to be ready. */

#ifdef CONFIG_LPC17_MAINOSC
  while ((getreg32(LPC17_SYSCON_SCS) & SYSCON_SCS_OSCSTAT) == 0);
#endif

  /* Setup up the divider value for the CPU clock.  The output of the divider is CCLK.
   * The input to the divider (PLLCLK) is equal to SYSCLK unless PLL0 is enabled. CCLK
   * will be further divided to produce peripheral clocks, but that peripheral clock
   * setup is performed in the peripheral device drivers.  Here only CCLK is
   * configured.
   */

  putreg32(BOARD_CCLKCFG_VALUE, LPC17_SYSCON_CCLKCFG);

  /* PLL0 is used to generate the CPU clock divider input (PLLCLK). */

#ifdef CONFIG_LPC17_PLL0
  /* Select the PLL0 source clock, multiplier, and pre-divider values.  NOTE that
   * a special "feed" sequence must be written to the PLL0FEED register in order
   * for changes to the PLL0CFG register to take effect.
   */

  putreg32(BOARD_CLKSRCSEL_VALUE, LPC17_SYSCON_CLKSRCSEL);
  putreg32(BOARD_PLL0CFG_VALUE, LPC17_SYSCON_PLL0CFG);
  putreg32(0xaa, LPC17_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_SYSCON_PLL0FEED);

  /* Enable the PLL. NOTE that a special "feed" sequence must be written to the
   * PLL0FEED register in order for changes to the PLL0CON register to take effect.
   */

  putreg32(SYSCON_PLLCON_PLLE, LPC17_SYSCON_PLL0CON);
  putreg32(0xaa, LPC17_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_SYSCON_PLL0FEED);

  /* Wait for PLL0 to lock */

  while ((getreg32(LPC17_SYSCON_PLL0STAT) & SYSCON_PLL0STAT_PLOCK) == 0);

  /* Enable and connect PLL0 */

  putreg32(SYSCON_PLLCON_PLLE|SYSCON_PLLCON_PLLC, LPC17_SYSCON_PLL0CON);
  putreg32(0xaa, LPC17_SYSCON_PLL0FEED);
  putreg32(0x55, LPC17_SYSCON_PLL0FEED);

  /* Wait for PLL to report that it is connected and enabled */

  while ((getreg32(LPC17_SYSCON_PLL0STAT) & (SYSCON_PLL0STAT_PLLE|SYSCON_PLL0STAT_PLLC))
          != (SYSCON_PLL0STAT_PLLE|SYSCON_PLL0STAT_PLLC));
#endif

  /* PLL1 receives its clock input from the main oscillator only and can be used to
   * provide a fixed 48 MHz clock only to the USB subsystem (if that clock cannot be
   * obtained from PLL0).
   */

#ifdef CONFIG_LPC17_PLL1
  /* Select the PLL1 multiplier, and pre-divider values.  NOTE that a special "feed"
   * sequence must be written to the PLL1FEED register in order for changes to the
   * PLL1CFG register to take effect.
   */

  putreg32(BOARD_PLL1CFG_VALUE, LPC17_SYSCON_PLL1CFG);
  putreg32(0xaa, LPC17_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_SYSCON_PLL1FEED);

  /* Enable the PLL. NOTE that a special "feed" sequence must be written to the
   * PLL1FEED register in order for changes to the PLL1CON register to take effect.
   */

  putreg32(SYSCON_PLLCON_PLLE, LPC17_SYSCON_PLL1CON);
  putreg32(0xaa, LPC17_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_SYSCON_PLL1FEED);

  /* Wait for PLL1 to lock */

  while ((getreg32(LPC17_SYSCON_PLL1STAT) & SYSCON_PLL1STAT_PLOCK) == 0);

  /* Enable and connect PLL1 */

  putreg32(SYSCON_PLLCON_PLLE|SYSCON_PLLCON_PLLC, LPC17_SYSCON_PLL1CON);
  putreg32(0xaa, LPC17_SYSCON_PLL1FEED);
  putreg32(0x55, LPC17_SYSCON_PLL1FEED);

  /* Wait for PLL to report that it is connected and enabled */

  while ((getreg32(LPC17_SYSCON_PLL1STAT) & (SYSCON_PLL1STAT_PLLE|SYSCON_PLL1STAT_PLLC))
         != (SYSCON_PLL1STAT_PLLE|SYSCON_PLL1STAT_PLLC));
#else
  /* Otherwise, setup up the USB clock divider to generate the USB clock from PLL0 */

  putreg32(BOARD_USBCLKCFG_VALUE, LPC17_SYSCON_USBCLKCFG);
#endif

  /* Disable all peripheral clocks.  They must be configured by each device driver
   * when the device driver is initialized.
   */

  putreg32(0, LPC17_SYSCON_PCLKSEL0);
  putreg32(0, LPC17_SYSCON_PCLKSEL1);

  /* Disable power to all peripherals (execpt GPIO).  Peripherals must be re-powered
   * one at a time by each device driver when the driver is initialized.
   */

  putreg32(SYSCON_PCONP_PCGPIO, LPC17_SYSCON_PCONP);

  /* Disable CLKOUT */

  putreg32(0, LPC17_SYSCON_CLKOUTCFG);

  /* Configure FLASH */

#ifdef CONFIG_LP17_FLASH
  putreg32(BOARD_FLASHCFG_VALUE, LPC17_SYSCON_FLASHCFG);
#endif
}

