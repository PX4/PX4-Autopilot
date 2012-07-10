/****************************************************************************
 *  arch/arm/src/lpc43/lpc43_cgu.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <errno.h>

#include "up_arch.h"
#include "lpc43_cgu.h"
#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Maximum/Threashold Frequencies *******************************************/

#define LOW_XTAL_FREQUENCY     15000000
#define MAX_XTAL_FREQUENCY     25000000

#define MAX_FCLKOUT_FREQUENCY 156000000
#define MAX_FCCO_FRQUENCY     320000000

/* Configuration ************************************************************/
/* This supports configuration of CGU clocking from board-specific parameters
 * that must be provided in the board.h header file.
 *
 * That header file must provided the following values:
 *
 * BOARD_XTAL_FREQUENCY - The LPC43xx XTAL oscillator input frequency
 */

#ifndef BOARD_XTAL_FREQUENCY
#  error "board.h must provide the LPC43xx cystal input frequency (BOARD_XTAL_FREQUENCY)"
#endif

#if BOARD_XTAL_FREQUENCY >= MAX_XTAL_FREQUENCY
#  error "BOARD_XTAL_FREQUENCY exceeds the maximum value"
#endif

#if !defined(BOARD_PLL1_DIRECT) && (BOARD_FCLKOUT_FREQUENCY > MAX_FCLKOUT_FREQUENCY)
#  error "BOARD_FCLKOUT_FREQUENCY exceed the maximum"
#endif

#if BOARD_FCCO_FREQUENCY > MAX_FCCO_FRQUENCY
#  error "BOARD_FCCO_FREQUENCY exceed the maximum"
#endif

/* Convert the user-friendly definitions in board.h to register bit settings */

#if BOARD_PLL_PSEL == 1
#  define CTRL_PSEL_VALUE PLL1_CTRL_PSEL_DIV1
#elif BOARD_PLL_PSEL == 2
#  define CTRL_PSEL_VALUE PLL1_CTRL_PSEL_DIV2
#elif BOARD_PLL_PSEL == 4
#  define CTRL_PSEL_VALUE PLL1_CTRL_PSEL_DIV4
#elif BOARD_PLL_PSEL == 8
#  define CTRL_PSEL_VALUE PLL1_CTRL_PSEL_DIV8
#else
#  error "Unsupported value of BOARD_PLL_PSEL"
#endif

#if BOARD_PLL_NSEL == 1
#  define CTRL_NSEL_VALUE PLL1_CTRL_NSEL_DIV1
#elif BOARD_PLL_NSEL == 2
#  define CTRL_NSEL_VALUE PLL1_CTRL_NSEL_DIV2
#elif BOARD_PLL_NSEL == 3
#  define CTRL_NSEL_VALUE PLL1_CTRL_NSEL_DIV3
#elif BOARD_PLL_NSEL == 4
#  define CTRL_NSEL_VALUE PLL1_CTRL_NSEL_DIV4
#else
#  error "Unsupported value of BOARD_PLL_NSEL"
#endif

#if (BOARD_PLL_MSEL > 0) && (BOARD_PLL_MSEL < 256)
#  define CTRL_MSEL_VALUE PLL1_CTRL_MSEL(BOARD_PLL_MSEL)
#else
#  error "Unsupported value of BOARD_PLL_NSEL"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_xtalconfig
 *
 * Description:
 *   Configure the cystal input to PLL1 using the settings provided in
 *   the board.h file.
 *
 ****************************************************************************/

static inline void lpc43_xtalconfig(void)
{
  /* Configure the crystal input to PLL1 */

  uint32_t regval;

  /* Set/clear the HF bit in the crystal oscillator control register.
   * - The bit must be cleared if low-frequency oscillators (<=15MHz)
   * - The HF bit must be set for high-frequency osciallators (>20MHz)
   * - For oscillators in the range 15-20 MHz, the HF setting does not matter.
   */

  regval = getreg32(LPC43_XTAL_OSC_CTRL);
#if BOARD_XTAL_FREQUENCY <= LOW_XTAL_FREQUENCY
  regval &= ~XTAL_OSC_CTRL_HF;
#else
  regval |= XTAL_OSC_CTRL_HF;
#endif
  putreg32(regval, LPC43_XTAL_OSC_CTRL);

  /* Enable the crystal oscillator by taking it out of power down mode */

  regval &= ~XTAL_OSC_CTRL_ENABLE;
  putreg32(regval, LPC43_XTAL_OSC_CTRL);

  /* Delay for stable clock input */

  up_mdelay(20);

  /* Select the crystal oscillator as the input to PLL1 */

  regval = getreg32(LPC43_PLL1_CTRL);
  regval &= PLL1_CTRL_CLKSEL_MASK;
  regval |= PLL1_CLKSEL_XTAL | PLL1_CTRL_AUTOBLOCK;
  putreg32(regval, LPC43_PLL1_CTRL);
}

/****************************************************************************
 * Name: lpc43_clockconfig
 *
 * Description:
 *   Configure PLL1 dividers and multipliers per the settings in the board.h
 *   file to generate the desired Fclkcout and Fcco frequencies.
 *
 ****************************************************************************/

static inline void lpc43_pll1config(void)
{
  uint32_t regval;

  /* Clear PLL1 controls:
   *
   *   - PLL1_CTRL_BYPASS:    Input clock bypass control
   *   - PLL1_CTRL_FBSEL:     PLL1 feedback select
   *   - PLL1_CTRL_DIRECT:    PLL1 direct CCO output
   *   - PLL1_CTRL_PSEL_MASK: Post-divider division ratio P (psel)
   *   - PLL1_CTRL_NSEL_MASK: Pre-divider division ratio N (nsel)
   *   - PLL1_CTRL_MSEL_MASK: Feedback-divider division ratio M (msel)
   */

  regval  = getreg32(LPC43_PLL1_CTRL);
  regval &= ~(PLL1_CTRL_BYPASS    | PLL1_CTRL_FBSEL     | PLL1_CTRL_DIRECT |
              PLL1_CTRL_PSEL_MASK | PLL1_CTRL_NSEL_MASK |
              PLL1_CTRL_MSEL_MASK);
  putreg32(regval, LPC43_PLL1_CTRL);
  
  /* Clear PLL1 controls:
   *
   *   - PLL1_CTRL_FBSEL:     Set in both integer and direct modes
   *   - PLL1_CTRL_DIRECT:    Set in direct mode
   *   - PLL1_CTRL_PSEL:      Set to the value from board.h
   *   - PLL1_CTRL_NSEL:      Set to the value from board.h
   *   - PLL1_CTRL_MSEL:      Set to the value from board.h
   */

#ifdef BOARD_PLL1_DIRECT
  regval |= (PLL1_CTRL_FBSEL | PLL1_CTRL_DIRECT | CTRL_PSEL_VALUE |
             CTRL_NSEL_VALUE | CTRL_MSEL_VALUE);
#else
  regval |= (PLL1_CTRL_FBSEL | CTRL_PSEL_VALUE  | CTRL_NSEL_VALUE |
             CTRL_MSEL_VALUE);
#endif
  putreg32(regval, LPC43_PLL1_CTRL);
}

/****************************************************************************
 * Name: lpc43_pll1enable
 *
 * Description:
 *   Take PLL1 out of power-down mode and wait until it is locked onto the
 *   input clock.
 *
 ****************************************************************************/

static inline void lpc43_pll1enable(void)
{
  uint32_t regval;

  /* Take PLL1 out of power down mode.  The reset state of the PD bit
   * is one, i.e., powered down.
   */
  
  regval  = getreg32(LPC43_PLL1_CTRL);
  regval &= ~PLL1_CTRL_PD;
  putreg32(regval, LPC43_PLL1_CTRL);

  /* When the power-down mode is terminated, PPL1 will resume its normal
   * operation and will make the lock signal high once it has regained
   * lock on the input clock
   *
   * Wait for PLL1 to report that it is locked.
   */

  while ((getreg32(LPC43_PLL1_STAT) & PLL1_STAT_LOCK) == 0);
}

/****************************************************************************
 * Name: lpc43_m4clkselect
 *
 * Description:
 *   Select PLL1 output as the Cortex-M4 source clock.
 *
 ****************************************************************************/

static inline void lpc43_m4clkselect(void)
{
  uint32_t regval;

  regval = getreg32(LPC43_BASE_M4_CLK);
  regval &= ~BASE_M4_CLK_CLKSEL_MASK;
  regval |= BASE_M4_CLKSEL_PLL1;
  putreg32(regval, LPC43_BASE_M4_CLK);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_clockconfig
 *
 * Description:
 *   Called to initialize the LPC43XX.  This does whatever setup is needed
 *   to put the MCU in a usable state.  This includes the initialization of
 *   clocking using the settings in board.h.
 *
 ****************************************************************************/

void lpc43_clockconfig(void)
{
  /* Configure the crystal input to PLL1 */

  lpc43_xtalconfig();

  /* Configure PLL1 */

  lpc43_pll1config();

  /* Enable PLL1 */

  lpc43_pll1enable();

  /* Set up PLL1 output as the M4 clock */

  lpc43_m4clkselect();
}
