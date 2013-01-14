/****************************************************************************
 * arch/arm/src/kinetis/kinetis_clockconfig.c
 * arch/arm/src/chip/kinetis_clockconfig.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "kinetis_internal.h"
#include "kinetis_mcg.h"
#include "kinetis_sim.h"
#include "kinetis_fmc.h"
#include "kinetis_llwu.h"
#include "kinetis_pinmux.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifndef CONFIG_ARCH_RAMFUNCS
# error "CONFIG_ARCH_RAMFUNCS must be defined for this logic"
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
 
void __ramfunc__
kinesis_setdividers(uint32_t div1, uint32_t div2, uint32_t div3, uint32_t div4);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinesis_portclocks
 *
 * Description:
 *   Enable all of the port clocks
 *
 ****************************************************************************/

static inline void kinesis_portclocks(void)
{
  uint32_t regval;

  /* Enable all of the port clocks */

  regval = getreg32(KINETIS_SIM_SCGC5);
  regval |= (SIM_SCGC5_PORTA | SIM_SCGC5_PORTB | SIM_SCGC5_PORTC |
             SIM_SCGC5_PORTD | SIM_SCGC5_PORTE);
  putreg32(regval, KINETIS_SIM_SCGC5);
}

/****************************************************************************
 * Name: kinetis_pllconfig
 *
 * Description:
 *   Initialize the PLL using the settings in board.h.  This assumes that
 *   the MCG is in default FLL Engaged Internal (FEI mode) out of reset.
 *
 ****************************************************************************/

void kinetis_pllconfig(void)
{
  uint32_t regval32;
  uint8_t regval8;

  /* Transition to FLL Bypassed External (FBE) mode */

#ifdef BOARD_EXTCLOCK
  /*   IRCS  = 0 (Internal Reference Clock Select)
   *   LP    = 0 (Low Power Select)
   *   EREFS = 0 (External Reference Select)
   *   HGO   = 0 (High Gain Oscillator Select)
   *   RANGE = 0 (Oscillator of 32 kHz to 40 kHz)
   */

  putreg8(0, KINETIS_MCG_C2);
#else
  /* Enable external oscillator:
   *
   *   IRCS  = 0 (Internal Reference Clock Select)
   *   LP    = 0 (Low Power Select)
   *   EREFS = 1 (External Reference Select)
   *   HGO   = 1 (High Gain Oscillator Select)
   *   RANGE = 2 (Oscillator of 8 MHz to 32 MHz)
   */

  putreg8(MCG_C2_EREFS | MCG_C2_HGO | MCG_C2_RANGE_VHIGH, KINETIS_MCG_C2);
#endif

  /* Released latched state of oscillator and GPIO */

  regval32 = getreg32(KINETIS_SIM_SCGC4);
  regval32 |= SIM_SCGC4_LLWU;
  putreg32(regval32, KINETIS_SIM_SCGC4);

  regval8 = getreg8(KINETIS_LLWU_CS);
  regval8 |= LLWU_CS_ACKISO;
  putreg8(regval8, KINETIS_LLWU_CS);

  /* Select external oscillator and Reference Divider and clear IREFS to
   * start the external oscillator.
   *
   *   IREFSTEN = 0 (Internal Reference Stop Enable)
   *   IRCLKEN  = 0 (Internal Reference Clock Enable)
   *   IREFS    = 0 (Internal Reference Select)
   *   FRDIV    = 3 (FLL External Reference Divider, RANGE!=0 divider=256)
   *   CLKS     = 2 (Clock Source Select, External reference clock)
   */

  putreg8(MCG_C1_FRDIV_DIV256 | MCG_C1_CLKS_EXTREF, KINETIS_MCG_C1);

  /* If we aren't using an oscillator input we don't need to wait for the
   * oscillator to initialize
   */

#ifndef BOARD_EXTCLOCK
  while ((getreg8(KINETIS_MCG_S) & MCG_S_OSCINIT) == 0);
#endif

  /* Wait for Reference clock Status bit to clear */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_IREFST) != 0);

  /* Wait for clock status bits to show that the clock source is the
   * external reference clock.
   */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_CLKST_MASK) != MCG_S_CLKST_EXTREF);

  /* We are now in  FLL Bypassed External (FBE) mode. Configure PLL
   * reference clock divider:
   *
   *   PLLCLKEN = 0
   *   PLLSTEN  = 0
   *   PRDIV    = Determined by PLL reference clock frequency
   *
   * Either the external clock or crystal frequency is used to select the
   * PRDIV value. Only reference clock frequencies are supported that will
   * produce a 2MHz reference clock to the PLL.
   */

  putreg8(MCG_C5_PRDIV(BOARD_PRDIV), KINETIS_MCG_C5);

  /* Ensure that MCG_C6 is at the reset default of 0: LOLIE disabled, PLL
   * disabled, clk monitor disabled, PLL VCO divider cleared.
   */

  putreg8(0, KINETIS_MCG_C6);

  /* Set system options dividers based on settings from the board.h file.
   *
   * MCG         = PLL
   * Core        = MCG / BOARD_OUTDIV1
   * bus         = MCG / BOARD_OUTDIV2
   * FlexBus     = MCG / BOARD_OUTDIV3
   * Flash clock = MCG / BOARD_OUTDIV4
   */

  kinesis_setdividers(BOARD_OUTDIV1, BOARD_OUTDIV2, BOARD_OUTDIV3, BOARD_OUTDIV4); 
 
  /* Set the VCO divider, VDIV, is defined in the board.h file.  VDIV
   * selects the amount to divide the VCO output of the PLL. The VDIV bits
   * establish the multiplication factor applied to the reference clock
   * frequency.  Also set
   *
   * LOLIE       = 0 (Loss of Lock Interrrupt Enable)
   * PLLS        = 1 (PLL Select)
   * CME         = 0 (Clock Monitor Enable)
   */

  putreg8(MCG_C6_PLLS | MCG_C6_VDIV(BOARD_VDIV), KINETIS_MCG_C6);

  /* Wait for the PLL status bit to set */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_PLLST) == 0);

  /* Wait for the PLL LOCK bit to set */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_LOCK) == 0);

  /* We are now running in PLL Bypassed External (PBE) mode.  Transition to
   * PLL Engaged External (PEE) mode by setting CLKS to 0
   */

  regval8 = getreg8(KINETIS_MCG_C1);
  regval8 &= ~MCG_C1_CLKS_MASK;
  putreg8(regval8, KINETIS_MCG_C1);

  /* Wait for clock status bits to update */

  while ((getreg8(KINETIS_MCG_S) & MCG_S_CLKST_MASK) != MCG_S_CLKST_PLL);

  /* We are now running in PLL Engaged External (PEE) mode. */
}

/****************************************************************************
 * Name: kinetis_traceconfig
 *
 * Description:
 *   Enable trace clocks.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_TRACE
static inline void kinetis_traceconfig(void)
{
  uint32_t regval;

  /* Set the trace clock to the core clock frequency in the SIM SOPT2 register */

  regval  = getreg32(KINETIS_SIM_SOPT2);
  regval |= SIM_SOPT2_TRACECLKSEL;
  putreg32(regval, KINETIS_SIM_SOPT2);

  /* Enable the TRACE_CLKOUT pin function on the configured pin */

  kinetis_gpioconfig(GPIO_TRACE_CLKOUT);
}
#else
#  define kinetis_traceconfig()
#endif

/****************************************************************************
 * Name: kinetis_fbconfig
 *
 * Description:
 *   Enable FlexBus clocking.
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_FLEXBUS
static inline void kinetis_fbconfig(void)
{
  uint32_t regval;

  /* Enable the clock to the FlexBus module */

  regval  = getreg32(KINETIS_SIM_SCGC7);
  regval |= SIM_SCGC7_FLEXBUS;
  putreg32(regval, KINETIS_SIM_SCGC7);

  /* Enable the FB_CLKOUT function on PTC3 (alt5 function) */

  kinetis_gpioconfig(GPIO_FB_CLKOUT);
}
#else
#  define kinetis_fbconfig()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_clockconfig
 *
 * Description:
 *   Called to initialize the Kinetis chip.  This does whatever setup is
 *   needed to put the  MCU in a usable state.  This includes the
 *   initialization of clocking using the settings in board.h.
 *
 ****************************************************************************/

void kinetis_clockconfig(void)
{
  /* Enable all of the port clocks */

  kinesis_portclocks();

  /* Configure the PLL based on settings in the board.h file */

  kinetis_pllconfig();

  /* For debugging, we will normally want to enable the trace clock and/or
   * the FlexBus clock.
   */

  kinetis_traceconfig();
  kinetis_fbconfig();
}

/****************************************************************************
 * Name: kinesis_setdividers
 *
 * Description:
 *  "This routine must be placed in RAM. It is a workaround for errata e2448.
 *   Flash prefetch must be disabled when the flash clock divider is changed.
 *   This cannot be performed while executing out of flash.  There must be a
 *   short delay after the clock dividers are changed before prefetch can be
 *   re-enabled."
 *
 * NOTE: This must have global scope only to prevent optimization logic from
 *   inlining the function.
 *
 ****************************************************************************/

void __ramfunc__
kinesis_setdividers(uint32_t div1, uint32_t div2, uint32_t div3, uint32_t div4)
{
  uint32_t regval;
  int i;

  /* Save the current value of the Flash Access Protection Register */

  regval = getreg32(KINETIS_FMC_PFAPR);
  
  /* Set M0PFD through M7PFD to 1 to disable prefetch */

  putreg32(FMC_PFAPR_M7PFD | FMC_PFAPR_M6PFD | FMC_PFAPR_M5PFD |
           FMC_PFAPR_M4PFD | FMC_PFAPR_M3PFD | FMC_PFAPR_M2PFD |
           FMC_PFAPR_M1PFD | FMC_PFAPR_M0PFD,
           KINETIS_FMC_PFAPR);

  /* Set clock dividers to desired value */

  putreg32(SIM_CLKDIV1_OUTDIV1(div1) | SIM_CLKDIV1_OUTDIV2(div2) |
           SIM_CLKDIV1_OUTDIV3(div3) | SIM_CLKDIV1_OUTDIV4(div4),
           KINETIS_SIM_CLKDIV1);

  /* Wait for dividers to change */

  for (i = 0 ; i < div4 ; i++);
  
  /* Re-store the saved value of FMC_PFAPR */

  putreg32(regval, KINETIS_FMC_PFAPR);
}



