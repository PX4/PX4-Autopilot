/**************************************************************************
 * arch/avr/src/at32uc3/at32uc3_clkinit.c
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "up_arch.h"

#include "at32uc3_config.h"
#include "up_internal.h"
#include "at32uc3_internal.h"
#include "at32uc3_pm.h"
#include "at32uc3_flashc.h"

/**************************************************************************
 * Private Definitions
 **************************************************************************/

#if defined(AVR32_CLOCK_OSC0) || \
    (defined (AVR32_CLOCK_PLL0) && defined(AVR32_CLOCK_PLL0_OSC0)) || \
    (defined (AVR32_CLOCK_PLL1) && defined(AVR32_CLOCK_PLL1_OSC0))
#  define NEED_OSC0
#endif

#if defined(AVR32_CLOCK_OSC1) || \
    (defined (AVR32_CLOCK_PLL0) && defined(AVR32_CLOCK_PLL0_OSC1)) || \
    (defined (AVR32_CLOCK_PLL1) && defined(AVR32_CLOCK_PLL1_OSC1))
#  define NEED_OSC1
#endif

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Global Variables
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_enableosc32
 *
 * Description:
 *   Initialiaze the 32KHz oscillaor.  This oscillaor is used by the RTC
 *   logic to provide the sysem timer.
 *
 **************************************************************************/

#ifdef AVR32_CLOCK_OSC32
static inline void up_enableosc32(void)
{
  uint32_t regval;
 
  /* Select the 32KHz oscillator crystal */

  regval = getreg32(AVR32_PM_OSCCTRL32);
  regval &= ~PM_OSCCTRL32_MODE_MASK;
  regval |= PM_OSCCTRL32_MODE_XTAL;
  putreg32(regval, AVR32_PM_OSCCTRL32);

  /* Enable the 32-kHz clock */

  regval = getreg32(AVR32_PM_OSCCTRL32);
  regval &= ~PM_OSCCTRL32_STARTUP_MASK;
  regval |= PM_OSCCTRL32_EN|(AVR32_OSC32STARTUP << PM_OSCCTRL32_STARTUP_SHIFT);
  putreg32(regval, AVR32_PM_OSCCTRL32);
}
#endif

/**************************************************************************
 * Name: up_enableosc0
 *
 * Description:
 *   Initialiaze OSC0 settings per the definitions in the board.h file.
 *
 **************************************************************************/

#ifdef NEED_OSC0
static inline void up_enableosc0(void)
{
  uint32_t regval;

  /* Enable OSC0 in the correct crystal mode by setting the mode value in OSCCTRL0 */

  regval  = getreg32(AVR32_PM_OSCCTRL0);
  regval &= ~PM_OSCCTRL_MODE_MASK;
#if AVR32_FOSC0 < 900000
  regval |= PM_OSCCTRL_MODE_XTALp9;  /* Crystal XIN 0.4-0.9MHz */
#elif AVR32_FOSC0 < 3000000
  regval |= PM_OSCCTRL_MODE_XTAL3;   /* Crystal XIN 0.9-3.0MHz */
#elif AVR32_FOSC0 < 8000000
  regval |= PM_OSCCTRL_MODE_XTAL8;   /* Crystal XIN 3.0-8.0MHz */
#else
  regval |= PM_OSCCTRL_MODE_XTALHI;  /* Crystal XIN above 8.0MHz */
#endif
  putreg32(regval, AVR32_PM_OSCCTRL0);

  /* Enable OSC0 using the startup time provided in board.h.  This startup time
   * is critical and depends on the characteristics of the crystal.
   */

  regval  = getreg32(AVR32_PM_OSCCTRL0);
  regval &= ~PM_OSCCTRL_STARTUP_MASK;
  regval |= (AVR32_OSC0STARTUP << PM_OSCCTRL_STARTUP_SHIFT);
  putreg32(regval, AVR32_PM_OSCCTRL0);

  /* Enable OSC0 */

  regval = getreg32(AVR32_PM_MCCTRL);
  regval |= PM_MCCTRL_OSC0EN;
  putreg32(regval, AVR32_PM_MCCTRL);

  /* Wait for OSC0 to be ready */

  while ((getreg32(AVR32_PM_POSCSR) & PM_POSCSR_OSC0RDY) == 0);
}
#endif

/**************************************************************************
 * Name: up_enableosc1
 *
 * Description:
 *   Initialiaze OSC0 settings per the definitions in the board.h file.
 *
 **************************************************************************/

#ifdef NEED_OSC1
static inline void up_enableosc1(void)
{
  uint32_t regval;

  /* Enable OSC1 in the correct crystal mode by setting the mode value in OSCCTRL1 */

  regval  = getreg32(AVR32_PM_OSCCTRL1);
  regval &= ~PM_OSCCTRL_MODE_MASK;
#if AVR32_FOSC1 < 900000
  regval |= PM_OSCCTRL_MODE_XTALp9;  /* Crystal XIN 0.4-0.9MHz */
#elif AVR32_FOSC1 < 3000000
  regval |= PM_OSCCTRL_MODE_XTAL3;   /* Crystal XIN 0.9-3.0MHz */
#elif AVR32_FOSC1 < 8000000
  regval |= PM_OSCCTRL_MODE_XTAL8;   /* Crystal XIN 3.0-8.0MHz */
#else
  regval |= PM_OSCCTRL_MODE_XTALHI;  /* Crystal XIN above 8.0MHz */
#endif
  putreg32(regval, AVR32_PM_OSCCTRL1);

  /* Enable OSC1 using the startup time provided in board.h.  This startup time
   * is critical and depends on the characteristics of the crystal.
   */

  regval  = getreg32(AVR32_PM_OSCCTRL1);
  regval &= ~PM_OSCCTRL_STARTUP_MASK;
  regval |= (AVR32_OSC1STARTUP << PM_OSCCTRL_STARTUP_SHIFT);
  putreg32(regval, AVR32_PM_OSCCTRL1);

  /* Enable OSC1 */

  regval = getreg32(AVR32_PM_MCCTRL);
  regval |= PM_MCCTRL_OSC1EN;
  putreg32(regval, AVR32_PM_MCCTRL);

  /* Wait for OSC1 to be ready */

  while ((getreg32(AVR32_PM_POSCSR) & PM_POSCSR_OSC1RDY) == 0);
}
#endif

/**************************************************************************
 * Name: up_enablepll0
 *
 * Description:
 *   Initialiaze PLL0 settings per the definitions in the board.h file.
 *
 **************************************************************************/

#ifdef AVR32_CLOCK_PLL0
static inline void up_enablepll0(void)
{
  /* Setup PLL0 */
  
  regval = (AVR32_PLL0_DIV << PM_PLL_PLLDIV_SHIFT) | (AVR32_PLL0_MUL << PM_PLL_PLLMUL_SHIFT) | (16 << PM_PLL_PLLCOUNT_SHIFT)

  /* Select PLL0/1 oscillator */

#if AVR32_CLOCK_PLL_OSC1
  regval |= PM_PLL_PLLOSC;
#endif

  putreg32(regval, AVR32_PM_PLL0);

  /* Set PLL0 options */

  regval = getreg32(AVR32_PM_PLL0);
  regval &= ~PM_PLL_PLLOPT_MASK
#if AVR32_PLL0_FREQ < 160000000
  regval |= PM_PLL_PLLOPT_VCO;
#endif
#if AVR32_PLL0_DIV2 != 0
  regval |= PM_PLL_PLLOPT_XTRADIV;
#endif
#if AVR32_PLL0_WBWM != 0
  regval |= PM_PLL_PLLOPT_WBWDIS;
#endif
  putreg32(regval, AVR32_PM_PLL0)

  /* Enable PLL0 */

  regval = getreg32(AVR32_PM_PLL0);
  regval |= PM_PLL_PLLEN;
  putreg32(regval, AVR32_PM_PLL0)
  
  /* Wait for PLL0 locked. */

  while ((getreg32(AVR32_PM_POSCSR) & PM_POSCSR_LOCK0) == 0);
}
#endif

/**************************************************************************
 * Name: up_enablepll1
 *
 * Description:
 *   Initialiaze PLL1 settings per the definitions in the board.h file.
 *
 **************************************************************************/

#ifdef AVR32_CLOCK_PLL1
static inline void up_enablepll1(void)
{
  /* Setup PLL1 */
  
  regval = (AVR32_PLL1_DIV << PM_PLL_PLLDIV_SHIFT) | (AVR32_PLL1_MUL << PM_PLL_PLLMUL_SHIFT) | (16 << PM_PLL_PLLCOUNT_SHIFT)

  /* Select PLL0/1 oscillator */
 
#if AVR32_CLOCK_PLL_OSC1
  regval |= PM_PLL_PLLOSC;
#endif

  putreg32(regval, AVR32_PM_PLL1);

  /* Set PLL1 options */

  regval = getreg32(AVR32_PM_PLL1);
  regval &= ~PM_PLL_PLLOPT_MASK
#if AVR32_PLL1_FREQ < 160000000
  regval |= PM_PLL_PLLOPT_VCO;
#endif
#if AVR32_PLL1_DIV2 != 0
  regval |= PM_PLL_PLLOPT_XTRADIV;
#endif
#if AVR32_PLL1_WBWM != 0
  regval |= PM_PLL_PLLOPT_WBWDIS;
#endif
  putreg32(regval, AVR32_PM_PLL1)

  /* Enable PLL1 */

  regval = getreg32(AVR32_PM_PLL1);
  regval |= PM_PLL_PLLEN;
  putreg32(regval, AVR32_PM_PLL1)

  /* Wait for PLL1 locked. */

  while ((getreg32(AVR32_PM_POSCSR) & PM_POSCSR_LOCK1) == 0);
}
#endif

/**************************************************************************
 * Name: up_clksel
 *
 * Description:
 *   Configure derived clocks.
 *
 **************************************************************************/

static inline void up_clksel(void)
{
  uint32_t regval = 0;

#if AVR32_CKSEL_CPUDIV != 0
  regval |= PM_CKSEL_CPUDIV;
  regval |= (AVR32_CKSEL_CPUDIV << PM_CKSEL_CPUSEL_SHIFT)
#endif

#if AVR32_CKSEL_HSBDIV != 0
  regval |= PM_CKSEL_HSBDIV;
  regval |= (AVR32_CKSEL_HSBDIV << PM_CKSEL_HSBSEL_SHIFT)
#endif

#if AVR32_CKSEL_PBADIV != 0
  regval |= PM_CKSEL_PBADIV;
  regval |= (AVR32_CKSEL_PBADIV << PM_CKSEL_PBASEL_SHIFT)
#endif

#if AVR32_CKSEL_PBBDIV != 0
  regval |= PM_CKSEL_PBBDIV;
  regval |= (AVR32_CKSEL_PBBDIV << PM_CKSEL_PBBSEL_SHIFT)
#endif

  putreg32(regval, AVR32_PM_CKSEL);

  /* Wait for CLKRDY */

  while ((getreg32(AVR32_PM_POSCSR) & PM_POSCSR_CKRDY) == 0);
}

/**************************************************************************
 * Name: up_fws
 *
 * Description:
 *   Setup FLASH wait states.
 *
 **************************************************************************/

static void up_fws(uint32_t cpuclock)
{
  uint32_t regval;
 
  regval = getreg32(AVR32_FLASHC_FCR);
  if (cpuclock > AVR32_FLASHC_FWS0_MAXFREQ)
    {
      regval |= FLASHC_FCR_FWS;
    }
  else
    {
      regval &= ~FLASHC_FCR_FWS;
    }
  putreg32(regval, AVR32_FLASHC_FCR);
}

/**************************************************************************
 * Name: up_mainclk
 *
 * Description:
 *   Select the main clock.
 *
 **************************************************************************/

static inline void up_mainclk(uint32_t mcsel)
{
  uint32_t regval;
 
  regval = getreg32(AVR32_PM_MCCTRL);
  regval &= ~PM_MCCTRL_MCSEL_MASK;
  regval |= mcsel;
  putreg32(regval, AVR32_PM_MCCTRL);
}

/**************************************************************************
 * Name: up_usbclock
 *
 * Description:
 *   Setup the USBB GCLK.
 *
 **************************************************************************/

#ifdef CONFIG_USBDEV
static inline void up_usbclock(void)
{
  uint32_t regval = 0;

#if defined(AVR32_CLOCK_USB_PLL0) || defined(AVR32_CLOCK_USB_PLL1)
  regval |= PM_GCCTRL_PLLSEL;
#endif
#if defined(AVR32_CLOCK_USB_OSC1) || defined(AVR32_CLOCK_USB_PLL1)
  regval |= PM_GCCTRL_OSCSEL;
#endif
#if AVR32_CLOCK_USB_DIV > 0


  u_avr32_pm_gcctrl.GCCTRL.diven  = diven;
  u_avr32_pm_gcctrl.GCCTRL.div    = div;
#endif
  putreg32(regval, AVR32_PM_GCCTRL(AVR32_PM_GCLK_USBB))

  /* Enable USB GCLK */
 
  regval = getreg32(AVR32_PM_GCCTRL(AVR32_PM_GCLK_USBB))
  regval |= PM_GCCTRL_CEN;
  putreg32(regval, AVR32_PM_GCCTRL(AVR32_PM_GCLK_USBB))
}
#endif

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_clkinit
 *
 * Description:
 *   Initialiaze clock/PLL settings per the definitions in the board.h
 *   file.
 *
 **************************************************************************/
 
void up_clkinitialize(void)
{
#ifdef AVR32_CLOCK_OSC32
  /* Enable the 32KHz oscillator (need by the RTC module) */

  up_enableosc32();
#endif

#ifdef NEED_OSC0
  /* Enable OSC0 using the settings in board.h */

  up_enableosc0();

  /* Set up FLASH wait states */

  up_fws(AVR32_FOSC0);
 
  /* Then switch the main clock to OSC0 */

  up_mainclk(PM_MCCTRL_MCSEL_OSC0);
#endif

#ifdef NEED_OSC1
  /* Enable OSC1 using the settings in board.h */

  up_enableosc1();
#endif

#ifdef AVR32_CLOCK_PLL0
  /* Enable PLL0 using the settings in board.h */

  up_enablepll0();
 
  /* Set up FLASH wait states */

  up_fws(AVR32_CPU_CLOCK);
 
  /* Then switch the main clock to PLL0 */

  up_mainclk(PM_MCCTRL_MCSEL_PLL0);
#endif

#ifdef AVR32_CLOCK_PLL1
  /* Enable PLL1 using the settings in board.h */

  up_enablepll1();
#endif

  /* Configure derived clocks */

  up_clksel();

  /* Set up the USBB GCLK */

#ifdef CONFIG_USBDEV
  void up_usbclock();
#endif
}



