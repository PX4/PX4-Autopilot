/****************************************************************************
 * config/olimex-lpc1766stk/src/up_lcd.c
 * arch/arm/src/board/up_lcd.c
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
 * POSSPBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/nokia6100.h>

#include "up_arch.h"
#include "lpc17_syscon.h"
#include "lpc17_pwm.h"
#include "lpc17_gpio.h"
#include "lpc1766stk_internal.h"

#if defined(CONFIG_NX_LCDDRIVER) && defined(CONFIG_LCD_NOKIA6100) && defined(CONFIG_LPC17_SSP0)

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER != 127
#  error "CONFIG_LCD_MAXPOWER must be 127"
#endif

/* Backlight OFF PWM setting */

#define NOKIA_BACKLIGHT_OFF 0x40

/* Define the CONFIG_LCD_NOKIADBG to enable detailed debug output (stuff you
 * would never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_NOKIADBG
#endif

#ifdef CONFIG_LCD_NOKIADBG
#  define lcddbg(format, arg...)  vdbg(format, ##arg)
#  define lcd_dumpgpio(m) lpc17_dumpgpio(LPC1766STK_LCD_RST, m)
#else
#  define lcddbg(x...)
#  define lcd_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nokia_blinitialize
 *
 * Description:
 *   Initialize PWM1 to manage the LCD backlight.
 *
 ****************************************************************************/

void nokia_blinitialize(void)
{
  uint32_t regval;

  /* Enable clocking of PWM1 */

  regval = getreg32(LPC17_SYSCON_PCONP);
  regval |= SYSCON_PCONP_PCPWM1;
  putreg32(regval, LPC17_SYSCON_PCONP);

  /* Disable and reset PWM1 */

  regval  = getreg32(LPC17_PWM1_TCR);
  regval &= ~(PWM_TCR_PWMEN|PWM_TCR_CNTREN);
  regval |= PWM_TCR_CNTRRST;
  putreg32(regval, LPC17_PWM1_TCR);

  /* Put PWM1 in timer mode */

  regval  = getreg32(LPC17_PWM1_CTCR);
  regval &= ~PWM_CTCR_MODE_MASK;
  regval |= PWM_CTCR_MODE_TIMER;
  putreg32(regval, LPC17_PWM1_CTCR);

  /* Reset on MR0 */

  putreg32(PWM_MCR_MR0R, LPC17_PWM1_MCR);

  /* Single edge controlled mod for PWM3 and enable output */

  regval  = getreg32(LPC17_PWM1_PCR);
  regval &= ~PWM_PCR_SEL3;
  regval |= PWM_PCR_ENA3;
  putreg32(regval, LPC17_PWM1_PCR);

  /* Clear prescaler */

  putreg32(0, LPC17_PWM1_PR);

  /* Set 8-bit resolution */

  putreg32(0xff, LPC17_PWM1_MCR);

  /* Enable PWM match 1 latch */

  regval  = getreg32(LPC17_PWM1_LER);
  regval |= PWM_LER_M0EN;
  putreg32(regval, LPC17_PWM1_LER);

  /* Clear match register 3 */

  putreg32(0, LPC17_PWM1_MR3);

  /* Enable PWM1 */
  
  regval |= PWM_LER_M3EN;
  putreg32(regval, LPC17_PWM1_LER);

  regval  = getreg32(LPC17_PWM1_TCR);
  regval &= ~(PWM_TCR_CNTRRST);
  regval |= (PWM_TCR_PWMEN|PWM_TCR_CNTREN);
  putreg32(regval, LPC17_PWM1_TCR);

  nokia_backlight(0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_nxdrvinit
 *
 * Description:
 *   Called NX initialization logic to configure the LCD.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *up_nxdrvinit(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

  /* Configure the LCD GPIOs */

  lcd_dumpgpio("up_nxdrvinit: On entry");
  lpc17_configgpio(LPC1766STK_LCD_RST);
  lpc17_configgpio(LPC1766STK_LCD_BL);
  lcd_dumpgpio("up_nxdrvinit: After GPIO setup");

  /* Reset the LCD */

  lpc17_gpiowrite(LPC1766STK_LCD_RST, false);
  up_udelay(10);
  lpc17_gpiowrite(LPC1766STK_LCD_RST, true);
  up_mdelay(5);

  /* Configure PWM1 to support the backlight */

  nokia_blinitialize();

  /* Get the SSP port (configure as a Freescale SPI port) */

  spi = up_spiinitialize(0);
  if (!spi)
    {
      glldbg("Failed to initialize SSP port 0\n");
    }
  else
    {
      /* Bind the SSP port to the LCD */

      dev = nokia_lcdinitialize(spi, devno);
      if (!dev)
        {
          glldbg("Failed to bind SSP port 0 to LCD %d: %d\n", devno);
        }
     else
        {
          gllvdbg("Bound SSP port 0 to LCD %d\n", devno);

          /* And turn the LCD on (CONFIG_LCD_MAXPOWER should be 1) */

          (void)dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name:  nokia_backlight
 *
 * Description:
 *   The Nokia 6100 backlight is controlled by logic outside of the LCD
 *   assembly.  This function must be provided by board specific logic to
 *   manage the backlight.  This function will receive a power value (0:
 *   full off - CONFIG_LCD_MAXPOWER: full on) and should set the backlight
 *   accordingly.
 *
 *   On the Olimex LPC1766STK, the backlight level is controlled by PWM1.
 *
 ****************************************************************************/

int nokia_backlight(unsigned int power)
{
  uint32_t regval;

  putreg32(NOKIA_BACKLIGHT_OFF + power, LPC17_PWM1_MR3);
  
  regval  = getreg32(LPC17_PWM1_LER);
  regval |= PWM_LER_M3EN;
  putreg32(regval, LPC17_PWM1_LER);
  return OK;
}

#endif /* CONFIG_NX_LCDDRIVER && CONFIG_LCD_NOKIA6100 && CONFIG_LPC17_SSP0 */
