/****************************************************************************
 * config/lpcxpresso-lpc1768/src/up_oled.c
 * arch/arm/src/board/up_oled.c
 *
 *   Copyright (C) 2011, 2013 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ug-9664hswag01.h>

#include "lpc17_gpio.h"
#include "lpc17_ssp.h"
#include "lpcxpresso_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* This module is only built if CONFIG_NX_LCDDRIVER is selected.  In this
 * case, it would be an error if SSP1 is not also enabled.
 */

#ifndef CONFIG_LPC17_SSP1
#  error "The OLED driver requires CONFIG_LPC17_SSP1 in the configuration"
#endif

#ifndef CONFIG_UG9664HSWAG01_POWER
#  error "This logic requires CONFIG_UG9664HSWAG01_POWER in the configuration"
#endif

/* Debug ********************************************************************/
/* Define the CONFIG_DEBUG_LCD to enable detailed debug output (stuff you
 * would never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

#ifdef CONFIG_DEBUG_LCD
#  define ugdbg(format, arg...)  vdbg(format, ##arg)
#  define oleddc_dumpgpio(m)     lpc17_dumpgpio(LPCXPRESSO_OLED_POWER, m)
#  define oledcs_dumpgpio(m)     lpc17_dumpgpio(LPCXPRESSO_OLED_CS, m)
#else
#  define ugdbg(x...)
#  define oleddc_dumpgpio(m)
#  define oledcs_dumpgpio(m)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_nxdrvinit
 *
 * Description:
 *   Called by NX initialization logic to configure the OLED.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *up_nxdrvinit(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

  /* Configure the OLED GPIOs. For the SPI interface, insert jumpers in J42,
   * J43, J45 pin1-2 and J46 pin 1-2.
   */
 
  oledcs_dumpgpio("up_nxdrvinit: After OLED CS setup");
  oleddc_dumpgpio("up_nxdrvinit: On entry");

  (void)lpc17_configgpio(LPCXPRESSO_OLED_POWER); /* OLED 11V power */
  (void)lpc17_configgpio(LPCXPRESSO_OLED_DC);    /* OLED Command/Data */

  oleddc_dumpgpio("up_nxdrvinit: After OLED Power/DC setup");

  /* Get the SSI port (configure as a Freescale SPI port) */

  spi = up_spiinitialize(1);
  if (!spi)
    {
      glldbg("Failed to initialize SSI port 1\n");
    }
  else
    {
      /* Bind the SSI port to the OLED */

      dev = ug_initialize(spi, devno);
      if (!dev)
        {
          glldbg("Failed to bind SSI port 1 to OLED %d: %d\n", devno);
        }
     else
        {
          gllvdbg("Bound SSI port 1 to OLED %d\n", devno);

          /* And turn the OLED on (dim) */

          (void)dev->setpower(dev, UG_POWER_DIM);
          return dev;
        }
    }
  return NULL;
}

/****************************************************************************
 * Name:  lpc17_ssp1cmddata
 *
 * Description:
 *   Set or clear the SD1329 D/Cn bit to select data (true) or command
 *   (false).  This function must be provided by platform-specific logic.
 *   This is an implementation of the cmddata method of the SPI
 *   interface defined by struct spi_ops_s (see include/nuttx/spi.h).
 *
 * Input Parameters:
 *
 *   spi - SPI device that controls the bus the device that requires the CMD/
 *         DATA selection.
 *   devid - If there are multiple devices on the bus, this selects which one
 *         to select cmd or data.  NOTE:  This design restricts, for example,
 *         one one SPI display per SPI bus.
 *   cmd - true: select command; false: select data
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

int lpc17_ssp1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY)
    {
      /* Set GPIO to 1 for data, 0 for command */

      (void)lpc17_gpiowrite(LPCXPRESSO_OLED_DC, !cmd);
      return OK;
    }
  return -ENODEV;
}

/****************************************************************************
 * Name:  ug_power
 *
 * Description:
 *   If the hardware supports a controllable OLED a power supply, this
 *   interface should be provided.  It may be called by the driver to turn
 *   the OLED power on and off as needed.
 *
 * Input Parameters:
 *
 *   devno - A value in the range of 0 throuh CONFIG_UG9664HSWAG01_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *   on - true:turn power on, false: turn power off.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_UG9664HSWAG01_POWER
void ug_power(unsigned int devno, bool on)
{
  gllvdbg("power %s\n", on ? "ON" : "OFF");
  (void)lpc17_gpiowrite(LPCXPRESSO_OLED_POWER, on);
}
#endif
