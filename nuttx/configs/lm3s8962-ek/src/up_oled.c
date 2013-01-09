/****************************************************************************
 * config/lm3s8962-ek/src/up_oled.c
 * arch/arm/src/board/up_oled.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/p14201.h>

#include "lm_gpio.h"
#include "lm3s8962ek_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Define the CONFIG_LCD_RITDEBUG to enable detailed debug output (stuff you
 * would never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_RITDEBUG
#endif

#ifdef CONFIG_LCD_RITDEBUG
#  define ritdbg(format, arg...)  vdbg(format, ##arg)
#  define oleddc_dumpgpio(m) lm_dumpgpio(OLEDDC_GPIO, m)
#  define oledcs_dumpgpio(m) lm_dumpgpio(OLEDCS_GPIO, m)
#else
#  define ritdbg(x...)
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
 *   Called NX initialization logic to configure the OLED.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *up_nxdrvinit(unsigned int devno)
{
  FAR struct spi_dev_s *spi;
  FAR struct lcd_dev_s *dev;

  /* Configure the OLED GPIOs */

  oledcs_dumpgpio("up_nxdrvinit: After OLEDCS setup");
  oleddc_dumpgpio("up_nxdrvinit: On entry");

  lm_configgpio(OLEDDC_GPIO); /* PC7: OLED display data/control select (D/Cn) */
  lm_configgpio(OLEDEN_GPIO); /* PC6: Enable +15V needed by OLED (EN+15V) */

  oleddc_dumpgpio("up_nxdrvinit: After OLEDDC/EN setup");

  /* Get the SSI port (configure as a Freescale SPI port) */

  spi = up_spiinitialize(0);
  if (!spi)
    {
      glldbg("Failed to initialize SSI port 0\n");
    }
  else
    {
      /* Bind the SSI port to the OLED */

      dev = rit_initialize(spi, devno);
      if (!dev)
        {
          glldbg("Failed to bind SSI port 0 to OLED %d: %d\n", devno);
        }
     else
        {
          gllvdbg("Bound SSI port 0 to OLED %d\n", devno);

          /* And turn the OLED on (CONFIG_LCD_MAXPOWER should be 1) */

          (void)dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }
  return NULL;
}

/******************************************************************************
 * Name:  lm_spicmddata
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
 ******************************************************************************/

int lm_spicmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
  if (devid == SPIDEV_DISPLAY)
    {
      /* Set GPIO to 1 for data, 0 for command */

      lm_gpiowrite(OLEDDC_GPIO, !cmd);
      return OK;
    }
  return -ENODEV;
}
