/****************************************************************************
 * config/stm32f4discovery/src/up_ug2864ambag01.c
 * arch/arm/src/board/up_ug2864ambag01.c
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

#include <debug.h>

#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ug-2864ambag01.h>

#include "stm32_gpio.h"
#include "stm32f4discovery-internal.h"

#ifdef CONFIG_LCD_UG2864AMBAG01

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The pin configurations here require that SPI1 is selected */

#ifndef CONFIG_STM32_SPI1
#  error "The OLED driver requires CONFIG_STM32_SPI1 in the configuration"
#endif

#ifndef CONFIG_SPI_CMDDATA
#  error "The OLED driver requires CONFIG_SPI_CMDDATA in the configuration"
#endif

/* Pin Configuration ********************************************************/
/* UG-2864AMBAG01 OLED Display (SPI 4-wire):
 *
 * --------------------------+----------------------------------------------
 * Connector CON10 J1:      | STM32F4Discovery
 * --------------+-----------+----------------------------------------------
 * CON10 J1:     | CON20 J2: | P1/P2:
 * --------------+-----------+----------------------------------------------
 * 1  3v3        | 3,4 3v3   | P2 3V
 * 3  /RESET     | 8 /RESET  | P2 PB6 (Arbitrary selection)
 * 5  /CS        | 7 /CS     | P2 PB7 (Arbitrary selection)(2)
 * 7  A0         | 9 A0      | P2 PB8 (Arbitrary selection)(2)
 * 9  LED+ (N/C) | -----     | -----
 * 2  5V Vcc     | 1,2 Vcc   | P2 5V
 * 4  DI         | 18 D1/SI  | P1 PA7 (GPIO_SPI1_MOSI == GPIO_SPI1_MOSI_1 (1))
 * 6  SCLK       | 19 D0/SCL | P1 PA5 (GPIO_SPI1_SCK == GPIO_SPI1_SCK_1 (2))
 * 8  LED- (N/C) | -----     | ------
 * 10 GND        | 20 GND    | P2 GND
 * --------------+-----------+----------------------------------------------
 * (1) Required because of on-board MEMS
 * (2) Note that the OLED CS and A0 are managed in the up_spi.c file.
 * -------------------------------------------------------------------------
 */

/* Definitions in stm32f4discovery-internal.h */

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, arg...)   dbg(format, ##arg)
#  define lcdvdbg(format, arg...)  vdbg(format, ##arg)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
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

  /* Configure the OLED GPIOs. This initial configuration is RESET low,
   * putting the OLED into reset state.
   */
 
  (void)stm32_configgpio(GPIO_OLED_RESET);

  /* Wait a bit then release the OLED from the reset state */

  up_mdelay(20);
  stm32_gpiowrite(GPIO_OLED_RESET, true);

  /* Get the SPI1 port interface */

  spi = up_spiinitialize(1);
  if (!spi)
    {
      lcddbg("Failed to initialize SPI port 1\n");
    }
  else
    {
      /* Bind the SPI port to the OLED */

      dev = ug2864ambag01_initialize(spi, devno);
      if (!dev)
        {
          lcddbg("Failed to bind SPI port 1 to OLED %d: %d\n", devno);
        }
     else
        {
          lcdvdbg("Bound SPI port 1 to OLED %d\n", devno);

          /* And turn the OLED on */

          (void)dev->setpower(dev, CONFIG_LCD_MAXPOWER);
          return dev;
        }
    }

  return NULL;
}
#endif /* CONFIG_LCD_UG2864AMBAG01 */
