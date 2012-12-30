/****************************************************************************
 * config/zp214xpa/src/up_ug2864ambag01.c
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

#include "up_arch.h"
#include "chip.h"
#include "lpc214x_pinsel.h"

#ifdef CONFIG_LCD_UG2864AMBAG01

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The pin configurations here requires that SPI1 is avaialable */

/* SPI should be configured with CMD/DATA support (and no transfer methods) */

#ifndef CONFIG_SPI_CMDDATA
#  error "The OLED driver requires CONFIG_SPI_CMDDATA in the configuration"
#endif

/* Pin Configuration ********************************************************/
/* UG-2864AMBAG01 OLED Display:
 *
 *   PIN NAME PIN CONFIGURATION
 *    1  3V3
 *    2  5V
 *    3  RESET P0.18/CAP1.3/MISO1/MAT1.3P0.18 RESET - General purpose output
 *    4  DI    P0.19/MAT1.2/MOSI1/CAP1.2P0.19 DI    - Alternate function 2
 *    5  CS    P0.20/MAT1.3/SSEL1/EINT3             - General purpose output
 *    6  SCK   P0.17/CAP1.2/SCK1/MAT1.2             - Alternate function 2
 *    7  A0    P0.23/VBUS                           - General purpose output
 *    8  N/C   LED-
 *    9  N/C   LED+ (BL)
 *   10  GND
 *
 * Definitions and configuration for DO, DI, CS, in up_spi1.c
 */

/* Use either FIO or legacy GPIO */

#ifdef CONFIG_LPC214x_FIO
#  define RESET_PIN_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_PIN_OFFSET)
#  define RESET_SET_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_SET_OFFSET)
#  define RESET_CLR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_CLR_OFFSET)
#  define RESET_DIR_REGISTER (LPC214X_FIO0_BASE+LPC214X_FIO_DIR_OFFSET)
#else
#  define RESET_PIN_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_PIN_OFFSET)
#  define RESET_SET_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_SET_OFFSET)
#  define RESET_CLR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_CLR_OFFSET)
#  define RESET_DIR_REGISTER (LPC214X_GPIO0_BASE+LPC214X_GPIO_DIR_OFFSET)
#endif

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
  uint32_t regval32;
  uint32_t bits32;

  /* Configure multiplexed pins as connected on the ZP213X/4XPA board:
   *
   *   PINSEL1 P0.18/CAP1.3/MISO1/MAT1.3 Bits 4-5=00 for P0.18
   */

  regval32  = getreg32(LPC214X_PINSEL1);
  regval32 &= ~LPC214X_PINSEL1_P018_MASK;
  regval32 |= LPC214X_PINSEL1_P018_GPIO;
  putreg32(regval32, LPC214X_PINSEL1);

  /* Set the RESET line low, putting the OLED into the reset state. */

  bits32 = (1 << 18);
  putreg32(bits32, RESET_CLR_REGISTER);
  regval32 = getreg32(RESET_DIR_REGISTER);
  putreg32(regval32 | bits32, RESET_DIR_REGISTER);

  lcdvdbg("RESET Pin Config: PINSEL1: %08x PIN: %08x DIR: %08x\n",
          getreg32(LPC214X_PINSEL1), getreg32(RESET_PIN_REGISTER),
          getreg32(RESET_DIR_REGISTER));

  /* Wait a bit then release the OLED from the reset state */

  up_mdelay(20);
  putreg32(bits32, RESET_SET_REGISTER);

  lcdvdbg("RESET release: PIN: %08x DIR: %08x\n",
          getreg32(RESET_PIN_REGISTER), getreg32(RESET_DIR_REGISTER));

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
