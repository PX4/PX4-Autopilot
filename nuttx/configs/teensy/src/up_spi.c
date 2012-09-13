/************************************************************************************
 * configs/teensy/src/up_spi.c
 * arch/arm/src/board/up_spi.c
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>
#include <avr/io.h>

#include "up_arch.h"
#include "chip.h"
#include "at90usb_internal.h"
#include "teensy_internal.h"

#ifdef CONFIG_AVR_SPI

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* Teensy SPI Connection
 *
 * -- ---- -- ------------------------- -------
 * J2 NAME PIN NAME                     PAD
 * -- ---- -- ------------------------- -------
 *  1 VIN  -- Connected to USB +RV
 *  2 GND  -- Connected to USB GND
 *  3 3V3  -- Not used                  ---
 *  4 NC   -- Not used
 *  5 CS   10 (SS/PCINT0) PB0           Pad B0
 *  6 DI   12 (PDI/PCINT2/MOSI) PB2     Pad B2
 *  7 SCK  11 (PCINT1/SCLK) PB1         Pad B1
 *  8 DO   13 (PDO/PCINT3/MISO) PB3     Pad B3
 *  9 IRQ  -- Not used                  ---
 * 10 CD   14 (PCINT4/OC.2A) PB4        Pad B4
 * 11 WP   15 (PCINT5/OC.1A) PB5        Pad B5
 * -- ---- -- ------------------------- -------
 */

#define TEENSY_CS (1 << 0)
#define TEENSY_CD (1 << 4)
#define TEENSY_WP (1 << 5)

/* The following enable debug output from this file (needs CONFIG_DEBUG too).
 * 
 * CONFIG_SPI_DEBUG - Define to enable basic SSP debug
 * CONFIG_SPI_VERBOSE - Define to enable verbose SSP debug
 */

#ifdef CONFIG_SPI_DEBUG
#  define sspdbg  lldbg
#  ifdef CONFIG_SPI_VERBOSE
#    define sspvdbg lldbg
#  else
#    define sspvdbg(x...)
#  endif
#else
#  undef CONFIG_SPI_VERBOSE
#  define sspdbg(x...)
#  define sspvdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: at90usb_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the LPC1766-STK.
 *
 ************************************************************************************/

void weak_function at90usb_spiinitialize(void)
{
  /* The Teensy board has no dedicated SPI devices so we assume that SS is used
   * for chip select:
   *
   *   "When the SPI is configured as a Master (MSTR in SPCR is set), the user
   *    can determine the direction of the SS pin.  If SS is configured as an
   *    output, the pin is a general output pin which does not affect the SPI
   *    system. ...
   *
   *   "If SS is configured as an input, it must be held high to ensure Master
   *    SPI operation. If the SS pin is driven low by peripheral circuitry when
   *    the SPI is configured as a Master with the SS pin defined as an input,
   *    the SPI system interprets this as another master selecting the SPI ...
   */

   DDRB  |= TEENSY_CS;                 /* B0 is an output */
   PORTB |= TEENSY_CS;                 /* Low de-selects */
   DDRB  &= ~(TEENSY_CD | TEENSY_WP);  /* B4 and B5 are inputs */
   PORTB |= (TEENSY_CD | TEENSY_WP);   /* Pull high */
}

/************************************************************************************
 * Name:  avr_spiselect and avr_spistatus
 *
 * Description:
 *   The external functions, avr_spiselect and avr_spistatus  must be provided by
 *   board-specific logic.  They are implementations of the select and status methods
 *   of the SPI interface defined by struct spi_ops_s (see include/nuttx/spi.h). All
 *   other methods (including up_spiinitialize()) are provided by common AVR logic.
 *   To use this common SPI logic on your board:
 *
 *   1. Provide logic in avr_sspinitialize() to configure SPI chip select pins.
 *   2. Provide avr_spiselect() and avr_spistatus() functions in your board-specific
 *      logic.  These functions will perform chip selection and status operations
 *      in the way your board is configured.
 *   3. Add a calls to at90usb_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling  mmcsd_spislotinitialize(),
 *      for example, will bind the SPI driver to the SPI MMC/SD driver).
 *
 ************************************************************************************/

void  avr_spiselect(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  sspdbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  /* Assert/de-assert the CS pin to the card */

  if (selected)
    {
       PORTB &= ~TEENSY_CS;
    }
  else
    {
       PORTB |= TEENSY_CS;
    }
}

uint8_t avr_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  uint8_t ret = 0;
  uint8_t regval = PINB;

  /* Both the CD and WP pins are pull high by the AT90USB and will be
   * grounded it a card is inserted or write protected.
   */

  if ((regval & TEENSY_CD) == 0)
    {
      ret |= SPI_STATUS_PRESENT;
    }
  
  if ((regval & TEENSY_WP) == 0)
    {
      ret |= SPI_STATUS_WRPROTECTED;
    }

  sspdbg("Returning %02x\n", ret);
  return ret;
}

#endif /* CONFIG_AVR_SPI */
