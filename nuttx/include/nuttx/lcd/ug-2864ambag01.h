/**************************************************************************************
 * include/nuttx/lcd/ug-2864ambag01.h
 *
 * Driver for Univision UG-2864AMBAG01 OLED display (wih SH1101A controller) in SPI
 * mode
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. Product Specification (Preliminary), Part Name: OEL Display Module, Part ID:
 *      UG-2864AMBAG01, Doc No: SASI-9015-A, Univision Technology Inc.
 *   2. SH1101A, 132 X 64 Dot Matrix OLED/PLED, Preliminary Segment/Common Driver with
 *      Controller, Sino Wealth
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
 **************************************************************************************/

#ifndef __INCLUDE_NUTTX_UG_8264AMBAG01_H
#define __INCLUDE_NUTTX_UG_8264AMBAG01_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <nuttx/arch.h>

#ifdef CONFIG_LCD_UG2864AMBAG01

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/
/* UG-2864AMBAG01 Configuration Settings:
 *
 * CONFIG_UG2864AMBAG01_SPIMODE - Controls the SPI mode
 * CONFIG_UG2864AMBAG01_FREQUENCY - Define to use a different bus frequency
 * CONFIG_UG2864AMBAG01_NINTERFACES - Specifies the number of physical UG-2864AMBAG01
 *   devices that will be supported.
 *
 * Required LCD driver settings:
 *
 * CONFIG_LCD_UG28AMBAG01 - Enable UG-2864AMBAG01 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 * CONFIG_LCD_MAXPOWER must be 1
 *
 * Option LCD driver settings:
 * CONFIG_LCD_LANDSCAPE, CONFIG_LCD_PORTRAIT, CONFIG_LCD_RLANDSCAPE, and
 *   CONFIG_LCD_RPORTRAIT - Display orientation.
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* SPI Interface
 *
 * "The serial interface consists of serial clock SCL, serial data SI, A0 and
 *  CS . SI is shifted into an 8-bit shift register on every rising edge of
 *  SCL in the order of D7, D6, … and D0. A0 is sampled on every eighth clock
 *  and the data byte in the shift register is written to the display data RAM
 *  or command register in the same clock."
 *
 * MODE 3:
 *   Clock polarity:  High (CPOL=1)
 *   Clock phase:     Sample on trailing (rising edge) (CPHA 1)
 */

#ifndef CONFIG_UG2864AMBAG01_SPIMODE
#  define CONFIG_UG2864AMBAG01_SPIMODE SPIDEV_MODE3
#endif

/* "This module determines whether the input data is interpreted as data or
 * command. When A0 = “H”, the inputs at D7 - D0 are interpreted as data and be
 * written to display RAM. When A0 = “L”, the inputs at D7 - D0 are interpreted
 * as command, they will be decoded and be written to the corresponding command
 * registers.
 */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be defined in your NuttX configuration"
#endif

/* CONFIG_UG2864AMBAG01_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_UG2864AMBAG01_NINTERFACES
#  define CONFIG_UG2864AMBAG01_NINTERFACES 1
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#if CONFIG_LCD_MAXCONTRAST <= 0|| CONFIG_LCD_MAXCONTRAST > 255
#  error "CONFIG_LCD_MAXCONTRAST exceeds supported maximum"
#endif

#if CONFIG_LCD_MAXCONTRAST < 255
#  warning "Optimal setting of CONFIG_LCD_MAXCONTRAST is 255"
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER != 1
#  warning "CONFIG_LCD_MAXPOWER exceeds supported maximum"
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

/* Color is 1bpp monochrome with leftmost column contained in bits 0  */

#ifdef CONFIG_NX_DISABLE_1BPP
#  warning "1 bit-per-pixel support needed"
#endif

/* Orientation */

#if defined(CONFIG_LCD_LANDSCAPE)
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RLANDSCAPE
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_PORTRAIT)
#  undef CONFIG_LCD_LANDSCAPE
#  undef CONFIG_LCD_RLANDSCAPE
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  undef CONFIG_LCD_LANDSCAPE
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_RPORTRAIT)
#  undef CONFIG_LCD_LANDSCAPE
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RLANDSCAPE
#else
#  define CONFIG_LCD_LANDSCAPE 1
#  warning "Assuming landscape orientation"
#endif

/* Some important "colors" */

#define UG_Y1_BLACK  0
#define UG_Y1_WHITE  1

/**************************************************************************************
 * Public Types
 **************************************************************************************/

/**************************************************************************************
 * Public Data
 **************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************
 * Public Function Prototypes
 **************************************************************************************/

/**************************************************************************************
 * Name:  ug2864ambag01_initialize
 *
 * Description:
 *   Initialize the UG-2864AMBAG01 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_UG2864AMBAG01_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

struct lcd_dev_s; /* See include/nuttx/lcd/lcd.h */
struct spi_dev_s; /* See include/nuttx/spi.h */
FAR struct lcd_dev_s *ug2864ambag01_initialize(FAR struct spi_dev_s *spi,
                                               unsigned int devno);

/************************************************************************************************
 * Name:  ug2864ambag01_fill
 *
 * Description:
 *   This non-standard method can be used to clear the entire display by writing one
 *   color to the display.  This is much faster than writing a series of runs.
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 **************************************************************************************/

void ug2864ambag01_fill(FAR struct lcd_dev_s *dev, uint8_t color);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_LCD_UG2864AMBAG01 */
#endif /* __INCLUDE_NUTTX_UG_8264AMBAG01_H */
