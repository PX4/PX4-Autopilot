/****************************************************************************
 * include/nuttx/lcd/nokia6100.h
 * Application interface to the Nokia 6100 LCD display
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

#ifndef __INCLUDE_NUTTX_NOKIA6100_H
#define __INCLUDE_NUTTX_NOKIA6100_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nokia 6100 Configuration Settings:
 *
 * CONFIG_NOKIA6100_SPIMODE - Controls the SPI mode
 * CONFIG_NOKIA6100_FREQUENCY - Define to use a different bus frequency
 * CONFIG_NOKIA6100_NINTERFACES - Specifies the number of physical Nokia
 *   6100 devices that will be supported.
 * CONFIG_NOKIA6100_BPP - Device supports 8, 12, and 16 bits per pixel.
 * CONFIG_NOKIA6100_S1D15G10 - Selects the Epson S1D15G10 display controller
 * CONFIG_NOKIA6100_PCF8833 - Selects the Phillips PCF8833 display controller
 * CONFIG_NOKIA6100_BLINIT - Initial backlight setting
 *
 * The following may need to be tuned for your hardware:
 * CONFIG_NOKIA6100_INVERT - Display inversion, 0 or 1, Default: 1
 * CONFIG_NOKIA6100_MY - Display row direction, 0 or 1, Default: 0
 * CONFIG_NOKIA6100_MX - Display column direction, 0 or 1, Default: 1
 * CONFIG_NOKIA6100_V - Display address direction, 0 or 1, Default: 0
 * CONFIG_NOKIA6100_ML - Display scan direction, 0 or 1, Default: 0
 * CONFIG_NOKIA6100_RGBORD - Display RGB order, 0 or 1, Default: 0
 *
 * Required LCD driver settings:
 * CONFIG_LCD_NOKIA6100 - Enable Nokia 6100 support
 * CONFIG_LCD_MAXCONTRAST - must be 63 with the Epson controller and 127 with
 *   the Phillips controller.
 * CONFIG_LCD_MAXPOWER - Maximum value of backlight setting.  The backlight
 *   control is managed outside of the 6100 driver so this value has no
 *   meaning to the driver.  Board-specific logic may place restrictions on
 *   this value.
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/**************************************************************************************
 * Name:  nokia_lcdinitialize
 *
 * Description:
 *   Initialize the NOKIA6100 video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throuh CONFIG_NOKIA6100_NINTERFACES-1.  This
 *     allows support for multiple LCD devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the specified
 *   LCD.  NULL is returned on any failure.
 *
 **************************************************************************************/

struct lcd_dev_s; /* see nuttx/lcd.h */
struct spi_dev_s; /* see nuttx/spi.h */
EXTERN FAR struct lcd_dev_s *nokia_lcdinitialize(FAR struct spi_dev_s *spi, unsigned int devno);

/**************************************************************************************
 * Name:  nokia_backlight
 *
 * Description:
 *   The Nokia 6100 backlight is controlled by logic outside of the LCD assembly.  This
 *   function must be provided by board specific logic to manage the backlight.  This
 *   function will receive a power value (0: full off - CONFIG_LCD_MAXPOWER: full on)
 *   and should set the backlight accordingly.
 *
 **************************************************************************************/

EXTERN int nokia_backlight(unsigned int power);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_NOKIA6100_H */
