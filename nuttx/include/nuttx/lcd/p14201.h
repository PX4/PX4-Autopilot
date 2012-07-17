/****************************************************************************
 * include/nuttx/lcd/p14201.h
 * Application interface to the RiT P14201 OLED driver
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

#ifndef __INCLUDE_NUTTX_P14201_H
#define __INCLUDE_NUTTX_P14201_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* P14201 Configuration Settings:
 *
 * CONFIG_P14201_SPIMODE - Controls the SPI mode
 * CONFIG_P14201_FREQUENCY - Define to use a different bus frequency
 * CONFIG_P14201_NINTERFACES - Specifies the number of physical P14201 devices that
 *   will be supported.
 * CONFIG_P14201_FRAMEBUFFER - If defined, accesses will be performed using an in-memory
 *   copy of the OLEDs GDDRAM.  This cost of this buffer is 128 * 96 / 2 = 6Kb.  If this
 *   is defined, then the driver will be fully functional. If not, then it will have the
 *   following limitations:
 *
 *   - Reading graphics memory cannot be supported, and
 *   - All pixel writes must be aligned to byte boundaries.
 *
 *   The latter limitation effectively reduces the 128x96 disply to 64x96.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_P14201 - Enable P14201 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 * CONFIG_LCD_MAXPOWER must be 1
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* Some important "colors" */

#define RIT_Y4_BLACK 0x00
#define RIT_Y4_WHITE 0x0f

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
 * Name:  rit_initialize
 *
 * Description:
 *   Initialize the P14201 video hardware.  The initial state of the OLED is fully
 *   initialized, display memory cleared, and the OLED ready to use, but with the power
 *   setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throuh CONFIG_P14201_NINTERFACES-1.  This allows
 *   support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the specified
 *   OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

struct lcd_dev_s; /* see nuttx/lcd.h */
struct spi_dev_s; /* see nuttx/spi.h */
EXTERN FAR struct lcd_dev_s *rit_initialize(FAR struct spi_dev_s *spi, unsigned int devno);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_P14201_H */
