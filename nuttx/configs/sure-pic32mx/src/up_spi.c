/************************************************************************************
 * configs/sure-pic32mx/src/up_spi.c
 * arch/arm/src/board/up_spi.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include "up_arch.h"
#include "chip.h"
#include "pic32mx-internal.h"
#include "sure-internal.h"

#if defined(CONFIG_PIC32MX_SPI2)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* The Sure PIC32MX has an SD slot connected on SPI2:
 *
 * SPI
 *  SCK2/PMA5/CN8/RG6    SCK    SD connector SCK, FLASH (U1) SCK*
 *  SDI2/PMA4/CN9/RG7    SDI    SD connector DO, FLASH (U1) SO*
 *  SDO2/PMA3/CN10/RG8   SDO    SD connector DI, FLASH (U1) SI*
 *
 * Chip Select.  Pulled up on-board
 *  TDO/AN11/PMA12/RB11  SD_CS  SD connector CS
 *
 * Status inputs.  All pulled up on-board
 *
 *  TCK/AN12/PMA11/RB12  SD_CD  SD connector CD
 *  TDI/AN13/PMA10/RB13  SD_WD  SD connector WD
 */

#define GPIO_SD_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN11)
#define GPIO_SD_CD (GPIO_INPUT|GPIO_INT|GPIO_PORTB|GPIO_PIN12)
#define GPIO_SD_WD (GPIO_INPUT|GPIO_PORTB|GPIO_PIN13)

/* Change notification numbers -- Not available for SD_CD. */

/* The following enable debug output from this file.
 * 
 * CONFIG_DEBUG_SPI && CONFIG_DEBUG - Define to enable basic SPI debug
 * CONFIG_DEBUG_VERBOSE - Define to enable verbose SPI debug
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_SPI
#  undef CONFIG_DEBUG_VERBOSE
#endif

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  ifdef CONFIG_DEBUG_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  define spidbg(x...)
#  define spivdbg(x...)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: pic32mx_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Sure PIC32MX board.
 *
 ************************************************************************************/

void weak_function pic32mx_spiinitialize(void)
{
  /* Configure the SPI2 chip select (CS) GPIO output, and the card detect (CD) and
   * write protect (WP) inputs.
   */

  pic32mx_configgpio(GPIO_SD_CS);
  pic32mx_configgpio(GPIO_SD_CD);
  pic32mx_configgpio(GPIO_SD_WD);
}

/************************************************************************************
 * Name:  pic32mx_spi2select and pic32mx_spi2status
 *
 * Description:
 *   The external functions, pic32mx_spi2select and pic32mx_spi2status 
 *   must be provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common PIC32MX logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI/SPI chip select
 *      pins.
 *   2. Provide pic32mx_spi2select() and pic32mx_spi2status() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

#ifdef CONFIG_PIC32MX_SPI2
void pic32mx_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spivdbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  /* The SD card chip select is pulled high and active low */

  if (devid == SPIDEV_MMCSD)
    {
      pic32mx_gpiowrite(GPIO_SD_CS, !selected);
    }
}

uint8_t pic32mx_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  uint8_t ret = 0;

  /* Card detect is pull up on-board.  If a low value is sensed then the card must
   * be present.
   */

  if (!pic32mx_gpioread(GPIO_SD_CD))
    {
      ret = SPI_STATUS_PRESENT;

      /* It seems that a high value indicatest the the card is write protected. */

      if (pic32mx_gpioread(GPIO_SD_WD))
        {
          ret |= SPI_STATUS_WRPROTECTED;
        }
    }
        
  spivdbg("Returning %d\n", ret);
  return ret;
}
#endif
#endif /* CONFIG_PIC32MX_SPI2 */
