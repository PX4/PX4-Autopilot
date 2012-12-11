/************************************************************************************
 * configs/pic32mx7mmb/src/up_spi.c
 * arch/arm/src/board/up_spi.c
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
#include "pic32mx7mmb_internal.h"

#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2) || \
    defined(CONFIG_PIC32MX_SPI3) || defined(CONFIG_PIC32MX_SPI4)

/************************************************************************************
 * Definitions
 ************************************************************************************/
/* SPI1 and SD Card
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RC4   SPI1    SD card slot              SPI1 data IN
 *   RD0   SPO1    SD card slot              SPI1 data OUT
 *   RD10  SCK1    SD card slot              SD card, SPI clock
 *
 *   RA9   SD_CS#  SD card slot              SD card, SPI chip select (active low)
 *   RG6   SD_WP   SD card slot              SD card, write protect
 *   RG7   SD_CD#  SD card slot              SD card, card detect (not)
 */

#define GPIO_SD_CS (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTA|GPIO_PIN9)
#define GPIO_SD_WP (GPIO_INPUT|GPIO_PORTG|GPIO_PIN6)
#define GPIO_SD_CD (GPIO_INPUT|GPIO_INT|GPIO_PORTG|GPIO_PIN7)

/* The following enable debug output from this file (needs CONFIG_DEBUG too).
 * 
 * CONFIG_DEBUG_SPI - Define to enable basic SPI debug
 */

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  define spivdbg llvdbg
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
 *   Called to configure SPI chip select GPIO pins for the Mikroelektronka PIC32MX7
 *   MMB board.
 *
 ************************************************************************************/

void weak_function pic32mx_spiinitialize(void)
{
  /* Configure the SPI chip select, write protect, and card detect GPIOs */

#ifdef CONFIG_PIC32MX_SPI1
  pic32mx_configgpio(GPIO_SD_CS);
  pic32mx_configgpio(GPIO_SD_WP);
  pic32mx_configgpio(GPIO_SD_CD);
#endif
}

/************************************************************************************
 * Name:  pic32mx_spiNselect, pic32mx_spiNstatus, and pic32mx_spiNcmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They are
 *   implementations of the select, status, and cmddata methods of the SPI interface
 *   defined by struct spi_ops_s (see include/nuttx/spi.h). All other methods 
 *   including up_spiinitialize()) are provided by common PIC32MX logic.  To use
 *   this common SPI logic on your board:
 *
 *   1. Provide logic in pic32mx_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide pic32mx_spiNselect() and pic32mx_spiNstatus() functions
 *      in your board-specific logic.  These functions will perform chip selection
 *      and status operations using GPIOs in the way your board is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      pic32mx_spiNcmddata() functions in your board-specific logic.  These
 *      functions will perform cmd/data selection operations using GPIOs in the way
 *      your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ************************************************************************************/

struct spi_dev_s;
enum spi_dev_e;

#ifdef CONFIG_PIC32MX_SPI1
void  pic32mx_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");

  if (devid == SPIDEV_MMCSD)
    {
      pic32mx_gpiowrite(GPIO_SD_CS, !selected);
    }
}

uint8_t pic32mx_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  uint8_t ret = 0;

  /* Card detect active low. */

  if (devid == SPIDEV_MMCSD)
    {
      if (!pic32mx_gpioread(GPIO_SD_CD))
        {
          ret = SPI_STATUS_PRESENT;

          /* A high value indicates the the card is write protected. */

          if (pic32mx_gpioread(GPIO_SD_WP))
            {
              ret |= SPI_STATUS_WRPROTECTED;
            }
        }
    }

  spidbg("Returning %02x\n", ret);
  return ret;
}
#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi1cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI2
void  pic31mx_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic31mx_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  spidbg("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic31mx_spi2cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI3
void  pic32mx_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  spidbg("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi3cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#ifdef CONFIG_PIC32MX_SPI4
void  pic32mx_spi4select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi4status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  spidbg("Returning nothing\n");
#warning "Missing logic"
  return 0;
}

#ifdef CONFIG_SPI_CMDDATA
int pic32mx_spi4cmddata(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool cmd)
{
#warning "Missing logic"
  return 0;
}
#endif
#endif

#endif /* CONFIG_PIC32MX_SPI1..4 */
