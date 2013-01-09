/************************************************************************************
 * configs/nucleus2g/src/up_ssp.c
 * arch/arm/src/board/up_ssp.c
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
#include "lpc17_internal.h"
#include "nucleus2g_internal.h"

#if defined(CONFIG_LPC17_SSP0) || defined(CONFIG_LPC17_SSP1)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SSP_DEBUG   /* Define to enable debug */
#undef SSP_VERBOSE /* Define to enable verbose debug */

#ifdef SSP_DEBUG
#  define sspdbg  lldbg
#  ifdef SSP_VERBOSE
#    define sspvdbg lldbg
#  else
#    define sspvdbg(x...)
#  endif
#else
#  undef SSP_VERBOSE
#  define sspdbg(x...)
#  define sspvdbg(x...)
#endif

/* Dump GPIO registers */

#ifdef SSP_VERBOSE
#  define ssp_dumpgpio(m) lpc17_dumpgpio(SDCCS_GPIO, m)
#else
#  define ssp_dumpgpio(m)
#endif

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: lpc17_sspinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Nucleus 2G.
 *
 ************************************************************************************/

void weak_function lpc17_sspinitialize(void)
{
  /* Configure the SPI-based microSD CS GPIO */

  ssp_dumpgpio("lpc17_sspinitialize() Entry)");

  /* SSP0 connects only to the MMC/SD slot on the Nucleus 1G board.
   * P0[15]/TXD1/SCK0/SCK              MMC_CLK
   * P0[16]/RXD1/SSEL0/SSEL            MMC_CD
   * P0[17]/CTS1/MISO0/MISO            MMC_DATA0
   * P0[18]/DCD1/MOSI0/MOSI            MMC_MISO
   *
   * In SPI mode the MMC/SD DATA3/CD functions as the SPI chip select.
   */

#ifdef CONFIG_LPC17_SSP0
  lpc17_configgpio(NUCLEUS2G_MMCSD_CS);
#endif
  
  /* SSP1 goes off the Nucleus 2G board to the Babel CAN board along with 3 chip
   * select pins.  However, it is currently not used on that board.
   */

#ifdef CONFIG_LPC17_SSP1
# warning "SSP1 chip selects not known"
#endif
  ssp_dumpgpio("lpc17_sspinitialize() Exit");
}

/************************************************************************************
 * Name:  lpc17_ssp0/ssp1select and lpc17_ssp0/ssp1status
 *
 * Description:
 *   The external functions, lpc17_ssp0/ssp1select and lpc17_ssp0/ssp1status 
 *   must be provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common LPC17xx logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in lpc17_boardinitialize() to configure SPI/SSP chip select
 *      pins.
 *   2. Provide lpc17_ssp0/ssp1select() and lpc17_ssp0/ssp1status() functions
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

#ifdef CONFIG_LPC17_SSP0
void  lpc17_ssp0select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  sspdbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
  ssp_dumpgpio("lpc17_spiselect() Entry");

  if (devid == SPIDEV_MMCSD)
    {
      /* Assert the CS pin to the card */

      lpc17_gpiowrite(NUCLEUS2G_MMCSD_CS, !selected);
    }
  ssp_dumpgpio("lpc17_spiselect() Exit");
}

uint8_t lpc17_ssp0status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  sspdbg("Returning SPI_STATUS_PRESENT\n");
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_LPC17_SSP1
# warning "SSP1 chip selects not known"
#endif

#endif /* CONFIG_LPC17_SSP0 || CONFIG_LPC17_SSP1 */
