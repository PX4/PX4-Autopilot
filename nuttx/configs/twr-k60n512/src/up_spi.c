/************************************************************************************
 * configs/twr-k60n512/src/up_spi.c
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

#include "up_arch.h"
#include "chip.h"
#include "kinetis_internal.h"
#include "twrk60-internal.h"

#if defined(CONFIG_KINETIS_SPI1) || defined(CONFIG_KINETIS_SPI2)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#ifdef CONFIG_DEBUG_SPI
#  define spidbg  lldbg
#  ifdef CONFIG_DEBUG_SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef CONFIG_DEBUG_SPI_VERBOSE
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
 * Name: kinetis_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the TWR-K60N512 board.
 *
 ************************************************************************************/

void weak_function kinetis_spiinitialize(void)
{
# warning "Missing logic"
}

/****************************************************************************
 * Name:  kinetis_spi1/2/3select and kinetis_spi1/2/3status
 *
 * Description:
 *   The external functions, kinetis_spi1/2/3select and kinetis_spi1/2/3status must be
 *   provided by board-specific logic.  They are implementations of the select
 *   and status methods of the SPI interface defined by struct spi_ops_s (see
 *   include/nuttx/spi.h). All other methods (including up_spiinitialize())
 *   are provided by common Kinetis logic.  To use this common SPI logic on your
 *   board:
 *
 *   1. Provide logic in kinetis_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide kinetis_spi1/2/3select() and kinetis_spi1/2/3status() functions in your
 *      board-specific logic.  These functions will perform chip selection and
 *      status operations using GPIOs in the way your board is configured.
 *   3. Add a calls to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

#ifdef CONFIG_KINETIS_SPI1
void kinetis_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_KINETIS_SPI2
void kinetis_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#ifdef CONFIG_KINETIS_SPI3
void kinetis_spi3select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
# warning "Missing logic"
}

uint8_t kinetis_spi3status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
# warning "Missing logic"
  return SPI_STATUS_PRESENT;
}
#endif

#endif /* CONFIG_KINETIS_SPI1 || CONFIG_KINETIS_SPI2 */
