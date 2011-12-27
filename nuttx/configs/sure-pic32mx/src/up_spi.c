/************************************************************************************
 * configs/sure-pic32mx/src/up_spi.c
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
#include "pic32mx-internal.h"
#include "sure-internal.h"

#if defined(CONFIG_PIC32MX_SPI2)

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* The following enable debug output from this file (needs CONFIG_DEBUG too).
 * 
 * CONFIG_SPI_DEBUG - Define to enable basic SPI debug
 * CONFIG_SPI_VERBOSE - Define to enable verbose SPI debug
 */

#ifdef CONFIG_SPI_DEBUG
#  define spidbg  lldbg
#  ifdef CONFIG_SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef CONFIG_SPI_VERBOSE
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
  /* Configure the SPI2 chip select GPIOs */

#ifdef CONFIG_PIC32MX_SPI2
#  warning "Missing logic"
#endif
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
void  pic32mx_spi2select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
  spidbg("devid: %d CS: %s\n", (int)devid, selected ? "assert" : "de-assert");
#warning "Missing logic"
}

uint8_t pic32mx_spi2status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  spidbg("Returning nothing\n");
#warning "Missing logic"
  return 0;
}
#endif
#endif /* CONFIG_PIC32MX_SPI2 */
