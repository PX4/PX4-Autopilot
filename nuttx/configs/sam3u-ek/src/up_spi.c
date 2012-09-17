/************************************************************************************
 * configs/sam3u-ek/src/up_spi.c
 * arch/arm/src/board/up_spi.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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
#include <errno.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include "up_arch.h"
#include "chip.h"
#include "sam3u_internal.h"
#include "sam3uek_internal.h"

#ifdef CONFIG_SAM3U_SPI

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Enables debug output from this file (needs CONFIG_DEBUG too) */

#undef SPI_DEBUG   /* Define to enable debug */
#undef SPI_VERBOSE /* Define to enable verbose debug */

#ifdef SPI_DEBUG
#  define spidbg  lldbg
#  ifdef SPI_VERBOSE
#    define spivdbg lldbg
#  else
#    define spivdbg(x...)
#  endif
#else
#  undef SPI_VERBOSE
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
 * Name: sam3u_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the SAM3U10E-EVAL board.
 *
 ************************************************************************************/

void weak_function sam3u_spiinitialize(void)
{
  /* The ZigBee module connects used NPCS0.  However, there is not yet any
   * ZigBee support.
   */

   /* The touchscreen connects using NPCS2 (PC14). */

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_ADS7843E)
   sam3u_configgpio(GPIO_TSC_NPCS2);
#endif
}

/****************************************************************************
 * Name:  sam3u_spicsnumber, sam3u_spiselect, sam3u_spistatus, and
 *        sam3u_spicmddata
 *
 * Description:
 *   These external functions must be provided by board-specific logic.  They
 *   include:
 *
 *   o sam3u_spicsnumbe and sam3u_spiselect which are helper functions to
 *     manage the board-specific aspects of the unique SAM3U chip select
 *     architecture.
 *   o sam3u_spistatus and sam3u_spicmddata:  Implementations of the status
 *     and cmddata methods of the SPI interface defined by struct spi_ops_
 *     (see include/nuttx/spi.h). All other methods including
 *     up_spiinitialize()) are provided by common SAM3U logic.
 *
 *  To use this common SPI logic on your board:
 *
 *   1. Provide logic in sam3u_boardinitialize() to configure SPI chip select
 *      pins.
 *   2. Provide sam3u_spicsnumber(), sam3u_spiselect() and sam3u_spistatus()
 *      functions in your board-specific logic.  These functions will perform
 *      chip selection and status operations using GPIOs in the way your board
 *      is configured.
 *   2. If CONFIG_SPI_CMDDATA is defined in the NuttX configuration, provide
 *      sam3u_spicmddata() functions in your board-specific logic.  This
 *      function will perform cmd/data selection operations using GPIOs in
 *      the way your board is configured.
 *   3. Add a call to up_spiinitialize() in your low level application
 *      initialization logic
 *   4. The handle returned by up_spiinitialize() may then be used to bind the
 *      SPI driver to higher level logic (e.g., calling 
 *      mmcsd_spislotinitialize(), for example, will bind the SPI driver to
 *      the SPI MMC/SD driver).
 *
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_spicsnumber
 *
 * Description:
 *   The SAM3U has 4 CS registers for controlling device features.  This
 *   function must be provided by board-specific code.  Given a logical device
 *   ID, this function returns a number from 0 to 3 that identifies one of
 *   these SAM3U CS resources.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   On success, a CS number from 0 to 3 is returned; A negated errno may
 *   be returned on a failure.
 *
 ****************************************************************************/

int sam3u_spicsnumber(enum spi_dev_e devid)
{
  int cs = -EINVAL;

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_ADS7843E)
  if (devid == SPIDEV_TOUCHSCREEN)
    {
      /* Assert the CS pin to the OLED display */

      cs = 2;
    }
#endif

  spidbg("devid: %d CS: %d\n", (int)devid, cs);
  return cs;
}

/****************************************************************************
 * Name: sam3u_spiselect
 *
 * Description:
 *   PIO chip select pins may be programmed by the board specific logic in
 *   one of two different ways.  First, the pins may be programmed as SPI
 *   peripherals.  In that case, the pins are completely controlled by the
 *   SPI driver.  This method still needs to be provided, but it may be only
 *   a stub.
 *
 *   An alternative way to program the PIO chip select pins is as a normal
 *   GPIO output.  In that case, the automatic control of the CS pins is
 *   bypassed and this function must provide control of the chip select.
 *   NOTE:  In this case, the GPIO output pin does *not* have to be the
 *   same as the NPCS pin normal associated with the chip select number.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *   selected - TRUE:Select the device, FALSE:De-select the device
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

void sam3u_spiselect(enum spi_dev_e devid, bool selected)
{
  /* The touchscreen chip select is implemented as a GPIO OUTPUT that must
   * be controlled by this function.  This is because the ADS7843E driver
   * must be able to sample the device BUSY GPIO input between SPI transfers.
   * However, the AD7843E will tri-state the BUSY input whenever the chip
   * select is de-asserted.  So the only option is to control the chip select
   * manually and hold it low throughout the SPI transfer.
   */

#if defined(CONFIG_INPUT) && defined(CONFIG_INPUT_ADS7843E)
  if (devid == SPIDEV_TOUCHSCREEN)
    {
      sam3u_gpiowrite(GPIO_TSC_NPCS2, !selected);
    }
#endif
}

/****************************************************************************
 * Name: sam3u_spistatus
 *
 * Description:
 *   Return status information associated with the SPI device.
 *
 * Input Parameters:
 *   devid - Identifies the (logical) device
 *
 * Returned Values:
 *   Bit-encoded SPI status (see include/nuttx/spi.h.
 *
 ****************************************************************************/

uint8_t sam3u_spistatus(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
  return 0;
}

#endif /* CONFIG_SAM3U_SPI */
