/****************************************************************************
 * config/shenzhou/src/up_mmcsd.c
 * arch/arm/src/board/up_mmcsd.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi.h>
#include <nuttx/mmcsd.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* SPI1 connects to the SD CARD (and to the SPI FLASH) */

#define HAVE_MMCSD           1 /* Assume that we have SD support */
#define STM32_MMCSDSPIPORTNO 1 /* Port is SPI1 */
#define STM32_MMCSDSLOTNO    0 /* There is only one slot */

#ifndef CONFIG_STM32_SPI1
#  undef HAVE_MMCSD
#else
#  ifdef CONFIG_SPI_OWNBUS
#    warning "SPI1 is shared with SD and FLASH but CONFIG_SPI_OWNBUS is defined"
#  endif
#endif

/* Can't support MMC/SD features if MMC/SD driver support is not selected */

#ifndef CONFIG_MMCSD
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SPI1=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor)
{
#ifdef HAVE_MMCSD
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  fvdbg("Initializing SPI port %d\n", STM32_MMCSDSPIPORTNO);

  spi = up_spiinitialize(STM32_MMCSDSPIPORTNO);
  if (!spi)
    {
      fdbg("Failed to initialize SPI port %d\n", STM32_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  fvdbg("Successfully initialized SPI port %d\n", STM32_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  fvdbg("Binding SPI port %d to MMC/SD slot %d\n",
          STM32_MMCSDSPIPORTNO, STM32_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(minor, STM32_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      fdbg("Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
            STM32_MMCSDSPIPORTNO, STM32_MMCSDSLOTNO, ret);
      return ret;
    }

  fvdbg("Successfuly bound SPI port %d to MMC/SD slot %d\n",
        STM32_MMCSDSPIPORTNO, STM32_MMCSDSLOTNO);
#endif
  return OK;
}

