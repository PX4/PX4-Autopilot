/****************************************************************************
 * configs/vsn/src/usbmsc.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Copyright (c) 2011 Uros Platise. All rights reserved.
 *
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Uros Platise <uros.platise@isotel.eu>
 *
 * Configure and register the STM32 MMC/SD SDIO block driver.
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

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "vsn.h"

#ifdef CONFIG_STM32_SDIO

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_EXAMPLES_USBMSC_DEVMINOR1
#  define CONFIG_EXAMPLES_USBMSC_DEVMINOR1 0
#endif

/* SLOT number(s) could depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_VSN
#  undef STM32_MMCSDSLOTNO
#  define STM32_MMCSDSLOTNO 0
#else
   /* Add configuration for new STM32 boards here */
#  error "Unrecognized STM32 board"
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int usbmsc_archinitialize(void)
{
  FAR struct sdio_dev_s *sdio;
  int ret;

  /* First, get an instance of the SDIO interface */

  message("usbmsc_archinitialize: "
          "Initializing SDIO slot %d\n",
          STM32_MMCSDSLOTNO);

  sdio = sdio_initialize(STM32_MMCSDSLOTNO);
  if (!sdio)
    {
      message("usbmsc_archinitialize: Failed to initialize SDIO slot %d\n",
              STM32_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MMC/SD driver */

  message("usbmsc_archinitialize: "
          "Bind SDIO to the MMC/SD driver, minor=%d\n",
          CONFIG_EXAMPLES_USBMSC_DEVMINOR1);

  ret = mmcsd_slotinitialize(CONFIG_EXAMPLES_USBMSC_DEVMINOR1, sdio);
  if (ret != OK)
    {
      message("usbmsc_archinitialize: "
              "Failed to bind SDIO to the MMC/SD driver: %d\n",
              ret);
      return ret;
    }
  message("usbmsc_archinitialize: "
          "Successfully bound SDIO to the MMC/SD driver\n");
  
  /* Then let's guess and say that there is a card in the slot.  I need to check to
   * see if the VSN board supports a GPIO to detect if there is a card in
   * the slot.
   */

   sdio_mediachange(sdio, true);
   return OK;
}

#endif /* CONFIG_STM32_SDIO */
