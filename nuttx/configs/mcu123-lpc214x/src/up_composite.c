/****************************************************************************
 * configs/mcu123-lpc214x/src/up_composite.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Configure and register the LPC214x MMC/SD SPI block driver.
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
#include <nuttx/usb/composite.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_EXAMPLES_COMPOSITE_DEVMINOR1
#  define CONFIG_EXAMPLES_COMPOSITE_DEVMINOR1 0
#endif

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_MCU123
#  undef LPC214X_MMCSDSPIPORTNO
#  define LPC214X_MMCSDSPIPORTNO 1
#  undef LPC214X_MMCSDSLOTNO
#  define LPC214X_MMCSDSLOTNO 0
#else
   /* Add configuration for new LPC214x boards here */
#  error "Unrecognized LPC214x board"
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
 * Name: composite_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int composite_archinitialize(void)
{
  /* If examples/composite is built as an NSH command, then SD slot should
   * already have been initized in nsh_archinitialize() (see up_nsh.c).  In
   * this case, there is nothing further to be done here.
   *
   * NOTE: CONFIG_NSH_BUILTIN_APPS is not a fool-proof indication that NSH
   * was built.
   */

#ifndef CONFIG_NSH_BUILTIN_APPS
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  message("composite_archinitialize: Initializing SPI port %d\n",
          LPC214X_MMCSDSPIPORTNO);

  spi = up_spiinitialize(LPC214X_MMCSDSPIPORTNO);
  if (!spi)
    {
      message("composite_archinitialize: Failed to initialize SPI port %d\n",
              LPC214X_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  message("composite_archinitialize: Successfully initialized SPI port %d\n",
          LPC214X_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  message("composite_archinitialize: Binding SPI port %d to MMC/SD slot %d\n",
          LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_EXAMPLES_COMPOSITE_DEVMINOR1, LPC214X_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      message("composite_archinitialize: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
              LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO, ret);
      return ret;
    }

  message("composite_archinitialize: Successfuly bound SPI port %d to MMC/SD slot %d\n",
          LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO);

#endif /* CONFIG_NSH_BUILTIN_APPS */

  return OK;
}
