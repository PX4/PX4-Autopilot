/****************************************************************************
 * config/lpc4330-xplorer/src/up_nsh.c
 * arch/arm/src/board/up_nsh.c
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

#include "chip.h"

#ifdef CONFIG_LPC43_SPIFI
#  include <nuttx/mtd.h>
#  include "lpc43_spifi.h"

#  ifdef CONFIG_SPFI_NXFFS
#    include <sys/mount.h>
#    include <nuttx/fs/nxffs.h>
#  endif
#endif

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_SPIFI_DEVNO
#  define CONFIG_SPIFI_DEVNO 0
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_spifi_initialize
 *
 * Description:
 *   Make the SPIFI (or part of it) into a block driver that can hold a
 *   file system.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SPIFI
static int nsh_spifi_initialize(void)
{
  FAR struct mtd_dev_s *mtd;
  int ret;

  /* Initialize the SPIFI interface and create the MTD driver instance */

  mtd = lpc43_spifi_initialize();
  if (!mtd)
    {
      fdbg("ERROR: lpc43_spifi_initialize failed\n");
      return -ENODEV;
    }

#ifndef CONFIG_SPFI_NXFFS
  /* And finally, use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_SPIFI_DEVNO, mtd);
  if (ret < 0)
    {
      fdbg("ERROR: Initializing the FTL layer: %d\n", ret);
      return ret;
    }
#else
  /* Initialize to provide NXFFS on the MTD interface */1G

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      fdbg("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  /* Mount the file system at /mnt/spifi */

  ret = mount(NULL, "/mnt/spifi", "nxffs", 0, NULL);
  if (ret < 0)
    {
      fdbg("ERROR: Failed to mount the NXFFS volume: %d\n", errno);
      return ret;
    }
#endif

  return OK;
}
#else
#  define nsh_spifi_initialize() (OK)
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
  /* Initialize the SPIFI block device */

  return nsh_spifi_initialize();
}
