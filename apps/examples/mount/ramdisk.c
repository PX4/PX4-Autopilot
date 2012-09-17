/****************************************************************************
 * examples/mount/ramdisk.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/ramdisk.h>
#include <nuttx/fs/mkfatfs.h>

#include "mount.h"

#ifndef CONFIG_EXAMPLES_MOUNT_DEVNAME

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#define BUFFER_SIZE (CONFIG_EXAMPLES_MOUNT_NSECTORS*CONFIG_EXAMPLES_MOUNT_SECTORSIZE)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct fat_format_s g_fmt = FAT_FORMAT_INITIALIZER;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: create_ramdisk
 *
 * Description:
 *   Create a RAM disk of the specified size formatting with a FAT file
 *   system
 *
 * Input Parameters:
 *   None
 *
 * Return:
 *   Zero on success, a negated errno on failure.
 *
 ****************************************************************************/

int create_ramdisk(void)
{
  char *pbuffer;
  int ret;

  /* Allocate a buffer to hold the file system image. */

  pbuffer = (char*)malloc(BUFFER_SIZE);
  if (!pbuffer)
    {
      printf("create_ramdisk: Failed to allocate ramdisk of size %d\n",
             BUFFER_SIZE);
      return -ENOMEM;
    }

  /* Register a RAMDISK device to manage this RAM image */

  ret = ramdisk_register(CONFIG_EXAMPLES_MOUNT_RAMDEVNO,
                    pbuffer,
                    CONFIG_EXAMPLES_MOUNT_NSECTORS,
                    CONFIG_EXAMPLES_MOUNT_SECTORSIZE,
                    true);
  if (ret < 0)
    {
      printf("create_ramdisk: Failed to register ramdisk at %s: %d\n",
             g_source, -ret);
      free(pbuffer);
      return ret;
    }

  /* Create a FAT filesystem on the ramdisk */

  ret = mkfatfs(g_source, &g_fmt);
  if (ret < 0)
    {
      printf("create_ramdisk: Failed to create FAT filesystem on ramdisk at %s\n",
             g_source);
      /* free(pbuffer); -- RAM disk is registered */
      return ret;
    }

  return 0;
}
#endif /* !CONFIG_EXAMPLES_MOUNT_DEVNAME */
