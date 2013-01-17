/****************************************************************************
 * examples/nxflat/nxflat_main.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/mount.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/ramdisk.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/nxflat.h>

#include "tests/romfs.h"
#include "tests/dirlist.h"
#include "tests/symtab.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Check configuration.  This is not all of the configuration settings that
 * are required -- only the more obvious.
 */

#if CONFIG_NFILE_DESCRIPTORS < 1
#  error "You must provide file descriptors via CONFIG_NFILE_DESCRIPTORS in your configuration file"
#endif

#ifdef CONFIG_BINFMT_DISABLE
#  error "The binary loader is disabled (CONFIG_BINFMT_DISABLE)!"
#endif

#ifndef CONFIG_NXFLAT
#  error "You must select CONFIG_NXFLAT in your configuration file"
#endif

#ifndef CONFIG_FS_ROMFS
#  error "You must select CONFIG_FS_ROMFS in your configuration file"
#endif

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "You must not disable mountpoints via CONFIG_DISABLE_MOUNTPOINT in your configuration file"
#endif

#ifdef CONFIG_BINFMT_DISABLE
#  error "You must not disable loadable modules via CONFIG_BINFMT_DISABLE in your configuration file"
#endif

/* Describe the ROMFS file system */

#define SECTORSIZE   512
#define NSECTORS(b)  (((b)+SECTORSIZE-1)/SECTORSIZE)
#define ROMFSDEV     "/dev/ram0"
#define MOUNTPT      "/mnt/romfs"

/* If CONFIG_DEBUG is enabled, use dbg instead of printf so that the
 * output will be synchronous with the debug output.
 */

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(format, arg...) dbg(format, ##arg)
#    define err(format, arg...)     dbg(format, ##arg)
#  else
#    define message(format, arg...) printf(format, ##arg)
#    define err(format, arg...)     fprintf(stderr, format, ##arg)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message                 dbg
#    define err                     dbg
#  else
#    define message                 printf
#    define err                     printf
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char delimiter[] =
  "****************************************************************************";

#ifndef CONFIG_BINFMT_EXEPATH
static char fullpath[128];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: testheader
 ****************************************************************************/

static inline void testheader(FAR const char *progname)
{
  message("\n%s\n* Executing %s\n%s\n\n", delimiter, progname, delimiter);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_main
 ****************************************************************************/

int nxflat_main(int argc, char *argv[])
{
  struct binary_s bin;
  int ret;
  int i;

  /* Initialize the NXFLAT binary loader */

  message("Initializing the NXFLAT binary loader\n");
  ret = nxflat_initialize();
  if (ret < 0)
    {
      err("ERROR: Initialization of the NXFLAT loader failed: %d\n", ret);
      exit(1);
    }

  /* Create a ROM disk for the ROMFS filesystem */

  message("Registering romdisk\n");
  ret = romdisk_register(0, (FAR uint8_t *)romfs_img, NSECTORS(romfs_img_len), SECTORSIZE);
  if (ret < 0)
    {
      err("ERROR: romdisk_register failed: %d\n", ret);
      nxflat_uninitialize();
      exit(1);
    }

  /* Mount the file system */

  message("Mounting ROMFS filesystem at target=%s with source=%s\n",
         MOUNTPT, ROMFSDEV);

  ret = mount(ROMFSDEV, MOUNTPT, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      err("ERROR: mount(%s,%s,romfs) failed: %s\n",
              ROMFSDEV, MOUNTPT, errno);
      nxflat_uninitialize();
    }

  /* Does the system support the PATH variable?  Has the PATH variable
   * already been set?  If YES and NO, then set the PATH variable to
   * the ROMFS mountpoint.
   */

#if defined(CONFIG_BINFMT_EXEPATH) && !defined(CONFIG_PATH_INITIAL)
  (void)setenv("PATH", MOUNTPT, 1);
#endif

  /* Now excercise every progrm in the ROMFS file system */

  for (i = 0; dirlist[i]; i++)
    {
      /* Output a seperated so that we can clearly discrinmate the output of
       * this program from the others.
       */

      testheader(dirlist[i]);

      /* Initialize the binary_s structure */

      memset(&bin, 0, sizeof(struct binary_s));

      /* If the binary loader does not support the PATH variable, then
       * create the full path to the executable program.  Otherwise,
       * use the relative path so that the binary loader will have to
       * search the PATH variable to find the executable.
       */

#ifdef CONFIG_BINFMT_EXEPATH
      bin.filename = dirlist[i];
#else
      snprintf(fullpath, 128, "%s/%s", MOUNTPT, dirlist[i]);
      bin.filename = fullpath;
#endif
      bin.exports  = exports;
      bin.nexports = NEXPORTS;

      /* Load the NXFLAT module */

      ret = load_module(&bin);
      if (ret < 0)
        {
          err("ERROR: Failed to load program '%s'\n", dirlist[i]);
          exit(1);
        }

      /* Execute the ELF module */

      ret = exec_module(&bin);
      if (ret < 0)
        {
          err("ERROR: Failed to execute program '%s'\n", dirlist[i]);
          unload_module(&bin);
        }

      message("Wait a bit for test completion\n");
      sleep(4);
    }

  message("End-of-Test.. Exit-ing\n");
  return 0;
}
