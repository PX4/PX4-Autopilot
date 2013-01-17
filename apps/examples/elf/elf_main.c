/****************************************************************************
 * examples/elf/elf_main.c
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
#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/symtab.h>

#include "tests/romfs.h"
#include "tests/dirlist.h"

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

#ifndef CONFIG_ELF
#  error "You must select CONFIG_ELF in your configuration file"
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
#define MOUNTPT      "/mnt/romfs"

#ifndef CONFIG_EXAMPLES_ELF_DEVMINOR
#  define CONFIG_EXAMPLES_ELF_DEVMINOR 0
#endif

#ifndef CONFIG_EXAMPLES_ELF_DEVPATH
#  define CONFIG_EXAMPLES_ELF_DEVPATH "/dev/ram0"
#endif

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

static unsigned int g_mminitial;  /* Initial memory usage */
static unsigned int g_mmstep;     /* Memory Usage at beginning of test step */

static const char delimiter[] =
  "****************************************************************************";

#ifndef CONFIG_BINFMT_EXEPATH
static char fullpath[128];
#endif

/****************************************************************************
 * Symbols from Auto-Generated Code
 ****************************************************************************/

extern const struct symtab_s exports[];
extern const int nexports;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_update
 ****************************************************************************/

static void mm_update(FAR unsigned int *previous, FAR const char *msg)
{
  struct mallinfo mmcurrent;

  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  mmcurrent = mallinfo();
#else
  (void)mallinfo(&mmcurrent);
#endif

  /* Show the change from the previous time */

  printf("\nMemory Usage %s:\n", msg);
  printf("  Before: %8u After: %8u Change: %8d\n",
         *previous, mmcurrent.uordblks, (int)mmcurrent.uordblks - (int)*previous);

  /* Set up for the next test */

  *previous =  mmcurrent.uordblks;
}

/****************************************************************************
 * Name: mm_initmonitor
 ****************************************************************************/

static void mm_initmonitor(void)
{
  struct mallinfo mmcurrent;

#ifdef CONFIG_CAN_PASS_STRUCTS
  mmcurrent = mallinfo();
#else
  (void)mallinfo(&mmcurrent);
#endif

  g_mminitial = mmcurrent.uordblks;
  g_mmstep    = mmcurrent.uordblks;

  printf("Initial memory usage: %d\n", mmcurrent.uordblks);
}

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
 * Name: elf_main
 ****************************************************************************/

int elf_main(int argc, char *argv[])
{
  struct binary_s bin;
  int ret;
  int i;

  /* Initialize the memory monitor */

  mm_initmonitor();

  /* Initialize the ELF binary loader */

  message("Initializing the ELF binary loader\n");
  ret = elf_initialize();
  if (ret < 0)
    {
      err("ERROR: Initialization of the ELF loader failed: %d\n", ret);
      exit(1);
    }

  mm_update(&g_mmstep, "after elf_initialize");

  /* Create a ROM disk for the ROMFS filesystem */

  message("Registering romdisk at /dev/ram%d\n", CONFIG_EXAMPLES_ELF_DEVMINOR);
  ret = romdisk_register(CONFIG_EXAMPLES_ELF_DEVMINOR, (FAR uint8_t *)romfs_img,
                         NSECTORS(romfs_img_len), SECTORSIZE);
  if (ret < 0)
    {
      err("ERROR: romdisk_register failed: %d\n", ret);
      elf_uninitialize();
      exit(1);
    }

  mm_update(&g_mmstep, "after romdisk_register");

  /* Mount the file system */

  message("Mounting ROMFS filesystem at target=%s with source=%s\n",
         MOUNTPT, CONFIG_EXAMPLES_ELF_DEVPATH);

  ret = mount(CONFIG_EXAMPLES_ELF_DEVPATH, MOUNTPT, "romfs", MS_RDONLY, NULL);
  if (ret < 0)
    {
      err("ERROR: mount(%s,%s,romfs) failed: %s\n",
              CONFIG_EXAMPLES_ELF_DEVPATH, MOUNTPT, errno);
      elf_uninitialize();
    }

  mm_update(&g_mmstep, "after mount");

  /* Does the system support the PATH variable?  Has the PATH variable
   * already been set?  If YES and NO, then set the PATH variable to
   * the ROMFS mountpoint.
   */

#if defined(CONFIG_BINFMT_EXEPATH) && !defined(CONFIG_PATH_INITIAL)
  (void)setenv("PATH", MOUNTPT, 1);
#endif

  /* Now excercise every program in the ROMFS file system */

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
      bin.nexports = nexports;

      /* Load the ELF module */

      ret = load_module(&bin);
      if (ret < 0)
        {
          err("ERROR: Failed to load program '%s'\n", dirlist[i]);
          exit(1);
        }

      mm_update(&g_mmstep, "after load_module");

      /* Execute the ELF module */

      ret = exec_module(&bin);

      mm_update(&g_mmstep, "after exec_module");

      if (ret < 0)
        {
          err("ERROR: Failed to execute program '%s'\n", dirlist[i]);
        }
      else
        {
          message("Wait a bit for test completion\n");
          sleep(4);
        }

      unload_module(&bin);
      mm_update(&g_mmstep, "after unload_module");
    }

  mm_update(&g_mmstep, "End-of-Test");
  return 0;
}
