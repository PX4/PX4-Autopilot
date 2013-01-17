/****************************************************************************
 * examples/posix_spawn/spawn_main.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
#include <fcntl.h>
#include <spawn.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/ramdisk.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/symtab.h>

#include "filesystem/romfs.h"

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
static const char g_redirect[] = "redirect";
static const char g_hello[]    = "hello";
static const char g_data[]     = "testdata.txt";

static char fullpath[128];

static char * const g_argv[4] =
  { "Argument 1", "Argument 2", "Argument 3", NULL };

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
 * Name: spawn_main
 ****************************************************************************/

int spawn_main(int argc, char *argv[])
{
  posix_spawn_file_actions_t file_actions;
  posix_spawnattr_t attr;
  FAR const char *filepath;
  pid_t pid;
  int ret;

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

  /* Make sure that we are using our symbol take */

  exec_setsymtab(exports, nexports);

  /*************************************************************************
   * Case 1: Simple program with arguments
   *************************************************************************/

  /* Output a seperated so that we can clearly discriminate the output of
   * this program from the others.
   */

  testheader(g_hello);

  /* Initialize the attributes file actions structure */

  ret = posix_spawn_file_actions_init(&file_actions);
  if (ret != 0)
    {
      err("ERROR: posix_spawn_file_actions_init failed: %d\n", ret);
    }
  posix_spawn_file_actions_dump(&file_actions);

  ret = posix_spawnattr_init(&attr);
  if (ret != 0)
    {
      err("ERROR: posix_spawnattr_init failed: %d\n", ret);
    }
  posix_spawnattr_dump(&attr);

  mm_update(&g_mmstep, "after file_action/attr init");

  /* If the binary loader does not support the PATH variable, then
   * create the full path to the executable program.  Otherwise,
   * use the relative path so that the binary loader will have to
   * search the PATH variable to find the executable.
   */

#ifdef CONFIG_BINFMT_EXEPATH
  filepath = g_hello;
#else
  snprintf(fullpath, 128, "%s/%s", MOUNTPT, g_hello);
  filepath = fullpath;
#endif

  /* Execute the program */

  mm_update(&g_mmstep, "before posix_spawn");

  ret = posix_spawn(&pid, filepath, &file_actions, &attr, NULL, (FAR char * const*)&g_argv);
  if (ret != 0)
    {
      err("ERROR: posix_spawn failed: %d\n", ret);
    }

  sleep(4);
  mm_update(&g_mmstep, "after posix_spawn");

  /* Free attibutes and file actions */

  ret = posix_spawn_file_actions_destroy(&file_actions);
  if (ret != 0)
    {
      err("ERROR: posix_spawn_file_actions_destroy failed: %d\n", ret);
    }
  posix_spawn_file_actions_dump(&file_actions);

  ret = posix_spawnattr_destroy(&attr);
  if (ret != 0)
    {
      err("ERROR: posix_spawnattr_destroy failed: %d\n", ret);
    }
  posix_spawnattr_dump(&attr);

  mm_update(&g_mmstep, "after file_action/attr destruction");

  /*************************************************************************
   * Case 2: Simple program with redirection of stdin to a file input
   *************************************************************************/

  /* Output a seperated so that we can clearly discriminate the output of
   * this program from the others.
   */

  testheader(g_redirect);

  /* Initialize the attributes file actions structure */

  ret = posix_spawn_file_actions_init(&file_actions);
  if (ret != 0)
    {
      err("ERROR: posix_spawn_file_actions_init failed: %d\n", ret);
    }
  posix_spawn_file_actions_dump(&file_actions);

  ret = posix_spawnattr_init(&attr);
  if (ret != 0)
    {
      err("ERROR: posix_spawnattr_init failed: %d\n", ret);
    }
  posix_spawnattr_dump(&attr);

  mm_update(&g_mmstep, "after file_action/attr init");

  /* Set up to close stdin (0) and open testdata.txt as the program input */

  ret = posix_spawn_file_actions_addclose(&file_actions, 0);
  if (ret != 0)
    {
      err("ERROR: posix_spawn_file_actions_addclose failed: %d\n", ret);
    }
  posix_spawn_file_actions_dump(&file_actions);

  snprintf(fullpath, 128, "%s/%s", MOUNTPT, g_data);
  ret = posix_spawn_file_actions_addopen(&file_actions, 0, fullpath, O_RDONLY, 0644);
  if (ret != 0)
    {
      err("ERROR: posix_spawn_file_actions_addopen failed: %d\n", ret);
    }
  posix_spawn_file_actions_dump(&file_actions);
  
  mm_update(&g_mmstep, "after adding file_actions");

  /* If the binary loader does not support the PATH variable, then
   * create the full path to the executable program.  Otherwise,
   * use the relative path so that the binary loader will have to
   * search the PATH variable to find the executable.
   */

#ifdef CONFIG_BINFMT_EXEPATH
  filepath = g_redirect;
#else
  snprintf(fullpath, 128, "%s/%s", MOUNTPT, g_redirect);
  filepath = fullpath;
#endif

  /* Execute the program */

  mm_update(&g_mmstep, "before posix_spawn");

  ret = posix_spawn(&pid, filepath, &file_actions, &attr, NULL, NULL);
  if (ret != 0)
    {
      err("ERROR: posix_spawn failed: %d\n", ret);
    }

  sleep(2);
  mm_update(&g_mmstep, "after posix_spawn");

  /* Free attibutes and file actions */

  ret = posix_spawn_file_actions_destroy(&file_actions);
  if (ret != 0)
    {
      err("ERROR: posix_spawn_file_actions_destroy failed: %d\n", ret);
    }
  posix_spawn_file_actions_dump(&file_actions);

  ret = posix_spawnattr_destroy(&attr);
  if (ret != 0)
    {
      err("ERROR: posix_spawnattr_destroy failed: %d\n", ret);
    }
  posix_spawnattr_dump(&attr);

  mm_update(&g_mmstep, "after file_action/attr destruction");

  /* Clean-up */

  elf_uninitialize();

  mm_update(&g_mmstep, "End-of-Test");
  return 0;
}
