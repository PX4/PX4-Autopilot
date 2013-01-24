/****************************************************************************
 * examples/nsh/nsh_main.c
 *
 *   Copyright (C) 2007-2012 Gregory Nutt. All rights reserved.
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

#include <sys/stat.h>
#include <stdint.h>
#include <stdio.h>
#include <sched.h>
#include <errno.h>

#include <nuttx/arch.h>
#if defined(CONFIG_FS_BINFS) && (CONFIG_BUILTIN)
#include <nuttx/binfmt/builtin.h>
#endif
#if defined(CONFIG_LIBC_EXECFUNCS) && defined(CONFIG_EXECFUNCS_SYMTAB)
#include <nuttx/binfmt/symtab.h>
#endif

#include <apps/nsh.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The NSH telnet console requires networking support (and TCP/IP) */

#ifndef CONFIG_NET
#  undef CONFIG_NSH_TELNET
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* If posix_spawn() is enabled as required for CONFIG_NSH_FILE_APPS, then
 * a symbol table is needed by the internals of posix_spawn().  The symbol
 * table is needed to support ELF and NXFLAT binaries to dynamically link to
 * the base code.  However, if only the BINFS file system is supported, then
 * no Makefile is needed.
 *
 * This is a kludge to plug the missing file system in the case where BINFS
 * is used.  REVISIT:  This will, of course, be in the way if you want to
 * support ELF or NXFLAT binaries!
 */

#if defined(CONFIG_LIBC_EXECFUNCS) && defined(CONFIG_EXECFUNCS_SYMTAB)
const struct symtab_s CONFIG_EXECFUNCS_SYMTAB[1];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_main
 ****************************************************************************/

int nsh_main(int argc, char *argv[])
{
  int exitval = 0;
  int ret;

  /* Call all C++ static constructors */

#if defined(CONFIG_HAVE_CXX) && defined(CONFIG_HAVE_CXXINITIALIZE)
  up_cxxinitialize();
#endif

  /* Make sure that we are using our symbol take */

#if defined(CONFIG_LIBC_EXECFUNCS) && defined(CONFIG_EXECFUNCS_SYMTAB)
  exec_setsymtab(CONFIG_EXECFUNCS_SYMTAB, 0);
#endif

  /* Register the BINFS file system */

#if defined(CONFIG_FS_BINFS) && (CONFIG_BUILTIN)
  ret = builtin_initialize();
  if (ret < 0)
    {
     fprintf(stderr, "ERROR: builtin_initialize failed: %d\n", ret);
     exitval = 1;
   }
#endif

  /* Initialize the NSH library */

  nsh_initialize();

  /* If the Telnet console is selected as a front-end, then start the
   * Telnet daemon.
   */

#ifdef CONFIG_NSH_TELNET
  ret = nsh_telnetstart();
  if (ret < 0)
    {
     /* The daemon is NOT running.  Report the the error then fail...
      * either with the serial console up or just exiting.
      */

     fprintf(stderr, "ERROR: Failed to start TELNET daemon: %d\n", ret);
     exitval = 1;
   }
#endif

  /* If the serial console front end is selected, then run it on this thread */

#ifdef CONFIG_NSH_CONSOLE
  ret = nsh_consolemain(0, NULL);

  /* nsh_consolemain() should not return.  So if we get here, something
   * is wrong.
   */

  fprintf(stderr, "ERROR: nsh_consolemain() returned: %d\n", ret);
  exitval = 1;
#endif

  return exitval;
}
