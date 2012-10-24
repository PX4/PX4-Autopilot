/****************************************************************************
 * binfmt/elf.c
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <elf.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/binfmt/binfmt.h>
#include <nuttx/binfmt/elf.h>

#ifdef CONFIG_ELF

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_BINFMT have to be
 * defined or CONFIG_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_ELF_DUMPBUFFER
#endif

#ifdef CONFIG_ELF_DUMPBUFFER
# define elf_dumpbuffer(m,b,n) bvdbgdumpbuffer(m,b,n)
#else
# define elf_dumpbuffer(m,b,n)
#endif

#ifndef MIN
#  define MIN(a,b) (a < b ? a : b)
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int elf_loadbinary(FAR struct binary_s *binp);
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void elf_dumploadinfo(FAR struct elf_loadinfo_s *loadinfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct binfmt_s g_elfbinfmt =
{
  NULL,             /* next */
  elf_loadbinary,   /* load */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_dumploadinfo
 ****************************************************************************/

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT)
static void elf_dumploadinfo(FAR struct elf_loadinfo_s *loadinfo)
{
  unsigned long dsize = loadinfo->datasize + loadinfo->bsssize;

  bdbg("LOAD_INFO:\n");
  bdbg("  ISPACE:\n");
  bdbg("    ispace:       %08lx\n", loadinfo->ispace);
  bdbg("    entryoffs:    %08lx\n", loadinfo->entryoffs);
  bdbg("    isize:        %08lx\n", loadinfo->isize);

  bdbg("  DSPACE:\n");
  bdbg("    dspace:       %08lx\n", loadinfo->dspace);
  if (loadinfo->dspace != NULL)
    {
      bdbg("      crefs:      %d\n",    loadinfo->dspace->crefs);
      bdbg("      region:     %08lx\n", loadinfo->dspace->region);
    }
  bdbg("    datasize:     %08lx\n", loadinfo->datasize);
  bdbg("    bsssize:      %08lx\n", loadinfo->bsssize);
  bdbg("      (pad):      %08lx\n", loadinfo->dsize - dsize);
  bdbg("    stacksize:    %08lx\n", loadinfo->stacksize);
  bdbg("    dsize:        %08lx\n", loadinfo->dsize);

  bdbg("  RELOCS:\n");
  bdbg("    relocstart:   %08lx\n", loadinfo->relocstart);
  bdbg("    reloccount:   %d\n",    loadinfo->reloccount);

  bdbg("  HANDLES:\n");
  bdbg("    filfd:        %d\n",    loadinfo->filfd);
}
#else
# define elf_dumploadinfo(i)
#endif

/****************************************************************************
 * Name: elf_loadbinary
 *
 * Description:
 *   Verify that the file is an ELF binary and, if so, load the ELF
 *   binary into memory
 *
 ****************************************************************************/

static int elf_loadbinary(struct binary_s *binp)
{
  struct elf_loadinfo_s loadinfo;  /* Contains globals for libelf */
  int                      ret;

  bvdbg("Loading file: %s\n", binp->filename);

  /* Initialize the xflat library to load the program binary. */

  ret = elf_init(binp->filename, &loadinfo);
  elf_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      bdbg("Failed to initialize for load of NXFLT program: %d\n", ret);
      goto errout;
    }

  /* Load the program binary */

  ret = elf_load(&loadinfo);
  elf_dumploadinfo(&loadinfo);
  if (ret != 0)
    {
      bdbg("Failed to load NXFLT program binary: %d\n", ret);
      goto errout_with_init;
    }

  /* Bind the program to the exported symbol table */

  ret = elf_bind(&loadinfo, binp->exports, binp->nexports);
  if (ret != 0)
    {
      bdbg("Failed to bind symbols program binary: %d\n", ret);
      goto errout_with_load;
    }

  /* Return the load information */

  binp->entrypt   = (main_t)(loadinfo.ispace + loadinfo.entryoffs);
  binp->ispace    = (void*)loadinfo.ispace;
  binp->dspace    = (void*)loadinfo.dspace;
  binp->isize     = loadinfo.isize;
  binp->stacksize = loadinfo.stacksize;

  elf_dumpbuffer("Entry code", (FAR const uint8_t*)binp->entrypt,
                    MIN(binp->isize - loadinfo.entryoffs,512));

  elf_uninit(&loadinfo);
  return OK;

errout_with_load:
  elf_unload(&loadinfo);
errout_with_init:
  elf_uninit(&loadinfo);
errout:
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_initialize
 *
 * Description:
 *   ELF support is built unconditionally.  However, it order to
 *   use this binary format, this function must be called during system
 *   format in order to register the ELF binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_initialize(void)
{
  int ret;

  /* Register ourselves as a binfmt loader */

  bvdbg("Registering ELF\n");

  ret = register_binfmt(&g_elfbinfmt);
  if (ret != 0)
    {
      bdbg("Failed to register binfmt: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: elf_uninitialize
 *
 * Description:
 *   Unregister the ELF binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void elf_uninitialize(void)
{
  unregister_binfmt(&g_elfbinfmt);
}

#endif /* CONFIG_ELF */

