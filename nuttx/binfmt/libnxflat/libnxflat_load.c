/****************************************************************************
 * binfmt/libnxflat/libnxflat_load.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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
#include <sys/mman.h>
#include <stdint.h>
#include <stdlib.h>
#include <nxflat.h>
#include <debug.h>
#include <errno.h>

#include <arpa/inet.h>
#include <nuttx/nxflat.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

#if defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_BINFMT)
static const char g_relocrel32i[]  = "RELOC_REL32I";
static const char g_relocrel32d[]  = "RELOC_REL32D";
static const char g_relocabs32[]   = "RELOC_AB32";
static const char g_undefined[]    = "UNDEFINED";

static const char *g_reloctype[] =
{
  g_relocrel32i,
  g_relocrel32d,
  g_relocabs32,
  g_undefined
};
#  define RELONAME(rl) g_reloctype[NXFLAT_RELOC_TYPE(rl)]
#else
#  define RELONAME(rl) "(no name)"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_load
 *
 * Description:
 *   Loads the binary specified by nxflat_init into memory, mapping
 *   the I-space executable regions, allocating the D-Space region,
 *   and inializing the data segment (relocation information is
 *   temporarily loaded into the BSS region.  BSS will be cleared
 *   by nxflat_bind() after the relocation data has been processed).
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int nxflat_load(struct nxflat_loadinfo_s *loadinfo)
{
  off_t    doffset;     /* Offset to .data in the NXFLAT file */
  uint32_t dreadsize;   /* Total number of bytes of .data to be read */
  uint32_t relocsize;   /* Memory needed to hold relocations */
  uint32_t extrasize;   /* MAX(BSS size, relocsize) */
  int      ret = OK;

  /* Calculate the extra space we need to allocate.  This extra space will be
   * the size of the BSS section.  This extra space will also be used
   * temporarily to hold relocation information.  So the allocated size of this
   * region will either be the size of .data + size of.bss section OR, the
   * size of .data + the relocation entries, whichever is larger
   *
   * This is the amount of memory that we have to have to hold the
   * relocations.
   */

  relocsize  = loadinfo->reloccount * sizeof(struct nxflat_reloc_s);

  /* In the file, the relocations should lie at the same offset as BSS.
   * The additional amount that we allocate have to be either (1) the
   * BSS size, or (2) the size of the relocation records, whicher is
   * larger.
   */

  extrasize = MAX(loadinfo->bsssize, relocsize);

  /* Use this additional amount to adjust the total size of the dspace
   * region.
   */

  loadinfo->dsize = loadinfo->datasize + extrasize;

  /* The number of bytes of data that we have to read from the file is
   * the data size plus the size of the relocation table.
   */

  dreadsize = loadinfo->datasize + relocsize;

  /* We'll need this a few times. */

  doffset = loadinfo->isize;

  /* We will make two mmap calls create an address space for the executable.
   * We will attempt to map the file to get the ISpace address space and
   * to allocate RAM to get the DSpace address space.  If the filesystem does
   * not support file mapping, the map() implementation should do the
   * right thing.
   */

  /* The following call will give as a pointer to the mapped file ISpace.
   * This may be in ROM, RAM, Flash, ... We don't really care where the memory
   * resides as long as it is fully initialized and ready to execute.
   */

  loadinfo->ispace = (uint32_t)mmap(NULL, loadinfo->isize, PROT_READ,
                                  MAP_SHARED|MAP_FILE, loadinfo->filfd, 0);
  if (loadinfo->ispace == (uint32_t)MAP_FAILED)
    {
      bdbg("Failed to map NXFLAT ISpace: %d\n", errno);
      return -errno;
    }

  bvdbg("Mapped ISpace (%d bytes) at %08x\n", loadinfo->isize, loadinfo->ispace);

  /* The following call will give a pointer to the allocated but
   * uninitialized ISpace memory.
   */

  loadinfo->dspace = (struct dspace_s *)malloc(SIZEOF_DSPACE_S(loadinfo->dsize));
  if (loadinfo->dspace == 0)
    {
      bdbg("Failed to allocate DSpace\n");
      ret = -ENOMEM;
      goto errout;
    }
  loadinfo->dspace->crefs = 1;

  bvdbg("Allocated DSpace (%d bytes) at %p\n", loadinfo->dsize, loadinfo->dspace->region);

  /* Now, read the data into allocated DSpace at doffset into the
   * allocated DSpace memory.
   */

  ret = nxflat_read(loadinfo, (char*)loadinfo->dspace->region, dreadsize, doffset);
  if (ret < 0)
    {
      bdbg("Failed to read .data section: %d\n", ret);
      goto errout;
    }
       
  bvdbg("TEXT: %08x Entry point offset: %08x Data offset: %08x\n",
      loadinfo->ispace, loadinfo->entryoffs, doffset);

  return OK;

errout:
  (void)nxflat_unload(loadinfo);
  return ret;
}

