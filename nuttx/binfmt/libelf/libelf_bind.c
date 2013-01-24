/****************************************************************************
 * binfmt/libelf/libelf_bind.c
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

#include <stdint.h>
#include <string.h>
#include <elf32.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/symtab.h>

#include "libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_BINFMT have to be
 * defined or CONFIG_ELF_DUMPBUFFER does nothing.
 */

#if !defined(CONFIG_DEBUG_VERBOSE) || !defined (CONFIG_DEBUG_BINFMT)
#  undef CONFIG_ELF_DUMPBUFFER
#endif

#ifndef CONFIG_ELF_BUFFERSIZE
#  define CONFIG_ELF_BUFFERSIZE 128
#endif

#ifdef CONFIG_ELF_DUMPBUFFER
# define elf_dumpbuffer(m,b,n) bvdbgdumpbuffer(m,b,n)
#else
# define elf_dumpbuffer(m,b,n)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_readrel
 *
 * Description:
 *   Read the ELF32_Rel structure into memory.
 *
 ****************************************************************************/

static inline int elf_readrel(FAR struct elf_loadinfo_s *loadinfo,
                              FAR const Elf32_Shdr *relsec,
                              int index, FAR Elf32_Rel *rel)
{
  off_t offset;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (relsec->sh_size / sizeof(Elf32_Rel)))
    {
      bdbg("Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = relsec->sh_offset + sizeof(Elf32_Rel) * index;

  /* And, finally, read the symbol table entry into memory */

  return elf_read(loadinfo, (FAR uint8_t*)rel, sizeof(Elf32_Rel), offset);
}

/****************************************************************************
 * Name: elf_relocate and elf_relocateadd
 *
 * Description:
 *   Perform all relocations associated with a section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int elf_relocate(FAR struct elf_loadinfo_s *loadinfo, int relidx,
                        FAR const struct symtab_s *exports, int nexports)

{
  FAR Elf32_Shdr *relsec = &loadinfo->shdr[relidx];
  FAR Elf32_Shdr *dstsec = &loadinfo->shdr[relsec->sh_info];
  Elf32_Rel       rel;
  Elf32_Sym       sym;
  uintptr_t       addr;
  int             symidx;
  int             ret;
  int             i;

  /* Examine each relocation in the section.  'relsec' is the section
   * containing the relations.  'dstsec' is the section containing the data
   * to be relocated.
   */

  for (i = 0; i < relsec->sh_size / sizeof(Elf32_Rel); i++)
    {
      /* Read the relocation entry into memory */

      ret = elf_readrel(loadinfo, relsec, i, &rel);
      if (ret < 0)
        {
          bdbg("Section %d reloc %d: Failed to read relocation entry: %d\n",
               relidx, i, ret);
          return ret;
        }

      /* Get the symbol table index for the relocation.  This is contained
       * in a bit-field within the r_info element.
       */

      symidx = ELF32_R_SYM(rel.r_info);

      /* Read the symbol table entry into memory */

      ret = elf_readsym(loadinfo, symidx, &sym);
      if (ret < 0)
        {
          bdbg("Section %d reloc %d: Failed to read symbol[%d]: %d\n",
               relidx, i, symidx, ret);
          return ret;
        }

      /* Get the value of the symbol (in sym.st_value) */

      ret = elf_symvalue(loadinfo, &sym, exports, nexports);
      if (ret < 0)
        {
          bdbg("Section %d reloc %d: Failed to get value of symbol[%d]: %d\n",
               relidx, i, symidx, ret);
          return ret;
        }

      /* Calculate the relocation address. */

      if (rel.r_offset < 0 || rel.r_offset > dstsec->sh_size - sizeof(uint32_t))
        {
          bdbg("Section %d reloc %d: Relocation address out of range, offset %d size %d\n",
               relidx, i, rel.r_offset, dstsec->sh_size);
          return -EINVAL;
        }

      addr = dstsec->sh_addr + rel.r_offset;

      /* If CONFIG_ADDRENV=y, then 'addr' lies in a virtual address space that
       * may not be in place now.  elf_addrenv_select() will temporarily
       * instantiate that address space.
       */

#ifdef CONFIG_ADDRENV
      ret = elf_addrenv_select(loadinfo);
      if (ret < 0)
        {
          bdbg("ERROR: elf_addrenv_select() failed: %d\n", ret);
          return ret;
        }
#endif

      /* Now perform the architecture-specific relocation */

      ret = arch_relocate(&rel, &sym, addr);
      if (ret < 0)
        {
#ifdef CONFIG_ADDRENV
          (void)elf_addrenv_restore(loadinfo);
#endif
          bdbg("ERROR: Section %d reloc %d: Relocation failed: %d\n", ret);
          return ret;
        }

      /* Restore the original address environment */

#ifdef CONFIG_ADDRENV
      ret = elf_addrenv_restore(loadinfo);
      if (ret < 0)
        {
          bdbg("ERROR: elf_addrenv_restore() failed: %d\n", ret);
          return ret;
        }
#endif
    }

  return OK;
}

static int elf_relocateadd(FAR struct elf_loadinfo_s *loadinfo, int relidx,
                           FAR const struct symtab_s *exports, int nexports)
{
  bdbg("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_bind(FAR struct elf_loadinfo_s *loadinfo,
             FAR const struct symtab_s *exports, int nexports)
{
  int ret;
  int i;

  /* Find the symbol and string tables */

  ret = elf_findsymtab(loadinfo);
  if (ret < 0)
    {
      return ret;
    }

  /* Allocate an I/O buffer.  This buffer is used by elf_symname() to
   * accumulate the variable length symbol name.
   */

  ret = elf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      bdbg("elf_allocbuffer failed: %d\n", ret);
      return -ENOMEM;
    }

  /* Process relocations in every allocated section */

  for (i = 1; i < loadinfo->ehdr.e_shnum; i++)
    {
      /* Get the index to the relocation section */
      
      int infosec = loadinfo->shdr[i].sh_info;
      if (infosec >= loadinfo->ehdr.e_shnum)
        {
          continue;
        }

      /* Make sure that the section is allocated.  We can't relocated
       * sections that were not loaded into memory.
       */

      if ((loadinfo->shdr[infosec].sh_flags & SHF_ALLOC) == 0)
        {
          continue;
        }

      /* Process the relocations by type */

      if (loadinfo->shdr[i].sh_type == SHT_REL)
        {
          ret = elf_relocate(loadinfo, i, exports, nexports);
        }
      else if (loadinfo->shdr[i].sh_type == SHT_RELA)
        {
          ret = elf_relocateadd(loadinfo, i, exports, nexports);
        }

      if (ret < 0)
        {
          break;
        }
    }

  /* Flush the instruction cache before starting the newly loaded module */

#ifdef CONFIG_ELF_ICACHE
  arch_flushicache((FAR void*)loadinfo->elfalloc, loadinfo->elfsize);
#endif

  return ret;
}

