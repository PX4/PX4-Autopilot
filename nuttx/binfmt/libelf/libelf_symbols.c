/****************************************************************************
 * binfmt/libelf/libelf_symbols.c
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

#include <stdlib.h>
#include <string.h>
#include <elf32.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/symtab.h>

#include "libelf.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_ELF_BUFFERINCR
#  define CONFIG_ELF_BUFFERINCR 32
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_symname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static int elf_symname(FAR struct elf_loadinfo_s *loadinfo,
                       FAR const Elf32_Sym *sym)
{
  FAR uint8_t *buffer;
  off_t  offset;
  size_t readlen;
  size_t bytesread;
  int ret;

  /* Get the file offset to the string that is the name of the symbol.  The
   * st_name member holds an offset into the file's symbol string table.
   */

  if (sym->st_name == 0)
    {
      bdbg("Symbol has no name\n");
      return -ENOENT;
    }

  offset = loadinfo->shdr[loadinfo->strtabidx].sh_offset + sym->st_name;

  /* Loop until we get the entire symbol name into memory */

  bytesread = 0;

  for (;;)
    {
      /* Get the number of bytes to read */

      readlen = loadinfo->buflen - bytesread;
      if (offset + readlen > loadinfo->filelen)
        {
          readlen = loadinfo->filelen - offset;
          if (readlen <= 0)
            {
              bdbg("At end of file\n");
              return -EINVAL;
            }
        }

      /* Read that number of bytes into the array */

      buffer = &loadinfo->iobuffer[bytesread];
      ret = elf_read(loadinfo, buffer, readlen, offset);
      if (ret < 0)
        {
          bdbg("elf_read failed: %d\n", ret);
          return ret;
        }

      bytesread += readlen;

      /* Did we read the NUL terminator? */

      if (memchr(buffer, '\0', readlen) != NULL)
        {
          /* Yes, the buffer contains a NUL terminator. */

          return OK;
        }

      /* No.. then we have to read more */

      ret = elf_reallocbuffer(loadinfo, CONFIG_ELF_BUFFERINCR);
      if (ret < 0)
        {
          bdbg("elf_reallocbuffer failed: %d\n", ret);
          return ret;
        }
    }

  /* We will not get here */

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_findsymtab
 *
 * Description:
 *   Find the symbol table section.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_findsymtab(FAR struct elf_loadinfo_s *loadinfo)
{
  int i;

  /* Find the symbol table section header and its associated string table */

  for (i = 1; i < loadinfo->ehdr.e_shnum; i++)
    {
      if (loadinfo->shdr[i].sh_type == SHT_SYMTAB)
        {
          loadinfo->symtabidx = i;
          loadinfo->strtabidx = loadinfo->shdr[i].sh_link;
          break;
        }
    }

  /* Verify that there is a symbol and string table */

  if (loadinfo->symtabidx == 0)
    {
      bdbg("No symbols in ELF file\n");
      return -EINVAL;
    }

  return OK;
}

/****************************************************************************
 * Name: elf_readsym
 *
 * Description:
 *   Read the ELFT symbol structure at the specfied index into memory.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   index    - Symbol table index
 *   sym      - Location to return the table entry
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_readsym(FAR struct elf_loadinfo_s *loadinfo, int index,
                FAR Elf32_Sym *sym)
{
  FAR Elf32_Shdr *symtab = &loadinfo->shdr[loadinfo->symtabidx];
  off_t offset;

  /* Verify that the symbol table index lies within symbol table */

  if (index < 0 || index > (symtab->sh_size / sizeof(Elf32_Sym)))
    {
      bdbg("Bad relocation symbol index: %d\n", index);
      return -EINVAL;
    }

  /* Get the file offset to the symbol table entry */

  offset = symtab->sh_offset + sizeof(Elf32_Sym) * index;

  /* And, finally, read the symbol table entry into memory */

  return elf_read(loadinfo, (FAR uint8_t*)sym, sizeof(Elf32_Sym), offset);
}

/****************************************************************************
 * Name: elf_symvalue
 *
 * Description:
 *   Get the value of a symbol.  The updated value of the symbol is returned
 *   in the st_value field of the symbol table entry.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sym      - Symbol table entry (value might be undefined)
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_symvalue(FAR struct elf_loadinfo_s *loadinfo, FAR Elf32_Sym *sym,
                 FAR const struct symtab_s *exports, int nexports)
{
  FAR const struct symtab_s *symbol;
  uintptr_t secbase;
  int ret;

  switch (sym->st_shndx)
    {
    case SHN_COMMON:
      {
        /* NuttX ELF modules should be compiled with -fno-common. */

        bdbg("SHN_COMMON: Re-compile with -fno-common\n");
        return -EINVAL;
      }

    case SHN_ABS:
      {
        /* st_value already holds the correct value */

        bvdbg("SHN_ABS: st_value=%08lx\n", (long)sym->st_value);
        return OK;
      }

    case SHN_UNDEF:
      {
        /* Get the name of the undefined symbol */

        ret = elf_symname(loadinfo, sym);
        if (ret < 0)
          {
            bdbg("SHN_UNDEF: Failed to get symbol name: %d\n", ret);
            return ret;
          }

        /* Check if the base code exports a symbol of this name */

#ifdef CONFIG_SYMTAB_ORDEREDBYNAME
        symbol = symtab_findorderedbyname(exports, (FAR char *)loadinfo->iobuffer, nexports);
#else
        symbol = symtab_findbyname(exports, (FAR char *)loadinfo->iobuffer, nexports);
#endif
        if (!symbol)
          {
            bdbg("SHN_UNDEF: Exported symbol \"%s\" not found\n", loadinfo->iobuffer);
            return -ENOENT;
          }

        /* Yes... add the exported symbol value to the ELF symbol table entry */

        bvdbg("SHN_ABS: name=%s %08x+%08x=%08x\n",
              loadinfo->iobuffer, sym->st_value, symbol->sym_value,
              sym->st_value + symbol->sym_value);

        sym->st_value += (Elf32_Word)((uintptr_t)symbol->sym_value);
      }
      break;

    default:
      {
        secbase = loadinfo->shdr[sym->st_shndx].sh_addr;

        bvdbg("Other: %08x+%08x=%08x\n",
              sym->st_value, secbase, sym->st_value + secbase);

        sym->st_value += secbase;
      }
      break;
    }

  return OK;
}
