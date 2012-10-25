/****************************************************************************
 * binfmt/libelf/libelf.h
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

#ifndef __BINFMT_LIBELF_LIBELF_H
#define __BINFMT_LIBELF_LIBELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <elf.h>

#include <nuttx/binfmt/elf.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Name: elf_verifyheader
 *
 * Description:
 *   Given the header from a possible ELF executable, verify that it is
 *   an ELF executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_verifyheader(FAR const Elf32_Ehdr *header);

/****************************************************************************
 * Name: elf_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_read(FAR struct elf_loadinfo_s *loadinfo, FAR uint8_t *buffer,
             size_t readsize, off_t offset);

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

int elf_findsymtab(FAR struct elf_loadinfo_s *loadinfo);

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
                FAR Elf32_Sym *sym);

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
 *   exports  - The symbol table to use for resolving undefined symbols.
 *   nexports - Number of symbols in the symbol table.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_symvalue(FAR struct elf_loadinfo_s *loadinfo, FAR Elf32_Sym *sym,
                 FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: elf_findctors
 *
 * Description:
 *   Find C++ static constructors.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ELF_CONSTRUCTORS
int elf_findctors(FAR struct elf_loadinfo_s *loadinfo);
#endif

/****************************************************************************
 * Name: elf_doctors
 *
 * Description:
 *   Execute C++ static constructors.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ELF_CONSTRUCTORS
int elf_doctors(FAR struct elf_loadinfo_s *loadinfo);
#endif

#endif /* __BINFMT_LIBELF_LIBELF_H */
