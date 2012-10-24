/****************************************************************************
 * include/nuttx/elf.h
 *
 *   Copyright (C) 2009, 2012 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_ELF_H
#define __INCLUDE_NUTTX_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <elf.h>
#include <nuttx/sched.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a desciption of the currently loaded instantiation
 * of an ELF binary.
 */

struct elf_loadinfo_s
{
  /* Instruction Space (ISpace):  This region contains the ELF file header
   * plus everything from the text section.  Ideally, will have only one mmap'ed
   * text section instance in the system for each module.
   */

  uint32_t ispace;             /* Address where hdr/text is loaded */
  uint32_t entryoffs;          /* Offset from ispace to entry point */
  uint32_t isize;              /* Size of ispace. */

  /* Data Space (DSpace): This region contains all information that in referenced
   * as data (other than the stack which is separately allocated).  There will be
   * a unique instance of DSpace (and stack) for each instance of a process.
   */

  FAR struct dspace_s *dspace; /* Allocated D-Space (data/bss/etc) */
  uint32_t datasize;           /* Size of data segment in dspace */
  uint32_t bsssize;            /* Size of bss segment in dspace */
  uint32_t stacksize;          /* Size of stack (not allocated) */
  uint32_t dsize;              /* Size of dspace (may be large than parts) */

  /* This is temporary memory where relocation records will be loaded. */

  uint32_t relocstart;         /* Start of array of struct flat_reloc */
  uint16_t reloccount;         /* Number of elements in reloc array */

  /* File descriptors */

  int    filfd;                /* Descriptor for the file being loaded */

  /* This is a copy of the ELF header (still in network order) */

  FAR struct elf_hdr_s header;
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * These are APIs exported by libelf (and may be used outside of NuttX):
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

EXTERN int elf_verifyheader(FAR const struct elf_hdr_s *header);

/****************************************************************************
 * Name: elf_init
 *
 * Description:
 *   This function is called to configure the library to process an ELF
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int elf_init(FAR const char *filename,
                    FAR struct elf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: elf_uninit
 *
 * Description:
 *   Releases any resources committed by elf_init().  This essentially
 *   undoes the actions of elf_init.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int elf_uninit(FAR struct elf_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: elf_load
 *
 * Description:
 *   Loads the binary into memory, allocating memory, performing relocations
 *   and inializing the data and bss segments.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int elf_load(FAR struct elf_loadinfo_s *loadinfo);

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

EXTERN int elf_read(FAR struct elf_loadinfo_s *loadinfo, FAR char *buffer,
                    FAR int readsize, int offset);

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

struct symtab_s;
EXTERN int elf_bind(FAR struct elf_loadinfo_s *loadinfo,
                    FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: elf_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially
 *   undoes the actions of elf_load.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int elf_unload(struct elf_loadinfo_s *loadinfo);

/****************************************************************************
 * These are APIs used internally only by NuttX:
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

EXTERN int elf_initialize(void);

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

EXTERN void elf_uninitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_ELF_H */
