/****************************************************************************
 * include/nuttx/binfmt/nxflat.h
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

#ifndef __INCLUDE_NUTTX_BINFMT_NXFLAT_H
#define __INCLUDE_NUTTX_BINFMT_NXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nxflat.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a desciption of the currently loaded instantiation
 * of an nxflat binary.
 */

struct nxflat_loadinfo_s
{
  /* Instruction Space (ISpace):  This region contains the nxflat file header
   * plus everything from the text section.
   *
   * The ISpace region is allocated using mmap() and, thus, can be shared by
   * multiple tasks.  Ideally, will have only one mmap'ed text section
   * instance in the system for each module.
   */

  uintptr_t ispace;        /* Address where hdr/text is loaded */
  uint32_t entryoffs;      /* Offset from ispace to entry point */
  uint32_t isize;          /* Size of ispace. */

  /* Data Space (DSpace): This region contains all information that is
   * referenced as data (other than the stack which is separately allocated).
   *
   * If CONFIG_ADDRENV=n, DSpace will be allocated using kmalloc() (or
   * kzalloc()).  If CONFIG_ADDRENV-y, then DSpace will be allocated using
   * up_addrenv_create().  In either case, there will be a unique instance
   * of DSpace (and stack) for each instance of a process.
   */

  struct dspace_s *dspace; /* Allocated D-Space (data/bss/etc) */
  uint32_t datasize;       /* Size of data segment in dspace */
  uint32_t bsssize;        /* Size of bss segment in dspace */
  uint32_t stacksize;      /* Size of stack (not allocated) */
  uint32_t dsize;          /* Size of dspace (may be large than parts) */

  /* This is temporary memory where relocation records will be loaded. */

  uint32_t relocstart;     /* Start of array of struct flat_reloc */
  uint16_t reloccount;     /* Number of elements in reloc array */

  /* Address environment.
   *
   * addrenv - This is the handle created by up_addrenv_create() that can be
   *   used to manage the tasks address space.
   * oldenv  - This is a value returned by up_addrenv_select() that must be 
   *   used to restore the current hardware address environment.
   */

#ifdef CONFIG_ADDRENV
  task_addrenv_t addrenv;  /* Task address environment */
  hw_addrenv_t   oldenv;   /* Saved hardware address environment */
#endif

  /* File descriptors */

  int    filfd;            /* Descriptor for the file being loaded */

  /* This is a copy of the NXFLAT header (still in network order) */

  struct nxflat_hdr_s header;
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
 * These are APIs exported by libnxflat (and may be used outside of NuttX):
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_verifyheader
 *
 * Description:
 *   Given the header from a possible NXFLAT executable, verify that it is
 *   an NXFLAT executable.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int nxflat_verifyheader(const struct nxflat_hdr_s *header);

/****************************************************************************
 * Name: nxflat_init
 *
 * Description:
 *   This function is called to configure the library to process an NXFLAT
 *   program binary.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int nxflat_init(const char *filename,
                       struct nxflat_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: nxflat_uninit
 *
 * Description:
 *   Releases any resources committed by nxflat_init().  This essentially
 *   undoes the actions of nxflat_init.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int nxflat_uninit(struct nxflat_loadinfo_s *loadinfo);

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

EXTERN int nxflat_load(struct nxflat_loadinfo_s *loadinfo);

/****************************************************************************
 * Name: nxflat_read
 *
 * Description:
 *   Read 'readsize' bytes from the object file at 'offset'
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int nxflat_read(struct nxflat_loadinfo_s *loadinfo, char *buffer,
                       int readsize, int offset);

/****************************************************************************
 * Name: nxflat_bind
 *
 * Description:
 *   Bind the imported symbol names in the loaded module described by
 *   'loadinfo' using the exported symbol values provided by 'symtab'
 *   After binding the module, clear the BSS region (which held the relocation
 *   data) in preparation for execution.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

struct symtab_s;
EXTERN int nxflat_bind(FAR struct nxflat_loadinfo_s *loadinfo,
                       FAR const struct symtab_s *exports, int nexports);

/****************************************************************************
 * Name: nxflat_unload
 *
 * Description:
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of nxflat_load.  It is called only under certain error
 *   conditions after the the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int nxflat_unload(struct nxflat_loadinfo_s *loadinfo);

/****************************************************************************
 * These are APIs used internally only by NuttX:
 ****************************************************************************/
/****************************************************************************
 * Name: nxflat_initialize
 *
 * Description:
 *   NXFLAT support is built unconditionally.  However, it order to
 *   use this binary format, this function must be called during system
 *   format in order to register the NXFLAT binary format.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int nxflat_initialize(void);

/****************************************************************************
 * Name: nxflat_uninitialize
 *
 * Description:
 *   Unregister the NXFLAT binary loader
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void nxflat_uninitialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_NXFLAT_H */
