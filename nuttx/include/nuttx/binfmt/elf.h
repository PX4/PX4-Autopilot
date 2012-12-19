/****************************************************************************
 * include/nuttx/binfmt/elf.h
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

#ifndef __INCLUDE_NUTTX_BINFMT_ELF_H
#define __INCLUDE_NUTTX_BINFMT_ELF_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <elf32.h>

#include <nuttx/binfmt/binfmt.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#ifndef CONFIG_ELF_ALIGN_LOG2
#  define CONFIG_ELF_ALIGN_LOG2 2
#endif

#ifndef CONFIG_ELF_STACKSIZE
#  define CONFIG_ELF_STACKSIZE 2048
#endif

#ifndef CONFIG_ELF_BUFFERSIZE
#  define CONFIG_ELF_BUFFERSIZE 128
#endif

#ifndef CONFIG_ELF_BUFFERINCR
#  define CONFIG_ELF_BUFFERINCR 32
#endif

/* Allocation array size and indices */

#define LIBELF_ELF_ALLOC     0
#ifdef CONFIG_BINFMT_CONSTRUCTORS
#  define LIBELF_CTORS_ALLOC 1
#  define LIBELF_CTPRS_ALLOC 2
#  define LIBELF_NALLOC      3
#else
#  define LIBELF_NALLOC      1
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct provides a desciption of the currently loaded instantiation
 * of an ELF binary.
 */

struct elf_loadinfo_s
{
  /* elfalloc is the base address of the memory that is allocated to hold the
   * ELF program image.
   *
   * If CONFIG_ADDRENV=n, elfalloc will be allocated using kmalloc() (or
   * kzalloc()).  If CONFIG_ADDRENV-y, then elfalloc will be allocated using
   * up_addrenv_create().  In either case, there will be a unique instance
   * of elfalloc (and stack) for each instance of a process.
   *
   * The alloc[] array in struct binary_s will hold memory that persists after
   * the ELF module has been loaded.
   */
 
  uintptr_t         elfalloc;    /* Memory allocated when ELF file was loaded */
  size_t            elfsize;     /* Size of the ELF memory allocation */
  off_t             filelen;     /* Length of the entire ELF file */
  Elf32_Ehdr        ehdr;        /* Buffered ELF file header */
  FAR Elf32_Shdr    *shdr;       /* Buffered ELF section headers */
  uint8_t           *iobuffer;   /* File I/O buffer */

  /* Constructors and destructors */
  
#ifdef CONFIG_BINFMT_CONSTRUCTORS
  FAR void          *ctoralloc;  /* Memory allocated for ctors */
  FAR void          *dtoralloc;  /* Memory allocated dtors */
  FAR binfmt_ctor_t *ctors;      /* Pointer to a list of constructors */
  FAR binfmt_dtor_t *dtors;      /* Pointer to a list of destructors */
  uint16_t           nctors;     /* Number of constructors */
  uint16_t           ndtors;     /* Number of destructors */
#endif

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

  uint16_t           symtabidx;  /* Symbol table section index */
  uint16_t           strtabidx;  /* String table section index */
  uint16_t           buflen;     /* size of iobuffer[] */
  int                filfd;      /* Descriptor for the file being loaded */
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
 * These are APIs exported by libelf (but are used only by the binfmt logic):
 ****************************************************************************/

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
 *   This function unloads the object from memory. This essentially undoes
 *   the actions of elf_load.  It is called only under certain error
 *   conditions after the the module has been loaded but not yet started.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

EXTERN int elf_unload(struct elf_loadinfo_s *loadinfo);

/****************************************************************************
 * These are APIs used outside of binfmt by NuttX:
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

/****************************************************************************
 * These are APIs must be provided by architecture-specific logic:
 ****************************************************************************/
/****************************************************************************
 * Name: arch_checkarch
 *
 * Description:
 *   Given the ELF header in 'hdr', verify that the ELF file is appropriate
 *   for the current, configured architecture.  Every architecture that uses
 *   the ELF loader must provide this function.
 *
 * Input Parameters:
 *   hdr - The ELF header read from the ELF file.
 *
 * Returned Value:
 *   True if the architecture supports this ELF file.
 *
 ****************************************************************************/

EXTERN bool arch_checkarch(FAR const Elf32_Ehdr *hdr);

/****************************************************************************
 * Name: arch_relocate and arch_relocateadd
 *
 * Description:
 *   Perform on architecture-specific ELF relocation.  Every architecture
 *   that uses the ELF loader must provide this function.
 *
 * Input Parameters:
 *   rel - The relocation type
 *   sym - The ELF symbol structure containing the fully resolved value.
 *   addr - The address that requires the relocation.
 *
 * Returned Value:
 *   Zero (OK) if the relocation was successful.  Otherwise, a negated errno
 *   value indicating the cause of the relocation failure.
 *
 ****************************************************************************/

EXTERN int arch_relocate(FAR const Elf32_Rel *rel, FAR const Elf32_Sym *sym,
                         uintptr_t addr);
EXTERN int arch_relocateadd(FAR const Elf32_Rela *rel,
                            FAR const Elf32_Sym *sym, uintptr_t addr);

/****************************************************************************
 * Name: arch_flushicache
 *
 * Description:
 *   Flush the instruction cache.
 *
 * Input Parameters:
 *   addr - Start address to flush
 *   len  - Number of bytes to flush
 *
 * Returned Value:
 *   True if the architecture supports this ELF file.
 *
 ****************************************************************************/

#ifdef CONFIG_ELF_ICACHE
EXTERN bool arch_flushicache(FAR void *addr, size_t len);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_BINFMT_ELF_H */
