/****************************************************************************
 * binfmt/libelf/libelf_ctors.c
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

#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/binfmt/elf.h>

#include "libelf.h"

#ifdef CONFIG_BINFMT_CONSTRUCTORS

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_loadctors
 *
 * Description:
 *  Load points to static constructors into an in-memory array.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_loadctors(FAR struct elf_loadinfo_s *loadinfo)
{
  FAR Elf32_Shdr *shdr;
  size_t ctorsize;
  int ctoridx;
  int ret;
  int i;

  DEBUGASSERT(loadinfo->ctors == NULL);

  /* Allocate an I/O buffer.  This buffer is used by elf_sectname() to
   * accumulate the variable length symbol name.
   */

  ret = elf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      bdbg("elf_allocbuffer failed: %d\n", ret);
      return -ENOMEM;
    }

  /* Find the index to the section named ".ctors."  NOTE:  On old ABI system,
   * .ctors is the name of the section containing the list of constructors;
   * On newer systems, the similar section is called .init_array.  It is 
   * expected that the linker script will force the sectino name to be ".ctors"
   * in either case.
   */

  ctoridx = elf_findsection(loadinfo, ".ctors");
  if (ctoridx < 0)
    {
      /* This may not be a failure.  -ENOENT indicates that the file has no
       * static constructor section.
       */

      bvdbg("elf_findsection .ctors section failed: %d\n", ctoridx);
      return ret == -ENOENT ? OK : ret;
    }

  /* Now we can get a pointer to the .ctor section in the section header
   * table.
   */

  shdr = &loadinfo->shdr[ctoridx];

  /* Get the size of the .ctor section and the number of constructors that
   * will need to be called.
   */

  ctorsize         = shdr->sh_size;
  loadinfo->nctors = ctorsize / sizeof(elf_ctor_t);

  bvdbg("ctoridx=%d ctorsize=%d sizeof(elf_ctor_t)=%d nctors=%d\n",
        ctoridx, ctorsize,  sizeof(elf_ctor_t), loadinfo->nctors);

  /* Check if there are any constructors.  It is not an error if there
   * are none.
   */

  if (loadinfo->nctors > 0)
    {
      /* Check an assumption that we made above */

      DEBUGASSERT(shdr->sh_size == loadinfo->nctors * sizeof(elf_ctor_t));

      /* In the old ABI, the .ctors section is not allocated.  In that case,
       * we need to allocate memory to hold the .ctors and then copy the
       * from the file into the allocated memory.
       *
       * SHF_ALLOC indicates that the section requires memory during
       * execution.
       */

      if ((shdr->sh_flags & SHF_ALLOC) == 0)
        {
          /* Not loaded -> Old ABI. */

          loadinfo->newabi = false;

          /* Allocate memory to hold a copy of the .ctor section */

          loadinfo->ctors  = (elf_ctor_t*)kmalloc(ctorsize);
          if (!loadinfo->ctors)
            {
              bdbg("Failed to allocate memory for .ctors\n");
              return -ENOMEM;
            }

          /* Read the section header table into memory */

          ret = elf_read(loadinfo, (FAR uint8_t*)loadinfo->ctors, ctorsize,
                         shdr->sh_offset);
          if (ret < 0)
            {
              bdbg("Failed to allocate .ctors: %d\n", ret);
              return ret;
            }

          /* Fix up all of the .ctor addresses.  Since the addresses
           * do not lie in allocated memory, there will be no relocation
           * section for them.
           */

          for (i = 0; i < loadinfo->nctors; i++)
            {
              FAR uintptr_t *ptr = (uintptr_t *)((FAR void *)(&loadinfo->ctors)[i]);

              bvdbg("ctor %d: %08lx + %08lx = %08lx\n",
                    i, *ptr, loadinfo->alloc, *ptr + loadinfo->alloc);

              *ptr += loadinfo->alloc;
            }
        }
      else
        {
          /* Loaded -> New ABI. */

          loadinfo->newabi = true;

          /* Save the address of the .ctors (actually, .init_array) where it was
           * loaded into memory.  Since the .ctors lie in allocated memory, they
           * will be relocated via the normal mechanism.
           */
 
          loadinfo->ctors = (elf_ctor_t*)shdr->sh_addr;
        }
    }

  return OK;
}

#endif /* CONFIG_BINFMT_CONSTRUCTORS */
