/****************************************************************************
 * binfmt/libelf/libelf_dtors.c
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
 * Name: elf_loaddtors
 *
 * Description:
 *  Load pointers to static destructors into an in-memory array.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_loaddtors(FAR struct elf_loadinfo_s *loadinfo)
{
  FAR Elf32_Shdr *shdr;
  size_t dtorsize;
  int dtoridx;
  int ret;
  int i;

  DEBUGASSERT(loadinfo->dtors == NULL);

  /* Allocate an I/O buffer if necessary.  This buffer is used by
   * elf_sectname() to accumulate the variable length symbol name.
   */

  ret = elf_allocbuffer(loadinfo);
  if (ret < 0)
    {
      bdbg("elf_allocbuffer failed: %d\n", ret);
      return -ENOMEM;
    }

  /* Find the index to the section named ".dtors."  NOTE:  On old ABI system,
   * .dtors is the name of the section containing the list of destructors;
   * On newer systems, the similar section is called .fini_array.  It is 
   * expected that the linker script will force the section name to be ".dtors"
   * in either case.
   */

  dtoridx = elf_findsection(loadinfo, ".dtors");
  if (dtoridx < 0)
    {
      /* This may not be a failure.  -ENOENT indicates that the file has no
       * static destructor section.
       */

      bvdbg("elf_findsection .dtors section failed: %d\n", dtoridx);
      return ret == -ENOENT ? OK : ret;
    }

  /* Now we can get a pointer to the .dtor section in the section header
   * table.
   */

  shdr = &loadinfo->shdr[dtoridx];

  /* Get the size of the .dtor section and the number of destructors that
   * will need to be called.
   */

  dtorsize         = shdr->sh_size;
  loadinfo->ndtors = dtorsize / sizeof(binfmt_dtor_t);

  bvdbg("dtoridx=%d dtorsize=%d sizeof(binfmt_dtor_t)=%d ndtors=%d\n",
        dtoridx, dtorsize,  sizeof(binfmt_dtor_t), loadinfo->ndtors);

  /* Check if there are any destructors.  It is not an error if there
   * are none.
   */

  if (loadinfo->ndtors > 0)
    {
      /* Check an assumption that we made above */

      DEBUGASSERT(shdr->sh_size == loadinfo->ndtors * sizeof(binfmt_dtor_t));

      /* In the old ABI, the .dtors section is not allocated.  In that case,
       * we need to allocate memory to hold the .dtors and then copy the
       * from the file into the allocated memory.
       *
       * SHF_ALLOC indicates that the section requires memory during
       * execution.
       */

      if ((shdr->sh_flags & SHF_ALLOC) == 0)
        {
          /* Allocate memory to hold a copy of the .dtor section */

          loadinfo->ctoralloc = (binfmt_dtor_t*)kmalloc(dtorsize);
          if (!loadinfo->ctoralloc)
            {
              bdbg("Failed to allocate memory for .dtors\n");
              return -ENOMEM;
            }

          loadinfo->dtors = (binfmt_dtor_t *)loadinfo->ctoralloc;

          /* Read the section header table into memory */

          ret = elf_read(loadinfo, (FAR uint8_t*)loadinfo->dtors, dtorsize,
                         shdr->sh_offset);
          if (ret < 0)
            {
              bdbg("Failed to allocate .dtors: %d\n", ret);
              return ret;
            }

          /* Fix up all of the .dtor addresses.  Since the addresses
           * do not lie in allocated memory, there will be no relocation
           * section for them.
           */

          for (i = 0; i < loadinfo->ndtors; i++)
            {
              FAR uintptr_t *ptr = (uintptr_t *)((FAR void *)(&loadinfo->dtors)[i]);

              bvdbg("dtor %d: %08lx + %08lx = %08lx\n",
                    i, *ptr, loadinfo->elfalloc, *ptr + loadinfo->elfalloc);

              *ptr += loadinfo->elfalloc;
            }
        }
      else
        {

          /* Save the address of the .dtors (actually, .init_array) where it was
           * loaded into memory.  Since the .dtors lie in allocated memory, they
           * will be relocated via the normal mechanism.
           */
 
          loadinfo->dtors = (binfmt_dtor_t*)shdr->sh_addr;
        }
    }

  return OK;
}

#endif /* CONFIG_BINFMT_CONSTRUCTORS */
