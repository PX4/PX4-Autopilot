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

#ifdef CONFIG_ELF_CONSTRUCTORS

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
 * Name: elf_sectname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int elf_sectname(FAR struct elf_loadinfo_s *loadinfo,
                               FAR const Elf32_Shdr *shdr)
{
  FAR Elf32_Shdr *shstr;
  FAR uint8_t *buffer;
  off_t  offset;
  size_t readlen;
  size_t bytesread;
  int shstrndx;
  int ret;

  /* Get the section header table index of the entry associated with the
   * section name string table. If the file has no section name string table,
   * this member holds the value SH_UNDEF.
   */

  shstrndx = loadinfo->ehdr.e_shstrndx;
  if (shstrndx == SHN_UNDEF)
    {
      bdbg("No section header string table\n");
      return -EINVAL;
    }

  /* Get the section name string table section header */

  shstr = &loadinfo->shdr[shstrndx];

  /* Get the file offset to the string that is the name of the section. This
   * is the sum of:
   *
   *   shstr->sh_offset: The file offset to the first byte of the section
   *     header string table data.
   *   shdr->sh_name: The offset to the name of the section in the section
   *     name table
   */

  offset = shstr->sh_offset + shdr->sh_name;

  /* Loop until we get the entire section name into memory */

  buffer    = loadinfo->iobuffer;
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
          bdbg("Failed to read section name\n");
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
 * Name: elf_findctors
 *
 * Description:
 *   Find C++ static constructors.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   On success, the index to the CTOR section is returned; A negated errno
 *   value is returned on failure.
 *
 ****************************************************************************/

static inline int elf_findctors(FAR struct elf_loadinfo_s *loadinfo)
{
  FAR const Elf32_Shdr *shdr;
  int ret;
  int i;

  /* Search through the shdr[] array in loadinfo for a section named .ctors */

  for (i = 0; i < loadinfo->ehdr.e_shnum; i++)
    {
      /* Get the name of this section */

      shdr = &loadinfo->shdr[i];
      ret  = elf_sectname(loadinfo, shdr);
      if (ret < 0)
        {
          bdbg("elf_sectname failed: %d\n", ret);
          return ret;
        }
 
      /* Check if the name of this section if ".ctors" */

      bvdbg("%d. Comparing \"%s\" and .ctors\"\n", i, loadinfo->iobuffer);

      if (strcmp(".ctors", (FAR const char *)loadinfo->iobuffer) == 0)
        {
          /* We found it... return the index */

          return i;
        }
    }

  /* We failed to find the .ctors sections.  This may not be an error; maybe
   * there are no static constructors.
   */

  return -ENOENT;
}

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

  /* Find the index to the section named ".ctors" */

  ctoridx = elf_findctors(loadinfo);
  if (ctoridx < 0)
    {
      /* This may not be a failure.  -ENOENT indicates that the file has no
       * static constructor section.
       */

      bvdbg("elf_findctors failed: %d\n", ctoridx);
      return ret == -ENOENT ? OK : ret;
    }

  /* Now we can get a pointer to the .ctor section in the section header
   * table.
   */

  shdr = &loadinfo->shdr[ctoridx];

  /* Allocate memory to hold a copy of the .ctor section */

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

      DEBUGASSERT(shdr->sh_entsize == sizeof(elf_ctor_t));

      loadinfo->ctors  = (elf_ctor_t)kmalloc(ctorsize);
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
        }

      /* Fix up all of the .ctor addresses */

      for (i = 0; i < loadinfo->nctors; i++)
        {
          FAR uintptr_t *ptr = (uintptr_t *)((FAR void *)(&loadinfo->ctors)[i]);

          bvdbg("ctor %d: %08lx + %08lx = %08lx\n", i,
                *ptr, loadinfo->alloc, *ptr + loadinfo->alloc);

          *ptr += loadinfo->alloc;
        }
    }

  return OK;
}

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

int elf_doctors(FAR struct elf_loadinfo_s *loadinfo)
{
  elf_ctor_t ctor = (elf_ctor_t)loadinfo->ctors;
  int i;

  /* Execute each constructor */

  for (i = 0; i < loadinfo->nctors; i++)
    {
      bvdbg("Calling ctor %d at %p\n", i, (FAR void *)ctor);

      ctor();
      ctor++;
    }

  return OK;
}

#endif /* CONFIG_ELF_CONSTRUCTORS */
