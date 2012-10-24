/****************************************************************************
 * binfmt/libelf/libelf_load.c
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

#include <sys/types.h>
#include <sys/stat.h>

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <elf.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/binfmt/elf.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef MAX
#define MAX(x,y) ((x) > (y) ? (x) : (y))
#endif

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_filelen
 *
 * Description:
 *  Get the size of the ELF file
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int elf_filelen(FAR struct elf_loadinfo_s *loadinfo)
{
  struct stat buf;
  int ret;

  /* Get the file stats */

  ret = fstat(loadinfo->filfd, &buf);
  if (ret < 0)
    {
      int errval = errno;
      bdbg("Failed to fstat file: %d\n", errval);
      return -errval;
    }

  /* Verify that it is a regular file */

  if (!S_ISREG(buf.st_mode))
    {
      bdbg("Not a regular file.  mode: %d\n", buf.st_mode);
      return -errval;
    }

  /* TODO:  Verify that the file is readable */

  /* Return the size of the file in the loadinfo structure */

  loadinfo->filelen = buf.st_size;
  return OK;
}

/****************************************************************************
 * Name: elf_loadshdrs
 *
 * Description:
 *   Loads section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int elf_loadshdrs(FAR struct elf_loadinfo_s *loadinfo)
{
  size_t shdrsize;
  ssize_t bytesread;
  uint8_t buffer;
  off_t offset;
  int ret;

  DEBUGASSERT(loadinfo->shdrs == NULL);

  /* Verify that there are sections */

  if (loadinfo->e_shum < 1)
    {
      bdbg("No section(?)\n");
      return -EINVAL;
    }

  /* Get the total size of the section header table */

  shdrsize = (size_t)loadinfo->ehdr.e_shentsize * (size_t)loadinfo->e_shum;
  if(loadinfo->e_shoff + shdrsize > loadinfo->filelen)
    {
      bdbg("Insufficent space in file for section header table\n");
      return -ESPIPE;
    }

  /* Allocate memory to hold a working copy of the sector header table */

  loadinfo->shdrs = (FAR Elf32_Shdr*)kmalloc(shdrsize);
  if (!loadinfo->shdrs)
    {
      bdbg("Failed to allocate the section header table. Size: %ld\n", (long)shdrsize);
      return -ENOMEM;
    }

  /* Seek to the start of the section header table */

  offset = lseek(loadinfo->filfd, loadinfo->e_shoff, SEEK_SET);
  if (offset == (off_t)-1)
    {
      int errval = errno;
      bdbg("See to %ld failed: %d\n", (long)loadinfo->e_shoff, errval);
      ret = -errval;
      goto errout_with_alloc;
    }

  /* Now load the section header table into the allocated memory */

  buffer = loadinfo->shdrs;
  while (shdrsize > 0)
    {
      bytesread = read(loadinfo->filfd, buffer, shdrsize);
      if (bytes < 0)
        {
          int errval = errno;

          /* EINTR just means that we received a signal */

          if (errno != EINTR)
            {
              bdbg("read() failed: %d\n", errval);
              ret = -errval;
              goto errout_with_alloc;
            }
        }
      else if (bytes == 0)
        {
          bdbg("Unexpectged end of file\n");
          ret = -ENODATA;
          goto errout_with_alloc;
        }
      else
        {
          buffer   += bytesread;
          shdrsize -= bytesread;
        }
    }

  return OK;

errout_with_alloc:
  kfree(loadinfo->shdrs);
  loadinfo->shdrs = 0;
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

int elf_load(FAR struct elf_loadinfo_s *loadinfo)
{
# warning "Missing logic"
  return -ENOSYS;
}

