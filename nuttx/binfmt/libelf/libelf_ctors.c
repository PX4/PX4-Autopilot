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

#include <debug.h>

#include <nuttx/binfmt/elf.h>

#include "libelf"

#ifdef CONFIG_ELF_CONSTRUCTORS

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef FAR void (*ctor_t)(void);

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

int elf_findctors(FAR struct elf_loadinfo_s *loadinfo)
{
  /* Search through the shdr[] array in loadinfo for a section named .ctors */
#warning "Missing logic"

  /* Get the address of the beginning of the constructros from the sh_addr
   * field of the section.  Save that in the ctors field of the loadinfo
   * structure.
   */
#warning "Missing logic"

  /* Get the number of constructors from the sh_size field of the section.
   * Save that number in the nctors field of the loadinfo structure.
   */ 
#warning "Missing logic"
  return -ENOSYS;
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
  ctor_t ctor = (ctor_t)loadinfo->ctors;
  int i;

  /* Execute each constructor */

  for (i = 0; i < loadinfo->nctors; i++)
    {
      ctor();
      ctor++;
    }
}

#endif /* CONFIG_ELF_CONSTRUCTORS
