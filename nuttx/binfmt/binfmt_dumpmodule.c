/****************************************************************************
 * binfmt/binfmt_dumpmodule.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/binfmt/binfmt.h>

#include "binfmt_internal.h"

#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_BINFMT) && !defined(CONFIG_BINFMT_DISABLE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************
 * Name: dump_module
 *
 * Description:
 *   Load a module into memory and prep it for execution.
 *
 * Returned Value:
 *   This is a NuttX internal function so it follows the convention that
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ***********************************************************************/

int dump_module(FAR const struct binary_s *bin)
{
  if (bin)
    {
      bdbg("Module:\n");
      bdbg("  filename:  %s\n", bin->filename);
      bdbg("  argv:      %p\n", bin->argv);
      bdbg("  entrypt:   %p\n", bin->entrypt);
      bdbg("  mapped:    %p size=%d\n", bin->mapped, bin->mapsize);
      bdbg("  alloc:     %p %p %p\n", bin->alloc[0], bin->alloc[1], bin->alloc[2]);
#ifdef CONFIG_BINFMT_CONSTRUCTORS
      bdbg("  ctors:     %p nctors=%d\n", bin->ctors, bin->nctors);
      bdbg("  dtors:     %p ndtors=%d\n", bin->dtors, bin->ndtors);
#endif
#ifdef CONFIG_ADDRENV
      bdbg("  addrenv:   %p\n", bin->addrenv);
#endif
      bdbg("  stacksize: %d\n", bin->stacksize);
    }
  return OK;
}
#endif


