/****************************************************************************
 * libc/unistd/lib_execsymtab.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/symtab.h>

#ifdef CONFIG_LIBC_EXECFUNCS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* If CONFIG_LIBC_EXECFUNCS is defined in the configuration, then the
 * following must also be defined:
 */

/* Symbol table used by exec[l|v] */

#ifndef CONFIG_EXECFUNCS_SYMTAB
#  error "CONFIG_EXECFUNCS_SYMTAB must be defined"
#endif

/* Number of Symbols in the Table */

#ifndef CONFIG_EXECFUNCS_NSYMBOLS
#  error "CONFIG_EXECFUNCS_NSYMBOLS must be defined"
#endif

/****************************************************************************
 * Public Variables
 ****************************************************************************/

extern const struct symtab_s CONFIG_EXECFUNCS_SYMTAB;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR const struct symtab_s *g_exec_symtab = &CONFIG_EXECFUNCS_SYMTAB;
static int g_exec_nsymbols = CONFIG_EXECFUNCS_NSYMBOLS;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: exec_getsymtab
 *
 * Description:
 *   Get the current symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The location to store the symbol table.
 *   nsymbols - The location to store the number of symbols in the symbol table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void exec_getsymtab(FAR const struct symtab_s **symtab, FAR int *nsymbols)
{
  irqstate_t flags;

  DEBUGASSERT(symtab && nsymbols);

  /* Disable interrupts very briefly so that both the symbol table and its
   * size are returned as a single atomic operation.
   */

  flags     = irqsave();
  *symtab   = g_exec_symtab;
  *nsymbols = g_exec_nsymbols;
  irqrestore(flags);
}

/****************************************************************************
 * Name: exec_setsymtab
 *
 * Description:
 *   Select a new symbol table selection as an atomic operation.
 *
 * Input Parameters:
 *   symtab - The new symbol table.
 *   nsymbols - The number of symbols in the symbol table.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void exec_setsymtab(FAR const struct symtab_s *symtab, int nsymbols)
{
  irqstate_t flags;

  DEBUGASSERT(symtab);

  /* Disable interrupts very briefly so that both the symbol table and its
   * size are set as a single atomic operation.
   */

  flags           = irqsave();
  g_exec_symtab   = symtab;
  g_exec_nsymbols = nsymbols;
  irqrestore(flags);
}

#endif /* CONFIG_LIBC_EXECFUNCS */