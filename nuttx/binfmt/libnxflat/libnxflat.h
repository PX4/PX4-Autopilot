/****************************************************************************
 * binfmt/libnxflat/libnxflat.h
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

#ifndef __BINFMT_LIBNXFLAT_LIBNXFLAT_H
#define __BINFMT_LIBNXFLAT_LIBNXFLAT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <nuttx/arch.h>
#include <nuttx/binfmt/nxflat.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Name: nxflat_addrenv_alloc
 *
 * Description:
 *   Allocate memory for the ELF image (elfalloc). If CONFIG_ADDRENV=n,
 *   elfalloc will be allocated using kzalloc().  If CONFIG_ADDRENV-y, then
 *   elfalloc will be allocated using up_addrenv_create().  In either case,
 *   there will be a unique instance of elfalloc (and stack) for each
 *   instance of a process.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   envsize - The size (in bytes) of the address environment needed for the
 *     ELF image.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nxflat_addrenv_alloc(FAR struct nxflat_loadinfo_s *loadinfo, size_t envsize);

/****************************************************************************
 * Name: nxflat_addrenv_select
 *
 * Description:
 *   Temporarity select the task's address environemnt.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
#  define nxflat_addrenv_select(l) up_addrenv_select((l)->addrenv, &(l)->oldenv)
#endif

/****************************************************************************
 * Name: nxflat_addrenv_restore
 *
 * Description:
 *   Restore the address environment before nxflat_addrenv_select() was called..
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef CONFIG_ADDRENV
#  define nxflat_addrenv_restore(l) up_addrenv_restore((l)->oldenv)
#endif

/****************************************************************************
 * Name: nxflat_addrenv_free
 *
 * Description:
 *   Release the address environment previously created by
 *   nxflat_addrenv_create().  This function  is called only under certain
 *   error conditions after the the module has been loaded but not yet
 *   started. After the module has been started, the address environment
 *   will automatically be freed when the module exits.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxflat_addrenv_free(FAR struct nxflat_loadinfo_s *loadinfo);

#endif /* __BINFMT_LIBNXFLAT_LIBNXFLAT_H */
