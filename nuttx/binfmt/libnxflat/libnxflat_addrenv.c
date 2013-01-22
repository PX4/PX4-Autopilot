/****************************************************************************
 * binfmt/libnxflat/libnxflat_addrenv.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

#include "libnxflat.h"

/****************************************************************************
 * Pre-Processor Definitions
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

int nxflat_addrenv_alloc(FAR struct nxflat_loadinfo_s *loadinfo, size_t envsize)
{
  FAR struct dspace_s *dspace;
#ifdef CONFIG_ADDRENV
  FAR void *vaddr;
  hw_addrenv_t oldenv;
  int ret;
#endif

  DEBUGASSERT(!loadinfo->dspace);

  /* Allocate the struct dspace_s container for the D-Space allocation */

  dspace = (FAR struct dspace_s *)kmalloc(sizeof(struct dspace_s));
  if (dspace == 0)
    {
      bdbg("ERROR: Failed to allocate DSpace\n");
      return -ENOMEM;
    }

#ifdef CONFIG_ADDRENV
  /* Create a D-Space address environment for the new NXFLAT task */

  ret = up_addrenv_create(envsize, &loadinfo->addrenv);
  if (ret < 0)
    {
      bdbg("ERROR: up_addrenv_create failed: %d\n", ret);
      goto errout_with_dspace;
    }

  /* Get the virtual address associated with the start of the address
   * environment.  This is the base address that we will need to use to
   * access the D-Space region (but only if the address environment has been
   * selected.
   */

  ret = up_addrenv_vaddr(loadinfo->addrenv, &vaddr);
  if (ret < 0)
    {
      bdbg("ERROR: up_addrenv_vaddr failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Clear all of the allocated D-Space memory.  We have to temporarily
   * selected the D-Space address environment to do this.
   */

  ret = up_addrenv_select(loadinfo->addrenv, &oldenv);
  if (ret < 0)
    {
      bdbg("ERROR: up_addrenv_select failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  memset(vaddr, 0, envsize);

  ret = up_addrenv_restore(oldenv);
  if (ret < 0)
    {
      bdbg("ERROR: up_addrenv_restore failed: %d\n", ret);
      goto errout_with_addrenv;
    }

  /* Success... save the fruits of our labor */

  loadinfo->dspace = dspace;
  dspace->crefs    = 1;  
  dspace->region   = (FAR uint8_t *)vaddr;
  return OK;

errout_with_addrenv:
  (void)up_addrenv_destroy(loadinfo->addrenv);
  loadinfo->addrenv = 0;

errout_with_dspace:
  kfree(dspace);
  return ret;
#else
  /* Allocate (and zero) memory to hold the ELF image */

  dspace->region = (FAR uint8_t *)kzalloc(envsize);
  if (!dspace->region)
    {
      kfree(dspace);
      return -ENOMEM;
    }

  loadinfo->dspace = dspace;
  dspace->crefs    = 1;  
  return OK;
#endif
}

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

void nxflat_addrenv_free(FAR struct nxflat_loadinfo_s *loadinfo)
{
  FAR struct dspace_s *dspace;
#ifdef CONFIG_ADDRENV
  int ret;
#endif

  DEBUGASSERT(loadinfo);
  dspace = loadinfo->dspace;

  if (dspace)
    {
#ifdef CONFIG_ADDRENV
      /* Destroy the address environment */

      ret = up_addrenv_destroy(loadinfo->addrenv);
      if (ret < 0)
        {
          bdbg("ERROR: up_addrenv_destroy failed: %d\n", ret);
        }

      loadinfo->addrenv = 0;
#else
      /* Free the allocated D-Space region */

      if (dspace->region)
        {
          kfree(dspace->region);
        }
#endif

      /* Now destroy the D-Space container */

      DEBUGASSERT(dspace->crefs == 1);
      kfree(dspace);
      loadinfo->dspace = NULL;
    }
}
