/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_unregister.c
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
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/nx/nxconsole.h>

#include "nxcon_internal.h"

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_unregister
 *
 * Description:
 *   Un-register to NX console device.
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void nxcon_unregister(NXCONSOLE handle)
{
  FAR struct nxcon_state_s *priv;
  char devname[NX_DEVNAME_SIZE];
  int i;

  DEBUGASSERT(handle);

  /* Get the reference to the driver structure from the handle */

  priv = (FAR struct nxcon_state_s *)handle;
  sem_destroy(&priv->exclsem);
#ifdef CONFIG_NXCONSOLE_NXKBDIN
  sem_destroy(&priv->waitsem);
#endif

  /* Free all allocated glyph bitmap */

  for (i = 0; i < CONFIG_NXCONSOLE_CACHESIZE; i++)
    {
      FAR struct nxcon_glyph_s *glyph = &priv->glyph[i];
      if (glyph->bitmap)
        {
          kfree(glyph->bitmap);
        }
    }

  /* Unregister the driver */

  snprintf(devname, NX_DEVNAME_SIZE, NX_DEVNAME_FORMAT, priv->minor);
  (void)unregister_driver(devname);

  /* Free the private data structure */

  kfree(handle);
}
