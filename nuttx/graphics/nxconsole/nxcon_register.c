/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_register.c
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
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "nxcon_internal.h"

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
 * Name: nxcon_allocate
 ****************************************************************************/

FAR struct nxcon_state_s *
  nxcon_register(NXCONSOLE handle, FAR struct nxcon_window_s *wndo,
                 FAR const struct nxcon_operations_s *ops, int minor)
{
  FAR struct nxcon_state_s *priv;
  char devname[NX_DEVNAME_SIZE];
  int ret;

  DEBUGASSERT(handle && wndo && ops && (unsigned)minor < 256);

  /* Allocate the driver structure */

  priv = (FAR struct nxcon_state_s *)kzalloc(sizeof(struct nxcon_state_s));
  if (!priv)
    {
      gdbg("Failed to allocate the NX driver structure\n");
      return NULL;
    }

  /* Initialize the driver structure */

  priv->ops     = ops;
  priv->handle  = handle;
  priv->minor   = minor;
  memcpy(&priv->wndo, wndo, sizeof( struct nxcon_window_s));

  sem_init(&priv->exclsem, 0, 1);
#ifdef CONFIG_DEBUG
  priv->holder  = NO_HOLDER;
#endif

#ifdef CONFIG_NXCONSOLE_NXKBDIN
  sem_init(&priv->waitsem, 0, 0);
#endif

  /* Select the font */

  priv->font = nxf_getfonthandle(wndo->fontid);
  if (!priv->font)
    {
      gdbg("Failed to get font ID %d: %d\n", wndo->fontid, errno);
      goto errout;
    }

  FAR const struct nx_font_s *fontset;

  /* Get information about the font set being used and save this in the
   * state structure
   */

  fontset         = nxf_getfontset(priv->font);
  priv->fheight   = fontset->mxheight;
  priv->fwidth    = fontset->mxwidth;
  priv->spwidth   = fontset->spwidth;

  /* Set up the text cache */

  priv->maxchars  = CONFIG_NXCONSOLE_MXCHARS;

  /* Set up the font glyph bitmap cache */

  priv->maxglyphs = CONFIG_NXCONSOLE_CACHESIZE;

  /* Set the initial display position */

  nxcon_home(priv);

  /* Show the cursor */

  priv->cursor.code = CONFIG_NXCONSOLE_CURSORCHAR;
  nxcon_showcursor(priv);

  /* Register the driver */

  snprintf(devname, NX_DEVNAME_SIZE, NX_DEVNAME_FORMAT, minor);
  ret = register_driver(devname, &g_nxcon_drvrops, 0666, priv);
  if (ret < 0)
    {
      gdbg("Failed to register %s\n", devname);
    }
  return (NXCONSOLE)priv;

errout:
  kfree(priv);
  return NULL;
}
