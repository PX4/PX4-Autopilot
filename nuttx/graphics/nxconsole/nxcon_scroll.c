/****************************************************************************
 * nuttx/graphics/nxconsole/nxcon_scroll.c
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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxfonts.h>

#include "nxcon_internal.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_movedisplay
 *
 * Description:
 *   This function implements the data movement for the scroll operation.  If
 *   we can read the displays framebuffer memory, then the job is pretty
 *   easy.  However, many displays (such as SPI-based LCDs) are often read-
 *   only.
 ****************************************************************************/

#ifdef CONFIG_NX_WRITEONLY
static inline void nxcon_movedisplay(FAR struct nxcon_state_s *priv,
                                     int bottom, int scrollheight)
{
  FAR struct nxcon_bitmap_s *bm;
  struct nxgl_rect_s rect;
  nxgl_coord_t row;
  int ret;
  int i;

  /* Move each row, one at a time.  They could all be moved at once (by calling
   * nxcon_redraw), but the since the region is cleared, then re-written, the
   * effect would not be good.  Below the region is also cleared and re-written,
   * however, in much smaller chunks.
   */

  rect.pt1.x = 0;
  rect.pt2.x = priv->wndo.wsize.w - 1;

  for (row = CONFIG_NXCONSOLE_LINESEPARATION; row < bottom; row += scrollheight)
    {
      /* Create a bounding box the size of one row of characters */

      rect.pt1.y = row;
      rect.pt2.y = row + scrollheight - 1;

      /* Clear the region */

      ret = priv->ops->fill(priv, &rect, priv->wndo.wcolor);
      if (ret < 0)
        {
          gdbg("fill failed: %d\n", errno);
        }

      /* Fill each character that might lie within in the bounding box */

      for (i = 0; i < priv->nchars; i++)
        {
          bm = &priv->bm[i];
          if (bm->pos.y <= rect.pt2.y && bm->pos.y + priv->fheight >= rect.pt1.y)
            {
              nxcon_fillchar(priv, &rect, bm);
            }
        }
    }

  /* Finally, clear the bottom part of the display */

  rect.pt1.y = bottom;
  rect.pt2.y = priv->wndo.wsize.h- 1;

  ret = priv->ops->fill(priv, &rect, priv->wndo.wcolor);
  if (ret < 0)
    {
      gdbg("nxcon_movedisplay: fill failed: %d\n", errno);
    }
}
#else
static inline void nxcon_movedisplay(FAR struct nxcon_state_s *priv,
                                     int bottom, int scrollheight)
{
  struct nxgl_rect_s rect;
  struct nxgl_point_s offset;
  int ret;

  /* Move the display in the range of 0-height up one scrollheight.  The
   * line at the bottom will be reset to the background color automatically.
   *
   * The source rectangle to be moved.
   */

  rect.pt1.x = 0;
  rect.pt1.y = scrollheight + CONFIG_NXCONSOLE_LINESEPARATION;
  rect.pt2.x = priv->wndo.wsize.w - 1;
  rect.pt2.y = priv->wndo.wsize.h - 1;

  /* The offset that determines how far to move the source rectangle */

  offset.x   = 0;
  offset.y   = -scrollheight;

  /* Move the source rectangle */

  ret = priv->ops->move(priv, &rect, &offset);
  if (ret < 0)
    {
      gdbg("move failed: %d\n", errno);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxcon_scroll
 ****************************************************************************/

void nxcon_scroll(FAR struct nxcon_state_s *priv, int scrollheight)
{
  int i;
  int j;

  /* Adjust the vertical position of each character */

  for (i = 0; i < priv->nchars; )
    {
      FAR struct nxcon_bitmap_s *bm = &priv->bm[i];

      /* Has any part of this character scrolled off the screen? */

      if (bm->pos.y < scrollheight + CONFIG_NXCONSOLE_LINESEPARATION)
        {
          /* Yes... Delete the character by moving all of the data */

          for (j = i; j < priv->nchars-1; j++)
            {
              memcpy(&priv->bm[j], &priv->bm[j+1], sizeof(struct nxcon_bitmap_s));
            }

          /* Decrement the number of cached characters ('i' is not incremented
           * in this case because it already points to the next character)
           */

          priv->nchars--;
        }

      /* No.. just decrement its vertical position (moving it "up" the
       * display by one line).
       */

      else
        {
          bm->pos.y -= scrollheight;

          /* We are keeping this one so increment to the next character */
 
          i++;
        }
    }

  /* And move the next display position up by one line as well */

  priv->fpos.y -= scrollheight;

  /* Move the display in the range of 0-height up one scrollheight. */

  nxcon_movedisplay(priv, priv->fpos.y, scrollheight);
}
