/****************************************************************************
 * graphics/nxbe/nxbe_colormap.c
 *
 *   Copyright (C) 2008-2009,2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include "nxbe.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxbe_colormap
 *
 * Description:
 *   Set the harware color map to the palette expected by NX
 *
 ****************************************************************************/

#if CONFIG_FB_CMAP
int nxbe_colormap(FAR NX_DRIVERTYPE *dev)
{
  struct fb_cmap_s cmap;
  uint8_t *alloc;
  uint8_t *red;
  uint8_t *green;
  uint8_t *blue;
  uint8_t rval;
  uint8_t gval;
  int     size;
  int     ndx;
  int     ret;
  int     i, j, k;

  /* Allocate the color map tables in one allocation:
   *
   *   size = 3 colors x CONFIG_NX_COLORS each x 8-bits per color
   */

  size  = 3 * CONFIG_NX_NCOLORS * sizeof(uint8_t);
  alloc = (uint8_t*)malloc(size);
  if (alloc == NULL)
    {
      return -ENOMEM;
    }
  memset(alloc, 0xff, size);

  /* Then get pointers to each color table */

  red   = alloc;
  green = &alloc[CONFIG_NX_NCOLORS];
  blue  = &alloc[2*CONFIG_NX_NCOLORS];

  /* Initialize the color map tables. 6*6*6 = 216, the rest
   * are (0xff, 0xfff 0xff)
   */

  ndx = 0;
  for (i = 0; i < 6; i++)
    {
      rval = (i * (CONFIG_NX_NCOLORS-1) / 5) << 8;
      for (j = 0; j < 6; j++)
        {
          gval = (j * (CONFIG_NX_NCOLORS-1) / 5) << 8;
          for (k = 0; k < 6; k++)
            {
              red[ndx]   = rval;
              green[ndx] = gval;
              blue[ndx]  = k * (CONFIG_NX_NCOLORS-1) / 5;
              ndx++;
            }
        }
    }

  /* Now configure the cmap structure */

  cmap.first  = 0;
  cmap.len    = CONFIG_NX_NCOLORS;
  cmap.red    = red;
  cmap.green  = green;
  cmap.blue   = blue;
#ifdef CONFIG_FB_TRANSPARENCY
  cmap.transp = NULL;
#endif

  /* Then set the color map */

  ret = dev->putcmap(dev, &cmap);

  free(alloc);
  return ret;
}
#endif
