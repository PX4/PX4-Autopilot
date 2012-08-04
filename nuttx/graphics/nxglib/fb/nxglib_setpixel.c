/****************************************************************************
 * graphics/nxglib/fb/nxglib_setpixel.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/fb.h>
#include <nuttx/nx/nxglib.h>

#include "nxglib_bitblit.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef NXGLIB_SUFFIX
#  error "NXGLIB_SUFFIX must be defined before including this header file"
#endif

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
 * Name: nxgl_setpixel_*bpp
 *
 * Descripton:
 *   Draw a single pixel in frambuffer memory at the given position and with
 *   the given color.   This is equivalent to nxgl_fillrectangle_*bpp() with
 *   a 1x1 rectangle but is more efficient.
 *
 ****************************************************************************/

void NXGL_FUNCNAME(nxgl_setpixel,NXGLIB_SUFFIX)
  (FAR struct fb_planeinfo_s *pinfo,
   FAR const struct nxgl_point_s *pos,
   NXGL_PIXEL_T color)
{
  FAR uint8_t *dest;

#if NXGLIB_BITSPERPIXEL < 8
  uint8_t shift;
  uint8_t mask;
#else
  FAR NXGL_PIXEL_T *pixel;
#endif

  /* Get the address of the first byte of the pixel to write */

  dest = pinfo->fbmem + pos->y * pinfo->stride + NXGL_SCALEX(pos->x);

#if NXGLIB_BITSPERPIXEL < 8

  /* Shift the color into the proper position */

# ifdef CONFIG_NX_PACKEDMSFIRST

#if NXGLIB_BITSPERPIXEL == 1
  shift   = (7 - (pos->x & 7));              /* Shift is 0, 1, ... 7 */
  mask    = (1 << shift);                    /* Mask is 0x01, 0x02, .. 0x80 */
  color <<= shift;                           /* Color is positioned under the mask */
#elif NXGLIB_BITSPERPIXEL == 2
  shift   = (6 - ((pos->x & 3) << 1));       /* Shift is 0, 2, 4, or 6 */
  mask    = (3 << shift);                    /* Mask is 0x03, 0x0c, 0x30, or 0xc0 */
  color <<= shift;                           /* Color is positioned under the mask */
#elif NXGLIB_BITSPERPIXEL == 4
  shift   = (4 - ((pos->x & 1) << 2));       /* Shift is 0 or 4 */
  mask    = (15 << shift);                   /* Mask is 0x0f or 0xf0 */
  color <<= shift;                           /* Color is positioned under the mask */
#else
#  error "Unsupport pixel depth"
#endif

# else /* CONFIG_NX_PACKEDMSFIRST */

#if NXGLIB_BITSPERPIXEL == 1
  shift   = (pos->x & 7);                    /* Shift is 0, 1, ... 7 */
  mask    = (1 << shift);                    /* Mask is 0x01, 0x02, .. 0x80 */
  color <<= shift;                           /* Color is positioned under the mask */
#elif NXGLIB_BITSPERPIXEL == 2
  shift   = (pos->x & 3) << 1;               /* Shift is 0, 2, 4, or 6 */
  mask    = (3 << shift);                    /* Mask is 0x03, 0x0c, 0x30, or 0xc0 */
  color <<= shift;                           /* Color is positioned under the mask */
#elif NXGLIB_BITSPERPIXEL == 4
  shift   = (pos->x & 1) << 2;               /* Shift is 0 or 4 */
  mask    = (15 << shift);                   /* Mask is 0x0f or 0xf0 */
  color <<= shift;                           /* Color is positioned under the mask */
#else
#  error "Unsupport pixel depth"
#endif
#endif /* CONFIG_NX_PACKEDMSFIRST */

  /* Handle masking of the fractional byte */

  *dest = (*dest & ~mask) | (color & mask);
#else

  /* Write the pixel (proper alignment assumed) */

   pixel = (FAR NXGL_PIXEL_T *)dest;
  *pixel = color;
#endif
}
