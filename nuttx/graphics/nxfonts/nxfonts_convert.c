/****************************************************************************
 * graphics/nxfonts/nxfonts_convert.c
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT}
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING}
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
#include <stddef.h>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nxfonts_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Make sure the bits-per-pixel value has been set */

#ifndef NXFONTS_BITSPERPIXEL
#  error "NXFONTS_BITSPERPIXEL must be defined on the command line"
#endif

/* Set up bit blit macros for this BPP */

#if NXFONTS_BITSPERPIXEL == 1

#  define NXF_PIXELMASK           0x01
#  define NXF_SCALEX(x)           ((x) >> 3)
#  define NXF_PIXEL_T             uint8_t
#  define NXF_MULTIPIXEL(p)       ((p) ? 0xff : 0x00)

#elif NXFONTS_BITSPERPIXEL == 2

#  define NXF_PIXELMASK           0x03
#  define NXF_SCALEX(x)           ((x) >> 2)
#  define NXF_PIXEL_T             uint8_t
#  define NXF_MULTIPIXEL(p)       ((uint8_t)(p) << 6 | (uint8_t)(p) << 4 | (uint8_t)(p) << 2 | (p))

#elif NXFONTS_BITSPERPIXEL == 4

#  define NXF_PIXELMASK           0x0f
#  define NXF_SCALEX(x)           ((x) >> 1)
#  define NXF_PIXEL_T             uint8_t
#  define NXF_MULTIPIXEL(p)       ((uint8_t)(p) << 4 | (p))

#elif NXFONTS_BITSPERPIXEL == 8

#  define NXF_SCALEX(x)           (x)
#  define NXF_PIXEL_T             uint8_t

#elif NXFONTS_BITSPERPIXEL == 16

#  define NXF_SCALEX(x)           ((x) << 1)
#  define NXF_PIXEL_T             uint16_t

#elif NXFONTS_BITSPERPIXEL == 24

#  define NXF_SCALEX(x)           (((x) << 1) + (x))
#  define NXF_PIXEL_T             uint32_t

#elif NXFONTS_BITSPERPIXEL == 32

#  define NXF_SCALEX(x)           ((x) << 2)
#  define NXF_PIXEL_T             uint32_t

#endif

#if NXFONTS_BITSPERPIXEL < 8
#  ifdef CONFIG_NX_PACKEDMSFIRST
#     define NXF_INITMASK         (NXF_PIXELMASK << (8 - NXFONTS_BITSPERPIXEL))
#  else
#     define NXF_INITMASK         NXF_PIXELMASK
#  endif
#endif

/* Form a function name by concatenating two strings */

#define _NXF_FUNCNAME(a,b) a ## b
#define NXF_FUNCNAME(a,b)  _NXF_FUNCNAME(a,b)

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
 * Name: nxf_convert_*bpp
 *
 * Description:
 *   Convert the 1BPP font to a new pixel depth
 *
 * Input Parameters:
 *   dest   - The destination buffer provided by the caller.
 *   height - The max height of the returned char in rows
 *   width  - The max width of the returned char in pixels
 *   stride - The width of the destination buffer in bytes
 *   bm     - Describes the character glyph to convert
 *   color  - The color to use for '1' bits in the font bitmap
 *            (0 bits are transparent)
 *
 * Returned Value:
 *  OK on Success, ERROR: on failure with errno set appropriately.
 *  (never fails)
 *
 ****************************************************************************/

int NXF_FUNCNAME(nxf_convert,NXFONTS_SUFFIX)
(FAR NXF_PIXEL_T *dest, uint16_t height, uint16_t width, uint16_t stride,
 FAR const struct nx_fontbitmap_s *bm, nxgl_mxpixel_t color)
{
  FAR uint8_t *line;
  FAR NXF_PIXEL_T *dptr;
  FAR const uint8_t *sptr;
  uint8_t bmbyte;
  int bmbit;
  int row;
  int col;
  int bmndx;

#if NXFONTS_BITSPERPIXEL < 8
  NXF_PIXEL_T mpixel;
  NXF_PIXEL_T mask;
  NXF_PIXEL_T pixel;
  int nbits;
#endif

  /* Get the starting position */

  line = (uint8_t*)dest + bm->metric.yoffset * stride + NXF_SCALEX(bm->metric.xoffset);

  /* Then copy the font */

  height = ngl_min(bm->metric.height, height - bm->metric.yoffset);
  width  = ngl_min(bm->metric.width, width - bm->metric.xoffset);

  /* Render each row of the glyph */

  sptr = bm->bitmap;
#if NXFONTS_BITSPERPIXEL < 8
  mpixel = NXF_MULTIPIXEL(color);

  /* Handle each row in both the input and output */

  for (row = 0; row < height; row++)
    {
      /* Process each byte in the glyph row */

      col   = 0;
      dptr  = (FAR NXF_PIXEL_T*)line;
      pixel = *dptr;
      mask  = NXF_INITMASK;
      nbits = 0;

      for (bmndx = 0; bmndx < bm->metric.stride && col < width; bmndx++)
        {
          bmbyte = *sptr++;

          /* Process each bit in one byte */

          for (bmbit = 7; bmbit >= 0 && col < width; bmbit--, col++)
            {
              /* Is the bit set? */

              if (bmbyte & (1 << bmbit))
                {
                  /* Yes.. set the bit to 'color' in the output */

                  pixel = ((pixel & ~mask) | (mpixel & mask));
                }

#ifdef CONFIG_NX_PACKEDMSFIRST
              mask >>= NXFONTS_BITSPERPIXEL;
#else
              mask <<= NXFONTS_BITSPERPIXEL;
#endif
              nbits += NXFONTS_BITSPERPIXEL;
              if (nbits >= 8)
                {
                  *dptr++ = pixel;
                  pixel = *dptr;
                  mask  = NXF_INITMASK;
                  nbits = 0;
                }
            }
        }

      /* The entire glyph row has been rendered.  Handle any fractional bytes at
       * the end of the row 
       */

      if (nbits > 0)
        {
          *dptr = pixel;
        }

      /* Point to the beginning of the next row */

      line += stride;
    }
#else
  /* Handle each row in both the input and output */

  for (row = 0; row < height; row++)
    {
      /* Process each byte in the glyph */

      col  = 0;
      dptr = (FAR NXF_PIXEL_T*)line;

      for (bmndx = 0; bmndx < bm->metric.stride && col < width; bmndx++)
        {
          bmbyte = *sptr++;

          /* Process each bit in the byte */

          for (bmbit = 7; bmbit >= 0 && col < width; bmbit--, col++)
           {
              /* Is the bit set? */

              if (bmbyte & (1 << bmbit))
                {
                  /* Yes.. set the bit to 'color' in the output */

                  *dptr++ = color;
                }
              else
                {
                  /* No... keep the background color in the output */

                  dptr++;
                }
            }
        }

      /* Advance to the beginning of the next line in the destination */

      line += stride;
    }
#endif
  return OK;
}
