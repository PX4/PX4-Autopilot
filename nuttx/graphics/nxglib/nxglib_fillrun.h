/****************************************************************************
 * graphics/nxglib/nxsglib_fullrun.h
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
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

#ifndef __GRAPHICS_NXGLIB_NXGLIB_FILLRUN_H
#define __GRAPHICS_NXGLIB_NXGLIB_FILLRUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL < 16
#  define NXGLIB_RUNTYPE uint8_t
#elif NXGLIB_BITSPERPIXEL == 16
#  define NXGLIB_RUNTYPE uint16_t
#else
#  define NXGLIB_RUNTYPE uint32_t
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 2
static uint8_t g_wide_2bpp[4] = { 0x00, 0x55, 0xaa, 0xff };
#endif

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
 * Name: nxgl_fillrun_*bpp
 *
 * Description:
 *   fill a run with the specified color.
 *
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 1
static inline void nxgl_fillrun_1bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                    size_t npixels)
{
  /* Get the number of bytes to fill */

  unsigned int nbytes = (npixels + 7) >> 3;

  /* Get the value of the byte to fill */

  uint8_t wide = (color & 1) != 0 ? 0xff : 0x00;

  /* Fill the run with the color (it is okay to run a fractional byte over
   * the end)
   */

  memset(run, wide, nbytes);
}

#elif NXGLIB_BITSPERPIXEL == 2
static inline void nxgl_fillrun_2bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Get the number of bytes to fill */

  unsigned int nbytes = (npixels + 3) >> 2;

  /* Get the value of the byte to fill */

  uint8_t wide = g_wide_2bpp[color & 3];

  /* Fill the run with the color (it is okay to run a fractional byte over
   * the end)
   */

  memset(run, wide, nbytes);
}

#elif NXGLIB_BITSPERPIXEL == 4
static inline void nxgl_fillrun_4bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Get the number of bytes to fill */

  unsigned int nbytes = (npixels + 1) >> 1;

  /* Get the value of the byte to fill */

  uint8_t narrow = (uint8_t)color & 0x0f;
  uint8_t wide   = narrow | (narrow << 4);

  /* Fill the run with the color (it is okay to run a fractional byte over
   * the end)
   */

  memset(run, wide, nbytes);
}

#elif NXGLIB_BITSPERPIXEL == 8
static inline void nxgl_fillrun_8bpp(FAR uint8_t *run, nxgl_mxpixel_t color,
                                     size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy the end */

  memset(run, color, npixels);
}

#elif NXGLIB_BITSPERPIXEL == 16
static inline void nxgl_fillrun_16bpp(FAR uint16_t *run, nxgl_mxpixel_t color,
                                      size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy the end */

  while (npixels-- > 0)
    {
      *run++ = (uint16_t)color;
    }
}

#elif NXGLIB_BITSPERPIXEL == 24
static inline void nxgl_fillrun_24bpp(FAR uint32_t *run, nxgl_mxpixel_t color, size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy the end */
#warning "Assuming 24-bit color is not packed"
  while (npixels-- > 0)
    {
      *run++ = (uint32_t)color;
    }
}

#elif NXGLIB_BITSPERPIXEL == 32
static inline void nxgl_fillrun_32bpp(FAR uint32_t *run, nxgl_mxpixel_t color, size_t npixels)
{
  /* Fill the run with the color (it is okay to run a fractional byte overy the end */

  while (npixels-- > 0)
    {
      *run++ = (uint32_t)color;
    }
}
#else
#  error "Unsupported value of NXGLIB_BITSPERPIXEL"
#endif
#endif /* __GRAPHICS_NXGLIB_NXGLIB_FILLRUN_H */


