/****************************************************************************
 * graphics/nxglib/nxsglib_copyrun.h
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

#ifndef __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H
#define __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>
#include <string.h>

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
 * Name: nxgl_copyrun_*bpp
 *
 * Description:
 *   Copy a row from an image into run.
 *
 ****************************************************************************/

#if NXGLIB_BITSPERPIXEL == 1
static inline void
nxgl_copyrun_1bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int remainder, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  uint8_t nextdata;
  unsigned int outpixels = 0;

  DEBUGASSERT(remainder > 0 && remainder < 8);

  /* Take only the first 8-remainder pixels from the first byte.
   * remainder is number between 1 and 7 (not zero!) that represents
   * the alignment of the pixel bits in the source.
   */

  indata = *src++;

#ifdef CONFIG_NX_PACKEDMSFIRST
  /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 0-(remainder-1)
   * are carried over to the first pass through the loop. For
   * example if remainder == 2:
   *
   * indata: xxAA AAAA maps to nextdata: AAAA AAxx
   */

  nextdata = (indata << remainder);

#else
  /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits (7-remainder)-7
   * are carried over to the first pass through the loop.  For example
   * if remainder == 2:
   *
   * indata: AAAA AAxx maps to nextdata: xxAA AAAA
   */
 
  nextdata = (indata >> remainder);

#endif

  /* Loop until all pixels have been packed into the destination.  Note:
   * a outpixels increments by 8 so a few extra pixels will be packed on
   * the output.  This should not be an issue.
   */

  while (outpixels < npixels)
    {
      /* Get the next byte from the source */

      indata  = *src++;
      outdata = nextdata;

      /* remainder is number between 1 and 7 (not zero!) that represents
       * the alignment of the pixel bits in the source.
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits (7-remainder)-7
       * are carried over from that last pass through the loop. For
       * example if remainder == 2:
       *
       *   nextdata = AAAA AAxx  - dest     = AAAA AABB
       *   src      = BBCC CCCC  - nextdata = CCCC CCxx
       */

       outdata |= (indata >> (8 - remainder));
       nextdata = (indata << remainder);
#else
      /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 0-(remainder-1)
       * are carried over from that last pass through the loop .  For
       * example if remainder == 2:
       *
       *   nextdata = xxAA AAAA  - dest     = BBAA AAAA
       *   src      = CCCC CCBB  - nextdata = xxCC CCCC
       */

       outdata |= (indata << (8 - remainder));
       nextdata = (indata >> remainder);
#endif

      /* Transfer the byte to the run buffer */

      *dest++ = outdata;
      outpixels += 8;
    }
}

#elif NXGLIB_BITSPERPIXEL == 2
static inline void
nxgl_copyrun_2bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int remainder, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  uint8_t nextdata;
  unsigned int outpixels = 0;
  unsigned int shift;

  DEBUGASSERT(remainder > 0 && remainder < 4);

  /* Take only the first 8-(2*remainder) pixels from the first byte.
   * remainder is number between 1 and 3 (not zero!) that represents
   * the alignment of the pixel bits in the source.
   */

  indata = *src++;
  shift  = (remainder << 1);

#ifdef CONFIG_NX_PACKEDMSFIRST
  /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 0-(2*remainder-1)
   * are carried over to the first pass through the loop. For
   * example if remainder == 1:
   *
   * indata: xxAA AAAA maps to nextdata: AAAA AAxx
   */

  nextdata = (indata << shift);
  
#else
  /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits (7-2*remainder)-7
   * are carried over to the first pass through the loop.  For example
   * if remainder == 1:
   *
   * indata: AAAA AAxx maps to nextdata: xxAA AAAA
   */
 
  nextdata = (indata >> shift);

#endif

  /* Loop until all pixels have been packed into the destination.  Note:
   * a outpixels increments by 8 so a few extra pixels will be packed on
   * the output.  This should not be an issue.
   */

  while (outpixels < npixels)
    {
      /* Get the next byte from the source */

      indata  = *src++;
      outdata = nextdata;

      /* remainder is number between 1 and 3 (not zero!) that represents
       * the alignment of the pixel bits in the source.
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits (7-2*remainder)-7
       * are carried over from that last pass through the loop. For example
       * if remainder == 1:
       *
       *   nextdata = AAAA AAxx  - dest     = AAAA AABB
       *   src      = BBCC CCCC  - nextdata = CCCC CCxx
       */

       outdata |= (indata >> (8 - shift));
       nextdata = (indata << shift);
#else
      /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 0-(2*remainder-1)
       * are carried over from that last pass through the loop.  For example
       * if remainder == 1:
       *
       *   nextdata = xxAA AAAA  - dest     = BBAA AAAA
       *   src      = CCCC CCBB  - nextdata = xxCC CCCC
       */

       outdata |= (indata << (8 - shift));
       nextdata = (indata >> shift);
#endif

      /* Transfer the byte to the run buffer */

      *dest++ = outdata;
      outpixels += 4;
    }
}

#elif NXGLIB_BITSPERPIXEL == 4
static inline void
nxgl_copyrun_4bpp(FAR const uint8_t *src, FAR uint8_t *dest,
                  unsigned int remainder, size_t npixels)
{
  uint8_t indata;
  uint8_t outdata;
  uint8_t nextdata;
  unsigned int outpixels = 0;

  DEBUGASSERT(remainder == 1);

  /* Take only the first 8-remainder pixels from the first byte.
   * remainder is number between 1 and 3 (not zero!) that represents
   * the alignment of the pixel bits in the source.
   */

  indata = *src++;

#ifdef CONFIG_NX_PACKEDMSFIRST
  /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 0-3
   * are carried over to the first pass through the loop. For
   * example:
   *
   * indata: xxxx AAAA maps to nextdata: AAAA xxxx
   */

  nextdata = (indata << 4);
  
#else
  /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 4-7
   * are carried over to the first pass through the loop.  For example:
   *
   * indata: AAAA xxxx maps to nextdata: xxxx AAAA
   */
 
  nextdata = (indata >> 4);

#endif

  /* Loop until all pixels have been packed into the destination.  Note:
   * a outpixels increments by 8 so a few extra pixels will be packed on
   * the output.  This should not be an issue.
   */

  while (outpixels < npixels)
    {
      /* Get the next byte from the source */

      indata  = *src++;
      outdata = nextdata;

      /* remainder is number between 1 and 3 (not zero!) that represents
       * the alignment of the pixel bits in the source.
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      /* If CONFIG_NX_PACKEDMSFIRST is defined, then bits 4-7
       * are carried over from that last pass through the loop (or are
       * ignored initially. For example if remainder == 1:
       *
       *   nextdata = AAAA xxxx  - dest     = AAAA BBBB
       *   src      = BBBB CCCC  - nextdata = CCCC xxxx
       */

      outdata |= (indata >> 4);
      nextdata = (indata << 4);
#else
      /* If CONFIG_NX_PACKEDMSFIRST is NOT defined, then bits 0-(remainder-1)
       * are carried over from that last pass through the loop (or are
       * ignored initially).  For example if remainder == 2:
       *
       *   nextdata = xxAA AAAA  - dest     = BBAA AAAA
       *   src      = CCCC CCBB  - nextdata = xxCC CCCC
       */

      outdata |= (indata << 4);
      nextdata = (indata >> 4);
#endif

      /* Transfer the byte to the run buffer */

      *dest++ = outdata;
      outpixels += 2;
    }
}
#endif
#endif /* __GRAPHICS_NXGLIB_NXGLIB_COPYRUN_H */


