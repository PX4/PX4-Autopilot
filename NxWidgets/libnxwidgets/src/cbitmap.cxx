/****************************************************************************
 * NxWidgets/libnxwidgets/src/cbitmap.hxx
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
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <cstring>

#include <nuttx/nx/nxglib.h>

#include "cbitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param bitmap The bitmap structure being wrapped.
 */

CBitmap::CBitmap(const struct SBitmap *bitmap) : m_bitmap(bitmap) {}

/**
 * Get the bitmap's color format.
 *
 * @return The bitmap's width.
 */

const uint8_t CBitmap::getColorFormat(void) const
{
  return m_bitmap->fmt;
}

/**
 * Get the bitmap's color format.
 *
 * @return The bitmap's color format.
 */

const uint8_t CBitmap::getBitsPerPixel(void) const
{
  return m_bitmap->bpp;
}

/**
 * Get the bitmap's width (in pixels/columns).
 *
 * @return The bitmap's pixel depth.
 */

const nxgl_coord_t CBitmap::getWidth(void) const
{
  return m_bitmap->width;
}

/**
 * Get the bitmap's height (in rows).
 *
 * @return The bitmap's height.
 */

const nxgl_coord_t CBitmap::getHeight(void) const
{
  return m_bitmap->height;
}

/**
 * Get the bitmap's width (in bytes).
 *
 * @return The bitmap's width.
 */

const nxgl_coord_t CBitmap::getStride(void) const
{
  return m_bitmap->stride;
}

/**
 * Get one row from the bit map image.
 *
 * @param x The offset into the row to get
 * @param y The row number to get
 * @param width The number of pixels to get from the row
 * @param data The memory location provided by the caller
 *   in which to return the data.  This should be at least
 *   (getWidth()*getBitsPerPixl() + 7)/8 bytes in length
 *   and properly aligned for the pixel color format.
 * @param True if the run was returned successfully.
 */

bool CBitmap::getRun(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                     FAR void *data)
{
  // Check ranges.  Casts to unsigned int are ugly but permit one-sided comparisons

  if (((unsigned int)x           <  (unsigned int)width) &&
      ((unsigned int)(x + width) <= (unsigned int)m_bitmap->width) &&
      ((unsigned int)y           <  (unsigned int)m_bitmap->height))
    {
      // Get starting position of the copy (this will only work if bpp
      // is an even multiple of bytes).

      FAR uint8_t *start  = (FAR uint8_t*)m_bitmap->data +
                             y * m_bitmap->stride + 
                             ((x * m_bitmap->bpp) >> 3);

      // Get the number of bytes to copy.

      unsigned int length = (width * m_bitmap->bpp) >> 3;

      // And do the copy

      memcpy(data, start, length);
      return true;
    }

  return false;
}
