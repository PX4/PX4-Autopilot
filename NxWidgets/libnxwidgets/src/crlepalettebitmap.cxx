/****************************************************************************
 * NxWidgets/libnxwidgets/src/crlepalettebitmap.hxx
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

#include "nxconfig.hxx"
#include "crlepalettebitmap.hxx"

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

CRlePaletteBitmap::CRlePaletteBitmap(const struct SRlePaletteBitmap *bitmap)
{
  m_bitmap      = bitmap;
  m_lut         = bitmap->lut[0];
  startOfImage();
}

/**
 * Get the bitmap's color format.
 *
 * @return The bitmap's width.
 */

const uint8_t CRlePaletteBitmap::getColorFormat(void) const
{
  return m_bitmap->fmt;
}

/**
 * Get the bitmap's color format.
 *
 * @return The bitmap's color format.
 */

const uint8_t CRlePaletteBitmap::getBitsPerPixel(void) const
{
  return m_bitmap->bpp;
}

/**
 * Get the bitmap's width (in pixels/columns).
 *
 * @return The bitmap's pixel depth.
 */

const nxgl_coord_t CRlePaletteBitmap::getWidth(void) const
{
  return m_bitmap->width;
}

/**
 * Get the bitmap's height (in rows).
 *
 * @return The bitmap's height.
 */

const nxgl_coord_t CRlePaletteBitmap::getHeight(void) const
{
  return m_bitmap->height;
}

/**
 * Get the bitmap's width (in bytes).
 *
 * @return The bitmap's width.
 */

const nxgl_coord_t CRlePaletteBitmap::getStride(void) const
{
  // This only works if the bpp is an even multiple of 8-bit bytes

  return (m_bitmap->width * m_bitmap->bpp) >> 3;
}

/**
 * Use the colors associated with a selected image.
 *
 * @param selected.  true: Use colors for a selected widget,
 *   false: Use normal (default) colors.
 */

void CRlePaletteBitmap::setSelected(bool selected)
{
  m_lut = m_bitmap->lut[selected ? 1 : 0];
}

/**
 * Get one row from the bit map image using the selected LUT.
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

bool CRlePaletteBitmap::getRun(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                               FAR void *data)
{
  // Check ranges.  Casts to unsigned int are ugly but permit one-sided comparisons

  if (((unsigned int)x           <  (unsigned int)width) &&
      ((unsigned int)(x + width) <= (unsigned int)m_bitmap->width))
    {
      // Seek to the requested row

      if (!seekRow(y))
        {
          return false;
        }

      // Offset to the starting X position

      if (x > 0)
        {
          if (!skipPixels(x))
            {
              return false;
            }
        }

      // Then copy the requested number of pixels

      return copyPixels(width, data);
    }

  return false;
}

/**
 * Reset to the beginning of the image
 */

void CRlePaletteBitmap::startOfImage(void)
{
  m_row         = 0;
  m_col         = 0;
  m_rle         = &m_bitmap->data[0];
  m_remaining   = m_rle->npixels;
}

/**
 * Advance position data ahead.  Called after npixels have
 * have been consume.
 *
 * @param npixels The number of pixels to advance
 * @return False if this goes beyond the end of the image
 */

bool CRlePaletteBitmap::advancePosition(nxgl_coord_t npixels)
{
  // Advance to the next column after consuming 'npixels' on this colum
  int newcol = m_col + npixels;

  // Have we consumed the entire row?

  while (newcol >= m_bitmap->width)
    {
      // Advance to the next row

      newcol -= m_bitmap->width;
      m_row++;

      // If we still have pixels to account for but we have exceeded the
      // the size of the bitmap, then return false

      if (newcol > 0 && m_row >= m_bitmap->height)
        {
          return false;
        }
    }

  m_col = newcol;
  return true;
}

/**
 * Seek ahead the specific number of pixels -- discarding
 * and advancing.
 *
 * @param npixels The number of pixels to skip
 */

bool CRlePaletteBitmap::skipPixels(nxgl_coord_t npixels)
{
  do
    {
      // Skip values in the current RLE entry

      if (m_remaining > npixels)
        {
          // Subtract from the number of pixels remaining

          m_remaining -= npixels;
          return advancePosition(npixels);
        }

      // Not enough (or just enough) in the current entry.  Take what we
      // can from the current entry and move to the next entry

      npixels -= m_remaining;
      if (!advancePosition(m_remaining))
        {
          return false;
        }

      m_rle++;
      m_remaining = m_rle->npixels;
    }
  while (npixels > 0);

  return true;
}
 
/** Seek to the beginning of the next row
 *
 * @return False if this was the last row of the image
 */

bool CRlePaletteBitmap::nextRow(void)
{
  return skipPixels(m_bitmap->width - m_col);
}

/** Seek to the beignning specific row
 *
 * @param row The row number to seek to
 * @return False if this goes beyond the end of the image
 */

bool CRlePaletteBitmap::seekRow(nxgl_coord_t row)
{
  // Is the current position already past the requested position?

  if (row < m_row || (row == m_row && m_col != 0))
    {
      // Yes.. rewind to the beginning of the image

      startOfImage();
    }

  // Seek ahead, row-by-row until we are at the beginning of
  // the requested row

  while (m_row < row)
    {
      if (!nextRow())
        {
          return false;
        }
    }

  return true;
}

/** Copy the pixels from the current RLE entry the specified number of times.
 *
 * @param npixels The number of pixels to copy.  Must be less than or equal
 *  to m_remaining (not checked here, must be handled by higher level code).
 * @param data The memory location provided by the caller
 *   in which to return the data.  This should be at least
 *   (getWidth()*getBitsPerPixl() + 7)/8 bytes in length
 *   and properly aligned for the pixel color format.
 */

void CRlePaletteBitmap::copyColor(nxgl_coord_t npixels, FAR void *data)
{
  // Right now, only a single pixel depth is supported

  nxwidget_pixel_t *nxlut = (nxwidget_pixel_t *)m_lut;
  nxwidget_pixel_t color = nxlut[m_rle->lookup];

  // Copy the requested pixels

  FAR nxwidget_pixel_t *dest = (FAR nxwidget_pixel_t *)data;
  for (int i = 0; i < npixels; i++)
    {
      *dest++ = color;
    }

  // Adjust the number of pixels remaining in the RLE entry

  m_remaining -= npixels;
}

/** Copy pixels from the current position
 *
 * @param npixels The number of pixels to copy
 * @param data The memory location provided by the caller
 *   in which to return the data.  This should be at least
 *   (getWidth()*getBitsPerPixl() + 7)/8 bytes in length
 *   and properly aligned for the pixel color format.
 * @return False if this goes beyond the end of the image
 */

bool CRlePaletteBitmap::copyPixels(nxgl_coord_t npixels, FAR void *data)
{
  FAR nxwidget_pixel_t *ptr = (FAR nxwidget_pixel_t *)data;
  do
    {
      // Get values in the current RLE entry

      if (m_remaining > npixels)
        {
          // Copy npixels from the current RLE entry (it won't be empty)

          copyColor(npixels, (FAR void *)ptr);

          // Then advance row/column position information

          return advancePosition(npixels);
        }

      // Not enough (or just enough) in the current entry.  Take what we
      // can from the current entry and move to the next entry

      uint8_t nTaken = m_remaining; // Temporary.. copyColor clobbers m_remaining
      npixels -= m_remaining;       // New number of pixels needed

      // Copy all of the colors available in this RLE entry

      copyColor(m_remaining, (FAR void *)ptr);

      // Then advance row/column position information

      if (!advancePosition(nTaken))
        {
          return false;
        }

      // Advance the destination pointer by the number of pixels taken
      // from the RLE entry

      ptr += nTaken;

      // Then move to the next RLE entry

      m_rle++;
      m_remaining = m_rle->npixels;
    }
  while (npixels > 0);

  return true;
}
