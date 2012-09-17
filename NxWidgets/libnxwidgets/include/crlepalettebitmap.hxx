/****************************************************************************
 * NxWidgets/libnxwidgets/include/crlepalettebitmap.hxx
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
 *
 ****************************************************************************/

#ifndef __INCLUDE_CRLEPALETTBITMAP_HXX
#define __INCLUDE_CRLEPALETTBITMAP_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "ibitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  /**
   * One Run-Length Encoded (RLE) value
   */

  struct SRlePaletteBitmapEntry
  {
    uint8_t          npixels; /**< Number of pixels */
    uint8_t          lookup;  /**< Pixel RGB lookup index */
  };

  /**
   * Run-Length Encoded (RLE), Paletted Bitmap Structure
   */

  struct SRlePaletteBitmap
  {
    uint8_t          bpp;     /**< Bits per pixel */
    uint8_t          fmt;     /**< Color format */
    uint8_t          nlut;    /**< Number of colors in the Look-Up Table (LUT) */
    nxgl_coord_t     width;   /**< Width in pixels */
    nxgl_coord_t     height;  /**< Height in rows */
    FAR const void  *lut[2];  /**< Pointers to the beginning of the Look-Up Tables (LUTs) */

    /**
     * The pointer to the beginning of the RLE data
     */

    FAR const struct SRlePaletteBitmapEntry *data;
  };
 
  /**
   * Class providing bitmap accessor for a bitmap represented by SRlePaletteBitmap.
   */

  class CRlePaletteBitmap : public IBitmap
  {
  protected:
    /**
     * The bitmap that is being managed
     */

    FAR const struct SRlePaletteBitmap *m_bitmap;  /**< The bitmap that is being managed */

    /**
     * Accessor state data
     */

    nxgl_coord_t     m_row;       /**< Logical row number */
    nxgl_coord_t     m_col;       /**< Logical column number */
    uint8_t          m_remaining; /**< Number of bytes remaining in current entry */
    FAR const void  *m_lut;       /**< The selected LUT */
    FAR const struct SRlePaletteBitmapEntry *m_rle; /**< RLE entry being processed */

    /**
     * Reset to the beginning of the image
     */

    void startOfImage(void);

    /**
     * Advance position data ahead.  Called after npixels have
     * have been consume.
     *
     * @param npixels The number of pixels to advance
     * @return False if this goes beyond the end of the image
     */

    bool advancePosition(nxgl_coord_t npixels);

    /**
     * Seek ahead the specific number of pixels -- discarding
     * and advancing.
     *
     * @param npixels The number of pixels to skip
     * @return False if this goes beyond the end of the image
     */

    bool skipPixels(nxgl_coord_t npixels);
 
    /** Seek to the beginning of the next row
     *
     * @return False if this was the last row of the image
     */

    bool nextRow(void);

    /** Seek to the beignning specific row
     *
     * @param row The row number to seek to
     * @return False if this goes beyond the end of the image
     */

    bool seekRow(nxgl_coord_t row);

    /** Copy the pixels from the current RLE entry the specified number of times.
     *
     * @param npixels The number of pixels to copy.  Must be less than or equal
     *  to m_remaining.
     * @param data The memory location provided by the caller
     *   in which to return the data.  This should be at least
     *   (getWidth()*getBitsPerPixl() + 7)/8 bytes in length
     *   and properly aligned for the pixel color format.
     */

    void copyColor(nxgl_coord_t npixels, FAR void *data);

    /** Copy pixels from the current position
     *
     * @param npixels The number of pixels to copy
     * @param data The memory location provided by the caller
     *   in which to return the data.  This should be at least
     *   (getWidth()*getBitsPerPixl() + 7)/8 bytes in length
     *   and properly aligned for the pixel color format.
     * @return False if this goes beyond the end of the image
     */

    bool copyPixels(nxgl_coord_t npixels, FAR void *data);

  public:

    /**
     * Constructor.
     *
     * @param bitmap The bitmap structure being wrapped.
     */

    CRlePaletteBitmap(const struct SRlePaletteBitmap *bitmap);

    /**
     * Destructor.
     */

    inline ~CRlePaletteBitmap(void) {}

    /**
     * Get the bitmap's color format.
     *
     * @return The bitmap's width.
     */

    const uint8_t getColorFormat(void) const;

    /**
     * Get the bitmap's color format.
     *
     * @return The bitmap's color format.
     */

    const uint8_t getBitsPerPixel(void) const;

    /**
     * Get the bitmap's width (in pixels/columns).
     *
     * @return The bitmap's pixel depth.
     */

    const nxgl_coord_t getWidth(void) const;

    /**
     * Get the bitmap's height (in rows).
     *
     * @return The bitmap's height.
     */

    const nxgl_coord_t getHeight(void) const;

    /**
     * Get the bitmap's width (in bytes).
     *
     * @return The bitmap's width.
     */

    const nxgl_coord_t getStride(void) const;

    /**
     * Use the colors associated with a selected image.
     *
     * @param selected.  true: Use colors for a selected widget,
     *   false: Use normal (default) colors.
     */

    void setSelected(bool selected);

    /**
     * Get one row from the bit map image using the selected colors.
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

    bool getRun(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                FAR void *data);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CRLEPALETTBITMAP_HXX
