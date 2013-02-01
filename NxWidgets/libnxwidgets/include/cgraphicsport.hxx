/****************************************************************************
 * NxWidgets/libnxwidgets/include/cgraphicsport.hxx
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
 * in all NxWidget files.  Thanks Antony!
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

#ifndef __INCLUDE_CGRAPHICSPORT_HXX
#define __INCLUDE_CGRAPHICSPORT_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include "nxconfig.hxx"
#include "inxwindow.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class  CNxFont;
  class  CNxString;
  class  CRect;
  struct SBitmap;
  
  /**
   * CGraphicsPort is the interface between a NXwidget and NX layer.
   */

  class CGraphicsPort
  {
  private:
    INxWindow     *m_pNxWnd;     /**< NX window interface. */
#ifdef CONFIG_NX_WRITEONLY
    nxgl_mxpixel_t m_backColor;  /**< The background color to use */
#endif

    /**
     * The underlying implementation for drawText functions
     * @param pos The window-relative x/y coordinate of the string.
     * @param bound The window-relative bounds of the string.
     * @param font The font to draw with.
     * @param string The string to output.
     * @param startIndex The start index within the string from which
     * drawing will commence.
     * @param length The number of characters to draw.
     * @param background Color to use for background if transparent is false.
     * @param transparent Whether to fill the background.
     */

    void _drawText(struct nxgl_point_s *pos, CRect *bound, CNxFont *font,
                   const CNxString &string, int startIndex, int length,
                   nxgl_mxpixel_t background, bool transparent);

  public:
    /**
     * Constructor.
     *
     * @param pNxWnd. An instance of the underlying window type.
     * @param backColor.  The background color is only needed if we
     *   cannot read from the graphics device.
     */

#ifdef CONFIG_NX_WRITEONLY
    CGraphicsPort(INxWindow *pNxWnd,
                  nxgl_mxpixel_t backColor = CONFIG_NXWIDGETS_DEFAULT_BACKGROUNDCOLOR);
#else
    CGraphicsPort(INxWindow *pNxWnd);
#endif

    /**
     * Destructor.
     */

    virtual ~CGraphicsPort();

    /**
     * Return the absolute x coordinate of the upper left hand corner of the
     * underlying window.
     *
     * @return The x coordinate of the window.
     */

    const nxgl_coord_t getX(void) const;
    
    /**
     * Return the absolute y coordinate of the upper left hand corner of the
     * underlying window.
     *
     * @return The y coordinate of the window.
     */

    const nxgl_coord_t getY(void) const;

    /**
     * Get the background color that will be used to fill in the spaces
     * when rendering fonts.  This background color is ONLY used if the
     * LCD device does not support reading GRAM contents.
     *
     * @return.  The current background color being used.
     */

#ifdef CONFIG_NX_WRITEONLY
    nxgl_mxpixel_t getBackColor(void) const
    {
       return m_backColor;
    }
#endif

    /**
     * Set the background color that will be used to fill in the spaces
     * when rendering fonts.  This background color is ONLY used if the
     * LCD device does not support reading GRAM contents.
     *
     * @return.  The current background color being used.
     */

#ifdef CONFIG_NX_WRITEONLY
    void setBackColor(nxgl_mxpixel_t backColor)
    {
       m_backColor = backColor;
    }
#endif

    /**
     * Draw a pixel into the window.
     *
     * @param x The window-relative x coordinate of the pixel.
     * @param y The window-relative y coordinate of the pixel.
     * @param color The color of the pixel.
     */

    void drawPixel(nxgl_coord_t x, nxgl_coord_t y, nxgl_mxpixel_t color);

    /**
     * Draw a horizontal line of the specified start position, width, and
     * color.
     *
     * @param x The x coordinate of the line.
     * @param y The y coordinate of the top-most end of the line.
     * @param width The width of the line in pixels.
     * @param color The color of the line.
     */

    void drawHorizLine(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                      nxgl_mxpixel_t color);

    /**
     * Draw a vertical line of the specified start position, width, and
     * color.
     *
     * @param x The x coordinate of the left-most end of the line.
     * @param y The y coordinate of the line.
     * @param height The height of the line in rows.
     * @param color The color of the line.
     */

    void drawVertLine(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t height,
                      nxgl_mxpixel_t color);

    /**
     * Draw a line of a fixed color in the window.
     *
     * @param x1 The x coordinate of the start point of the line.
     * @param y1 The y coordinate of the start point of the line.
     * @param x2 The x coordinate of the end point of the line.
     * @param y2 The y coordinate of the end point of the line.
     * @param color The color of the line.
     */

    void drawLine(nxgl_coord_t x1, nxgl_coord_t y1,
                  nxgl_coord_t x2, nxgl_coord_t y2,
                  nxgl_mxpixel_t color);

    /**
     * Draw a filled rectangle of the specified start position, end position,
     * width, and color.
     *
     * @param x The window-relative x coordinate of the rectangle.
     * @param y The window-relative y coordinate of the rectangle.
     * @param width The width of the rectangle in pixels.
     * @param height The height of the rectangle in rows.
     * @param color The color of the rectangle.
     */

    void drawFilledRect(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                        nxgl_coord_t height, nxgl_mxpixel_t color);

    /**
     * Draw an unfilled rectangle to the window
     *
     * @param x The window-relative x coordinate of the rectangle.
     * @param y The window-relative y coordinate of the rectangle.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * @param color The color of the rectangle .
     */

    void drawRect(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width,
                  nxgl_coord_t height, nxgl_mxpixel_t color);

    /**
     * Draw a bevelled rectangle to the window.
     *
     * @param x The x coordinate of the rectangle.
     * @param y The y coordinate of the rectangle.
     * @param width The width of the rectangle.
     * @param height The height of the rectangle.
     * @param shineColor The color of the top/left sides.
     * @param shadowColor The color of the bottom/right sides.
     */

    void drawBevelledRect(nxgl_coord_t x, nxgl_coord_t y,
                          nxgl_coord_t width, nxgl_coord_t height,
                          nxgl_mxpixel_t shineColor,
                          nxgl_mxpixel_t shadowColor);

    /**
     * Draw a filled circle at the specified position, size, and color.
     *
     * @param center The window-relative coordinates of the circle center.
     * @param radius The radius of the rectangle in pixels.
     * @param color The color of the rectangle.
     */

    inline void drawFilledCircle(struct nxgl_point_s *center, nxgl_coord_t radius,
                                 nxgl_mxpixel_t color)
    {
      (void)m_pNxWnd->drawFilledCircle(center, radius, color);
    }

    /**
     * Draw a string to the window.
     * @param pos The window-relative x/y coordinate of the string.
     * @param bound The window-relative bounds of the string.
     * @param font The font to draw with.
     * @param string The string to output.
     */

    void drawText(struct nxgl_point_s *pos, CRect *bound, CNxFont *font,
                  const CNxString &string);

    /**
     * Draw a particular length of a string to the window in a secific color.
     * @param pos The window-relative x/y coordinate of the string.
     * @param bound The window-relative bounds of the string.
     * @param font The font to draw with.
     * @param string The string to output.
     * @param startIndex The start index within the string from which
     * drawing will commence.
     * @param length The number of characters to draw.
     * @param color The color of the string.
     */

    void drawText(struct nxgl_point_s *pos, CRect *bound, CNxFont *font,
                  const CNxString &string, int startIndex, int length,
                  nxgl_mxpixel_t color);

    /**
     * Draw a portion of a string to the window.
     * @param pos The window-relative x/y coordinate of the string.
     * @param bound The window-relative bounds of the string.
     * @param font The font to draw with.
     * @param string The string to output.
     * @param startIndex The start index within the string from which
     * drawing will commence.
     * @param length The number of characters to draw.
     */

    void drawText(struct nxgl_point_s *pos, CRect *bound, CNxFont *font,
                  const CNxString &string, int startIndex, int length);

    /**
     * Draw a portion of a string to the window and fill the background
     * in one go.
     * @param pos The window-relative x/y coordinate of the string.
     * @param bound The window-relative bounds of the string.
     * @param font The font to draw with.
     * @param string The string to output.
     * @param startIndex The start index within the string from which
     * drawing will commence.
     * @param length The number of characters to draw.
     * @param color Foreground color
     * @param background Background color
     */

    void drawText(struct nxgl_point_s *pos, CRect *bound, CNxFont *font,
                  const CNxString &string, int startIndex, int length,
                  nxgl_mxpixel_t color, nxgl_mxpixel_t background);
    
    /**
     * Draw an opaque bitmap to the window.
     *
     * @param x The window-relative x coordinate to draw the bitmap to.
     * @param y The window-relative y coordinate to draw the bitmap to.
     * @param width The width of the bitmap to draw.
     * @param height The height of the bitmap to draw.
     * @param bitmap Pointer to the bitmap to draw.
     * @param bitmapX The window-relative x coordinate within the supplied
     *  bitmap to use as the origin.
     * @param bitmapY The window-relative y coordinate within the supplied
     *  bitmap to use as the origin.
     */

    void drawBitmap(nxgl_coord_t x, nxgl_coord_t y,
                    nxgl_coord_t width, nxgl_coord_t height,
                    const struct SBitmap *bitmap, int bitmapX, int bitmapY);

    /**
     * Draw a bitmap to the window, using the supplied transparent
     * color as an invisible color.
     *
     * @param x The window-relative x coordinate to draw the bitmap to.
     * @param y The window-relative y coordinate to draw the bitmap to.
     * @param width The width of the bitmap to draw.
     * @param height The height of the bitmap to draw.
     * @param bitmap Pointer to the bitmap to draw.
     * @param bitmapX The window-relative x coordinate within the supplied bitmap to use as
     * the origin.
     * @param bitmapY The window-relative y coordinate within the supplied bitmap to use as
     * the origin.
     * @param transparentColor The transparent color used in the bitmap.
     */

    void drawBitmap(nxgl_coord_t x, nxgl_coord_t y,
                    nxgl_coord_t width, nxgl_coord_t height,
                    const struct SBitmap *bitmap, int bitmapX, int  bitmapY,
                    nxgl_mxpixel_t transparentColor);

    /**
     * Draw a bitmap to the port in greyscale.
     *
     * @param x The window-relative x coordinate to draw the bitmap to.
     * @param y The window-relative y coordinate to draw the bitmap to.
     * @param width The width of the bitmap to draw.
     * @param height The height of the bitmap to draw.
     * @param bitmap Pointer to the bitmap to draw.
     * @param bitmapX The window-relative x coordinate within the supplied bitmap to use as
     * the origin.
     * @param bitmapY The window-relative y coordinate within the supplied bitmap to use as
     * the origin.
     */

    void drawBitmapGreyScale(nxgl_coord_t x, nxgl_coord_t y,
                             nxgl_coord_t width, nxgl_coord_t height,
                             const struct SBitmap *bitmap, int bitmapX, int bitmapY);

    /**
     * Copy a rectangular region from the source coordinateinates to the
     * destination coordinateinates.
     *
     * @param sourceX Source x coordinate.
     * @param sourceY Source y coordinate.
     * @param destX Destination x coordinate.
     * @param destY Destination y coordinate.
     * @param width Width of the rectangle to copy.
     * @param height Height of the rectangle to copy.
     */

    void copy(nxgl_coord_t sourceX, nxgl_coord_t sourceY,
              nxgl_coord_t destX, nxgl_coord_t destY,
              nxgl_coord_t width, nxgl_coord_t height);
 
    /**
     * Move a region by a specified distance in two dimensions.
     *
     * @param x X coordinate of the source area to move.
     * @param y Y coordinate of the source area to move.
     * @param deltaX Horizontal distance to move.
     * @param deltaY Vertical distance to move.
     * @param width Width of the area to move.
     * @param height Height of the area to move.
     */

    void move(nxgl_coord_t x, nxgl_coord_t y,
              nxgl_coord_t deltaX, nxgl_coord_t deltaY,
              nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Convert the region to greyscale.
     *
     * @param x X coordinate of the region to change.
     * @param y Y coordinate of the region to change.
     * @param width Width of the region to change.
     * @param height Height of the region to change.
     */

    void greyScale(nxgl_coord_t x, nxgl_coord_t y,
                   nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Invert colors in a region.  NOTE:  This allocates an in-memory
     * buffer the size of one row in graphic memory.  So it may only be
     * useful for inverting small regions and its only current use of for
     * the inverted cursor text.
     *
     * @param x X coordinate of the region to change.
     * @param y Y coordinate of the region to change.
     * @param width Width of the region to change.
     * @param height Height of the region to change.
     */

    void invert(nxgl_coord_t x, nxgl_coord_t y,
                nxgl_coord_t width, nxgl_coord_t height);

  };
}

#endif // __cplusplus

#endif // __INCLUDE_CGRAPHICSPORT_HXX
