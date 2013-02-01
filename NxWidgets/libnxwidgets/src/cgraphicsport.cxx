/****************************************************************************
 * NxWidgets/libnxwidgets/src/cgraphicsport.cxx
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <cerrno>
#include <debug.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/rgbcolors.h>

#include "nxconfig.hxx"
#include "inxwindow.hxx"
#include "cnxstring.hxx"
#include "crect.hxx"
#include "cnxfont.hxx"
#include "cgraphicsport.hxx"
#include "cwidgetstyle.hxx"
#include "cbitmap.hxx"
#include "singletons.hxx"

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
 * @param pNxWnd. An instance of the underlying window type.
 * @param backColor.  The background color is only needed if we
 *   cannot read from the graphics device.
 */

#ifdef CONFIG_NX_WRITEONLY
CGraphicsPort::CGraphicsPort(INxWindow *pNxWnd, nxgl_mxpixel_t backColor)
{
  m_pNxWnd    = pNxWnd;
  m_backColor = backColor;
}
#else
CGraphicsPort::CGraphicsPort(INxWindow *pNxWnd)
{
  m_pNxWnd = pNxWnd;
}
#endif

/**
 * Destructor.
 */

CGraphicsPort::~CGraphicsPort(void)
{
  // m_pNxWnd is not deleted.  This is an abstract base class and
  // the caller of the CGraphicsPort instance is responsible for
  // the window destruction.
};

/**
 * Return the absolute x coordinate of the upper left hand corner of the
 * underlying window.
 *
 * @return The x coordinate of the underlying window.
 */

const nxgl_coord_t CGraphicsPort::getX(void) const
{
  struct nxgl_point_s pos;
  (void)m_pNxWnd->getPosition(&pos);
  return pos.x;
};

/**
 * Return the absolute y coordinate of the upper left hand corner of the
 * underlying window.
 *
 * @return The y coordinate of the underlying window.
 */

const nxgl_coord_t CGraphicsPort::getY(void) const
{
  struct nxgl_point_s pos;
  (void)m_pNxWnd->getPosition(&pos);
  return pos.y;
};

/**
 * Draw a pixel into the window.
 *
 * @param x The window-relative x coordinate of the pixel.
 * @param y The window-relative y coordinate of the pixel.
 * @param color The color of the pixel.
 */

void CGraphicsPort::drawPixel(nxgl_coord_t x, nxgl_coord_t y,
                              nxgl_mxpixel_t color)
{
  struct nxgl_point_s pos;
  pos.x = x;
  pos.y = y;
  m_pNxWnd->setPixel(&pos, color);
}

/**
 * Draw a horizontal line of the specified start position, width, and color.
 *
 * @param x The x coordinate of the line.
 * @param y The y coordinate of the top-most end of the line.
 * @param width The width of the line in pixels.
 * @param color The color of the line.
 */

void CGraphicsPort::drawHorizLine(nxgl_coord_t x, nxgl_coord_t y,
                                  nxgl_coord_t width, nxgl_mxpixel_t color)
{
  FAR struct nxgl_rect_s dest;
  nxgl_coord_t halfwidth;

  // Express the line as a rectangle

  halfwidth = width >> 1;

  dest.pt1.x = x;
  dest.pt1.y = y;
  dest.pt2.x = x + width - 1;
  dest.pt2.y = y;

  // Draw the line

  if (!m_pNxWnd->fill(&dest, color))
    {
      gdbg("INxWindow::fill failed\n");
    }
}

/**
 * Draw a vertical line of the specified start position, width, and
 * color.
 *
 * @param x The x coordinate of the left-most end of the line.
 * @param y The y coordinate of the line.
 * @param height The height of the line in rows.
 * @param color The color of the line.
 */

void CGraphicsPort::drawVertLine(nxgl_coord_t x, nxgl_coord_t y,
                                 nxgl_coord_t height, nxgl_mxpixel_t color)
{
  FAR struct nxgl_rect_s dest;

  // Express the line as a rectangle

  dest.pt1.x = x;
  dest.pt1.y = y;
  dest.pt2.x = x;
  dest.pt2.y = y + height - 1;

  // Draw the line

  if (!m_pNxWnd->fill(&dest, color))
    {
      gdbg("INxWindow::fill failed\n");
    }
}

/**
 * Draw a line of a fixed color in the window.
 *
 * @param x1 The x coordinate of the start point of the line.
 * @param y1 The y coordinate of the start point of the line.
 * @param x2 The x coordinate of the end point of the line.
 * @param y2 The y coordinate of the end point of the line.
 * @param color The color of the line.
 */

void CGraphicsPort::drawLine(nxgl_coord_t x1, nxgl_coord_t y1,
                             nxgl_coord_t x2, nxgl_coord_t y2,
                             nxgl_mxpixel_t color)
{
  struct nxgl_vector_s vector;

  vector.pt1.x = x1;
  vector.pt1.y = y1;
  vector.pt2.x = x2;
  vector.pt2.y = y2;

  if (!m_pNxWnd->drawLine(&vector, 1, color))
    {
      gdbg("INxWindow::drawLine failed\n");
    }
}

/**
 * Draw a filled rectangle to the bitmap.
 *
 * @param x The window-relative x coordinate of the rectangle.
 * @param y The window-relative y coordinate of the rectangle.
 * @param width The width of the rectangle in pixels.
 * @param height The height of the rectangle in rows.
 * @param color The color of the rectangle.
 */

void CGraphicsPort::drawFilledRect(nxgl_coord_t x, nxgl_coord_t y,
                                   nxgl_coord_t width, nxgl_coord_t height,
                                   nxgl_mxpixel_t color)
{
  struct nxgl_rect_s rect;
  rect.pt1.x = x;
  rect.pt1.y = y;
  rect.pt2.x = x + width - 1;
  rect.pt2.y = y + height - 1;
  m_pNxWnd->fill(&rect, color);
}

/**
 * Draw an unfilled rectangle to the window
 *
 * @param x The window-relative x coordinate of the rectangle.
 * @param y The window-relative y coordinate of the rectangle.
 * @param width The width of the rectangle.
 * @param height The height of the rectangle.
 * @param color The color of the rectangle outline.
 */

void CGraphicsPort::drawRect(nxgl_coord_t x, nxgl_coord_t y,
                             nxgl_coord_t width, nxgl_coord_t height,
                             nxgl_mxpixel_t color)
{
  drawHorizLine(x, y, width, color);
  drawHorizLine(x, y + height - 1, width, color);
  drawVertLine(x, y, height, color);
  drawVertLine(x + width -1, y, height, color);
}

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

void CGraphicsPort::drawBevelledRect(nxgl_coord_t x, nxgl_coord_t y,
                                     nxgl_coord_t width, nxgl_coord_t height,
                                     nxgl_mxpixel_t shineColor,
                                     nxgl_mxpixel_t shadowColor)
{
  drawHorizLine(x, y, width, shineColor);                        // Top
  drawHorizLine(x, y + height - 1, width, shadowColor);          // Bottom
  drawVertLine(x, y + 1, height - 2, shineColor);                // Left
  drawVertLine(x + width - 1, y + 1, height - 2, shadowColor);   // Right
}

/**
 * Draw an opaque bitmap to the window.
 *
 * @param x The window-relative x coordinate to draw the bitmap to.
 * @param y The window-relative y coordinate to draw the bitmap to.
 * @param width The width of the bitmap to draw.
 * @param height The height of the bitmap to draw.
 * @param bitmap Pointer to the bitmap to draw.
 * @param bitmapX The window-relative x coordinate within the supplied
 *    bitmap to use as the origin.
 * @param bitmapY The window-relative y coordinate within the supplied
 *   bitmap to use as the origin.
 */

void CGraphicsPort::drawBitmap(nxgl_coord_t x, nxgl_coord_t y,
                               nxgl_coord_t width, nxgl_coord_t height,
                               const struct SBitmap *bitmap,
                               int bitmapX, int bitmapY)
{
  // origin - The origin of the upper, left-most corner of the full bitmap.
  //          Both dest and origin are in window coordinates, however, origin
  //          may lie outside of the display.

  struct nxgl_point_s origin;
  origin.x   = x - bitmapX;
  origin.y   = y - bitmapY;

  // dest - Describes the rectangular on the display that will receive the
  //        the bit map.

  struct nxgl_rect_s dest;
  dest.pt1.x = x;
  dest.pt1.y = y;
  dest.pt2.x = x + width - 1;
  dest.pt2.y = y + height - 1;

  // Blit the bitmap

  (void)m_pNxWnd->bitmap(&dest, (FAR const void *)bitmap->data, &origin, bitmap->stride);
}

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

void CGraphicsPort::drawBitmap(nxgl_coord_t x, nxgl_coord_t y,
                               nxgl_coord_t width, nxgl_coord_t height,
                               const struct SBitmap *bitmap,
                               int bitmapX, int  bitmapY,
                               nxgl_mxpixel_t transparentColor)
{
  // Get the starting position in the image, offset by bitmapX and bitmapY into the image.

  FAR uint8_t *srcLine = (uint8_t *)bitmap->data +
                         bitmapY * bitmap->stride +
                         ((bitmapX * bitmap->bpp + 7) >> 3);
  FAR nxwidget_pixel_t *srcPtr  = (nxwidget_pixel_t *)srcLine;
  
  // Loop until all rows have been displayed

  nxgl_coord_t firstX = x;
  nxgl_coord_t lastX  = x + width;
  nxgl_coord_t lastY  = y + height;

  while (y < lastY)
    {
      // Search for the next non-transparent pixel in the source image

      while (*srcPtr == transparentColor)
        {
          // The pixel is transparent.  Move to the next column

          if (++x >= lastX)
            {
              // We are at the end of the row.  Reset to the next row

              x = firstX;
              y++;

              // Was that the last row:

              if (y >= lastY)
                {
                  // Yes, then we are finished

                  return;
                }

              // Update the source data pointers to the beginning of the next
              // row

              srcLine += bitmap->stride;
              srcPtr   = (nxwidget_pixel_t *)srcLine;
            }
          else
            {
              // This is another transparent pixel on the same row

              srcPtr++;
            }
        }

      // srcPtr points to the next non-transparent color.  Save this
      // as the start of a run that has length of at least one

      FAR nxwidget_pixel_t *runPtr   = srcPtr;
      nxgl_coord_t          runX     = x;
      nxgl_coord_t          runWidth = 0;

      // Now search for the next transparent pixel on the same row.
      // This will determine the length of the run

      do
        {
          // This is another non-transparent pixel on the same row

          runWidth++;
          srcPtr++;

          // Move to the next column

          if (++x >= lastX)
            {
              // We are at the end of the row.  Reset to the next row

              x = firstX;
              y++;

              // Update the source data pointers to the beginning of the next
              // row

              srcLine += bitmap->stride;
              srcPtr   = (nxwidget_pixel_t *)srcLine;

              // And break out of this loop

              break;
            }
        }
      while (*srcPtr != transparentColor);

      // When we come out of the above loop, either (1) srcPtr points to the
      // next transparent pixel on the same row, or (2) srcPtr points to the
      // first pixel in the next row.

      // origin - The origin of the upper, left-most corner of the full bitmap.
      //          Both dest and origin are in window coordinates, however, origin
      //          may lie outside of the display.

      struct nxgl_point_s origin;
      origin.x   = runX;
      origin.y   = y;

      // dest - Describes the rectangular on the display that will receive the
      //        the bit map.

      struct nxgl_rect_s dest;
      dest.pt1.x = runX;
      dest.pt1.y = y;
      dest.pt2.x = runX + runWidth - 1;
      dest.pt2.y = y;

      // Blit the bitmap

      (void)m_pNxWnd->bitmap(&dest, (FAR const void *)runPtr, &origin, bitmap->stride);
    }
}

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

void CGraphicsPort::drawBitmapGreyScale(nxgl_coord_t x, nxgl_coord_t y,
                                        nxgl_coord_t width, nxgl_coord_t height,
                                        const struct SBitmap *bitmap,
                                        int bitmapX, int bitmapY)
{
  // Working buffer.  Holds one converted row from the bitmap

  FAR nxwidget_pixel_t *run = new nxwidget_pixel_t[width];

  // Pointer to the beginning of the first source row

  FAR uint8_t *src = (FAR uint8_t *)bitmap->data + bitmapY * bitmap->stride + bitmapX;

  // Setup non-changing blit parameters

  struct nxgl_point_s origin;
  origin.x   = 0;
  origin.y   = 0;

  struct nxgl_rect_s dest;
  dest.pt1.x = x;
  dest.pt1.y = y;
  dest.pt2.x = x + width - 1;
  dest.pt2.y = y;

  // Convert each row to greyscale and send it to the display
   
  for (int row = 0; row < height; row++)
    {
      // Convert the next row

      FAR nxwidget_pixel_t *runSrc  = (FAR nxwidget_pixel_t *)src;
      FAR nxwidget_pixel_t *runDest = run;

      for (int col = 0; col < width; col++)
        {
          // Get the next RGB pixel and break out the individual components

          nxwidget_pixel_t rgb = *runSrc++;
          nxwidget_pixel_t r = RGB2RED(rgb);
          nxwidget_pixel_t g = RGB2GREEN(rgb);
          nxwidget_pixel_t b = RGB2BLUE(rgb);

          // A truly accurate greyscale conversion would be complex.  Let's
          // just average.
 
          nxwidget_pixel_t avg = (r + g + b) / 3;
          *runDest++ = MKRGB(avg, avg, avg);
        }

      // Now blit the single row

      (void)m_pNxWnd->bitmap(&dest, (FAR void *)bitmap->data, &origin, bitmap->stride);

       // Setup for the next source row

       y++;
       dest.pt1.y = y;
       dest.pt2.y = y;

       src += bitmap->stride;
    }

  delete run;
}

/**
 * Draw a string to the window.
 *
 * @param pos The window-relative x/y coordinate of the string.
 * @param bound The window-relative bounds of the string.
 * @param font The font to draw with.
 * @param string The string to output.
 */

void CGraphicsPort::drawText(struct nxgl_point_s *pos, CRect *bound,
                             CNxFont *font, const CNxString &string)
{
  _drawText(pos, bound, font, string, 0, string.getLength(), 0, true);
}

/**
 * Draw a particular length of a string to the window in a secific color.
 *
 * @param pos The window-relative x/y coordinate of the string.
 * @param bound The window-relative bounds of the string.
 * @param font The font to draw with.
 * @param string The string to output.
 * @param startIndex The start index within the string from which
 *   drawing will commence.
 * @param length The number of characters to draw.
 * @param color The color of the string.
 */

void CGraphicsPort::drawText(struct nxgl_point_s *pos, CRect *bound,
                             CNxFont *font, const CNxString &string,
                             int startIndex, int length,
                             nxgl_mxpixel_t color)
{
  // Temporarily change the font color

  nxgl_mxpixel_t savedColor = font->getColor();
  font->setColor(color);

  // Draw the string with this new color

  _drawText(pos, bound, font, string, startIndex, length, 0, true);

  // Restore the font color

  font->setColor(savedColor);
}

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

void CGraphicsPort::drawText(struct nxgl_point_s *pos, CRect *bound,
                             CNxFont *font, const CNxString &string,
                             int startIndex, int length)
{
  _drawText(pos, bound, font, string, startIndex, length, 0, true);
}

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

void CGraphicsPort::drawText(struct nxgl_point_s *pos, CRect *bound,
                             CNxFont *font, const CNxString &string,
                             int startIndex, int length,
                             nxgl_mxpixel_t color,
                             nxgl_mxpixel_t background)
{
  nxgl_mxpixel_t savedColor = font->getColor();
  font->setColor(color);
  
  _drawText(pos, bound, font, string, startIndex, length, background, false);
  
  font->setColor(savedColor);
}

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

void CGraphicsPort::_drawText(struct nxgl_point_s *pos, CRect *bound,
                              CNxFont *font, const CNxString &string,
                              int startIndex, int length,
                              nxgl_mxpixel_t background,
                              bool transparent)
{
  // Verify index and length

  int stringLength = string.getLength();
  if (startIndex >= stringLength)
    {
      return;
    }

  int endIndex = startIndex + length;
  if (endIndex > stringLength)
    {
      endIndex = stringLength;
    }

#ifdef CONFIG_NX_WRITEONLY
  if (transparent)
    {
      // Can't render transparently without reading memory.

      transparent = false;
      background = m_backColor;
    }
#endif
    
  // Allocate a bit of memory to hold the largest rendered font

  unsigned int bmWidth   = ((unsigned int)font->getMaxWidth() * CONFIG_NXWIDGETS_BPP + 7) >> 3;
  unsigned int bmHeight  = (unsigned int)font->getHeight();

  unsigned int glyphSize =  bmWidth * bmHeight;
  FAR uint8_t  *glyph    =  new uint8_t[glyphSize];

  // Get the bounding rectangle in NX form

  struct nxgl_rect_s boundingBox;
  bound->getNxRect(&boundingBox);
  
  // Loop setup

  struct SBitmap bitmap;
  bitmap.bpp    = CONFIG_NXWIDGETS_BPP;
  bitmap.fmt    = CONFIG_NXWIDGETS_FMT;
  bitmap.data   = (FAR const nxgl_mxpixel_t*)glyph;

  // Loop for each letter in the sub-string

  for (int i = startIndex; i < endIndex; i++)
    {
      // Get the next letter in the string

      const nxwidget_char_t letter = string.getCharAt(i);

      // Get the font metrics for this letter

      struct nx_fontmetric_s metrics;
      font->getCharMetrics(letter, &metrics);

      // Get the width of the font (in pixels)

      nxgl_coord_t fontWidth  = (nxgl_coord_t)(metrics.width + metrics.xoffset);

      // Does the letter have height?  Spaces have width, but no height

      if (metrics.height > 0 || !transparent)
        {
          // Set the current, effective size of the bitmap

          bitmap.width  = fontWidth;
          bitmap.height = bmHeight;
          bitmap.stride = (fontWidth * bitmap.bpp + 7) >> 3;

          // Describe the destination of the font as a bounding box

          struct nxgl_rect_s dest;
          dest.pt1.x = pos->x;
          dest.pt1.y = pos->y;
          dest.pt2.x = pos->x + fontWidth - 1;
          dest.pt2.y = pos->y + bmHeight - 1;

          // Get the interection of the font box and the bounding box

          struct nxgl_rect_s intersection;
          nxgl_rectintersect(&intersection, &dest, &boundingBox);

          // Skip to the next character if this one is completely outside
          // the bounding box.

          if (!nxgl_nullrect(&intersection))
            {
              // If we have been given a background color, use it to fill the array.
              // Otherwise initialize the bitmap memory by reading from the display.
              // The font renderer always renders the fonts on a transparent background.
              
              if (!transparent)
                {
                  // Set the glyph memory to the background color

                  nxwidget_pixel_t *bmPtr   = (nxwidget_pixel_t *)bitmap.data;
                  unsigned int      npixels = fontWidth * bmHeight;
                  for (unsigned int j = 0; j < npixels; j++)
                    {
                      *bmPtr++ = background;
                    }
                }
              else
                {
                  // Read the current contents of the destination into the glyph memory

                  m_pNxWnd->getRectangle(&dest, &bitmap);
                }

              // Render the font into the initialized bitmap

              font->drawChar(&bitmap, letter);

              // Then put the font on the display

              if (!m_pNxWnd->bitmap(&intersection, (FAR const void *)bitmap.data,
                                   pos, bitmap.stride))
                {
                  gvdbg("nx_bitmapwindow failed: %d\n", errno);
                }
            }
        }

      // Adjust the X position for the next character in the string        

      pos->x += fontWidth;
    }

  delete glyph;
}

/**
 * Copy a rectangular region from the source coordinates to the
 * destination coordinates.
 *
 * @param sourceX Source x coordinate.
 * @param sourceY Source y coordinate.
 * @param destX Destination x coordinate.
 * @param destY Destination y coordinate.
 * @param width Width of the rectangle to copy.
 * @param height Height of the rectangle to copy.
 */

void CGraphicsPort::copy(nxgl_coord_t sourceX, nxgl_coord_t sourceY,
                         nxgl_coord_t destX, nxgl_coord_t destY,
                         nxgl_coord_t width, nxgl_coord_t height)
{
  struct nxgl_rect_s rect;
  struct nxgl_point_s offset;

  rect.pt1.x = sourceX;
  rect.pt1.y = sourceY;
  rect.pt1.x = sourceX + width - 1;
  rect.pt1.y = sourceY + height - 1;

  offset.x = destX - sourceX;
  offset.y = destY - sourceY;

  (void)m_pNxWnd->move(&rect, &offset);
}

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

void CGraphicsPort::move(nxgl_coord_t x, nxgl_coord_t y,
                         nxgl_coord_t deltaX, nxgl_coord_t deltaY,
                         nxgl_coord_t width, nxgl_coord_t height)
{
  struct nxgl_rect_s rect;
  rect.pt1.x = x;
  rect.pt1.y = y;
  rect.pt2.x = x + width - 1;
  rect.pt2.y = y + height - 1;

  struct nxgl_point_s offset;
  offset.x = deltaX;
  offset.y = deltaY;

  (void)m_pNxWnd->move(&rect, &offset);
}

/**
 * Convert the region to greyscale.
 *
 * @param x X coordinate of the region to change.
 * @param y Y coordinate of the region to change.
 * @param width Width of the region to change.
 * @param height Height of the region to change.
 */

void CGraphicsPort::greyScale(nxgl_coord_t x, nxgl_coord_t y,
                              nxgl_coord_t width, nxgl_coord_t height)
{
  // Allocate memory to hold one row of graphics data

  unsigned int stride    = ((unsigned int)width * CONFIG_NXWIDGETS_BPP + 7) >> 3;
  FAR uint8_t *rowBuffer = new uint8_t[height * stride];
  if (!rowBuffer)
    {
      return;
    }

  // Describe the receiving bitmap memory

  SBitmap rowBitmap;
  rowBitmap.bpp    = CONFIG_NXWIDGETS_BPP;
  rowBitmap.fmt    = CONFIG_NXWIDGETS_FMT;
  rowBitmap.width  = width;
  rowBitmap.height = 1;
  rowBitmap.stride = stride;
  rowBitmap.data   = (FAR const nxgl_mxpixel_t *)rowBuffer;

  // Partially describe the source rectangle

  struct nxgl_rect_s rect;
  rect.pt1.x = x;
  rect.pt2.x = x + width - 1;

  // Partially setup the graphics origin

  struct nxgl_point_s origin;
  origin.x = x;

  // Loop for each row in the region to be inverted

  for (int row = 0; row < height; row++)
    {
      // Read the graphic memory corresponding to the row

      rect.pt1.y = rect.pt2.y = y + row;
      m_pNxWnd->getRectangle(&rect, &rowBitmap);

      // Convert each bit to  each bit

      nxwidget_pixel_t *ptr = (nxwidget_pixel_t *)rowBuffer;
      for (int col = 0; col < width; col++)
        {
          // Get the next RGB pixel and break out the individual components

          nxwidget_pixel_t color = *ptr;
          uint8_t red   = RGB2RED(color);
          uint8_t green = RGB2GREEN(color);
          uint8_t blue  = RGB2BLUE(color);

          // A truly accurate greyscale conversion would be complex.  Let's
          // just average.

          nxwidget_pixel_t avg = (red + green + blue) / 3;
          *ptr++ = MKRGB(avg, avg, avg);
        }

      // Then write the row back to graphics memory

      origin.y = rect.pt1.y;
      m_pNxWnd->bitmap(&rect, (FAR const void *)rowBitmap.data,
                       &origin, rowBitmap.stride) ;
    }

  delete rowBuffer;
}

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

void CGraphicsPort::invert(nxgl_coord_t x, nxgl_coord_t y,
                           nxgl_coord_t width, nxgl_coord_t height)
{
  // Allocate memory to hold one row of graphics data

  unsigned int stride    = ((unsigned int)width * CONFIG_NXWIDGETS_BPP + 7) >> 3;
  FAR uint8_t *rowBuffer = new uint8_t[height * stride];
  if (!rowBuffer)
    {
      return;
    }

  // Describe the receiving bitmap memory

  SBitmap rowBitmap;
  rowBitmap.bpp    = CONFIG_NXWIDGETS_BPP;
  rowBitmap.fmt    = CONFIG_NXWIDGETS_FMT;
  rowBitmap.width  = width;
  rowBitmap.height = 1;
  rowBitmap.stride = stride;
  rowBitmap.data   = (FAR const nxgl_mxpixel_t *)rowBuffer;

  // Partially describe the source rectangle

  struct nxgl_rect_s rect;
  rect.pt1.x = x;
  rect.pt2.x = x + width - 1;

  // Partially setup the graphics origin

  struct nxgl_point_s origin;
  origin.x = x;

  // Loop for each row in the region to be inverted

  for (int row = 0; row < height; row++)
    {
      // Read the graphic memory corresponding to the row

      rect.pt1.y = rect.pt2.y = y + row;
      m_pNxWnd->getRectangle(&rect, &rowBitmap);

      // Invert each bit

      nxwidget_pixel_t *ptr = (nxwidget_pixel_t *)rowBuffer;
      for (int col = 0; col < width; col++)
        {
          nxwidget_pixel_t color = *ptr;
          uint8_t red   = RGB2RED(color);
          uint8_t green = RGB2GREEN(color);
          uint8_t blue  = RGB2BLUE(color);
          *ptr++ = MKRGB(~red, ~green, ~blue);
        }

      // Then write the row back to graphics memory

      origin.y = rect.pt1.y;
      m_pNxWnd->bitmap(&rect, (FAR const void *)rowBitmap.data,
                       &origin, rowBitmap.stride) ;
    }

  delete rowBuffer;
};

