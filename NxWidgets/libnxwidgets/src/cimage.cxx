/****************************************************************************
 * NxWidgets/libnxwidgets/include/cimage.cxx
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
#include <debug.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "cgraphicsport.hxx"
#include "ibitmap.hxx"
#include "cbitmap.hxx"
#include "cimage.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
 
/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor for a label containing a string.
 *
 * @param pWidgetControl The controlling widget for the display
 * @param x The x coordinate of the image box, relative to its parent.
 * @param y The y coordinate of the image box, relative to its parent.
 * @param width The width of the textbox.
 * @param height The height of the textbox.
 * @param bitmap The source bitmap image.
 * @param style The style that the widget should use.  If this is not
 *        specified, the button will use the global default widget
 *        style.
 */

CImage::CImage(CWidgetControl *pWidgetControl, nxgl_coord_t x, nxgl_coord_t y,
               nxgl_coord_t width, nxgl_coord_t height, FAR IBitmap *bitmap,
               CWidgetStyle *style)
   : CNxWidget(pWidgetControl, x, y, width, height, 0, style)
{
  // Save the IBitmap instance

  m_bitmap   = bitmap;

  // Position the top/lef corner of the bitmap in the top/left corner of the display

  m_origin.x = 0;
  m_origin.y = 0;
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the
 * widget's parent.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CImage::getPreferredDimensions(CRect &rect) const
{
  nxgl_coord_t width  = m_bitmap->getWidth();
  nxgl_coord_t height = m_bitmap->getHeight();

  if (!m_flags.borderless)
    {
      width  += (m_borderSize.left + m_borderSize.right);
      height += (m_borderSize.top + m_borderSize.bottom);
    }

  rect.setX(m_rect.getX());
  rect.setY(m_rect.getY());
  rect.setWidth(width);
  rect.setHeight(height);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */
 
void CImage::drawContents(CGraphicsPort *port)
{
  // Get the the drawable region

  CRect rect;
  getRect(rect);

  // Allocate a working buffer that will hold one row of the bitmap

  FAR nxwidget_pixel_t *buffer = new nxwidget_pixel_t[rect.getWidth()];

  // Set up a simple bitmap structure to describe one row

  struct SBitmap bitmap;
  bitmap.bpp    = m_bitmap->getBitsPerPixel();
  bitmap.fmt    = m_bitmap->getColorFormat();
  bitmap.width  = rect.getWidth();
  bitmap.height = 1;
  bitmap.stride = (rect.getWidth() * m_bitmap->getBitsPerPixel()) >> 3;
  bitmap.data   = buffer;

  // This is the number of rows that we can draw at the top of the display
 
  nxgl_coord_t nTopRows = m_bitmap->getHeight() - m_origin.y;
  if (nTopRows > rect.getHeight())
    {
      nTopRows = rect.getHeight(); 
    }
  else if (nTopRows < 0)
    {
      nTopRows = 0;
    }

  // This the starting row in the bitmap image where we will begin drawing.

  nxgl_coord_t imageRow = m_origin.y;

  // This the starting row in the display image where we will begin drawing.

  nxgl_coord_t displayRow = rect.getY();

  // Are we going to draw any rows at the top of the display?

  if (nTopRows > 0)
    {
      // This is the number of columns that we can draw on the left side of
      // the display

      nxgl_coord_t nLeftPixels = m_bitmap->getWidth() - m_origin.x;
  
      // This is the number of rows that we have to pad on the right if the display
      // width is wider than the image width

      nxgl_coord_t nRightPad;
      if (nLeftPixels >= rect.getWidth())
        {
          nRightPad = 0;
        }
      else
        {
          nRightPad = rect.getWidth() - nLeftPixels;
        }

      // Apply the padding to the right hand side

      FAR nxwidget_pixel_t *ptr       = &buffer[nLeftPixels];
      nxwidget_pixel_t      backColor = getBackgroundColor();

      for (int column = nLeftPixels; column < rect.getWidth(); column++)
        {
          *ptr++ = backColor;
        }

      // The is the row number of the first row that we cannot draw into

      nxgl_coord_t lastTopRow = nTopRows + m_origin.y;

      // Now draw the rows from the offset position

      for (; imageRow < lastTopRow; imageRow++, displayRow++)
        {
          // Get the graphics data for the right hand side of this row

          if (!m_bitmap->getRun(m_origin.x, imageRow, nLeftPixels, buffer))
            {
              gvdbg("IBitmap::getRun failed at image row\n", imageRow);
              delete buffer;
              return;
            }

          // Replace any transparent pixels with the background color.
          // Then we can use the faster opaque drawBitmap() function.
  
          ptr = buffer;
          for (int i = 0; i < nLeftPixels; i++, ptr++)
            {
              if (*ptr == CONFIG_NXWIDGETS_TRANSPARENT_COLOR)
                {
                  *ptr = backColor;
                }
            }

          // And put these on the display

          port->drawBitmap(rect.getX(), displayRow, rect.getWidth(), 1,
                           &bitmap, 0, 0);
        }
    }

  // Are we going to draw any rows at the top of the display?

  if (nTopRows < rect.getHeight())
    {
      // Pad the entire row

      FAR nxwidget_pixel_t *ptr      = buffer;
      nxwidget_pixel_t      backColor = getBackgroundColor();

      for (int column = 0; column < rect.getWidth(); column++)
        {
          *ptr++ = backColor;
        }

      // Now draw the rows from the offset position

      for (; displayRow < rect.getHeight(); displayRow++)
        {
          // Put the padded row on the display

          port->drawBitmap(rect.getX(), displayRow, rect.getWidth(), 1,
                           &bitmap, 0, 0);
        }
    }

   delete buffer;
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CImage::drawBorder(CGraphicsPort *port)
{
  if (!isBorderless())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Set the horizontal position of the bitmap.  Zero is the left edge
 * of the bitmap and values >0 will move the bit map to the right.
 * This method is useful for horizontal scrolling a large bitmap
 * within a smaller window
 */

void CImage::setImageLeft(nxgl_coord_t column)
{
  if (column > 0 && column <= m_bitmap->getWidth())
    {
      m_origin.x = column;
    }
}

/**
 * Set the vertical position of the bitmap.  Zero is the top edge
 * of the bitmap and values >0 will move the bit map down.
 * This method is useful for vertical scrolling a large bitmap
 * within a smaller window
 */

void CImage::setImageTop(nxgl_coord_t row)
{
  if (row > 0 && row <= m_bitmap->getHeight())
    {
      m_origin.x = row;
    }
}

