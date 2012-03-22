/****************************************************************************
 * NxWidgets/libnxwidgets/include/cbutton.hxx
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

#ifndef __INCLUDE_CGLYPHBUTTON_HXX
#define __INCLUDE_CGLYPHBUTTON_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "cwidgetstyle.hxx"

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
   * Forward references
   */

  class CWidgetControl;
  struct SBitmap;

  /**
   * Class representing a button that will display a glyph.
   * Class representing a clickable button that contains a bitmap image
   * instead of text. Displays two bitmaps, one when the button is clicked,
   * and one when it isn't.  The bitmaps should be the same size.
   */

  class CGlyphButton : public CNxWidget
  {
  protected:

    FAR const struct SBitmap *m_bitmapNormal;  /**< Bitmap when button is not clicked */
    FAR const struct SBitmap *m_bitmapClicked; /**< Bitmap when button is clicked */
    nxgl_coord_t              m_bitmapX;       /**< X coordinate of the bitmaps */
    nxgl_coord_t              m_bitmapY;       /**< Y coordinate of the bitmaps */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawContents(CGraphicsPort *port);

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     *
     * @param port The CGraphicsPort to draw to.
     * @see redraw()
     */

    virtual void drawBorder(CGraphicsPort *port);

    /**
     *
     * @param port Graphics port to draw to.
     */

    virtual void drawOutline(CGraphicsPort *port);

    /**
     * Redraws the button.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    virtual void onClick(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Raises an action event and redraws the button.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     */

    virtual void onRelease(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Redraws the button.
     *
     * @param x The x coordinate of the mouse.
     * @param y The y coordinate of the mouse.
     */

    virtual void onReleaseOutside(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CGlyphButton(const CGlyphButton &button) : CNxWidget(button) { }

  public:

    /**
     * Constructor.
     *
     * @param pWidgetControl The widget control for the display.
     * @param x The x coordinate of the button.
     * @param y The y coordinate of the button.
     * @param width The width of the button.
     * @param height The height of the button.
     * @param normalGlyph Glyph to display when unclicked.
     * @param clickedGlyph Glyph to display when clicked.
     * @param bitmapX The x coordinate at which the bitmaps will be drawn.
     * @param bitmapY The y coordinate at which the bitmaps will be drawn.
     * @param style The style that the button should use.  If this is not
     *   specified, the button will use the values stored in the global
     *   g_defaultWidgetStyle object.  The button will copy the properties of
     *   the style into its own internal style object.
     */

    CGlyphButton(CWidgetControl *pWidgetControl,
                 nxgl_coord_t x, nxgl_coord_t y,
                 nxgl_coord_t width, nxgl_coord_t height,
                 nxgl_coord_t bitmapX, nxgl_coord_t bitmapY,
                 FAR const struct SBitmap *normalGlyph,
                 FAR const struct SBitmap *clickedGlyph,
                 CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Destructor.
     */

    virtual inline ~CGlyphButton(void) { }

    /**
     * Insert the dimensions that this widget wants to have into the rect
     * passed in as a parameter.  All coordinates are relative to the widget's
     * parent.
     * @param rect Reference to a rect to populate with data.
     */

    virtual void getPreferredDimensions(CRect &rect) const;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CGLYPHBUTTON_HXX
