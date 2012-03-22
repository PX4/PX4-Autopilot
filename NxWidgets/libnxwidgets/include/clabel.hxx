/****************************************************************************
 * NxWidgets/libnxwidgets/include/clabel.hxx
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

#ifndef __INCLUDE_CLABEL_HXX
#define __INCLUDE_CLABEL_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "cwidgetstyle.hxx"
#include "cnxstring.hxx"

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
  class CRect;

  /**
   * Single-line label widget.  Can align text both vertically and
   * horizontally in different ways.
   */

  class CLabel : public CNxWidget
  {
  public:

    /**
     * Enum of horizontal alignment options.
     */

    enum TextAlignmentHoriz
    {
      TEXT_ALIGNMENT_HORIZ_CENTER = 0,  /**< Centre the text */
      TEXT_ALIGNMENT_HORIZ_LEFT   = 1,  /**< Align left */
      TEXT_ALIGNMENT_HORIZ_RIGHT  = 2   /**< Align right */
    };

    /**
     * Enum of vertical alignment options.
     */

    enum TextAlignmentVert
    {
      TEXT_ALIGNMENT_VERT_CENTER = 0,  /**< Align to centre of textbox */
      TEXT_ALIGNMENT_VERT_TOP    = 1,  /**< Align to top of textbox */
      TEXT_ALIGNMENT_VERT_BOTTOM = 2   /**< Align to bottom of textbox */
    };

  protected:
    CNxString           m_text;        /**< Text that the textbox will display */
    struct nxgl_point_s m_align;       /**< X/Y offset for text alignment */
    TextAlignmentHoriz  m_hAlignment;  /**< Horizontal alignment of the text */
    TextAlignmentVert   m_vAlignment;  /**< Vertical alignment of the text */
    bool                m_textChange;  /**< Redraw is due to a text change */
    bool                m_highlighted; /**< Label is highlighted */

    /**
     * Draw the area of this widget that falls within the clipping region.
     * Called by the redraw() function to draw all visible regions.
     * @param port The CGraphicsPort to draw to.
     *
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
     * Resize the widget to the new dimensions.
     *
     * @param width The new width.
     * @param height The new height.
     */

    virtual void onResize(nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Calculate the vertical position of the string based on the font
     *
     * height and the alignment options.
     */

    virtual void calculateTextPositionVertical(void);

    /**
     * Calculate the position of the string based on its length and the
     * alignment options.
     */

    virtual void calculateTextPositionHorizontal(void);

    /**
     * Updates the GUI after the text has changed.
     */

    virtual void onTextChange(void);

    /**
     * Copy constructor is protected to prevent usage.
     */

    inline CLabel(const CLabel &label) : CNxWidget(label) { };

  public:

    /**
     * Constructor for a label containing a string.
     *
     * @param pWidgetControl The controlling widget for the display
     * @param x The x coordinate of the text box, relative to its parent.
     * @param y The y coordinate of the text box, relative to its parent.
     * @param width The width of the textbox.
     * @param height The height of the textbox.
     * @param text Pointer to a string to display in the textbox.
     * @param style The style that the button should use.  If this is not
     *        specified, the button will use the global default widget
     *        style.
     */

    CLabel(CWidgetControl *pWidgetControl, nxgl_coord_t x, nxgl_coord_t y,
           nxgl_coord_t width, nxgl_coord_t height, const CNxString &text,
           CWidgetStyle *style = (CWidgetStyle *)NULL);

    /**
     * Destructor.
     */

    virtual inline ~CLabel() { }

    /**
     * Set the horizontal alignment of text within the label.
     *
     * @param alignment The horizontal position of the text.
     */

    virtual void setTextAlignmentHoriz(TextAlignmentHoriz alignment);

    /**
     * Set the vertical alignment of text within the label.
     *
     * @param alignment The vertical position of the text.
     */

    virtual void setTextAlignmentVert(TextAlignmentVert alignment);
    
    /**
     * Set the horizontal alignment of text within the label.
     *
     * @param alignment The horizontal position of the text.
     */

    inline const TextAlignmentHoriz getTextAlignmentHoriz(void) const
    {
      return m_hAlignment;
    }

    /**
     * Set the vertical alignment of text within the label.
     *
     * @param alignment The vertical position of the text.
     */

    inline const TextAlignmentVert getTextAlignmentVert(void) const
    {
      return m_vAlignment;
    }
    
    /**
     * Returns the string shown in the label.
     *
     * @return The label's text.
     */

    virtual inline const CNxString &getText(void) const
    {
      return m_text;
    }

    /**
     * Set the text displayed in the label.
     *
     * @param text String to display.
     */

    virtual void setText(const CNxString &text);

    /**
     * Append new text to the end of the current text displayed in the
     * label.
     *
     * @param text String to append.
     */

    virtual void appendText(const CNxString &text);

    /**
     * Insert text at the specified index.
     *
     * @param text The text to insert.
     * @param index Index at which to insert the text.
     */

    virtual void insertText(const CNxString &text, const int index);

    /**
     * Control the highlight state.
     *
     * @param highlightOn True(1), the label will be highlighted
     */

    virtual void highlight(bool highlightOn);

    /**
     * Return the current highlight state.
     *
     * @return True if the label is highlighted
     */

    virtual inline bool isHighlighted(void) const
    {
      return m_highlighted;
    }

    /**
     * Insert the dimensions that this widget wants to have into the rect
     * passed in as a parameter.  All coordinates are relative to the
     * widget's parent.
     *
     * @param rect Reference to a rect to populate with data.
     */

    virtual void getPreferredDimensions(CRect &rect) const;

    /**
     * Sets the font.
     *
     * @param font A pointer to the font to use.
     */

    virtual void setFont(CNxFont *font);

    /**
     * Is the redraw due to a text-only change?
     *
     * @return True if the redraw was caused by a text change
     */

    virtual inline bool isTextChange(void) const
    {
      return m_textChange;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CLABEL_HXX
