/****************************************************************************
 * NxWidgets/libnxwidgets/include/cnxfont.hxx
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
 ****************************************************************************/

#ifndef __INCLUDE_CNXFONT_HXX
#define __INCLUDE_CNXFONT_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#if defined(__cplusplus)

namespace NXWidgets
{
  class CNxString;
  struct SBitmap;

  /**
   * Class defining the properties of one font.
   */

  class CNxFont
  {
  private:
    enum nx_fontid_e m_fontId;              /**< The font ID. */
    NXHANDLE m_fontHandle;                  /**< The font handle */
    FAR const struct nx_font_s *m_pFontSet; /** < The font set metrics */
    nxgl_mxpixel_t m_fontColor;             /**< Color to draw the font with when rendering. */
    nxgl_mxpixel_t m_transparentColor;      /**< Background color that should not be rendered. */

  public:

    /**
     * CNxFont Constructor.
     *
     * @param fontid The font ID to use.
     * @param fontColor The font color to use.
     * @param transparentColor The color in the font bitmap used as the
     *    background color.
     */

    CNxFont(enum nx_fontid_e fontid, nxgl_mxpixel_t fontColor,
            nxgl_mxpixel_t transparentColor);

    /**
     * CNxFont Destructor.
     */

    ~CNxFont() { }

    /**
     * Checks if supplied character is blank in the current font.
     *
     * @param letter The character to check.
     * @return True if the glyph contains any pixels to be drawn.  False if
     * the glyph is blank.
     */

    const bool isCharBlank(const nxwidget_char_t letter) const;

    /**
     * Gets the color currently being used as the drawing color.
     *
     * @return The current drawing color.
     */
 
    inline const nxgl_mxpixel_t getColor() const
    {
      return m_fontColor;
    }

    /**
     * Sets the color to use as the drawing color.  If set, this overrides
     * the colors present in a non-monochrome font.
     * @param color The new drawing color.
     */

    inline void setColor(const nxgl_mxpixel_t color)
    {
      m_fontColor = color;
    }

    /**
     * Get the color currently being used as the transparent background
     * color.
     * @return The transparent background color.
     */
 
    inline const nxgl_mxpixel_t getTransparentColor() const
    {
      return m_transparentColor;
    }
    
    /**
     * Sets the transparent background color to a new value.
     * @param color The new background color.
     */
 
    inline void setTransparentColor(const nxgl_mxpixel_t color)
    {
      m_transparentColor = color;
    }

    /**
     * Draw an individual character of the font to the specified bitmap.
     *
     * @param bitmap The bitmap to draw to.
     * @param letter The character to output.
     */

    void drawChar(FAR SBitmap *bitmap, nxwidget_char_t letter);
    
    /**
     * Get the width of a string in pixels when drawn with this font.
     *
     * @param text The string to check.
     * @return The width of the string in pixels.
     */

    nxgl_coord_t getStringWidth(const CNxString &text) const;

    /**
     * Get the width of a portion of a string in pixels when drawn with this
     * font.
     *
     * @param text The string to check.
     * @param startIndex The start point of the substring within the string.
     * @param length The length of the substring in chars.
     * @return The width of the substring in pixels.
     */
 
    nxgl_coord_t getStringWidth(const CNxString& text,
                                int startIndex, int length) const;

    /**
     * Gets font metrics for a particular character
     *
     *
     * @param letter The character to get the width of.
     * @param metrics The location to return the font metrics
     */

    void getCharMetrics(nxwidget_char_t letter,
                        FAR struct nx_fontmetric_s *metrics) const;

    /**
     * Get the width of an individual character.
     *
     * @param letter The character to get the width of.
     * @return The width of the character in pixels.
     */

    nxgl_coord_t getCharWidth(nxwidget_char_t letter) const;

    /**
     * Get the height of an individual character.
     *
     * @param letter The letter to get the height of.
     * @return The height of the character in pixels.
     */
 
    inline nxgl_coord_t getCharHeight(nxwidget_char_t letter) const;

    /**
     * Gets the maximum width of the font.
     *
     * @return The height of the font.
     */

    inline const uint8_t getMaxWidth(void) const
    {
      return m_pFontSet->mxwidth;
    }

    /**
     * Gets the height of the font.
     *
     * @return The height of the font.
     */

    inline const uint8_t getHeight(void) const
    {
      return m_pFontSet->mxheight;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CNXFONT_HXX
