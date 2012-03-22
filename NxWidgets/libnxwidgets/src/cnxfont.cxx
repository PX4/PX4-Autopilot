/****************************************************************************
 * NxWidgets/libnxwidgets/src/cnxfont.cxx
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

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <cstring>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nxfonts.h>

#include "nxconfig.hxx"
#include "cnxstring.hxx"
#include "cstringiterator.hxx"
#include "cnxfont.hxx"
#include "cbitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CNxFont Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * CNxFont Constructor.
 *
 * @param fontid The font ID to use.
 * @param fontColor The font color to use.
 * @param transparentColor The color in the font bitmap used as the
 *    background color.
 */

CNxFont::CNxFont(enum nx_fontid_e fontid, nxgl_mxpixel_t fontColor,
                 nxgl_mxpixel_t transparentColor)
{
  m_fontId           = fontid;
  m_fontHandle       = nxf_getfonthandle(fontid);
  m_pFontSet         = nxf_getfontset(m_fontHandle);
  m_fontColor        = fontColor;
  m_transparentColor = transparentColor;
}

/**
 * Checks if supplied character is blank in the current font.
 *
 * @param letter The character to check.
 * @return True if the glyph contains any pixels to be drawn.  False if
 *   the glyph is blank.
 */

const bool CNxFont::isCharBlank(const nxwidget_char_t letter) const
{
  FAR const struct nx_fontbitmap_s *fbm;

  /* Get the bitmap associated with the character */

  fbm = nxf_getbitmap(m_fontHandle, (uint16_t)letter);
  return (fbm == NULL);
}

/**
 * Draw an individual character of the font to the specified bitmap.
 *
 * @param bitmap The bitmap to draw use. The caller should use
 *   the getFontMetrics method to assure that the buffer will hold
 *   the font.
 * @param letter The character to output.
 *
 * @return The width of the string in pixels.
 */

void CNxFont::drawChar(FAR SBitmap *bitmap, nxwidget_char_t letter)
{
  // Get the NX bitmap associated with the font

  FAR const struct nx_fontbitmap_s *fbm;
  fbm = nxf_getbitmap(m_fontHandle, letter);
  if (fbm)
    {
      // Get information about the font bitmap

      uint8_t fwidth  = fbm->metric.width + fbm->metric.xoffset;
      uint8_t fheight = fbm->metric.height + fbm->metric.yoffset;
      uint8_t fstride = (fwidth * bitmap->bpp + 7) >> 3;

      // Then render the glyph into the bitmap memory

      (void)FONT_RENDERER((FAR nxgl_mxpixel_t*)bitmap->data, fheight,
                          fwidth, fstride, fbm, m_fontColor);
    }
}

/**
 * Get the width of a string in pixels when drawn with this font.
 *
 * @param text The string to check.
 * @return The width of the string in pixels.
 */

nxgl_coord_t CNxFont::getStringWidth(const CNxString &text) const
{
  CStringIterator *iter = text.newStringIterator();

  // Get the width of the string of characters
  // Move to the first character

  unsigned int width = 0;
  if (iter->moveToFirst())
    {
      // moveToFirst returns true if there is at least one character
  
      do
        {
          // Add the width of the font bitmap for this character and
          // move the the next character

          nxwidget_char_t ch = iter->getChar();
          
          width += getCharWidth(ch);
        }
      while (iter->moveToNext());
    }

  // Return the total width

  delete iter;
  return width;
}

/**
 * Get the width of a portion of a string in pixels when drawn with this
 * font.
 *
 * @param text The string to check.
 * @param startIndex The start point of the substring within the string.
 * @param length The length of the substring in chars.
 * @return The width of the substring in pixels.
 */
 
nxgl_coord_t CNxFont::getStringWidth(const CNxString &text,
                                     int startIndex, int length) const
{
  CStringIterator *iter = text.newStringIterator();

  // Get the width of the string of characters

  iter->moveTo(startIndex);
  unsigned int width = 0;

  while (length-- > 0)
    {
      // Add the width of the font bitmap for this character

      nxwidget_char_t ch = iter->getChar();
      width += getCharWidth(ch);

      // Position to the next character in the string

      if (!iter->moveToNext())
        {
          break;
        }
    }

  // Return the total width

  delete iter;
  return width;
}

/**
 * Gets font metrics for a particular character
 *
 * @param letter The character to get the width of.
 * @param metrics The location to return the font metrics
 */

void CNxFont::getCharMetrics(nxwidget_char_t letter,
                             FAR struct nx_fontmetric_s *metrics) const
{
  FAR const struct nx_fontbitmap_s *fbm;

  // Get the font bitmap for this character

  fbm = nxf_getbitmap(m_fontHandle, letter);
  if (fbm)
    {
      // Return the metrics of the font

      memcpy(metrics, &fbm->metric, sizeof(struct nx_fontmetric_s));
    }
  else
    {
      // Use the metrics of a space which has no width and no height
      // but an xoffset equal to the standard width of a space.

      metrics->stride  = (m_pFontSet->spwidth * CONFIG_NXWIDGETS_BPP + 7) >> 3;
      metrics->width   = 0;
      metrics->height  = 0;
      metrics->xoffset = m_pFontSet->spwidth;
      metrics->yoffset = 0;
    }
}

/**
 * Get the width of an individual character.
 *
 * @param letter The character to get the width of.
 * @return The width of the character in pixels.
 */

nxgl_coord_t CNxFont::getCharWidth(nxwidget_char_t letter) const
{
  FAR const struct nx_fontbitmap_s *fbm;
  nxgl_coord_t width;

  /* Get the font bitmap for this character */

  fbm = nxf_getbitmap(m_fontHandle, letter);
  if (fbm)
    {
      /* Return the width of the font -- including the X offset */

      width = fbm->metric.width + fbm->metric.xoffset;
    }
  else
    {
      /* Use the width of a space */

     width = m_pFontSet->spwidth;
    }

  return width;
}
