/****************************************************************************
 * include/clistboxdataitem.hxx
 * NxWidgets/libnxwidgets/
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

#ifndef __INCLUDE_CLISTBOXDATAITEM_HXX
#define __INCLUDE_CLISTBOXDATAITEM_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "clistdataitem.hxx"
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
   * Class representing a data item within a ListBox.
   */

  class CListBoxDataItem : public CListDataItem
  {
  private:
    nxwidget_pixel_t m_normalTextColor;    /**< Color used for text when not selected. */
    nxwidget_pixel_t m_normalBackColor;    /**< Color used for background when not selected. */
    nxwidget_pixel_t m_selectedTextColor;  /**< Color used for text when selected. */
    nxwidget_pixel_t m_selectedBackColor;  /**< Color used for background when selected. */

  public:

    /**
     * Constructor.
     *
     * @param text The text to display in the item.
     * @param value The value of the item.
     * @param normalTextColor Color to draw the text with when not selected.
     * @param normalBackColor Color to draw the background with when not selected.
     * @param selectedTextColor Color to draw the text with when selected.
     * @param selectedBackColor Color to draw the background with when selected.
     */

    CListBoxDataItem(const CNxString &text, const uint32_t value,
                     const nxwidget_pixel_t normalTextColor,
                     const nxwidget_pixel_t normalBackColor,
                     const nxwidget_pixel_t selectedTextColor,
                     const nxwidget_pixel_t selectedBackColor);

    /**
     * Get the color used for text when the item is unselected.
     *
     * @return The text color when the item is unselected.
     */

    inline nxwidget_pixel_t getNormalTextColor(void) const
    {
      return m_normalTextColor;
    }

    /**
     * Get the color used for the background when the item is unselected.
     *
     * @return The background color when the item is unselected.
     */

    inline nxwidget_pixel_t getNormalBackColor(void) const
    {
      return m_normalBackColor;
    }

    /**
     * Get the color used for text when the item is selected.
     *
     * @return The text color when the item is selected.
     */

    inline nxwidget_pixel_t getSelectedTextColor(void) const
    {
      return m_selectedTextColor;
    }

    /**
     * Get the color used for the background when the item is selected.
     *
     * @return The background color when the item is selected.
     */

    inline nxwidget_pixel_t getSelectedBackColor(void) const
    {
      return m_selectedBackColor;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CLISTBOXDATAITEM_HXX

