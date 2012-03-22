/****************************************************************************
 * NxWidgets/libnxwidgets/include/crectcache.hxx
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

#ifndef __INCLUDE_CRECTCACHE_HXX
#define __INCLUDE_CRECTCACHE_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cnxwidget.hxx"
#include "cwidgetstyle.hxx"
#include "tnxarray.hxx"

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
   * Maintains a list of foreground (ie. above children) and background (with
   * child overlapped-rects removed) rectangles representing the visible portions
   * of a widget.
   */

  class CRectCache
  {
  private:
    TNxArray<CRect> m_foregroundRegions; /**< List of the widget's visible regions */
    TNxArray<CRect> m_backgroundRegions; /**< List of the widget's visible regions with child rects removed */
    const CNxWidget *m_widget;           /**< Owning widget */
    bool m_foregroundInvalid;            /**< True if the foreground cache needs refreshing */
    bool m_backgroundInvalid;            /**< True if the background cache needs refreshing */

    /**
     * Cache the foreground regions.
     */

    void cacheForegroundRegions(void);

    /**
     * Cache the background regions.
     */

    void cacheBackgroundRegions(void);

  public:

    /**
     * Constructor.
     *
     * @param widget Widget that contains the rect cache.
     */

    CRectCache(const CNxWidget *widget);

    /**
     * Destructor.
     */

    inline ~CRectCache() { }

    /**
     * Rebuild the cache if it is invalid.
     */

    void cache(void);

    /**
     * Invalidates the cache.
     */

    inline void invalidate(void)
    {
      m_foregroundInvalid = true;
      m_backgroundInvalid = true;
    };

    /**
     * Return the list of background regions.  These are regions that are not overlapped by
     * child widgets.
     *
     * @return The list of background regions.
     */

    inline TNxArray<CRect> *getBackgroundRegions(void)
    {
      return &m_backgroundRegions;
    }

    /**
     * Return the list of foreground regions.  These are regions that represent the entire
     * visible surface of the widget - that is, any regions not overlapped by ancestors or
     * sublings of the widget - including any regions that are actually overlapped by
     * child widgets.
     *
     * @return The list of foreground regions.
     */

    inline TNxArray<CRect> *getForegroundRegions(void)
    {
      return &m_foregroundRegions;
    }

    /**
     * Works out which rectangles in the invalidRects list overlap this
     * widget, then cuts the rectangles into smaller pieces.  The overlapping
     * pieces are pushed into validRects, and the non-overlapping pieces are
     * pushed back into the invalidRects vector.
     *
     * @param invalidRects A vector of regions that need to be tested
     * for collisions against this widget; they represent regions that need
     * to be redrawn.
     * @param validRects A vector of regions that represents areas of the
     * display that do not need to be redrawn.
     * @param sender Pointer to the widget that initiated the split.
     */

    void splitRectangles(TNxArray<CRect> *invalidRects,
                         TNxArray<CRect> *validRects,
                         FAR const CNxWidget *sender) const;

    /**
     * Move any rectangles from the visibleRects list that overlap this widget
     * into the invisibleRects list.  Used during visible region calculations.
     *
     * @param visibleRects A vector of regions that are not overlapped.
     * @param invisibleRects A vector of regions that are overlapped.
     * @param widget The widget that requested the lists.
     * @see splitRectangles()
     */

    void removeOverlappedRects(TNxArray<CRect> *visibleRects,
                               TNxArray<CRect> *invisibleRects,
                               FAR const CNxWidget* widget) const;
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CRECTCACHE_HXX

