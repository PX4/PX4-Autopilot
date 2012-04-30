/****************************************************************************
 * NxWidgets/libnxwidgets/include/cbgwindow.hxx
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

#ifndef __INCLUDE_CRECT_HXX
#define __INCLUDE_CRECT_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/nx/nx.h>

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
   * Class describing a rectangle.
   */

  class CRect
  {
  public:
    struct nxgl_point_s m_pos;   /**< The position of the rectangle in the window */
    struct nxgl_size_s  m_size;  /**< The size of the rectangle */

    /**
     * Constructor.
     */

    CRect(void);

    /**
     * Constructor.
     *
     * @param x The x coordinate of the rect.
     * @param y The y coordinate of the rect.
     * @param width The width of the rect.
     * @param height The height of the rect.
     */

    CRect(nxgl_coord_t x, nxgl_coord_t y,
          nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Constructor.
     *
     * @param rect Pointer to an NX rectangle
     */

    CRect(FAR const nxgl_rect_s *rect);

    /**
     * Copy constructor.
     *
     * @param rect CRect to copy.
     */

    CRect(const CRect& rect);

    /**
     * Create a rect object from the supplied coordinates.
     *
     * @param x1 The x coordinate of the rect's top-left corner.
     * @param y1 The y coordinate of the rect's top-left corner.
     * @param x2 The x coordinate of the rect's bottom-right corner.
     * @param y2 The y coordinate of the rect's bottom-right corner.
     * @return A new rect.
     */

    static CRect fromCoordinates(nxgl_coord_t x1, nxgl_coord_t y1,
                                 nxgl_coord_t x2, nxgl_coord_t y2);

    /**
     * Get the rectangle's left x coordinate.
     *
     * @return The rectangle's x coordinate.
     */

    inline nxgl_coord_t getX(void) const
    {
      return m_pos.x;
    }

    /**
     * Get the rectangle's top y coordinate.
     * @return The rectangle's y coordinate.
     */

    inline nxgl_coord_t getY(void) const
    {
      return m_pos.y;
    }

    /**
     * Get the rectangle's width.
     *
     * @return The rectangle's width.
     */

    inline nxgl_coord_t getWidth(void) const
    {
      return m_size.w;
    }

    /**
     * Get the size of the rectangle
     *
     * @return The rectangle's size
     */

    inline void getSize(struct nxgl_size_s &size) const
    {
      size.h = m_size.h;
      size.w = m_size.w;
    }

    /**
     * Get the rectangle's height.
     *
     * @return The rectangle's height.
     */

    inline nxgl_coord_t getHeight(void) const
    {
      return m_size.h;
    }

    /**
     * Get the NX rectangle representation
     *
     * @param rect Pointer to NX rectangle
     */

    inline void getNxRect(FAR struct nxgl_rect_s *rect) const
    {
      rect->pt1.x = m_pos.x;
      rect->pt1.y = m_pos.y;
      rect->pt2.x = m_pos.x + m_size.w - 1;
      rect->pt2.y = m_pos.y + m_size.h - 1;
    }

    /**
     * Set the rectangle's left x coordinate.
     *
     * @param x The new x coordinate.
     */

    inline void setX(nxgl_coord_t x)
    {
      m_pos.x = x;
    }

    /**
     * Set the rectangle's top y coordinate.
     *
     * @param y The new y coordinate.
     */

    inline void setY(nxgl_coord_t y)
    {
      m_pos.y = y;
    }

    /**
     * Set the rect's width.
     *
     * @param width The new width.
     */
    inline void setWidth(nxgl_coord_t width)
    {
      m_size.w = width;
    }

    /**
     * Set the rect's height.
     *
     * @param height The new height.
     */

    inline void setHeight(nxgl_coord_t height)
    {
      m_size.h = height;
    }

    /**
     * Get the NX rectangle representation
     *
     * @param rect Pointer to NX rectangle
     */

    inline void setNxRect(FAR const struct nxgl_rect_s *rect)
    {
      m_pos.x = rect->pt1.x;
      m_pos.y = rect->pt1.y;
      m_size.w = rect->pt2.x - rect->pt1.x + 1;
      m_size.h = rect->pt2.y - rect->pt1.y + 1;
    }

    /**
     * Set the x coordinate of the rect's bottom-right corner.  If x2 is less
     * than the rect's current x coordinate the method automatically adjusts
     * the coordinates so that the rect's width is never negative.  Changing this
     * property will change the width of the rect.
     *
     * @param x2 The x coordinate of the rect's bottom-right corner.
     */

    void setX2(nxgl_coord_t x2);

    /**
     * Set the y coordinate of the rect's bottom-right corner.  If y2 is less
     * than the rect's current y coordinate the method automatically adjusts
     * the coordinates so that the rect's height is never negative.  Changing this
     * property will change the height of the rect.
     *
     * @param y2 The y coordinate of the rect's bottom-right corner.
     */

    void setY2(nxgl_coord_t y2);

    /**
     * Get the x coordinate of the rectangle's right side.
     *
     * @return The x coordinate of the rectangle's right side.
     */

    inline nxgl_coord_t getX2(void) const
    {
      return m_pos.x + m_size.w - 1;
    }

    /**
     * Get the y coordinate of the rectangle's bottom side.
     *
     * @return The y coordinate of the rerectangle's bottom side.
     */

    inline nxgl_coord_t getY2(void) const
    {
      return m_pos.y + m_size.h - 1;
    }

    /**
     * Offset the rectangle position by the specified dx, dy values.
     *
     * @param dx X offset value
     * @param dy Y offset value
     */

    inline void offset(nxgl_coord_t dx, nxgl_coord_t dy)
    {
      m_pos.x += dx;
      m_pos.y += dy;
    }

    /**
     * Determines if the rectangle has two dimensions; in other words, does it
     * have both height and width?  Negative width or height is considered not to
     * be valid.
     *
     * @return True if the rect has height and width; false if not.
     */

    inline bool hasDimensions(void) const
    {
      return m_size.w > 0 && m_size.h > 0;
    }

    /**
     * Populates dest with a rectangle representating the intersection
     * of this rectangle and rect.
     *
     * @param rect The rectangle to intersect with this.
     * @param dest The destination rectangle.
     */

    void getIntersect(const CRect& rect, CRect& dest) const;

    /**
     * Populates dest with a rectangle representating the smallest
     * rectangle that contains this rectangle and rect.
     *
     * @param rect The rectangle to add to this.
     * @param dest The destination rectangle.
     */

    void getAddition(const CRect &rect, CRect &dest) const;

    /**
     * Clips this rect to the region that intersects the supplied rect.
     *
     * @param rect CRect to intersect.
     */

    void clipToIntersect(const CRect &rect);

    /**
     * Expands this rect so that it includes the area described by the supplied
     * rect.
     *
     * @param rect CRect to include.
     */

    void expandToInclude(const CRect &rect);

    /**
     * Check if the supplied rect intersects this.
     *
     * @param rect CRect to check for intersection with this.
     * @return True if the rect intersects this; false if not.
     */

    bool intersects(const CRect &rect) const;

    /**
     * Check if the rect contains the supplied point.
     *
     * @param x X coordinate of the point.
     * @param y Y coordinate of the point.
     * @return True if the rect contains the point; false if not.
     */

    bool contains(nxgl_coord_t x, nxgl_coord_t y) const;

    /**
     * Copy the properties of this rect to the destination rect.
     *
     * @param dest Destination rect to copy to.
     */

    void copyTo(CRect &dest) const;

    /**
     * Overloaded & operator.  Returns the intersect of this rectangle and the
     * rectangle passed as the "rect" argument".
     *
     * @param rect The rectangle to intersect with this.
     */

    CRect operator&(const CRect &rect);

    /**
     * Overloaded + operator.  Returns the smallest rectangle that can contain
     * this rectangle and the rectangle passed as the "rect" argument".
     *
     * @param rect The rectangle to add to this.
     */

    CRect operator+(const CRect &rect);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CRECT_HXX
