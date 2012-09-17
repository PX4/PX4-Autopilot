/****************************************************************************
 * NxWidgets/libnxwidgets/src/cbgwindow.cxx
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

#include "crect.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 */

CRect::CRect(void)
{
  m_pos.x  = 0;
  m_pos.y  = 0;
  m_size.w = 0;
  m_size.h = 0;
}

/**
 * Constructor.
 *
 * @param x The x coordinate of the rect.
 * @param y The y coordinate of the rect.
 * @param width The width of the rect.
 * @param height The height of the rect.
 */

CRect::CRect(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t width, nxgl_coord_t height)
{
  m_pos.x  = x;
  m_pos.y  = y;
  m_size.w = width;
  m_size.h = height;
}

/**
 * Constructor.
 *
 * @param rect Pointer to an NX rectangle
 */

CRect::CRect(FAR const nxgl_rect_s *rect)
{
  setNxRect(rect);
}

/**
 * Copy constructor.
 *
 * @param rect CRect to copy.
 */

CRect::CRect(const CRect &rect)
{
  m_pos.x  = rect.getX();
  m_pos.y  = rect.getY();
  m_size.w = rect.getWidth();
  m_size.h = rect.getHeight();
}

/**
 * Create a rect object from the supplied coordinates.
 *
 * @param x1 The x coordinate of the rect's top-left corner.
 * @param y1 The y coordinate of the rect's top-left corner.
 * @param x2 The x coordinate of the rect's bottom-right corner.
 * @param y2 The y coordinate of the rect's bottom-right corner.
 * @return A new rect.
 */

CRect fromCoordinates(nxgl_coord_t x1, nxgl_coord_t y1,
                      nxgl_coord_t x2, nxgl_coord_t y2)
{
  // Ensure x2 is the larger value

  if (x2 < x1)
  {
    nxgl_coord_t swap = x1;
    x1 = x2;
    x2 = swap;
  }
  nxgl_coord_t width = x2 - x1 + 1;

  // Ensure y2 is the larger value

  if (y2 < y1)
  {
    nxgl_coord_t swap = y1;
    y1 = y2;
    y2 = swap;
  }
  nxgl_coord_t height = y2 - y1 + 1;

  // Create the CRect instance

  return CRect(x1, y1, width, height);
}

/**
 * Set the x coordinate of the rect's bottom-right corner.  If x2 is less
 * than the rect's current x coordinate the method automatically adjusts
 * the coordinates so that the rect's width is never negative.  Changing this
 * property will change the width of the rect.
 *
 * @param x2 The x coordinate of the rect's bottom-right corner.
 */
 
void CRect::setX2(nxgl_coord_t x2)
{
  // Ensure that x contains the smaller value

  if (x2 < m_pos.x)
    {
      nxgl_coord_t swap = m_pos.x;
      m_pos.x           = x2;
      x2                = swap;
    }

  // Then set the width

  m_size.w              = x2 - m_pos.x + 1;
}

/**
 * Set the y coordinate of the rect's bottom-right corner.  If y2 is less
 * than the rect's current y coordinate the method automatically adjusts
 * the coordinates so that the rect's height is never negative.  Changing this
 * property will change the height of the rect.
 *
 * @param y2 The y coordinate of the rect's bottom-right corner.
 */

void CRect::setY2(nxgl_coord_t y2)
{
  // Ensure that y contains the smaller value

  if (y2 < m_pos.y)
    {
      nxgl_coord_t swap = m_pos.y;
      m_pos.y           = y2;
      y2                = swap;
    }

  // Then set the height

  m_size.h              = y2 - m_pos.y + 1;
}

/**
 * Populates dest with a rectangle representating the intersection
 * of this rectangle and rect.
 *
 * @param rect The rectangle to intersect with this.
 * @param dest The destination rectangle.
 */

void CRect::getIntersect(const CRect &rect, CRect &dest) const
{
  nxgl_coord_t x1 = ngl_max(m_pos.x, rect.getX());
  nxgl_coord_t y1 = ngl_max(m_pos.y, rect.getY());
  nxgl_coord_t x2 = ngl_min(getX2(), rect.getX2());
  nxgl_coord_t y2 = ngl_min(getY2(), rect.getY2());

  dest.setX(x1);
  dest.setY(y1);
  dest.setWidth(x2 - x1 + 1);
  dest.setHeight(y2 - y1 + 1);
}

/**
 * Populates dest with a rectangle representating the smallest
 * rectangle that contains this rectangle and rect.
 *
 * @param rect The rectangle to add to this.
 * @param dest The destination rectangle.
 */

void CRect::getAddition(const CRect &rect, CRect &dest) const
{

  nxgl_coord_t x1 = ngl_min(m_pos.x, rect.getX());
  nxgl_coord_t y1 = ngl_min(m_pos.y, rect.getY());
  nxgl_coord_t x2 = ngl_max(getX2(), rect.getX2());
  nxgl_coord_t y2 = ngl_max(getY2(), rect.getY2());

  dest.setX(x1);
  dest.setY(y1);
  dest.setWidth(x2 - x1 + 1);
  dest.setHeight(y2 - y1 + 1);
}

/**
 * Clips this rect to the region that intersects the supplied rect.
 *
 * @param rect CRect to intersect.
 */

void CRect::clipToIntersect(const CRect &rect)
{
  CRect clipped;
  getIntersect(rect, clipped);

  setX(clipped.getX());
  setY(clipped.getY());
  setWidth(clipped.getWidth());
  setHeight(clipped.getHeight());
}

/**
 * Expands this rect so that it includes the area described by the supplied
 * rect.
 *
 * @param rect CRect to include.
 */

void CRect::expandToInclude(const CRect& rect)
{
  CRect addition;
  getAddition(rect, addition);

  setX(addition.getX());
  setY(addition.getY());
  setWidth(addition.getWidth());
  setHeight(addition.getHeight());
}

/**
 * Check if the supplied rect intersects this.
 *
 * @param rect CRect to check for intersection with this.
 * @return True if the rect intersects this; false if not.
 */

bool CRect::intersects(const CRect &rect) const
{
  return ((m_pos.x + m_size.w > rect.getX()) &&
          (m_pos.y + m_size.h > rect.getY()) &&
          (m_pos.x            < rect.getX() + rect.getWidth()) &&
          (m_pos.y            < rect.getY() + rect.getHeight()));
}

/**
 * Check if the rect contains the supplied point.
 *
 * @param x X coordinate of the point.
 * @param y Y coordinate of the point.
 * @return True if the rect contains the point; false if not.
 */

bool CRect::contains(nxgl_coord_t x, nxgl_coord_t y) const
{
  return ((x >= m_pos.x)            && (y >= m_pos.y) &&
          (x <  m_pos.x + m_size.w) && (y <  m_pos.y + m_size.h));
}

/**
 * Copy the properties of this rect to the destination rect.
 *
 * @param dest Destination rect to copy to.
 */

void CRect::copyTo(CRect &dest) const
{
  dest.setX(m_pos.x);
  dest.setY(m_pos.y);
  dest.setWidth(m_size.w);
  dest.setHeight(m_size.h);
}

/**
 * Overloaded & operator.  Returns the intersect of this rectangle and the
 * rectangle passed as the "rect" argument".
 *
 * @param rect The rectangle to intersect with this.
 */

CRect CRect::operator&(const CRect &rect)
{
  CRect dest;
  getIntersect(rect, dest);
  return dest;
}

/**
 * Overloaded + operator.  Returns the smallest rectangle that can contain
 * this rectangle and the rectangle passed as the "rect" argument".
 *
 * @param rect The rectangle to add to this.
 */

CRect CRect::operator+(const CRect &rect)
{
  CRect dest;
  getAddition(rect, dest);
  return dest;
}
