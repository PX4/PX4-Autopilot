/****************************************************************************
 * NxWidgets/libnxwidgets/src/crectcache.cxx
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

#include <nuttx/nx/nxglib.h>

#include "crectcache.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * CNxFont Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param widget Widget that contains the rect cache.
 */

CRectCache::CRectCache(const CNxWidget* widget)
{
  m_widget = widget;
  m_foregroundInvalid = true;
  m_backgroundInvalid = true;
}

/**
 * Rebuild the cache if it is invalid.
 */

void CRectCache::cache(void)
{
  cacheBackgroundRegions();
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

void CRectCache::splitRectangles(TNxArray<CRect>* invalidRects,
                                 TNxArray<CRect>* validRects,
                                 FAR const CNxWidget *sender) const
{
  // Check for collisions with any rectangles in the vector

  for (int i = 0; i < invalidRects->size(); i++)
    {
      // Get rectangle to check

      CRect checkRect = invalidRects->at(i);
      nxgl_coord_t splitX[4];
      nxgl_coord_t splitY[4];
      unsigned int rectXCount   = 0;
      unsigned int rectYCount   = 0;
      unsigned int overlapXRect = 0;
      unsigned int overlapYRect = 0;
 
      nxgl_coord_t width  = checkRect.getWidth();
      nxgl_coord_t height = checkRect.getHeight();

      if (m_widget->checkCollision(checkRect.getX(), checkRect.getY(), width, height))
        {
          // Got a collision.  We need to split this rectangle
          // Get clipped dimensions of widget

          CRect widgetRect;
          m_widget->getRectClippedToHierarchy(widgetRect);

          // Vertical split
          // Start at left edge of rectangle

          splitX[0] = checkRect.getX();

          // Check for second split

          if (checkRect.getX() < widgetRect.getX())
            {
              // Widget is to the right of the invalid rectangle (or in the centre)

              if (splitX[rectXCount] != widgetRect.getX())
                {
                  rectXCount++;
                  splitX[rectXCount] = widgetRect.getX();

                  // The next rectangle is the overlap

                  overlapXRect = rectXCount;
                }
            }
          else
            {
              // Widget rectangle is on the left of the invalid rectangle

              if (splitX[rectXCount] != widgetRect.getX2() + 1)
                {
                  // We've found the start of the overlapping rectangle!

                  overlapXRect = rectXCount;
                  rectXCount++;

                  // Split is either the end of the widget or the end of the
                  // invalid rect, whichever comes first

                  if (widgetRect.getX2()  <= checkRect.getX2())
                    {
                      splitX[rectXCount] = widgetRect.getX2() + 1;
                    }
                  else
                    {
                      splitX[rectXCount] = checkRect.getX2() + 1;
                    }
                }
              else
                {
                  // Found the start of the overlapping rectangle

                  overlapXRect = rectXCount;
                }
            }

          // Check for third split

          if (widgetRect.getX2() + 1 <= checkRect.getX2() + 1)
            {
              // Widget ends before the invalid rectangle

              if (splitX[rectXCount] != widgetRect.getX2() + 1)
                {
                  // Record end of overlap

                  rectXCount++;
                  splitX[rectXCount] = widgetRect.getX2() + 1;
                }
            }

          // Store end of invalid rectangle

          if (splitX[rectXCount] <= checkRect.getX2())
            {
              rectXCount++;
              splitX[rectXCount] = checkRect.getX2() + 1;
            }

          // Horizontal split
          // Start at left edge of rectangle

          splitY[0] = checkRect.getY();

          // Check for second split

          if (checkRect.getY() < widgetRect.getY())
            {
              // Widget below the invalid rectangle (or in the centre)

              if (splitY[rectYCount] != widgetRect.getY())
                {
                  rectYCount++;
                  splitY[rectYCount] = widgetRect.getY();

                  // The next rectangle is the overlap

                  overlapYRect = rectYCount;
                }
            }
          else
            {
              // Widget rectangle above the invalid rectangle

              if (splitY[rectYCount] != widgetRect.getY2() + 1)
                {
                  // We've found the start of the overlapping rectangle!

                  overlapYRect = rectYCount;
                  rectYCount++;

                  // Split is either the end of the widget or the end of the
                  // invalid rect, whichever comes first

                  if (widgetRect.getY2() <= checkRect.getY2())
                    {
                      splitY[rectYCount] = widgetRect.getY2() + 1;
                    }
                  else
                    {
                      splitY[rectYCount] = checkRect.getY2() + 1;
                    }
                }
              else
                {
                  // Found the start of the overlapping rectangle

                  overlapYRect = rectYCount;
                }
            }

          // Check for third split

          if (widgetRect.getY2() < checkRect.getY2())
            {
              // Widget ends before the invalid rectangle

              if (splitY[rectYCount] != widgetRect.getY2() + 1)
                {
                  // Record end of overlap

                  rectYCount++;
                  splitY[rectYCount] = widgetRect.getY2() + 1;
                }
            }

          // Store end of invalid rectangle

          if (splitY[rectYCount] <= checkRect.getY2())
            {
              rectYCount++;
              splitY[rectYCount] = checkRect.getY() + 1;
            }

          // Remove the original rectangle

          invalidRects->erase(i);

          // Force the loop to re-examine the new rectangle at this index

          i--;

          // Add the new rectangles (not the overlap; that's the one we need to draw)

          for (unsigned int xRects = 0; xRects < rectXCount; xRects++)
            {
              for (unsigned int yRects = 0; yRects < rectYCount; yRects++)
                {
                  // Is this the overlap?

                  if ((overlapXRect == xRects) && (overlapYRect == yRects))
                    {
                      // Got the overlap, so set the output values

                      CRect overlapRect;
                      overlapRect.setX(splitX[xRects]);
                      overlapRect.setY(splitY[yRects]);
                      overlapRect.setX2(splitX[xRects + 1] - 1);
                      overlapRect.setY2(splitY[yRects + 1] - 1);

                      if (overlapRect.hasDimensions())
                        {
                          validRects->push_back(overlapRect);
                        }
                    }
                  else
                    {
                      // Not an overlap; add to vector

                      CRect newRect;
                      newRect.setX(splitX[xRects]);
                      newRect.setY(splitY[yRects]);
                      newRect.setX2(splitX[xRects + 1] - 1);
                      newRect.setY2(splitY[yRects + 1] - 1);
                      
                     // Insert the new rectangle at the start so we don't

                      if (newRect.hasDimensions())
                        {
                          invalidRects->push_back(newRect);
                        }

                     // Increase iterator to compensate for insertion

                     i++;
                    }
                }
            }
        }
    }
}

/**
 * Move any rectangles from the visibleRects list that overlap this widget
 * into the invisibleRects list.  Used during visible region calculations.
 *
 * @param visibleRects A vector of regions that are not overlapped.
 * @param invisibleRects A vector of regions that are overlapped.
 * @param widget The widget that requested the lists.
 * @see splitRectangles()
 */

void CRectCache::removeOverlappedRects(TNxArray<CRect> *visibleRects,
                                       TNxArray<CRect> *invisibleRects,
                                       FAR const CNxWidget* widget) const
{
  const CNxWidget* parent = m_widget;
  int widgetIndex = -1;

  while ((widget != NULL) && (parent != NULL))
    {
      // Locate widget in the list; we add one to the index to
      // ensure that we deal with the next widget up in the z-order

      widgetIndex = parent->getWidgetIndex(widget) + 1;

      // Widget should never be the bottom item on the screen

      if (widgetIndex > 0)
        {
          // Remove any overlapped rectangles

          for (int i = widgetIndex; i < parent->getChildCount(); i++)
            {
              if (visibleRects->size() > 0)
                {
                  parent->getChild(i)->getCRectCache()->splitRectangles(visibleRects, invisibleRects, widget);
                }
              else
                {
                  break;
                }
            }
        }

      if (visibleRects->size() > 0)
        {
          widget = parent;

          if (parent != NULL)
            {
              parent = parent->getParent();
            }
        }
      else
        {
          return;
        }
    }
}

/**
 * Cache the foreground regions.
 */

void CRectCache::cacheForegroundRegions(void)
{
  if (m_foregroundInvalid)
    {
      // Use internal region cache to store the non-overlapped rectangles
      // We will use this to clip the widget

      m_foregroundRegions.clear();

      // Create pointer to a vector to store the overlapped rectangles
      // We can discard this later as we don't need it

      TNxArray<CRect>* invisibleRects = new TNxArray<CRect>();

      // Copy the clipped widget dimensions into a rect

      CRect rect;
      m_widget->getRectClippedToHierarchy(rect);

      // Do we have a visible region left?

      if (rect.hasDimensions())
        {
          // Add rect to list

          m_foregroundRegions.push_back(rect);
      
          // Request refresh

          if (m_widget->getParent() != NULL)
            {
              m_widget->getParent()->getCRectCache()->removeOverlappedRects(&m_foregroundRegions, invisibleRects, m_widget);
            }
        }

      // Tidy up

      delete invisibleRects;
      m_foregroundInvalid = false;
    }
}

/**
 * Cache the background regions.
 */

void CRectCache::cacheBackgroundRegions(void)
{
  // Ensure that foreground is up to date

  cacheForegroundRegions();

  if (m_backgroundInvalid)
    {
      // Cache visible regions not overlapped by children

      m_backgroundRegions.clear();

      // Create pointer to a vector to store the overlapped rectangles
      // We can discard this later as we don't need it

      TNxArray<CRect>* invisibleRects = new TNxArray<CRect>();

      // Copy all foreground regions into the new vector

      for (int i = 0; i < m_foregroundRegions.size(); i++)
        {
          m_backgroundRegions.push_back(m_foregroundRegions[i]);
        }

      // Remove all child rects from the visible vector

      for (int i = 0; i < m_widget->getChildCount(); i++)
        {
          if (m_backgroundRegions.size() > 0)
            {
              m_widget->getChild(i)->getCRectCache()->splitRectangles(&m_backgroundRegions, invisibleRects, m_widget);
            }
          else
            {
              break;
            }
        }

      // Tidy up

      delete invisibleRects;
      m_backgroundInvalid = false;
    }
}


