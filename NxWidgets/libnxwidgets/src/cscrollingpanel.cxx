/****************************************************************************
 * NxWidgets/libnxwidgets/src/cscrollingpanel.cxx
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

#include "cwidgetcontrol.hxx"
#include "cscrollingpanel.hxx"
#include "cgraphicsport.hxx"
#include "singletons.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param pWidgetControl The widget control for the display.
 * @param x The x coordinate of the widget.
 * @param y The y coordinate of the widget.
 * @param width The width of the widget.
 * @param height The height of the widget.
 * @param flags The usual widget flags.
 * @param style The style that the widget should use.  If this is not
 *   specified, the widget will use the values stored in the global
 *   g_defaultWidgetStyle object.  The widget will copy the properties of
 *   the style into its own internal style object.
 */

CScrollingPanel::CScrollingPanel(CWidgetControl *pWidgetControl,
                                 nxgl_coord_t x, nxgl_coord_t y,
                                 nxgl_coord_t width, nxgl_coord_t height,
                                 uint32_t flags, CWidgetStyle* style)
: CNxWidget(pWidgetControl, x, y, width, height, flags, style)
{
  m_widgetControl = pWidgetControl;

  // NOTE: CScrollingPanel is temporarily borderless because there was no
  // easy way to redraw only the required part of the border.

  m_flags.permeable = true;
  m_flags.borderless = true;

  CRect rect;
  getClientRect(rect);

  m_canvasWidth   = rect.getWidth();
  m_canvasHeight  = rect.getHeight();
  m_canvasX       = 0;
  m_canvasY       = 0;

  setAllowsVerticalScroll(true);
  setAllowsHorizontalScroll(true);
  setContentScrolled(true);
}

/**
 * Scroll the panel by the specified amounts.
 *
 * @param dx The horizontal distance to scroll.
 * @param dy The vertical distance to scroll.
 */

void CScrollingPanel::scroll(int32_t dx, int32_t dy)
{
  CRect rect;
  getClientRect(rect);

  // Prevent scrolling outside boundaries

  if (m_canvasX + dx < -(m_canvasWidth - rect.getWidth()))
    {
      dx = -(m_canvasWidth - rect.getWidth()) - m_canvasX;
    }
  else if (m_canvasX + dx > 0)
    {
      dx = -m_canvasX;
    }

  if (m_canvasY + dy < -(m_canvasHeight - rect.getHeight()))
    {
      dy = -(m_canvasHeight - rect.getHeight()) - m_canvasY;
    }
  else if (m_canvasY + dy > 0)
    {
      dy = -m_canvasY;
    }

  // Prevent scrolling in disallowed planes

  if (!allowsVerticalScroll())
    {
      dy = 0;
    }

  if (!allowsHorizontalScroll())
    {
      dx = 0;
    }

  // Perform scroll if necessary

  if ((dx != 0) || (dy != 0))
    {
      // Only scroll if content scrolling is enabled

      if (m_isContentScrolled)
        {
          // Perform scroll

          TNxArray<CRect> revealedRects;
          CGraphicsPort *port = m_widgetControl->getGraphicsPort();
          port->move(getX(), getY(), dx, dy, rect.getWidth(), rect.getHeight());

          if (dx > 0)
            {
              revealedRects.push_back(CRect(getX(), getY(), dx, rect.getHeight()));
            }
          else if (dx < 0)
            {
              revealedRects.push_back(CRect(getX() + rect.getWidth() + dx, getY(), -dx, rect.getHeight()));
            }

          if (dy > 0)
            {
              revealedRects.push_back(CRect(getX(), getY(), rect.getWidth(), dy));
            }
          else if (dy < 0)
            {
              revealedRects.push_back(CRect(getX(), getY() + rect.getHeight() + dy, rect.getWidth(), -dy));
            }

          // Adjust the scroll values

          m_canvasY += dy;
          m_canvasX += dx;

          // Move children but do not redraw.
          
          scrollChildren(dx, dy, false);
          
          if (revealedRects.size() > 0)
            {
              // Draw background to revealed sections

              for (int i = 0; i < revealedRects.size(); ++i)
                {
                  CRect &rrect = revealedRects[i];

                  gvdbg("Redrawing %d,%d,%d,%d after scroll\n",
                        rrect.getX(), rrect.getY(),
                        rrect.getWidth(), rrect.getHeight());

                  port->drawFilledRect(rrect.getX(), rrect.getY(),
                                       rrect.getWidth(), rrect.getHeight(),
                                       getBackgroundColor());
                  
                  // Check if any children intersect this region.
                  // If it does, it should be redrawn.

                  for (int j = 0; j < m_children.size(); ++j)
                    {
                      CRect crect = m_children[j]->getBoundingBox();
                      if (crect.intersects(rrect))
                        {
                          m_children[j]->redraw();
                        }
                    }
                }
            }
        }
      else
        {
          // Adjust the scroll values

          m_canvasY += dy;
          m_canvasX += dx;
          
          // Scroll all child widgets and redraw them

          scrollChildren(dx, dy, true);
        }

      // Notify event handlers

      m_widgetEventHandlers->raiseScrollEvent(dx, dy);
    }
}

/**
 * Reposition the panel's scrolling region to the specified coordinates.
 *
 * @param x The new x coordinate of the scrolling region.
 * @param y The new y coordinate of the scrolling region.
 */

void CScrollingPanel::jump(int32_t x, int32_t y)
{
  // Calculate difference between jump value and current value and scroll

  scroll(x - m_canvasX, y - m_canvasY);
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CScrollingPanel::drawContents(CGraphicsPort *port)
{
  port->drawFilledRect(getX(), getY(), getWidth(), getHeight(),
                       getBackgroundColor());
}

/**
 * Draw the area of this widget that falls within the clipping region.
 * Called by the redraw() function to draw all visible regions.
 *
 * @param port The CGraphicsPort to draw to.
 * @see redraw()
 */

void CScrollingPanel::drawBorder(CGraphicsPort *port)
{
  // Stop drawing if the widget indicates it should not have an outline

  if (!isBorderless())
    {
      port->drawBevelledRect(getX(), getY(), getWidth(), getHeight(),
                             getShadowEdgeColor(), getShineEdgeColor());
    }
}

/**
 * Scrolls the panel to match the drag.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 * @param vX The horizontal drag distance.
 * @param vY The vertical drag distance.
 */

void CScrollingPanel::onDrag(nxgl_coord_t x, nxgl_coord_t y,
                             nxgl_coord_t vX, nxgl_coord_t vY)
{
  scroll(vX, vY);
}

/**
 * Starts the dragging system.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CScrollingPanel::onClick(nxgl_coord_t x, nxgl_coord_t y)
{
  startDragging(x, y);
}

/**
 * Scroll all child widgets by the specified amounts.  Actually uses
 * the widget's moveTo() function to reposition them.
 *
 * @param dx The horizontal distance to scroll.
 * @param dy The vertical distance to scroll.
 * @param do_redraw Redraw widgets after moving.
 */

void CScrollingPanel::scrollChildren(int32_t dx, int32_t dy, bool do_redraw)
{
  nxgl_coord_t widgetX = 0;
  nxgl_coord_t widgetY = 0;
  nxgl_coord_t thisX   = getX();
  nxgl_coord_t thisY   = getY();
  CNxWidget   *widget  = (CNxWidget *)NULL;

  for (int32_t i = 0; i < m_children.size(); i++)
    {
      widget  = m_children[i];
      bool oldstate = widget->isDrawingEnabled();
      
      if (!do_redraw)
        {
          widget->disableDrawing();
        }
      
      widgetX = (widget->getX() - thisX) + dx;
      widgetY = (widget->getY() - thisY) + dy;
      widget->moveTo(widgetX, widgetY);
      
      if (!do_redraw && oldstate)
        {
          widget->enableDrawing();
        }
    }
}
