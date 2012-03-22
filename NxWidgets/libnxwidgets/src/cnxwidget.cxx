/****************************************************************************
 * NxWidgets/libnxwidgets/src/cnxwidget.cxx
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

#include "cnxwidget.hxx"
#include "cgraphicsport.hxx"
#include "cwidgeteventhandler.hxx"
#include "cnxfont.hxx"
#include "cwidgetstyle.hxx"
#include "cwidgeteventargs.hxx"
#include "crectcache.hxx"
#include "cwidgetcontrol.hxx"
#include "singletons.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define DOUBLE_CLICK_BOUNDS 10

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

using namespace NXWidgets;

/**
 * Constructor.
 *
 * @param pWidgetControl The controllwing widget for the display
 * @param x The x coordinate of the widget.
 * @param y The y coordinate of the widget.
 * @param width The width of the widget.
 * @param height The height of the widget.
 * @param flags Bitmask specifying some set-up values for the widget.
 * @param style The style that the button should use.  If this is not
 *        specified, the button will use the global default widget
 *        style.
 * @see WidgetFlagType.
 */

CNxWidget::CNxWidget(CWidgetControl *pWidgetControl,
                     nxgl_coord_t x, nxgl_coord_t y,
                     nxgl_coord_t width, nxgl_coord_t height,
                     uint32_t flags, const CWidgetStyle *style)
{
  // Save the controlling widget

  m_widgetControl = pWidgetControl;

  // Set properties from parameters.  If this is a child widget, then
  // this coordinates are relative to the parent widget.

  m_rect.setX(x);
  m_rect.setY(y);
  m_rect.setWidth(width);
  m_rect.setHeight(height);

  // Do we need to fetch the default style?

  if (style == (CWidgetStyle *)NULL)
    {
      // Get the style from the controlling widget.  This allows different
      // widgets within a window to have the same style, unique to the window.

      pWidgetControl->getWidgetStyle(&m_style);
    }
  else
    {
      // Use specified style

      useWidgetStyle(style);
    }

  // Add ourself to the list of controlled widgets

  pWidgetControl->addControlledWidget(this);

  // Mask flags against bitmasks and logical NOT twice to obtain boolean values

  m_flags.borderless      = (!(!(flags & WIDGET_BORDERLESS)));
  m_flags.draggable       = (!(!(flags & WIDGET_DRAGGABLE)));
  m_flags.permeable       = (!(!(flags & WIDGET_PERMEABLE)));
  m_flags.doubleClickable = (!(!(flags & WIDGET_DOUBLE_CLICKABLE)));

  // Dragging values

  m_grabPointX          = 0;
  m_grabPointY          = 0;
  m_newX                = 0;
  m_newY                = 0;

  // Set initial flag values

  m_flags.clicked                   = false;
  m_flags.dragging                  = false;
  m_flags.hasFocus                  = false;
  m_flags.deleted                   = false;
  m_flags.drawingEnabled            = false;
  m_flags.enabled                   = true;
  m_flags.erased                    = true;
  m_flags.shelved                   = false;
  m_flags.visibleRegionCacheInvalid = true;
  m_flags.hidden                    = false;
  m_flags.modal                     = false;

  // Set hierarchy pointers

  m_parent              = (CNxWidget *)NULL;
  m_focusedChild        = (CNxWidget *)NULL;

  // Double-click

  clock_gettime(CLOCK_REALTIME, &m_lastClickTime);
  m_lastClickX          = 0;
  m_lastClickY          = 0;
  m_doubleClickBounds   = DOUBLE_CLICK_BOUNDS;

  // Set other default values

  m_refcon              = 0;

  // Set border size to 1 line
  
  m_borderSize.top      = 1;
  m_borderSize.right    = 1;
  m_borderSize.bottom   = 1;
  m_borderSize.left     = 1;
  
  m_closeType           = CLOSE_TYPE_CLOSE;
  m_rectCache           = new CRectCache(this);
  m_widgetEventHandlers = new CWidgetEventHandlerList(this);
}

/**
 * Destructor.
 */

CNxWidget::~CNxWidget(void)
{
  if (!m_flags.deleted)
    {
      m_flags.deleted = true;

      // Unset the clicked pointer if necessary

      if (m_widgetControl->getClickedWidget() == this)
        {
          m_widgetControl->setClickedWidget((CNxWidget *)NULL);
        }
    }

  if (m_parent != (CNxWidget *)NULL)
    {
      m_parent->removeChild(this);
    }

  // Delete children

  while (m_children.size() > 0)
    {
      m_children[0]->destroy();
    }

  // Delete shelved children

  while (m_shelvedWidgets.size() > 0)
    {
      m_shelvedWidgets[0]->destroy();
    }

  // Remove ourselve from the controlled widget list

  m_widgetControl->removeControlledWidget(this);

  // Delete instances.  NOTE that we do not delete the controlling
  // widget.  It persists until the window is closed.

  delete m_rectCache;
  delete m_widgetEventHandlers;
}

/**
 * Get the x coordinate of the widget in "Widget space".
 *
 * @return Widget space x coordinate.
 */

const nxgl_coord_t CNxWidget::getX(void) const
{
  if (m_parent != (CNxWidget *)NULL)
    {
      return m_parent->getX() + m_rect.getX();
    }

  return m_rect.getX();
}

/**
 * Get the y coordinate of the widget in "Widget space".
 *
 * @return Widget space y coordinate.
 */

const nxgl_coord_t CNxWidget::getY(void) const
{
  if (m_parent != (CNxWidget *)NULL)
    {
      return m_parent->getY() + m_rect.getY();
    }

  return m_rect.getY();
}

/**
 * Get the x coordinate of the widget relative to its parent.
 *
 * @return Parent-space x coordinate.
 */

const nxgl_coord_t CNxWidget::getRelativeX(void) const
{
  return m_rect.getX();
}

/**
 * Get the y coordinate of the widget relative to its parent.
 *
 * @return Parent-space y coordinate.
 */

const nxgl_coord_t CNxWidget::getRelativeY(void) const
{
  return m_rect.getY();
}

/**
 * Has the widget been marked for deletion?  This function recurses up the widget
 * hierarchy and only returns true if all of the widgets in the ancestor
 * chain are not deleted.  
 *
 * Widgets marked for deletion are automatically deleted and should not be
 * interacted with.
 *
 * @return True if marked for deletion.
 */

const bool CNxWidget::isDeleted(void) const
{
  if (m_parent != (CNxWidget *)NULL)
    {
      if (m_parent->isDeleted())
        {
          return true;
        }
    }

  return m_flags.deleted;
}

/**
 * Is the widget allowed to draw?  This function recurses up the widget
 * hierarchy and only returns true if all of the widgets in the ancestor
 * chain are visible.
 *
 * @return True if drawing is enabled.
 */

const bool CNxWidget::isDrawingEnabled(void) const
{
  if (m_parent != (CNxWidget *)NULL)
    {
      if (m_parent->isDrawingEnabled())
        {
          // Drawing is enabled if the widget is drawable, not deleted, and not shelved

          return (m_flags.drawingEnabled && (!m_flags.deleted) && (!m_flags.shelved) && (!m_flags.hidden));
        }
    }
  else
    {
      return (m_flags.drawingEnabled && (!m_flags.deleted) && (!m_flags.shelved) && (!m_flags.hidden));
    }

  return false;
}

/**
 * Is the widget hidden?  This function recurses up the widget
 * hierarchy and returns true if any of the widgets in the ancestor
 * chain are hidden.
 *
 * @return True if hidden.
 */

const bool CNxWidget::isHidden(void) const
{
  if (m_parent != (CNxWidget *)NULL)
    {
      if (!m_parent->isHidden())
        {
          // Hidden if the widget is deleted, shelved or hidden

          return (m_flags.deleted || m_flags.shelved || m_flags.hidden);
        }
    }
  else
    {
      return (m_flags.deleted || m_flags.shelved || m_flags.hidden);
    }

  return true;
}

/**
 * Is the widget enabled?  This function recurses up the widget
 * hierarchy and only returns true if all of the widgets in the ancestor
 * chain are enabled.
 *
 * @return True if enabled.
 */

const bool CNxWidget::isEnabled() const
{
  if (m_parent != (CNxWidget *)NULL)
    {
      if (m_parent->isEnabled())
        {
          // Enabled if the widget is enabled, not deleted, not shelved and not hidden

          return (m_flags.enabled && (!m_flags.deleted) && (!m_flags.shelved) && (!m_flags.hidden));
        }
    }
  else
    {
      return (m_flags.enabled && (!m_flags.deleted) && (!m_flags.shelved) && (!m_flags.hidden));
    }

  return false;
}

/**
 * Is the widget modal?  Only true if the Widget singleton is also modal.
 *
 * @return True if the widget is modal.
 */

const bool CNxWidget::isModal(void) const
{
  return m_widgetControl->isModal() & m_flags.modal;
}

/**
 * Insert the dimensions that this widget wants to have into the rect
 * passed in as a parameter.  All coordinates are relative to the widget's
 * parent.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CNxWidget::getPreferredDimensions(CRect &rect) const
{
  rect = m_rect;
}

/**
 * Insert the properties of the space within this widget that is available
 * for children into the rect passed in as a parameter.
 * All coordinates are relative to this widget.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CNxWidget::getClientRect(CRect &rect) const
{
  if (m_flags.borderless)
    {
      rect.setX(0);
      rect.setY(0);
      rect.setWidth(getWidth());
      rect.setHeight(getHeight());
    }
  else
    {
      rect.setX(m_borderSize.left);
      rect.setY(m_borderSize.top);
      rect.setWidth(getWidth() - (m_borderSize.left + m_borderSize.right));
      rect.setHeight(getHeight() - (m_borderSize.top + m_borderSize.bottom));
    }
}

/**
 * Insert the properties of the space within this widget that is
 * available for children into the rect passed in as a parameter.
 * Identical to getClientRect() except that all coordinates are
 * absolute positions within the window.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CNxWidget::getRect(CRect &rect) const
{
  getClientRect(rect);
  rect.setX(rect.getX() + getX());
  rect.setY(rect.getY() + getY());
}

/**
 * Clips the supplied rect to the boundaries defined by this widget and
 * this widget's parents.
 *
 * @param rect Reference to a rect to populate with data.
 */

void CNxWidget::getRectClippedToHierarchy(CRect &rect) const
{
  // Copy the widget's properties into the rect.  NOTE that if this is
  // a child widget, getX() and getY() will return the actual X and Y
  // positions (and not the parent-relative X and Y positions).

  rect.setX(getX());
  rect.setY(getY());
  rect.setWidth(getWidth());
  rect.setHeight(getHeight());

  // And clip it

  clipRectToHierarchy(rect);
}

/**
 * Gets a pointer to the vector of all of the visible regions of this widget,
 * including any covered by children.
 *
 * @return A pointer to a vector of all visible regions.
 */

TNxArray<CRect> *CNxWidget::getForegroundRegions(void)
{
  return m_rectCache->getForegroundRegions();
}

/**
 * Sets this widget's reference constant.  This should be unique,
 * at least amongst this widget's siblings.
 *
 * @param refcon The reference constant.
 */

uint32_t CNxWidget::setRefcon(uint32_t refcon)
{
  uint32_t old = m_refcon;
  m_refcon     = refcon;
  return old;
}

/**
 * Sets this widget's border state.
 *
 * @param isBorderless The border state.
 */

void CNxWidget::setBorderless(bool borderless)
{
  m_flags.borderless = borderless;
  invalidateVisibleRectCache();
}

/**
 * Sets the font.
 *
 * @param font A pointer to the font to use.
 */

void CNxWidget::setFont(CNxFont *font)
{
  m_style.font = font;
}

/**
 * Draws the visible regions of the widget and the widget's child widgets.
 */

void CNxWidget::redraw(void)
{
  if (isDrawingEnabled())
    {
      cacheVisibleRects();

      if (m_rectCache->getBackgroundRegions()->size() > 0)
        {
          // Get the graphics port for drawing in this window

          CGraphicsPort *port = m_widgetControl->getGraphicsPort();

          // Draw all visible rectangles

          for (int i = 0; i < m_rectCache->getBackgroundRegions()->size(); i++)
            {
              drawBorder(port);
              drawContents(port);
            }
        }

      // Remember that the widget is no longer erased

      m_flags.erased = false;
      drawChildren();
    }
}

/**
 * Erases the visible regions of the widget by redrawing the widgets
 * behind it.
 */

void CNxWidget::erase(void)
{
  if (!m_flags.erased)
    {
      cacheVisibleRects();

      if (m_parent != (CNxWidget *)NULL)
        {
          m_parent->eraseWidget(this);
        }

      // Remember that the widget has been erased

      m_flags.erased = true;
      invalidateVisibleRectCache();
    }
}

/**
 * Enables the widget.
 *
 * @return True if the widget was enabled.
 */

bool CNxWidget::enable(void)
{
  if (!m_flags.enabled)
    {
      m_flags.enabled = true;
      onEnable();
      redraw();
      m_widgetEventHandlers->raiseEnableEvent();
      return true;
    }

  return false;
}

/**
 * Disabled the widget.
 *
 * @return True if the widget was disabled.
 */

bool CNxWidget::disable(void)
{
  if (m_flags.enabled)
    {
      m_flags.enabled = false;
      onDisable();
      redraw();
      m_widgetEventHandlers->raiseDisableEvent();
      return true;
    }

  return false;
}

/**
 * Erases the widget, marks it as deleted, and moves it to the CNxWidget
 * deletion queue.  Widgets are automatically deleted by the framework and
 * should not be deleted externally.
 */

void CNxWidget::close(void)
{
  if (!m_flags.deleted)
    {
      m_widgetEventHandlers->raiseCloseEvent();
      m_widgetEventHandlers->disable();

      m_flags.deleted = true;
      m_flags.drawingEnabled = false;
    
      // Unset clicked widget if necessary

      CNxWidget *clickedWidget = m_widgetControl->getClickedWidget();
      if (clickedWidget == this)
        {
          release(clickedWidget->getX(), clickedWidget->getY());
        }

      // Ensure the widget isn't running modally

      stopModal();
      erase();

      if (m_parent != (CNxWidget *)NULL)
        {
          m_parent->closeChild(this);
        }
    }
}

/**
 * Erases the widget, removes it from the main hierarchy and sets it to
 * invisible.  Widgets hidden in this way will be partioned off from
 * other widgets and will no longer be processed.
 *
 * @return True if the widget was shelved.
 * @see unshelve()
 */

bool CNxWidget::shelve(void)
{
  if (!m_flags.shelved)
    {
      m_widgetEventHandlers->raiseShelveEvent();
      m_widgetEventHandlers->disable();

      m_flags.shelved = true;
      m_flags.drawingEnabled = false;

      // Unset clicked widget if necessary

      CNxWidget *clickedWidget = m_widgetControl->getClickedWidget();
      if (clickedWidget == this)
        {
          release(clickedWidget->getX(), clickedWidget->getY());
        }

      // Ensure the widget isn't running modally

      stopModal();
      erase();

      if (m_parent != (CNxWidget *)NULL)
        {
          m_parent->shelveChild(this);
        }
      return true;
    }

  return false;
}

/**
 * Moves the widget back into the hierarchy and redraws it.  Widgets shown
 * in this way will be unpartioned and will be processed again.
 *
 * @return True if the widget was unshelved.
 * @see shelve()
 */

bool CNxWidget::unshelve(void)
{
  if (m_flags.shelved)
    {
      m_widgetEventHandlers->enable();
      m_widgetEventHandlers->raiseUnshelveEvent();

      m_flags.drawingEnabled = true;
      m_flags.shelved = false;

      if (m_parent != (CNxWidget *)NULL)
        {
          m_parent->moveShelvedToChildList(this);
          m_parent->invalidateVisibleRectCache();  
        }

      redraw();
      return true;
    }

  return false;
}

/**
 * Draws the widget and makes it visible.
 * Does not steal focus from other widgets.
 *
 * @return True if the widget was shown.
 * @see hide()
 */

bool CNxWidget::show(void)
{
  if (m_flags.hidden)
    {
      m_flags.hidden = false;

      // Ensure that widgets behind this do not draw over the
      // top of the newly-visible widget

      m_parent->invalidateLowerWidgetsVisibleRectCache(this);
      m_widgetEventHandlers->raiseShowEvent();
      redraw();
      return true;
    }

  return false;
}

/**
 * Erases the widget and makes it invisible.
 * Does not re-assign focus to another widget.
 *
 * @return True if the widget was hidden.
 * @see show()
 */

bool CNxWidget::hide(void)
{
  if (!m_flags.hidden)
    {
      m_flags.hidden = true;

      // Ensure the widget isn't running modally

      stopModal();
      m_widgetEventHandlers->raiseHideEvent();
      erase();
      return true;
    }

  return false;
}

/**
 * Click this widget at the supplied coordinates.  This should only be
 * overridden in subclasses if the default click behaviour needs to be changed.
 * If the subclassed widget should just respond to a standard click,
 * the onClick() method should be overridden instead.
 *
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @return True if the click was successful.
 */

bool CNxWidget::click(nxgl_coord_t x, nxgl_coord_t y)
{
  if (!isEnabled() || !checkCollision(x, y))
    {
      return false;
    }

  // Check for a double-click

  if (isDoubleClick(x, y))
    {
      return doubleClick(x, y);
    }

  // Work out which child was clicked

  for (int i = m_children.size() - 1; i > -1; i--)
    {
      if (m_children[i]->click(x, y))
        {
          return true;
        }
    }

  // Ensure that the click has occurred on a region of this widget
  // not obscured by its siblings

  if (!checkCollisionWithForegroundRects(x, y))
    {
      return false;
    }

  // Handle clicks on this

  m_flags.clicked = true;

  // Record data for double-click

  clock_gettime(CLOCK_REALTIME, &m_lastClickTime);
  m_lastClickX = x;
  m_lastClickY = y;

  // Take focus away from child widgets

  setFocusedWidget((CNxWidget *)NULL);

  // Tell controlling widget that the clicked widget has changed

  m_widgetControl->setClickedWidget(this);

  // Run any code in the inherited class

  onClick(x, y);
  m_widgetEventHandlers->raiseClickEvent(x, y);
  return true;
}

/**
 * Check if the click is a double-click.
 *
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @return True if the click is a double-click.
 */
 
bool CNxWidget::isDoubleClick(nxgl_coord_t x, nxgl_coord_t y)
{
  // Check for a double-click

  if (m_flags.doubleClickable && hasFocus() && m_widgetControl->doubleClick())
    {
      // Within the allowed region?

      if ((m_lastClickX > x - m_doubleClickBounds) && (m_lastClickX < x + m_doubleClickBounds))
        {
          if ((m_lastClickY > y - m_doubleClickBounds) && (m_lastClickY < y + m_doubleClickBounds))
            {
              return true;
            }
        }
    }

  return false;
}

/**
 * Double-click this widget at the supplied coordinates.  This
 * should only be overridden in subclasses if the default
 * double-click behaviour needs to be changed.  If the subclassed
 * widget should just respond to a standard double-click, the
 * onDoubleClick() method should be overridden instead.
 *
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @return True if the click was successful.
 */

bool CNxWidget::doubleClick(nxgl_coord_t x, nxgl_coord_t y)
{
  if (!isEnabled() || !checkCollision(x, y))
    {
      return false;
    }

  // Work out which child was clicked.  Allow the
  // child to determine if it has been double-clicked or not
  // in case the second click has fallen on a different
  // child to the first.

  for (int i = m_children.size() - 1; i > -1; i--)
    {
      if (m_children[i]->click(x, y))
        {
          return true;
        }
    }

  // Ensure that the click has occurred on a region of this widget
  // not obscured by its siblings

  if (!checkCollisionWithForegroundRects(x, y))
    {
      return false;
    }

  m_flags.clicked = true;

  // Record data for double-click

  clock_gettime(CLOCK_REALTIME, &m_lastClickTime);
  m_lastClickX = x;
  m_lastClickY = y;

  // Take focus away from child widgets

  setFocusedWidget((CNxWidget *)NULL);

  // Tell controlling widget that the clicked widget has changed

  m_widgetControl->setClickedWidget(this);

  onDoubleClick(x, y);
  m_widgetEventHandlers->raiseDoubleClickEvent(x, y);
  return true;
}

/**
 * Release this widget at the supplied coordinates.  This should only be
 * overridden in subclasses if the default release behaviour needs to be
 * changed.  If the subclassed widget should just respond to a standard
 * release, the onRelease() method should be overridden instead.
 *
 * @param x X coordinate of the release.
 * @param y Y coordinate of the release.
 * @return True if the release was successful.
 */

bool CNxWidget::release(nxgl_coord_t x, nxgl_coord_t y)
{
  if (!m_flags.clicked)
    {
      return false;
    }

  m_flags.clicked = false;
  stopDragging(x, y);

  if (m_widgetControl->getClickedWidget() == this)
    {
      m_widgetControl->setClickedWidget((CNxWidget *)NULL);
    }

  // Determine which release event to fire

  if (checkCollision(x, y))
    {
      onRelease(x, y);

      // Release occurred within widget; raise release

      m_widgetEventHandlers->raiseReleaseEvent(x, y);
    }
  else
    {
      onReleaseOutside(x, y);

      // Release occurred outside widget; raise release

      m_widgetEventHandlers->raiseReleaseOutsideEvent(x, y);
    }

  return true;
}

/**
 * Drag the widget to the supplied coordinates.
 *
 * @param x The x coordinate of the mouse.
 * @param y The y coordinate of the mouse.
 * @param vX The horizontal distance that the mouse was dragged.
 * @param vY The vertical distance that the mouse was dragged.
 * @return True if the drag was successful.
 */

bool CNxWidget::drag(nxgl_coord_t x, nxgl_coord_t y, nxgl_coord_t vX, nxgl_coord_t vY)
{
  if ((isEnabled()) && (m_flags.dragging))
    {
      if ((vX != 0) || (vY != 0))
        {
          onDrag(x, y, vX, vY);
          m_widgetEventHandlers->raiseDragEvent(x, y, vX, vY);
        }

      return true;
    }

  return false;
}

/**
 * Send a keypress to the widget.
 *
 * @param key The keycode to send to the widget.
 * @return True if the keypress was processed.
 */

bool CNxWidget::keyPress(nxwidget_char_t key)
{
  if (!isEnabled())
    {
      return false;
    }
    
  // Raise keypress for this widget

  m_widgetEventHandlers->raiseKeyPressEvent(key);

  // Handle active child

  if (m_focusedChild != NULL)
    {
      m_focusedChild->keyPress(key);
    }

  return true;
}

/**
 * Send a cursor control event to the widget.
 *
 * @param cursorControl The cursor control code o send to the widget.
 * @return True if the cursor control was processed.
 */

bool CNxWidget::cursorControl(ECursorControl control)
{
  if (!isEnabled())
    {
      return false;
    }
    
  // Raise cursor control for this widget

  m_widgetEventHandlers->raiseCursorControlEvent(control);

  // Handle active child

  if (m_focusedChild != NULL)
    {
      m_focusedChild->cursorControl(control);
    }

  return true;
}

/**
 * Give the widget focus.
 *
 * @return True if the widget received focus correctly.
 */

bool CNxWidget::focus(void)
{
  if (!isEnabled())
    {
      return false;
    }

  // Remember if the widget has focus

  bool hadFocus = m_flags.hasFocus;
  m_flags.hasFocus = true;

  // Notify parent that this widget has focus

  if (m_parent != (CNxWidget *)NULL)
    {
      m_parent->setFocusedWidget(this);
    }

  // Raise an event only if the widget did not have focus

  if (!hadFocus)
    {
      onFocus();
    
      m_widgetEventHandlers->raiseFocusEvent();
      return true;
    }

  return false;
}

/**
 * Remove focus from the widget.
 *
 * @return True if the widget lost focus correctly.
 */

bool CNxWidget::blur(void)
{
  // Remember if the widget had focus

  bool hadFocus = m_flags.hasFocus;

  m_flags.hasFocus = false;

  // Take focus away from child widgets

  if (m_focusedChild != (CNxWidget *)NULL)
    {
      m_focusedChild->blur();
      m_focusedChild = (CNxWidget *)NULL;
      m_widgetControl->clearFocusedWidget(this);
    }

  // Raise an event only if the widget had focus

  if (hadFocus)
    {
      onBlur();
    
      m_widgetEventHandlers->raiseBlurEvent();
      return true;
    }

  return false;
}

/**
 * Move the widget to the new coordinates.
 * Co-ordinates are relative to the parent widget.
 *
 * @param x The new x coordinate.
 * @param y The new y coordinate.
 * @return True if the move was successful.
 */

bool CNxWidget::moveTo(nxgl_coord_t x, nxgl_coord_t y)
{
  // Enforce widget to stay within parent confines if necessary

  if (m_parent != (CNxWidget *)NULL)
    {
      if (!m_parent->isPermeable())
        {
          CRect parentRect;
          m_parent->getClientRect(parentRect);

          // Check x coordinate

          if (x < parentRect.getX())
            {
              x = parentRect.getX();

              // Check width against new value

              if (x + getWidth() > parentRect.getX2() + 1)
                {
                  return false;
                }
            }
          else if (x + getWidth() > parentRect.getX2() + 1)
            {
              x = (parentRect.getX() + parentRect.getX()) - getWidth();

              // Check new x value

              if (x < parentRect.getX())
                {
                  return false;
                }
            }

          // Check y coordinate

          if (y < parentRect.getY())
            {
              y = parentRect.getY();

              // Check height against new value

              if (y + getHeight() > parentRect.getY2() + 1)
                {
                  return false;
                }
            }
          else if (y + getHeight() > parentRect.getY2() + 1)
            {
              y = (parentRect.getY() + parentRect.getY()) - getHeight();

              // Check new y value

              if (y < parentRect.getY())
                {
                  return false;
                }
            }
        }
    }
      
  // Perform move if necessary

  if ((m_rect.getX() != x) || (m_rect.getY() != y))
    {
      erase();

      nxgl_coord_t oldX = m_rect.getX();
      nxgl_coord_t oldY = m_rect.getY();

      m_rect.setX(x);
      m_rect.setY(y);

      redraw();
      m_widgetEventHandlers->raiseMoveEvent(x, y, x - oldX, y - oldY);
      return true;
    }

  return false;
}

/**
 * Resize the widget to the new dimensions.
 *
 * @param width The new width.
 * @param height The new height.
 * @return True if the resize was successful.
 */

bool CNxWidget::resize(nxgl_coord_t width, nxgl_coord_t height)
{
  // Enforce widget to stay within parent confines if necessary

  if (m_parent != (CNxWidget *)NULL)
    {
      if (!m_parent->isPermeable())
        {
          CRect parentRect;
          m_parent->getClientRect(parentRect);

          // Check width

         if (m_rect.getX() + width > parentRect.getX2() + 1)
           {
            width = parentRect.getX2() + 1 - m_rect.getX();
           }

          // Check height

          if (m_rect.getY() + height > parentRect.getY2() + 1)
            {
              height = parentRect.getY2() + 1 - m_rect.getY();
            }
        }
    }

  if (getWidth() != width || getHeight() != height)
    {
      // Remember if the widget is permeable

      bool wasPermeable = m_flags.permeable;

      // Remember if widget was drawing

      bool wasDrawEnabled = m_flags.drawingEnabled;

      m_flags.permeable = true;
      erase();
      disableDrawing();

      m_rect.setWidth(width);
      m_rect.setHeight(height);

      // Handle visible region caching

      if (m_parent != (CNxWidget *)NULL)
        {
          m_parent->invalidateLowerWidgetsVisibleRectCache(this);
        }

      onResize(width, height);

      // Reset the permeable value

      m_flags.permeable = wasPermeable;

      // Reset drawing value

      m_flags.drawingEnabled = wasDrawEnabled;
      redraw();
      m_widgetEventHandlers->raiseResizeEvent(width, height);
      return true;
    }

  return false;
}

/**
 * Resize and move the widget in one operation.
 * Only performs one redraw so it is faster than calling the
 * two separate functions.
 *
 * @param x The new x coordinate.
 * @param y The new y coordinate.
 * @param width The new width.
 * @param height The new height.
 * @return True if the widget was adjusted successfully.
 */

bool CNxWidget::changeDimensions(nxgl_coord_t x, nxgl_coord_t y,
                                 nxgl_coord_t width, nxgl_coord_t height)
{
  bool wasDrawing = m_flags.drawingEnabled;
  m_flags.drawingEnabled = false;
  bool moved = moveTo(x, y);
  m_flags.drawingEnabled = wasDrawing;
  return (resize(width, height) | moved);
}

/**
 * Raises the widget to the top of its parent's widget stack.
 *
 * @return True if the raise was successful.
 */

bool CNxWidget::raiseToTop(void)
{
  if (m_parent != (CNxWidget *)NULL)
    {
      if (m_parent->raiseWidgetToTop(this))
        {
          m_widgetEventHandlers->raiseMoveForwardEvent();
          return true;
        }
    }

  return false;
}

/**
 * Lowers the widget to the bottom of its parent's widget stack.
 *
 * @return True if the lower was successful.
 */

bool CNxWidget::lowerToBottom(void)
{
  if (m_parent != (CNxWidget *)NULL)
    {
      if (m_parent->lowerWidgetToBottom(this))
        {
          m_widgetEventHandlers->raiseMoveBackwardEvent();
          return true;
        }
    }

  return false;
}

/**
 * Raises the supplied widget to the top of this widget's child stack.
 * The supplied widget pointer must be a child of this widget.
 *
 * @param widget A pointer to the child widget to raise.
 * @return True if the raise was successful.
 */

bool CNxWidget::raiseWidgetToTop(CNxWidget *widget)
{
  // Locate widget in the stack

  int index = getWidgetIndex(widget);

  if ((index > -1) && (index < m_children.size() - 1))
    {
      m_children.erase(index);
      m_children.push_back(widget);

      widget->invalidateVisibleRectCache();

      // Invalidate all widgets that collide with the depth-swapped widget

      for (int i = 0; i < m_children.size(); i++)
        {
          if (m_children[i]->checkCollision(widget))
            {
              m_children[i]->invalidateVisibleRectCache();
            }
        }

      widget->redraw();
      return true;
    }

  return false;
}

/**
 * Lowers the supplied widget to the bottom of this widget's child stack.
 * The supplied widget pointer must be a child of this widget.
 *
 * @param widget A pointer to the child widget to lower.
 * @return True if the lower was successful.
 */

bool CNxWidget::lowerWidgetToBottom(CNxWidget *widget)
{
  // Locate widget in the stack

  int index = getWidgetIndex(widget);

  if (index > 0)
    {
      widget->erase();

      // Handle visible region caching

      widget->invalidateVisibleRectCache();
      invalidateLowerWidgetsVisibleRectCache(widget);

      m_children.erase(index);
      m_children.insert(0, widget);

      widget->redraw();
      return true;
    }

  return false;
}

/**
 * Moves the supplied child widget to the deletion queue.
 * For framework use only.
 *
 * @param widget A pointer to the child widget.
 */

void CNxWidget::moveChildToDeleteQueue(CNxWidget *widget)
{
  // Locate widget in main vector

  for (int i = 0; i < m_children.size(); i++)
    {
      if (m_children[i] == widget)
        {
          // Add widget to controlling widget's delete vector

          m_widgetControl->addToDeleteQueue(widget);

          // Remove widget from main vector

          m_children.erase(i);
          break;
        }
    }
}

/**
 * Moves the supplied child widget to the shelved widget list.
 * For framework use only.
 *
 * @param widget A pointer to the child widget.
 * @return True if the widget was moved successfully.
 * @see moveShelvedToChildList()
 * @see hide()
 */

bool CNxWidget::moveChildToShelvedList(CNxWidget *widget)
{
  // Locate widget in main vector

  for (int i = 0; i < m_children.size(); i++)
    {
      if (m_children[i] == widget)
        {
          // Add widget to shelved vector

          m_shelvedWidgets.push_back(widget);

          // Remove widget from main vector

          m_children.erase(i);
          return true;
        }
    }

  return false;
}

/**
 * Moves the supplied child widget from the shelved list back
 * to the child widget list.
 * For framework use only.
 *
 * @param widget A pointer to the shelved widget.
 * @return True if the widget was moved successfully.
 * @see moveChildtoShelvedList()
 * @see show()
 */

bool CNxWidget::moveShelvedToChildList(CNxWidget *widget)
{
  // Locate widget in shelved vector

  for (int i = 0; i < m_shelvedWidgets.size(); i++)
    {
      if (m_shelvedWidgets[i] == widget)
        {
          // Add widget to end of main vector

          m_children.push_back(widget);

          // Remove widget from shelved vector

          m_shelvedWidgets.erase(i);
          return true;
        }
    }

  return false;
}

/**
 * Sets the supplied widget as the focused child.  The widget must
 * be a child of this widget.
 *
 * @param widget A pointer to the child widget.
 * @see getFocusedWidget()
 */

void CNxWidget::setFocusedWidget(CNxWidget *widget)
{
  if (m_focusedChild != widget)
    {
      if (m_focusedChild != NULL)
        {
          // Blur the current active widget

          m_focusedChild->blur();
        }
    }

  // Remember the new active widget

  m_focusedChild = widget;

  // Make this widget active too

  focus();

  // Route keyboard input to the focused widget

  m_widgetControl->setFocusedWidget(this);
}

/**
 * Checks if the supplied coordinates collide with this widget.
 *
 * @param x The x coordinate to check.
 * @param y The y coordinate to check.
 * @return True if a collision occurred.
 */

bool CNxWidget::checkCollision(nxgl_coord_t x, nxgl_coord_t y) const
{
  if (isHidden())
    {
      return false;
    }

  // Get the clipped rect

  CRect rect;
  getRectClippedToHierarchy(rect);
  return rect.contains(x, y);
}

/**
 * Checks if the supplied rectangle definition collides with this widget.
 *
 * @param x The x coordinate of the rectangle to check.
 * @param y The y coordinate of the rectangle to check.
 * @param width The width of the rectangle to check.
 * @param height The height of the rectangle to check.
 * @return True if a collision occurred.
 */

bool CNxWidget::checkCollision(nxgl_coord_t x, nxgl_coord_t y,
                               nxgl_coord_t width, nxgl_coord_t height) const
{
  if (isHidden())
    {
      return false;
    }

  // Get the clipped rect

  CRect rect;
  getRectClippedToHierarchy(rect);
  return rect.intersects(CRect(x, y, width, height));
}

/**
 * Checks if the supplied widget collides with this widget.
 *
 * @param widget A pointer to another widget to check for collisions with.
 * @return True if a collision occurred.
 */

bool CNxWidget::checkCollision(CNxWidget *widget) const
{
  // Get the clipped rect

  CRect rect;
  widget->getRectClippedToHierarchy(rect);
  return rect.intersects(m_rect);
}

/**
 * Invalidate the visible region cache for all widgets below the supplied
 * widget in this widget's child stack.  This will cause those widgets to
 *
 * recalculate their visible regions next time they try to draw themselves.
 * @param widget A pointer to a child widget.
 */

void CNxWidget::invalidateLowerWidgetsVisibleRectCache(CNxWidget *widget)
{
  // Find the widget

  int widgetIndex = getWidgetIndex(widget);  

  // Invalidate lower widgets

  for (int i = widgetIndex - 1; i > -1; i--)
    {
      if (m_children[i]->checkCollision(widget))
        {
          m_children[i]->invalidateVisibleRectCache();
        }
    }
}

/**
 * Adds a widget to this widget's child stack.  The widget is added to the
 * top of the stack.  Note that the widget can only be added if it is not
 * already a child of another widget.
 *
 * @param widget A pointer to the widget to add to the child list.
 * @see insertWidget()
 */

void CNxWidget::addWidget(CNxWidget *widget)
{
  if (widget->getParent() == NULL)
    {
      widget->setParent(this);
      m_children.push_back(widget);

      // Should the widget steal the focus?

      if (widget->hasFocus())
        {
          setFocusedWidget(widget);
        }

      widget->enableDrawing();
      invalidateVisibleRectCache();
      widget->redraw();
    }
}

/**
 * Inserts a widget into this widget's child stack at the bottom of the
 * stack.  Note that the widget can only be added if it is not already
 * a child of another widget.
 *
 * @param widget A pointer to the widget to add to the child list.
 * @see addWidget()
 */

void CNxWidget::insertWidget(CNxWidget *widget)
{
  if (widget->getParent() == NULL)
    {
      widget->setParent(this);
      m_children.insert(0, widget);

      widget->enableDrawing();
      invalidateVisibleRectCache();
      widget->redraw();
    }
}

/**
 * Rebuild the list of this widget's visible regions
 */

void CNxWidget::cacheVisibleRects(void) const
{
  m_rectCache->cache();
}

/**
 * Mark this widget's visible region cache as invalid, and do the same
 * to its child widgets.
 */

void CNxWidget::invalidateVisibleRectCache(void)
{
  m_rectCache->invalidate();

  // Invalidate child cache

  for (int i = 0; i < m_children.size(); i++)
    {
      m_children[i]->invalidateVisibleRectCache();
    }
}

/**
 * Erase a child widget by drawing the widgets behind it.
 *
 * @param widget The child widget to erase.
 */

void CNxWidget::eraseWidget(CNxWidget *widget)
{
  // Locate the widget

  int widgetIndex = getWidgetIndex(widget);

  // Ensure rect cache is up to date

  widget->cacheVisibleRects();

  // Order all lower widgets to redraw themselves based on the erased widget's
  // visible rect cache

  for (int i = widgetIndex - 1; i > -1; i--)
    {
      m_children[i]->redrawDirty(widget->getForegroundRegions(), widget);
    }

  // Order this widget to redraw itself based on any remaining rectangles
  // in the erased widget's rect cache

  redrawDirty(widget->getForegroundRegions(), widget);
  invalidateVisibleRectCache();
}

/**
 * Redraw any visible regions of this widget that have become corrupted.
 *
 * @param invalidRects A list of corrupt regions.
 * @param sender A pointer to the widget that corrupted the regions.
 */

void CNxWidget::redrawDirty(TNxArray<CRect> *invalidRects, CNxWidget *sender)
{
  if (isDrawingEnabled())
    {
      // Draw any children first

      redrawDirtyChildren(invalidRects, sender);

      // Create an array that will contain all of the rects from the
      // original array that overlap this widget

      TNxArray<CRect> *overlappingRects = new TNxArray<CRect>();

      // Remove any non-overlapping rectangles from dirty vector and add to
      // overlapping vector

      m_rectCache->splitRectangles(invalidRects, overlappingRects, sender);

      // Create an array that will contain all of the rects that overlap this
      // widget clipped to its parent

      TNxArray<CRect> *rectsToDraw = new TNxArray<CRect>();
 
      // Split from overlappingRects into rectsToDraw, giving us an array
      // of rects that overlap only the visible portions of this widget

      m_rectCache->splitRectangles(overlappingRects, rectsToDraw, sender);
    
      // Get the graphics port for drawing in this window

      CGraphicsPort *port = m_widgetControl->getGraphicsPort();

      // Draw the dirty rects

      if (rectsToDraw->size() > 0)
        {
          for (int i = 0; i < rectsToDraw->size(); i++)
            {
              drawBorder(port);
              drawContents(port);
            }
        }
    
      // Copy all of the overlapping rects we didn't draw back to the main
      // array of rects that need to be drawn by another widget

      for (int i = 0; i < overlappingRects->size(); i++)
        {
          invalidRects->push_back(overlappingRects->at(i));
        }

     // Clean up

      delete overlappingRects;
      delete rectsToDraw;
    }
}

/**
 * Clips a rectangular region to the dimensions of this widget and its ancestors.
 *
 * @param rect The region that needs to be clipped.
 */

void CNxWidget::clipRectToHierarchy(CRect &rect) const {

  const CNxWidget *parent = m_parent;
  const CNxWidget *widget = this;
  CRect thisRect;

  while (parent != NULL)
    {
      // Standard widgets can draw into client space

      parent->getClientRect(thisRect);

      // Adjust rect to screen space

      thisRect.offset(parent->getX(), parent->getY());
      rect.clipToIntersect(thisRect);

      // Send up to parent

      widget = parent;
      parent = parent->getParent();
    }
}

 /**
  * Swaps the depth of the supplied child widget.
  *
  * @param widget A pointer to the child widget that needs to swap depths.
  * @return True if the swap was successful.
  */

bool CNxWidget::swapWidgetDepth(CNxWidget *widget)
{
  // Can we swap?

  if (m_children.size() > 1)
    {
      int widgetSource = 0;
      int widgetDest = 0;

      // Locate the widget in the vector

      widgetSource = getWidgetIndex(widget);

      // Attempt to raise up

      int i = getHigherVisibleWidget(widgetSource);
      if (i > -1)
        {
          // Raise

          widgetDest = i;
        }
      else
        {
          // Lower to bottom of stack

          widgetDest = 0;
        }
    
      // Erase the widget from the screen

      eraseWidget(widget);

      // Swap

      CNxWidget *tmp = m_children[widgetSource];
      m_children[widgetSource] = m_children[widgetDest];
      m_children[widgetDest] = tmp;

      // Invalidate the widgets below the top affected widget

      if (widgetSource < widgetDest)
        {
          // Source lower; invalidate from dest down

          m_children[widgetDest]->invalidateVisibleRectCache();
          invalidateLowerWidgetsVisibleRectCache(m_children[widgetDest]);
        }
      else
        {
          // Dest lower; invalidate from source down

          m_children[widgetSource]->invalidateVisibleRectCache();
          invalidateLowerWidgetsVisibleRectCache(m_children[widgetSource]);
        }
    
      // Redraw the widget

      widget->redraw();
      return true;
    }

  return false;
}

/**
 * Swap the depth of this widget.
 *
 * @return True if the swap was successful.
 */

bool CNxWidget::swapDepth(void)
{
  if (m_parent != (CNxWidget *)NULL)
    {
      return m_parent->swapWidgetDepth(this);
    }

  return false;
}

/**
 * Remove this widget from the widget hierarchy.  Returns
 * responsibility for deleting the widget back to the developer.
 * Does not unregister the widget from the VBL system.
 * Does not erase the widget from the display.
 *
 * @return True if the widget was successfully removed.
 */

bool CNxWidget::remove(void)
{
  if (m_parent != (CNxWidget *)NULL)
    {
      return m_parent->removeChild(this);
    }

  return false;
}

/**
 * Remove a child widget from the widget hierarchy.  Returns
 * responsibility for deleting the widget back to the developer.
 * Does not unregister the widget from the VBL system.
 * Does not erase the widget from the display.
 *
 * @param widget Pointer to the widget to remove from the hierarchy.
 * @return True if the widget was succesfully removed.
 */

bool CNxWidget::removeChild(CNxWidget *widget)
{
  // Do we need to make another widget active?

  if (m_focusedChild == widget)
    {
      m_focusedChild = (CNxWidget *)NULL;
      m_widgetControl->clearFocusedWidget(this);
    }

  // Unset clicked widget if necessary

  CNxWidget *clickedWidget = m_widgetControl->getClickedWidget();
  if (clickedWidget == widget)
    {
      clickedWidget->release(clickedWidget->getX(), clickedWidget->getY());
    }

  // Divorce child from parent

  widget->setParent((CNxWidget *)NULL);
  widget->disableDrawing();

  // Locate widget in main vector

  for (int i = 0; i < m_children.size(); i++)
    {
      if (m_children[i] == widget)
        {
          // Remove widget from main vector

          m_children.erase(i);
          return true;
        }
    }

  // Try to locate in shelved vector

  for (int i = 0; i < m_shelvedWidgets.size(); i++)
    {
      if (m_shelvedWidgets[i] == widget)
        {
          // Divorce child from parent

          m_shelvedWidgets[i]->setParent((CNxWidget *)NULL);

          // Remove widget from shelved vector

          m_shelvedWidgets.erase(i);
          widget->disableDrawing();
          return true;
        }
    }

  return false;
}

/**
 * Run the widget modally.
 */

void CNxWidget::goModal(void)
{
  // Remember that we're running modally

  m_flags.modal = true;

  // Steal focus

  focus();

  // Loop until no longer modal

  while (isModal())
    {
      // Process pending events
 
      bool interestingEvent = m_widgetControl->pollEvents(this);

      // Did any interesting events occur?

      if (!interestingEvent)
        {
          // No, give up the CPU until something interesting happens.
 
          m_widgetControl->waitForModalEvent();
        }
    }
}

/**
 * Get the index of the specified child widget.
 *
 * @param widget The widget to get the index of.
 * @return The index of the widget.  -1 if the widget is not found.
 */

const int CNxWidget::getWidgetIndex(const CNxWidget *widget) const
{
  for (int i = 0; i < m_children.size(); i++)
    {
      if (m_children[i] == widget)
        {
          return i;
        }
    }

  return -1;
}

/**
 * Get the child widget at the specified index.
 *
 * @param index Index of the child to retrieve.
 * @return Pointer to the child at the specified index.
 */

const CNxWidget *CNxWidget::getChild(const int index) const
{
  if (index < (int)m_children.size())
    {
      return m_children[index];
    }
  return (CNxWidget *)NULL;
}

/**
 * Sets the border size.  The border cannot be drawn over in the
 * drawContents() method.
 *
 * @param borderSize The new border size.
 */

void CNxWidget::setBorderSize(const WidgetBorderSize &borderSize)
{
  m_borderSize.top    = borderSize.top;
  m_borderSize.right  = borderSize.right;
  m_borderSize.bottom = borderSize.bottom;
  m_borderSize.left   = borderSize.left;
}

/**
 * Use the provided widget style
 */

void CNxWidget::useWidgetStyle(const CWidgetStyle *style)
{
  m_style.colors.background         = style->colors.background;
  m_style.colors.selectedBackground = style->colors.selectedBackground;
  m_style.colors.shineEdge          = style->colors.shineEdge;
  m_style.colors.shadowEdge         = style->colors.shadowEdge;
  m_style.colors.highlight          = style->colors.highlight;
  m_style.colors.disabledText       = style->colors.disabledText;
  m_style.colors.enabledText        = style->colors.enabledText;
  m_style.colors.selectedText       = style->colors.selectedText;
  m_style.font                      = style->font;
}

/**
 * Checks if the supplied coordinates collide with a portion of this widget
 * that is not obscured by its siblings, but that may be obscured by
 * its children.
 *
 * @param x X coordinate of the click.
 * @param y Y coordinate of the click.
 * @return True if a collision occurred; false if not.
 */

bool CNxWidget::checkCollisionWithForegroundRects(nxgl_coord_t x, nxgl_coord_t y) const
{
  if (isHidden())
    {
      return false;
    }

  cacheVisibleRects();
  
  CRect *rect;
  for (int i = 0; i < m_rectCache->getForegroundRegions()->size(); ++i)
    {
      rect = &(m_rectCache->getForegroundRegions()->at(i));
      if (rect->contains(x, y))
        {
          return true;
        }
    }

  return false;
}

/**
 * Draw all visible regions of this widget's children.
 */

void CNxWidget::drawChildren(void)
{
  for (int i = 0; i < m_children.size(); i++)
    {
      m_children[i]->redraw();
    }
}

/**
 * Erase and remove the supplied child widget from this widget and
 * send it to the deletion queue.
 *
 * @param widget The widget to close.
 * @see close().
 */

void CNxWidget::closeChild(CNxWidget *widget)
{
  if (widget == NULL)
    {
      return;
    }

  // Ensure widget knows it is being closed

  widget->close();

  // Do we need to make another widget active?

  if (m_focusedChild == widget)
    {
      m_focusedChild = (CNxWidget *)NULL;
      m_widgetControl->clearFocusedWidget(this);

      // Try to choose highest widget

      for (int i = m_children.size() - 1; i > -1; i--)
        {
          if ((m_children[i] != widget) && (!m_children[i]->isHidden()))
            {
              m_focusedChild = m_children[i];
            }
        }

      // Where should the focus go?

      if (m_focusedChild != NULL)
        {
          // Send focus to the new active widget

          m_focusedChild->focus();

          // Route keyboard input to the focused widget

          m_widgetControl->setFocusedWidget(this);
        }
      else
        {
          // Give focus to this

          setFocusedWidget((CNxWidget *)NULL);
        }
    }

  moveChildToDeleteQueue(widget);
}

/**
 * Erase the supplied child widget and move it out of the main child
 * list into the shelved list.  The widget remains in memory and can
 * be restored by calling "unshelve()" on the widget.
 *
 * @param widget The widget to hide.
 */

void CNxWidget::shelveChild(CNxWidget *widget)
{
  if (widget == NULL)
    {
      return;
    }

  // Ensure widget knows it is being shelved

  widget->shelve();

  // Do we need to give another widget focus?

  if (m_focusedChild == widget)
    {
      m_focusedChild = (CNxWidget *)NULL;
      m_widgetControl->clearFocusedWidget(this);

      // Try to choose highest widget

      for (int i = m_children.size() - 1; i > -1; i--)
        {
          if ((m_children[i] != widget) && (!m_children[i]->isHidden()))
            {
              m_focusedChild = m_children[i];
            }
        }

      // Where should the focus go?

      if (m_focusedChild != NULL)
        {
          // Send focus to the new active widget

          m_focusedChild->focus();

          // Route keyboard input to the focused widget

          m_widgetControl->setFocusedWidget(this);
        }
      else
        {
          // Give focus to this

          setFocusedWidget((CNxWidget *)NULL);
        }
    }

  moveChildToShelvedList(widget);
}

    /**
     * Redraws all regions of child widgets that fall within the invalidRects
     * regions.
     *
     * @param invalidRects List of invalid regions that need to be redrawn.
     * @param sender Pointer to the widget that initiated the redraw.
     */

void CNxWidget::redrawDirtyChildren(TNxArray<CRect> *invalidRects, CNxWidget *sender)
{
  for (int i = m_children.size() - 1; i > -1 ; i--)
    {
      if (invalidRects->size() > 0)
        {
          if (m_children.at(i) != sender)
            {
              m_children[i]->redrawDirty(invalidRects, sender);
            }
        }
      else
        {
          break;
        }
    }
}

/**
 * Get the index of the next visible widget higher up the z-order.
 *
 * @param startIndex The starting index.
 * @return The index of the next highest visible widget.
 */

const int CNxWidget::getHigherVisibleWidget(const int startIndex) const
{
  for (int i = startIndex; i < m_children.size(); i++)
    {
      if (!m_children[i]->isHidden())
        {
          return i;
        }
    }

  return -1;
}

/**
 * Get the index of the next visible widget lower down the z-order.
 *
 * @param startIndex The starting index.
 * @return The index of the next lowest visible widget.
 */

const int CNxWidget::getLowerVisibleWidget(const int startIndex) const
{
  for (int i = startIndex; i > -1; i--)
    {
      if (!m_children[i]->isHidden())
        {
          return i;
        }
    }

  return -1;
}

/**
 * Notify this widget that it is being dragged, and set its drag point.
 *
 * @param x The x coordinate of the drag position relative to this widget.
 * @param y The y coordinate of the drag position relative to this widget.
 */

void CNxWidget::startDragging(nxgl_coord_t x, nxgl_coord_t y)
{
  if (m_flags.draggable)
    {
      m_flags.dragging = true;
      m_flags.clicked  = true;
      m_grabPointX     = x - getX();
      m_grabPointY     = y - getY();
      m_newX           = m_rect.getX();
      m_newY           = m_rect.getY();

      onDragStart();
    }
}

/**
 * Notify this widget that it is no longer being dragged.
 */

void CNxWidget::stopDragging(nxgl_coord_t x, nxgl_coord_t y)
{
  if (m_flags.dragging)
    {
      onDragStop();
      m_flags.dragging = false;
      m_widgetEventHandlers->raiseDropEvent(x, y);
    }
}
