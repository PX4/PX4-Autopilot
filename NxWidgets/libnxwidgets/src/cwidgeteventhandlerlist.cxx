/****************************************************************************
 * NxWidgets/libnxwidgets/src/cwidgeteventhandlerlist.cxx
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
#include "cwidgeteventhandler.hxx"
#include "cwidgeteventargs.hxx"

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
 * @param widget The owning widget.
 */
 
CWidgetEventHandlerList::CWidgetEventHandlerList(CNxWidget *widget)
{
  m_widget    = widget;
  m_isEnabled = true;
}

/**
 * Check if the object raises events or not.
 *
 * @return True if events are enabled.
 */

bool CWidgetEventHandlerList::isEnabled(void) const
{
  return m_isEnabled && !m_widget->isDeleted();
}

/**
 * Adds a widget event handler.  The event handler will receive
 * all events raised by this object.
 * @param eventHandler A pointer to the event handler.
 */
 
void CWidgetEventHandlerList::addWidgetEventHandler(CWidgetEventHandler *eventHandler)
{
  // Prevent insertion if the handler already exists

  for (int i = 0; i < m_widgetEventHandlers.size(); i++)
    {
      if (m_widgetEventHandlers.at(i) == eventHandler)
        {
          return;
        }
    }

  // Add the new handler

  m_widgetEventHandlers.push_back(eventHandler);
}

/**
 * Remove a widget event handler.
 *
 * @param eventHandler A pointer to the event handler to remove.
 */

void CWidgetEventHandlerList::removeWidgetEventHandler(CWidgetEventHandler *eventHandler)
{
  for (int i = 0; i < m_widgetEventHandlers.size(); i++)
    {
      if (m_widgetEventHandlers.at(i) == eventHandler)
        {
          m_widgetEventHandlers.erase(i);
          return;
        }
    }
}

/**
 * Raise a click event to the event handler.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CWidgetEventHandlerList::raiseClickEvent(nxgl_coord_t x, nxgl_coord_t y)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleClickEvent(e);
        }
    }
}

/**
 * Raise a double-click event to the event handler.
 *
 * @param x The x coordinate of the click.
 * @param y The y coordinate of the click.
 */

void CWidgetEventHandlerList::raiseDoubleClickEvent(nxgl_coord_t x, nxgl_coord_t y)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleDoubleClickEvent(e);
        }
    }
}

/**
 * Raise a mouse release event to the event handler.
 *
 * @param x The x coordinate of the release.
 * @param y The y coordinate of the release.
 */

void CWidgetEventHandlerList::raiseReleaseEvent(nxgl_coord_t x, nxgl_coord_t y)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleReleaseEvent(e);
        }
    }
}

/**
 * Raise a mouse release-outside event to the event handler.
 *
 * @param x The x coordinate of the release.
 * @param y The y coordinate of the release.
 */

void CWidgetEventHandlerList::raiseReleaseOutsideEvent(nxgl_coord_t x, nxgl_coord_t y)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleReleaseOutsideEvent(e);
        }
    }
}

/**
 * Raise a mouse drag event to the event handler.
 *
 * @param x The x coordinate of the mouse when the drag started.
 * @param y The y coordinate of the mouse when the drag started.
 * @param vX The horizontal distance dragged.
 * @param vY The vertical distance dragged.
 */

void CWidgetEventHandlerList::raiseDragEvent(nxgl_coord_t x, nxgl_coord_t y,
                                             nxgl_coord_t vX, nxgl_coord_t vY)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, vX, vY, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleDragEvent(e);
        }
    }
}

/**
 * Raise a widget drop event to the event handler.
 *
 * @param x The x coordinate of the mouse when the drop occurred.
 * @param y The y coordinate of the mouse when the drop occurred.
 */

void CWidgetEventHandlerList::raiseDropEvent(nxgl_coord_t x, nxgl_coord_t y)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
           m_widgetEventHandlers.at(i)->handleDropEvent(e);
        }
    }
}

/**
 * Raise a key press event to the event handler.
 *
 * @param key The code of the key that caused the event.
 */

void CWidgetEventHandlerList::raiseKeyPressEvent(nxwidget_char_t key)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, key);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleKeyPressEvent(e);
        }
    }
}

/**
 * Raise a cursor control event to the event handler.
 *
 * @param cursorControl The cursor control code that caused the event.
 */

void CWidgetEventHandlerList::raiseCursorControlEvent(ECursorControl cursorControl)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, (nxwidget_char_t)cursorControl);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleCursorControlEvent(e);
        }
    }
}

/**
 * Raise a focus event to the event handler.
 */

void CWidgetEventHandlerList::raiseFocusEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleFocusEvent(e);
        }
    }
}

/**
 * Raise a blur event to the event handler.
 */

void CWidgetEventHandlerList::raiseBlurEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleBlurEvent(e);
        }
    }
}

/**
 * Raise a close event to the event handler.
 */

void CWidgetEventHandlerList::raiseCloseEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleCloseEvent(e);
        }
    }
}

/**
 * Raise a hide event to the event handler.
 */

void CWidgetEventHandlerList::raiseHideEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleHideEvent(e);
        }
    }
}

/**
 * Raise a show event to the event handler.
 */

void CWidgetEventHandlerList::raiseShowEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleShowEvent(e);
        }
    }
}

/**
 * Raise an enable event to the event handler.
 */

void CWidgetEventHandlerList::raiseEnableEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleEnableEvent(e);
        }
    }
}

/**
 * Raise a disable event to the event handler.
 */

void CWidgetEventHandlerList::raiseDisableEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleDisableEvent(e);
        }
    }
}

/**
 * Raise a value change event to the event handler.
 */

void CWidgetEventHandlerList::raiseValueChangeEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleValueChangeEvent(e);
        }
    }
}

/**
 * Raise a resize event to the event handler.
 *
 * @param width The new width of the widget.
 * @param height The new height of the widget.
 */

void CWidgetEventHandlerList::raiseResizeEvent(nxgl_coord_t width, nxgl_coord_t height)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleResizeEvent(e);
        }
    }
}

/**
 * Raise a move event to the event handler.
 *
 * @param x The new x coordinate of the widget.
 * @param y The new y coordinate of the widget.
 * @param vX The horizontal distance moved.
 * @param vY The vertical distance moved.
 */

void CWidgetEventHandlerList::raiseMoveEvent(nxgl_coord_t x, nxgl_coord_t y,
                                             nxgl_coord_t vX, nxgl_coord_t vY)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, x, y, vX, vY, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleMoveEvent(e);
        }
    }
}

/**
 * Raise an action event to the event handler.  This should be called
 * when a widget's purpose has been fulfilled.  For example, in the case
 * of a button, this event is raised when the button is released within
 * its boundaries.  The button has produced a valid click, and thus
 * fulfilled its purpose, so it needs to raise an "action" event.
 */

void CWidgetEventHandlerList::raiseActionEvent(void)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, 0, 0, KEY_CODE_NONE);

      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleActionEvent(e);
        }
    }
}

/**
 * Raises a scroll event.  Fired when the panel scrolls.
 *
 * @param vX Horizontal distance scrolled.
 * @param vY Vertical distance scrolled.
 */

void CWidgetEventHandlerList::raiseScrollEvent(nxgl_coord_t vX, nxgl_coord_t vY)
{
  if (isEnabled())
    {
      CWidgetEventArgs e(m_widget, 0, 0, vX, vY, KEY_CODE_NONE);
  
      for (int i = 0; i < m_widgetEventHandlers.size(); i++)
        {
          m_widgetEventHandlers.at(i)->handleScrollEvent(e);
        }
    }
}
