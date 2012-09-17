/****************************************************************************
 * NxWidgets/libnxwidgets/include/cwidgeteventhandlerlist.hxx
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

#ifndef __INCLUDE_CWIDGETEVENTHANDLERLIST_HXX
#define __INCLUDE_CWIDGETEVENTHANDLERLIST_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "cwidgeteventhandler.hxx"
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
  class CNxWidget;
  class CListDataItem;

  /**
   * List of widget event handlers.
   */
  class CWidgetEventHandlerList
  {
  protected:
    TNxArray<CWidgetEventHandler*> m_widgetEventHandlers; /**< List of event handlers */
    CNxWidget *m_widget;                                  /**< Owning widget */
    bool m_isEnabled;                                     /**< Indicates if events are active */

  public:

    /**
     * Constructor.
     *
     * @param widget The owning widget.
     */
 
    CWidgetEventHandlerList(CNxWidget *widget);

    /**
     * Destructor.
     */

    ~CWidgetEventHandlerList(void) { }
    
    /**
     * Check if the object raises events or not.
     *
     * @return True if events are enabled.
     */

    bool isEnabled(void) const;

    /**
     * Get the event handler at the specified index.
     *
     * @param index The index of the event handler.
     * @return The event handler at the specified index.
     */

    inline CWidgetEventHandler *at(int index) const
    {
      return m_widgetEventHandlers.at(index);
    }

    /**
     * Get the size of the array.
     *
     * @return The size of the array.
     */

    inline nxgl_coord_t size(void) const
    {
      return m_widgetEventHandlers.size();
    }

    /**
     * Adds a widget event handler.  The event handler will receive
     * all events raised by this object.
     * @param eventHandler A pointer to the event handler.
     */
 
    void addWidgetEventHandler(CWidgetEventHandler *eventHandler);

    /**
     * Remove a widget event handler.
     *
     * @param eventHandler A pointer to the event handler to remove.
     */

    void removeWidgetEventHandler(CWidgetEventHandler *eventHandler);

    /**
     * Enables event raising.
     */

    inline void enable(void)
    {
      m_isEnabled = true;
    }

    /**
     * Disables event raising.
     */

    inline void disable(void)
    {
      m_isEnabled = false;
    }

    /**
     * Raise a click event to the event handler.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    void raiseClickEvent(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Raise a double-click event to the event handler.
     *
     * @param x The x coordinate of the click.
     * @param y The y coordinate of the click.
     */

    void raiseDoubleClickEvent(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Raise a mouse release event to the event handler.
     *
     * @param x The x coordinate of the release.
     * @param y The y coordinate of the release.
     */

    void raiseReleaseEvent(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Raise a mouse release-outside event to the event handler.
     *
     * @param x The x coordinate of the release.
     * @param y The y coordinate of the release.
     */

    void raiseReleaseOutsideEvent(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Raise a mouse drag event to the event handler.
     *
     * @param x The x coordinate of the mouse when the drag started.
     * @param y The y coordinate of the mouse when the drag started.
     * @param vX The horizontal distance dragged.
     * @param vY The vertical distance dragged.
     */

    void raiseDragEvent(nxgl_coord_t x, nxgl_coord_t y,
                        nxgl_coord_t vX, nxgl_coord_t vY);

    /**
     * Raise a widget drop event to the event handler.
     *
     * @param x The x coordinate of the mouse when the drop occurred.
     * @param y The y coordinate of the mouse when the drop occurred.
     */

    void raiseDropEvent(nxgl_coord_t x, nxgl_coord_t y);

    /**
     * Raise a key press event to the event handler.
     *
     * @param key The character code of the key that caused the event.
     */

    void raiseKeyPressEvent(nxwidget_char_t key);

    /**
     * Raise a cursor control event to the event handler.
     *
     * @param cursorControl The cursor control code that caused the event.
     */

    void raiseCursorControlEvent(ECursorControl cursorControl);

    /**
     * Raise a focus event to the event handler.
     */

    void raiseFocusEvent(void);

    /**
     * Raise a blur event to the event handler.
     */

    void raiseBlurEvent(void);

    /**
     * Raise a close event to the event handler.
     */

    void raiseCloseEvent(void);

    /**
     * Raise a hide event to the event handler.
     */

    void raiseHideEvent(void);

    /**
     * Raise a show event to the event handler.
     */

    void raiseShowEvent(void);

    /**
     * Raise an enable event to the event handler.
     */

    void raiseEnableEvent(void);

    /**
     * Raise a disable event to the event handler.
     */

    void raiseDisableEvent(void);

    /**
     * Raise a value change event to the event handler.
     */

    void raiseValueChangeEvent(void);

    /**
     * Raise a resize event to the event handler.
     *
     * @param width The new width of the widget.
     * @param height The new height of the widget.
     */

    void raiseResizeEvent(nxgl_coord_t width, nxgl_coord_t height);

    /**
     * Raise a move event to the event handler.
     *
     * @param x The new x coordinate of the widget.
     * @param y The new y coordinate of the widget.
     * @param vX The horizontal distance moved.
     * @param vY The vertical distance moved.
     */

    void raiseMoveEvent(nxgl_coord_t x, nxgl_coord_t y,
                        nxgl_coord_t vX, nxgl_coord_t vY);

    /**
     * Raise an action event to the event handler.  This should be called
     * when a widget's purpose has been fulfilled.  For example, in the case
     * of a button, this event is raised when the button is released within
     * its boundaries.  The button has produced a valid click, and thus
     * fulfilled its purpose, so it needs to raise an "action" event.
     */

    void raiseActionEvent(void);

    /**
     * Raises a scroll event.  Fired when the panel scrolls.
     *
     * @param vX Horizontal distance scrolled.
     * @param vY Vertical distance scrolled.
     */

    void raiseScrollEvent(nxgl_coord_t vX, nxgl_coord_t vY);
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CWIDGETEVENTHANDLERLIST_HXX
