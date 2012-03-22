/****************************************************************************
 * NxWidgets/libnxwidgets/include/cwidgeteventargs.hxx
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

#ifndef __INCLUDE_CWIDGETEVENTARGS_HXX
#define __INCLUDE_CWIDGETEVENTARGS_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "teventargs.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Interesting Key events ***************************************************/

#define KEY_CODE_NONE      ((nxwidget_char_t)0x00) /**< No key (NUL) */
#define KEY_CODE_BACKSPACE ((nxwidget_char_t)0x08) /**< Backspace key */
#define KEY_CODE_DELETE    ((nxwidget_char_t)0x7f) /**< Delete key */
#define KEY_CODE_ENTER     ((nxwidget_char_t)0x0d) /**< Enter key (carriage return) */

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/
 
#if defined(__cplusplus)

namespace NXWidgets
{
  class CNxWidget;

  /**
   *  Cursor control events
   */

  typedef enum
  {
    CURSOR_HOME = 1,  // Set the cursor to the beginning
    CURSOR_END,       // Set the cursor to the end
    CURSOR_UP,        // Set the cursor up one row
    CURSOR_DOWN,      // Set the cursor down one row
    CURSOR_LEFT,      // Move the cursor left one column
    CURSOR_RIGHT,     // Move the cursor right one column
    CURSOR_PAGEUP,    // Move the cursor up one page
    CURSOR_PAGEDOWN,  // Move the cursor down one page
  } ECursorControl;

  /**
   * Event arguments passed to listeners when a CNxWidget object raises an event.
   */

  class CWidgetEventArgs : public TEventArgs<CNxWidget*>
  {
  private:
    nxgl_coord_t    m_x;   /**< X coordinateinate of the event. */
    nxgl_coord_t    m_y;   /**< Y coordinateinate of the event. */
    nxgl_coord_t    m_vX;  /**< X distance moved during event, for dragging. */
    nxgl_coord_t    m_vY;  /**< Y distance moved during event, for dragging. */
    nxwidget_char_t m_key; /**< The key code / cursor code that raised the event. */

  public:

    /**
     * Constructor.
     *
     * @param source Pointer to the CNxWidget object that raised the event.
     * @param x The x coordinate of the event.
     * @param y The y coordinate of the event.
     * @param vX The x distance of the event.
     * @param vY The y distance of the event.
     * @param keyCode The keycode of the event.
     */

    CWidgetEventArgs(CNxWidget *source, const nxgl_coord_t x, const nxgl_coord_t y,
                     const nxgl_coord_t vX, const nxgl_coord_t vY,
                     const nxwidget_char_t key)
                   : TEventArgs<CNxWidget*>(source)
    {
      m_x   = x;
      m_y   = y;
      m_vX  = vX;
      m_vY  = vY;
      m_key = key;
    }

    /**
     * Get the x coordinate of the mouse event. Applies only to the following
     * events: mouse click, mouse double click, mouse drag, mouse release
     *
     * @return The x coordinate of the event.
     */

    inline const nxgl_coord_t getX(void) const
    {
      return m_x;
    }

    /**
     * Get the y coordinate of the mouse event. Applies only to the following
     * events: mouse click, mouse double click, mouse drag, mouse release
     *
     * @return The y coordinate of the event.
     */

    inline const nxgl_coord_t getY(void) const
    {
      return m_y;
    }

    /**
     * Get the x-axis mouse move distance of the mouse event.  Applies
     * only to the mouse drag event.
     *
     * @return The x-axis mouse move distance of the event.
     */

    inline const nxgl_coord_t getVX(void) const
    {
      return m_vX;
    }

    /**
     * Get the y-axis mouse move distance of the event.  Applies
     * only to the mouse drag event.
     *
     * @return The y-axis mouse move distance of the event.
     */

    inline const nxgl_coord_t getVY(void) const
    {
      return m_vY;
    }

    /**
     * Get the key press that generated the event. Applies only to
     * the key press event.
     *
     * @return The key that generated the event.
     */

    inline const nxwidget_char_t getKey(void) const
    {
      return m_key;
    }

    /**
     * Get the cursor control that generated the event.  Applies only
     * to the curso control event.
     *
     * @return The cursor control that generated the event.
     */

    inline const ECursorControl getCursorControl(void) const
    {
      return (ECursorControl)m_key;
    }
  };
}

#endif // __cplusplus

#endif // __INCLUDE_CWIDGETEVENTARGS_HXX
