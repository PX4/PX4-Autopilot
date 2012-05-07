/****************************************************************************
 * NxWidgets/libnxwidgets/src/cwidgetcontrol.cxx
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <cstring>
#include <cerrno>

#include "nxconfig.hxx"
#include "cnxserver.hxx"
#include "cnxwidget.hxx"
#include "cnxfont.hxx"
#include "cwidgetstyle.hxx"
#include "cnxtimer.hxx"
#include "cgraphicsport.hxx"
#include "cwidgetcontrol.hxx"
#include "singletons.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Static Data
 ****************************************************************************/

using namespace NXWidgets;

/****************************************************************************
 * Method Implementations
 ****************************************************************************/

/**
 * CWidgetControl constructor (for the case where the display size is known)
 *
 * @param style The style that the widget should use.  If this is not
 * specified, the widget will use the values stored in the global
 * defaultCWidgetStyle object.  The widget will copy the properties of
 * the style into its own internal style object.
 */

CWidgetControl::CWidgetControl(FAR const CWidgetStyle *style)
{
  // Initialize state

  m_port               = (CGraphicsPort *)NULL;
  m_modal              = false;
  m_clickedWidget      = (CNxWidget *)NULL;
  m_focusedWidget      = (CNxWidget *)NULL;

  // Initialize data that we will get from the position callback

  m_hWindow            = NULL;
  m_size.h             = 0;
  m_size.w             = 0;
  m_pos.x              = 0;
  m_pos.y              = 0;
  m_bounds.pt1.x       = 0;
  m_bounds.pt1.y       = 0;
  m_bounds.pt2.y       = 0;
  m_bounds.pt2.y       = 0;

  // Initialize the mouse/touchscreen event and keyboard data structures

  memset(&m_mouse, 0, sizeof(struct SMouse));
  m_nCh                = 0;
  m_nCc                = 0;

  // Intialize semaphores:
  //
  // m_modalSem. The semaphore that will wake up the modal loop on mouse or
  //   keypress events
  // m_geoSem.  The semaphore that will synchronize window size and position
  //   informatin.

  sem_init(&m_modalSem, 0, 0);
#ifdef CONFIG_NX_MULTIUSER
  sem_init(&m_geoSem, 0, 0);
#endif

  // Do we need to fetch the default style?

  if (style == (CWidgetStyle *)NULL)
    {
      // Get the style from the controlling widget

      copyWidgetStyle(&m_style, g_defaultWidgetStyle);
    }
  else
    {
      // Use specified style

      copyWidgetStyle(&m_style, style);
    }
}
  

/**
 * Destructor.
 */
 
CWidgetControl::~CWidgetControl(void)
{
  // Stop any ongoing modal operation

  stopModal();

  // Delete any contained instances

  if (m_port)
    {
      delete m_port;
    }

  // Flush the delete queue

  processDeleteQueue();

  // Delete controlled widget list

  while (m_widgets.size() > 0)
    {
      m_widgets[0]->destroy();
    }
}

/**
 * Run the widget modally.  This will run the CWidgetControl
 * application until stopModal() is called.
 */
 
void CWidgetControl::goModal(void)
{
  // Enter modal

  m_modal = true;

  // Loop until stopModal() is called

  while (m_modal)
    {
      // Process pending events
 
      bool interestingEvent = pollEvents();

      // Did any interesting events occur?

      if (!interestingEvent)
        {
          // No, give up the CPU until something interesting happens.
 
          waitForModalEvent();
        }
    }
}

/**
 * Wait for an interesting modal event to occur (like a mouse or keyboard event)
 */

void CWidgetControl::waitForModalEvent(void)
{
  // It would be an error if this were called outside of a modal loop
 
  if (m_modal)
    {
      // Wait for an interesting event (like a mouse or keyboard event)

      (void)sem_wait(&m_modalSem);
    }
}

/**
 * Wake up the modal loop
 */

void CWidgetControl::wakeupModalLoop(void)
{
  if (m_modal)
    {
      (void)sem_post(&m_modalSem);
    }
}

/**
 * Stop the widget running modally.
 */

void CWidgetControl::stopModal(void)
{
  if (m_modal)
    {
      // No longer modal

      m_modal = false;

      // Wake up the modal loop so that it can terminate properly

      (void)sem_post(&m_modalSem);
    }
}

/**
 * Run all code that needs to take place on a periodic basis.
 * This is normally called from and is the main body of goModal()
 * with widget == NULL.
 *
 * @param widget Sub-widget to run, used for modal widgets; omit
 * this parameter to run the whole system.
 * @return True means some interesting event occurred
 */
 
bool CWidgetControl::pollEvents(CNxWidget *widget)
{
  // Delete any queued widgets

  processDeleteQueue();

  // Handle mouse input

  bool mouseEvent = pollMouseEvents(widget);

  // Handle keyboard input

  bool keyboardEvent = pollKeyboardEvents();

  // Handle cursor control input

  bool cursorControlEvent = pollCursorControlEvents();
  return mouseEvent || keyboardEvent || cursorControlEvent;
}

/**
 * Get the index of the specified controlled widget.
 *
 * @param widget The widget to get the index of.
 * @return The index of the widget.  -1 if the widget is not found.
 */

const int CWidgetControl::getWidgetIndex(const CNxWidget *widget) const
{
  for (int i = 0; i < m_widgets.size(); i++)
    {
      if (m_widgets[i] == widget)
        {
          return i;
        }
    }

  return -1;
}

/**
 * Redraws any dirty regions within the supplied region.
 *
 * @param rect The region to redraw
 */

void CWidgetControl::eraseRect(CRect rect)
{
  // Create pointer to a vector to store the invalid rectangles

  TNxArray<CRect> *invalidRectangles = new TNxArray<CRect>();

  if (invalidRectangles != (TNxArray<CRect> *)NULL)
    {   
      // Add rectangle into the vector

      invalidRectangles->push_back(rect);

      // Refresh children

      for (int i = m_widgets.size() - 1; i > -1 ; i--)
        {
          if (invalidRectangles->size() > 0)
            {
              m_widgets[i]->redrawDirty(invalidRectangles, (CNxWidget *)NULL);
            }
          else
            {
              break;
            }
        }

      // Tidy up

      delete invalidRectangles;
    }
}

/**
 * Remove a controlled widget
 *
 * @param widget The widget to be removed
 */

void CWidgetControl::removeControlledWidget(CNxWidget *widget)
{
  // Locate the widget

  int index = getWidgetIndex(widget);
  if (index >= 0)
    {
      m_widgets.erase(index);
    }
}

/**
 * Add a widget to the list of widgets to be deleted.
 * Must never be called by anything other than the framework itself.
 *
 * @param widget The widget to add to the delete queue.
 */
 
void CWidgetControl::addToDeleteQueue(CNxWidget *widget)
{
  m_deleteQueue.push_back(widget);
}

/**
 * Set the clicked widget pointer.  Note that this should not be
 * called by code other than within the CWidgetControl library itself.
 *
 * @param widget The new clicked widget.
 */

void CWidgetControl::setClickedWidget(CNxWidget *widget)
{
  // Do we have a clicked widget already?

  if (m_clickedWidget != (CNxWidget *)NULL)
    {
      // Ensure that the existing clicked widget is released *outside* its bounds

      m_clickedWidget->release(m_clickedWidget->getX() - 10, 0);
    }
  
  // Update the pointer

  m_clickedWidget = widget;
}

/**
 * Set the focused widget that will receive keyboard input.
 *
 * @param widget The new focused widget.
 */

void CWidgetControl::setFocusedWidget(CNxWidget *widget)
{
  if (m_focusedWidget != widget)
    {
      // This widget will now receive focus

      m_focusedWidget = widget;

      // Make sure that the widget knows that is has focus

      widget->focus();
    }
#warning "Widgets with focus need to go on a stack so that focus can be restored"    
}

/**
 * Set the size of the window.  This is normally reported by an NX callback.  But
 * the toolbar widget control does not get NX callbacks and has to get the 
 * window size throught this method.  This method should not be called by user
 * code
 *
 * @param hWindow The window handle that should be used to communicate
 *        with the window
 * @param bounds. The size of the underlying window.
 */

void CWidgetControl::setWindowBounds(NXHANDLE hWindow, FAR const struct nxgl_rect_s *bounds)
{
  // The first callback is important.  This is the handshake that proves
  // that we are truly communicating with the servier.  This is also
  // a critical point because this is when we know the physical
  // dimensions of the underlying window.

  if (!m_hWindow)
    {
      // Save one-time server specific information
 
      m_hWindow = hWindow;
      nxgl_rectcopy(&m_bounds, bounds);

      // Wake up any threads waiting for initial position information.
      // REVISIT:  If the window is moved or repositioned, then the
      // position and size data will be incorrect for a period of time.
      // That case should be handled here as well.

      giveGeoSem();
    }
}

/**
 * This event is called from CCallback instance to provide
 * notifications of certain NX-server related events. This event,
 * in particular, will occur when the position or size of the underlying
 * window occurs.
 *
 * @param hWindow The window handle that should be used to communicate
 *        with the window
 * @param pos The position of the window in the physical device space.
 * @param size The size of the window.
 * @param bounds The size of the underlying display (pixels x rows)
 */

void CWidgetControl::geometryEvent(NXHANDLE hWindow,
                                   FAR const struct nxgl_size_s *size,
                                   FAR const struct nxgl_point_s *pos,
                                   FAR const struct nxgl_rect_s *bounds)
{
  // Save positional data that may change dynamically

  m_pos.x   = pos->x;
  m_pos.y   = pos->y;
  m_size.h  = size->h;
  m_size.w  = size->w;
  setWindowBounds(hWindow, bounds);
}

/**
 * This event is called from CCallback instance to provide
 * notifications of certain NX-server related events. This event,
 * in particular, will occur when the a portion of the window that was
 * previously obscured is now exposed.
 *
 * @param rect The region in the window that must be redrawn.
 * @param more True means that more re-draw requests will follow
 */

void CWidgetControl::redrawEvent(FAR const struct nxgl_rect_s *nxRect, bool more)
{
  CRect rect;
  rect.setNxRect(nxRect);
  eraseRect(rect);
}

/**
 * This event is called from CCallback instance to provide notifications of
 * certain NX-server related events. This event, in particular, means that
 * new mouse data is available for the window.
 *
 * @param pos The (x,y) position of the mouse.
 * @param buttons See NX_MOUSE_* definitions.
 */
     
void CWidgetControl::newMouseEvent(FAR const struct nxgl_point_s *pos, uint8_t buttons)
{
  // Save the mouse X/Y position

  m_mouse.x = pos->x;
  m_mouse.y = pos->y;

  // Update button press states

  clearMouseEvents();

  if ((buttons & NX_MOUSE_LEFTBUTTON) != 0)
    {
      // Handle left button press events.  leftHeld means that the left mouse
      // button was pressed on the previous sample as well.

      if (m_mouse.leftHeld)
        {
          m_mouse.leftDrag = 1;
        }
      else
        {
          // New left button press

          m_mouse.leftPressed = 1;

          (void)clock_gettime(CLOCK_REALTIME, &m_mouse.leftPressTime);

          // Check for double click event

          if (elapsedTime(&m_mouse.leftReleaseTime) <= CONFIG_NXWIDGETS_DOUBLECLICK_TIME)
            {
              m_mouse.doubleClick = 1;
            }
        }

      m_mouse.leftHeld = 1;
    }
  else
    {
      // Handle left button release events

      if (m_mouse.leftHeld)
        {
          // New left button release

          m_mouse.leftReleased  = 1;
          (void)clock_gettime(CLOCK_REALTIME, &m_mouse.leftReleaseTime);
        }

      m_mouse.leftHeld = 0;
    }

#if 0 // Center and right buttons not used
  if ((buttons & NX_MOUSE_CENTERBUTTON) != 0)
    {
      // Handle center button press events.  centerHeld means that the center mouse
      // button was pressed on the previous sample as well.

      if (m_mouse.centerHeld)
        {
          m_mouse.centerDrag = 1;
        }
      else
        {
          // New center button press

          m_mouse.centerPressed = 1;
        }

      m_mouse.centerHeld = 1;
    }
  else
    {
      // Handle center button release events

      if (m_mouse.centerHeld)
        {
          // New center button release

          m_mouse.centerReleased = 1;
        }

      m_mouse.centerHeld = 0;
    }

  if ((buttons & NX_MOUSE_RIGHTBUTTON) != 0)
    {
      // Handle right button press events.  rightHeld means that the right mouse
      // button was pressed on the previous sample as well.

      if (m_mouse.rightHeld)
        {
          m_mouse.rightDrag = 1;
        }
      else
        {
          // New right button press

          m_mouse.rightPressed = 1;
        }

      m_mouse.rightHeld = 1;
    }
  else
    {
      // Handle right button release events

      if (m_mouse.rightHeld)
        {
          // New right button release

          m_mouse.rightReleased = 1;
        }

      m_mouse.rightHeld = 0;
    }
#endif

  // Then wake up the modal loop

  wakeupModalLoop();
}

/**
 * This event is called from CCallback instance to provide notifications of
 * certain NX-server related events. This event, in particular, means that
 * keyboard/keypad data is available for the window.
 *
 * @param nCh The number of characters that are available in pStr[].
 * @param pStr The array of characters.
 */
    
void CWidgetControl::newKeyboardEvent(uint8_t nCh, FAR const uint8_t *pStr)
{
  FAR uint8_t *pBuffer = &m_kbdbuf[m_nCh];

  // Append each new character to keyboard character buffer

  for (uint8_t i = 0;
       i < nCh && m_nCh < CONFIG_NXWIDGETS_KBDBUFFER_SIZE;
       i++, m_nCh++)
    {
      *pBuffer++ = *pStr++;
    }

  // Then wake up the modal loop

  wakeupModalLoop();
}

/**
 * This event means that cursor control data is available for the window.
 *
 * @param cursorControl The cursor control code received.
 */
    
void CWidgetControl::newCursorControlEvent(ECursorControl cursorControl)
{
  // Append the new cursor control

  if (m_nCc < CONFIG_NXWIDGETS_CURSORCONTROL_SIZE)
    {
      m_controls[m_nCc] = (uint8_t)cursorControl;
      m_nCc++;
    }

  // Then wake up the modal loop

  wakeupModalLoop();
}

/**
 * Copy a widget style
 *
 * @param dest The destination style
 * @param src The source to use
 */

void CWidgetControl::copyWidgetStyle(CWidgetStyle *dest, const CWidgetStyle *src)
{
  dest->colors.background         = src->colors.background;
  dest->colors.selectedBackground = src->colors.selectedBackground;
  dest->colors.shineEdge          = src->colors.shineEdge;
  dest->colors.shadowEdge         = src->colors.shadowEdge;
  dest->colors.highlight          = src->colors.highlight;
  dest->colors.disabledText       = src->colors.disabledText;
  dest->colors.enabledText        = src->colors.enabledText;
  dest->colors.selectedText       = src->colors.selectedText;
  dest->font                      = src->font;
}

/**
 * Return the elapsed time in millisconds
 *
 * @param tp A time in the past from which to compute the elapsed time.
 * @return The elapsed time since tp
 */
 
uint32_t CWidgetControl::elapsedTime(FAR const struct timespec *startTime)
{
  struct timespec endTime;

  (void)clock_gettime(CLOCK_REALTIME, &endTime);
  if (startTime->tv_sec <= endTime.tv_sec)
    {
      // Get the elapsed seconds

      uint32_t seconds = endTime.tv_sec - startTime->tv_sec;

      // Get the elapsed nanoseconds, borrowing from the seconds if necessary
 
      int32_t endNanoSeconds = endTime.tv_nsec;
      if (startTime->tv_nsec > endNanoSeconds)
        {
          // Are there any seconds to borrow from?

          if (seconds < 1)
            {
              // startTime is in the future???

              return 0;
            }

          // Borrow from the seconds

          seconds--;
          endNanoSeconds += 1000000000;
        }
      uint32_t nanoseconds = endNanoSeconds - startTime->tv_nsec;

      // Then return the elapsed time in milliseconds

      return seconds * 1000000 + nanoseconds / 1000;
    }

  // startTime is in the future???

  return 0;
}

/**
 * Pass clicks to the widget hierarchy.  If a single widget
 * is supplied, only that widget is sent the click.  That widget
 * should be running modally.
 *
 * @param x Click xcoordinate.
 * @param y Click ycoordinate.
 * @param widget Pointer to a modally-running widget or NULL.
 */

void CWidgetControl::handleLeftClick(nxgl_coord_t x, nxgl_coord_t y, CNxWidget *widget)
{
  // Working with a modal widget or the whole structure?

  if (widget == (CNxWidget *)NULL)
    {
      // All widgets

      for (int i = m_widgets.size() - 1; i > -1; i--)
        {
          if (m_widgets[i]->click(x, y))
            {
              return;
            }
        }
    }
  else
    {
      // One widget

      (void)widget->click(x, y);
    }
}

/**
 * Delete any widgets in the deletion queue.
 */
 
void CWidgetControl::processDeleteQueue(void)
{
  int i = 0;
  while (i < m_deleteQueue.size())
    {
      m_deleteQueue[i]->destroy();
      i++;
    }

  m_deleteQueue.clear();
}

/**
 * Process mouse/touchscreen events and send throughout the hierarchy.
 *
 * @param widget to process, used for modal widgets; omit this parameter
 *    to run the whole system.
 * @return True means a mouse event occurred
 */

bool CWidgetControl::pollMouseEvents(CNxWidget *widget)
{
  bool mouseEvent = true;  // Assume that an interesting mouse event occurred

  // All widgets

  if (m_mouse.leftPressed)
    {
       // Handle a new left button press event

       handleLeftClick(m_mouse.x, m_mouse.y, widget);
    }
  else if (m_mouse.leftDrag)
    {
      // The left button is still being held down
 
      if (m_clickedWidget != (CNxWidget *)NULL)
        {
          // Handle a mouse drag event

          m_clickedWidget->drag(m_mouse.x, m_mouse.y,
                                m_mouse.x - m_mouse.lastX,
                                m_mouse.y - m_mouse.lastY);
        }
    }
  else if (m_clickedWidget != (CNxWidget *)NULL)
    {
      // Mouse left button release event

      m_clickedWidget->release(m_mouse.x, m_mouse.y);
    }
  else
    {
      // No interesting mouse events

      mouseEvent = false;
    }

  // Clear all press and release events once they have been processed

  clearMouseEvents();

  // Save the mouse position for the next poll

  m_mouse.lastX = m_mouse.x;
  m_mouse.lastY = m_mouse.y;
  return mouseEvent;
}

/**
 * Process keypad events and send throughout the hierarchy.
 *
 * @return True means a keyboard event occurred
 */

bool CWidgetControl::pollKeyboardEvents(void)
{
  bool keyboardEvent = false;  // Assume no interesting keyboard events

  // Keyboard presses with no focused widget is not an interesting
  // event

  if (m_focusedWidget != (CNxWidget *)NULL)
    {
      // Forward each character to the widget with the focus

      for (uint8_t i = 0; i < m_nCh; i++)
        {
          m_focusedWidget->keyPress((nxwidget_char_t)m_kbdbuf[i]);
          keyboardEvent = true;
        }
    }

  // All of the buffered characters have been consumed

  m_nCh = 0;
  return keyboardEvent;
}

/**
 * Process cursor control events and send throughout the hierarchy.
 *
 * @return True means a cursor control event was processes
 */

bool CWidgetControl::pollCursorControlEvents(void)
{
  bool cursorControlEvent = false;  // Assume no interesting cursor events

  // Cursor controls with no focused widget is not an interesting
  // event

  if (m_focusedWidget != (CNxWidget *)NULL)
    {
      // Forward each cursor control to the widget with the focus

      for (uint8_t i = 0; i < m_nCc; i++)
        {
          m_focusedWidget->cursorControl((ECursorControl)m_controls[i]);
          cursorControlEvent = true;
        }
    }

  // All of the buffered events have been consumed

  m_nCc = 0;
  return cursorControlEvent;
}

/**
 * Take the geometry semaphore (handling signal interruptions)
 */

#ifdef CONFIG_NX_MULTIUSER
void CWidgetControl::takeGeoSem(void)
{
  // Take the geometry semaphore.  Retry is an error occurs (only if
  // the error is due to a signal interruption).

  int ret;
  do
    {
      ret = sem_wait(&m_geoSem);
    }
  while (ret < 0 && errno == EINTR);
}
#endif

/**
 * Clear all mouse events
 */

void CWidgetControl::clearMouseEvents(void)
{
  // Clear all press and release events once they have been processed

  m_mouse.leftPressed    = 0;
  m_mouse.leftReleased   = 0;
  m_mouse.leftDrag       = 0;

#if 0 // Center and right buttons are not used
  m_mouse.centerPressed  = 0;
  m_mouse.centerReleased = 0;
  m_mouse.centerDrag     = 0;

  m_mouse.rightPressed   = 0;
  m_mouse.rightReleased  = 0;
  m_mouse.rightDrag      = 0;
#endif

  m_mouse.doubleClick    = 0;
}
