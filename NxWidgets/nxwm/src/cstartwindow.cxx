/********************************************************************************************
 * NxWidgets/nxwm/src/cnxconsole.cxx
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
 
#include <nuttx/config.h>

#include "cwidgetcontrol.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "ctaskbar.hxx"
#include "cstartwindow.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CNxConsole Method Implementations
 ********************************************************************************************/

using namespace NxWM;

/**
 * CStartWindow Constructor
 *
 * @param taskbar.  A pointer to the parent task bar instance
 * @param window.  The window to be used by this application.
 */

CStartWindow::CStartWindow(CTaskbar *taskbar, CApplicationWindow *window)
{
  // Save the constructor data

  m_taskbar = taskbar;
  m_window  = window;

  // Add our personalized window label

  NXWidgets::CNxString myName = getName();
  window->setWindowLabel(myName);

  // Add our callbacks to the application window

  window->registerCallbacks(static_cast<IApplicationCallback *>(this));
}

/**
 * CStartWindow Constructor
 */

CStartWindow::~CStartWindow(void)
{
  // Although we didn't create it, we are responsible for deleting the
  // application window

  if (m_window)
    {
      delete m_window;
    }

  // Then stop and delete all applications

  stopAllApplications();
}

/**
 * Each implementation of IApplication must provide a method to recover
 * the contained CApplicationWindow instance.
 */

CApplicationWindow *CStartWindow::getWindow(void) const
{
  return m_window;
}

/**
 * Get the icon associated with the application
 *
 * @return An instance if IBitmap that may be used to rend the
 *   application's icon.  This is an new IBitmap instance that must
 *   be deleted by the caller when it is no long needed.
 */

NXWidgets::IBitmap *CStartWindow::getIcon(void)
{
  NXWidgets::CRlePaletteBitmap *bitmap =
    new NXWidgets::CRlePaletteBitmap(&g_startBitmap);

  return bitmap;
}

/**
 * Get the name string associated with the application
 *
 * @return A copy if CNxString that contains the name of the application.
 */

NXWidgets::CNxString CStartWindow::getName(void)
{
  return NXWidgets::CNxString("Start Window");
}

/**
 * Start the application.
 *
 * @return True if the application was successfully started.
 */

bool CStartWindow::run(void)
{
  // We don't have a thread of execution.  We only respond to button presses

  return true;
}

/**
 * Stop the application.
 */

void CStartWindow::stop(void)
{
  // We don't have a thread of execution.  We only respond to button presses
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but not at the top of the hierarchy
 */

void CStartWindow::hide(void)
{
  // Disable drawing and events on all icons

  for (int i = 0; i < m_slots.size(); i++)
    {
      NXWidgets::CImage *image = m_slots.at(i).image;
      image->disableDrawing();
      image->setRaisesEvents(false);
    }
}

/**
 * Redraw the entire window.  The application has been maximized or
 * otherwise moved to the top of the hiearchy
 */

void CStartWindow::redraw(void)
{
  // Recover the NXTK window instance contained in the application window

  NXWidgets::CNxTkWindow *window = m_window->getWindow();

  // Get the widget control associated with the NXTK window

  NXWidgets::CWidgetControl *control =  window->getWidgetControl();

  // Get the graphics port for drawing on the widget control

  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

  // Get the size of the application window

  struct nxgl_size_s windowSize;
  if (!window->getSize(&windowSize))
    {
      return;
    }

  // Fill the entire window with the background color

  port->drawFilledRect(0, 0, windowSize.w, windowSize.h,
                       CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR);

  // Begin adding icons in the upper left corner

  struct nxgl_point_s iconPos;
  iconPos.x = CONFIG_NXWM_STARTWINDOW_HSPACING;
  iconPos.y = CONFIG_NXWM_STARTWINDOW_VSPACING;

  // Add each icon in the list of applications

  for (int i = 0; i < m_slots.size(); i++)
    {
      // Get the icon associated with this application

      NXWidgets::CImage *image = m_slots.at(i).image;

      // Disable drawing of the icon image; disable events from the icon

      image->disableDrawing();
      image->setRaisesEvents(false);

      // Get the size of the icon image

      NXWidgets::CRect rect;
      image->getPreferredDimensions(rect);

      // Center the image in the region defined by the maximum icon size

      struct nxgl_point_s imagePos;
      imagePos.x = iconPos.x;
      imagePos.y = iconPos.y;

      if (rect.getWidth() < m_iconSize.w)
        {
          imagePos.x += ((m_iconSize.w - rect.getWidth()) >> 1);
        }

      if (rect.getHeight() < m_iconSize.h)
        {
          imagePos.y += ((m_iconSize.h - rect.getHeight()) >> 1);
        }

      // Set the position of the icon bitmap

      image->moveTo(imagePos.x, imagePos.y);

      // Then re-draw the icon at the new position

      image->enableDrawing();
      image->redraw();
      image->setRaisesEvents(true);

      // Advance the next icon position to the left

      iconPos.x +=  m_iconSize.w + CONFIG_NXWM_TASKBAR_HSPACING;

      // If there is insufficient space on on this row for the next
      // max-sized icon, then advance to the next row

      if (iconPos.x + m_iconSize.w >= windowSize.w)
        {
          iconPos.x  = CONFIG_NXWM_STARTWINDOW_HSPACING;
          iconPos.y += m_iconSize.h + CONFIG_NXWM_TASKBAR_VSPACING;

          // Don't try drawing past the bottom of the window

          if (iconPos.y >= windowSize.w)
            {
              break;
            }
        }
    }
}

/**
 * Add the application to the start window.  The general sequence for
 * setting up the start window is:
 *
 * 1. Call CTaskBar::openApplicationWindow to create a window for the start window,
 * 2. Use the window to instantiate CStartWindow
 * 3. Call CStartWindow::addApplication numerous times to install applications
 *    in the start window.
 * 4. Call CTaskBar::startApplication (initially minimized) to start the start
 *    window application.
 *
 * @param app.  The new application to add to the start window
 * @return true on success
 */

bool CStartWindow::addApplication(IApplication *app)
{
  // Recover the NXTK window instance contained in the application window

  NXWidgets::CNxTkWindow *window = m_window->getWindow();

  // Get the widget control associated with the NXTK window

  NXWidgets::CWidgetControl *control =  window->getWidgetControl();

  // Get the bitmap icon that goes with this application

  NXWidgets::IBitmap *bitmap = app->getIcon();

  // Create a CImage instance to manage the application icon

  NXWidgets::CImage *image =
    new NXWidgets::CImage(control, 0, 0, bitmap->getWidth(),
                          bitmap->getHeight(), bitmap, 0);
  if (!image)
    {
      return false;
    }

  // Configure the image, disabling drawing for now

  image->setBorderless(true);
  image->disableDrawing();

  // Register to get events from the mouse clicks on the image

  image->addWidgetEventHandler(this);

  // Add the application to end of the list of applications managed by
  // the start window

  struct SStartWindowSlot slot;
  slot.app   = app;
  slot.image = image;
  m_slots.push_back(slot);

  // Re-calculate the icon bounding box

  getIconBounds();
  return true;
}

/**
 * Called when the window minimize button is pressed.
 */

void CStartWindow::minimize(void)
{
  m_taskbar->minimizeApplication(static_cast<IApplication*>(this));
}

/**
 * Called when the window minimize close is pressed.
 */

void CStartWindow::close(void)
{
  m_taskbar->stopApplication(static_cast<IApplication*>(this));
}

/**
 * Calculate the icon bounding box
 */

void CStartWindow::getIconBounds(void)
{
  // Visit each icon

  struct nxgl_size_s maxSize;
  maxSize.w = 0;
  maxSize.h = 0;

  for (int i = 0; i < m_slots.size(); i++)
    {
      // Get the icon associated with this application

      NXWidgets::CImage *image = m_slots.at(i).image;

      // Get the size of the icon image

      NXWidgets::CRect rect;
      image->getPreferredDimensions(rect);

      // Keep the maximum height and width

      if (rect.getWidth() > maxSize.h)
        {
          maxSize.w = rect.getWidth() ;
        }

      if (rect.getHeight() > maxSize.h)
        {
          maxSize.h = rect.getHeight();
        }
    }

  // And save the new maximum size

  m_iconSize.w = maxSize.w;
  m_iconSize.h = maxSize.h;
}

/**
 * Stop all applications
 */

void CStartWindow::stopAllApplications(void)
{
  // Stop all applications and remove them from the task bar.  Clearly, there 
  // are some ordering issues here... On an orderly system shutdown, disconnection
  // should really occur priority to deleting instances

  while (!m_slots.empty())
    {
      // Stop the application (and remove it from the task bar)

      IApplication *app = m_slots.at(0).app;
      m_taskbar->stopApplication(app);

      // Now, delete the image and the application
 
      delete app;
      delete m_slots.at(0).image;

      // And discard the data in this slot

      m_slots.erase(0);
    }
}

/**
 * Handle a mouse button click event.
 *
 * @param e The event data.
 */

void CStartWindow::handleClickEvent(const NXWidgets::CWidgetEventArgs &e)
{
  // icon was clicked?

  for (int i = 0; i < m_slots.size(); i++)
    {
      // Is this the clicked icon?

      NXWidgets::CImage *image = m_slots.at(i).image;
      if (image->isClicked())
        {
          // Start a new copy of the application

          m_taskbar->startApplication(m_slots.at(i).app, false);

          // Then break out of the loop

          break;
        }
    }
}

