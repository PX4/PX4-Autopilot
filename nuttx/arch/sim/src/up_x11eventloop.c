/****************************************************************************
 * arch/sim/src/up_x11eventloop.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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

#include <stdio.h>

#include <X11/Xlib.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ***************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern int up_buttonevent(int x, int y, int buttons);

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* Defined in up_x11framebuffer.c */

extern Display *g_display;

volatile int g_eventloop;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ***************************************************************************/

/****************************************************************************
 * Name: up_buttonmap
 ***************************************************************************/

static int up_buttonmap(int state)
{
  /* Remove any X11 dependencies.  Just maps Button1Mask to bit 0. */

  return ((state & Button1Mask) != 0) ? 1 : 0;
}

/****************************************************************************
 * Public Functions
 ***************************************************************************/

/****************************************************************************
 * Name: up_x11events
 *
 * Description:
 *   Called periodically from the IDLE loop to check for queued X11 events.
 *
 ***************************************************************************/

void up_x11events(void)
{
  XEvent event;

  /* Check if there are any pending, queue X11 events. */

  if (XPending(g_display) > 0)
    {
      /* Yes, get the event (this should not block since we know there are
       * pending events)
       */

      XNextEvent(g_display, &event);

      /* Then process the event */

      switch (event.type)
        {
          case MotionNotify : /* Enabled by ButtonMotionMask */
            {
              up_buttonevent(event.xmotion.x, event.xmotion.y,
                             up_buttonmap(event.xmotion.state));
            }
            break;

          case ButtonPress  : /* Enabled by ButtonPressMask */
          case ButtonRelease : /* Enabled by ButtonReleaseMask */
            {
              up_buttonevent(event.xbutton.x, event.xbutton.y,
                             up_buttonmap(event.xbutton.state));
            }
            break;

          default :
            break;
        }
    }
}
