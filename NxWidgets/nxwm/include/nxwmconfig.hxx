/****************************************************************************
 * NxWidgets/nxwm/include/nxwmconfig.hxx
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

#ifndef __INCLUDE_NXWMCONFIG_HXX
#define __INCLUDE_NXWMCONFIG_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <nuttx/input/touchscreen.h>

#include "nxconfig.hxx"
#include "crlepalettebitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* General Configuration ****************************************************/
/**
 * Required settings:
 *
 * CONFIG_HAVE_CXX        : C++ support is required
 * CONFIG_NX              : NX must enabled
 * CONFIG_NX_MULTIUSER=y  : NX must be configured in multiuse mode
 * CONFIG_NXCONSOLE=y     : For NxConsole support
 * CONFIG_SCHED_ONEXIT    : Support for on_exit()
 *
 * General settings:
 *
 * CONFIG_NXWM_DEFAULT_FONTID - the NxWM default font ID. Default:
 *   NXFONT_DEFAULT
 * CONFIG_NXWM_TOUCHSCREEN - Define to build in touchscreen support.
 */

#ifndef CONFIG_HAVE_CXX
#  error "C++ support is required (CONFIG_HAVE_CXX)"
#endif

/**
 * NX Multi-user support is required
 */

#ifndef CONFIG_NX
#  error "NX support is required (CONFIG_NX)"
#endif

#ifndef CONFIG_NX_MULTIUSER
#  error "NX multi-user support is required (CONFIG_NX_MULTIUSER)"
#endif

/**
 * NxConsole support is (probably) required
 */

#ifndef CONFIG_NXCONSOLE
#  warning "NxConsole support may be needed (CONFIG_NXCONSOLE)"
#endif

/**
 * on_exit() support is (probably) required.  on_exit() is the normal
 * mechanism used by NxWM applications to clean-up on a application task
 * exit.
 */

#ifndef CONFIG_SCHED_ONEXIT
#  warning "on_exit() support may be needed (CONFIG_SCHED_ONEXIT)"
#endif

/**
 * Default font ID
 */

#ifndef CONFIG_NXWM_DEFAULT_FONTID
#  define CONFIG_NXWM_DEFAULT_FONTID NXFONT_DEFAULT
#endif

/* Colors *******************************************************************/
/**
 * Color configuration
 *
 * CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR - Normal background color.  Default:
 *    MKRGB(148,189,215)
 * CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR - Select background color. 
 *    Default:  MKRGB(206,227,241)
 * CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR - Color of the bright edge of a border.
 *    Default: MKRGB(255,255,255)
 * CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR - Color of the shadowed edge of a border.
 *    Default: MKRGB(0,0,0)
 * CONFIG_NXWM_DEFAULT_FONTCOLOR - Default fong color.  Default:
 *    MKRGB(0,0,0)
 * CONFIG_NXWM_TRANSPARENT_COLOR - The "transparent" color.  Default:
 *    MKRGB(0,0,0)
 */

/**
 * Normal background color
 */

#ifndef CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR
#  define CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR  MKRGB(148,189,215)
#endif

/**
 * Default selected background color
 */

#ifndef CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR
#  define CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR  MKRGB(206,227,241)
#endif

/**
 * Border colors
 */

#ifndef CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR
#  define CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR  MKRGB(248,248,248)
#endif

#ifndef CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR
#  define CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR  MKRGB(35,58,73)
#endif

/**
 * The default font color
 */

#ifndef CONFIG_NXWM_DEFAULT_FONTCOLOR
#  define CONFIG_NXWM_DEFAULT_FONTCOLOR  MKRGB(255,255,255)
#endif

/**
 * The transparent color
 */

#ifndef CONFIG_NXWM_TRANSPARENT_COLOR
#  define CONFIG_NXWM_TRANSPARENT_COLOR  MKRGB(0,0,0)
#endif

/* Task Bar Configuation  ***************************************************/
/**
 * Horizontal and vertical spacing of icons in the task bar.
 *
 * CONFIG_NXWM_TASKBAR_VSPACING - Vertical spacing.  Default: 2 pixels
 * CONFIG_NXWM_TASKBAR_HSPACING - Horizontal spacing.  Default: 2 rows
 *
 * Task bar location.  Default is CONFIG_NXWM_TASKBAR_TOP.
 *
 * CONFIG_NXWM_TASKBAR_TOP - Task bar is at the top of the display
 * CONFIG_NXWM_TASKBAR_BOTTOM - Task bar is at the bottom of the display
 * CONFIG_NXWM_TASKBAR_LEFT - Task bar is on the left side of the display
 * CONFIG_NXWM_TASKBAR_RIGHT - Task bar is on the right side of the display
 *
 * CONFIG_NXWM_TASKBAR_WIDTH - Task bar thickness (either vertical or
 *   horizontal).  Default: 25 + 2*spacing
 */

/**
 * Horizontal and vertical spacing of icons in the task bar.
 */

#ifndef CONFIG_NXWM_TASKBAR_VSPACING
#  define CONFIG_NXWM_TASKBAR_VSPACING (2)
#endif

#ifndef CONFIG_NXWM_TASKBAR_HSPACING
#  define CONFIG_NXWM_TASKBAR_HSPACING (2)
#endif

/**
 * Check task bar location
 */

#if defined(CONFIG_NXWM_TASKBAR_TOP)
#  if defined(CONFIG_NXWM_TASKBAR_BOTTOM) || defined (CONFIG_NXWM_TASKBAR_LEFT) || defined (CONFIG_NXWM_TASKBAR_RIGHT)
#    warning "Multiple task bar positions specified"
#  endif
#elif defined(CONFIG_NXWM_TASKBAR_BOTTOM)
#  if defined (CONFIG_NXWM_TASKBAR_LEFT) || defined (CONFIG_NXWM_TASKBAR_RIGHT)
#    warning "Multiple task bar positions specified"
#  endif
#elif defined(CONFIG_NXWM_TASKBAR_LEFT)
#  if defined (CONFIG_NXWM_TASKBAR_RIGHT)
#    warning "Multiple task bar positions specified"
#  endif
#elif !defined(CONFIG_NXWM_TASKBAR_RIGHT)
#  warning "No task bar position specified"
#  define CONFIG_NXWM_TASKBAR_TOP 1
#endif

/**
 * At present, all icons are 25 pixels in "width" and, hence require a
 * task bar of at least that size.
 */

#ifndef CONFIG_NXWM_TASKBAR_WIDTH
#  if defined(CONFIG_NXWM_TASKBAR_TOP) || defined(CONFIG_NXWM_TASKBAR_BOTTOM)
#    define CONFIG_NXWM_TASKBAR_WIDTH (25+2*CONFIG_NXWM_TASKBAR_HSPACING)
#  else
#    define CONFIG_NXWM_TASKBAR_WIDTH (25+2*CONFIG_NXWM_TASKBAR_VSPACING)
#  endif
#endif

/* Tool Bar Configuration ***************************************************/
/**
 * CONFIG_NXWM_TOOLBAR_HEIGHT.  The height of the tool bar in each
 *   application window. At present, all icons are 21 pixels in height and,
 *   hence require a task bar of at least that size.
 */

#ifndef CONFIG_NXWM_TOOLBAR_HEIGHT
#    define CONFIG_NXWM_TOOLBAR_HEIGHT (21+2*CONFIG_NXWM_TASKBAR_HSPACING)
#endif

/* Background Image **********************************************************/
/**
 * CONFIG_NXWM_BACKGROUND_IMAGE - The name of the image to use in the
 *   background window.  Default:NXWidgets::g_nuttxBitmap
 */

#ifndef CONFIG_NXWM_BACKGROUND_IMAGE
#  define CONFIG_NXWM_BACKGROUND_IMAGE NXWidgets::g_nuttxBitmap
#endif

/* Start Window Configuration ***********************************************/
/**
 * Horizontal and vertical spacing of icons in the task bar.
 *
 * CONFIG_NXWM_STARTWINDOW_VSPACING - Vertical spacing.  Default: 2 pixels
 * CONFIG_NXWM_STARTWINDOW_HSPACING - Horizontal spacing.  Default: 2 rows
 * CONFIG_NXWM_STARTWINDOW_ICON - The glyph to use as the start window icon
 * CONFIG_NXWM_STARTWINDOW_MQNAME - The well known name of the message queue
 *   Used to communicated from CWindowControl tothe start window thread. 
 *   Default: "/dev/nxwm"
 * CONFIG_NXWM_STARTWINDOW_MXMSGS - The maximum number of messages to queue
 *   before blocking.  Defualt 32
 * CONFIG_NXWM_STARTWINDOW_MXMPRIO - The message priority. Default: 42.
 * CONFIG_NXWM_STARTWINDOW_PRIO - Priority of the NxConsole task.  Default:
 *   SCHED_PRIORITY_DEFAULT.  NOTE:  This priority should be less than
 *   CONFIG_NXWIDGETS_SERVERPRIO or else there may be data overrun errors.
 *   Such errors would most likely appear as duplicated rows of data on the
 *   display.
 * CONFIG_NXWM_STARTWINDOW_STACKSIZE - The stack size to use when starting the
 *   NxConsole task.  Default: 2048 bytes.
 */

#ifndef CONFIG_NXWM_STARTWINDOW_VSPACING
#  define CONFIG_NXWM_STARTWINDOW_VSPACING (2)
#endif

#ifndef CONFIG_NXWM_STARTWINDOW_HSPACING
#  define CONFIG_NXWM_STARTWINDOW_HSPACING (2)
#endif

/**
 * The start window glyph
 */

#ifndef CONFIG_NXWM_STARTWINDOW_ICON
#  define CONFIG_NXWM_STARTWINDOW_ICON NxWM::g_playBitmap
#endif

/**
 * Start window task parameters
 */

#ifndef CONFIG_NXWM_STARTWINDOW_MQNAME
#  define CONFIG_NXWM_STARTWINDOW_MQNAME  "/dev/nxwm"
#endif

#ifndef CONFIG_NXWM_STARTWINDOW_MXMSGS
#  ifdef CONFIG_NX_MXCLIENTMSGS
#    define CONFIG_NXWM_STARTWINDOW_MXMSGS CONFIG_NX_MXCLIENTMSGS
#  else
#    define CONFIG_NXWM_STARTWINDOW_MXMSGS 32
#  endif
#endif

#ifndef CONFIG_NXWM_STARTWINDOW_MXMPRIO
#  define CONFIG_NXWM_STARTWINDOW_MXMPRIO 42
#endif

#ifndef CONFIG_NXWM_STARTWINDOW_PRIO
#  define CONFIG_NXWM_STARTWINDOW_PRIO  SCHED_PRIORITY_DEFAULT
#endif

#if CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWM_STARTWINDOW_PRIO
#  warning "CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWM_STARTWINDOW_PRIO"
#  warning" -- This can result in data overrun errors"
#endif

#ifndef CONFIG_NXWM_STARTWINDOW_STACKSIZE
#  define CONFIG_NXWM_STARTWINDOW_STACKSIZE  2048
#endif

/* NxConsole Window *********************************************************/
/**
 * NxConsole Window Configuration
 *
 * CONFIG_NXWM_NXCONSOLE_PRIO - Priority of the NxConsole task.  Default:
 *   SCHED_PRIORITY_DEFAULT.  NOTE:  This priority should be less than
 *   CONFIG_NXWIDGETS_SERVERPRIO or else there may be data overrun errors.
 *   Such errors would most likely appear as duplicated rows of data on the
 *   display.
 * CONFIG_NXWM_NXCONSOLE_STACKSIZE - The stack size to use when starting the
 *   NxConsole task.  Default: 2048 bytes.
 * CONFIG_NXWM_NXCONSOLE_WCOLOR - The color of the NxConsole window background.
 *   Default:  MKRGB(192,192,192)
 * CONFIG_NXWM_NXCONSOLE_FONTCOLOR - The color of the fonts to use in the
 *   NxConsole window.  Default: MKRGB(0,0,0)
 * CONFIG_NXWM_NXCONSOLE_FONTID - The ID of the font to use in the NxConsole
 *   window.  Default: CONFIG_NXWM_DEFAULT_FONTID
 * CONFIG_NXWM_NXCONSOLE_ICON - The glyph to use as the NxConsole icon
 */

#ifndef CONFIG_NXWM_NXCONSOLE_PRIO
#  define CONFIG_NXWM_NXCONSOLE_PRIO  SCHED_PRIORITY_DEFAULT
#endif

#if CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWM_NXCONSOLE_PRIO
#  warning "CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWM_NXCONSOLE_PRIO"
#  warning" -- This can result in data overrun errors"
#endif

#ifndef CONFIG_NXWM_NXCONSOLE_STACKSIZE
#  define CONFIG_NXWM_NXCONSOLE_STACKSIZE  2048
#endif

#ifndef CONFIG_NXWM_NXCONSOLE_WCOLOR
#  define CONFIG_NXWM_NXCONSOLE_WCOLOR  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR
#endif

#ifndef CONFIG_NXWM_NXCONSOLE_FONTCOLOR
#  define CONFIG_NXWM_NXCONSOLE_FONTCOLOR  CONFIG_NXWM_DEFAULT_FONTCOLOR
#endif

#ifndef CONFIG_NXWM_NXCONSOLE_FONTID
#  define CONFIG_NXWM_NXCONSOLE_FONTID  CONFIG_NXWM_DEFAULT_FONTID
#endif

/**
 * The NxConsole window glyph
 */

#ifndef CONFIG_NXWM_NXCONSOLE_ICON
#  define CONFIG_NXWM_NXCONSOLE_ICON NxWM::g_cmdBitmap
#endif

/* Touchscreen device *******************************************************/
/**
 * Touchscreen device settings
 *
 * CONFIG_NXWM_TOUCHSCREEN_DEVNO - Touchscreen device minor number, i.e., the
 *   N in /dev/inputN.  Default: 0
 * CONFIG_NXWM_TOUCHSCREEN_DEVNO - The full path to the touchscreen device.
 *   Default: "/dev/input0"
 * CONFIG_NXWM_TOUCHSCREEN_SIGNO - The realtime signal used to wake up the
 *   touchscreen listener thread.  Default: 5
 * CONFIG_NXWM_TOUCHSCREEN_LISTENERPRIO - Priority of the touchscreen listener
 *   thread.  Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_NXWM_TOUCHSCREEN_LISTENERSTACK - Touchscreen listener thread stack
 *   size.  Default 1024
 */

#ifndef CONFIG_NXWM_TOUCHSCREEN_DEVNO
#  define CONFIG_NXWM_TOUCHSCREEN_DEVNO 0
#endif

#ifndef CONFIG_NXWM_TOUCHSCREEN_DEVPATH
#  define CONFIG_NXWM_TOUCHSCREEN_DEVPATH "/dev/input0"
#endif

#ifndef CONFIG_NXWM_TOUCHSCREEN_SIGNO
#  define CONFIG_NXWM_TOUCHSCREEN_SIGNO 5
#endif

#ifndef CONFIG_NXWM_TOUCHSCREEN_LISTENERPRIO
#  define CONFIG_NXWM_TOUCHSCREEN_LISTENERPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_NXWM_TOUCHSCREEN_LISTENERSTACK
#  define CONFIG_NXWM_TOUCHSCREEN_LISTENERSTACK 1024
#endif

/* Calibration display ******************************************************/
/**
 * Calibration display settings:
 *
 * CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR - The background color of the
 *   touchscreen calibration display.  Default:  Same as
 *   CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR
 * CONFIG_NXWM_CALIBRATION_LINECOLOR - The color of the lines used in the
 *   touchscreen calibration display.  Default:  MKRGB(0, 0, 128) (dark blue)
 * CONFIG_NXWM_CALIBRATION_CIRCLECOLOR - The color of the circle in the
 *   touchscreen calibration display.  Default:  MKRGB(255, 255, 255) (white)
 * CONFIG_NXWM_CALIBRATION_CIRCLECOLOR - The color of the circle in the
 *   touchscreen calibration display after the touch is recorder.  Default: 
 *   MKRGB(255, 255, 96) (very light yellow)
 * CONFIG_NXWM_CALIBRATION_ICON - The ICON to use for the touchscreen
 *   calibration application.  Default:  NxWM::g_calibrationBitmap
 * CONFIG_NXWM_CALIBRATION_SIGNO - The realtime signal used to wake up the
 *   touchscreen calibration thread.  Default: 5
 * CONFIG_NXWM_CALIBRATION_LISTENERPRIO - Priority of the touchscreen listener
 *   thread.  Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_NXWM_CALIBRATION_LISTENERSTACK - Touchscreen listener thread stack
 *   size.  Default 2048
 */

#ifndef CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR
#  define CONFIG_NXWM_CALIBRATION_BACKGROUNDCOLOR CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR
#endif

#ifndef CONFIG_NXWM_CALIBRATION_LINECOLOR
#  define CONFIG_NXWM_CALIBRATION_LINECOLOR MKRGB(0, 0, 128)
#endif

#ifndef CONFIG_NXWM_CALIBRATION_CIRCLECOLOR
#  define CONFIG_NXWM_CALIBRATION_CIRCLECOLOR MKRGB(255, 255, 255)
#endif

#ifndef CONFIG_NXWM_CALIBRATION_TOUCHEDCOLOR
#  define CONFIG_NXWM_CALIBRATION_TOUCHEDCOLOR MKRGB(255, 255, 96)
#endif

#ifndef CONFIG_NXWM_CALIBRATION_ICON
#  define CONFIG_NXWM_CALIBRATION_ICON NxWM::g_calibrationBitmap
#endif

#ifndef CONFIG_NXWM_CALIBRATION_SIGNO
#  define CONFIG_NXWM_CALIBRATION_SIGNO 5
#endif

#ifndef CONFIG_NXWM_CALIBRATION_LISTENERPRIO
#  define CONFIG_NXWM_CALIBRATION_LISTENERPRIO SCHED_PRIORITY_DEFAULT
#endif

#ifndef CONFIG_NXWM_CALIBRATION_LISTENERSTACK
#  define CONFIG_NXWM_CALIBRATION_LISTENERSTACK 2048
#endif

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/
/**
 * Hook to support monitoring of memory usage by the NxWM unit test.
 */

#ifdef CONFIG_NXWM_UNITTEST
#  ifdef CONFIG_HAVE_FILENAME
void _showTestStepMemory(FAR const char *file, int line, FAR const char *msg);
#    define showTestStepMemory(msg) \
        _showTestStepMemory((FAR const char*)__FILE__, (int)__LINE__, msg)
#  else
void showTestStepMemory(FAR const char *msg);
#  endif
#else
#  define showTestStepMemory(msg)
#endif

#endif // __INCLUDE_NXWMCONFIG_HXX
