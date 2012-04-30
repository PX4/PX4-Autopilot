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

#include "nxconfig.hxx"
#include "crlepalettebitmap.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* General Configuration ****************************************************/
/**
 * NX Multi-user support is required
 */

#ifndef CONFIG_NX_MULTIUSER
#  warning "NX multi-user support is required (CONFIG_NX_MULTIUSER)"
#endif

/**
 * Default font ID
 */

#ifndef CONFIG_NXWM_DEFAULT_FONTID
#  define CONFIG_NXWM_DEFAULT_FONTID NXFONT_DEFAULT
#endif

/* Colors *******************************************************************/
/**
 * Normal background color
 */

#ifndef CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR
#  define CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR  MKRGB(160,160,160)
#endif

/**
 * Default selected background color
 */

#ifndef CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR
#  define CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR  MKRGB(120,192,192)
#endif

/**
 * Default foreground color
 */

#ifndef CONFIG_NXWM_DEFAULT_FOREGROUNDCOLOR
#  define CONFIG_NXWM_DEFAULT_FOREGROUNDCOLOR  MKRGB(192,192,192)
#endif

/**
 * Default selected foreground color
 */

#ifndef CONFIG_NXWM_DEFAULT_SELECTEDFOREGROUNDCOLOR
#  define CONFIG_NXWM_DEFAULT_SELECTEDFOREGROUNDCOLOR  MKRGB(248,248,248)
#endif

/**
 * Border colors
 */

#ifndef CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR
#  define CONFIG_NXWM_DEFAULT_SHINEEDGECOLOR  MKRGB(248,248,248)
#endif

#ifndef CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR
#  define CONFIG_NXWM_DEFAULT_SHADOWEDGECOLOR  MKRGB(0,0,0)
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
#  if defined defined (CONFIG_NXWM_TASKBAR_RIGHT)
#    warning "Multiple task bar positions specified"
#  endif
#elif !defined(CONFIG_NXWM_TASKBAR_RIGHT)
#  warning "No task bar position specified"
#  define CONFIG_NXWM_TASKBAR_TOP 1
#endif

/**
 * At present, all icons are 25 pixels in "widgth" and, hence require a
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

#ifndef CONFIG_NXWM_TOOLBAR_HEIGHT
#  define CONFIG_NXWM_TOOLBAR_HEIGHT CONFIG_NXWM_TASKBAR_WIDTH
#endif

/* Background Image **********************************************************/

#ifndef CONFIG_NXWM_BACKGROUND_IMAGE
#  define CONFIG_NXWM_BACKGROUND_IMAGE NXWidgets::g_nuttxBitmap
#endif

/* Start Window Configuration ***********************************************/
/**
 * Horizontal and vertical spacing of icons in the task bar.
 */

#ifndef CONFIG_NXWM_STARTWINDOW_VSPACING
#  define CONFIG_NXWM_STARTWINDOW_VSPACING (2)
#endif

#ifndef CONFIG_NXWM_STARTWINDOW_HSPACING
#  define CONFIG_NXWM_STARTWINDOW_HSPACING (2)
#endif

#endif // __INCLUDE_NXWMCONFIG_HXX
