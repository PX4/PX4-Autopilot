/****************************************************************************
 * NxWidgets/libnxwidgets/include/nxconfig.hxx
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

#ifndef __INCLUDE_NXCONFIG_HXX
#define __INCLUDE_NXCONFIG_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/rgbcolors.h>
#include <nuttx/nx/nxfonts.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* NX Configuration *********************************************************/
/**
 * Prerequisites:
 *
 * CONFIG_HAVE_CXX=y   : C++ support is required
 * CONFIG_NX=y         : NX graphics support must be enabled
 * CONFIG_NX_MOUSE=y   : Required to enable NX mouse/touchscreen support
 * CONFIG_NX_KBD=y     : Required to enabled NX keyboard support
 * CONFIG_NX_NPLANES=1 : Only a single video plane is supported
 *
 * NX Server/Device Configuration
 *
 * CONFIG_NXWIDGETS_DEVNO - LCD device number (in case there are more than
 *   one LCDs connected.  Default: 0
 * CONFIG_NXWIDGETS_VPLANE - Only a single video plane is supported. Default: 0
 * CONFIG_NXWIDGETS_SERVERPRIO - Priority of the NX server.  This applies
 *   only if NX is configured in multi-user mode (CONFIG_NX_MULTIUSER=y).
 *   Default: SCHED_PRIORITY_DEFAULT+1.  NOTE:  Of the three priority
 *   definitions here, CONFIG_NXWIDGETS_SERVERPRIO should have the highest
 *   priority to avoid data overrun race conditions. Such errors would most
 *   likely appear as duplicated rows of data on the display.
 * CONFIG_NXWIDGETS_CLIENTPRIO - The thread that calls CNxServer::connect()
 *   will be re-prioritized to this priority.  This applies only if NX is
 *   configured in multi-user mode (CONFIG_NX_MULTIUSER=y). Default:
 *   SCHED_PRIORITY_DEFAULT
 * CONFIG_NXWIDGETS_LISTENERPRIO - Priority of the NX event listener thread.
 *   This applies only if NX is configured in multi-user mode
 *   (CONFIG_NX_MULTIUSER=y). Default: SCHED_PRIORITY_DEFAULT
 * CONFIG_NXWIDGETS_EXTERNINIT - Define to support external display
 *   initialization.
 * CONFIG_NXWIDGETS_SERVERSTACK - NX server thread stack size (in multi-user
 * mode).  Default 2048
 * CONFIG_NXWIDGETS_LISTENERSTACK - NX listener thread stack size (in multi-user
 * mode).  Default 2048
 * CONFIG_NXWIDGET_EVENTWAIT - Build in support for external window event, modal
 *   loop management logic.  This includes methods to wait for windows events
 *   to occur so that looping logic can sleep until something interesting
 *   happens with the window.
 *
 * NXWidget Configuration
 *
 * CONFIG_NXWIDGETS_BPP - Supported bits-per-pixel {8, 16, 24, 32}.  Default:
 *   The smallest BPP configuration supported by NX.
 * CONFIG_NXWIDGETS_SIZEOFCHAR - Size of character {1 or 2 bytes}.  Default
 *   Determined by CONFIG_NXWIDGETS_SIZEOFCHAR
 *
 * NXWidget Default Values
 *
 * CONFIG_NXWIDGETS_DEFAULT_FONTID - Default font ID.  Default: NXFONT_DEFAULT
 * CONFIG_NXWIDGETS_TNXARRAY_INITIALSIZE, CONFIG_NXWIDGETS_TNXARRAY_SIZEINCREMENT -
 *   Default dynamic array parameters.  Default: 16, 8
 *
 * CONFIG_NXWIDGETS_DEFAULT_BACKGROUNDCOLOR - Normal background color.  Default:
 *   MKRGB(148,189,215)
 * CONFIG_NXWIDGETS_DEFAULT_SELECTEDBACKGROUNDCOLOR - Default selected background
 *   color.  Default: MKRGB(206,227,241)
 * CONFIG_NXWIDGETS_DEFAULT_SHINEEDGECOLOR - Shiny side boarder color. Default
 *   MKRGB(248,248,248)
 * CONFIG_NXWIDGETS_DEFAULT_SHADOWEDGECOLOR - Shadowed side border color.
 *   Default: MKRGB(35,58,73)
 * CONFIG_NXWIDGETS_DEFAULT_HIGHLIGHTCOLOR - Highlight color.  Default:
 *   MKRGB(192,192,192)
 * CONFIG_NXWIDGETS_DEFAULT_DISABLEDTEXTCOLOR - Text color on a disabled widget:
 *   Default: MKRGB(192,192,192)
 * CONFIG_NXWIDGETS_DEFAULT_ENABLEDTEXTCOLOR - Text color on a enabled widget:
 *   Default: MKRGB(248,248,248)
 * CONFIG_NXWIDGETS_DEFAULT_SELECTEDTEXTCOLOR - Text color on a selected widget:
 *   Default: MKRGB(0,0,0)
 * CONFIG_NXWIDGETS_DEFAULT_FONTCOLOR - Default font color: Default:
 *   MKRGB(255,255,255)
 * CONFIG_NXWIDGETS_TRANSPARENT_COLOR - Transparent color: Default: MKRGB(0,0,0)
 *
 * Keypad behavior
 *
 * CONFIG_NXWIDGETS_FIRST_REPEAT_TIME - Time taken before a key starts
 *   repeating (in milliseconds).  Default: 500
 * CONFIG_NXWIDGETS_CONTINUE_REPEAT_TIME - Time taken before a repeating key
 *   repeats again (in milliseconds).  Default: 200
 * CONFIG_NXWIDGETS_DOUBLECLICK_TIME - Left button release-press time for
 *   double click (in milliseconds).  Default: 350
 * CONFIG_NXWIDGETS_KBDBUFFER_SIZE - Size of incoming character buffer, i.e.,
 *   the maximum number of characters that can be entered between NX polling
 *   cycles without losing data.
 * CONFIG_NXWIDGETS_CURSORCONTROL_SIZE - Size of incoming cursor control
 *   buffer, i.e., the maximum number of cursor controls that can between
 *   entered by NX polling cycles without losing data.  Default: 4
 */

/* Prerequisites ************************************************************/
/**
 * C++ support is required
 */

#ifndef CONFIG_HAVE_CXX
#  error "C++ support is required (CONFIG_HAVE_CXX)"
#endif

/**
 * NX graphics support must be enabled
 */

#ifndef CONFIG_NX
#  error "NX graphics support is required (CONFIG_NX)"
#endif

/**
 * Required to enable NX mouse/touchscreen support
 */

#ifndef CONFIG_NX_MOUSE
#  warning "NX mouse/touchscreen support is required (CONFIG_NX_MOUSE)"
#endif

/**
 * Required to enabled NX keyboard support
 */

#ifndef CONFIG_NX_KBD
#  warning "NX keyboard support is required (CONFIG_NX_KBD)"
#endif

/**
 * Only a single video plane is supported
 */

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES 1
#endif

#if CONFIG_NX_NPLANES != 1
#  error "Only a single color plane is supported (CONFIG_NX_NPLANES)"
#endif

/* NxConsole checks.  This just simplifies the conditional compilation by
 * reducing the AND of these three conditions to a single condition.
 */

#if !defined(CONFIG_NX_KBD) || !defined(CONFIG_NXCONSOLE)
#  undef CONFIG_NXCONSOLE_NXKBDIN
#endif

/* NX Server/Device Configuration *******************************************/
/**
 * LCD device number (in case there are more than one LCDs connected)
 */

#ifndef CONFIG_NXWIDGETS_DEVNO
#  define CONFIG_NXWIDGETS_DEVNO 0
#endif

/**
 * Only a single video plane is supported
 */

#ifndef CONFIG_NXWIDGETS_VPLANE
#  define CONFIG_NXWIDGETS_VPLANE 0
#endif

/**
 * Priority of the NX server (in multi-user mode)
 */

#ifndef CONFIG_NXWIDGETS_SERVERPRIO
#  define CONFIG_NXWIDGETS_SERVERPRIO (SCHED_PRIORITY_DEFAULT+1)
#endif

#ifndef CONFIG_NXWIDGETS_CLIENTPRIO
#  define CONFIG_NXWIDGETS_CLIENTPRIO SCHED_PRIORITY_DEFAULT
#endif

#if CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWIDGETS_CLIENTPRIO
#  warning "CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWIDGETS_CLIENTPRIO"
#  warning" -- This can result in data overrun errors"
#endif

/**
 * NX server thread stack size (in multi-user mode)
 */

#ifndef CONFIG_NXWIDGETS_SERVERSTACK
#  define CONFIG_NXWIDGETS_SERVERSTACK 2048
#endif

/**
 * Priority of the NX event listener thread (in multi-user mode)
 */

#ifndef CONFIG_NXWIDGETS_LISTENERPRIO
#  define CONFIG_NXWIDGETS_LISTENERPRIO SCHED_PRIORITY_DEFAULT
#endif

#if CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWIDGETS_LISTENERPRIO
#  warning "CONFIG_NXWIDGETS_SERVERPRIO <= CONFIG_NXWIDGETS_LISTENERPRIO"
#  warning" -- This can result in data overrun errors"
#endif

/**
 * NX listener thread stack size (in multi-user mode)
 */

#ifndef CONFIG_NXWIDGETS_LISTENERSTACK
#  define CONFIG_NXWIDGETS_LISTENERSTACK 2048
#endif

/* NXWidget Configuration ***************************************************/
/**
 * Bits per pixel
 */

#ifndef CONFIG_NXWIDGETS_BPP
#  if !defined(CONFIG_NX_DISABLE_8BPP)
#    warning "Assuming 8-bits per pixel, RGB 3:3:2"
#    define CONFIG_NXWIDGETS_BPP 8
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    warning "Assuming 16-bits per pixel, RGB 5:6:5"
#    define CONFIG_NXWIDGETS_BPP 16
#  elif !defined(CONFIG_NX_DISABLE_24BPP)
#    warning "Assuming 24-bits per pixel, RGB 8:8:8"
#    define CONFIG_NXWIDGETS_BPP 24
#  elif !defined(CONFIG_NX_DISABLE_32BPP)
#    warning "Assuming 32-bits per pixel, RGB 8:8:8"
#    define CONFIG_NXWIDGETS_BPP 32
#  else
#    error "No supported pixel depth is enabled"
#  endif
#endif

#if CONFIG_NXWIDGETS_BPP == 8
#  ifdef CONFIG_NX_DISABLE_8BPP
#    error "NX 8-bit support is disabled (CONFIG_NX_DISABLE_8BPP)"
#  endif
#  define CONFIG_NXWIDGETS_FMT FB_FMT_RGB8_332
#  define MKRGB                RGBTO8
#  define RGB2RED              RBG8RED
#  define RGB2GREEN            RBG8GREEN
#  define RGB2BLUE             RBG8BLUE
#  define FONT_RENDERER        nxf_convert_8bpp
#elif CONFIG_NXWIDGETS_BPP == 16
#  ifdef CONFIG_NX_DISABLE_16BPP
#    error "NX 16-bit support is disabled (CONFIG_NX_DISABLE_16BPP)"
#  endif
#  define CONFIG_NXWIDGETS_FMT FB_FMT_RGB16_565
#  define MKRGB                RGBTO16
#  define RGB2RED              RBG16RED
#  define RGB2GREEN            RBG16GREEN
#  define RGB2BLUE             RBG16BLUE
#  define FONT_RENDERER        nxf_convert_16bpp
#elif CONFIG_NXWIDGETS_BPP == 24
#  ifdef CONFIG_NX_DISABLE_24BPP
#    error "NX 24-bit support is disabled (CONFIG_NX_DISABLE_24BPP)"
#  endif
#  define CONFIG_NXWIDGETS_FMT FB_FMT_RGB24
#  define MKRGB                RGBTO24
#  define RGB2RED              RBG24RED
#  define RGB2GREEN            RBG24GREEN
#  define RGB2BLUE             RBG24BLUE
#  define FONT_RENDERER        nxf_convert_24bpp
#elif CONFIG_NXWIDGETS_BPP == 32
#  ifdef CONFIG_NX_DISABLE_32BPP
#    error "NX 32-bit support is disabled (CONFIG_NX_DISABLE_32BPP)"
#  endif
#  define CONFIG_NXWIDGETS_FMT FB_FMT_RGB32
#  define MKRGB                RGBTO24
#  define RGB2RED              RBG24RED
#  define RGB2GREEN            RBG24GREEN
#  define RGB2BLUE             RBG24BLUE
#  define FONT_RENDERER        nxf_convert_32bpp
#else
#  error "Pixel depth not supported (CONFIG_NXWIDGETS_BPP)"
#endif

/* Size of a character */

#ifndef CONFIG_NXWIDGETS_SIZEOFCHAR
# if CONFIG_NXFONTS_CHARBITS <= 8
#    define CONFIG_NXWIDGETS_SIZEOFCHAR 1
# else
#    define CONFIG_NXWIDGETS_SIZEOFCHAR 2
# endif
#endif

#if CONFIG_NXWIDGETS_SIZEOFCHAR != 1 && CONFIG_NXWIDGETS_SIZEOFCHAR != 2
#  error "Unsupported character width (CONFIG_NXWIDGETS_SIZEOFCHAR)"
#endif

/* NXWidget Default Values **************************************************/
/**
 * Default font ID
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_FONTID
#  define CONFIG_NXWIDGETS_DEFAULT_FONTID NXFONT_DEFAULT
#endif

/**
 * Default dynamic array parameters
 */

#ifndef CONFIG_NXWIDGETS_TNXARRAY_INITIALSIZE
#  define CONFIG_NXWIDGETS_TNXARRAY_INITIALSIZE 16
#endif

#ifndef CONFIG_NXWIDGETS_TNXARRAY_SIZEINCREMENT
#  define CONFIG_NXWIDGETS_TNXARRAY_SIZEINCREMENT 8
#endif

/**
 * Normal background color
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_BACKGROUNDCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_BACKGROUNDCOLOR MKRGB(148,189,215)
#endif

/**
 * Default selected background color
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_SELECTEDBACKGROUNDCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_SELECTEDBACKGROUNDCOLOR MKRGB(206,227,241)
#endif

/**
 * Shiny side border color
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_SHINEEDGECOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_SHINEEDGECOLOR MKRGB(248,248,248)
#endif

/**
 * Shadowed side border color
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_SHADOWEDGECOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_SHADOWEDGECOLOR MKRGB(35,58,73)
#endif

/**
 * Highlight color
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_HIGHLIGHTCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_HIGHLIGHTCOLOR MKRGB(192,192,192)
#endif

/* Text colors */

#ifndef CONFIG_NXWIDGETS_DEFAULT_DISABLEDTEXTCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_DISABLEDTEXTCOLOR MKRGB(192,192,192)
#endif

#ifndef CONFIG_NXWIDGETS_DEFAULT_ENABLEDTEXTCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_ENABLEDTEXTCOLOR MKRGB(248,248,248)
#endif

#ifndef CONFIG_NXWIDGETS_DEFAULT_SELECTEDTEXTCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_SELECTEDTEXTCOLOR MKRGB(0,0,0)
#endif

/**
 * Default font color
 */

#ifndef CONFIG_NXWIDGETS_DEFAULT_FONTCOLOR
#  define CONFIG_NXWIDGETS_DEFAULT_FONTCOLOR MKRGB(255,255,255)
#endif

/**
 * Transparent color
 */

#ifndef CONFIG_NXWIDGETS_TRANSPARENT_COLOR
#  define CONFIG_NXWIDGETS_TRANSPARENT_COLOR MKRGB(0,0,0)
#endif

/* Keypad behavior **********************************************************/
/**
 * Time taken before a key starts repeating (in milliseconds).
 */

#ifndef CONFIG_NXWIDGETS_FIRST_REPEAT_TIME
#  define CONFIG_NXWIDGETS_FIRST_REPEAT_TIME 500
#endif

/**
 * Time taken before a repeating key repeats again (in milliseconds).
 */

#ifndef CONFIG_NXWIDGETS_CONTINUE_REPEAT_TIME
#  define CONFIG_NXWIDGETS_CONTINUE_REPEAT_TIME 200
#endif

/**
 * Left button release-press time for double click (in milliseconds).
 */

#ifndef CONFIG_NXWIDGETS_DOUBLECLICK_TIME
#  define CONFIG_NXWIDGETS_DOUBLECLICK_TIME 350
#endif

/**
 * Size of incoming character buffer, i.e., the maximum number of characters
 * that can be entered between NX polling cycles without losing data.
 */

#ifndef CONFIG_NXWIDGETS_KBDBUFFER_SIZE
#  define CONFIG_NXWIDGETS_KBDBUFFER_SIZE 8
#endif

/**
 * Size of incoming cursor control buffer, i.e., the maximum number of cursor
 * controls that can between entered by NX polling cycles without losing data.
 */

#ifndef CONFIG_NXWIDGETS_CURSORCONTROL_SIZE
#  define CONFIG_NXWIDGETS_CURSORCONTROL_SIZE 4
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

namespace NXWidgets
{
#if CONFIG_NXWIDGETS_BPP == 8
  typedef uint8_t nxwidget_pixel_t;
#elif CONFIG_NXWIDGETS_BPP == 16
  typedef uint16_t nxwidget_pixel_t;
#elif CONFIG_NXWIDGETS_BPP == 24
  typedef uint32_t nxwidget_pixel_t;
#elif CONFIG_NXWIDGETS_BPP == 32
  typedef uint32_t nxwidget_pixel_t;
#else
#  error "Pixel depth is unknown"
#endif

#if CONFIG_NXWIDGETS_SIZEOFCHAR == 2
  typedef uint16_t nxwidget_char_t;
#elif CONFIG_NXWIDGETS_SIZEOFCHAR == 1
  typedef uint8_t nxwidget_char_t;
#else
#  error "Character width is unknown"
#endif
}

/****************************************************************************
 * Implementation Classes
 ****************************************************************************/

#endif // __INCLUDE_NXCONFIG_HXX
