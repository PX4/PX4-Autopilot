/****************************************************************************
 * include/nuttx/nx/nxconsole.h
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

#ifndef __INCLUDE_NUTTX_NX_NXCONSOLE_H
#define __INCLUDE_NUTTX_NX_NXCONSOLE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxtk.h>

#ifdef CONFIG_NXCONSOLE

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Nx Console prerequistes */

#ifndef CONFIG_NX
#  warning "NX is not enabled (CONFIG_NX)
#endif

#ifndef CONFIG_NX_MULTIUSER
#  warning "NX Console requires multi-user support (CONFIG_NX_MULTIUSER)"
#endif

/* Nx Console configuration options:
 *
 * CONFIG_NXCONSOLE
 *   Enables building of the NxConsole driver.
 *
 * Output text/graphics options:
 *
 * CONFIG_NXCONSOLE_BPP
 *   Currently, NxConsole supports only a single pixel depth. This
 *   configuration setting must be provided to support that single pixel depth.
 *   Default: The smallest enabled pixel depth. (see CONFIG_NX_DISABLE_*BPP)
 * CONFIG_NXCONSOLE_CURSORCHAR
 *   The bitmap code to use as the cursor.  Default '_'
 * CONFIG_NXCONSOLE_MXCHARS
 *   NxConsole needs to remember every character written to the console so
 *   that it can redraw the window. This setting determines the size of some
 *   internal memory allocations used to hold the character data. Default: 128.
 * CONFIG_NXCONSOLE_CACHESIZE
 *   NxConsole supports caching of rendered fonts. This font caching is required
 *   for two reasons: (1) First, it improves text performance, but more
 *   importantly (2) it preserves the font memory. Since the NX server runs on
 *   a separate server thread, it requires that the rendered font memory persist
 *   until the server has a chance to render the font. (NOTE: There is still
 *   inherently a race condition in this!). Unfortunately, the font cache would
 *   be quite large if all fonts were saved. The CONFIG_NXCONSOLE_CACHESIZE setting
 *   will control the size of the font cache (in number of glyphs). Only that
 *   number of the most recently used glyphs will be retained. Default: 16.
 * CONFIG_NXCONSOLE_LINESEPARATION
 *   This the space (in rows) between each row of test.  Default: 0
 * CONFIG_NXCONSOLE_NOWRAP
 *   By default, lines will wrap when the test reaches the right hand side
 *   of the window. This setting can be defining to change this behavior so
 *   that the text is simply truncated until a new line is  encountered.
 *
 * Input options:
 *
 * CONFIG_NXCONSOLE_NXKBDIN
 *   Take input from the NX keyboard input callback.  By default, keyboard
 *   input is taken from stdin (/dev/console).  If this option is set, then
 *   the interface nxcon_kbdin() is enabled.  That interface may be driven
 *   by window callback functions so that keyboard input *only* goes to the
 *   top window.
 * CONFIG_NXCONSOLE_KBDBUFSIZE
 *   If CONFIG_NXCONSOLE_NXKBDIN is enabled, then this value may be used to
 *   define the size of the per-window keyboard input buffer.  Default: 16
 * CONFIG_NXCONSOLE_NPOLLWAITERS
 *   The number of threads that can be waiting for read data available.
 *   Default: 4
 */

/* Cursor character */

#ifndef CONFIG_NXCONSOLE_CURSORCHAR
#  define CONFIG_NXCONSOLE_CURSORCHAR '_'
#endif

/* The maximum number of characters that can be remembered */

#ifndef CONFIG_NXCONSOLE_MXCHARS
#  define CONFIG_NXCONSOLE_MXCHARS 128
#endif

/* Font cache -- this is the number or pre-rendered font glyphs that can be
 * remembered.
 */

#ifndef CONFIG_NXCONSOLE_CACHESIZE
#  define CONFIG_NXCONSOLE_CACHESIZE 16
#endif

/* Pixel depth */

#ifndef CONFIG_NXCONSOLE_BPP
#  if !defined(CONFIG_NX_DISABLE_1BPP)
#    define CONFIG_NXCONSOLE_BPP 1
#  elif !defined(CONFIG_NX_DISABLE_2BPP)
#    define CONFIG_NXCONSOLE_BPP 2
#  elif !defined(CONFIG_NX_DISABLE_4BPP)
#    define CONFIG_NXCONSOLE_BPP 4
#  elif !defined(CONFIG_NX_DISABLE_8BPP)
#    define CONFIG_NXCONSOLE_BPP 8
#  elif !defined(CONFIG_NX_DISABLE_16BPP)
#    define CONFIG_NXCONSOLE_BPP 16
//#elif !defined(CONFIG_NX_DISABLE_24BPP)
//#    define CONFIG_NXCONSOLE_BPP 24
#  elif !defined(CONFIG_NX_DISABLE_32BPP)
#    define CONFIG_NXCONSOLE_BPP 32
#  else
#    error "No pixel depth provided"
#  endif
#endif

/* Space (in rows) between lines */

#ifndef CONFIG_NXCONSOLE_LINESEPARATION
#  define CONFIG_NXCONSOLE_LINESEPARATION 0
#endif

/* Input options */

#ifndef CONFIG_NX_KBD
#  undef CONFIG_NXCONSOLE_NXKBDIN
#endif

#ifdef CONFIG_NXCONSOLE_NXKBDIN

#  ifndef CONFIG_NXCONSOLE_KBDBUFSIZE
#    define CONFIG_NXCONSOLE_KBDBUFSIZE 16
#  elif (CONFIG_NXCONSOLE_KBDBUFSIZE < 1) || (CONFIG_NXCONSOLE_KBDBUFSIZE > 255)
#    error "CONFIG_NXCONSOLE_KBDBUFSIZE is out of range (1-255)"
#  endif

#  ifndef CONFIG_NXCONSOLE_NPOLLWAITERS
#    define CONFIG_NXCONSOLE_NPOLLWAITERS 4
#  endif

#else
#  undef CONFIG_NXCONSOLE_KBDBUFSIZE
#  define CONFIG_NXCONSOLE_KBDBUFSIZE 0
#  define CONFIG_NXCONSOLE_NPOLLWAITERS 0
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the handle that can be used to access the consoles */

typedef FAR void *NXCONSOLE;

/* This structure describes the window and font characteristics */

struct nxcon_window_s
{
  nxgl_mxpixel_t wcolor[CONFIG_NX_NPLANES]; /* Window background color */
  nxgl_mxpixel_t fcolor[CONFIG_NX_NPLANES]; /* Font color */
  struct nxgl_size_s wsize;                 /* Window size */
  int fontid;                               /* The ID of the font to use */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nx_register
 *
 * Description:
 *   Register a console device on a raw NX window.  The device will be
 *   registered at /dev/nxconN where N is the provided minor number.
 *
 * Input Parameters:
 *   hwnd - A handle that will be used to access the window.  The window must
 *     persist and this handle must be valid for the life of the NX console.
 *   wndo - Describes the window and font to be used.  The information in
 *     this structure is copied and the original need not persist after
 *     nxtool_register() returns.
 *   minor - The device minor number
 *
 * Return:
 *   A non-NULL handle is returned on success. 
 *
 ****************************************************************************/

EXTERN NXCONSOLE nx_register(NXWINDOW hwnd, FAR struct nxcon_window_s *wndo,
                             int minor);

/****************************************************************************
 * Name: nxtk_register
 *
 * Description:
 *   Register a console device on a framed NX window.  The device will be
 *   registered at /dev/nxconN where N is the provided minor number.
 *
 * Input Parameters:
 *   hfwnd - A handle that will be used to access the window.  The window must
 *     persist and this handle must be valid for the life of the NX console.
 *   wndo - Describes the window and font to be used.  The information in
 *     this structure is copied and the original need not persist after
 *     nxtool_register() returns.
 *   minor - The device minor number
 *
 * Return:
 *   A non-NULL handle is returned on success. 
 *
 ****************************************************************************/

EXTERN NXCONSOLE nxtk_register(NXTKWINDOW hfwnd,
                               FAR struct nxcon_window_s *wndo, int minor);

/****************************************************************************
 * Name: nxtool_register
 *
 * Description:
 *   Register a console device on a toolbar of a framed NX window.  The
 *   device will be registered at /dev/nxconN where N is the provided minor
 *   number.
 *
 * Input Parameters:
 *   hfwnd - A handle that will be used to access the toolbar.  The toolbar
 *     must persist and this handle must be valid for the life of the NX
 *     console.
 *   wndo - Describes the window and font to be used.  The information in
 *     this structure is copied and the original need not persist after
 *     nxtool_register() returns.
 *   minor - The device minor number
 *
 * Return:
 *   A non-NULL handle is returned on success. 
 *
 ****************************************************************************/

EXTERN NXCONSOLE nxtool_register(NXTKWINDOW hfwnd,
                                 FAR struct nxcon_window_s *wndo, int minor);

/****************************************************************************
 * Name: nxcon_unregister
 *
 * Description:
 *   Un-register to NX console device.
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void nxcon_unregister(NXCONSOLE handle);

/****************************************************************************
 * Name: nxcon_redraw
 *
 * Description:
 *   Re-draw a portion of the NX console.  This function should be called
 *   from the appropriate window callback logic.
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *   rect - The rectangle that needs to be re-drawn (in window relative
 *          coordinates)
 *   more - true:  More re-draw requests will follow
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void nxcon_redraw(NXCONSOLE handle, FAR const struct nxgl_rect_s *rect,
                         bool more);

/****************************************************************************
 * Name: nxcon_kbdin
 *
 * Description:
 *  This function should be driven by the window kbdin callback function
 *  (see nx.h).  When the NxConsole is the top window and keyboard input is
 *  received on the top window, that window callback should be directed to
 *  this function.  This function will buffer the keyboard data and may
 *  it available to the NxConsole as stdin.
 *
 *  If CONFIG_NXCONSOLE_NXKBDIN is not selected, then the NxConsole will
 *  receive its input from stdin (/dev/console).  This works great but
 *  cannot be shared between different windows.  Chaos will ensue if you
 *  try to support multiple NxConsole windows without CONFIG_NXCONSOLE_NXKBDIN
 *
 * Input Parameters:
 *   handle - A handle previously returned by nx_register, nxtk_register, or
 *     nxtool_register.
 *   buffer   - The array of characters
 *   buflen  - The number of characters that are available in buffer[]
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_NXCONSOLE_NXKBDIN
EXTERN void nxcon_kbdin(NXCONSOLE handle, FAR const uint8_t *buffer,
                        uint8_t buflen);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_NXCONSOLE */
#endif /* __INCLUDE_NUTTX_NX_NXCONSOLE_H */
