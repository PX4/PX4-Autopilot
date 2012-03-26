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

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

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
 *   wndo - Describes the window and font to be used
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
 *   wndo - Describes the window and font to be used
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
 *   wndo - Describes the window and font to be used
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXCONSOLE_H */
