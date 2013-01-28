/****************************************************************************
 * examples/nximage/nximage.h
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

#ifndef __APPS_EXAMPLES_NXIMAGE_NXIMAGE_H
#define __APPS_EXAMPLES_NXIMAGE_NXIMAGE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX
#  error "NX is not enabled (CONFIG_NX)"
#endif

#ifndef CONFIG_EXAMPLES_NXIMAGE_VPLANE
#    define CONFIG_EXAMPLES_NXIMAGE_VPLANE 0
#endif

#ifndef CONFIG_EXAMPLES_NXIMAGE_BPP
#  define CONFIG_EXAMPLES_NXIMAGE_BPP 16
#endif

#if defined(CONFIG_EXAMPLES_NXIMAGE_XSCALEp5)
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0
#elif defined(CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5)
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALEp5
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0
#elif defined(CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0)
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALEp5
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5
#else
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALEp5
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5
#  undef CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0
#  define CONFIG_EXAMPLES_NXIMAGE_XSCALE1p0 1
#endif

#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALEp5)
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0
#elif defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5)
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALEp5
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0
#elif defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0)
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALEp5
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5
#else
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALEp5
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p0
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5
#  undef CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0
#  define CONFIG_EXAMPLES_NXIMAGE_YSCALE1p0 1
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#    define msgflush()
#  else
#    define message(...) printf(__VA_ARGS__)
#    define msgflush() fflush(stdout)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#    define msgflush()
#  else
#    define message printf
#    define msgflush() fflush(stdout)
#  endif
#endif

/* Image Information ********************************************************/

#define IMAGE_HEIGHT       160  /* Number of rows in the raw image */
#define IMAGE_WIDTH        160  /* Number of columns in the raw image */

#if defined(CONFIG_EXAMPLES_NXIMAGE_XSCALEp5)
#  define SCALED_WIDTH     80   /* Number of columns in the scaled image */
#elif defined(CONFIG_EXAMPLES_NXIMAGE_XSCALE1p5)
#  define SCALED_WIDTH     240  /* Number of columns in the scaled image */
#elif defined(CONFIG_EXAMPLES_NXIMAGE_XSCALE2p0)
#  define SCALED_WIDTH     320  /* Number of columns in the scaled image */
#else
#  define SCALED_WIDTH     160  /* Number of columns in the scaled image */
#endif

#if defined(CONFIG_EXAMPLES_NXIMAGE_YSCALEp5)
#  define SCALED_HEIGHT    80   /* Number of rows in the scaled image */
#elif defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE1p5)
#  define SCALED_HEIGHT    240  /* Number of rows in the scaled image */
#elif defined(CONFIG_EXAMPLES_NXIMAGE_YSCALE2p0)
#  define SCALED_HEIGHT    320  /* Number of rows in the scaled image */
#else
#  define SCALED_HEIGHT    160  /* Number of rows in the scaled image */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum exitcode_e
{
  NXEXIT_SUCCESS = 0,
  NXEXIT_EXTINITIALIZE,
  NXEXIT_FBINITIALIZE,
  NXEXIT_FBGETVPLANE,
  NXEXIT_LCDINITIALIZE,
  NXEXIT_LCDGETDEV,
  NXEXIT_NXOPEN,
  NXEXIT_NXREQUESTBKGD,
  NXEXIT_NXSETBGCOLOR
};

struct nximage_data_s
{
  /* The NX handles */

  NXHANDLE hnx;
  NXHANDLE hbkgd;

  /* The screen resolution */

  nxgl_coord_t xres;
  nxgl_coord_t yres;

  volatile bool havepos;
  sem_t sem;
  volatile int code;
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/* NXIMAGE state data */

extern struct nximage_data_s g_nximage;

/* NX callback vtables */

extern const struct nx_callback_s g_nximagecb;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_NXIMAGE_EXTERNINIT
extern FAR NX_DRIVERTYPE *up_nxdrvinit(unsigned int devno);
#endif

/* Background window interfaces */

extern void nximage_image(NXWINDOW hwnd);

/* Image interfaces */

extern nxgl_mxpixel_t nximage_bgcolor(void);
extern nxgl_mxpixel_t nximage_avgcolor(nxgl_mxpixel_t color1, nxgl_mxpixel_t color2);
extern void nximage_blitrow(FAR nxgl_mxpixel_t *run, FAR const void **state);

#endif /* __APPS_EXAMPLES_NXIMAGE_NXIMAGE_H */
