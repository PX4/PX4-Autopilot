/****************************************************************************
 * arch/sim/src/up_framebuffer.c
 *
 *   Copyright (C) 2008-2010 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fb.h>
#include "up_internal.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SIM_FBWIDTH
#  define CONFIG_SIM_FBWIDTH  480 /* Framebuffer width in pixels */
#endif

#ifndef CONFIG_SIM_FBHEIGHT
#  define CONFIG_SIM_FBHEIGHT 240 /* Framebuffer height in pixels */
#endif

#ifndef CONFIG_SIM_FBBPP
#  define CONFIG_SIM_FBBPP    16  /* Framebuffer bytes per pixel (RGB) */
#endif

#undef FB_FMT
#if CONFIG_SIM_FBBPP == 1
#  define FB_FMT FB_FMT_RGB1
#elif CONFIG_SIM_FBBPP == 4
#  define FB_FMT FB_FMT_RGB4
#elif CONFIG_SIM_FBBPP == 8
#  define FB_FMT FB_FMT_RGB8
#elif CONFIG_SIM_FBBPP == 16
#  define FB_FMT FB_FMT_RGB16_565
#elif CONFIG_SIM_FBBPP == 24
#  define FB_FMT FB_FMT_RGB24
#elif CONFIG_SIM_FBBPP == 32
#  define FB_FMT FB_FMT_RGB32
#else
#  error "Unsupported BPP"
#endif

/* Framebuffer characteristics in bytes */

#define FB_WIDTH ((CONFIG_SIM_FBWIDTH * CONFIG_SIM_FBBPP + 7) / 8)
#define FB_SIZE  (FB_WIDTH * CONFIG_SIM_FBHEIGHT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

  /* Get information about the video controller configuration and the configuration
   * of each color plane.
   */

static int up_getvideoinfo(FAR struct fb_vtable_s *vtable, FAR struct fb_videoinfo_s *vinfo);
static int up_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno, FAR struct fb_planeinfo_s *pinfo);

  /* The following is provided only if the video hardware supports RGB color mapping */

#ifdef CONFIG_FB_CMAP
static int up_getcmap(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap);
static int up_putcmap(FAR struct fb_vtable_s *vtable, FAR const struct fb_cmap_s *cmap);
#endif
  /* The following is provided only if the video hardware supports a hardware cursor */

#ifdef CONFIG_FB_HWCURSOR
static int up_getcursor(FAR struct fb_vtable_s *vtable, FAR struct fb_cursorattrib_s *attrib);
static int up_setcursor(FAR struct fb_vtable_s *vtable, FAR struct fb_setcursor_s *setttings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The simulated framebuffer memory */

#ifndef CONFIG_SIM_X11FB
static uint8_t g_fb[FB_SIZE];
#endif

/* This structure describes the simulated video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt      = FB_FMT,
  .xres     = CONFIG_SIM_FBWIDTH,
  .yres     = CONFIG_SIM_FBHEIGHT,
  .nplanes  = 1,
};

#ifndef CONFIG_SIM_X11FB
/* This structure describes the single, simulated color plane */

static const struct fb_planeinfo_s g_planeinfo =
{
  .fbmem    = (FAR void *)&g_fb,
  .fblen    = FB_SIZE,
  .stride   = FB_WIDTH,
  .bpp      = CONFIG_SIM_FBBPP,
};
#else
/* This structure describes the single, X11 color plane */

static struct fb_planeinfo_s g_planeinfo;
#endif

/* Current cursor position */

#ifdef CONFIG_FB_HWCURSOR
static struct fb_cursorpos_s g_cpos;

/* Current cursor size */

#ifdef CONFIG_FB_HWCURSORSIZE
static struct fb_cursorsize_s g_csize;
#endif
#endif

/* The framebuffer object -- There is no private state information in this simple
 * framebuffer simulation.
 */

struct fb_vtable_s g_fbobject =
{
  .getvideoinfo  = up_getvideoinfo,
  .getplaneinfo  = up_getplaneinfo,
#ifdef CONFIG_FB_CMAP
  .getcmap       = up_getcmap,
  .putcmap       = up_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor     = up_getcursor,
  .setcursor     = up_setcursor,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getvideoinfo
 ****************************************************************************/

static int up_getvideoinfo(FAR struct fb_vtable_s *vtable,
                           FAR struct fb_videoinfo_s *vinfo)
{
  dbg("vtable=%p vinfo=%p\n", vtable, vinfo);
  if (vtable && vinfo)
    {
      memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
      return OK;
    }
  dbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: up_getplaneinfo
 ****************************************************************************/

static int up_getplaneinfo(FAR struct fb_vtable_s *vtable, int planeno,
                           FAR struct fb_planeinfo_s *pinfo)
{
  dbg("vtable=%p planeno=%d pinfo=%p\n", vtable, planeno, pinfo);
  if (vtable && planeno == 0 && pinfo)
    {
      memcpy(pinfo, &g_planeinfo, sizeof(struct fb_planeinfo_s));
      return OK;
    }
  dbg("Returning EINVAL\n");
  return -EINVAL;
}

/****************************************************************************
 * Name: up_getcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int up_getcmap(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap)
{
  int len;
  int i;

  dbg("vtable=%p cmap=%p len=%d\n", vtable, cmap, cmap->len);
  if (vtable && cmap)
    {
      for (i = cmap->first, len = 0; i < 256 && len < cmap->len; i++, len++)
        {
          cmap->red[i]    = i;
          cmap->green[i]  = i;
          cmap->blue[i]   = i;
#ifdef CONFIG_FB_TRANSPARENCY
          cmap->transp[i] = i;
#endif
        }

      cmap->len = len;
      return OK;
    }
  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: up_putcmap
 ****************************************************************************/

#ifdef CONFIG_FB_CMAP
static int up_putcmap(FAR struct fb_vtable_s *vtable, FAR const struct fb_cmap_s *cmap)
{
#ifdef CONFIG_SIM_X11FB
  return up_x11cmap(cmap->first, cmap->len, cmap->red, cmap->green, cmap->blue, NULL);
#else
  dbg("vtable=%p cmap=%p len=%d\n", vtable, cmap, cmap->len);
  if (vtable && cmap)
    {
      return OK;
    }
  dbg("Returning EINVAL\n");
  return -EINVAL;
#endif
}
#endif

/****************************************************************************
 * Name: up_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int up_getcursor(FAR struct fb_vtable_s *vtable,
                        FAR struct fb_cursorattrib_s *attrib)
{
  dbg("vtable=%p attrib=%p\n", vtable, attrib);
  if (vtable && attrib)
    {
#ifdef CONFIG_FB_HWCURSORIMAGE
      attrib->fmt      = FB_FMT;
#endif
      dbg("pos:      (x=%d, y=%d)\n", g_cpos.x, g_cpos.y);
      attrib->pos      = g_cpos;
#ifdef CONFIG_FB_HWCURSORSIZE
      attrib->mxsize.h = CONFIG_SIM_FBHEIGHT;
      attrib->mxsize.w = CONFIG_SIM_FBWIDTH;
      dbg("size:     (h=%d, w=%d)\n", g_csize.h, g_csize.w);
      attrib->size     = g_csize;
#endif
      return OK;
    }
  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Name: 
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int up_setcursor(FAR struct fb_vtable_s *vtable,
                       FAR struct fb_setcursor_s *setttings)
{
  dbg("vtable=%p setttings=%p\n", vtable, setttings);
  if (vtable && setttings)
    {
      dbg("flags:   %02x\n", settings->flags);
      if ((flags & FB_CUR_SETPOSITION) != 0)
        {
          g_cpos = settings->pos;
          dbg("pos:     (h:%d, w:%d)\n", g_cpos.x, g_cpos.y);
        }
#ifdef CONFIG_FB_HWCURSORSIZE
      if ((flags & FB_CUR_SETSIZE) != 0)
        {
          g_csize = settings->size;
          dbg("size:    (h:%d, w:%d)\n", g_csize.h, g_csize.w);
        }
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
      if ((flags & FB_CUR_SETIMAGE) != 0)
        {
          dbg("image:   (h:%d, w:%d) @ %p\n",
              settings->img.height, settings->img.width, settings->img.image);
        }
#endif
      return OK;
    }
  dbg("Returning EINVAL\n");
  return -EINVAL;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_fbinitialize
 *
 * Description:
 *   Initialize the video hardware
 *
 ****************************************************************************/

int up_fbinitialize(void)
{
#ifdef CONFIG_SIM_X11FB
  return up_x11initialize(CONFIG_SIM_FBWIDTH, CONFIG_SIM_FBHEIGHT,
                          &g_planeinfo.fbmem, &g_planeinfo.fblen,
                          &g_planeinfo.bpp, &g_planeinfo.stride);
#else
  return OK;
#endif
}

/****************************************************************************
 * Name: up_fbgetvplane
 *
 * Description:
 *   Return a a reference to the framebuffer object for the specified video plane.
 *
 * Input parameters:
 *   None
 *
 * Returned value:
 *   Reference to the framebuffer object (NULL on failure)
 *
 ***************************************************************************/

FAR struct fb_vtable_s *up_fbgetvplane(int vplane)
{
  if (vplane == 0)
    {
      return &g_fbobject;
    }
  else
    {
      return NULL;
    }
}

/****************************************************************************
 * Name: up_fbteardown
 ****************************************************************************/

void fb_uninitialize(void)
{
}

