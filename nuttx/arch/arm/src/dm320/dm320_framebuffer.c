/****************************************************************************
 * arch/arm/src/dm320/dm320_framebuffer.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fb.h>
#include <nuttx/nx/nxglib.h>

#include "up_arch.h"
#include "dm320_memorymap.h"
#include "dm320_osd.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/* Configuration ********************************************************/

/* Video (X,Y) base offset */

#ifndef CONFIG_DM320_BASEX
#  define CONFIG_DM320_BASEX 0x090
#endif

#ifndef CONFIG_DM320_BASEY
#  define CONFIG_DM320_BASEY 0x010
#endif

/* Background color */

#ifndef CONFIG_DM320_BKGDCLUT
#  define CONFIG_DM320_BKGDCLUT 0x00ff
#endif

#ifdef CONFIG_FB_HWCURSORIMAGE
#  error "Cursor image not supported"
#endif

/* Window selections */

#if defined(CONFIG_DM320_OSD0_RGB16) && defined(CONFIG_DM320_OSD1_RGB16)
#  error "Both CONFIG_DM320_OSD0_RGB16 and CONFIG_DM320_OSD1_RGB16 defined"
#endif

#if defined(CONFIG_DM320_VID0_DISABLE) && defined(CONFIG_DM320_VID1_DISABLE) && \
    defined(CONFIG_DM320_OSD0_DISABLE)  && defined(CONFIG_DM320_OSD1_DISABLE)
#  error "All windows are disabled, at least one window must be enabled"
#endif

#undef CONFIG_DM320_DISABLE_PINGPONG
#define CONFIG_DM320_DISABLE_PINGPONG 1 /* Not supported by interface */

/* Window positions and sizes */

#define MAX_XRES 640
#define MAX_YRES 480

#ifndef CONFIG_DM320_VID0_DISABLE
#  ifndef CONFIG_DM320_VID0_XRES
#    define CONFIG_DM320_VID0_XRES MAX_XRES
#  elif CONFIG_DM320_VID0_XRES > MAX_XRES
#    error "VID0 XRES out of range"
#  endif
#  ifndef CONFIG_DM320_VID0_XPOS
#    define CONFIG_DM320_VID0_XPOS 0
#  elif CONFIG_DM320_VID0_XPOS + CONFIG_DM320_VID0_XRES > MAX_XRES
#    error "VID0 XPOS out of range"
#  endif
#  ifndef CONFIG_DM320_VID0_YRES
#    define CONFIG_DM320_VID0_YRES MAX_YRES
#  elif CONFIG_DM320_VID0_YRES > MAX_YRES
#    error "VID0 YRES out of range"
#  endif
#  ifndef CONFIG_DM320_VID0_YPOS
#    define CONFIG_DM320_VID0_YPOS 0
#  elif CONFIG_DM320_VID0_YPOS + CONFIG_DM320_VID0_YRES > MAX_YRES
#    error "VID0 YPOS out of range"
#  endif
#endif

#ifndef CONFIG_DM320_VID1_DISABLE
#  ifndef CONFIG_DM320_VID1_XRES
#    define CONFIG_DM320_VID1_XRES MAX_XRES
#  elif CONFIG_DM320_VID1_XRES > MAX_XRES
#    error "VID1 XRES out of range"
#  endif
#  ifndef CONFIG_DM320_VID1_XPOS
#    define CONFIG_DM320_VID1_XPOS 0
#  elif CONFIG_DM320_VID1_XPOS + CONFIG_DM320_VID1_XRES > MAX_XRES
#    error "VID0 XPOS out of range"
#  endif
#  ifndef CONFIG_DM320_VID1_YRES
#    define CONFIG_DM320_VID1_YRES MAX_YRES
#  elif CONFIG_DM320_VID1_YRES > MAX_YRES
#    error "VID1 YRES out of range"
#  endif
#  ifndef CONFIG_DM320_VID1_YPOS
#    define CONFIG_DM320_VID1_YPOS 0
#  elif CONFIG_DM320_VID1_YPOS + CONFIG_DM320_VID1_YRES > MAX_YRES
#    error "VID1 YPOS out of range"
#  endif
#endif

#ifndef CONFIG_DM320_OSD0_DISABLE
#  ifndef CONFIG_DM320_OSD0_XRES
#    define CONFIG_DM320_OSD0_XRES MAX_XRES
#  elif CONFIG_DM320_OSD0_XRES > MAX_XRES
#    error "OSD0 XRES out of range"
#  endif
#  ifndef CONFIG_DM320_OSD0_XPOS
#    define CONFIG_DM320_OSD0_XPOS 0
#  elif CONFIG_DM320_OSD0_XPOS + CONFIG_DM320_OSD0_XRES > MAX_XRES
#    error "OSD0 XPOS out of range"
#  endif
#  ifndef CONFIG_DM320_OSD0_YRES
#    define CONFIG_DM320_OSD0_YRES MAX_YRES
#  elif CONFIG_DM320_OSD0_YRES > MAX_YRES
#    error "OSD0 YRES out of range"
#  endif
#  ifndef CONFIG_DM320_OSD0_YPOS
#    define CONFIG_DM320_OSD0_YPOS 0
#  elif CONFIG_DM320_OSD0_YPOS + CONFIG_DM320_OSD0_YRES > MAX_YRES
#    error "OSD0 YPOS out of range"
#  endif
#endif

#ifndef CONFIG_DM320_OSD1_DISABLE
#  ifndef CONFIG_DM320_OSD1_XRES
#    define CONFIG_DM320_OSD1_XRES MAX_XRES
#  elif CONFIG_DM320_OSD1_XRES > MAX_XRES
#    error "OSD1 XRES out of range"
#  endif
#  ifndef CONFIG_DM320_OSD1_XPOS
#    define CONFIG_DM320_OSD1_XPOS 0
#  elif CONFIG_DM320_OSD1_XPOS + CONFIG_DM320_OSD1_XRES > MAX_XRES
#    error "OSD1 XPOS out of range"
#  endif
#  ifndef CONFIG_DM320_OSD1_YRES
#    define CONFIG_DM320_OSD1_YRES MAX_YRES
#  elif CONFIG_DM320_OSD0_YRES > MAX_YRES
#    error "OSD0 YRES out of range"
#  endif
#  ifndef CONFIG_DM320_OSD1_YPOS
#    define CONFIG_DM320_OSD1_YPOS 0
#  elif CONFIG_DM320_OSD1_YPOS + CONFIG_DM320_OSD1_YRES > MAX_YRES
#    error "OSD1 YPOS out of range"
#  endif
#endif

/* Cursor selections */

#ifdef CONFIG_FB_HWCURSOR
#  ifndef CONFIG_DM320_CURSORCLUT
#    define CONFIG_DM320_CURSORCLUT 0x00
#  endif
#  ifndef CONFIG_DM320_CURSORLINEWIDTH
#    define CONFIG_DM320_CURSORLINEWIDTH 1
#  endif
#  if CONFIG_DM320_CURSORLINEWIDTH > 7
#    error "Rectangular cursor width is out of range"
#  endif
#  ifndef CONFIG_DM320_CURSORLINEHEIGHT
#    define CONFIG_DM320_CURSORLINEHEIGHT 1
#  endif
#  if CONFIG_DM320_CURSORLINEHEIGHT > 7
#    error "Rectangular cursor height is out of range"
#  endif
#  ifndef CONFIG_DM320_RECTCURSOR_WIDTH
#    define CONFIG_DM320_RECTCURSOR_WIDTH MAX_XRES
#  elif CONFIG_DM320_RECTCURSOR_WIDTH > MAX_XRES
#    error "Cursor width out of range"
#  endif
#  ifndef CONFIG_DM320_RECTCURSOR_HEIGHT
#    define CONFIG_DM320_RECTCURSOR_HEIGHT MAX_YRES
#  elif CONFIG_DM320_RECTCURSOR_HEIGHT > MAX_YRES
#    error "Cursor width out of range"
#  endif
#endif

/* DM320 ****************************************************************/

/* Video planes.  This long messy conditional compilation results in
 * consecutive plane numbers assigned for enable planes and the total
 * number of planes
 */

#ifndef CONFIG_DM320_VID0_DISABLE
#  define DM320_VIDWIN0         (0)            /* Have VID0 */
#  ifndef CONFIG_DM320_VID1_DISABLE
#    define DM320_VIDWIN1       (1)            /* Have VID0+VID1 */
#    ifndef CONFIG_DM320_OSD0_DISABLE
#      define DM320_OSDWIN0     (2)            /* Have VID0+VID1+OSD0 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1   (3)            /* Have VID0+VID1+OSD0+OSD1 */
#        define DM320_NFRAMES   (4)
#      else
#        define DM320_NFRAMES   (3)            /* Have VID0+VID1+OSD0 but not OSD1 */
#      endif
#    else                                      /* Have VID0+VID1 but not OSD0 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1   (2)            /* Have VID0+VID1+OSD1 but not OSD0 */
#        define DM320_NFRAMES   (3)
#      else
#        define DM320_NFRAMES   (2)            /* Have VID0+VID1 but not OSD0 or OSD1 */
#      endif
#    endif
#  else                                        /* Have VID0 but not VID1 */
#    ifndef CONFIG_DM320_OSD0_DISABLE
#      define DM320_OSDWIN0     (1)            /* Have VID0+OSD0 but not VID1 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1   (2)            /* Have VID0+OSD0+OSD1 but not VID1 */
#        define DM320_NFRAMES   (3)
#      else
#        define DM320_NFRAMES   (2)            /* Have VID0+OSD0 but not VID1 or OSD1 */
#      endif
#    else                                      /* Have VID0 but not VID1 or OSD0 */
#        ifndef CONFIG_DM320_OSD1_DISABLE
#          define DM320_OSDWIN1 (1)            /* Have VID0+OSD1 but not VID1 or OSD0 */
#          define DM320_NFRAMES (3)
#        else
#          define DM320_NFRAMES (2)            /* Have VID0 but not VID1, OSD0, or OSD1 */
#        endif
#      endif
#    endif
#else                                          /* Don't have VID0 */
#  ifndef CONFIG_DM320_VID1_DISABLE
#    define DM320_VIDWIN1     (0)            /* Have VID1 but not VID0 */
#    ifndef CONFIG_DM320_OSD0_DISABLE
#     define DM320_OSDWIN0    (1)            /* Have VID1+OSD0 not VID0 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1 (2)            /* Have VID1+OSD0+OSD1 not VID0 */
#        define DM320_NFRAMES (3)
#      else
#        define DM320_NFRAMES (2)            /* Have VID1+OSD0 but not VID0 or OSD1 */
#      endif
#    else                                    /* Have VID1 but not VID0 or OSD0 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1 (1)            /* Have VID1+OSD1 but not VID0 or OSD0 */
#        define DM320_NFRAMES (2)
#      else
#        define DM320_NFRAMES (2)            /* Have VID1 but not VID0, OSD0 or OSD1 */
#      endif
#    endif
#  else                                      /* Don't have VID0 or VID1 */
#    ifndef CONFIG_DM320_OSD0_DISABLE
#      define DM320_OSDWIN0   (0)            /* Have OSD0 but not VID0 or VID1 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1 (1)            /* Have OSD0+OSD1 but not VID0 or VID1 */
#        define DM320_NFRAMES (2)
#      else
#        define DM320_NFRAMES (1)            /* Have OSD0 but VID0, VID, or OSD1 */
#      endif
#    else                                    /* Don't have VID0, VID1, or OSD0 */
#      ifndef CONFIG_DM320_OSD1_DISABLE
#        define DM320_OSDWIN1 (0)            /* Have OSD1 but not VID0, VID1, or OSD0 */
#        define DM320_NFRAMES (1)
#      else
#        error "No video planes enabled"
#      endif
#    endif
#  endif
#endif

/* Bits per pixel */

#define DM320_VID0_BPP          (16)
#define DM320_VID1_BPP          (16)
#ifdef CONFIG_DM320_OSD0_RGB16
#  define DM320_OSD0_BPP        (16)
#else
#  define DM320_OSD0_BPP        (8)
#endif
#ifdef CONFIG_DM320_OSD1_RGB16
#  define DM320_OSD1_BPP        (16)
#else
#  define DM320_OSD1_BPP        (8)
#endif

/* These are DM320-specific ranges for the BASEPX/Y registers */

#define DM320_MIN_BASEPX        (24)
#define DM320_MAX_BASEPX        (1023)
#define DM320_MIN_BASEPY        (1)
#define DM320_MAX_BASEPY        (511)

#if (CONFIG_DM320_BASEX < DM320_MIN_BASEPX || CONFIG_DM320_BASEX > DM320_MAX_BASEPX)
#  error "CONFIG_DM320_BASEX is out of range"
#endif

#if (CONFIG_DM320_BASEY < DM320_MIN_BASEPY || CONFIG_DM320_BASEY > DM320_MAX_BASEPY)
#  error "CONFIG_DM320_BASEY is out of range"
#endif

/* The width of a line in bytes */

#define DM320_VID0_STRIDE       (CONFIG_DM320_VID0_XRES * DM320_VID0_BPP / 8)
#define DM320_VID1_STRIDE       (CONFIG_DM320_VID1_XRES * DM320_VID1_BPP / 8)
#define DM320_OSD0_STRIDE       (CONFIG_DM320_OSD0_XRES * DM320_OSD0_BPP / 8)
#define DM320_OSD1_STRIDE       (CONFIG_DM320_OSD1_XRES * DM320_OSD1_BPP / 8)

/* The area of the screen in bytes */

#define DM320_VID0_FBLEN        (DM320_VID0_STRIDE * CONFIG_DM320_VID0_YRES)
#define DM320_VID1_FBLEN        (DM320_VID1_STRIDE * CONFIG_DM320_VID1_YRES)
#define DM320_OSD0_FBLEN        (DM320_OSD0_STRIDE * CONFIG_DM320_OSD0_YRES)
#define DM320_OSD1_FBLEN        (DM320_OSD1_STRIDE * CONFIG_DM320_OSD1_YRES)

/* Video/OSD modes */

#ifndef CONFIG_DM320_VID0_DISABLE
#  ifdef CONFIG_DM320_VID0_FRAMEMODE
#    define DM320_VID0MODE 0x0003
#  else
#    define DM320_VID0MODE 0x0001
#  endif
#else
#  define DM320_VID0MODE   0x0000
#endif

#ifndef CONFIG_DM320_VID1_DISABLE
#  ifdef CONFIG_DM320_VID1_FRAMEMODE
#    define DM320_VID1MODE 0x0300
#  else
#    define DM320_VID1MODE 0x0100
#  endif
#else
#  define DM320_VID1MODE   0x0000
#endif

#define DM320_VIDMODE (DM320_VID0MODE|DM320_VID1MODE)

#ifndef CONFIG_DM320_OSD0_DISABLE
#  ifdef CONFIG_DM320_OSD0_FRAMEMODE
#    define OSD0MODE_FRAME 0x0002
#  else
#    define OSD0_FRAMEMODE 0x0000
#  endif

#  ifdef CONFIG_DM320_OSD0_TRANSPMODE
#    ifdef CONFIG_DM320_OSD0_BLEND8THS
#      if CONFIG_DM320_OSD0_BLEND8THS == 8
#        define OSD0MODE_TRANSPMODE 0x0004
#      elif CONFIG_DM320_OSD0_BLEND8THS == 7
#        define OSD0MODE_TRANSPMODE 0x000c
#      elif CONFIG_DM320_OSD0_BLEND8THS == 6
#        define OSD0MODE_TRANSPMODE 0x0014
#      elif CONFIG_DM320_OSD0_BLEND8THS == 5
#        define OSD0MODE_TRANSPMODE 0x001c
#      elif CONFIG_DM320_OSD0_BLEND8THS == 4
#        define OSD0MODE_TRANSPMODE 0x0024
#      elif CONFIG_DM320_OSD0_BLEND8THS == 3
#        define OSD0MODE_TRANSPMODE 0x002c
#      elif CONFIG_DM320_OSD0_BLEND8THS == 2
#        define OSD0MODE_TRANSPMODE 0x0034
#      elif CONFIG_DM320_OSD0_BLEND8THS == 0
#        define OSD0MODE_TRANSPMODE 0x003c
#      else
#        error "Invalid OSD0 transparency selection"
#      endif
#    else
#      define OSD0MODE_TRANSPMODE 0x0004
#    endif
#  else
#    define OSD0MODE_TRANSPMODE 0x0000
#  endif

#  ifdef CONFIG_DM320_OSD0_RGB16
#    define OSD0MODE_RGB16 0x2000
#  else
#    define OSD0MODE_RGB16 0x0000
#  endif

#  define INITIAL_OSD0MODE (0x00c1|OSD0MODE_TRANSPMODE|OSD0_FRAMEMODE|OSD0MODE_RGB16)
#endif

#ifndef CONFIG_DM320_OSD1_DISABLE
#  ifdef CONFIG_DM320_OSD1_FRAMEMODE
#    define OSD1MODE_FRAME 0x0002
#  else
#    define OSD1MODE_FRAME 0x0000
#  endif

#  ifdef CONFIG_DM320_OSD1_TRANSPMODE
#    ifdef CONFIG_DM320_OSD1_BLEND8THS
#      if CONFIG_DM320_OSD1_BLEND8THS == 8
#        define OSD1MODE_TRANSPMODE 0x0004
#      elif CONFIG_DM320_OSD1_BLEND8THS == 7
#        define OSD1MODE_TRANSPMODE 0x000c
#      elif CONFIG_DM320_OSD1_BLEND8THS == 6
#        define OSD1MODE_TRANSPMODE 0x0014
#      elif CONFIG_DM320_OSD1_BLEND8THS == 5
#        define OSD1MODE_TRANSPMODE 0x001c
#      elif CONFIG_DM320_OSD1_BLEND8THS == 4
#        define OSD1MODE_TRANSPMODE 0x0024
#      elif CONFIG_DM320_OSD1_BLEND8THS == 3
#        define OSD1MODE_TRANSPMODE 0x002c
#      elif CONFIG_DM320_OSD1_BLEND8THS == 2
#        define OSD1MODE_TRANSPMODE 0x0034
#      elif CONFIG_DM320_OSD1_BLEND8THS == 0
#        define OSD1MODE_TRANSPMODE 0x003c
#      else
#        error "Invalid OSD1 transparency selection"
#      endif
#    else
#      define OSD1MODE_TRANSPMODE 0x0004
#    endif
#  else
#    define OSD1MODE_TRANSPMODE 0x0000
#  endif

#  ifdef CONFIG_DM320_OSD1_RGB16
#    define OSD1MODE_RGB16 0x2000
#  else
#    define OSD1MODE_RGB16 0x0000
#  endif

#  ifdef CONFIG_DM32_OSD1_ATTRIB
#    define OSD1MODE_ATTRIB 0x8000
#  else
#    define OSD1MODE_ATTRIB 0x0000
#  endif

#  define INITIAL_OSD1MODE (0x00c1|OSD1MODE_TRANSPMODE|OSD1MODE_FRAME|OSD1MODE_RGB16|OSD1MODE_ATTRIB)
#endif

/* Rectangular cursor mode */

#ifdef CONFIG_FB_HWCURSOR
#  define DM320_RECTCURSOR_SETUP \
   ((CONFIG_DM320_CURSORLINEHEIGHT << 1) | \
    (CONFIG_DM320_CURSORLINEWIDTH <<4) | \
    (CONFIG_DM320_CURSORCLUT << 8))
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Initialization */

static int  dm320_allocvideomemory(void);
static void dm320_freevideomemory(void);
static void dm320_hwinitialize(void);

/* Framebuffer interface methods */

#ifndef CONFIG_DM320_VID0_DISABLE
static int dm320_getvid0videoinfo(FAR struct fb_vtable_s *vtable, FAR struct fb_videoinfo_s *vinfo);
static int dm320_getvid0planeinfo(FAR struct fb_vtable_s *vtable, int planeno, FAR struct fb_planeinfo_s *pinfo);
#endif
#ifndef CONFIG_DM320_VID1_DISABLE
static int dm320_getvid1videoinfo(FAR struct fb_vtable_s *vtable, FAR struct fb_videoinfo_s *vinfo);
static int dm320_getvid1planeinfo(FAR struct fb_vtable_s *vtable, int planeno, FAR struct fb_planeinfo_s *pinfo);
#endif
#ifndef CONFIG_DM320_OSD0_DISABLE
static int dm320_getosd0videoinfo(FAR struct fb_vtable_s *vtable, FAR struct fb_videoinfo_s *vinfo);
static int dm320_getosd0planeinfo(FAR struct fb_vtable_s *vtable, int planeno, FAR struct fb_planeinfo_s *pinfo);
#endif
#ifndef CONFIG_DM320_OSD1_DISABLE
static int dm320_getosd1videoinfo(FAR struct fb_vtable_s *vtable, FAR struct fb_videoinfo_s *vinfo);
static int dm320_getosd1planeinfo(FAR struct fb_vtable_s *vtable, int planeno, FAR struct fb_planeinfo_s *pinfo);
#endif
#if defined(CONFIG_FB_CMAP) && (!defined(CONFIG_DM320_OSD0_DISABLE) && !defined(CONFIG_DM320_OSD1_DISABLE))
static int dm320_getcmap(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap);
static int dm320_putcmap(FAR struct fb_vtable_s *vtable, FAR const struct fb_cmap_s *cmap);
#endif
#ifdef CONFIG_FB_HWCURSOR
static int dm320_getcursor(FAR struct fb_vtable_s *vtable, FAR struct fb_cursorattrib_s *attrib);
static int dm320_setcursor(FAR struct fb_vtable_s *vtable, FAR struct fb_setcursor_s *setttings);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* These are the addresses of allocated framebuffer memory regions */

#ifndef CONFIG_DM320_VID0_DISABLE
static FAR void *g_vid0base = 0;
#ifndef CONFIG_DM320_DISABLE_PINGPONG
static FAR void *g_vid0ppbase = 0;
#endif

static struct fb_vtable_s g_vid0vtable =
{
  .getvideoinfo = dm320_getvid0videoinfo,
  .getplaneinfo = dm320_getvid0planeinfo,
#ifdef CONFIG_FB_HWCURSOR
  .getcursor    = dm320_getcursor,
  .setcursor    = dm320_setcursor,
#endif
};

#endif

#ifndef CONFIG_DM320_VID1_DISABLE
static FAR void *g_vid1base = 0;

static struct fb_vtable_s g_vid1vtable =
{
  .getvideoinfo = dm320_getvid1videoinfo,
  .getplaneinfo = dm320_getvid1planeinfo,
#ifdef CONFIG_FB_HWCURSOR
  .getcursor    = dm320_getcursor,
  .setcursor    = dm320_setcursor,
#endif
};
#endif

#ifndef CONFIG_DM320_OSD0_DISABLE
static FAR void *g_osd0base = 0;
static struct fb_vtable_s g_osd0vtable =
{
  .getvideoinfo = dm320_getosd0videoinfo,
  .getplaneinfo = dm320_getosd0planeinfo,
#if defined(CONFIG_FB_CMAP) && (!defined(CONFIG_DM320_OSD0_DISABLE) && !defined(CONFIG_DM320_OSD1_DISABLE))
  .getcmap      = dm320_getcmap,
  .putcmap      = dm320_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor    = dm320_getcursor,
  .setcursor    = dm320_setcursor,
#endif
};

#endif

#ifndef CONFIG_DM320_OSD1_DISABLE
static FAR void *g_osd1base = 0;
static struct fb_vtable_s g_osd1vtable =
{
  .getvideoinfo = dm320_getosd1videoinfo,
  .getplaneinfo = dm320_getosd1planeinfo,
#if defined(CONFIG_FB_CMAP) && (!defined(CONFIG_DM320_OSD0_DISABLE) && !defined(CONFIG_DM320_OSD1_DISABLE))
  .getcmap      = dm320_getcmap,
  .putcmap      = dm320_putcmap,
#endif
#ifdef CONFIG_FB_HWCURSOR
  .getcursor    = dm320_getcursor,
  .setcursor    = dm320_setcursor,
#endif
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void dm320_blankscreen(uint8_t *buffer, int len)
{
  memset(buffer, 0xff, len);
}

static inline uint32_t dm320_physaddr(FAR void *fb_vaddr)
{
  return (uint32_t)fb_vaddr - DM320_SDRAM_VADDR;
}

#ifndef CONFIG_DM320_VID0_DISABLE
static inline uint32_t dm320_vid0upperoffset(void)
{
  return (((dm320_physaddr(g_vid0base) / 32) >> 16) & 0xff);
}

static inline uint32_t dm320_vid0loweroffset(void)
{
  return ((dm320_physaddr(g_vid0base) / 32) & 0xffff);
}

#ifndef CONFIG_DM320_DISABLE_PINGPONG
static inline uint32_t dm320_vid0ppupperoffset(void)
{
  return (((dm320_physaddr(g_vid0ppbase) / 32) >> 16) & 0xff);
}

static inline uint32_t dm320_vid0pploweroffset(void)
{
  return ((dm320_physaddr(g_vid0ppbase) / 32) & 0xffff);
}
#endif
#endif

#ifndef CONFIG_DM320_VID1_DISABLE
static inline uint32_t dm320_vid1upperoffset(void)
{
  return (((dm320_physaddr(g_vid1base) / 32) >> 16) & 0xff);
}

static inline uint32_t dm320_vid1loweroffset(void)
{
  return ((dm320_physaddr(g_vid1base) / 32) & 0xffff);
}
#endif

#ifndef CONFIG_DM320_OSD0_DISABLE
static inline uint32_t dm320_osd0upperoffset(void)
{
  return (((dm320_physaddr(g_osd0base) / 32) >> 16) & 0xff);
}

static inline uint32_t dm320_osd0loweroffset(void)
{
  return ((dm320_physaddr(g_osd0base) / 32) & 0xffff);
}
#endif

#ifndef CONFIG_DM320_OSD1_DISABLE
static inline uint32_t dm320_osd1upperoffset(void)
{
  return (((dm320_physaddr(g_osd1base) / 32) >> 16) & 0xff);
}

static inline uint32_t dm320_osd1loweroffset(void)
{
  return ((dm320_physaddr(g_osd1base) / 32) & 0xffff);
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * dm320_allocvideomemory
 ****************************************************************************/

static int dm320_allocvideomemory(void)
{
#ifndef CONFIG_DM320_VID0_DISABLE
#ifndef CONFIG_DM320_DISABLE_PINGPONG
  g_vid0base   = (FAR void *)malloc(2 * DM320_VID0_FBLEN);
  g_vid0ppbase = (FAR char*)g_vid0base + DM320_VID0_FBLEN;
#else
  g_vid0base   = (FAR void *)malloc(DM320_VID0_FBLEN);
#endif
  if (!g_vid0base)
    {
      goto errout;
    }
#endif

#ifndef CONFIG_DM320_VID1_DISABLE
  g_vid1base = (FAR void *)malloc(DM320_VID1_FBLEN);
  if (!g_vid1base)
    {
      goto errout;
    }
#endif

#ifndef CONFIG_DM320_OSD0_DISABLE
  g_osd0base = (FAR void *)malloc(DM320_OSD0_FBLEN);
  if (!g_osd0base)
    {
      goto errout;
    }
#endif

#ifndef CONFIG_DM320_OSD1_DISABLE
  g_osd1base = (FAR void *)malloc(DM320_OSD1_FBLEN);
  if (!g_osd1base)
    {
      goto errout;
    }
#endif

  return OK;

errout:
  dm320_freevideomemory();
  return -ENOMEM;
}

/****************************************************************************
 * Name: dm320_freevideomemory
 ****************************************************************************/

static void dm320_freevideomemory(void)
{
#ifndef CONFIG_DM320_VID0_DISABLE
  if (g_vid0base)
    {
      free(g_vid0base);
      g_vid0base = NULL;
#ifndef CONFIG_DM320_DISABLE_PINGPONG
      g_vid0ppbase = NULL;
#endif
    }
#endif

#ifndef CONFIG_DM320_VID1_DISABLE
  if (g_vid1base != 0)
    {
      free(g_vid1base);
      g_vid1base = NULL;
    }
#endif

#ifndef CONFIG_DM320_OSD0_DISABLE
  if (g_osd0base != 0)
    {
      free(g_osd0base);
      g_osd0base = NULL;
    }
#endif

#ifndef CONFIG_DM320_OSD1_DISABLE
  if (g_osd1base != 0)
    {
      free(g_osd1base);
      g_osd1base = NULL;
    }
#endif
}

/****************************************************************************
 * Name: dm320_disable
 ****************************************************************************/

static void dm320_disable(void)
{
  /* Disable all planes */

  gvdbg("Inactivate OSD:\n");

  putreg16(0, DM320_OSD_OSDWIN0MD); /* Win0 mode = 0 (1:active) */
  putreg16(0, DM320_OSD_OSDWIN1MD); /* Win1 mode = 0 (1:active) */
  putreg16(0, DM320_OSD_RECTCUR);   /* Rectangular cursor mode = 0 (1:active) */

  gvdbg("DM320_OSD_OSDWIN0MD:   %04x\n", getreg16(DM320_OSD_OSDWIN0MD));
  gvdbg("DM320_OSD_OSDWIN1MD:   %04x\n", getreg16(DM320_OSD_OSDWIN1MD));
  gvdbg("DM320_OSD_RECTCUR:     %04x\n", getreg16(DM320_OSD_RECTCUR));
}

/****************************************************************************
 * Name: dm320_hwinitialize
 ****************************************************************************/

static void dm320_hwinitialize(void)
{
  /* Disable all planes */

  dm320_disable();

  /* Initialize the main video to correct the origin */

  gvdbg("Setup main video origin:\n");

  putreg16(CONFIG_DM320_BASEX, DM320_OSD_BASEPX);
  putreg16(CONFIG_DM320_BASEY, DM320_OSD_BASEPY);

  gvdbg("DM320_OSD_BASEPX:      %04x\n", getreg16(DM320_OSD_BASEPX));
  gvdbg("DM320_OSD_BASEPY:      %04x\n", getreg16(DM320_OSD_BASEPY));

  /* Set up the frame buffer address registers */

  gvdbg("Setup framebuffer addresses:\n");


  putreg16(((dm320_osd1upperoffset() << 8) |
        dm320_osd0upperoffset()), DM320_OSD_OSDWINADH);
  putreg16(dm320_osd0loweroffset(), DM320_OSD_OSDWIN0ADL);
  putreg16(dm320_osd1loweroffset(), DM320_OSD_OSDWIN1ADL);

  gvdbg("DM320_OSD_OSDWINADH:   %04x\n", getreg16(DM320_OSD_OSDWINADH));
  gvdbg("DM320_OSD_OSDWIN0ADL:  %04x\n", getreg16(DM320_OSD_OSDWIN0ADL));
  gvdbg("DM320_OSD_OSDWIN1ADL:  %04x\n", getreg16(DM320_OSD_OSDWIN1ADL));

  /* Set up VID WIN0 */

#if defined(CONFIG_DM320_VID0_DISABLE) || defined(CONFIG_DM320_VID1_DISABLE)
  putreg16(((dm320_vid1upperoffset() << 8) | dm320_vid0upperoffset()), DM320_OSD_VIDWINADH);
#endif

#ifndef CONFIG_DM320_VID0_DISABLE
  gvdbg("Initialize video win0:\n");
  putreg16(dm320_vid0loweroffset(), DM320_OSD_VIDWIN0ADL);

  gvdbg("DM320_OSD_VIDWINADH:   %04x\n", getreg16(DM320_OSD_VIDWINADH));
  gvdbg("DM320_OSD_VIDWIN0ADL:  %04x\n", getreg16(DM320_OSD_VIDWIN0ADL));
  dm320_blankscreen((uint8_t *)g_vid0base, DM320_VID0_FBLEN);

#ifndef CONFIG_DM320_DISABLE_PINGPONG
  putreg16(dm320_vid0ppupperoffset(), DM320_OSD_PPVWIN0ADH);
  putreg16(dm320_vid0pploweroffset(), DM320_OSD_PPVWIN0ADL);

  gvdbg("DM320_OSD_PPVWIN0ADH:  %04x\n", getreg16(DM320_OSD_PPVWIN0ADH));
  gvdbg("DM320_OSD_PPVWIN0ADL:  %04x\n", getreg16(DM320_OSD_PPVWIN0ADL));
  dm320_blankscreen((uint8_t *)g_vid0ppbase, DM320_VID0_FBLEN);
#endif

  putreg16(CONFIG_DM320_VID0_XPOS, DM320_OSD_VIDWIN0XP);
  putreg16(CONFIG_DM320_VID0_YPOS, DM320_OSD_VIDWIN0YP);
  putreg16((CONFIG_DM320_VID0_XRES >> 4), DM320_OSD_VIDWIN0OFST);
  putreg16(CONFIG_DM320_VID0_XRES, DM320_OSD_VIDWIN0XL);
  putreg16(CONFIG_DM320_VID0_YRES, DM320_OSD_VIDWIN0YL);

  gvdbg("DM320_OSD_VIDWIN0XP:   %04x\n", getreg16(DM320_OSD_VIDWIN0XP));
  gvdbg("DM320_OSD_VIDWIN0YP:   %04x\n", getreg16(DM320_OSD_VIDWIN0YP));
  gvdbg("DM320_OSD_VIDWIN0OFST: %04x\n", getreg16(DM320_OSD_VIDWIN0OFST));
  gvdbg("DM320_OSD_VIDWIN0XL:   %04x\n", getreg16(DM320_OSD_VIDWIN0XL));
  gvdbg("DM320_OSD_VIDWIN0YL:   %04x\n", getreg16(DM320_OSD_VIDWIN0YL));
#endif

  /* Set up VID WIN1 */

#ifndef CONFIG_DM320_VID1_DISABLE
  gvdbg("Initialize video win1:\n");
  putreg16(dm320_vid1loweroffset(), DM320_OSD_VIDWIN1ADL);

  gvdbg("DM320_OSD_VIDWINADH:   %04x\n", getreg16(DM320_OSD_VIDWINADH));
  gvdbg("DM320_OSD_VIDWIN1ADL:  %04x\n", getreg16(DM320_OSD_VIDWIN1ADL));
  dm320_blankscreen((uint8_t *)g_vid1base, DM320_VID1_FBLEN);

  putreg16(CONFIG_DM320_VID1_XPOS, DM320_OSD_VIDWIN1XP);
  putreg16(CONFIG_DM320_VID1_XPOS, DM320_OSD_VIDWIN1YP);
  putreg16((CONFIG_DM320_VID1_XRES >> 4), DM320_OSD_VIDWIN1OFST);
  putreg16(CONFIG_DM320_VID1_XRES, DM320_OSD_VIDWIN1XL);
  putreg16(CONFIG_DM320_VID1_YRES, DM320_OSD_VIDWIN1YL);

  gvdbg("DM320_OSD_VIDWIN1XP:   %04x\n", getreg16(DM320_OSD_VIDWIN1XP));
  gvdbg("DM320_OSD_VIDWIN1YP:   %04x\n", getreg16(DM320_OSD_VIDWIN1YP));
  gvdbg("DM320_OSD_VIDWIN1OFST: %04x\n", getreg16(DM320_OSD_VIDWIN1OFST));
  gvdbg("DM320_OSD_VIDWIN1XL:   %04x\n", getreg16(DM320_OSD_VIDWIN1XL));
  gvdbg("DM320_OSD_VIDWIN1YL:   %04x\n", getreg16(DM320_OSD_VIDWIN1YL));
#endif

  putreg16(DM320_VIDMODE, DM320_OSD_VIDWINMD);
  gvdbg("DM320_OSD_VIDWINMD:    %04x\n", getreg16(DM320_OSD_VIDWINMD));

  /* Set up OSD WIN0 */

#ifndef CONFIG_DM320_OSD0_DISABLE
  gvdbg("Initialize OSD win0:\n");
  dm320_blankscreen((uint8_t *)g_osd0base, DM320_OSD0_FBLEN);

  putreg16(CONFIG_DM320_OSD0_XPOS, DM320_OSD_OSDWIN0XP);
  putreg16(CONFIG_DM320_OSD0_YPOS, DM320_OSD_OSDWIN0YP);
#ifdef CONFIG_DM320_OSD1_RGB16
  putreg16((CONFIG_DM320_OSD0_XRES >> 4), DM320_OSD_OSDWIN0OFST);
#else
  putreg16((CONFIG_DM320_OSD0_XRES >> 5), DM320_OSD_OSDWIN0OFST);
#endif
  putreg16(CONFIG_DM320_OSD0_XRES, DM320_OSD_OSDWIN0XL);
  putreg16(CONFIG_DM320_OSD0_YRES, DM320_OSD_OSDWIN0YL);
  putreg16(INITIAL_OSD0MODE, DM320_OSD_OSDWIN0MD);

  gvdbg("DM320_OSD_OSDWIN0XP:   %04x\n", getreg16(DM320_OSD_OSDWIN0XP));
  gvdbg("DM320_OSD_OSDWIN0YP:   %04x\n", getreg16(DM320_OSD_OSDWIN0YP));
  gvdbg("DM320_OSD_OSDWIN0OFST: %04x\n", getreg16(DM320_OSD_OSDWIN0OFST));
  gvdbg("DM320_OSD_OSDWIN0XL:   %04x\n", getreg16(DM320_OSD_OSDWIN0XL));
  gvdbg("DM320_OSD_OSDWIN0YL:   %04x\n", getreg16(DM320_OSD_OSDWIN0YL));
  gvdbg("DM320_OSD_OSDWIN0MD:   %04x\n", getreg16(DM320_OSD_OSDWIN0MD));
#endif

  /* Set up OSD WIN1 */

#ifndef CONFIG_DM320_OSD1_DISABLE
  gvdbg("Initialize OSD win1\n");
  dm320_blankscreen((uint8_t *)g_osd1base, DM320_OSD1_FBLEN);

  putreg16(CONFIG_DM320_OSD1_XPOS, DM320_OSD_OSDWIN1XP);
  putreg16(CONFIG_DM320_OSD1_YPOS, DM320_OSD_OSDWIN1YP);
#ifdef CONFIG_DM320_OSD1_RGB16
  putreg16((CONFIG_DM320_OSD1_XRES >> 4), DM320_OSD_OSDWIN1OFST);
#else
  putreg16((CONFIG_DM320_OSD1_XRES >> 5), DM320_OSD_OSDWIN1OFST);
#endif
  putreg16(CONFIG_DM320_OSD1_XRES, DM320_OSD_OSDWIN1XL);
  putreg16(CONFIG_DM320_OSD1_YRES, DM320_OSD_OSDWIN1YL);
  putreg16(INITIAL_OSD1MODE, DM320_OSD_OSDWIN1MD);

  gvdbg("DM320_OSD_OSDWIN1XP:   %04x\n", getreg16(DM320_OSD_OSDWIN1XP));
  gvdbg("DM320_OSD_OSDWIN1YP:   %04x\n", getreg16(DM320_OSD_OSDWIN1YP));
  gvdbg("DM320_OSD_OSDWIN1OFST: %04x\n", getreg16(DM320_OSD_OSDWIN1OFST));
  gvdbg("DM320_OSD_OSDWIN1XL:   %04x\n", getreg16(DM320_OSD_OSDWIN1XL));
  gvdbg("DM320_OSD_OSDWIN1YL:   %04x\n", getreg16(DM320_OSD_OSDWIN1YL));
  gvdbg("DM320_OSD_OSDWIN1MD:   %04x\n", getreg16(DM320_OSD_OSDWIN1MD));
#endif

  /* Set up the rectangular cursor with defaults */

#ifdef CONFIG_FB_HWCURSOR
  gdbg("Initialize rectangular cursor\n");

  putreg16(0, DM320_OSD_CURXP);
  putreg16(0, DM320_OSD_CURYP);
  putreg16(CONFIG_DM320_RECTCURSOR_WIDTH, DM320_OSD_CURXL);
  putreg16(CONFIG_DM320_RECTCURSOR_HEIGHT, DM320_OSD_CURYL);

  /* DM320_RECTCURSOR_SETUP:
   *
   * Bit 0: 0=rectangular cursor inactive 1=on             0
   * Bits 113: Vertical line height: {1,2,4,6,8,10,12,14}  CONFIG_DM320_CURSORLINEHEIGHT
   * 4:6: Horizontal line width: {1,4,8,16,20,24,28}       CONFIG_DM320_CURSORLINEWIDTH
   * 7: 0=ROM lookup table, 1=RAM lookup table             0
   * 8:15: Rectangular cursor color pallette address       CONFIG_DM320_CURSORCLUT
   */

  putreg16(DM320_RECTCURSOR_SETUP, DM320_OSD_RECTCUR);

  gvdbg("DM320_OSD_CURXP:       %04x\n", getreg16(DM320_OSD_CURXP));
  gvdbg("DM320_OSD_CURYP:       %04x\n", getreg16(DM320_OSD_CURYP));
  gvdbg("DM320_OSD_CURXL:       %04x\n", getreg16(DM320_OSD_CURXL));
  gvdbg("DM320_OSD_CURYL:       %04x\n", getreg16(DM320_OSD_CURYL));
  gvdbg("DM320_OSD_RECTCUR:     %04x\n", getreg16(DM320_OSD_RECTCUR));
#endif

  /* Set main window to the hardware default state.  That initial
   * state is:
   *
   * FIELD                                         SETTING
   * Bits 0-7: background color clut               CONFIG_DM320_BKGDCLUT
   * Bit 8: background clut 0=ROM 1=RAM            0
   * Bit 9: field signal 0=normal 1=inverted       0
   * Bit 10: vid window H expansion: 1=x9/8        0
   * Bit 11: vid window V expansion: 1=x6/5        0
   * Bit 12: expansion filter 0=off 1=on           0
   * Bit 13: osd window H expansion: 1=x9/8        0
   * Bit 14: osd window V expansion: 1=x6/5        0
   * Bit 1515: 0=offset binary, 1=complement of 2  0
   */

  putreg16(CONFIG_DM320_BKGDCLUT, DM320_OSD_OSDMODE);
  gvdbg("DM320_OSD_OSDMODE:     %04x\n", getreg16(DM320_OSD_OSDMODE));
}

/****************************************************************************
 * Name: dm320_getvid0videoinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_VID0_DISABLE
static int dm320_getvid0videoinfo(FAR struct fb_vtable_s *vtable,
                                  FAR struct fb_videoinfo_s *vinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !vinfo)
    {
      return -EINVAL;
    }
#endif

  vinfo->fmt     = FB_FMT_UYVY;
  vinfo->xres    = CONFIG_DM320_VID0_XRES;
  vinfo->yres    = CONFIG_DM320_VID0_YRES;
  vinfo->nplanes = 1;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getvid0planeinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_VID0_DISABLE
static int dm320_getvid0planeinfo(FAR struct fb_vtable_s *vtable, int planeno,
                                  FAR struct fb_planeinfo_s *pinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !pinfo)
    {
      return -EINVAL;
    }
#endif

  pinfo->fbmem  = g_vid0base;
  pinfo->fblen  = DM320_VID0_FBLEN;
  pinfo->stride = DM320_VID0_STRIDE;
  pinfo->bpp    = DM320_VID0_BPP;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getvid1videoinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_VID1_DISABLE
static int dm320_getvid1videoinfo(FAR struct fb_vtable_s *vtable,
                                  FAR struct fb_videoinfo_s *vinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !vinfo)
    {
      return -EINVAL;
    }
#endif

  vinfo->fmt     = FB_FMT_UYVY;
  vinfo->xres    = CONFIG_DM320_VID1_XRES;
  vinfo->yres    = CONFIG_DM320_VID1_YRES;
  vinfo->nplanes = 1;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getvid1planeinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_VID1_DISABLE
static int dm320_getvid1planeinfo(FAR struct fb_vtable_s *vtable, int planeno,
                                  FAR struct fb_planeinfo_s *pinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !pinfo)
    {
      return -EINVAL;
    }
#endif

  pinfo->fbmem  = g_vid1base;
  pinfo->fblen  = DM320_VID1_FBLEN;
  pinfo->stride = DM320_VID1_STRIDE;
  pinfo->bpp    = DM320_VID1_BPP;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getosd0osdeoinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_OSD0_DISABLE
static int dm320_getosd0videoinfo(FAR struct fb_vtable_s *vtable,
                                  FAR struct fb_videoinfo_s *vinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !vinfo)
    {
      return -EINVAL;
    }
#endif

#ifdef CONFIG_DM320_OSD0_RGB16
  vinfo->fmt     = FB_FMT_RGB16_565;
#else
  vinfo->fmt     = FB_FMT_RGB8;
#endif
  vinfo->xres    = CONFIG_DM320_OSD0_XRES;
  vinfo->yres    = CONFIG_DM320_OSD0_YRES;
  vinfo->nplanes = 1;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getosd0planeinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_OSD0_DISABLE
static int dm320_getosd0planeinfo(FAR struct fb_vtable_s *vtable, int planeno,
                                  FAR struct fb_planeinfo_s *pinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !pinfo)
    {
      return -EINVAL;
    }
#endif

  pinfo->fbmem  = g_osd0base;
  pinfo->fblen  = DM320_OSD0_FBLEN;
  pinfo->stride = DM320_OSD0_STRIDE;
  pinfo->bpp    = DM320_OSD0_BPP;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getosd1osdeoinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_OSD1_DISABLE
static int dm320_getosd1videoinfo(FAR struct fb_vtable_s *vtable,
                                  FAR struct fb_videoinfo_s *vinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !vinfo)
    {
      return -EINVAL;
    }
#endif

#ifdef CONFIG_DM320_OSD1_RGB16
  vinfo->fmt     = FB_FMT_RGB16_565;
#else
  vinfo->fmt     = FB_FMT_RGB8;
#endif
  vinfo->xres    = CONFIG_DM320_OSD1_XRES;
  vinfo->yres    = CONFIG_DM320_OSD1_YRES;
  vinfo->nplanes = 1;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getosd1planeinfo
 ****************************************************************************/

#ifndef CONFIG_DM320_OSD1_DISABLE
static int dm320_getosd1planeinfo(FAR struct fb_vtable_s *vtable, int planeno,
                                  FAR struct fb_planeinfo_s *pinfo)
{
#ifdef CONFIG_DEBUG
  if (!vtable || !pinfo)
    {
      return -EINVAL;
    }
#endif

  pinfo->fbmem  = g_osd1base;
  pinfo->fblen  = DM320_OSD1_FBLEN;
  pinfo->stride = DM320_OSD1_STRIDE;
  pinfo->bpp    = DM320_OSD1_BPP;
  return OK;
}
#endif

/****************************************************************************
 * Name: dm320_getcmap
 ****************************************************************************/

#if defined(CONFIG_FB_CMAP) && (!defined(CONFIG_DM320_OSD0_DISABLE) && !defined(CONFIG_DM320_OSD1_DISABLE))
static int dm320_getcmap(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap)
{
  /* I don't think the RAM clut is readable */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: dm320_putcmap
 ****************************************************************************/

#if defined(CONFIG_FB_CMAP) && (!defined(CONFIG_DM320_OSD0_DISABLE) && !defined(CONFIG_DM320_OSD1_DISABLE))
static int dm320_putcmap(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap)
{
  irqstate_t flags;
  uint16_t regval;
  uint8_t y;
  uint8_t u;
  uint8_t v;
  int len
  int i;

#ifdef CONFIG_DEBUG
  if (!vtable || !cmap || !cmap->read || !cmap->green || !cmap->blue)
    {
      return -EINVAL;
    }
#endif

  flags = irqsave();
  for (i = cmap.first, len = 0; i < 256 && len < cmap.len, i++, len++)
    {
       /* Convert the RGB to YUV */

       nxgl_rgb2yuv(cmap->red[i], cmap->green[i], cmap->blue[i], &y, &u, &v);

       /* Program the CLUT */

       while (getreg16(DM320_OSD_MISCCTL) & 0x8);
       putreg16(((uint16_t)y) << 8 | uint16_t(u)), DM320_OSD_CLUTRAMYCB);
       putreg16(((uint16_t)v << 8 | i), DM320_OSD_CLUTRAMCR);
    }

  /* Select RAM clut */

#if !defined(CONFIG_DM320_OSD0_DISABLE) && !defined(CONFIG_DM320_OSD0_RGB16)
  regval = getreg16(DM320_OSD_OSDWIN0MD);
  regval |= 0x1000;
  putreg16(regval, DM320_OSD_OSDWIN0MD);
#endif

#if !defined(CONFIG_DM320_OSD1_DISABLE) && !defined(CONFIG_DM320_OSD1_RGB16)
  regval = getreg16(DM320_OSD_OSDWIN1MD);
  regval |= 0x1000;
  putreg16(regval, DM320_OSD_OSDWIN1MD);
#endif

  return 0;
}
#endif

/****************************************************************************
 * Name: dm320_getcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int dm320_getcursor(FAR struct fb_vtable_s *vtable, FAR struct fb_cursorattrib_s *attrib)
{
  irqstate_t flags;

#ifdef CONFIG_DEBUG
  if (!vtable || !attrib)
    {
      return -EINVAL;
    }
#endif

  flags = irqsave();
  attrib->pos.x = getreg16(DM320_OSD_CURXP);
  attrib->pos.y = getreg16(DM320_OSD_CURYP);

#ifdef CONFIG_FB_HWCURSORSIZE
  attrib->size.w = getreg16(DM320_OSD_CURXL);
  attrib->size.h = getreg16(DM320_OSD_CURYL);
#endif
  irqrestore();

  attrib->mxsize.w = MAX_XRES;
  attrib->mxsize.h = MAX_YRES;

  gvdbg("DM320_OSD_CURXP:       %04x\n", attrib->pos.x);
  gvdbg("DM320_OSD_CURYP:       %04x\n", attrib->pos.y);
#ifdef CONFIG_FB_HWCURSORSIZE
  gvdbg("DM320_OSD_CURXL:       %04x\n", attrib->size.w);
  gvdbg("DM320_OSD_CURYL:       %04x\n", attrib->size.h);
#else
  gvdbg("DM320_OSD_CURXL:       %04x\n", getreg16(DM320_OSD_CURXL));
  gvdbg("DM320_OSD_CURYL:       %04x\n", getreg16(DM320_OSD_CURYL));
#endif
  gvdbg("DM320_OSD_RECTCUR:     %04x\n", getreg16(DM320_OSD_RECTCUR));
}
#endif

/****************************************************************************
 * Name: dm320_setcursor
 ****************************************************************************/

#ifdef CONFIG_FB_HWCURSOR
static int dm320_setcursor(FAR struct fb_vtable_s *vtable, FAR struct fb_setcursor_s *settings)
{
  irqstate_t flags;
  uint16_t regval;

#ifdef CONFIG_DEBUG
  if (!vtable || !settings)
    {
      return -EINVAL;
    }
#endif

  /* Set cursor position */

  flags = irqsave();
  if ((settings->flags & FB_CUR_SETPOSITION) != 0)
    {
      gvdbg("x=%d y=%d\n", settings->pos.x, settings->pos.y);

     if (settings->pos.x > MAX_YRES)
       {
          settings->pos.x = MAX_YRES;
       }

     if (settings->pos.y > MAX_YRES)
       {
          settings->pos.y = MAX_YRES;
       }

     putreg16(settings->pos.x, DM320_OSD_CURXP);
     putreg16(settings->pos.y, DM320_OSD_CURYP);
   }

#ifdef CONFIG_FB_HWCURSORSIZE
  if ((settings->flags & FB_CUR_SETSIZE) != 0)
    {
      gvdbg("h=%d w=%d\n", settings->size.h, settings->size.w);

     if (settings->size.w > MAX_YRES)
       {
          settings->size.w = MAX_YRES;
       }

     if (settings->size.h > MAX_YRES)
       {
          settings->size.h = MAX_YRES;
       }

     flags = irqsave();
     putreg16(settings->size.w, DM320_OSD_CURXL);
     putreg16(settings->size.h, DM320_OSD_CURYL);
     restore_flags(flags);
   }
#endif

  regval = getreg16(DM320_OSD_RECTCUR);
  if ((settings->flags & FB_CUR_ENABLE) != 0)
    {
      regval |= 1;
    }
  else
    {
      regval &= ~1;
    }
  putreg16(regval, DM320_OSD_RECTCUR);
  restore_flags(flags);

  gvdbg("DM320_OSD_CURXP:       %04x\n", getreg16(DM320_OSD_CURXP));
  gvdbg("DM320_OSD_CURYP:       %04x\n", getreg16(DM320_OSD_CURYP));
  gvdbg("DM320_OSD_CURXL:       %04x\n", getreg16(DM320_OSD_CURXL));
  gvdbg("DM320_OSD_CURYL:       %04x\n", getreg16(DM320_OSD_CURYL));
  gvdbg("DM320_OSD_RECTCUR:     %04x\n", getreg16(DM320_OSD_RECTCUR));
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
  int ret;

  gvdbg("Allocating framebuffers\n");
  ret = dm320_allocvideomemory();
  if (ret != 0)
    {
      gdbg("Failed to allocate video buffers\n");
      return ret;
    }

  /* Initialize the hardware */

  gvdbg("Initializing hardware\n");
  dm320_hwinitialize();
  return 0;
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
  switch (vplane)
    {
#ifndef CONFIG_DM320_VID0_DISABLE
      case DM320_VIDWIN0: /* VID0 window */
        return &g_vid0vtable;
#endif
#ifndef CONFIG_DM320_VID1_DISABLE
      case DM320_VIDWIN1: /* VID1 window */
        return &g_vid1vtable;
#endif
#ifndef CONFIG_DM320_OSD0_DISABLE
      case DM320_OSDWIN0: /* OSD2 window */
        return &g_osd0vtable;
#endif
#ifndef CONFIG_DM320_OSD1_DISABLE
      case DM320_OSDWIN1: /* OSD2 window */
        return &g_osd1vtable;
#endif
      default:
        break;
    }
  return NULL;
}

/****************************************************************************
 * Name: up_fbteardown
 ****************************************************************************/

void fb_teardown(void)
{
  /* Disable the hardware */

  dm320_disable();

  /* Free the video buffers */

  dm320_freevideomemory();
}
