/****************************************************************************
 * arch/sim/src/up_lcd.c
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

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lcd/lcd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 100
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 100
#endif

/* Simulated LCD geometry and color format */

#ifndef CONFIG_SIM_FBWIDTH
#  define CONFIG_SIM_FBWIDTH  320 /* Framebuffer width in pixels */
#endif

#ifndef CONFIG_SIM_FBHEIGHT
#  define CONFIG_SIM_FBHEIGHT 240 /* Framebuffer height in pixels */
#endif

#ifndef CONFIG_SIM_FBBPP
#  define CONFIG_SIM_FBBPP    16  /* Framebuffer bytes per pixel (RGB) */
#endif

#define FB_STRIDE ((CONFIG_SIM_FBBPP * CONFIG_SIM_FBWIDTH + 7) >> 3)

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

/* Debug ********************************************************************/
/* Define CONFIG_DEBUG_LCD to enable detailed LCD debug output. Verbose debug must
 * also be enabled.
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#  undef CONFIG_DEBUG_LCD
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

#ifdef CONFIG_DEBUG_LCD
# define lcddbg(format, arg...)  vdbg(format, ##arg)
#else
# define lcddbg(x...)
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct sim_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t power;        /* Current power setting */
};

/****************************************************************************
 * Private Function Protototypes
 ****************************************************************************/

/* LCD Data Transfer Methods */

static int sim_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                      size_t npixels);
static int sim_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels);

/* LCD Configuration */

static int sim_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo);
static int sim_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                            FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int sim_getpower(struct lcd_dev_s *dev);
static int sim_setpower(struct lcd_dev_s *dev, int power);
static int sim_getcontrast(struct lcd_dev_s *dev);
static int sim_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefore be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint8_t g_runbuffer[FB_STRIDE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = FB_FMT,               /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = CONFIG_SIM_FBWIDTH,   /* Horizontal resolution in pixel columns */
  .yres    = CONFIG_SIM_FBHEIGHT,  /* Vertical resolution in pixel rows */
  .nplanes = 1,                    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = sim_putrun,            /* Put a run into LCD memory */
  .getrun = sim_getrun,            /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = CONFIG_SIM_FBBPP,      /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct sim_dev_s g_lcddev = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = sim_getvideoinfo,
    .getplaneinfo = sim_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = sim_getpower,
    .setpower     = sim_setpower,
    .getcontrast  = sim_getcontrast,
    .setcontrast  = sim_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sim_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int sim_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  lcddbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  return OK;
}

/****************************************************************************
 * Name:  sim_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int sim_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  lcddbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sim_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int sim_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sim_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int sim_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sim_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on). On backlit LCDs, this setting may correspond to the backlight setting.
 *
 ****************************************************************************/

static int sim_getpower(struct lcd_dev_s *dev)
{
  gvdbg("power: %d\n", 0);
  return g_lcddev.power;
}

/****************************************************************************
 * Name:  sim_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sim_setpower(struct lcd_dev_s *dev, int power)
{
  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  g_lcddev.power = power;
  return OK;
}

/****************************************************************************
 * Name:  sim_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sim_getcontrast(struct lcd_dev_s *dev)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sim_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sim_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  gvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int up_lcdinitialize(void)
{
  gvdbg("Initializing\n");
  return OK;
}

/****************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/****************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the LCD support
 *
 ****************************************************************************/

void up_lcduninitialize(void)
{
}

