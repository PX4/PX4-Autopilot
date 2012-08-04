/****************************************************************************
 * include/nuttx/lcd/lcd.h
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_LCD_H
#define __INCLUDE_NUTTX_LCD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <nuttx/fb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Friendlier names */

#define LCD_FULL_OFF     (0)
#define LCD_FULL_ON      CONFIG_LCD_MAXPOWER

#define LCD_MIN_CONTRAST (0)
#define LCD_MAX_CONTRAST CONFIG_LCD_MAXCONTRAST

/****************************************************************************
 * Type Definitions
 ****************************************************************************/

/* This structure describes one color plane.  Some YUV formats may support
 * up to 4 planes (although they probably wouldn't be used on LCD hardware).
 * The framebuffer driver provides the video memory address in its
 * corresponding fb_planeinfo_s structure.  The LCD driver, instead, provides
 * methods to transfer data to/from the LCD color plane.
 */

struct lcd_planeinfo_s
{
  /* LCD Data Transfer ******************************************************/
  /* This method can be used to write a partial raster line to the LCD:
   *
   *  row     - Starting row to write to (range: 0 <= row < yres)
   *  col     - Starting column to write to (range: 0 <= col <= xres-npixels)
   *  buffer  - The buffer containing the run to be written to the LCD
   *  npixels - The number of pixels to write to the LCD
   *            (range: 0 < npixels <= xres-col)
   */

  int (*putrun)(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                size_t npixels);

  /* This method can be used to read a partial raster line from the LCD:
   *
   *  row     - Starting row to read from (range: 0 <= row < yres)
   *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
   *  buffer  - The buffer in which to return the run read from the LCD
   *  npixels - The number of pixels to read from the LCD
   *            (range: 0 < npixels <= xres-col)
   */

  int (*getrun)(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                size_t npixels);

  /* Plane color characteristics ********************************************/

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

  uint8_t *buffer; 

  /* This is the number of bits in one pixel.  This may be one of {1, 2, 4,
   * 8, 16, 24, or 32} unless support for one or more of those resolutions
   * has been disabled.
   */

  uint8_t  bpp;
};

/* This structure defines an LCD interface */

struct lcd_dev_s
{
  /* LCD Configuration ******************************************************/
  /* Get information about the LCD video controller configuration and the
   * configuration of each LCD color plane.
   */

  int (*getvideoinfo)(FAR struct lcd_dev_s *dev,
         FAR struct fb_videoinfo_s *vinfo);
  int (*getplaneinfo)(FAR struct lcd_dev_s *dev, unsigned int planeno,
         FAR struct lcd_planeinfo_s *pinfo);

  /* LCD RGB Mapping ********************************************************/
  /* The following are provided only if the video hardware supports RGB color
   * mapping
   */

#ifdef CONFIG_FB_CMAP
  int (*getcmap)(FAR struct lcd_dev_s *dev, FAR struct fb_cmap_s *cmap);
  int (*putcmap)(FAR struct lcd_dev_s *dev,
         FAR const struct fb_cmap_s *cmap);
#endif

  /* Cursor Controls ********************************************************/
  /* The following are provided only if the video hardware supports a
   * hardware cursor
   */

#ifdef CONFIG_FB_HWCURSOR
  int (*getcursor)(FAR struct lcd_dev_s *dev,
        FAR struct fb_cursorattrib_s *attrib);
  int (*setcursor)(FAR struct lcd_dev_s *dev,
        FAR struct fb_setcursor_s *settings);
#endif

  /* LCD Specific Controls **************************************************/
  /* Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full
   * on).  On backlit LCDs, this setting may correspond to the backlight
   * setting.
   */

  int (*getpower)(struct lcd_dev_s *dev);

  /* Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full
   * on).  On backlit LCDs, this setting may correspond to the backlight
   * setting.
   */

  int (*setpower)(struct lcd_dev_s *dev, int power);

  /* Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST) */

  int (*getcontrast)(struct lcd_dev_s *dev);

  /* Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST) */

  int (*setcontrast)(struct lcd_dev_s *dev, unsigned int contrast);
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_lcdinitialize, up_lcdgetdev, up_lcduninitialize
 *
 * Description:
 *   If an architecture supports a parallel or serial LCD, then it must
 *   provide APIs to access the LCD as follows:
 *
 *   up_lcdinitialize   - Initialize the LCD video hardware.  The initial
 *                        state of the LCD is fully initialized, display
 *                        memory cleared, and the LCD ready to use, but with
 *                        the power setting at 0 (full off).
 *   up_lcdgetdev       - Return a a reference to the LCD object for
 *                        the specified LCD.  This allows support for
 *                        multiple LCD devices.
 *   up_lcduninitialize - Unitialize the LCD support
 *
 ***************************************************************************/

EXTERN int up_lcdinitialize(void);
EXTERN FAR struct lcd_dev_s *up_lcdgetdev(int lcddev);
EXTERN void up_lcduninitialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_LCD_H */
