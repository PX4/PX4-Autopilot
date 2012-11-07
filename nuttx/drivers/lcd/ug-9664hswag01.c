/**************************************************************************************
 * drivers/lcd/ug-9664hswag01.c
 * Driver for the Univision UG-9664HSWAG01 Display with the Solomon Systech SSD1305 LCD
 * controller.
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Reference: "Product Specification, OEL Display Module, UG-9664HSWAG01", Univision
 *            Technology Inc., SAS1-6020-B, January 3, 2008.
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
 **************************************************************************************/

/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/ug-9664hswag01.h>

#include "ssd1305.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/
/* UG-9664HSWAG01 Configuration Settings:
 *
 * CONFIG_UG9664HSWAG01_SPIMODE - Controls the SPI mode
 * CONFIG_UG9664HSWAG01_FREQUENCY - Define to use a different bus frequency
 * CONFIG_UG9664HSWAG01_NINTERFACES - Specifies the number of physical
 *   UG-9664HSWAG01 devices that will be supported.  NOTE:  At present, this
 *   must be undefined or defined to be 1.
 * CONFIG_UG9664HSWAG01_POWER
 *   If the hardware supports a controllable OLED a power supply, this
 *   configuration shold be defined.  (See ug_power() below).
 * 
 * Required LCD driver settings:
 * CONFIG_LCD_UG9664HSWAG01 - Enable UG-9664HSWAG01 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 * CONFIG_LCD_MAXPOWER should be 2:  0=off, 1=dim, 2=normal
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

/* Verify that all configuration requirements have been met */

/* The UG-9664HSWAG01 spec says that is supports SPI mode 0,0 only.  However, somtimes
 * you need to tinker with these things.
 */

#ifndef CONFIG_UG9664HSWAG01_SPIMODE
#  define CONFIG_UG9664HSWAG01_SPIMODE SPIDEV_MODE0
#endif

/* SPI frequency */

#ifndef CONFIG_UG9664HSWAG01_FREQUENCY
#  define CONFIG_UG9664HSWAG01_FREQUENCY 3500000
#endif

/* CONFIG_UG9664HSWAG01_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_UG9664HSWAG01_NINTERFACES
#  define CONFIG_UG9664HSWAG01_NINTERFACES 1
#endif

#if CONFIG_UG9664HSWAG01_NINTERFACES != 1
#  warning "Only a single UG-9664HSWAG01 interface is supported"
#  undef CONFIG_UG9664HSWAG01_NINTERFACES
#  define CONFIG_UG9664HSWAG01_NINTERFACES 1
#endif

/* Verbose debug must also be enabled to use the extra OLED debug */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_LCD
#endif

/* Check contrast selection */

#ifndef CONFIG_LCD_MAXCONTRAST
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#if CONFIG_LCD_MAXCONTRAST <= 0 || CONFIG_LCD_MAXCONTRAST > 255
#  error "CONFIG_LCD_MAXCONTRAST exceeds supported maximum"
#endif

#if CONFIG_LCD_MAXCONTRAST < 255
#  warning "Optimal setting of CONFIG_LCD_MAXCONTRAST is 255"
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 2
#endif

#if CONFIG_LCD_MAXPOWER != 2
#  warning "CONFIG_LCD_MAXPOWER should be 2"
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 2
#endif

/* The OLED requires CMD/DATA SPI support */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be defined in your NuttX configuration"
#endif

/* Color is 1bpp monochrome with leftmost column contained in bits 0  */

#ifdef CONFIG_NX_DISABLE_1BPP
#  warning "1 bit-per-pixel support needed"
#endif

/* Color Properties *******************************************************************/
/* The SSD1305 display controller can handle a resolution of 132x64. The OLED
 * on the base board is 96x64.
 */

#define UG_DEV_XRES     132
#define UG_XOFFSET      18

/* Display Resolution */

#define UG_XRES         96
#define UG_YRES         64

/* Color depth and format */

#define UG_BPP          1
#define UG_COLORFMT     FB_FMT_Y1

/* Bytes per logical row and actual device row */

#define UG_XSTRIDE      (UG_XRES >> 3) /* Pixels arrange "horizontally for user" */
#define UG_YSTRIDE      (UG_YRES >> 3) /* But actual device arrangement is "vertical" */

/* The size of the shadow frame buffer */

#define UG_FBSIZE       (UG_XRES * UG_YSTRIDE)

/* Bit helpers */

#define LS_BIT          (1 << 0)
#define MS_BIT          (1 << 7)

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
# define lcddbg(format, arg...)  vdbg(format, ##arg)
#else
# define lcddbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct ug_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi;
  uint8_t contrast;
  uint8_t powered;

 /* The SSD1305 does not support reading from the display memory in SPI mode.
  * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
  * a shadow copy of the framebuffer memory.
  */

  uint8_t fb[UG_FBSIZE];
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void ug_select(FAR struct spi_dev_s *spi);
static inline void ug_deselect(FAR struct spi_dev_s *spi);
#else
static void ug_select(FAR struct spi_dev_s *spi);
static void ug_deselect(FAR struct spi_dev_s *spi);
#endif

/* LCD Data Transfer Methods */

static int ug_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                     size_t npixels);
static int ug_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                     size_t npixels);

/* LCD Configuration */

static int ug_getvideoinfo(FAR struct lcd_dev_s *dev,
                           FAR struct fb_videoinfo_s *vinfo);
static int ug_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int ug_getpower(struct lcd_dev_s *dev);
static int ug_setpower(struct lcd_dev_s *dev, int power);
static int ug_getcontrast(struct lcd_dev_s *dev);
static int ug_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline void up_clear(FAR struct ug_dev_s  *priv);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

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

static uint8_t g_runbuffer[UG_XSTRIDE+1];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = UG_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = UG_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = UG_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,              /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = ug_putrun,             /* Put a run into LCD memory */
  .getrun = ug_getrun,             /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = UG_BPP,                /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct ug_dev_s g_ugdev = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = ug_getvideoinfo,
    .getplaneinfo = ug_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = ug_getpower,
    .setpower     = ug_setpower,
    .getcontrast  = ug_getcontrast,
    .setcontrast  = ug_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ug_powerstring
 *
 * Description:
 *   Convert the power setting to a string.
 *
 **************************************************************************************/


static inline FAR const char *ug_powerstring(uint8_t power)
{
  if (power == UG_POWER_OFF)
    {
      return "OFF";
    }
  else if (power == UG_POWER_DIM)
    {
      return "DIM";
    }
  else if (power == UG_POWER_ON)
    {
      return "ON";
    }
  else
    {
      return "ERROR";
    }
}

/**************************************************************************************
 * Function: ug_select
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ug_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, true);
}
#else
static void ug_select(FAR struct spi_dev_s *spi)
{
  /* Select UG-9664HSWAG01 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Now make sure that the SPI bus is configured for the UG-9664HSWAG01 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_UG9664HSWAG01_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_UG9664HSWAG01_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_UG9664HSWAG01_FREQUENCY);
#endif
}
#endif

/**************************************************************************************
 * Function: ug_deselect
 *
 * Description:
 *   De-select the SPI
 *
 * Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ug_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
}
#else
static void ug_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select UG-9664HSWAG01 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name:  ug_putrun
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
 **************************************************************************************/

static int ug_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ug_dev_s *priv = &g_ugdev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t devcol;
  uint8_t fbmask;
  uint8_t page;
  uint8_t usrmask;
  uint8_t i;
  int     pixlen;

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)UG_XRES)
    {
      pixlen = (int)UG_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > UG_YRES)
    {
      return OK;
    }

  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel
   * position in the frame buffer memory.  Pixels are organized like
   * this:
   *
   *  --------+---+---+---+---+-...-+-----+
   *  Segment | 0 | 1 | 2 | 3 | ... | 131 |
   *  --------+---+---+---+---+-...-+-----+
   *  Bit 0   |   | X |   |   |     |     |
   *  Bit 1   |   | X |   |   |     |     |
   *  Bit 2   |   | X |   |   |     |     |
   *  Bit 3   |   | X |   |   |     |     |
   *  Bit 4   |   | X |   |   |     |     |
   *  Bit 5   |   | X |   |   |     |     |
   *  Bit 6   |   | X |   |   |     |     |
   *  Bit 7   |   | X |   |   |     |     | 
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * UG_XRES + col];
  ptr     = fbptr;
#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

      if ((*buffer & usrmask) != 0)
        {
          *ptr++ |= fbmask;
        }
      else
        {
          *ptr++ &= ~fbmask;
        }

      /* Inc/Decrement to the next source pixel */

#ifdef CONFIG_NX_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
          usrmask = MS_BIT;
        }
      else
        {
          usrmask >>= 1;
        }
#else
      if (usrmask == MS_BIT)
        {
          buffer++;
          usrmask = LS_BIT;
        }
      else
        {
          usrmask <<= 1;
        }
#endif
    }

  /* Offset the column position to account for smaller horizontal
   * display range.
   */

  devcol = col + UG_XOFFSET;

  /* Select and lock the device */

  ug_select(priv->spi);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */

  (void)SPI_SEND(priv->spi, SSD1305_SETPAGESTART+page);         /* Set the page start */
  (void)SPI_SEND(priv->spi, SSD1305_SETCOLL + (devcol & 0x0f)); /* Set the low column */
  (void)SPI_SEND(priv->spi, SSD1305_SETCOLH + (devcol >> 4));   /* Set the high column */

  /* Select data transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, false);

  /* Then transfer all of the data */

  (void)SPI_SNDBLOCK(priv->spi, fbptr, pixlen);

  /* Unlock and de-select the device */

  ug_deselect(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  ug_getrun
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
 **************************************************************************************/

static int ug_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                     size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ug_dev_s *priv = &g_ugdev;
  FAR uint8_t *fbptr;
  uint8_t page;
  uint8_t fbmask;
  uint8_t usrmask;
  uint8_t i;
  int     pixlen;

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)UG_XRES)
    {
      pixlen = (int)UG_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > UG_YRES)
    {
      return -EINVAL;
    }

  /* Then transfer the display data from the shadow frame buffer memory */
  /* Get the page number.  The range of 64 lines is divided up into eight
   * pages of 8 lines each.
   */

  page = row >> 3;

  /* Update the shadow frame buffer memory. First determine the pixel
   * position in the frame buffer memory.  Pixels are organized like
   * this:
   *
   *  --------+---+---+---+---+-...-+-----+
   *  Segment | 0 | 1 | 2 | 3 | ... | 131 |
   *  --------+---+---+---+---+-...-+-----+
   *  Bit 0   |   | X |   |   |     |     |
   *  Bit 1   |   | X |   |   |     |     |
   *  Bit 2   |   | X |   |   |     |     |
   *  Bit 3   |   | X |   |   |     |     |
   *  Bit 4   |   | X |   |   |     |     |
   *  Bit 5   |   | X |   |   |     |     |
   *  Bit 6   |   | X |   |   |     |     |
   *  Bit 7   |   | X |   |   |     |     | 
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * UG_XRES + col];
#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  *buffer = 0;
  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */
      
      uint8_t byte = *fbptr++;
      if ((byte & fbmask) != 0)
        {
          *buffer |= usrmask;
        }

      /* Inc/Decrement to the next destination pixel. Hmmmm. It looks like
       * this logic could write past the end of the user buffer.  Revisit
       * this!
       */

#ifdef CONFIG_NX_PACKEDMSFIRST
      if (usrmask == LS_BIT)
        {
          buffer++;
         *buffer = 0;
          usrmask = MS_BIT;
        }
      else
        {
          usrmask >>= 1;
        }
#else
      if (usrmask == MS_BIT)
        {
          buffer++;
         *buffer = 0;
          usrmask = LS_BIT;
        }
      else
        {
          usrmask <<= 1;
        }
#endif
    }

  return OK;
}

/**************************************************************************************
 * Name:  ug_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ug_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ug_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ug_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ug_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ug_getpower(struct lcd_dev_s *dev)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;
  DEBUGASSERT(priv);
  gvdbg("powered: %s\n", ug_powerstring(priv->powered));
  return priv->powered;
}

/**************************************************************************************
 * Name:  ug_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ug_setpower(struct lcd_dev_s *dev, int power)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;

  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER);
  gvdbg("power: %s powered: %s\n",
        ug_powerstring(power), ug_powerstring(priv->powered));

  /* Select and lock the device */

  ug_select(priv->spi);
  if (power <= UG_POWER_OFF)
    {
      /* Turn the display off */

      (void)SPI_SEND(priv->spi, SSD1305_DISPOFF);       /* Display off */

      /* Remove power to the device */

      ug_power(0, false);
      priv->powered = UG_POWER_OFF;
    }
  else
    {
      /* Turn the display on, dim or normal */

      if (power == UG_POWER_DIM)
        {
          (void)SPI_SEND(priv->spi, SSD1305_DISPONDIM); /* Display on, dim mode */
        }
      else /* if (power > UG_POWER_DIM) */
        {
          (void)SPI_SEND(priv->spi, SSD1305_DISPON);    /* Display on, normal mode */
          power = UG_POWER_ON;
        }
      (void)SPI_SEND(priv->spi, SSD1305_DISPRAM);       /* Resume to RAM content display */

      /* Restore power to the device */

      ug_power(0, true);
      priv->powered = power;
    }
  ug_deselect(priv->spi);

  return OK;
}

/**************************************************************************************
 * Name:  ug_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ug_getcontrast(struct lcd_dev_s *dev)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;
  DEBUGASSERT(priv);
  return (int)priv->contrast;
}

/**************************************************************************************
 * Name:  ug_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ug_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ug_dev_s *priv = (struct ug_dev_s *)dev;

  gvdbg("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  if (contrast > 255)
    {
      return -EINVAL;
    }

  /* Select and lock the device */

  ug_select(priv->spi);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the contrast */

  (void)SPI_SEND(priv->spi, SSD1305_SETCONTRAST);  /* Set contrast control register */
  (void)SPI_SEND(priv->spi, contrast);             /* Data 1: Set 1 of 256 contrast steps */
  priv->contrast = contrast;
  
  /* Unlock and de-select the device */

  ug_deselect(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  up_clear
 *
 * Description:
 *   Clear the display.
 *
 **************************************************************************************/

static inline void up_clear(FAR struct ug_dev_s  *priv)
{
  FAR struct spi_dev_s *spi  = priv->spi;
  int page;
  int i;

  /* Clear the framebuffer */

  memset(priv->fb, UG_Y1_BLACK, UG_FBSIZE);

  /* Select and lock the device */

  ug_select(priv->spi);

  /* Go through all 8 pages */

  for (page = 0, i = 0; i < 8; i++)
    {
      /* Select command transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

      /* Set the starting position for the run */

      (void)SPI_SEND(priv->spi, SSD1305_SETPAGESTART+i);
      (void)SPI_SEND(priv->spi, SSD1305_SETCOLL + (UG_XOFFSET & 0x0f));
      (void)SPI_SEND(priv->spi, SSD1305_SETCOLH + (UG_XOFFSET >> 4));

      /* Select data transfer */

      SPI_CMDDATA(spi, SPIDEV_DISPLAY, false);

       /* Then transfer all 96 columns of data */

       (void)SPI_SNDBLOCK(priv->spi, &priv->fb[page * UG_XRES], UG_XRES);
    }

  /* Unlock and de-select the device */

  ug_deselect(spi);
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ug_initialize
 *
 * Description:
 *   Initialize the UG-9664HSWAG01 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready to
 *   use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_UG9664HSWAG01_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the specified
 *   OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *ug_initialize(FAR struct spi_dev_s *spi, unsigned int devno)
{
  /* Configure and enable LCD */
 
  FAR struct ug_dev_s  *priv = &g_ugdev;

  gvdbg("Initializing\n");
  DEBUGASSERT(spi && devno == 0);

  /* Save the reference to the SPI device */

  priv->spi = spi;

  /* Select and lock the device */

  ug_select(spi);

  /* Make sure that the OLED off */

  ug_power(0, false);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Configure the device */

  (void)SPI_SEND(spi, SSD1305_SETCOLL + 2);       /* Set low column address */
  (void)SPI_SEND(spi, SSD1305_SETCOLH + 2);       /* Set high column address */
  (void)SPI_SEND(spi, SSD1305_SETSTARTLINE+0);    /* Display start set */
  (void)SPI_SEND(spi, SSD1305_SCROLL_STOP);       /* Stop horizontal scroll */
  (void)SPI_SEND(spi, SSD1305_SETCONTRAST);       /* Set contrast control register */
  (void)SPI_SEND(spi, 0x32);                      /* Data 1: Set 1 of 256 contrast steps */
  (void)SPI_SEND(spi, SSD1305_SETBRIGHTNESS);     /* Brightness for color bank */
  (void)SPI_SEND(spi, 0x80);                      /* Data 1: Set 1 of 256 contrast steps */
  (void)SPI_SEND(spi, SSD1305_MAPCOL131);         /* Set segment re-map */
  (void)SPI_SEND(spi, SSD1305_DISPNORMAL);        /* Set normal display */
/*(void)SPI_SEND(spi, SSD1305_DISPINVERTED);         Set inverse display */
  (void)SPI_SEND(spi, SSD1305_SETMUX);            /* Set multiplex ratio */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 1: MUX ratio -1: 15-63 */
  (void)SPI_SEND(spi, SSD1305_SETOFFSET);         /* Set display offset */
  (void)SPI_SEND(spi, 0x40);                      /* Data 1: Vertical shift by COM: 0-63 */
  (void)SPI_SEND(spi, SSD1305_MSTRCONFIG);        /* Set dc-dc on/off */
  (void)SPI_SEND(spi, SSD1305_MSTRCONFIG_EXTVCC); /* Data 1: Select external Vcc */
  (void)SPI_SEND(spi, SSD1305_SETCOMREMAPPED);    /* Set com output scan direction */
  (void)SPI_SEND(spi, SSD1305_SETDCLK);           /* Set display clock divide
                                                   * ratio/oscillator/frequency */
  (void)SPI_SEND(spi, 15 << SSD1305_DCLKFREQ_SHIFT | 0 << SSD1305_DCLKDIV_SHIFT);
  (void)SPI_SEND(spi, SSD1305_SETCOLORMODE);      /* Set area color mode on/off & low power
                                                   * display mode */
  (void)SPI_SEND(spi, SSD1305_COLORMODE_MONO | SSD1305_POWERMODE_LOW);
  (void)SPI_SEND(spi, SSD1305_SETPRECHARGE);      /* Set pre-charge period */
  (void)SPI_SEND(spi, 15 << SSD1305_PHASE2_SHIFT | 1 << SSD1305_PHASE1_SHIFT);
  (void)SPI_SEND(spi, SSD1305_SETCOMCONFIG);      /* Set COM configuration */
  (void)SPI_SEND(spi, SSD1305_COMCONFIG_ALT);     /* Data 1, Bit 4: 1=Alternative COM pin configuration */
  (void)SPI_SEND(spi, SSD1305_SETVCOMHDESEL);     /* Set VCOMH deselect level */
  (void)SPI_SEND(spi, SSD1305_VCOMH_x7p7);        /* Data 1: ~0.77 x Vcc  */
  (void)SPI_SEND(spi, SSD1305_SETLUT);            /* Set look up table for area color */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 1: Pulse width: 31-63 */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 2: Color A: 31-63 */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 3: Color B: 31-63 */
  (void)SPI_SEND(spi, 0x3f);                      /* Data 4: Color C: 31-63 */
  (void)SPI_SEND(spi, SSD1305_DISPON);            /* Display on, normal mode */
  (void)SPI_SEND(spi, SSD1305_DISPRAM);           /* Resume to RAM content display */

  /* Let go of the SPI lock and de-select the device */

  ug_deselect(spi);

  /* Clear the framebuffer */

  up_mdelay(100);
  up_clear(priv);
  return &priv->dev;
}
