/**************************************************************************************
 * drivers/lcd/ug-2864ambag01.c
 * Driver for Univision UG-2864AMBAG01 OLED display (wih SH1101A controller) in SPI
 * mode
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   1. Product Specification (Preliminary), Part Name: OEL Display Module, Part ID:
 *      UG-2864AMBAG01, Doc No: SASI-9015-A, Univision Technology Inc.
 *   2. SH1101A, 132 X 64 Dot Matrix OLED/PLED, Preliminary Segment/Common Driver with
 *      Controller, Sino Wealth
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
 * Device memory organization:
 *
 *          +----------------------------+
 *          |           Column           |
 *  --------+----+---+---+---+-...-+-----+
 *  Page    | 0  | 1 | 2 | 3 | ... | 131 |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 0  | D0 | X |   |   |     |     |
 *          | D1 | X |   |   |     |     |
 *          | D2 | X |   |   |     |     |
 *          | D3 | X |   |   |     |     |
 *          | D4 | X |   |   |     |     |
 *          | D5 | X |   |   |     |     |
 *          | D6 | X |   |   |     |     |
 *          | D7 | X |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 1  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 2  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 3  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 4  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 5  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 6  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *  Page 7  |    |   |   |   |     |     |
 *  --------+----+---+---+---+-...-+-----+
 *
 *  -----------------------------------+---------------------------------------
 *  Landscape Display:                 | Reverse Landscape Display:
 *  --------+-----------------------+  |  --------+---------------------------+
 *          |       Column          |  |          |         Column            |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 0  | 0 | 1 | 2 |     | 131 |  |  Page 7  | 131 | 130 | 129 |     | 0 |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 1  | V                     |  |  Page 6  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 2  | V                     |  |  Page 5  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 3  | V                     |  |  Page 4  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 4  | V                     |  |  Page 3  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 5  | V                     |  |  Page 2  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 6  | V                     |  |  Page 1  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  Page 7  | V                     |  |  Page 0  |                         ^ |
 *  --------+---+---+---+-...-+-----+  |  --------+-----+-----+-----+-...-+---+
 *  -----------------------------------+---------------------------------------
 *
 *  -----------------------------------+---------------------------------------
 *  Portrait Display:                  | Reverse Portrait Display:
 *  -----------+---------------------+ |  -----------+---------------------+
 *             |         Page        | |             |       Page          |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 0   | 0 | 1 | 2 |     | 7 | |  Column 131 | 7 | 6 | 5 |     | 0 |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 1   | >   >   >    >    > | |  Column 130 |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 2   |                     | |  Column 129 |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  ...        |                     | |  ...        |                     |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  Column 131 |                     | |  Column 0   | <   <   <    <    < |
 *  -----------+---+---+---+-...-+---+ |  -----------+---+---+---+-...-+---+
 *  -----------------------------------+----------------------------------------
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
#include <nuttx/lcd/ug-2864ambag01.h>

#include <arch/irq.h>

#ifdef CONFIG_LCD_UG2864AMBAG01

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/
/* Limitations of the current configuration that I hope to fix someday */

#if CONFIG_UG2864AMBAG01_NINTERFACES != 1
#  warning "This implementation supports only a single OLED device"
#  undef CONFIG_UG2864AMBAG01_NINTERFACES
#  define CONFIG_UG2864AMBAG01_NINTERFACES 1
#endif

#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
#  warning "No support yet for portrait modes"
#  define CONFIG_LCD_LANDSCAPE 1
#  undef CONFIG_LCD_PORTRAIT
#  undef CONFIG_LCD_RLANDSCAPE
#  undef CONFIG_LCD_RPORTRAIT
#elif defined(CONFIG_LCD_RLANDSCAPE)
#  warning "Reverse landscape mode is untested and, hence, probably buggy"
#endif

/* SH1101A Commands *******************************************************************/

#define SH1101A_SETCOLL(ad)      (0x00 | ((ad) & 0x0f)) /* Set Lower Column Address: (00h - 0fh) */
#define SH1101A_SETCOLH(ad)      (0x10 | ((ad) & 0x0f)) /* Set Higher Column Address: (10h - 1fh) */
#define SH1101A_STARTLINE(ln)    (0x40 | ((ln) & 0x3f)) /* Set Display Start Line: (40h - 7fh) */
#define SH1101A_CONTRAST_MODE    (0x81)                 /* Set Contrast Control Register: (Double Bytes Command) */
#  define SH1101A_CONTRAST(c)    (c)
#define SH1101A_SEGREMAP(m)      (0xa0 | ((m) & 0x01))  /* Set Segment Re-map: (a0h - a1h) */
#  define SH1101A_REMAPRIGHT     SH1101A_SEGREMAP(0)    /*   Right rotation */
#  define SH1101A_REMAPPLEFT     SH1101A_SEGREMAP(1)    /*   Left rotation */
#define SH1101A_EDISPOFFON(s)    (0xa4 | ((s) & 0x01))  /* Set Entire Display OFF/ON: (a4h - a5h) */
#  define SH1101A_EDISPOFF       SH1101A_EDISPOFFON(0)  /*   Display off */
#  define SH1101A_EDISPON        SH1101A_EDISPOFFON(1)  /*   Display on */
#define SH1101A_NORMREV(s)       (0xa6 | ((s) & 0x01))  /* Set Normal/Reverse Display: (a6h -a7h) */
#  define SH1101A_NORMAL         SH1101A_NORMREV(0)     /*   Normal display */
#  define SH1101A_REVERSE        SH1101A_NORMREV(1)     /*   Reverse display */
#define SH1101A_MRATIO_MODE      (0xa8)                 /* Set Multiplex Ration: (Double Bytes Command) */
#  define SH1101A_MRATIO(d)      ((d) & 0x3f)
#define SH1101A_DCDC_MODE        (0xad)                 /* Set DC-DC OFF/ON: (Double Bytes Command) */
#  define SH1101A_DCDC_OFF       (0x8a)
 # define SH1101A_DCDC_ON        (0x8b)
#define SH1101A_DISPOFFON(s)     (0xae | ((s) & 0x01))  /* Display OFF/ON: (aeh - afh) */
#  define SH1101A_DISPOFF        SH1101A_DISPOFFON(0)   /*   Display off */
#  define SH1101A_DISPON         SH1101A_DISPOFFON(1)   /*   Display on */
#define SH1101A_PAGEADDR(a)      (0xb0 | ((a) & 0x0f))  /* Set Page Address: (b0h - b7h) */
#define SH1101A_SCANDIR(d)       (0xc0 | ((d) & 0x08))  /* Set Common Output Scan Direction: (c0h - c8h) */
#  define SH1101A_SCANFROMCOM0   SH1101A_SCANDIR(0x00)  /*   Scan from COM[0] to COM[n-1]*/
#  define SH1101A_SCANTOCOM0     SH1101A_SCANDIR(0x08)  /*   Scan from COM[n-1] to COM[0] */
#define SH1101A_DISPOFFS_MODE    (0xd3)                 /* Set Display Offset: (Double Bytes Command) */
#  define SH1101A_DISPOFFS(o)    ((o) & 0x3f)
#define SH1101A_CLKDIV_SET       (0xd5)                 /* Set Display Clock Divide Ratio/Oscillator Frequency: (Double Bytes Command) */
#  define SH1101A_CLKDIV(f,d)    ((((f) & 0x0f) << 4) | ((d) & 0x0f))
#define SH1101A_CHRGPER_SET      (0xd9)                 /* Set Dis-charge/Pre-charge Period: (Double Bytes Command) */
#  define SH1101A_CHRGPER(d,p)   ((((d) & 0x0f) << 4) | ((p) & 0x0f))
#define SH1101A_CMNPAD_CONFIG    (0xda)                 /* Set Common pads hardware configuration: (Double Bytes Command) */
  #define SH1101A_CMNPAD(c)      ((0x02) | ((c) & 0x10))
#define SH1101A_VCOM_SET         (0xdb)                 /* Set VCOM Deselect Level: (Double Bytes Command) */
#  define SH1101A_VCOM(v)        (v)
#define SH1101A_RMWSTART         (0xe0)                 /* Read-Modify-Write: (e0h) */
#define SH1101A_NOP              (0xe3)                 /* NOP: (e3h) */
#define SH1101A_END              (0xee)                 /* End: (eeh) */

#define SH1101A_WRDATA(d)        (d)                    /* Write Display Data */
#define SH1101A_STATUS_BUSY      (0x80)                 /* Read Status */
#define SH1101A_STATUS_ONOFF     (0x40)
#define SH1101A_RDDATA(d)        (d)                    /* Read Display Data */

/* Color Properties *******************************************************************/
/* Display Resolution
 *
 * The SH1101A display controller can handle a resolution of 132x64. The UG-2864AMBAG01
 * on the base board is 128x64.
 */

#define UG2864AMBAG01_DEV_XRES    128 /* Only 128 of 131 columns used */
#define UG2864AMBAG01_DEV_YRES    64  /* 8 pages each 8 rows */
#define UG2864AMBAG01_DEV_XOFFSET 2   /* Offset to logical column 0 */
#define UG2864AMBAG01_DEV_PAGES   8   /* 8 pages */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#  define UG2864AMBAG01_XRES      UG2864AMBAG01_DEV_XRES
#  define UG2864AMBAG01_YRES      UG2864AMBAG01_DEV_YRES
#else
#  define UG2864AMBAG01_XRES      UG2864AMBAG01_DEV_YRES
#  define UG2864AMBAG01_YRES      UG2864AMBAG01_DEV_XRES
#endif

/* Color depth and format */

#define UG2864AMBAG01_BPP          1
#define UG2864AMBAG01_COLORFMT     FB_FMT_Y1

/* Bytes per logical row and actual device row */

#define UG2864AMBAG01_XSTRIDE      (UG2864AMBAG01_XRES >> 3)
#define UG2864AMBAG01_YSTRIDE      (UG2864AMBAG01_YRES >> 3)

/* Default contrast */

#define UG2864AMBAG01_CONTRAST     (128)

/* The size of the shadow frame buffer or one row buffer.
 *
 * Frame buffer size: 128 columns x 64 rows / 8 bits-per-pixel
 * Row size:          128 columns x 8 rows-per-page / 8 bits-per-pixel
 */

#define UG2864AMBAG01_FBSIZE       (UG2864AMBAG01_XSTRIDE * UG2864AMBAG01_YRES)
#define UG2864AMBAG01_ROWSIZE      (UG2864AMBAG01_XSTRIDE)

/* Bit helpers */

#define LS_BIT                     (1 << 0)
#define MS_BIT                     (1 << 7)

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg(format, arg...)   dbg(format, ##arg)
#  define lcdvdbg(format, arg...)  vdbg(format, ##arg)
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct ug2864ambag01_dev_s
{
  struct lcd_dev_s       dev;      /* Publically visible device structure */

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s  *spi;      /* Cached SPI device reference */
  uint8_t                contrast; /* Current contrast setting */
  bool                   on;       /* true: display is on */


 /* The SH1101A does not support reading from the display memory in SPI mode.
  * Since there is 1 BPP and access is byte-by-byte, it is necessary to keep
  * a shadow copy of the framebuffer memory. At 128x64, this amounts to 1KB.
  */

  uint8_t fb[UG2864AMBAG01_FBSIZE];
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* Low-level SPI helpers */

#ifdef CONFIG_SPI_OWNBUS
static inline void ug2864ambag01_configspi(FAR struct spi_dev_s *spi);
#  define ug2864ambag01_lock(spi)
#  define ug2864ambag01_unlock(spi)
#else
#  define ug2864ambag01_configspi(spi)
static void ug2864ambag01_lock(FAR struct spi_dev_s *spi);
static void ug2864ambag01_unlock(FAR struct spi_dev_s *spi);
#endif

/* LCD Data Transfer Methods */

static int ug2864ambag01_putrun(fb_coord_t row, fb_coord_t col,
                                FAR const uint8_t *buffer, size_t npixels);
static int ug2864ambag01_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                                size_t npixels);

/* LCD Configuration */

static int ug2864ambag01_getvideoinfo(FAR struct lcd_dev_s *dev,
                                      FAR struct fb_videoinfo_s *vinfo);
static int ug2864ambag01_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int ug2864ambag01_getpower(struct lcd_dev_s *dev);
static int ug2864ambag01_setpower(struct lcd_dev_s *dev, int power);
static int ug2864ambag01_getcontrast(struct lcd_dev_s *dev);
static int ug2864ambag01_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

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

static uint8_t g_runbuffer[UG2864AMBAG01_ROWSIZE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = UG2864AMBAG01_COLORFMT, /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = UG2864AMBAG01_XRES,     /* Horizontal resolution in pixel columns */
  .yres    = UG2864AMBAG01_YRES,     /* Vertical resolution in pixel rows */
  .nplanes = 1,                      /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = ug2864ambag01_putrun,    /* Put a run into LCD memory */
  .getrun = ug2864ambag01_getrun,    /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer,   /* Run scratch buffer */
  .bpp    = UG2864AMBAG01_BPP,       /* Bits-per-pixel */
};

/* This is the OLED driver instance (only a single device is supported for now) */

static struct ug2864ambag01_dev_s g_oleddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = ug2864ambag01_getvideoinfo,
    .getplaneinfo = ug2864ambag01_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = ug2864ambag01_getpower,
    .setpower     = ug2864ambag01_setpower,
    .getcontrast  = ug2864ambag01_getcontrast,
    .setcontrast  = ug2864ambag01_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: ug2864ambag01_configspi
 *
 * Description:
 *   Configure the SPI for use with the UG-2864AMBAG01
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifdef CONFIG_SPI_OWNBUS
static inline void ug2864ambag01_configspi(FAR struct spi_dev_s *spi)
{
  lcdvdbg("Mode: %d Bits: 8 Frequency: %d\n",
          CONFIG_UG2864AMBAG01_SPIMODE, CONFIG_UG2864AMBAG01_FREQUENCY);

  /* Configure SPI for the UG-2864AMBAG01.  But only if we own the SPI bus.  Otherwise,
   * don't bother because it might change.
   */

  SPI_SETMODE(spi, CONFIG_UG2864AMBAG01_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_UG2864AMBAG01_FREQUENCY);
}
#endif

/**************************************************************************************
 * Name: ug2864ambag01_lock
 *
 * Description:
 *   Select the SPI, locking and  re-configuring if necessary
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void ug2864ambag01_lock(FAR struct spi_dev_s *spi)
{
  /* Lock the SPI bus if there are multiple devices competing for the SPI bus. */

  SPI_LOCK(spi, true);

  /* Now make sure that the SPI bus is configured for the UG-2864AMBAG01 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_UG2864AMBAG01_SPIMODE);
  SPI_SETBITS(spi, 8);
  SPI_SETFREQUENCY(spi, CONFIG_UG2864AMBAG01_FREQUENCY);
}
#endif

/**************************************************************************************
 * Name: ug2864ambag01_unlock
 *
 * Description:
 *   De-select the SPI
 *
 * Input Parameters:
 *   spi  - Reference to the SPI driver structure
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

#ifndef CONFIG_SPI_OWNBUS
static inline void ug2864ambag01_unlock(FAR struct spi_dev_s *spi)
{
  /* De-select UG-2864AMBAG01 chip and relinquish the SPI bus. */

  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name:  ug2864ambag01_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 * Input Parameters:
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ug2864ambag01_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                                size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ug2864ambag01_dev_s *priv = (FAR struct ug2864ambag01_dev_s *)&g_oleddev;
  FAR uint8_t *fbptr;
  FAR uint8_t *ptr;
  uint8_t devcol;
  uint8_t fbmask;
  uint8_t page;
  uint8_t usrmask;
  int pixlen;
  uint8_t i;

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)UG2864AMBAG01_XRES)
    {
      pixlen = (int)UG2864AMBAG01_XRES - (int)col;
    }

  /* Verify that some portion of the run remains on the display */

  if (pixlen <= 0 || row > UG2864AMBAG01_YRES)
    {
      return OK;
    }

  /* Perform coordinate conversion for reverse landscape mode */

#ifdef CONFIG_LCD_RLANDSCAPE
  row = (UG2864AMBAG01_YRES-1) - row;
  col = (UG2864AMBAG01_XRES-1) - col;
#endif

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
   *     D0   |   | X |   |   |     |     |
   *     D1   |   | X |   |   |     |     |
   *     D2   |   | X |   |   |     |     |
   *     D3   |   | X |   |   |     |     |
   *     D4   |   | X |   |   |     |     |
   *     D5   |   | X |   |   |     |     |
   *     D6   |   | X |   |   |     |     |
   *     D7   |   | X |   |   |     |     | 
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
  fbptr   = &priv->fb[page * UG2864AMBAG01_XRES + col];
#ifdef CONFIG_LCD_RLANDSCAPE
  ptr     = fbptr + pixlen - 1;
#else
  ptr     = fbptr;
#endif
#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */

#ifdef CONFIG_LCD_RLANDSCAPE
      if ((*buffer & usrmask) != 0)
        {
          *ptr-- |= fbmask;
        }
      else
        {
          *ptr-- &= ~fbmask;
        }
#else
      if ((*buffer & usrmask) != 0)
        {
          *ptr++ |= fbmask;
        }
      else
        {
          *ptr++ &= ~fbmask;
        }
#endif

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

  devcol = col + UG2864AMBAG01_DEV_XOFFSET;

  /* Lock and select device */

  ug2864ambag01_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the starting position for the run */
  /* Set the column address to the XOFFSET value */
  
  SPI_SEND(priv->spi, SH1101A_SETCOLL(devcol & 0x0f));
  SPI_SEND(priv->spi, SH1101A_SETCOLH(devcol >> 4));

  /* Set the page address */

  SPI_SEND(priv->spi, SH1101A_PAGEADDR(page));

  /* Select data transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, false);

  /* Then transfer all of the data */

  (void)SPI_SNDBLOCK(priv->spi, fbptr, pixlen);

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ug2864ambag01_unlock(priv->spi);
  return OK;
}
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ug2864ambag01_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD.
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 **************************************************************************************/

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
static int ug2864ambag01_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  /* Because of this line of code, we will only be able to support a single UG device */

  FAR struct ug2864ambag01_dev_s *priv = &g_oleddev;
  FAR uint8_t *fbptr;
  uint8_t page;
  uint8_t fbmask;
  uint8_t usrmask;
  int pixlen;
  uint8_t i;

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Clip the run to the display */

  pixlen = npixels;
  if ((unsigned int)col + (unsigned int)pixlen > (unsigned int)UG2864AMBAG01_XRES)
    {
      pixlen = (int)UG2864AMBAG01_XRES - (int)col;
    }

  /* Verify that some portion of the run is actually the display */

  if (pixlen <= 0 || row > UG2864AMBAG01_YRES)
    {
      return -EINVAL;
    }

  /* Perform coordinate conversion for reverse landscape mode */

#ifdef CONFIG_LCD_RLANDSCAPE
  row = (UG2864AMBAG01_YRES-1) - row;
  col = (UG2864AMBAG01_XRES-1) - col;
#endif

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
   *     D0   |   | X |   |   |     |     |
   *     D1   |   | X |   |   |     |     |
   *     D2   |   | X |   |   |     |     |
   *     D3   |   | X |   |   |     |     |
   *     D4   |   | X |   |   |     |     |
   *     D5   |   | X |   |   |     |     |
   *     D6   |   | X |   |   |     |     |
   *     D7   |   | X |   |   |     |     | 
   *  --------+---+---+---+---+-...-+-----+
   *
   * So, in order to draw a white, horizontal line, at row 45. we
   * would have to modify all of the bytes in page 45/8 = 5.  We
   * would have to set bit 45%8 = 5 in every byte in the page.
   */

  fbmask  = 1 << (row & 7);
#ifdef CONFIG_LCD_RLANDSCAPE
  fbptr   = &priv->fb[page * (UG2864AMBAG01_XRES-1) + col + pixlen];
#else
  fbptr   = &priv->fb[page * UG2864AMBAG01_XRES + col];
#endif
#ifdef CONFIG_NX_PACKEDMSFIRST
  usrmask = MS_BIT;
#else
  usrmask = LS_BIT;
#endif

  *buffer = 0;
  for (i = 0; i < pixlen; i++)
    {
      /* Set or clear the corresponding bit */
      
#ifdef CONFIG_LCD_RLANDSCAPE
      uint8_t byte = *fbptr--;
#else
      uint8_t byte = *fbptr++;
#endif
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
#else
#  error "Configuration not implemented"
#endif

/**************************************************************************************
 * Name:  ug2864ambag01_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ug2864ambag01_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ug2864ambag01_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ug2864ambag01_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  ug2864ambag01_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ug2864ambag01_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct ug2864ambag01_dev_s *priv = (FAR struct ug2864ambag01_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdvdbg("power: %s\n", priv->on ? "ON" : "OFF");
  return priv->on ? CONFIG_LCD_MAXPOWER : 0;
}

/**************************************************************************************
 * Name:  ug2864ambag01_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ug2864ambag01_setpower(struct lcd_dev_s *dev, int power)
{
  struct ug2864ambag01_dev_s *priv = (struct ug2864ambag01_dev_s *)dev;
  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER && priv->spi);

  lcdvdbg("power: %d [%d]\n", power, priv->on ? CONFIG_LCD_MAXPOWER : 0);

  /* Lock and select device */

  ug2864ambag01_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  if (power <= 0)
    {
      /* Turn the display off */

      (void)SPI_SEND(priv->spi, SH1101A_DISPOFF);
      priv->on = false;
    }
  else
    {
      /* Turn the display on */

      (void)SPI_SEND(priv->spi, SH1101A_DISPON); /* Display on, dim mode */
      priv->on = true;
    }

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ug2864ambag01_unlock(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  ug2864ambag01_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ug2864ambag01_getcontrast(struct lcd_dev_s *dev)
{
  struct ug2864ambag01_dev_s *priv = (struct ug2864ambag01_dev_s *)dev;
  DEBUGASSERT(priv);

  lcdvdbg("contrast: %d\n", priv->contrast);
  return priv->contrast;
}

/**************************************************************************************
 * Name:  ug2864ambag01_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ug2864ambag01_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct ug2864ambag01_dev_s *priv = (struct ug2864ambag01_dev_s *)dev;
  unsigned int scaled;

  lcdvdbg("contrast: %d\n", contrast);
  DEBUGASSERT(priv);

  /* Verify the contrast value */

#ifdef CONFIG_DEBUG
  if (contrast > CONFIG_LCD_MAXCONTRAST)
    {
      return -EINVAL;
    }
#endif

  /* Scale contrast:  newcontrast = 255 * contrast / CONFIG_LCD_MAXCONTRAST
   * Where contrast is in the range {1,255}
   */

#if CONFIG_LCD_MAXCONTRAST != 255
  scaled = ((contrast << 8) - 1) / CONFIG_LCD_MAXCONTRAST;
#else
  scaled = contrast;
#endif

  /* Lock and select device */

  ug2864ambag01_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  /* Select command transfer */

  SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

  /* Set the contrast */

  (void)SPI_SEND(priv->spi, SH1101A_CONTRAST_MODE);    /* Set contrast control register */
  (void)SPI_SEND(priv->spi, SH1101A_CONTRAST(scaled)); /* Data 1: Set 1 of 256 contrast steps */
  priv->contrast = contrast;

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ug2864ambag01_unlock(priv->spi);
  return OK;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ug2864ambag01_initialize
 *
 * Description:
 *   Initialize the UG-2864AMBAG01 video hardware.  The initial state of the
 *   OLED is fully initialized, display memory cleared, and the OLED ready
 *   to use, but with the power setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 through CONFIG_UG2864AMBAG01_NINTERFACES-1.
 *     This allows support for multiple OLED devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for
 *   the specified OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *ug2864ambag01_initialize(FAR struct spi_dev_s *spi, unsigned int devno)
{
  FAR struct ug2864ambag01_dev_s  *priv = &g_oleddev;

  lcdvdbg("Initializing\n");
  DEBUGASSERT(spi && devno == 0);

  /* Save the reference to the SPI device */

  priv->spi = spi;

  /* Configure the SPI */

  ug2864ambag01_configspi(spi);

  /* Lock and select device */

  ug2864ambag01_lock(priv->spi);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Select command transfer */

  SPI_CMDDATA(spi, SPIDEV_DISPLAY, true);

  /* Configure the device */

  SPI_SEND(spi, SH1101A_DISPOFF);         /* Display off */
  SPI_SEND(spi, SH1101A_SETCOLL(0));      /* Set lower column address */
  SPI_SEND(spi, SH1101A_SETCOLH(0));      /* Set higher column address */
  SPI_SEND(spi, SH1101A_STARTLINE(0));    /* Set display start line */
  SPI_SEND(spi, SH1101A_PAGEADDR(0));     /* Set page address */
  SPI_SEND(spi, SH1101A_CONTRAST_MODE);   /* Contrast control */
  SPI_SEND(spi ,UG2864AMBAG01_CONTRAST);  /*   Default contrast */
  SPI_SEND(spi, SH1101A_REMAPPLEFT);      /* Set segment remap left */
  SPI_SEND(spi, SH1101A_EDISPOFF);        /* Normal display */
  SPI_SEND(spi, SH1101A_NORMAL);          /* Normal (un-reversed) display mode */
  SPI_SEND(spi, SH1101A_MRATIO_MODE);     /* Multiplex ratio */
  SPI_SEND(spi, SH1101A_MRATIO(0x3f));    /*   Duty = 1/64 */
  SPI_SEND(spi, SH1101A_SCANTOCOM0);      /* Com scan direction: Scan from COM[n-1] to COM[0] */
  SPI_SEND(spi, SH1101A_DISPOFFS_MODE);   /* Set display offset */
  SPI_SEND(spi, SH1101A_DISPOFFS(0));
  SPI_SEND(spi, SH1101A_CLKDIV_SET);      /* Set clock divider */
  SPI_SEND(spi, SH1101A_CLKDIV(0,0));
  SPI_SEND(spi, SH1101A_CMNPAD_CONFIG);   /* Set common pads */
  SPI_SEND(spi, SH1101A_CMNPAD(0x10));
  SPI_SEND(spi, SH1101A_VCOM_SET);
  SPI_SEND(spi, SH1101A_VCOM(0x40));
  SPI_SEND(spi, SH1101A_DCDC_MODE);       /* DC/DC control mode: on */
  SPI_SEND(spi, SH1101A_DCDC_ON);
  SPI_SEND(spi, SH1101A_DISPON);          /* display ON  */

  /* De-select and unlock the device */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  ug2864ambag01_unlock(priv->spi);

  /* Clear the display */

  up_mdelay(100);
  ug2864ambag01_fill(&priv->dev, UG_Y1_BLACK);
  return &priv->dev;
}

/**************************************************************************************
 * Name:  ug2864ambag01_fill
 *
 * Description:
 *   This non-standard method can be used to clear the entire display by writing one
 *   color to the display.  This is much faster than writing a series of runs.
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 **************************************************************************************/

void ug2864ambag01_fill(FAR struct lcd_dev_s *dev, uint8_t color)
{
  FAR struct ug2864ambag01_dev_s  *priv = &g_oleddev;
  unsigned int page;

  /* Make an 8-bit version of the selected color */

  if (color & 1)
    {
      color = 0xff;
    }
  else
    {
      color = 0;
    }

  /* Initialize the framebuffer */

  memset(priv->fb, color, UG2864AMBAG01_FBSIZE);

  /* Lock and select device */

  ug2864ambag01_lock(priv->spi);
  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, true);

  /* Visit each page */

  for (page = 0; page < UG2864AMBAG01_DEV_PAGES; page++)
    {
      /* Select command transfer */

      SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, true);

      /* Set the column address to the XOFFSET value */
  
      SPI_SEND(priv->spi, SH1101A_SETCOLL(UG2864AMBAG01_DEV_XOFFSET));
      SPI_SEND(priv->spi, SH1101A_SETCOLH(0));

      /* Set the page address */

      SPI_SEND(priv->spi, SH1101A_PAGEADDR(page));

      /* Select data transfer */

      SPI_CMDDATA(priv->spi, SPIDEV_DISPLAY, false);

      /* Transfer one page of the selected color */

      (void)SPI_SNDBLOCK(priv->spi, &priv->fb[page * UG2864AMBAG01_XRES],
                         UG2864AMBAG01_XRES);
    }

  /* De-select and unlock the device */

  SPI_SELECT(priv->spi, SPIDEV_DISPLAY, false);
  ug2864ambag01_unlock(priv->spi);
}

#endif /* CONFIG_LCD_UG2864AMBAG01 */
