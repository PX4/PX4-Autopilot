/**************************************************************************************
 * drivers/lcd/p14201.c
 * Driver for RiT P14201 series display (wih sd1329 IC controller)
 *
 *   Copyright (C) 2010, 2012 Gregory Nutt. All rights reserved.
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
#include <nuttx/lcd/p14201.h>

#include <arch/irq.h>

#include "sd1329.h"

#ifdef CONFIG_LCD_P14201

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/

/* P14201 Configuration Settings:
 *
 * CONFIG_P14201_SPIMODE - Controls the SPI mode
 * CONFIG_P14201_FREQUENCY - Define to use a different bus frequency
 * CONFIG_P14201_NINTERFACES - Specifies the number of physical P14201 devices that
 *   will be supported.
 * CONFIG_P14201_FRAMEBUFFER - If defined, accesses will be performed using an in-memory
 *   copy of the OLEDs GDDRAM.  This cost of this buffer is 128 * 96 / 2 = 6Kb.  If this
 *   is defined, then the driver will be fully functional. If not, then it will have the
 *   following limitations:
 *
 *   - Reading graphics memory cannot be supported, and
 *   - All pixel writes must be aligned to byte boundaries.
 *
 *   The latter limitation effectively reduces the 128x96 disply to 64x96.
 *
 * Required LCD driver settings:
 * CONFIG_LCD_P14201 - Enable P14201 support
 * CONFIG_LCD_MAXCONTRAST should be 255, but any value >0 and <=255 will be accepted.
 * CONFIG_LCD_MAXPOWER must be 1
 *
 * Required SPI driver settings:
 * CONFIG_SPI_CMDDATA - Include support for cmd/data selection.
 */

#ifndef CONFIG_SPI_CMDDATA
#  error "CONFIG_SPI_CMDDATA must be defined in your NuttX configuration"
#endif

/* The P14201 spec says that is supports SPI mode 0,0 only.  However,
 * somtimes you need to tinker with these things.
 */

#ifndef CONFIG_P14201_SPIMODE
#  define CONFIG_P14201_SPIMODE SPIDEV_MODE2
#endif

/* CONFIG_P14201_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_P14201_NINTERFACES
#  define CONFIG_P14201_NINTERFACES 1
#endif

#if CONFIG_P14201_NINTERFACES != 1
#  error "This implementation supports only a single OLED device"
#endif

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 255
#endif

#if CONFIG_LCD_MAXCONTRAST <= 0|| CONFIG_LCD_MAXCONTRAST > 255
#  error "CONFIG_LCD_MAXCONTRAST exceeds supported maximum"
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER != 1
#  warning "CONFIG_LCD_MAXPOWER exceeds supported maximum"
#  undef CONFIG_LCD_MAXPOWER
#  define CONFIG_LCD_MAXPOWER 1
#endif

/* Color is 4bpp greyscale with leftmost column contained in bits 7:4  */

#if defined(CONFIG_NX_DISABLE_4BPP) || !defined(CONFIG_NX_PACKEDMSFIRST)
#  warning "4-bit, big-endian pixel support needed"
#endif

/* Define the CONFIG_LCD_RITDEBUG to enable detailed debug output (stuff you would
 * never want to see unless you are debugging this file).
 *
 * Verbose debug must also be enabled
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_RITDEBUG
#endif

/* Color Properties *******************************************************************/

/* Display Resolution */

#define RIT_XRES         128
#define RIT_YRES         96

/* Color depth and format */

#define RIT_BPP          4
#define RIT_COLORFMT     FB_FMT_Y4

/* Default contrast */

#define RIT_CONTRAST    ((23 * (CONFIG_LCD_MAXCONTRAST+1) / 32) - 1)

/* Helper Macros **********************************************************************/

#define rit_sndcmd(p,b,l)  rit_sndbytes(p,b,l,true);
#define rit_snddata(p,b,l) rit_sndbytes(p,b,l,false);

/* Debug ******************************************************************************/

#ifdef CONFIG_LCD_RITDEBUG
#  define ritdbg(format, arg...)  vdbg(format, ##arg)
#else
#  define ritdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct rit_dev_s
{
  struct lcd_dev_s       dev;      /* Publically visible device structure */
  FAR struct spi_dev_s  *spi;      /* Cached SPI device reference */
  uint8_t                contrast; /* Current contrast setting */
  bool                   on;       /* true: display is on */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* Low-level SPI helpers */

static inline void rit_configspi(FAR struct spi_dev_s *spi);
#ifdef CONFIG_SPI_OWNBUS
static inline void rit_select(FAR struct spi_dev_s *spi);
static inline void rit_deselect(FAR struct spi_dev_s *spi);
#else
static void rit_select(FAR struct spi_dev_s *spi);
static void rit_deselect(FAR struct spi_dev_s *spi);
#endif
static void rit_sndbytes(FAR struct rit_dev_s *priv, FAR const uint8_t *buffer,
              size_t buflen, bool cmd);
static void rit_sndcmds(FAR struct rit_dev_s *priv, FAR const uint8_t *table);

/* LCD Data Transfer Methods */

static int rit_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int rit_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int rit_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int rit_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int rit_getpower(struct lcd_dev_s *dev);
static int rit_setpower(struct lcd_dev_s *dev, int power);
static int rit_getcontrast(struct lcd_dev_s *dev);
static int rit_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

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

static uint8_t g_runbuffer[RIT_XRES / 2];

/* CONFIG_P14201_FRAMEBUFFER - If defined, accesses will be performed using an in-memory
 *   copy of the OLEDs GDDRAM.  This cost of this buffer is 128 * 64 / 2 = 4Kb.  If this
 *   is defined, then the driver will be full functional. If not, then:
 *
 *   - Reading graphics memory cannot be supported, and
 *   - All pixel writes must be aligned to byte boundaries.
 */

#ifdef CONFIG_P14201_FRAMEBUFFER
static uint8_t g_framebuffer[RIT_YRES * RIT_XRES / 2];
#endif

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = RIT_COLORFMT,         /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = RIT_XRES,             /* Horizontal resolution in pixel columns */
  .yres    = RIT_YRES,             /* Vertical resolution in pixel rows */
  .nplanes = 1,                    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = rit_putrun,            /* Put a run into LCD memory */
  .getrun = rit_getrun,            /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = RIT_BPP,               /* Bits-per-pixel */
};

/* This is the OLED driver instance (only a single device is supported for now) */

static struct rit_dev_s g_oleddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = rit_getvideoinfo,
    .getplaneinfo = rit_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = rit_getpower,
    .setpower     = rit_setpower,
    .getcontrast  = rit_getcontrast,
    .setcontrast  = rit_setcontrast,
  },
};

/* A table of magic initialization commands. This initialization sequence is
 * derived from RiT Application Note for the P14201 (with a few tweaked values
 * as discovered in some Luminary code examples).
 */

static const uint8_t g_initcmds[] =
{
  3,  SSD1329_CMD_LOCK,                 /* Set lock command */
      SSD1329_LOCK_OFF,                 /* Disable locking */
      SSD1329_NOOP,
  2,  SSD1329_SLEEP_ON,                 /* Matrix display OFF */
      SSD1329_NOOP,
  3,  SSD1329_ICON_ALL,                 /* Set all ICONs to OFF */
      SSD1329_ICON_OFF,                 /* OFF selection */
      SSD1329_NOOP,
  3,  SSD1329_MUX_RATIO,                /* Set MUX ratio */
      95,                               /* 96 MUX */
      SSD1329_NOOP,
  3,  SSD1329_SET_CONTRAST,             /* Set contrast */
      RIT_CONTRAST,                     /* Default contrast */
      SSD1329_NOOP,
  3,  SSD1329_PRECHRG2_SPEED,           /* Set second pre-charge speed */
     (31 << 1) | SSD1329_PRECHRG2_DBL,  /* Pre-charge speed == 32, doubled */
      SSD1329_NOOP,
  3,  SSD1329_GDDRAM_REMAP,             /* Set GDDRAM re-map */
     (SSD1329_COM_SPLIT|                /* Enable COM slip even/odd */
      SSD1329_COM_REMAP|                /* Enable COM re-map */
      SSD1329_NIBBLE_REMAP),            /* Enable nibble re-map */
      SSD1329_NOOP,
  3,  SSD1329_VERT_START,               /* Set Display Start Line */
      0,                                /* Line = 0 */
      SSD1329_NOOP,
  3,  SSD1329_VERT_OFFSET,              /* Set Display Offset */
      0,                                /* Offset = 0 */
      SSD1329_NOOP,
  2,  SSD1329_DISP_NORMAL,              /* Display mode normal */
      SSD1329_NOOP,
  3,  SSD1329_PHASE_LENGTH,             /* Set Phase Length */
      1 |                               /* Phase 1 period = 1 DCLK */
     (1 << 4),                          /* Phase 2 period = 1 DCLK */
      SSD1329_NOOP,
  3,  SSD1329_FRAME_FREQ,
      35,                               /* 35 DCLK's per row */
      SSD1329_NOOP,
  3,  SSD1329_DCLK_DIV,                 /* Set Front Clock Divider / Oscillator Frequency */
      2 |                               /* Divide ration = 3 */
     (14 << 4),                         /* Oscillator Frequency, FOSC, setting */
      SSD1329_NOOP,
  17, SSD1329_GSCALE_LOOKUP,            /* Look Up Table for Gray Scale Pulse width */
       1,  2,  3,  4,  5,               /* Value for GS1-5 level Pulse width */
       6,  8, 10, 12, 14,               /* Value for GS6-10 level Pulse width */
      16, 19, 22, 26, 30,               /* Value for GS11-15 level Pulse width */
      SSD1329_NOOP,
  3,  SSD1329_PRECHRG2_PERIOD,          /* Set Second Pre-charge Period */
      1,                                /* 1 DCLK */
      SSD1329_NOOP,
  3,  SSD1329_PRECHRG1_VOLT,            /* Set First Precharge voltage, VP */
      0x3f,                             /* 1.00 x Vcc */
      SSD1329_NOOP,
  0                                     /* Zero length command terminates table */
};

/* Turn the maxtrix display on (sleep mode off) */

static const uint8_t g_sleepoff[] =
{
  SSD1329_SLEEP_OFF,                    /* Matrix display ON */
  SSD1329_NOOP,
};

/* Turn the maxtrix display off (sleep mode on) */

static const uint8_t g_sleepon[] =
{
  SSD1329_SLEEP_ON,                     /* Matrix display OFF */
  SSD1329_NOOP,
};

/* Set horizontal increment mode */

static const uint8_t g_horzinc[] =
{
  SSD1329_GDDRAM_REMAP,
 (SSD1329_COM_SPLIT|SSD1329_COM_REMAP|SSD1329_NIBBLE_REMAP),
};

/* The following set a window that covers the entire display */

static const uint8_t g_setallcol[] =
{
  SSD1329_SET_COLADDR,
  0,
  (RIT_XRES/2)-1
};

static const uint8_t g_setallrow[] =
{
  SSD1329_SET_ROWADDR,
  0,
  RIT_YRES-1
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: rit_configspi
 *
 * Description:
 *   Configure the SPI for use with the P14201
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

static inline void rit_configspi(FAR struct spi_dev_s *spi)
{
#ifdef CONFIG_P14201_FREQUENCY
  ritdbg("Mode: %d Bits: 8 Frequency: %d\n",
         CONFIG_P14201_SPIMODE, CONFIG_P14201_FREQUENCY);
#else
  ritdbg("Mode: %d Bits: 8\n", CONFIG_P14201_SPIMODE);
#endif

  /* Configure SPI for the P14201.  But only if we own the SPI bus.  Otherwise, don't
   * bother because it might change.
   */

#ifdef CONFIG_SPI_OWNBUS
  SPI_SETMODE(spi, CONFIG_P14201_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_P14201_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_P14201_FREQUENCY)
#endif
#endif
}

/**************************************************************************************
 * Name: rit_select
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

#ifdef CONFIG_SPI_OWNBUS
static inline void rit_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, true);
}
#else
static void rit_select(FAR struct spi_dev_s *spi)
{
  /* Select P14201 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Now make sure that the SPI bus is configured for the P14201 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_P14201_SPIMODE);
  SPI_SETBITS(spi, 8);
#ifdef CONFIG_P14201_FREQUENCY
  SPI_SETFREQUENCY(spi, CONFIG_P14201_FREQUENCY);
#endif
}
#endif

/**************************************************************************************
 * Name: rit_deselect
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

#ifdef CONFIG_SPI_OWNBUS
static inline void rit_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
}
#else
static void rit_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select P14201 chip and relinquish the SPI bus. */

  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name: rit_sndbytes
 *
 * Description:
 *   Send a sequence of command or data bytes to the SSD1329 controller.
 *
 * Input Parameters:
 *   spi    - Reference to the SPI driver structure
 *   buffer - A reference to memory containing the command bytes to be sent.
 *   buflen - The number of command bytes in buffer to be sent
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The caller as selected the OLED device.
 *
 **************************************************************************************/

static void rit_sndbytes(FAR struct rit_dev_s *priv, FAR const uint8_t *buffer,
                         size_t buflen, bool cmd)
{
  FAR struct spi_dev_s *spi = priv->spi;
  uint8_t tmp;

  ritdbg("buflen: %d cmd: %s [%02x %02x %02x]\n",
         buflen, cmd ? "YES" : "NO", buffer[0], buffer[1], buffer[2] );
  DEBUGASSERT(spi);

  /* Clear/set the D/Cn bit to enable command or data mode */

  (void)SPI_CMDDATA(spi, SPIDEV_DISPLAY, cmd);

  /* Loop until the entire command/data block is transferred */

  while (buflen-- > 0)
    {
      /* Write the next byte to the controller */

      tmp = *buffer++;
      (void)SPI_SEND(spi, tmp);
   }
}

/**************************************************************************************
 * Name: rit_sndcmd
 *
 * Description:
 *   Send multiple commands from a table of commands.
 *
 * Input Parameters:
 *   spi    - Reference to the SPI driver structure
 *   table  - A reference to table containing all of the commands to be sent.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 **************************************************************************************/

static void rit_sndcmds(FAR struct rit_dev_s *priv, FAR const uint8_t *table)
{
  int cmdlen;

  /* Table terminates with a zero length command */

  while ((cmdlen = *table++) != 0)
    {
      ritdbg("command: %02x cmdlen: %d\n", *table, cmdlen);
      rit_sndcmd(priv, table, cmdlen);
      table += cmdlen;
    }
}

/**************************************************************************************
 * Name:  rit_clear
 *
 * Description:
 *   This method can be used to clear the entire display.
 *
 * Input Parameters:
 *   priv   - Reference to private driver structure
 *
 * Assumptions:
 *   Caller has selected the OLED section.
 *
 **************************************************************************************/

#ifdef CONFIG_P14201_FRAMEBUFFER
static inline void rit_clear(FAR struct rit_dev_s *priv)
{
  FAR uint8_t *ptr = g_framebuffer;
  unsigned int row;

  ritdbg("Clear display\n");

  /* Initialize the framebuffer */

  memset(g_framebuffer, (RIT_Y4_BLACK << 4) | RIT_Y4_BLACK, RIT_YRES * RIT_XRES / 2);

  /* Set a window to fill the entire display */

  rit_sndcmd(priv, g_setallcol, sizeof(g_setallcol));
  rit_sndcmd(priv, g_setallrow, sizeof(g_setallrow));
  rit_sndcmd(priv, g_horzinc,   sizeof(g_horzinc));

  /* Display each row */

  for(row = 0; row < RIT_YRES; row++)
    {
      /* Display a horizontal run */

      rit_snddata(priv, ptr, RIT_XRES / 2);
      ptr += RIT_XRES / 2;
    }
}
#else
static inline void rit_clear(FAR struct rit_dev_s *priv)
{
  unsigned int row;

  ritdbg("Clear display\n");

  /* Create a black row */

  memset(g_runbuffer, (RIT_Y4_BLACK << 4) | RIT_Y4_BLACK, RIT_XRES / 2);

  /* Set a window to fill the entire display */

  rit_sndcmd(priv, g_setallcol, sizeof(g_setallcol));
  rit_sndcmd(priv, g_setallrow, sizeof(g_setallrow));
  rit_sndcmd(priv, g_horzinc,   sizeof(g_horzinc));

  /* Display each row */

  for(row = 0; row < RIT_YRES; row++)
    {
      /* Display a horizontal run */

      rit_snddata(priv, g_runbuffer, RIT_XRES / 2);
    }
}
#endif

/**************************************************************************************
 * Name:  rit_putrun
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

#ifdef CONFIG_P14201_FRAMEBUFFER
static int rit_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                      size_t npixels)
{
  FAR struct rit_dev_s *priv = (FAR struct rit_dev_s *)&g_oleddev;
  uint8_t cmd[3];
  uint8_t *run;
  int start;
  int end;
  int aend;
  int i;

  ritdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Toss out the special case of the empty run now */

  if (npixels < 1)
    {
      return OK;
    }

  /* Get the beginning of the line containing run in the framebuffer */

  run = g_framebuffer + row * RIT_XRES / 2;

  /* Get the starting and ending byte offsets containing the run.
   * the run starts at &run[start] and continues through run[end-1].
   * However, the first and final pixels at these locations may
   * not be byte aligned.
   */

  start = col >> 1;
  aend  = (col + npixels) >> 1;
  end   = (col + npixels + 1) >> 1;
  ritdbg("start: %d aend: %d end: %d\n", start, aend, end);

  /* Copy the run into the framebuffer, handling nibble alignment.
   *
   * CASE 1: First pixel X position is byte aligned
   *
   *  example col=6 npixels = 8           example col=6 npixels=7
   *
   *  Run:    |AB|AB|AB|AB|               |AB|AB|AB|AB|
   *  GDDRAM row:
   *  Byte    | 0| 1| 2| 3| 4| 5| 6|      | 0| 1| 2| 3| 4| 5| 6|
   *  Pixel:  |--|--|--|AB|AB|AB|AB|      |--|--|--|AB|AB|AB|A-|
   *
   *  start = 3                           start = 3
   *  aend  = 6                           aend  = 6
   *  end   = 6                           end   = 7
   *
   */

  if ((col & 1) == 0)
    {
      /* Check for the special case of only 1 pixel being blitted */

      if (npixels > 1)
        {
          /* Beginning of buffer is properly aligned, from start to aend */

          memcpy(&run[start], buffer, aend - start);
        }

       /* An even number of byte-aligned pixel pairs have been written (where
        * zero counts as an even number).  If npixels was was odd (including
        * npixels == 1), then handle the final, byte aligned pixel.
        */

       if (aend != end)
         {
           /* The leftmost column is contained in source bits 7:4 and in
            * destination bits 7:4
            */

           run[aend] = (run[aend] & 0x0f) | (buffer[aend - start] & 0xf0);
         }
    }

  /* CASE 2: First pixel X position is byte aligned
   *
   *  example col=7 npixels = 8           example col=7 npixels=7
   *
   *  Run:    |AB|AB|AB|AB|               |AB|AB|AB|AB|
   *  GDDRAM row:
   *  Byte    | 0| 1| 2| 3| 4| 5| 6| 7|   | 0| 1| 2| 3| 4| 5| 6|
   *  Pixel:  |--|--|--|-A|BA|BA|BA|B-|   |--|--|--|-A|BA|BA|BA|
   *
   *  start = 3                           start = 3
   *  aend  = 7                           aend  = 7
   *  end   = 8                           end   = 7
   */

  else
    {
      uint8_t curr = buffer[0];
      uint8_t last;

      /* Handle the initial unaligned pixel. Source bits 7:4 into
       * destination bits 3:0.  In the special case of npixel == 1,
       * this finished the job.
       */

      run[start] = (run[start] & 0xf0) | (curr >> 4);

      /* Now construct the rest of the bytes in the run (possibly special
       * casing the final, partial byte below).
       */

      for (i = start + 1; i < aend; i++)
        {
          /* bits 3:0 from previous byte to run bits 7:4;
           * bits 7:4 of current byte to run bits 3:0
           */

          last   = curr;
          curr   = buffer[i-start];
          run[i] = (last << 4) | (curr >> 4);
        }

       /* An odd number of unaligned pixel have been written (where npixels
        * may have been as small as one).  If npixels was was even, then handle
        * the final, unaligned pixel.
        */

      if (aend != end)
        {
          /* The leftmost column is contained in source bits 3:0 and in
           * destination bits 7:4
           */

          run[aend] = (run[aend] & 0x0f) | (curr << 4);
        }
    }

  /* Select the SD1329 controller */

  rit_select(priv->spi);

  /* Setup a window that describes a run starting at the specified column
   * and row, and ending at the column + npixels on the same row.
   */

  cmd[0] = SSD1329_SET_COLADDR;
  cmd[1] = start;
  cmd[2] = end - 1;
  rit_sndcmd(priv, cmd, 3);

  cmd[0] = SSD1329_SET_ROWADDR;
  cmd[1] = row;
  cmd[2] = row;
  rit_sndcmd(priv, cmd, 3);

  /* Write the run to GDDRAM. */

  rit_sndcmd(priv, g_horzinc, sizeof(g_horzinc));
  rit_snddata(priv, &run[start], end - start);

  /* De-select the SD1329 controller */

  rit_deselect(priv->spi);
  return OK;
}
#else
static int rit_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                      size_t npixels)
{
  FAR struct rit_dev_s *priv = (FAR struct rit_dev_s *)&g_oleddev;
  uint8_t cmd[3];

  ritdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  if (npixels > 0)
    {
      /* Check that the X and Y coordinates are within range */

      DEBUGASSERT(col < RIT_XRES && (col + npixels) <= RIT_XRES && row < RIT_YRES);

      /* Check that the X coordinates are aligned to 8-bit boundaries
       * (this needs to get fixed somehow)
       */

      DEBUGASSERT((col & 1) == 0 && (npixels & 1) == 0);

      /* Select the SD1329 controller */

      rit_select(priv->spi);

      /* Setup a window that describes a run starting at the specified column
       * and row, and ending at the column + npixels on the same row.
       */

      cmd[0] = SSD1329_SET_COLADDR;
      cmd[1] = col >> 1;
      cmd[2] = ((col + npixels) >> 1) - 1;
      rit_sndcmd(priv, cmd, 3);

      cmd[0] = SSD1329_SET_ROWADDR;
      cmd[1] = row;
      cmd[2] = row;
      rit_sndcmd(priv, cmd, 3);

      /* Write the run to GDDRAM. */

      rit_sndcmd(priv, g_horzinc, sizeof(g_horzinc));
      rit_snddata(priv, buffer, npixels >> 1);

      /* De-select the SD1329 controller */

      rit_deselect(priv->spi);
    }

  return OK;
}
#endif

/**************************************************************************************
 * Name:  rit_getrun
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

#ifdef CONFIG_P14201_FRAMEBUFFER
static int rit_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  uint8_t *run;
  int start;
  int end;
  int aend;
  int i;

  ritdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer);

  /* Can't read from OLED GDDRAM in SPI mode, but we can read from the framebuffer */
  /* Toss out the special case of the empty run now */

  if (npixels < 1)
    {
      return OK;
    }

  /* Get the beginning of the line containing run in the framebuffer */

  run = g_framebuffer + row * RIT_XRES / 2;

  /* Get the starting and ending byte offsets containing the run.
   * the run starts at &run[start] and continues through run[end-1].
   * However, the first and final pixels at these locations may
   * not be byte aligned (see examples in putrun()).
   */

  start = col >> 1;
  aend  = (col + npixels) >> 1;
  end   = (col + npixels + 1) >> 1;

  /* Copy the run into the framebuffer, handling nibble alignment */

  if ((col & 1) == 0)
    {
      /* Check for the special case of only 1 pixels being copied */

      if (npixels > 1)
        {
          /* Beginning of buffer is properly aligned, from start to aend */

          memcpy(buffer, &run[start], aend - start + 1);
        }

      /* Handle any final pixel (including the special case where npixels == 1). */

      if (aend != end)
        {
          /* The leftmost column is contained in source bits 7:4 and in
           * destination bits 7:4
           */

          buffer[aend - start] = run[aend] & 0xf0;
        }
    }
  else
    {
      uint8_t curr = run[start];
      uint8_t last;

      /* Now construct the rest of the bytes in the run (possibly special
       * casing the final, partial byte below).
       */

      for (i = start + 1; i < aend; i++)
        {
          /* bits 3:0 from previous byte to run bits 7:4;
           * bits 7:4 of current byte to run bits 3:0
           */

          last = curr;
          curr = run[i];
          *buffer++ = (last << 4) | (curr >> 4);
        }

      /* Handle any final pixel (including the special case where npixels == 1). */

      if (aend != end)
        {
          /* The leftmost column is contained in source bits 3:0 and in
           * destination bits 7:4
           */

          *buffer = (curr << 4);
        }
    }

  return OK;
}
#else
static int rit_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  /* Can't read from OLED GDDRAM in SPI mode */

  return -ENOSYS;
}
#endif

/**************************************************************************************
 * Name:  rit_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int rit_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
        g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  rit_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int rit_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  rit_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int rit_getpower(FAR struct lcd_dev_s *dev)
{
  FAR struct rit_dev_s *priv = (FAR struct rit_dev_s *)dev;
  DEBUGASSERT(priv);

  gvdbg("power: %s\n", priv->on ? "ON" : "OFF");
  return priv->on ? CONFIG_LCD_MAXPOWER : 0;
}

/**************************************************************************************
 * Name:  rit_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int rit_setpower(struct lcd_dev_s *dev, int power)
{
  struct rit_dev_s *priv = (struct rit_dev_s *)dev;
  DEBUGASSERT(priv && (unsigned)power <= CONFIG_LCD_MAXPOWER && priv->spi);

  gvdbg("power: %d\n", power);

  /* Select the SD1329 controller */

  rit_select(priv->spi);

  /* Only two power settings -- 0: sleep on, 1: sleep off */

  if (power > 0)
    {
      /* Re-initialize the SSD1329 controller */

      rit_sndcmds(priv, g_initcmds);

      /* Take the display out of sleep mode */

      rit_sndcmd(priv, g_sleepoff, sizeof(g_sleepoff));
      priv->on = true;
    }
  else
    {
      /* Put the display into sleep mode */

      rit_sndcmd(priv, g_sleepon, sizeof(g_sleepon));
      priv->on = false;
    }

  /* De-select the SD1329 controller */

  rit_deselect(priv->spi);
  return OK;
}

/**************************************************************************************
 * Name:  rit_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int rit_getcontrast(struct lcd_dev_s *dev)
{
  struct rit_dev_s *priv = (struct rit_dev_s *)dev;

  gvdbg("contrast: %d\n", priv->contrast);
  return priv->contrast;
}

/**************************************************************************************
 * Name:  rit_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int rit_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct rit_dev_s *priv = (struct rit_dev_s *)dev;
  uint8_t cmd[3];

  gvdbg("contrast: %d\n", contrast);
  DEBUGASSERT(contrast <= CONFIG_LCD_MAXCONTRAST);

  /* Select the SD1329 controller */

  rit_select(priv->spi);

  /* Set new contrast */

  cmd[0] = SSD1329_SET_CONTRAST;
  cmd[1] = contrast;
  cmd[2] = SSD1329_NOOP;
  rit_sndcmd(priv, cmd, 3);

  /* De-select the SD1329 controller */

  rit_deselect(priv->spi);
  priv->contrast = contrast;
  return OK;
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  rit_initialize
 *
 * Description:
 *   Initialize the P14201 video hardware.  The initial state of the OLED is fully
 *   initialized, display memory cleared, and the OLED ready to use, but with the power
 *   setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throuh CONFIG_P14201_NINTERFACES-1.  This allows
 *   support for multiple OLED devices.
 *
 * Returned Value:
 *   On success, this function returns a reference to the LCD object for the specified
 *   OLED.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *rit_initialize(FAR struct spi_dev_s *spi, unsigned int devno)
{
  FAR struct rit_dev_s *priv = (FAR struct rit_dev_s *)&g_oleddev;
  DEBUGASSERT(devno == 0 && spi);

  gvdbg("Initializing devno: %d\n", devno);

  /* Driver state data */

  priv->spi      = spi;
  priv->contrast = RIT_CONTRAST;
  priv->on       = false;

  /* Select the SD1329 controller */

  rit_configspi(spi);
  rit_select(spi);

  /* Clear the display */

  rit_clear(priv);

  /* Configure (but don't enable) the OLED */

  rit_sndcmds(priv, g_initcmds);

  /* De-select the SD1329 controller */

  rit_deselect(spi);
  return &priv->dev;
}
#endif /* CONFIG_LCD_P14201 */
