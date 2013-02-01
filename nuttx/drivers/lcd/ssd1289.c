/**************************************************************************************
 * drivers/lcd/ssd1289.c
 *
 * Generic LCD driver for LCDs based on the Solomon Systech SSD1289 LCD controller.
 * Think of this as a template for an LCD driver that you will proably ahve to
 * customize for any particular LCD hardware.
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: SSD1289, Rev 1.3, Apr 2007, Solomon Systech Limited
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
#include <nuttx/lcd/ssd1289.h>

#include "ssd1289.h"

#ifdef CONFIG_LCD_SSD1289

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  define CONFIG_LCD_MAXPOWER 1
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* Check orientation */

#if defined(CONFIG_LCD_PORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) || defined(CONFIG_LCD_RPORTRAIT)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RPORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_LANDSCAPE)
#  ifdef CONFIG_LCD_RLANDSCAPE
#    error "Cannot define both landscape and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_RLANDSCAPE)
#  define CONFIG_LCD_LANDSCAPE 1
#endif

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

/* Display/Color Properties ***********************************************************/
/* Display Resolution */

#if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE) 
#  define SSD1289_XRES       320
#  define SSD1289_YRES       240
#else
#  define SSD1289_XRES       240
#  define SSD1289_YRES       320
#endif

/* Color depth and format */

#define SSD1289_BPP           16
#define SSD1289_COLORFMT      FB_FMT_RGB16_565

/* LCD Profiles ***********************************************************************/
/* Many details of the controller initialization must, unfortunately, vary from LCD to
 * LCD.  I have looked at the spec and at three different drivers for LCDs that have 
 * SSD1289 controllers.  I have tried to summarize these differences as "LCD profiles"
 *
 * Most of the differences between LCDs are nothing more than a few minor bit
 * settings.  The most significant difference betwen LCD drivers in is the
 * manner in which the LCD is powered up and in how the power controls are set.
 * My suggestion is that if you have working LCD initialization code, you should
 * simply replace the code in ssd1289_hwinitialize with your working code.
 */

#if defined (CONFIG_SSD1289_PROFILE2)
#  undef SSD1289_USE_SIMPLE_INIT

  /* PWRCTRL1:  AP=smalll-to-medium, DC=Flinex24, BT=+5/-4, DCT=Flinex24 */

#  define PWRCTRL1_SETTING \
     (SSD1289_PWRCTRL1_AP_SMMED | SSD1289_PWRCTRL1_DC_FLINEx24 | \
      SSD1289_PWRCTRL1_BT_p5m4  | SSD1289_PWRCTRL1_DCT_FLINEx24)

  /* PWRCTRL2: 5.1v */

#  define PWRCTRL2_SETTING SSD1289_PWRCTRL2_VRC_5p1V

  /* PWRCTRL3: x 2.165
   * NOTE: Many drivers have bit 8 set which is not defined in the SSD1289 spec.
   */

#  define PWRCTRL3_SETTING SSD1289_PWRCTRL3_VRH_x2p165

   /* PWRCTRL4: VDV=9 + VCOMG */

#  define PWRCTRL4_SETTING (SSD1289_PWRCTRL4_VDV(9) | SSD1289_PWRCTRL4_VCOMG)

   /* PWRCTRL5: VCM=56 + NOTP */

#  define PWRCTRL5_SETTING (SSD1289_PWRCTRL5_VCM(56) | SSD1289_PWRCTRL5_NOTP)

#elif defined (CONFIG_SSD1289_PROFILE3)
#  undef SSD1289_USE_SIMPLE_INIT

  /* PWRCTRL1:  AP=smalll-to-medium, DC=Flinex24, BT=+5/-4, DCT=Flinex24 */

#  define PWRCTRL1_SETTING \
     (SSD1289_PWRCTRL1_AP_SMMED | SSD1289_PWRCTRL1_DC_FLINEx24 | \
      SSD1289_PWRCTRL1_BT_p5m4  | SSD1289_PWRCTRL1_DCT_FLINEx24)

  /* PWRCTRL2: 5.1v */

#  define PWRCTRL2_SETTING SSD1289_PWRCTRL2_VRC_5p1V

  /* PWRCTRL3: x 2.165
   * NOTE: Many drivers have bit 8 set which is not defined in the SSD1289 spec.
   */

#  define PWRCTRL3_SETTING SSD1289_PWRCTRL3_VRH_x2p165

   /* PWRCTRL4: VDV=9 + VCOMG */

#  define PWRCTRL4_SETTING (SSD1289_PWRCTRL4_VDV(9) | SSD1289_PWRCTRL4_VCOMG)

   /* PWRCTRL5: VCM=56 + NOTP */

#  define PWRCTRL5_SETTING (SSD1289_PWRCTRL5_VCM(56) | SSD1289_PWRCTRL5_NOTP)

#else /* if defined (CONFIG_SSD1289_PROFILE1) */
#  undef SSD1289_USE_SIMPLE_INIT
#  define SSD1289_USE_SIMPLE_INIT 1

  /* PWRCTRL1:  AP=medium-to-large, DC=Fosc/4, BT=+5/-4, DCT=Fosc/4 */

#  define PWRCTRL1_SETTING \
     (SSD1289_PWRCTRL1_AP_MEDLG | SSD1289_PWRCTRL1_DC_FOSd4 | \
      SSD1289_PWRCTRL1_BT_p5m4  | SSD1289_PWRCTRL1_DCT_FOSd4)

  /* PWRCTRL2: 5.3v */

#  define PWRCTRL2_SETTING SSD1289_PWRCTRL2_VRC_5p3V

  /* PWRCTRL3: x 2.570
   * NOTE: Many drivers have bit 8 set which is not defined in the SSD1289 spec.
   */

#  define PWRCTRL3_SETTING SSD1289_PWRCTRL3_VRH_x2p570

   /* PWRCTRL4: VDV=12 + VCOMG */

#  define PWRCTRL4_SETTING (SSD1289_PWRCTRL4_VDV(12) | SSD1289_PWRCTRL4_VCOMG)

   /* PWRCTRL5: VCM=60 + NOTP */

#  define PWRCTRL5_SETTING (SSD1289_PWRCTRL5_VCM(60) | SSD1289_PWRCTRL5_NOTP)

#endif

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg  dbg
#  define lcdvdbg vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct ssd1289_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct ssd1289_lcd_s *lcd;  /* The contained platform-specific, LCD interface */
  uint8_t power;                  /* Current power setting */

  /* These fields simplify and reduce debug output */

#ifdef CONFIG_DEBUG_LCD
  bool put;                       /* Last raster operation was a putrun */
  fb_coord_t firstrow;            /* First row of the run */
  fb_coord_t lastrow;             /* Last row of the run */
  fb_coord_t col;                 /* Column of the run */
  size_t npixels;                 /* Length of the run */
#endif

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

  uint16_t runbuffer[SSD1289_XRES];
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Low Level LCD access */

static void ssd1289_putreg(FAR struct ssd1289_lcd_s *lcd, uint8_t regaddr,
                           uint16_t regval);
#ifndef CONFIG_LCD_NOGETRUN
static uint16_t ssd1289_readreg(FAR struct ssd1289_lcd_s *lcd, uint8_t regaddr);
#endif
static inline void ssd1289_gramwrite(FAR struct ssd1289_lcd_s *lcd, uint16_t rgbcolor);
#ifndef CONFIG_LCD_NOGETRUN
static inline void ssd1289_readsetup(FAR struct ssd1289_lcd_s *lcd, FAR uint16_t *accum);
static inline uint16_t ssd1289_gramread(FAR struct ssd1289_lcd_s *lcd, FAR uint16_t *accum);
#endif
static void ssd1289_setcursor(FAR struct ssd1289_lcd_s *lcd, uint16_t column,
                              uint16_t row);

/* LCD Data Transfer Methods */

#if 0 /* Sometimes useful */
static void ssd1289_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels);
#else
#  define ssd1289_dumprun(m,r,n)
#endif

#ifdef CONFIG_DEBUG_LCD
static void ssd1289_showrun(FAR struct ssd1289_dev_s *priv, fb_coord_t row,
                            fb_coord_t col, size_t npixels, bool put);
#else
#  define ssd1289_showrun(p,r,c,n,b)
#endif

static int ssd1289_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int ssd1289_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int ssd1289_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int ssd1289_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int ssd1289_getpower(FAR struct lcd_dev_s *dev);
static int ssd1289_setpower(FAR struct lcd_dev_s *dev, int power);
static int ssd1289_getcontrast(FAR struct lcd_dev_s *dev);
static int ssd1289_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline int ssd1289_hwinitialize(FAR struct ssd1289_dev_s *priv);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This driver can support only a signal SSD1289 device.  This is due to an
 * unfortunate decision made whent he getrun and putrun methods were designed. The
 * following is the single SSD1289 driver state instance:
 */

static struct ssd1289_dev_s g_lcddev;

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ssd1289_putreg(lcd, 
 *
 * Description:
 *   Write to an LCD register
 *
 **************************************************************************************/

static void ssd1289_putreg(FAR struct ssd1289_lcd_s *lcd, uint8_t regaddr, uint16_t regval)
{
  /* Set the index register to the register address and write the register contents */

  lcd->index(lcd, regaddr);
  lcd->write(lcd, regval);
}

/**************************************************************************************
 * Name:  ssd1289_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static uint16_t ssd1289_readreg(FAR struct ssd1289_lcd_s *lcd, uint8_t regaddr)
{
  /* Set the index register to the register address and read the register contents */

  lcd->index(lcd, regaddr);
  return lcd->read(lcd);
}
#endif

/**************************************************************************************
 * Name:  ssd1289_gramselect
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 **************************************************************************************/

static inline void ssd1289_gramselect(FAR struct ssd1289_lcd_s *lcd)
{
  lcd->index(lcd, SSD1289_DATA);
}

/**************************************************************************************
 * Name:  ssd1289_gramwrite
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 **************************************************************************************/

static inline void ssd1289_gramwrite(FAR struct ssd1289_lcd_s *lcd, uint16_t data)
{
  lcd->write(lcd, data);
}

/**************************************************************************************
 * Name:  ssd1289_readsetup
 *
 * Description:
 *   Prime the operation by reading one pixel from the GRAM memory if necessary for
 *   this LCD type.  When reading 16-bit gram data, there may be some shifts in the
 *   returned data:
 *
 *   - ILI932x: Discard first dummy read; no shift in the return data
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static inline void ssd1289_readsetup(FAR struct ssd1289_lcd_s *lcd, FAR uint16_t *accum)
{
  /* Read-ahead one pixel */

  *accum = lcd->read(lcd);
}
#endif

/**************************************************************************************
 * Name:  ssd1289_gramread
 *
 * Description:
 *   Read one correctly aligned pixel from the GRAM memory.  Possibly shifting the
 *   data and possibly swapping red and green components.
 *
 *   - ILI932x: Unknown -- assuming colors are in the color order 
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static inline uint16_t ssd1289_gramread(FAR struct ssd1289_lcd_s *lcd, FAR uint16_t *accum)
{
  /* Read the value (GRAM register already selected) */

  return lcd->read(lcd);
}
#endif

/**************************************************************************************
 * Name:  ssd1289_setcursor
 *
 * Description:
 *   Set the cursor position.  In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 **************************************************************************************/

static void ssd1289_setcursor(FAR struct ssd1289_lcd_s *lcd, uint16_t column, uint16_t row)
{
#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
  ssd1289_putreg(lcd, SSD1289_XADDR, column);    /* 0-239 */
  ssd1289_putreg(lcd, SSD1289_YADDR, row);       /* 0-319 */
#elif defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RLANDSCAPE)
  ssd1289_putreg(lcd, SSD1289_XADDR, row);       /* 0-239 */
  ssd1289_putreg(lcd, SSD1289_YADDR, column);    /* 0-319 */
#endif
}

/**************************************************************************************
 * Name:  ssd1289_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 **************************************************************************************/

#if 0 /* Sometimes useful */
static void ssd1289_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels)
{
  int i, j;

  syslog("\n%s:\n", msg);
  for (i = 0; i < npixels; i += 16)
    {
      up_putc(' ');
      syslog(" ");
      for (j = 0; j < 16; j++)
        {
          syslog(" %04x", *run++);
        }
      up_putc('\n');
    }
}
#endif

/**************************************************************************************
 * Name:  ssd1289_showrun
 *
 * Description:
 *   When LCD debug is enabled, try to reduce then amount of ouptut data generated by
 *   ssd1289_putrun and ssd1289_getrun
 *
 **************************************************************************************/

#ifdef CONFIG_DEBUG_LCD
static void ssd1289_showrun(FAR struct ssd1289_dev_s *priv, fb_coord_t row,
                            fb_coord_t col, size_t npixels, bool put)
{
  fb_coord_t nextrow = priv->lastrow + 1;

  /* Has anything changed (other than the row is the next row in the sequence)? */

  if (put == priv->put && row == nextrow && col == priv->col &&
      npixels == priv->npixels)
    {
      /* No, just update the last row */

      priv->lastrow = nextrow;
    }
  else
    {
      /* Yes... then this is the end of the preceding sequence.  Output the last run
       * (if there were more than one run in the sequence).
       */

      if (priv->firstrow != priv->lastrow)
        {
          lcddbg("...\n");
          lcddbg("%s row: %d col: %d npixels: %d\n",
                 priv->put ? "PUT" : "GET",
                 priv->lastrow, priv->col, priv->npixels);
        }

      /* And we are starting a new sequence.  Output the first run of the
       * new sequence
       */

      lcddbg("%s row: %d col: %d npixels: %d\n",
             put ? "PUT" : "GET", row, col, npixels);

      /* And save information about the run so that we can detect continuations
       * of the sequence.
       */

      priv->put      = put;
      priv->firstrow = row;
      priv->lastrow  = row;
      priv->col      = col;
      priv->npixels  = npixels;
    }
}
#endif

/**************************************************************************************
 * Name:  ssd1289_putrun
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

static int ssd1289_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                          size_t npixels)
{
  FAR struct ssd1289_dev_s *priv = &g_lcddev;
  FAR struct ssd1289_lcd_s *lcd = priv->lcd;
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  ssd1289_showrun(priv, row, col, npixels, true);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Select the LCD */

  lcd->select(lcd);

  /* Write the run to GRAM. */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates -- Here the edge away from the row of buttons on
   * the STM3240G-EVAL is used as the top.
   */

  /* Write the GRAM data, manually incrementing X */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      ssd1289_setcursor(lcd, col, row);
      ssd1289_gramselect(lcd);
      ssd1289_gramwrite(lcd, *src);

      /* Increment to the next column */

      src++;
      col++;
    }
#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Convert coordinates -- Here the edge next to the row of buttons on
   * the STM3240G-EVAL is used as the top.
   */

  col = (SSD1289_XRES-1) - col;
  row = (SSD1289_YRES-1) - row;

  /* Set the cursor position */

  ssd1289_setcursor(lcd, col, row);

  /* Then write the GRAM data, auto-decrementing X */

  ssd1289_gramselect(lcd);
  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position (auto-decrements to the next column) */

      ssd1289_gramwrite(lcd, *src);
      src++;
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates.  In this configuration, the top of the display is to the left
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  col = (SSD1289_XRES-1) - col;

  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      ssd1289_setcursor(lcd, row, col);
      ssd1289_gramselect(lcd);
      ssd1289_gramwrite(lcd, *src);

      /* Increment to the next column */

      src++;
      col--;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates.  In this configuration, the top of the display is to the right
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  row = (SSD1289_YRES-1) - row;
  
  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      ssd1289_setcursor(lcd, row, col);
      ssd1289_gramselect(lcd);
      ssd1289_gramwrite(lcd, *src);

      /* Decrement to the next column */

      src++;
      col++;
    }
#endif

  /* De-select the LCD */

  lcd->deselect(lcd);
  return OK;
}

/**************************************************************************************
 * Name:  ssd1289_getrun
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

static int ssd1289_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                          size_t npixels)
{
#ifndef CONFIG_LCD_NOGETRUN
  FAR struct ssd1289_dev_s *priv = &g_lcddev;
  FAR struct ssd1289_lcd_s *lcd = priv->lcd;
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
  uint16_t accum;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  ssd1289_showrun(priv, row, col, npixels, false);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Select the LCD */

  lcd->select(lcd);

  /* Read the run from GRAM. */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates -- Here the edge away from the row of buttons on
   * the STM3240G-EVAL is used as the top.
   */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      ssd1289_setcursor(lcd, row, col);
      ssd1289_gramselect(lcd);
      ssd1289_readsetup(lcd, &accum);
      *dest++ = ssd1289_gramread(lcd, &accum);

      /* Increment to the next column */

      col++;
    }
#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Convert coordinates -- Here the edge next to the row of buttons on
   * the STM3240G-EVAL is used as the top.
   */

  col = (SSD1289_XRES-1) - col;
  row = (SSD1289_YRES-1) - row;

  /* Set the cursor position */

  ssd1289_setcursor(lcd, col, row);

  /* Then read the GRAM data, auto-decrementing Y */

  ssd1289_gramselect(lcd);

  /* Prime the pump for unaligned read data */

  ssd1289_readsetup(lcd, &accum);

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position (autoincrements to the next row) */

      *dest++ = ssd1289_gramread(lcd, &accum);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates.  In this configuration, the top of the display is to the left
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  col = (SSD1289_XRES-1) - col;

  /* Then read the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      ssd1289_setcursor(lcd, row, col);
      ssd1289_gramselect(lcd);
      ssd1289_readsetup(lcd, &accum);
      *dest++ = ssd1289_gramread(lcd, &accum);

      /* Increment to the next column */

      col--;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates.  In this configuration, the top of the display is to the right
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  row = (SSD1289_YRES-1) - row;
  
  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      ssd1289_setcursor(lcd, row, col);
      ssd1289_gramselect(lcd);
      ssd1289_readsetup(lcd, &accum);
      *dest++ = ssd1289_gramread(lcd, &accum);

      /* Decrement to the next column */

      col++;
    }
#endif

  /* De-select the LCD */

  lcd->deselect(lcd);
  return OK;
#else
  return -ENOSYS;
#endif
}

/**************************************************************************************
 * Name:  ssd1289_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int ssd1289_getvideoinfo(FAR struct lcd_dev_s *dev,
                                 FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: 1\n",
          SSD1289_COLORFMT, SSD1289_XRES, SSD1289_YRES);

  vinfo->fmt     = SSD1289_COLORFMT;    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = SSD1289_XRES;        /* Horizontal resolution in pixel columns */
  vinfo->yres    = SSD1289_YRES;        /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                   /* Number of color planes supported */
  return OK;
}

/**************************************************************************************
 * Name:  ssd1289_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int ssd1289_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct ssd1289_dev_s *priv = (FAR struct ssd1289_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, SSD1289_BPP);

  pinfo->putrun = ssd1289_putrun;            /* Put a run into LCD memory */
  pinfo->getrun = ssd1289_getrun;            /* Get a run from LCD memory */
  pinfo->buffer = (uint8_t*)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = SSD1289_BPP;               /* Bits-per-pixel */
  return OK;
}

/**************************************************************************************
 * Name:  ssd1289_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1289_getpower(FAR struct lcd_dev_s *dev)
{
  lcdvdbg("power: %d\n", 0);
  return g_lcddev.power;
}

/**************************************************************************************
 * Name:  ssd1289_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1289_poweroff(FAR struct ssd1289_lcd_s *lcd)
{
  /* Set the backlight off */

  lcd->backlight(lcd, 0);

  /* Turn the display off */

  ssd1289_putreg(lcd, SSD1289_DSPCTRL, 0);

  /* Remember the power off state */

  g_lcddev.power = 0;
  return OK;
}

/**************************************************************************************
 * Name:  ssd1289_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int ssd1289_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct ssd1289_dev_s *priv = (FAR struct ssd1289_dev_s *)dev;
  FAR struct ssd1289_lcd_s *lcd  = priv->lcd;

  lcdvdbg("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Set the backlight level */

      lcd->backlight(lcd, power);

      /* Then turn the display on:
       * D=ON(3) CM=0 DTE=1 GON=1 SPT=0 VLE=0 PT=0
       */

      ssd1289_putreg(lcd, SSD1289_DSPCTRL,
                     (SSD1289_DSPCTRL_ON | SSD1289_DSPCTRL_GON |
                      SSD1289_DSPCTRL_DTE | SSD1289_DSPCTRL_VLE(0))); 

      g_lcddev.power = power;
    }
  else
    {
      /* Turn the display off */

      ssd1289_poweroff(lcd);
    }

  return OK;
}

/**************************************************************************************
 * Name:  ssd1289_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ssd1289_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  ssd1289_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int ssd1289_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  ssd1289_hwinitialize
 *
 * Description:
 *   Initialize the LCD hardware.
 *
 **************************************************************************************/

static inline int ssd1289_hwinitialize(FAR struct ssd1289_dev_s *priv)
{
  FAR struct ssd1289_lcd_s *lcd  = priv->lcd;
#ifndef CONFIG_LCD_NOGETRUN
  uint16_t id;
#endif
  int ret;

  /* Select the LCD */

  lcd->select(lcd);

  /* Read the device ID.  Skip verification of the device ID is the LCD is
   * write-only. What choice do we have?
   */

#ifndef CONFIG_LCD_NOGETRUN
  id = ssd1289_readreg(lcd, SSD1289_DEVCODE);
  if (id != 0)
    {
      lcddbg("LCD ID: %04x\n", id);
    }

  /* If we could not get the ID, then let's just assume that this is an SSD1289.
   * Perhaps we have some early register access issues.  This seems to happen.
   * But then perhaps we should not even bother to read the device ID at all?
   */

  else
    {
      lcddbg("No LCD ID, assuming SSD1289\n");
      id = SSD1289_DEVCODE_VALUE;
    }

  /* Check if the ID is for the SSD1289 */

  if (id == SSD1289_DEVCODE_VALUE)
#endif
    {
      /* LCD controller configuration.  Many details of the controller initialization
       * must, unfortunately, vary from LCD to LCD.  I have looked at the spec and at
       * three different drivers for LCDs that have SSD1289 controllers.  I have tried
       * to summarize these differences as profiles (defined above).  Some other
       * alternatives are noted below.
       *
       * Most of the differences between LCDs are nothing more than a few minor bit
       * settings.  The most significant difference betwen LCD drivers in is the
       * manner in which the LCD is powered up and in how the power controls are set.
       * My suggestion is that if you have working LCD initialization code, you should
       * simply replace the following guesses with your working code.
       */

      /* Most drivers just enable the oscillator */

#ifdef SSD1289_USE_SIMPLE_INIT
      ssd1289_putreg(lcd, SSD1289_OSCSTART, SSD1289_OSCSTART_OSCEN);
#else
      /* But one goes through a more complex start-up sequence.  Something like the
       * following:
       *
       * First, put the display in INTERNAL operation:
       * D=INTERNAL(1) CM=0 DTE=0 GON=1 SPT=0 VLE=0 PT=0
       */

      ssd1289_putreg(lcd, SSD1289_DSPCTRL,
                     (SSD1289_DSPCTRL_INTERNAL | SSD1289_DSPCTRL_GON |
                      SSD1289_DSPCTRL_VLE(0))); 

      /* Then enable the oscillator */

      ssd1289_putreg(lcd, SSD1289_OSCSTART, SSD1289_OSCSTART_OSCEN);

      /* Turn the display on:
       * D=ON(3) CM=0 DTE=0 GON=1 SPT=0 VLE=0 PT=0
       */

      ssd1289_putreg(lcd, SSD1289_DSPCTRL,
                     (SSD1289_DSPCTRL_ON | SSD1289_DSPCTRL_GON |
                      SSD1289_DSPCTRL_VLE(0))); 

     /* Take the LCD out of sleep mode */

      ssd1289_putreg(lcd, SSD1289_SLEEP, 0);
      up_mdelay(30);

      /* Turn the display on:
       * D=INTERNAL(1) CM=0 DTE=1 GON=1 SPT=0 VLE=0 PT=0
       */

      ssd1289_putreg(lcd, SSD1289_DSPCTRL,
                     (SSD1289_DSPCTRL_ON | SSD1289_DSPCTRL_DTE |
                      SSD1289_DSPCTRL_GON | SSD1289_DSPCTRL_VLE(0))); 
#endif

      /* Set up power control registers.  There is a lot of variability
       * from LCD-to-LCD in how the power registers are configured.
       */

      ssd1289_putreg(lcd, SSD1289_PWRCTRL1, PWRCTRL1_SETTING);
      ssd1289_putreg(lcd, SSD1289_PWRCTRL2, PWRCTRL2_SETTING);

      /* One driver adds a delay here.. I doubt that this is really necessary. */
      /* up_mdelay(15); */

      ssd1289_putreg(lcd, SSD1289_PWRCTRL3, PWRCTRL3_SETTING);
      ssd1289_putreg(lcd, SSD1289_PWRCTRL4, PWRCTRL4_SETTING);
      ssd1289_putreg(lcd, SSD1289_PWRCTRL5, PWRCTRL5_SETTING);

      /* One driver does an odd setting of the the driver output control.
       * No idea why.
       */
#if 0
      ssd1289_putreg(lcd, SSD1289_OUTCTRL,
                     (SSD1289_OUTCTRL_MUX(12) | SSD1289_OUTCTRL_TB |
                      SSD1289_OUTCTRL_BGR | SSD1289_OUTCTRL_CAD));

      /* The same driver does another small delay here */

      up_mdelay(15);
#endif

      /* After this point, the drivers differ only in some varying register
       * bit settings.
       */

      /* Set the driver output control.
       * PORTRAIT MODES:
       *    MUX=319, TB=1, SM=0, BGR=1, CAD=0, REV=1, RL=0
       * LANDSCAPE MODES:
       *    MUX=319, TB=0, SM=0, BGR=1, CAD=0, REV=1, RL=0
       */

#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
      ssd1289_putreg(lcd, SSD1289_OUTCTRL,
                     (SSD1289_OUTCTRL_MUX(319) | SSD1289_OUTCTRL_TB |
                      SSD1289_OUTCTRL_BGR      | SSD1289_OUTCTRL_REV);
#else
      ssd1289_putreg(lcd, SSD1289_OUTCTRL,
                     (SSD1289_OUTCTRL_MUX(319) | SSD1289_OUTCTRL_BGR |
                      SSD1289_OUTCTRL_REV));
#endif

      /* Set the LCD driving AC waveform
       * NW=0, WSMD=0, EOR=1, BC=1, ENWD=0, FLD=0
       */

      ssd1289_putreg(lcd, SSD1289_ACCTRL,
                     (SSD1289_ACCTRL_EOR | SSD1289_ACCTRL_BC));

      /* Take the LCD out of sleep mode (isn't this redundant in the non-
       * simple case?)
       */

      ssd1289_putreg(lcd, SSD1289_SLEEP, 0);

      /* Set entry mode */

#if defined(CONFIG_LCD_PORTRAIT) || defined(CONFIG_LCD_RPORTRAIT)
      /* LG=0, AM=0, ID=3, TY=2, DMODE=0, WMODE=0, OEDEF=0, TRANS=0, DRM=3
       * Alternative TY=2 (But TY only applies in 262K color mode anyway)
       */

      ssd1289_putreg(lcd, SSD1289_ENTRY,
                    (SSD1289_ENTRY_ID_HINCVINC | SSD1289_ENTRY_TY_C |
                     SSD1289_ENTRY_DMODE_RAM | SSD1289_ENTRY_DFM_65K));
#else
      /* LG=0, AM=1, ID=3, TY=2, DMODE=0, WMODE=0, OEDEF=0, TRANS=0, DRM=3 */
      /* Alternative TY=2 (But TY only applies in 262K color mode anyway) */

      ssd1289_putreg(lcd, SSD1289_ENTRY,
                    (SSD1289_ENTRY_AM | SSD1289_ENTRY_ID_HINCVINC |
                     SSD1289_ENTRY_TY_C | SSD1289_ENTRY_DMODE_RAM |
                     SSD1289_ENTRY_DFM_65K));
#endif

      /* Clear compare registers */

      ssd1289_putreg(lcd, SSD1289_CMP1, 0);
      ssd1289_putreg(lcd, SSD1289_CMP2, 0);

      /* One driver puts a huge, 100 millisecond delay here */
      /* up_mdelay(100); */

      /* Set Horizontal and vertical porch.
       * Horizontal porch:  239 pixels per line, delay=28
       * Vertical porch:    VBP=3, XFP=0
       */

      ssd1289_putreg(lcd, SSD1289_HPORCH,
                     (28 << SSD1289_HPORCH_HBP_SHIFT) | (239 << SSD1289_HPORCH_XL_SHIFT));
      ssd1289_putreg(lcd, SSD1289_VPORCH,
                     (3 << SSD1289_VPORCH_VBP_SHIFT)  | (0 << SSD1289_VPORCH_XFP_SHIFT));

      /* Set display control.
       * D=ON(3), CM=0 (not 8-color), DTE=1, GON=1, SPT=0, VLE=1 PT=0
       */

      ssd1289_putreg(lcd, SSD1289_DSPCTRL,
                     (SSD1289_DSPCTRL_ON  | SSD1289_DSPCTRL_DTE |
                      SSD1289_DSPCTRL_GON | SSD1289_DSPCTRL_VLE(1))); 

      /* Frame cycle control.  Alternative: SSD1289_FCYCCTRL_DIV8 */

      ssd1289_putreg(lcd, SSD1289_FCYCCTRL, 0);

      /* Gate scan start position = 0 */

      ssd1289_putreg(lcd, SSD1289_GSTART, 0);

      /* Clear vertical scrolling */

      ssd1289_putreg(lcd, SSD1289_VSCROLL1, 0);
      ssd1289_putreg(lcd, SSD1289_VSCROLL2, 0);

      /* Setup window 1 (0-319) */

      ssd1289_putreg(lcd, SSD1289_W1START, 0);
      ssd1289_putreg(lcd, SSD1289_W1END, 319);

      /* Disable window 2 (0-0) */

      ssd1289_putreg(lcd, SSD1289_W2START, 0);
      ssd1289_putreg(lcd, SSD1289_W2END, 0);

      /* Horizontal start and end (0-239) */

      ssd1289_putreg(lcd, SSD1289_HADDR,
                    (0 << SSD1289_HADDR_HSA_SHIFT) | (239 << SSD1289_HADDR_HEA_SHIFT));

      /* Vertical start and end (0-319) */

      ssd1289_putreg(lcd, SSD1289_VSTART, 0);
      ssd1289_putreg(lcd, SSD1289_VEND, 319);

      /* Gamma controls */

      ssd1289_putreg(lcd, SSD1289_GAMMA1, 0x0707);
      ssd1289_putreg(lcd, SSD1289_GAMMA2, 0x0204); /* Alternative: 0x0704 */
      ssd1289_putreg(lcd, SSD1289_GAMMA3, 0x0204);
      ssd1289_putreg(lcd, SSD1289_GAMMA4, 0x0502);
      ssd1289_putreg(lcd, SSD1289_GAMMA5, 0x0507);
      ssd1289_putreg(lcd, SSD1289_GAMMA6, 0x0204);
      ssd1289_putreg(lcd, SSD1289_GAMMA7, 0x0204);
      ssd1289_putreg(lcd, SSD1289_GAMMA8, 0x0502);
      ssd1289_putreg(lcd, SSD1289_GAMMA9, 0x0302);
      ssd1289_putreg(lcd, SSD1289_GAMMA10, 0x0302); /* Alternative: 0x1f00 */

      /* Clear write mask */

      ssd1289_putreg(lcd, SSD1289_WRMASK1, 0);
      ssd1289_putreg(lcd, SSD1289_WRMASK2, 0);

      /* Set frame frequency = 65Hz (This should not be necessary since this
       * is the default POR value)
       */

      ssd1289_putreg(lcd, SSD1289_FFREQ, SSD1289_FFREQ_OSC_FF65);

      /* Set the cursor at the home position and set the index register to
       * the gram data register (I can't imagine these are necessary).
       */

      ssd1289_setcursor(lcd, 0, 0);
      ssd1289_gramselect(lcd);

      /* One driver has a 50 msec delay here */
      /* up_mdelay(50); */

      ret = OK;
    }
#ifndef CONFIG_LCD_NOGETRUN
  else
    {
      lcddbg("Unsupported LCD type\n");
      ret = -ENODEV;
    }
#endif

  /* De-select the LCD */

  lcd->deselect(lcd);
  return ret;
}

 /*************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  ssd1289_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

FAR struct lcd_dev_s *ssd1289_lcdinitialize(FAR struct ssd1289_lcd_s *lcd)
{
  int ret;

  lcdvdbg("Initializing\n");

  /* If we ccould support multiple SSD1289 devices, this is where we would allocate
   * a new driver data structure... but we can't.  Why not?  Because of a bad should
   * the form of the getrun() and putrun methods.
   */

  FAR struct ssd1289_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = ssd1289_getvideoinfo;
  priv->dev.getplaneinfo = ssd1289_getplaneinfo;
  priv->dev.getpower     = ssd1289_getpower;
  priv->dev.setpower     = ssd1289_setpower;
  priv->dev.getcontrast  = ssd1289_getcontrast;
  priv->dev.setcontrast  = ssd1289_setcontrast;
  priv->lcd              = lcd;

  /* Configure and enable LCD */

  ret = ssd1289_hwinitialize(priv);
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      ssd1289_clear(&priv->dev, 0);

      /* Turn the display off */

      ssd1289_poweroff(lcd);
      return &g_lcddev.dev;
    }

  return NULL;
}

/**************************************************************************************
 * Name:  ssd1289_clear
 *
 * Description:
 *   This is a non-standard LCD interface just for the stm3240g-EVAL board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 **************************************************************************************/

void ssd1289_clear(FAR struct lcd_dev_s *dev, uint16_t color)
{
  FAR struct ssd1289_dev_s *priv = (FAR struct ssd1289_dev_s *)dev;
  FAR struct ssd1289_lcd_s *lcd  = priv->lcd;
  uint32_t i;

  /* Select the LCD and home the cursor position */

  lcd->select(lcd);
  ssd1289_setcursor(lcd, 0, 0);

  /* Prepare to write GRAM data */

  ssd1289_gramselect(lcd);

  /* Copy color into all of GRAM.  Orientation does not matter in this case. */

  for (i = 0; i < SSD1289_XRES * SSD1289_YRES; i++)
    {
      ssd1289_gramwrite(lcd, color);
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
}

#endif /* CONFIG_LCD_SSD1289 */
