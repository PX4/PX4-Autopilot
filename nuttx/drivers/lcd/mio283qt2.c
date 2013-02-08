/**************************************************************************************
 * drivers/lcd/mio283qt2.c
 *
 * This is a driver for the MI0283QT-2 LCD from Multi-Inno Technology Co., Ltd.  This
 * LCD is based on the Himax HX8347-D LCD controller.
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 * 1) LCD Module Specification, Model : MI0283QT-2, Multi-Inno Technology Co.,
 *    Ltd., Revision 1.0
 * 2) Data Sheet: HX8347-D(T), 240RGB x 320 dot, 262K color, with internal GRAM, TFT
 *    Mobile Single Chip Driver Version 02 March, Doc No. HX8347-D(T)-DS, Himax
 *    Technologies, Inc., 2009, 
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
#include <nuttx/lcd/mio283qt2.h>

#ifdef CONFIG_LCD_MIO283QT2

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
#  define MIO283QT2_XRES       320
#  define MIO283QT2_YRES       240
#else
#  define MIO283QT2_XRES       240
#  define MIO283QT2_YRES       320
#endif

/* Color depth and format */

#define MIO283QT2_BPP           16
#define MIO283QT2_COLORFMT      FB_FMT_RGB16_565

/* Hardware LCD/LCD controller definitions ********************************************/
/* In this driver, I chose to use all literal constants for register address and
 * values.  Some recent experiences have shown me that during LCD bringup, it is more
 * important to know the binary values rather than nice, people friendly names.  Sad,
 * but true.
 */

#define HIMAX_ID 0x0047

/* LCD Profiles ***********************************************************************/
/* Many details of the controller initialization must, unfortunately, vary from LCD to
 * LCD.  I have looked at the spec and at three different drivers for LCDs that have 
 * MIO283QT2 controllers.  I have tried to summarize these differences as "LCD profiles"
 *
 * Most of the differences between LCDs are nothing more than a few minor bit
 * settings.  The most significant difference betwen LCD drivers in is the
 * manner in which the LCD is powered up and in how the power controls are set.
 * My suggestion is that if you have working LCD initialization code, you should
 * simply replace the code in mio283qt2_hwinitialize with your working code.
 */

#if defined (CONFIG_MIO283QT2_PROFILE2)
#  undef MIO283QT2_USE_SIMPLE_INIT

  /* PWRCTRL1:  AP=smalll-to-medium, DC=Flinex24, BT=+5/-4, DCT=Flinex24 */

#  define PWRCTRL1_SETTING \
     (MIO283QT2_PWRCTRL1_AP_SMMED | MIO283QT2_PWRCTRL1_DC_FLINEx24 | \
      MIO283QT2_PWRCTRL1_BT_p5m4  | MIO283QT2_PWRCTRL1_DCT_FLINEx24)

  /* PWRCTRL2: 5.1v */

#  define PWRCTRL2_SETTING MIO283QT2_PWRCTRL2_VRC_5p1V

  /* PWRCTRL3: x 2.165
   * NOTE: Many drivers have bit 8 set which is not defined in the MIO283QT2 spec.
   */

#  define PWRCTRL3_SETTING MIO283QT2_PWRCTRL3_VRH_x2p165

   /* PWRCTRL4: VDV=9 + VCOMG */

#  define PWRCTRL4_SETTING (MIO283QT2_PWRCTRL4_VDV(9) | MIO283QT2_PWRCTRL4_VCOMG)

   /* PWRCTRL5: VCM=56 + NOTP */

#  define PWRCTRL5_SETTING (MIO283QT2_PWRCTRL5_VCM(56) | MIO283QT2_PWRCTRL5_NOTP)

#elif defined (CONFIG_MIO283QT2_PROFILE3)
#  undef MIO283QT2_USE_SIMPLE_INIT

  /* PWRCTRL1:  AP=smalll-to-medium, DC=Flinex24, BT=+5/-4, DCT=Flinex24 */

#  define PWRCTRL1_SETTING \
     (MIO283QT2_PWRCTRL1_AP_SMMED | MIO283QT2_PWRCTRL1_DC_FLINEx24 | \
      MIO283QT2_PWRCTRL1_BT_p5m4  | MIO283QT2_PWRCTRL1_DCT_FLINEx24)

  /* PWRCTRL2: 5.1v */

#  define PWRCTRL2_SETTING MIO283QT2_PWRCTRL2_VRC_5p1V

  /* PWRCTRL3: x 2.165
   * NOTE: Many drivers have bit 8 set which is not defined in the MIO283QT2 spec.
   */

#  define PWRCTRL3_SETTING MIO283QT2_PWRCTRL3_VRH_x2p165

   /* PWRCTRL4: VDV=9 + VCOMG */

#  define PWRCTRL4_SETTING (MIO283QT2_PWRCTRL4_VDV(9) | MIO283QT2_PWRCTRL4_VCOMG)

   /* PWRCTRL5: VCM=56 + NOTP */

#  define PWRCTRL5_SETTING (MIO283QT2_PWRCTRL5_VCM(56) | MIO283QT2_PWRCTRL5_NOTP)

#else /* if defined (CONFIG_MIO283QT2_PROFILE1) */
#  undef MIO283QT2_USE_SIMPLE_INIT
#  define MIO283QT2_USE_SIMPLE_INIT 1

  /* PWRCTRL1:  AP=medium-to-large, DC=Fosc/4, BT=+5/-4, DCT=Fosc/4 */

#  define PWRCTRL1_SETTING \
     (MIO283QT2_PWRCTRL1_AP_MEDLG | MIO283QT2_PWRCTRL1_DC_FOSd4 | \
      MIO283QT2_PWRCTRL1_BT_p5m4  | MIO283QT2_PWRCTRL1_DCT_FOSd4)

  /* PWRCTRL2: 5.3v */

#  define PWRCTRL2_SETTING MIO283QT2_PWRCTRL2_VRC_5p3V

  /* PWRCTRL3: x 2.570
   * NOTE: Many drivers have bit 8 set which is not defined in the MIO283QT2 spec.
   */

#  define PWRCTRL3_SETTING MIO283QT2_PWRCTRL3_VRH_x2p570

   /* PWRCTRL4: VDV=12 + VCOMG */

#  define PWRCTRL4_SETTING (MIO283QT2_PWRCTRL4_VDV(12) | MIO283QT2_PWRCTRL4_VCOMG)

   /* PWRCTRL5: VCM=60 + NOTP */

#  define PWRCTRL5_SETTING (MIO283QT2_PWRCTRL5_VCM(60) | MIO283QT2_PWRCTRL5_NOTP)

#endif

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg              dbg
#  define lcdvdbg             vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct mio283qt2_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct mio283qt2_lcd_s *lcd;  /* The contained platform-specific, LCD interface */
  uint8_t power;                  /* Current power setting */

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

  uint16_t runbuffer[MIO283QT2_XRES];
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Low Level LCD access */

static void mio283qt2_putreg(FAR struct mio283qt2_lcd_s *lcd, uint8_t regaddr,
             uint16_t regval);
#ifndef CONFIG_LCD_NOGETRUN
static uint16_t mio283qt2_readreg(FAR struct mio283qt2_lcd_s *lcd, uint8_t regaddr);
#endif
static inline void mio283qt2_gramwrite(FAR struct mio283qt2_lcd_s *lcd,
             uint16_t rgbcolor);
#ifndef CONFIG_LCD_NOGETRUN
static inline void mio283qt2_readsetup(FAR struct mio283qt2_lcd_s *lcd,
             FAR uint16_t *accum);
static inline uint16_t mio283qt2_gramread(FAR struct mio283qt2_lcd_s *lcd,
             FAR uint16_t *accum);
#endif
static void mio283qt2_setarea(FAR struct mio283qt2_lcd_s *lcd,
                              uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

/* LCD Data Transfer Methods */

static int mio283qt2_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int mio283qt2_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int mio283qt2_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int mio283qt2_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int mio283qt2_getpower(FAR struct lcd_dev_s *dev);
static int mio283qt2_setpower(FAR struct lcd_dev_s *dev, int power);
static int mio283qt2_getcontrast(FAR struct lcd_dev_s *dev);
static int mio283qt2_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline int mio283qt2_hwinitialize(FAR struct mio283qt2_dev_s *priv);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This driver can support only a signal MIO283QT2 device.  This is due to an
 * unfortunate decision made whent he getrun and putrun methods were designed. The
 * following is the single MIO283QT2 driver state instance:
 */

static struct mio283qt2_dev_s g_lcddev;

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  mio283qt2_putreg(lcd, 
 *
 * Description:
 *   Write to an LCD register
 *
 **************************************************************************************/

static void mio283qt2_putreg(FAR struct mio283qt2_lcd_s *lcd,
                             uint8_t regaddr, uint16_t regval)
{
  /* Set the index register to the register address and write the register contents */

  lcd->index(lcd, regaddr);
  lcd->write(lcd, regval);
}

/**************************************************************************************
 * Name:  mio283qt2_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static uint16_t mio283qt2_readreg(FAR struct mio283qt2_lcd_s *lcd, uint8_t regaddr)
{
  /* Set the index register to the register address and read the register contents. */

  lcd->index(lcd, regaddr);
  return lcd->read(lcd);
}
#endif

/**************************************************************************************
 * Name:  mio283qt2_gramselect
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 **************************************************************************************/

static inline void mio283qt2_gramselect(FAR struct mio283qt2_lcd_s *lcd)
{
  lcd->index(lcd, 0x22);
}

/**************************************************************************************
 * Name:  mio283qt2_gramwrite
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 **************************************************************************************/

static inline void mio283qt2_gramwrite(FAR struct mio283qt2_lcd_s *lcd, uint16_t data)
{
  lcd->write(lcd, data);
}

/**************************************************************************************
 * Name:  mio283qt2_readsetup
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
static inline void mio283qt2_readsetup(FAR struct mio283qt2_lcd_s *lcd,
                                       FAR uint16_t *accum)
{
#if 0 /* Probably not necessary... untested */
  /* Read-ahead one pixel */

  *accum = lcd->read(lcd);
#endif
}
#endif

/**************************************************************************************
 * Name:  mio283qt2_gramread
 *
 * Description:
 *   Read one correctly aligned pixel from the GRAM memory.  Possibly shifting the
 *   data and possibly swapping red and green components.
 *
 *   - ILI932x: Unknown -- assuming colors are in the color order 
 *
 **************************************************************************************/

#ifndef CONFIG_LCD_NOGETRUN
static inline uint16_t mio283qt2_gramread(FAR struct mio283qt2_lcd_s *lcd,
                                          FAR uint16_t *accum)
{
  /* Read the value (GRAM register already selected) */

  return lcd->read(lcd);
}
#endif

/**************************************************************************************
 * Name:  mio283qt2_setarea
 *
 * Description:
 *   Set the cursor position.  In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 **************************************************************************************/

static void mio283qt2_setarea(FAR struct mio283qt2_lcd_s *lcd,
                              uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
   mio283qt2_putreg(lcd, 0x03, (x0 & 0x00ff)); /* set x0 */
   mio283qt2_putreg(lcd, 0x02, (x0 >> 8));     /* set x0 */
   mio283qt2_putreg(lcd, 0x05, (x1 & 0x00ff)); /* set x1 */
   mio283qt2_putreg(lcd, 0x04, (x1 >> 8));     /* set x1 */
   mio283qt2_putreg(lcd, 0x07, (y0 & 0x00ff)); /* set y0 */
   mio283qt2_putreg(lcd, 0x06, (y0 >> 8));     /* set y0 */
   mio283qt2_putreg(lcd, 0x09, (y1 & 0x00ff)); /* set y1 */
   mio283qt2_putreg(lcd, 0x08, (y1 >> 8));     /* set y1 */
}

/**************************************************************************************
 * Name:  mio283qt2_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 **************************************************************************************/

#if 0 /* Sometimes useful */
static void mio283qt2_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels)
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
 * Name:  mio283qt2_putrun
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

static int mio283qt2_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                            size_t npixels)
{
  FAR struct mio283qt2_dev_s *priv = &g_lcddev;
  FAR struct mio283qt2_lcd_s *lcd = priv->lcd;
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Select the LCD */

  lcd->select(lcd);

  /* Write the run to GRAM. */

  mio283qt2_setarea(lcd, col, row, col + npixels - 1, row);
  mio283qt2_gramselect(lcd);

  for (i = 0; i < npixels; i++)
    {
      mio283qt2_gramwrite(lcd, *src);
      src++;
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
  return OK;
}

/**************************************************************************************
 * Name:  mio283qt2_getrun
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

static int mio283qt2_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                            size_t npixels)
{
#ifndef CONFIG_LCD_NOGETRUN
  FAR struct mio283qt2_dev_s *priv = &g_lcddev;
  FAR struct mio283qt2_lcd_s *lcd = priv->lcd;
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
  uint16_t accum;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Select the LCD */

  lcd->select(lcd);

  /* Read the run from GRAM. */

  /* Select the LCD */

  lcd->select(lcd);

  /* Red the run fram GRAM. */

  mio283qt2_setarea(lcd, col, row, col + npixels - 1, row);
  mio283qt2_gramselect(lcd);

  /* Prime the pump for unaligned read data */

  mio283qt2_readsetup(lcd, &accum);

  for (i = 0; i < npixels; i++)
    {
      *dest++ = mio283qt2_gramread(lcd, &accum);
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
  return OK;
#else
  return -ENOSYS;
#endif
}

/**************************************************************************************
 * Name:  mio283qt2_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int mio283qt2_getvideoinfo(FAR struct lcd_dev_s *dev,
                                  FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: 1\n",
          MIO283QT2_COLORFMT, MIO283QT2_XRES, MIO283QT2_XRES);

  vinfo->fmt     = MIO283QT2_COLORFMT;  /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  vinfo->xres    = MIO283QT2_XRES;      /* Horizontal resolution in pixel columns */
  vinfo->yres    = MIO283QT2_YRES;      /* Vertical resolution in pixel rows */
  vinfo->nplanes = 1;                   /* Number of color planes supported */
  return OK;
}

/**************************************************************************************
 * Name:  mio283qt2_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int mio283qt2_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                                  FAR struct lcd_planeinfo_s *pinfo)
{
  FAR struct mio283qt2_dev_s *priv = (FAR struct mio283qt2_dev_s *)dev;

  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, MIO283QT2_BPP);

  pinfo->putrun = mio283qt2_putrun;          /* Put a run into LCD memory */
  pinfo->getrun = mio283qt2_getrun;          /* Get a run from LCD memory */
  pinfo->buffer = (uint8_t*)priv->runbuffer; /* Run scratch buffer */
  pinfo->bpp    = MIO283QT2_BPP;             /* Bits-per-pixel */
  return OK;
}

/**************************************************************************************
 * Name:  mio283qt2_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int mio283qt2_getpower(FAR struct lcd_dev_s *dev)
{
  lcdvdbg("power: %d\n", 0);
  return g_lcddev.power;
}

/**************************************************************************************
 * Name:  mio283qt2_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int mio283qt2_poweroff(FAR struct mio283qt2_lcd_s *lcd)
{
  /* Set the backlight off */

  lcd->backlight(lcd, 0);

  /* Turn the display off */

  mio283qt2_putreg(lcd, 0x28, 0x0000); /* GON=0, DTE=0, D=0 */

  /* Remember the power off state */

  g_lcddev.power = 0;
  return OK;
}

/**************************************************************************************
 * Name:  mio283qt2_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int mio283qt2_setpower(FAR struct lcd_dev_s *dev, int power)
{
  FAR struct mio283qt2_dev_s *priv = (FAR struct mio283qt2_dev_s *)dev;
  FAR struct mio283qt2_lcd_s *lcd  = priv->lcd;

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

      /* Display on */

      mio283qt2_putreg(lcd, 0x28, 0x0038); /* GON=1, DTE=1, D=2 */
      up_mdelay(40);
      mio283qt2_putreg(lcd, 0x28, 0x003c); /* GON=1, DTE=1, D=3 */

      g_lcddev.power = power;
    }
  else
    {
      /* Turn the display off */

      mio283qt2_poweroff(lcd);
    }

  return OK;
}

/**************************************************************************************
 * Name:  mio283qt2_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int mio283qt2_getcontrast(FAR struct lcd_dev_s *dev)
{
  lcdvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  mio283qt2_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int mio283qt2_setcontrast(FAR struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  mio283qt2_hwinitialize
 *
 * Description:
 *   Initialize the LCD hardware.
 *
 **************************************************************************************/

static inline int mio283qt2_hwinitialize(FAR struct mio283qt2_dev_s *priv)
{
  FAR struct mio283qt2_lcd_s *lcd  = priv->lcd;
#ifndef CONFIG_LCD_NOGETRUN
  uint16_t id;
#endif

  /* Select the LCD */

  lcd->select(lcd);

  /* Read the HIMAX ID registger (0x00) */

#ifndef CONFIG_LCD_NOGETRUN
  id = mio283qt2_readreg(lcd, 0x00);
  lcddbg("LCD ID: %04x\n", id);

  /* Check if the ID is for the MIO283QT2 */

  if (id == HIMAX_ID)
#endif
    {
      /* Driving ability */

      mio283qt2_putreg(lcd, 0xea, 0x0000);  /* PTBA[15:8] */
      mio283qt2_putreg(lcd, 0xeb, 0x0020);  /* PTBA[7:0] */
      mio283qt2_putreg(lcd, 0xec, 0x000c);  /* STBA[15:8] */
      mio283qt2_putreg(lcd, 0xed, 0x00c4);  /* STBA[7:0] */
      mio283qt2_putreg(lcd, 0xe8, 0x0040);  /* OPON[7:0] */
      mio283qt2_putreg(lcd, 0xe9, 0x0038);  /* OPON1[7:0] */
      mio283qt2_putreg(lcd, 0xf1, 0x0001);  /* OTPS1B */
      mio283qt2_putreg(lcd, 0xf2, 0x0010);  /* GEN */
      mio283qt2_putreg(lcd, 0x27, 0x00a3);

      /* Power voltage */

      mio283qt2_putreg(lcd, 0x1b, 0x001b);  /* VRH = 4.65 */
      mio283qt2_putreg(lcd, 0x1a, 0x0001);  /* BT */
      mio283qt2_putreg(lcd, 0x24, 0x002f);  /* VMH */
      mio283qt2_putreg(lcd, 0x25, 0x0057);  /* VML */

      /* Vcom offset */

      mio283qt2_putreg(lcd, 0x23, 0x008d);  /* For flicker adjust */

      /* Power on */

      mio283qt2_putreg(lcd, 0x18, 0x0036);
      mio283qt2_putreg(lcd, 0x19, 0x0001); /* Start oscillator */
      mio283qt2_putreg(lcd, 0x01, 0x0000); /* Wakeup */
      mio283qt2_putreg(lcd, 0x1f, 0x0088);
      up_mdelay(5);
      mio283qt2_putreg(lcd, 0x1f, 0x0080);
      up_mdelay(5);
      mio283qt2_putreg(lcd, 0x1f, 0x0090);
      up_mdelay(5);
      mio283qt2_putreg(lcd, 0x1f, 0x00d0);
      up_mdelay(5);

      /* Gamma 2.8 setting  */

      mio283qt2_putreg(lcd, 0x40, 0x0000);
      mio283qt2_putreg(lcd, 0x41, 0x0000);
      mio283qt2_putreg(lcd, 0x42, 0x0001);
      mio283qt2_putreg(lcd, 0x43, 0x0013);
      mio283qt2_putreg(lcd, 0x44, 0x0010);
      mio283qt2_putreg(lcd, 0x45, 0x0026);
      mio283qt2_putreg(lcd, 0x46, 0x0008);
      mio283qt2_putreg(lcd, 0x47, 0x0051);
      mio283qt2_putreg(lcd, 0x48, 0x0002);
      mio283qt2_putreg(lcd, 0x49, 0x0012);
      mio283qt2_putreg(lcd, 0x4a, 0x0018);
      mio283qt2_putreg(lcd, 0x4b, 0x0019);
      mio283qt2_putreg(lcd, 0x4c, 0x0014);

      mio283qt2_putreg(lcd, 0x50, 0x0019);
      mio283qt2_putreg(lcd, 0x51, 0x002f);
      mio283qt2_putreg(lcd, 0x52, 0x002c);
      mio283qt2_putreg(lcd, 0x53, 0x003e);
      mio283qt2_putreg(lcd, 0x54, 0x003f);
      mio283qt2_putreg(lcd, 0x55, 0x003f);
      mio283qt2_putreg(lcd, 0x56, 0x002e);
      mio283qt2_putreg(lcd, 0x57, 0x0077);
      mio283qt2_putreg(lcd, 0x58, 0x000b);
      mio283qt2_putreg(lcd, 0x59, 0x0006);
      mio283qt2_putreg(lcd, 0x5a, 0x0007);
      mio283qt2_putreg(lcd, 0x5b, 0x000d);
      mio283qt2_putreg(lcd, 0x5c, 0x001d);
      mio283qt2_putreg(lcd, 0x5d, 0x00cc);

      /* 4K Color Selection */

      mio283qt2_putreg(lcd, 0x17, 0x0003);
      mio283qt2_putreg(lcd, 0x17, 0x0005); /* 0x0005=65k, 0x0006=262k */

      /* Panel characteristics */

      mio283qt2_putreg(lcd, 0x36, 0x0000);

      /* Display Setting */

      mio283qt2_putreg(lcd, 0x01, 0x0000); /* IDMON=0, INVON=0, NORON=0, PTLON=0 */

#if defined(CONFIG_LCD_LANDSCAPE)
      mio283qt2_putreg(lcd, 0x16, 0x00a8); /* MY=1, MX=0, MV=1, ML=0, BGR=1 */
#elif defined(CONFIG_LCD_PORTRAIT)
      mio283qt2_putreg(lcd, 0x16, 0x0008); /* MY=0, MX=0, MV=0, ML=0, BGR=1 */
#elif defined(CONFIG_LCD_RLANDSCAPE)
      mio283qt2_putreg(lcd, 0x16, 0x0068); /* MY=0, MX=1, MV=1, ML=0, BGR=1 */
#elif defined(CONFIG_LCD_RPORTRAIT)
      mio283qt2_putreg(lcd, 0x16, 0x00c8); /* MY=1, MX=0, MV=1, ML=0, BGR=1 */
#endif

      /* Window setting */

      mio283qt2_setarea(lcd, 0, 0, (MIO283QT2_XRES-1), (MIO283QT2_YRES-1));
      return OK;
    }
#ifndef CONFIG_LCD_NOGETRUN
  else
    {
      lcddbg("Unsupported LCD type\n");
      return -ENODEV;
    }
#endif

  /* De-select the LCD */

  lcd->deselect(lcd);
}

 /*************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  mio283qt2_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

FAR struct lcd_dev_s *mio283qt2_lcdinitialize(FAR struct mio283qt2_lcd_s *lcd)
{
  int ret;

  lcdvdbg("Initializing\n");

  /* If we ccould support multiple MIO283QT2 devices, this is where we would allocate
   * a new driver data structure... but we can't.  Why not?  Because of a bad should
   * the form of the getrun() and putrun methods.
   */

  FAR struct mio283qt2_dev_s *priv = &g_lcddev;

  /* Initialize the driver data structure */

  priv->dev.getvideoinfo = mio283qt2_getvideoinfo;
  priv->dev.getplaneinfo = mio283qt2_getplaneinfo;
  priv->dev.getpower     = mio283qt2_getpower;
  priv->dev.setpower     = mio283qt2_setpower;
  priv->dev.getcontrast  = mio283qt2_getcontrast;
  priv->dev.setcontrast  = mio283qt2_setcontrast;
  priv->lcd              = lcd;

  /* Configure and enable LCD */

  ret = mio283qt2_hwinitialize(priv);
  if (ret == OK)
    {
      /* Clear the display (setting it to the color 0=black) */

      mio283qt2_clear(&priv->dev, 0);

      /* Turn the display off */

      mio283qt2_poweroff(lcd);
      return &g_lcddev.dev;
    }

  return NULL;
}

/**************************************************************************************
 * Name:  mio283qt2_clear
 *
 * Description:
 *   This is a non-standard LCD interface just for the stm3240g-EVAL board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 **************************************************************************************/

void mio283qt2_clear(FAR struct lcd_dev_s *dev, uint16_t color)
{
  FAR struct mio283qt2_dev_s *priv = (FAR struct mio283qt2_dev_s *)dev;
  FAR struct mio283qt2_lcd_s *lcd  = priv->lcd;
  uint32_t i;

  /* Select the LCD and set the drawring area */

  lcd->select(lcd);
  mio283qt2_setarea(lcd, 0, 0, (MIO283QT2_XRES-1), (MIO283QT2_YRES-1));

  /* Prepare to write GRAM data */

  mio283qt2_gramselect(lcd);

  /* Copy color into all of GRAM.  Orientation does not matter in this case. */

  for (i = 0; i < MIO283QT2_XRES * MIO283QT2_YRES; i++)
    {
      mio283qt2_gramwrite(lcd, color);
    }

  /* De-select the LCD */

  lcd->deselect(lcd);
}

#endif /* CONFIG_LCD_MIO283QT2 */
