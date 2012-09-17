/************************************************************************************
 * nuttx/configs/compal_e99/src/ssd1783.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Laurent Latil <laurent@latil.nom.fr>
 *           Denis 'GNUtoo' Carikli <GNUtoo@no-log.org>
 *           Alan Carvalho de Assis <acassis@gmail.com>
 *
 * This driver for SSD1783 used part of SSD1783 driver developed by
 * Christian Vogel <vogelchr@vogel.cx> for Osmocom-BB and relicensed
 * to BSD with permission from author.
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
 ************************************************************************************/

#include <nuttx/config.h>
#include <nuttx/fb.h>

#include <sys/types.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include <arch/calypso/uwire.h>
#include <arch/calypso/clock.h>

#include "up_arch.h"
#include "ssd1783.h"

/* Color depth and format */
#define LCD_BPP          16
#define LCD_COLORFMT     FB_FMT_RGB16_555

/* Display Resolution */
#  define LCD_XRES       98
#  define LCD_YRES       67

/* Debug ******************************************************************************/
#ifdef CONFIG_DEBUG_LCD
# define lcddbg(format, arg...)  vdbg(format, ##arg)
#else
# define lcddbg(x...)
#endif

/** This should be put elsewhere */
#ifdef __CC_ARM               /* ARM Compiler        */
#define lcd_inline            static __inline
#elif defined (__ICCARM__)    /* for IAR Compiler */
#define lcd_inline            inline
#elif defined (__GNUC__)      /* GNU GCC Compiler */
#define lcd_inline            static __inline
#else
#define lcd_inline            static
#endif

static void lcd_clear(void);
static void fb_ssd1783_send_cmdlist(const struct ssd1783_cmdlist *p);

/* LCD Data Transfer Methods */
int lcd_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
    size_t npixels);
int lcd_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
    size_t npixels);

/* LCD Configuration */
static int lcd_getvideoinfo(FAR struct lcd_dev_s *dev,
    FAR struct fb_videoinfo_s *vinfo);
static int lcd_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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
static int lcd_getpower(struct lcd_dev_s *dev);
static int lcd_setpower(struct lcd_dev_s *dev, int power);
static int lcd_getcontrast(struct lcd_dev_s *dev);
static int lcd_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization (LCD ctrl / backlight) */
static inline void lcd_initialize(void);

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

static uint16_t g_runbuffer[LCD_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{ .fmt = LCD_COLORFMT, /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres = LCD_XRES, /* Horizontal resolution in pixel columns */
  .yres = LCD_YRES, /* Vertical resolutiSend a command list to the LCD panelon in pixel rows */
  .nplanes = 1, /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{ .putrun = lcd_putrun, /* Put a run into LCD memory */
//  .getrun = lcd_getrun, /* Get a run from LCD memory */
  .buffer = (uint8_t*) g_runbuffer, /* Run scratch buffer */
  .bpp = LCD_BPP, /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct ssd1783_dev_s g_lcddev =
{ .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = lcd_getvideoinfo,
    .getplaneinfo = lcd_getplaneinfo,

/* LCD RGB Mapping -- Not supported */
/* Cursor Controls -- Not supported */

/* LCD Specific Controls */
    .getpower = lcd_getpower,
    .setpower = lcd_setpower,
//    .getcontrast = lcd_getcontrast,
//    .setcontrast = lcd_setcontrast,
  },
  .power=0
};

/* we trust gcc to move this expensive bitshifting out of
   the loops in the drawing funtcions */
static uint8_t rgb_to_pixel(uint16_t color)
{
  uint8_t ret;

  ret  = (FB_COLOR_TO_R(color) & 0xe0);      /* 765 = RRR */
  ret |= (FB_COLOR_TO_G(color) & 0xe0) >> 3; /* 432 = GGG */
  ret |= (FB_COLOR_TO_B(color) & 0xc0) >> 6; /*  10 =  BB */

  return ret;
}

/* somehow the palette is messed up, RRR seems to have the
   bits reversed!  R0 R1 R2 G G G B B ---> R2 R1 R0 G G G B B */
uint8_t fix_rrr(uint8_t v){
  return (v & 0x5f) | (v & 0x80) >> 2 | (v & 0x20) << 2;
}


lcd_inline void write_data(uint16_t datain)
{
  uint16_t dataout = 0x0100 | fix_rrr(rgb_to_pixel(datain));
  uwire_xfer(SSD1783_DEV_ID,SSD1783_UWIRE_BITLEN,&dataout, NULL);
}

static void fb_ssd1783_send_cmdlist(const struct ssd1783_cmdlist *p)
{
  int i=0;

  while(p->is_cmd != END)
    {
      uint16_t sendcmd = p->data;
      if(p->is_cmd == DATA)
        sendcmd |= 0x0100; /* 9th bit is cmd/data flag */
      uwire_xfer(SSD1783_DEV_ID, SSD1783_UWIRE_BITLEN, &sendcmd, NULL);
      p++;
      i++;
    }
}

static void lcd_write_prepare(unsigned int x1, unsigned int x2, unsigned int y1, unsigned int y2)
{;
  DEBUGASSERT( (x1 < x2 )&& (y1 < y2));
  struct ssd1783_cmdlist prepare_disp_write_cmds[] = {
    { CMD,  0x15 },                  /*  set column address */
    { DATA, x1 },
    { DATA, x2 },
    { CMD,  0x75 },                 /*  set page address (Y) */
    { DATA, y1 },
    { DATA, y2 },
    { CMD,  0x5c },                  /* enter write display ram mode */
    { END,  0x00 }
  };
  dbg("x1:%d, x2:%d, y1:%d, y2:%d\n",x1, x2,y1, y2);
  fb_ssd1783_send_cmdlist(prepare_disp_write_cmds);
}

/**************************************************************************************
 * Name:  lcd_putrun
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

int lcd_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
    size_t npixels)
{
  int i;
  FAR const uint16_t *src = (FAR const uint16_t*) buffer;

  /* Buffer must be provided and aligned to a 16-bit address boundary */
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);


  /* Write the run to GRAM. */
  lcd_write_prepare(col,col+npixels, row,row+1);

  for (i = 0; i < npixels; i++)
  {
    write_data(*src++);
  }
  fb_ssd1783_send_cmdlist(nop);

  return OK;
}

/**************************************************************************************
 * Name:  lcd_getrun
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

int lcd_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
    size_t npixels)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  lcd_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int lcd_getvideoinfo(FAR struct lcd_dev_s *dev,
    FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
      g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  lcd_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int lcd_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
    FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  lcd_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int lcd_getpower(struct lcd_dev_s *dev)
{
  gvdbg("power: %d\n", 0);
  return g_lcddev.power;
}

/**************************************************************************************
 * Name:  lcd_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on).
 *   Used here to set pwm duty on timer used for backlight.
 *
 **************************************************************************************/

static int lcd_setpower(struct lcd_dev_s *dev, int power)
{
  uint16_t reg;

  if (g_lcddev.power == power) {
    return OK;
  }

  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */
  reg = getreg16(ASIC_CONF_REG);
  if (power)
    {
      reg = getreg16(ASIC_CONF_REG);
      /* LCD Set I/O(3) / SA0 to I/O(3) mode */
      reg &= ~( (1 << 12) | (1 << 10) | (1 << 7) | (1 << 1)) ;
      /* don't set function pins to I2C Mode, C155 uses UWire */
      /* TWL3025: Set SPI+RIF RX clock to rising edge */
      reg |= (1 << 13) | (1 << 14);
      putreg16(reg, ASIC_CONF_REG);

      /* LCD Set I/O(3) to output mode and enable C155 backlight (IO1) */
      /* FIXME: Put the display backlight control to backlight.c */
      reg = getreg16(IO_CNTL_REG);
      reg &= ~( (1 << 3) | (1 << 1));
      putreg16(reg, IO_CNTL_REG);

      /* LCD Set I/O(3) output low */
      reg = getreg16(ARMIO_LATCH_OUT);
      reg &= ~(1 << 3);
      reg |= (1 << 1);
      putreg16(reg, ARMIO_LATCH_OUT);
    }
    else
    {
      gvdbg("powering LCD off...\n");
      /* Switch pin from PWL to LT */
      reg &= ~ASCONF_PWL_ENA;
      putreg8(reg, ASIC_CONF_REG);
      /* Disable pwl */
      putreg8(0x00, PWL_REG(PWL_CTRL));
    }
    return OK;
}


/**************************************************************************************
 * Name:  lcd_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int lcd_getcontrast(struct lcd_dev_s *dev)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  lcd_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int lcd_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  lcd_lcdinitialize
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/
static inline void lcd_initialize(void)
{
  gvdbg("%s: initializing LCD.\n",__FUNCTION__);
  calypso_reset_set(RESET_EXT, 0);
  usleep(5000);
  uwire_init();
  usleep(5000);
  fb_ssd1783_send_cmdlist(ssd1783_initdata);
}

/**************************************************************************************
 * Public Functions
 **************************************************************************************/


/**************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 **************************************************************************************/

int up_lcdinitialize(void)
{
  gvdbg("Initializing\n");

  lcd_initialize();

  /* Clear the display  */
  lcd_clear();

  return OK;
}

/**************************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows support
 *   for multiple LCD devices.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}


/**************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Un-initialize the LCD support
 *
 **************************************************************************************/

void up_lcduninitialize(void)
{
  lcd_setpower(&g_lcddev.dev, 0);
}

/**************************************************************************************
 * Name:  lcd_clear
 *
 * Description:
 *   Fill the LCD ctrl memory with given color
 *
 **************************************************************************************/

void lcd_clear()
{
  struct ssd1783_cmdlist prepare_disp_write_cmds[] = {
    { CMD,  0x8E },
    { DATA, 0x00 },
    { DATA, 0x00 },
    { DATA, LCD_XRES },
    { DATA, LCD_YRES },
    { END,  0x00 }
  };

  struct ssd1783_cmdlist nop[] = {
    { CMD, 0x25 }, // NOP command
    { END, 0x00 }
  };

  fb_ssd1783_send_cmdlist(prepare_disp_write_cmds);
  fb_ssd1783_send_cmdlist(nop);
}
