/**************************************************************************************
 * drivers/lcd/nokia6100.c
 * Nokia 6100 LCD Display Driver
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References:
 *   "Nokia 6100 LCD Display Driver," Revision 1, James P. Lynch ("Nokia 6100 LCD
 *    Display Driver.pdf")
 *   "S1D15G0D08B000," Seiko Epson Corportation, 2002.
 *   "Data Sheet, PCF8833 STN RGB 132x132x3 driver," Phillips, 2003 Feb 14.
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
#include <nuttx/lcd/nokia6100.h>

#ifdef CONFIG_NOKIA6100_PCF8833
#  include "pcf8833.h"
#endif
#ifdef CONFIG_NOKIA6100_S1D15G10
#  include "s1d15g10.h"
#endif

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Configuration **********************************************************************/
/* Verify that all configuration requirements have been met */

/* Nokia 6100 Configuration Settings:
 *
 * CONFIG_NOKIA6100_SPIMODE - Controls the SPI mode
 * CONFIG_NOKIA6100_FREQUENCY - Define to use a different bus frequency
 * CONFIG_NOKIA6100_NINTERFACES - Specifies the number of physical Nokia 6100 devices that
 *   will be supported.
 * CONFIG_NOKIA6100_BPP - Device supports 8, 12, and 16 bits per pixel.
 * CONFIG_NOKIA6100_S1D15G10 - Selects the Epson S1D15G10 display controller
 * CONFIG_NOKIA6100_PCF8833 - Selects the Phillips PCF8833 display controller
 * CONFIG_NOKIA6100_BLINIT - Initial backlight setting
 *
 * The following may need to be tuned for your hardware:
 * CONFIG_NOKIA6100_INVERT - Display inversion, 0 or 1, Default: 1
 * CONFIG_NOKIA6100_MY - Display row direction, 0 or 1, Default: 0
 * CONFIG_NOKIA6100_MX - Display column direction, 0 or 1, Default: 1
 * CONFIG_NOKIA6100_V - Display address direction, 0 or 1, Default: 0
 * CONFIG_NOKIA6100_ML - Display scan direction, 0 or 1, Default: 0
 * CONFIG_NOKIA6100_RGBORD - Display RGB order, 0 or 1, Default: 0
 *
 * Required LCD driver settings:
 * CONFIG_LCD_NOKIA6100 - Enable Nokia 6100 support
 * CONFIG_LCD_MAXCONTRAST - must be 63 with the Epson controller and 127 with
 *   the Phillips controller.
 * CONFIG_LCD_MAXPOWER - Maximum value of backlight setting.  The backlight control is
 *   managed outside of the 6100 driver so this value has no meaning to the driver.
 */

/* Mode 0,0 should be use.  However, somtimes you need to tinker with these things. */

#ifndef CONFIG_NOKIA6100_SPIMODE
#  define CONFIG_NOKIA6100_SPIMODE SPIDEV_MODE0
#endif

/* Default frequency: 1Mhz */

#ifndef CONFIG_NOKIA6100_FREQUENCY
#  define CONFIG_NOKIA6100_FREQUENCY 1000000
#endif

/* CONFIG_NOKIA6100_NINTERFACES determines the number of physical interfaces
 * that will be supported.
 */

#ifndef CONFIG_NOKIA6100_NINTERFACES
#  define CONFIG_NOKIA6100_NINTERFACES 1
#endif

#if CONFIG_NOKIA6100_NINTERFACES != 1
#  error "This implementation supports only a single LCD device"
#endif

/* Only support for 8 and 12 BPP currently implemented */

#if !defined(CONFIG_NOKIA6100_BPP)
#  warning "Assuming 8BPP"
#  define CONFIG_NOKIA6100_BPP 8
#endif

#if CONFIG_NOKIA6100_BPP != 8 && CONFIG_NOKIA6100_BPP != 12
#  if CONFIG_NOKIA6100_BPP == 16
#    error "Support for 16BPP no yet implemented"
#  else
#    error "LCD supports only 8, 12, and 16BPP"
#  endif
#endif

#if CONFIG_NOKIA6100_BPP == 8
#  ifdef CONFIG_NX_DISABLE_8BPP
#    warning "8-bit pixel support needed"
#  endif
#elif CONFIG_NOKIA6100_BPP == 12
#  if defined(CONFIG_NX_DISABLE_12BPP) || !defined(CONFIG_NX_PACKEDMSFIRST)
#    warning "12-bit, big-endian pixel support needed"
#  endif
#elif CONFIG_NOKIA6100_BPP == 16
#  ifdef CONFIG_NX_DISABLE_16BPP
#    warning "16-bit pixel support needed"
#  endif
#endif

/* Exactly one LCD controller must be selected. "The Olimex boards have both display
 * controllers possible; if the LCD has a GE-12 sticker on it, it’s a Philips PCF8833.
 * If it has a GE-8 sticker, it’s an Epson controller."
 */

#if defined(CONFIG_NOKIA6100_S1D15G10) && defined(CONFIG_NOKIA6100_PCF8833)
#  error "Both CONFIG_NOKIA6100_S1D15G10 and CONFIG_NOKIA6100_PCF8833 are defined"
#endif

#if !defined(CONFIG_NOKIA6100_S1D15G10) && !defined(CONFIG_NOKIA6100_PCF8833)
#  error "One of CONFIG_NOKIA6100_S1D15G10 or CONFIG_NOKIA6100_PCF8833 must be defined"
#endif

/* Delay geometry defaults */

#ifndef CONFIG_NOKIA6100_INVERT
#  define CONFIG_NOKIA6100_INVERT 1
#endif

#ifndef CONFIG_NOKIA6100_MY
#  define CONFIG_NOKIA6100_MY 0
#endif

#ifndef CONFIG_NOKIA6100_MX
#  define CONFIG_NOKIA6100_MX 1
#endif

#ifndef CONFIG_NOKIA6100_V
#  define CONFIG_NOKIA6100_V 0
#endif

#ifndef CONFIG_NOKIA6100_ML
#  define CONFIG_NOKIA6100_ML 0
#endif

#ifndef CONFIG_NOKIA6100_RGBORD
#  define CONFIG_NOKIA6100_RGBORD 0
#endif

/* Check contrast selection */

#ifdef CONFIG_NOKIA6100_S1D15G10

#  if !defined(CONFIG_LCD_MAXCONTRAST)
#    define CONFIG_LCD_MAXCONTRAST 63
#  endif
#  if CONFIG_LCD_MAXCONTRAST != 63
#    error "CONFIG_LCD_MAXCONTRAST must be 63 with the Epson LCD controller"
#  endif
#  define NOKIA_DEFAULT_CONTRAST 32

#else /* CONFIG_NOKIA6100_PCF8833 */

#  if !defined(CONFIG_LCD_MAXCONTRAST)
#    define CONFIG_LCD_MAXCONTRAST 127
#  endif
#  if CONFIG_LCD_MAXCONTRAST != 127
#    error "CONFIG_LCD_MAXCONTRAST must be 127 with the Phillips LCD controller"
#  endif
#  define NOKIA_DEFAULT_CONTRAST 48

#endif

/* Check initial backlight setting */

#ifndef CONFIG_NOKIA6100_BLINIT
#  define CONFIG_NOKIA6100_BLINIT (NOKIA_DEFAULT_CONTRAST/3)
#endif

/* Word width must be 9 bits */

#if defined(CONFIG_NOKIA6100_WORDWIDTH) && CONFIG_NOKIA6100_WORDWIDTH != 9
#  error "CONFIG_NOKIA6100_WORDWIDTH must be 9"
#endif
#ifndef CONFIG_NOKIA6100_WORDWIDTH
#  define CONFIG_NOKIA6100_WORDWIDTH 9
#endif

/* If bit 9 is set, the byte is data; clear for commands */

#define NOKIA_LCD_DATA  (1 << 8)

/* Define CONFIG_LCD_REGDEBUG to enable register-level debug output.
 * (Verbose debug must also be enabled)
 */

#ifndef CONFIG_DEBUG
#  undef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_DEBUG_GRAPHICS
#endif

#ifndef CONFIG_DEBUG_VERBOSE
#  undef CONFIG_LCD_REGDEBUG
#endif

/* Controller independent definitions *************************************************/

#ifdef CONFIG_NOKIA6100_PCF8833
#  define LCD_NOP   PCF8833_NOP
#  define LCD_RAMWR PCF8833_RAMWR
#  define LCD_PASET PCF8833_PASET
#  define LCD_CASET PCF8833_CASET
#endif
#ifdef CONFIG_NOKIA6100_S1D15G10
#  define LCD_NOP   S1D15G10_NOP
#  define LCD_RAMWR S1D15G10_RAMWR
#  define LCD_PASET S1D15G10_PASET
#  define LCD_CASET S1D15G10_CASET
#endif

/* Color Properties *******************************************************************/

/* Display Resolution */

#define NOKIA_XRES           132
#define NOKIA_YRES           132

/* Color depth and format */

#if CONFIG_NOKIA6100_BPP == 8
#  define NOKIA_BPP          8
#  define NOKIA_COLORFMT     FB_FMT_RGB8_332
#  define NOKIA_STRIDE       NOKIA_XRES
#  define NOKIA_PIX2BYTES(p) (p)
#elif CONFIG_NOKIA6100_BPP == 12
#  define NOKIA_BPP          12
#  define NOKIA_COLORFMT     FB_FMT_RGB12_444
#  define NOKIA_STRIDE       ((3*NOKIA_XRES+1)/2)
#  define NOKIA_PIX2BYTES(p) ((3*(p)+1)/2)
#elif CONFIG_NOKIA6100_BPP == 16
#  define NOKIA_BPP          16
#  define NOKIA_COLORFMT     FB_FMT_RGB16_565
#  define NOKIA_STRIDE       (2*NOKIA_XRES)
#  define NOKIA_PIX2BYTES(p) (2*(p))
#endif

/* Handle any potential strange behavior at edges */

#if 0 /* REVISIT */
#define NOKIA_PGBIAS         2 /* May not be necessary */
#define NOKIA_COLBIAS        0
#define NOKIA_XBIAS          2 /* May not be necessary, if so need to subtract from XRES */
#define NOKIA_YBIAS          0
#else
#define NOKIA_PGBIAS         0
#define NOKIA_COLBIAS        0
#define NOKIA_XBIAS          0
#define NOKIA_YBIAS          0
#endif

#define NOKIA_ENDPAGE        131
#define NOKIA_ENDCOL         131

/* Debug ******************************************************************************/

#ifdef CONFIG_LCD_REGDEBUG
# define lcddbg(format, arg...)  llvdbg(format, ##arg)
#else
# define lcddbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* This structure describes the state of this driver */

struct nokia_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  FAR struct spi_dev_s *spi; /* Contained SPI driver instance */
  uint8_t contrast;          /* Current contrast setting */
  uint8_t power;             /* Current power (backlight) setting */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/

/* SPI support */

static inline void nokia_configspi(FAR struct spi_dev_s *spi);
#ifdef CONFIG_SPI_OWNBUS
static inline void nokia_select(FAR struct spi_dev_s *spi);
static inline void nokia_deselect(FAR struct spi_dev_s *spi);
#else
static void nokia_select(FAR struct spi_dev_s *spi);
static void nokia_deselect(FAR struct spi_dev_s *spi);
#endif
static void nokia_sndcmd(FAR struct spi_dev_s *spi, const uint8_t cmd);
static void nokia_cmdarray(FAR struct spi_dev_s *spi, int len, const uint8_t *cmddata);
static void nokia_clrram(FAR struct spi_dev_s *spi);

/* LCD Data Transfer Methods */

static int nokia_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                        size_t npixels);
static int nokia_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                        size_t npixels);

/* LCD Configuration */

static int nokia_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo);
static int nokia_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int nokia_getpower(struct lcd_dev_s *dev);
static int nokia_setpower(struct lcd_dev_s *dev, int power);
static int nokia_getcontrast(struct lcd_dev_s *dev);
static int nokia_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static int nokia_initialize(struct nokia_dev_s *priv);

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

#if CONFIG_NOKIA6100_BPP == 8
static uint8_t g_runbuffer[NOKIA_XRES];
#elif CONFIG_NOKIA6100_BPP == 12
static uint8_t g_runbuffer[(3*NOKIA_XRES+1)/2];
#else /* CONFIG_NOKIA6100_BPP == 16 */
static uint16_t g_runbuffer[NOKIA_XRES];
#endif

/* g_rowbuf is another buffer, but used internally by the Nokia 6100 driver in order
 * expand the pixel data into 9-bit data needed by the LCD.  There are some
 * customizations that would eliminate the need for this extra buffer and for the
 * extra expansion/copy, but those customizations would require a special, non-standard
 * SPI driver that could expand 8- to 9-bit data on the fly.
 */

static uint16_t g_rowbuf[NOKIA_STRIDE+1];

/* Device Driver Data Structures ******************************************************/

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = NOKIA_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = NOKIA_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = NOKIA_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                 /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = nokia_putrun,           /* Put a run into LCD memory */
  .getrun = nokia_getrun,           /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer,  /* Run scratch buffer */
  .bpp    = NOKIA_BPP,              /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct nokia_dev_s g_lcddev = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = nokia_getvideoinfo,
    .getplaneinfo = nokia_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = nokia_getpower,
    .setpower     = nokia_setpower,
    .getcontrast  = nokia_getcontrast,
    .setcontrast  = nokia_setcontrast,
  },
};

/* LCD Command Strings ****************************************************************/

#ifdef CONFIG_NOKIA6100_S1D15G10
/* Display control:
 * P1: Specifies the CL dividing ratio, F1 and F2 drive-pattern switching period.
 * P2: Specifies the duty of the module on block basis
 * P3: Specify number of lines to be inversely highlighted on LCD panel
 * P4: 0: Dispersion P40= 1: Non-dispersion
 */

#if 1 // CONFIG_NOKIA6100_BPP == 12
static const uint8_t g_disctl[] =
{
  S1D15G10_DISCTL,                  /* Display control */
  DISCTL_CLDIV_2|DISCTL_PERIOD_8,   /* P1: Divide clock by 2; switching period = 8 */
//DISCTL_CLDIV_NONE|DISCTL_PERIOD_8, /* P1: No clock division; switching period = 8 */
  32,                               /* P2: nlines/4 - 1 = 132/4 - 1 = 32 */
  0,                                /* P3: No inversely highlighted lines */
  0                                 /* P4: No disperion */
};
#else /* CONFIG_NOKIA6100_BPP == 8 */
static const uint8_t g_disctl[] =
{
  S1D15G10_DISCTL,                  /* Display control */
  DISCTL_CLDIV_2|DISCTL_PERIOD_FLD, /* P1: Divide clock by 2; switching period = field */
  32,                               /* P2: nlines/4 - 1 = 132/4 - 1 = 32 */
  0,                                /* P3: No inversely highlighted lines */
  0                                 /* P4: No disperion */
};
#endif

/* Common scan direction:
 * P1: Cpecify the common output scan direction.
 */

static const uint8_t g_comscn[] =
{
  S1D15G10_COMSCN,                  /* Common scan direction */
  1                                 /* 0x01 = Scan 1->68, 132<-69 */
};

/* Power control:
 * P1: Turn on or off the liquid crystal driving power circuit, booster/step-down
 *    circuits and voltage follower circuit.
 */

static const uint8_t g_pwrctr[] =
{
  S1D15G10_PWRCTR,                  /* Power control */
  PWCTR_REFVOLTAGE|PWCTR_REGULATOR|PWCTR_BOOSTER2|PWCTR_BOOSTER1
};

/* Data control:
 * P1: Specify the normal or inverse display of the page address and also to specify
 *     the page address scanning direction
 * P2: RGB sequence
 * P3: Grayscale setup
 */

static const uint8_t g_datctl[] = 
{
  S1D15G10_DATCTL,                  /* Data control */
  0
#if CONFIG_NOKIA6100_MY != 0        /* Display row direction */
  |DATCTL_PGADDR_INV                /* Page address inverted */
#endif
#if CONFIG_NOKIA6100_MX != 0        /* Display column direction */
  |DATCTL_COLADDR_REV               /* Column address reversed */
#endif
#if CONFIG_NOKIA6100_V != 0         /* Display address direction */
  |DATCTL_ADDR_PGDIR                /* Address scan in page direction */
#endif
  ,
#if CONFIG_NOKIA6100_RGBORD != 0
  DATCTL_BGR,                       /* RGB->BGR */
#else
  0,                                /* RGB->RGB */
#endif
#if CONFIG_NOKIA6100_BPP == 8
  DATCTL_8GRAY                      /* Selects 8-bit color */
#elif CONFIG_NOKIA6100_BPP == 12
  DATCTL_16GRAY_A                   /* Selects 16-bit color, Type A */
#else
#  error "16-bit mode not yet implemented"
#endif
};

/* Voltage control (contrast setting):
 * P1: Volume value
 * P2: Resistance ratio
 * (May need to be tuned for individual displays)
 */

static const uint8_t g_volctr[] =
{
  S1D15G10_VOLCTR,                  /* Volume control */
  NOKIA_DEFAULT_CONTRAST,           /* Volume value */
  2                                 /* Resistance ratio */
};

/* 256-color position set (RGBSET8) */

#if CONFIG_NOKIA6100_BPP == 8
static const uint8_t g_rgbset8[] =
{
  S1D15G10_RGBSET8,                 /* 256-color position set */
  0, 2, 4, 6, 9, 11, 13, 15,        /* Red tones */
  0, 2, 4, 6, 9, 11, 13, 15,        /* Green tones */
  0, 5, 10, 15                      /* Blue tones */
};
#endif

/* Page address set (PASET) */

static const uint8_t g_paset[] =
{
  S1D15G10_PASET,                   /* Page start address set */
  NOKIA_PGBIAS,
  131
};

/* Column address set (CASET) */
  
static const uint8_t g_caset[] =
{
  S1D15G10_CASET,                   /* Column start address set */
  NOKIA_COLBIAS,          
  131
};
#endif /* CONFIG_NOKIA6100_S1D15G10 */

#ifdef CONFIG_NOKIA6100_PCF8833

/* Color interface pixel format (COLMOD) */

#if CONFIG_NOKIA6100_BPP == 12
static const uint8_t g_colmod[] =
{
  PCF8833_COLMOD,                   /* Color interface pixel format */
  PCF8833_FMT_12BPS                 /* 12 bits-per-pixel */
};
#else /* CONFIG_NOKIA6100_BPP == 8 */
static const uint8_t g_colmod[] =
{
  PCF8833_COLMOD,                   /* Color interface pixel format */
  PCF8833_FMT_8BPS                  /* 8 bits-per-pixel */
};
#endif

/* Memory data access control(MADCTL) */

static const uint8_t g_madctl[] =
{
  PCF8833_MADCTL,                   /* Memory data access control*/
  0
#ifdef CONFIG_NOKIA6100_RGBORD != 0
  |MADCTL_RGB                      /* RGB->BGR */
#endif
#ifdef CONFIG_NOKIA6100_MY != 0    /* Display row direction */
  |MADCTL_MY                       /* Mirror Y */
#endif
#ifdef CONFIG_NOKIA6100_MX != 0    /* Display column direction */
  |MADCTL_MX                       /* Mirror X */
#endif
#ifdef CONFIG_NOKIA6100_V != 0     /* Display address direction */
  |MADCTL_V                        /* ertical RAM write; in Y direction */
#endif
#ifdef CONFIG_NOKIA6100_ML != 0    /* Display scan direction */
  |MADCTL_LAO                      /* Line address order bottom to top */
#endif
};

/* Set contrast (SETCON) */

static const uint8_t g_setcon[] =
{
  PCF8833_SETCON,                   /* Set contrast */
  NOKIA_DEFAULT_CONTRAST
};

#endif /* CONFIG_NOKIA6100_PCF8833 */

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Function: nokia_configspi
 *
 * Description:
 *   Configure the SPI for use with the Nokia 6100
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
 
static inline void nokia_configspi(FAR struct spi_dev_s *spi)
{
  lcddbg("Mode: %d Bits: %d Frequency: %d\n",
         CONFIG_NOKIA6100_SPIMODE, CONFIG_NOKIA6100_WORDWIDTH, CONFIG_NOKIA6100_FREQUENCY);

  /* Configure SPI for the Nokia 6100.  But only if we own the SPI bus.  Otherwise, don't
   * bother because it might change.
   */

#ifdef CONFIG_SPI_OWNBUS
  SPI_SETMODE(spi, CONFIG_NOKIA6100_SPIMODE);
  SPI_SETBITS(spi, CONFIG_NOKIA6100_WORDWIDTH);
  SPI_SETFREQUENCY(spi, CONFIG_NOKIA6100_FREQUENCY)
#endif
}

/**************************************************************************************
 * Function: nokia_select
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
static inline void nokia_select(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just select the chip */

  lcddbg("SELECTED\n");
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);
}
#else
static void nokia_select(FAR struct spi_dev_s *spi)
{
  /* Select Nokia 6100 chip (locking the SPI bus in case there are multiple
   * devices competing for the SPI bus
   */

  lcddbg("SELECTED\n");
  SPI_LOCK(spi, true);
  SPI_SELECT(spi, SPIDEV_DISPLAY, true);

  /* Now make sure that the SPI bus is configured for the Nokia 6100 (it
   * might have gotten configured for a different device while unlocked)
   */

  SPI_SETMODE(spi, CONFIG_NOKIA6100_SPIMODE);
  SPI_SETBITS(spi, CONFIG_NOKIA6100_WORDWIDTH);
  SPI_SETFREQUENCY(spi, CONFIG_NOKIA6100_FREQUENCY);
}
#endif

/**************************************************************************************
 * Function: nokia_deselect
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
static inline void nokia_deselect(FAR struct spi_dev_s *spi)
{
  /* We own the SPI bus, so just de-select the chip */

  lcddbg("DE-SELECTED\n");
  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
}
#else
static void nokia_deselect(FAR struct spi_dev_s *spi)
{
  /* De-select Nokia 6100 chip and relinquish the SPI bus. */

  lcddbg("DE-SELECTED\n");
  SPI_SELECT(spi, SPIDEV_DISPLAY, false);
  SPI_LOCK(spi, false);
}
#endif

/**************************************************************************************
 * Name:  nokia_sndcmd
 *
 * Description:
 *   Send a 1-byte command.
 *
 **************************************************************************************/

static void nokia_sndcmd(FAR struct spi_dev_s *spi, const uint8_t cmd)
{
  /* Select the LCD */

  lcddbg("cmd: %02x\n", cmd);
  nokia_select(spi);

  /* Send the command. Bit 8 == 0 denotes a command */

  (void)SPI_SEND(spi, (uint16_t)cmd);

  /* De-select the LCD */

  nokia_deselect(spi);
}

/**************************************************************************************
 * Name:  nokia_cmddata
 *
 * Description:
 *   Send a 1-byte command followed by datlen data bytes.
 *
 **************************************************************************************/

static void nokia_cmddata(FAR struct spi_dev_s *spi, uint8_t cmd, int datlen,
                          const uint8_t *data)
{
  uint16_t *rowbuf = g_rowbuf;
  int i;

  lcddbg("cmd: %02x datlen: %d\n", cmd, datlen);
  DEBUGASSERT(datlen <= NOKIA_STRIDE);

  /* Copy the command into the line buffer. Bit 8 == 0 denotes a command. */

  *rowbuf++ = cmd;

  /* Copy any data after the command into the line buffer */

  for (i = 0; i < datlen; i++)
    {
      /* Bit 8 == 1 denotes data */

      *rowbuf++ = (uint16_t)*data++ | NOKIA_LCD_DATA;
    }

  /* Select the LCD */

  nokia_select(spi);

  /* Send the line buffer.  */

  (void)SPI_SNDBLOCK(spi, g_rowbuf, datlen+1);

  /* De-select the LCD */

  nokia_deselect(spi);
}

/**************************************************************************************
 * Name:  nokia_ramwr
 *
 * Description:
 *   Send a RAMWR command followed by datlen data bytes.
 *
 **************************************************************************************/

static void nokia_ramwr(FAR struct spi_dev_s *spi, int datlen, const uint8_t *data)
{
  nokia_cmddata(spi, LCD_RAMWR, datlen, data);
}

/**************************************************************************************
 * Name:  nokia_cmdarray
 *
 * Description:
 *   Send a RAMWR command followed by len-1 data bytes.
 *
 **************************************************************************************/

static void nokia_cmdarray(FAR struct spi_dev_s *spi, int len, const uint8_t *cmddata)
{
#ifdef CONFIG_LCD_REGDEBUG
  int i;

  for (i = 0; i < len; i++)
    {
      lcddbg("cmddata[%d]: %02x\n", i, cmddata[i]);
    }
#endif
  nokia_cmddata(spi, cmddata[0], len-1, &cmddata[1]);
}

/**************************************************************************************
 * Name:  nokia_clrram
 *
 * Description:
 *   Send a 1-byte command followed by len-1 data bytes.
 *
 **************************************************************************************/

static void nokia_clrram(FAR struct spi_dev_s *spi)
{
  uint16_t *rowbuf = g_rowbuf;
  int i;

  /* Set all zero data in the line buffer */

  for (i = 0; i < NOKIA_STRIDE; i++)
    {
      /* Bit 8 == 1 denotes data */

      *rowbuf++ = NOKIA_LCD_DATA;
    }
  
  /* Select the LCD and send the RAMWR command */

  nokia_select(spi);
  SPI_SEND(spi, LCD_RAMWR);

  /* Send the line buffer, once for each row.  */

  for (i = 0; i < NOKIA_YRES; i++)
    {
      (void)SPI_SNDBLOCK(spi, g_rowbuf, NOKIA_STRIDE);
    }

  /* De-select the LCD */

  nokia_deselect(spi);
}

/**************************************************************************************
 * Name:  nokia_putrun
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

static int nokia_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                        size_t npixels)
{
  struct nokia_dev_s *priv = &g_lcddev;
  FAR struct spi_dev_s *spi = priv->spi;
  uint16_t cmd[3];

  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);

#if NOKIA_XBIAS > 0
  col += NOKIA_XBIAS;
#endif
#if NOKIA_YBIAS > 0
  row += NOKIA_YBIAS;
#endif
  DEBUGASSERT(buffer && col >=0 && (col + npixels) <= NOKIA_XRES && row >= 0 && row < NOKIA_YRES);

  /* Set up to write the run. */

  nokia_select(spi);
  cmd[0] = LCD_PASET;
  cmd[1] = col | NOKIA_LCD_DATA;
  cmd[2] = NOKIA_ENDPAGE | NOKIA_LCD_DATA;
  (void)SPI_SNDBLOCK(spi, cmd, 3);
  nokia_deselect(spi);

  /* De-select the LCD */

  nokia_select(spi);
  cmd[0] = LCD_CASET;
  cmd[1] = row | NOKIA_LCD_DATA;
  cmd[2] = NOKIA_ENDCOL  | NOKIA_LCD_DATA;
  (void)SPI_SNDBLOCK(spi, cmd, 3); 
  nokia_deselect(spi);

  /* Then send the run */

  nokia_ramwr(spi, NOKIA_PIX2BYTES(npixels), buffer);
  return OK;
}

/**************************************************************************************
 * Name:  nokia_getrun
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

static int nokia_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                        size_t npixels)
{
  gvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* At present, this is a write-only LCD driver */

#warning "Not implemented"
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  nokia_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int nokia_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  nokia_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int nokia_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  nokia_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on. On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int nokia_getpower(struct lcd_dev_s *dev)
{
  struct nokia_dev_s *priv = (struct nokia_dev_s *)dev;
  gvdbg("power: %d\n", priv->power);
  return priv->power;
}

/**************************************************************************************
 * Name:  nokia_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int nokia_setpower(struct lcd_dev_s *dev, int power)
{
  struct nokia_dev_s *priv = (struct nokia_dev_s *)dev;
  int ret;

  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level.  The backlight power is controlled outside of the LCD
   * assembly and must be managmed by board-specific logic.
   */

  ret = nokia_backlight(power);
  if (ret == OK)
    {
      priv->power = power;
    }
  return ret;
}

/**************************************************************************************
 * Name:  nokia_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int nokia_getcontrast(struct lcd_dev_s *dev)
{
  struct nokia_dev_s *priv = (struct nokia_dev_s *)dev;
  gvdbg("contrast: %d\n", priv->contrast);
  return priv->contrast;
}

/**************************************************************************************
 * Name:  nokia_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int nokia_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  struct nokia_dev_s *priv = (struct nokia_dev_s *)dev;

  if (contrast < CONFIG_LCD_MAXCONTRAST)
    {
#ifdef CONFIG_NOKIA6100_S1D15G10
       while (priv->contrast < contrast)
         {
           nokia_sndcmd(priv->spi, S1D15G10_VOLUP);
           priv->contrast++;
         }
       while (priv->contrast > contrast)
         {
           nokia_sndcmd(priv->spi, S1D15G10_VOLDOWN);
           priv->contrast--;
         }
#else /* CONFIG_NOKIA6100_PCF8833 */
       uint8_t cmd[2];

       cmd[0] = PCF8833_SETCON;
       cmd[1] = priv->contrast;
       nokia_sndarry(priv->spi, 2, cmd);
       priv->contrast = contrast;
#endif
    }

  gvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  nokia_initialize
 *
 * Description:
 *   Initialize the LCD controller.
 *
 **************************************************************************************/

#ifdef CONFIG_NOKIA6100_S1D15G10
static int nokia_initialize(struct nokia_dev_s *priv)
{
  struct spi_dev_s *spi = priv->spi;

  /* Configure the display */

  nokia_cmdarray(spi, sizeof(g_disctl), g_disctl);   /* Display control */
  nokia_cmdarray(spi, sizeof(g_comscn), g_comscn);   /* Common scan direction */
  nokia_sndcmd(spi, S1D15G10_OSCON);                 /* Internal oscilator ON */
  nokia_sndcmd(spi, S1D15G10_SLPOUT);                /* Sleep out */
  nokia_cmdarray(spi, sizeof(g_volctr), g_volctr);   /* Volume control (contrast) */
  nokia_cmdarray(spi, sizeof(g_pwrctr), g_pwrctr);   /* Turn on voltage regulators */
  up_mdelay(100);
#ifdef CONFIG_NOKIA6100_INVERT
  nokia_sndcmd(spi, S1D15G10_DISINV);                /* Invert display */
#else
  nokia_sndcmd(spi, S1D15G10_DISNOR);                /* Normal display */
#endif
  nokia_cmdarray(spi, sizeof(g_datctl), g_datctl);   /* Data control */
#if CONFIG_NOKIA6100_BPP == 8
  nokia_cmdarray(spi, sizeof(g_rgbset8), g_rgbset8); /* Set up color lookup table */
  nokia_sndcmd(spi, S1D15G10_NOP);
#endif
  nokia_cmdarray(spi, sizeof(g_paset), g_paset);     /* Page address set */
  nokia_cmdarray(spi, sizeof(g_paset), g_caset);     /* Column address set */
  nokia_clrram(spi);
  nokia_sndcmd(spi, S1D15G10_DISON);                 /* Display on */
  return OK;
}
#endif

#ifdef CONFIG_NOKIA6100_PCF8833
static int nokia_initialize(struct nokia_dev_s *priv)
{
  struct struct spi_dev_s *spi = priv->spi;

  nokia_sndcmd(spi, PCF8833_SLEEPOUT);              /* Exit sleep mode */
  nokia_sndcmd(spi, PCF8833_BSTRON);                /* Turn on voltage booster */
#ifdef CONFIG_NOKIA6100_INVERT
  nokia_sndcmd(spi, PCF8833_INVON);                 /* Invert display */
#else
  nokia_sndcmd(spi, PCF8833_INVOFF);                /* Don't invert display */
#endif
  nokia_cmdarray(spi, sizeof(g_madctl), g_madctl);  /* Memory data access control */
  nokia_cmdarray(spi, sizeof(g_colmod), g_colmod);  /* Color interface pixel format */
  nokia_cmdarray(spi, sizeof(g_setcon), g_setcon);  /* Set contrast */
  nokia_sndcmd(spi, PCF8833_NOP);                   /* No operation */
  nokia_clrram(spi);
  nokia_sndcmd(spi, PCF8833_DISPON);                /* Display on */
  return OK;
}
#endif /* CONFIG_NOKIA6100_PCF8833 */

/**************************************************************************************
 * Public Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  nokia_lcdinitialize
 *
 * Description:
 *   Initialize the NOKIA6100 video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off == sleep mode).
 *
 * Input Parameters:
 *
 *   spi - A reference to the SPI driver instance.
 *   devno - A value in the range of 0 throuh CONFIG_NOKIA6100_NINTERFACES-1.  This
 *     allows support for multiple LCD devices.
 *
 * Returned Value:
 *
 *   On success, this function returns a reference to the LCD object for the specified
 *   LCD.  NULL is returned on any failure.
 *
 **************************************************************************************/

FAR struct lcd_dev_s *nokia_lcdinitialize(FAR struct spi_dev_s *spi, unsigned int devno)
{
  struct nokia_dev_s *priv = &g_lcddev;

  gvdbg("Initializing\n");
  DEBUGASSERT(devno == 0);

  /* Initialize the driver data structure */
  
  priv->spi      = spi;                     /* Save the SPI instance */
  priv->contrast = NOKIA_DEFAULT_CONTRAST;  /* Initial contrast setting */

  /* Configure and enable the LCD controller */

  nokia_configspi(spi);
  if (nokia_initialize(priv) == OK)
    {
      /* Turn on the backlight */

      nokia_backlight(CONFIG_NOKIA6100_BLINIT);
      return &priv->dev;
    }
  return NULL;
}
