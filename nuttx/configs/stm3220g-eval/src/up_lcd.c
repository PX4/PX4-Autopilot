/**************************************************************************************
 * configs/stm3220g-eval/src/up_lcd.c
 * arch/arm/src/board/up_lcd.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Diego Sanchez <dsanchez@nx-engineering.com>
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
/* This driver supports the following LCDs on the STM324xG_EVAL board:
 *
 *   AM-240320L8TNQW00H (LCD_ILI9320 or LCD_ILI9321) OR 
 *   AM-240320D5TOQW01H (LCD_ILI9325)
 */
 
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

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_internal.h"
#include "stm3220g-internal.h"

#if !defined(CONFIG_STM32_ILI9320_DISABLE) || !defined(CONFIG_STM32_ILI9325_DISABLE)

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/
/* CONFIG_STM32_ILI9320_DISABLE may be defined to disabled the AM-240320L8TNQW00H
 *  (LCD_ILI9320 or LCD_ILI9321)
 * CONFIG_STM32_ILI9325_DISABLE  may be defined to disabled the AM-240320D5TOQW01H
 *  (LCD_ILI9325)
 */

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
#  define STM3220G_XRES       320
#  define STM3220G_YRES       240
#else
#  define STM3220G_XRES       240
#  define STM3220G_YRES       320
#endif

/* Color depth and format */

#define STM3220G_BPP          16
#define STM3220G_COLORFMT     FB_FMT_RGB16_565

/* STM3220G-EVAL LCD Hardware Definitions *********************************************/
/* LCD /CS is CE4,  Bank 3 of NOR/SRAM Bank 1~4 */

#define STM3220G_LCDBASE      ((uintptr_t)(0x60000000 | 0x08000000))
#define LCD                   ((struct lcd_regs_s *)STM3220G_LCDBASE)

#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0a
#define LCD_REG_12            0x0c
#define LCD_REG_13            0x0d
#define LCD_REG_14            0x0e
#define LCD_REG_15            0x0f
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1a
#define LCD_REG_27            0x1b
#define LCD_REG_28            0x1c
#define LCD_REG_29            0x1d
#define LCD_REG_30            0x1e
#define LCD_REG_31            0x1f
#define LCD_REG_32            0x20
#define LCD_REG_33            0x21
#define LCD_REG_34            0x22
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_43            0x2b
#define LCD_REG_45            0x2d
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_58            0x3a
#define LCD_REG_59            0x3b
#define LCD_REG_60            0x3c
#define LCD_REG_61            0x3d
#define LCD_REG_62            0x3e
#define LCD_REG_63            0x3f
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4a
#define LCD_REG_75            0x4b
#define LCD_REG_76            0x4c
#define LCD_REG_77            0x4d
#define LCD_REG_78            0x4e
#define LCD_REG_79            0x4f
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_106           0x6a
#define LCD_REG_118           0x76
#define LCD_REG_128           0x80
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_131           0x83
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_134           0x86
#define LCD_REG_135           0x87
#define LCD_REG_136           0x88
#define LCD_REG_137           0x89
#define LCD_REG_139           0x8b
#define LCD_REG_140           0x8c
#define LCD_REG_141           0x8d
#define LCD_REG_143           0x8f
#define LCD_REG_144           0x90
#define LCD_REG_145           0x91
#define LCD_REG_146           0x92
#define LCD_REG_147           0x93
#define LCD_REG_148           0x94
#define LCD_REG_149           0x95
#define LCD_REG_150           0x96
#define LCD_REG_151           0x97
#define LCD_REG_152           0x98
#define LCD_REG_153           0x99
#define LCD_REG_154           0x9a
#define LCD_REG_157           0x9d
#define LCD_REG_164           0xa4
#define LCD_REG_192           0xc0
#define LCD_REG_193           0xc1
#define LCD_REG_229           0xe5

/* LCD IDs */

#define ILI9320_ID            0x9320
#define ILI9321_ID            0x9321
#define ILI9325_ID            0x9325

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

/* LCD type */

enum lcd_type_e
{
  LCD_TYPE_UNKNOWN = 0,
  LCD_TYPE_ILI9320,
  LCD_TYPE_ILI9325
};

/* This structure describes the LCD registers */

struct lcd_regs_s
{
  volatile uint16_t address;
  volatile uint16_t value;
};

/* This structure describes the state of this driver */

struct stm3220g_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t  type;        /* LCD type. See enum lcd_type_e */
  uint8_t  power;       /* Current power setting */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Low Level LCD access */

static void stm3220g_writereg(uint8_t regaddr, uint16_t regval);
static uint16_t stm3220g_readreg(uint8_t regaddr);
static inline void stm3220g_gramselect(void);
static inline void stm3220g_writegram(uint16_t rgbval);
static void stm3220g_readnosetup(FAR uint16_t *accum);
static uint16_t stm3220g_readnoshift(FAR uint16_t *accum);
static void stm3220g_setcursor(uint16_t col, uint16_t row);

/* LCD Data Transfer Methods */

static int stm3220g_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int stm3220g_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int stm3220g_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int stm3220g_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int stm3220g_getpower(struct lcd_dev_s *dev);
static int stm3220g_setpower(struct lcd_dev_s *dev, int power);
static int stm3220g_getcontrast(struct lcd_dev_s *dev);
static int stm3220g_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* Initialization */

static inline void stm3220g_lcdinitialize(void);

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

static uint16_t g_runbuffer[STM3220G_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = STM3220G_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = STM3220G_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = STM3220G_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo = 
{
  .putrun = stm3220g_putrun,       /* Put a run into LCD memory */
  .getrun = stm3220g_getrun,       /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = STM3220G_BPP,          /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct stm3220g_dev_s g_lcddev = 
{
  .dev =
  {
    /* LCD Configuration */
 
    .getvideoinfo = stm3220g_getvideoinfo,
    .getplaneinfo = stm3220g_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = stm3220g_getpower,
    .setpower     = stm3220g_setpower,
    .getcontrast  = stm3220g_getcontrast,
    .setcontrast  = stm3220g_setcontrast,
  },
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  stm3220g_writereg
 *
 * Description:
 *   Write to an LCD register
 *
 **************************************************************************************/

static void stm3220g_writereg(uint8_t regaddr, uint16_t regval)
{
  /* Write the register address then write the register value */

  LCD->address = regaddr;
  LCD->value   = regval;
}

/**************************************************************************************
 * Name:  stm3220g_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 **************************************************************************************/

static uint16_t stm3220g_readreg(uint8_t regaddr)
{
  /* Write the register address then read the register value */

  LCD->address = regaddr;
  return LCD->value;
}

/**************************************************************************************
 * Name:  stm3220g_gramselect
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 **************************************************************************************/

static inline void stm3220g_gramselect(void)
{
  LCD->address = LCD_REG_34;
}

/**************************************************************************************
 * Name:  stm3220g_writegram
 *
 * Description:
 *   Write one pixel to the GRAM memory
 *
 **************************************************************************************/

static inline void stm3220g_writegram(uint16_t rgbval)
{
  /* Write the value (GRAM register already selected) */

  LCD->value = rgbval;
}

/**************************************************************************************
 * Name:  stm3220g_readnosetup
 *
 * Description:
 *   Prime the operation by reading one pixel from the GRAM memory if necessary for
 *   this LCD type.  When reading 16-bit gram data, there may be some shifts in the
 *   returned data:
 *
 *   - ILI932x: Discard first dummy read; no shift in the return data
 *
 **************************************************************************************/
 
static void stm3220g_readnosetup(FAR uint16_t *accum)
{
  /* Read-ahead one pixel */

  *accum  = LCD->value;
}

/**************************************************************************************
 * Name:  stm3220g_readnoshift
 *
 * Description:
 *   Read one correctly aligned pixel from the GRAM memory.  Possibly shifting the
 *   data and possibly swapping red and green components.
 *
 *   - ILI932x: Unknown -- assuming colors are in the color order 
 *
 **************************************************************************************/

static uint16_t stm3220g_readnoshift(FAR uint16_t *accum)
{
  /* Read the value (GRAM register already selected) */

  return LCD->value;
}

/**************************************************************************************
 * Name:  stm3220g_setcursor
 *
 * Description:
 *   Set the cursor position.  In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 **************************************************************************************/

static void stm3220g_setcursor(uint16_t col, uint16_t row)
{
  stm3220g_writereg(LCD_REG_32, row); /* GRAM horizontal address */
  stm3220g_writereg(LCD_REG_33, col); /* GRAM vertical address */
}

/**************************************************************************************
 * Name:  stm3220g_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 **************************************************************************************/

#if 0 /* Sometimes useful */
static void stm3220g_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels)
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
 * Name:  stm3220g_putrun
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

static int stm3220g_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Write the run to GRAM. */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates -- Here the edge away from the row of buttons on
   * the STM3220G-EVAL is used as the top.
   */

  /* Write the GRAM data, manually incrementing X */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3220g_setcursor(col, row);
      stm3220g_gramselect();
      stm3220g_writegram(*src++);

      /* Increment to next column */

      col++;
    }
#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Convert coordinates -- Here the edge next to the row of buttons on
   * the STM3220G-EVAL is used as the top.
   */

  col = (STM3220G_XRES-1) - col;
  row = (STM3220G_YRES-1) - row;

  /* Set the cursor position */

  stm3220g_setcursor(col, row);

  /* Then write the GRAM data, auto-decrementing X */

  stm3220g_gramselect();
  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position (auto-decrements to the next column) */

      stm3220g_writegram(*src++);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates.  In this configuration, the top of the display is to the left
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  col = (STM3220G_XRES-1) - col;

  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3220g_setcursor(row, col);
      stm3220g_gramselect();
      stm3220g_writegram(*src++);

      /* Increment to next column */

      col--;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates.  In this configuration, the top of the display is to the right
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  row = (STM3220G_YRES-1) - row;
  
  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3220g_setcursor(row, col);
      stm3220g_gramselect();
      stm3220g_writegram(*src++);

      /* Decrement to next column */

      col++;
    }
#endif
  return OK;
}

/**************************************************************************************
 * Name:  stm3220g_getrun
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

static int stm3220g_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
  void (*readsetup)(FAR uint16_t *accum);
  uint16_t (*readgram)(FAR uint16_t *accum);
  uint16_t accum;
  int i;
 
  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcdvdbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Configure according to the LCD type.  Kind of silly with only one LCD type. */

  switch (g_lcddev.type)
   {
     case LCD_TYPE_ILI9320:
     case LCD_TYPE_ILI9325:
       readsetup = stm3220g_readnosetup;
       readgram  = stm3220g_readnoshift;
       break;

     default:  /* Shouldn't happen */
       return -ENOSYS;
   }
 
  /* Read the run from GRAM. */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates -- Here the edge away from the row of buttons on
   * the STM3220G-EVAL is used as the top.
   */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      stm3220g_setcursor(row, col);
      stm3220g_gramselect();
      readsetup(&accum);
      *dest++ = readgram(&accum);

      /* Increment to next column */

      col++;
    }
#elif defined(CONFIG_LCD_RLANDSCAPE)
  /* Convert coordinates -- Here the edge next to the row of buttons on
   * the STM3220G-EVAL is used as the top.
   */

  col = (STM3220G_XRES-1) - col;
  row = (STM3220G_YRES-1) - row;

  /* Set the cursor position */

  stm3220g_setcursor(col, row);

  /* Then read the GRAM data, auto-decrementing Y */

  stm3220g_gramselect();

  /* Prime the pump for unaligned read data */

  readsetup(&accum);

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position (autoincrements to the next row) */

      *dest++ = readgram(&accum);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates.  In this configuration, the top of the display is to the left
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  col = (STM3220G_XRES-1) - col;

  /* Then read the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      stm3220g_setcursor(row, col);
      stm3220g_gramselect();
      readsetup(&accum);
      *dest++ = readgram(&accum);

      /* Increment to next column */

      col--;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates.  In this configuration, the top of the display is to the right
   * of the buttons (if the board is held so that the buttons are at the botton of the
   * board).
   */

  row = (STM3220G_YRES-1) - row;
  
  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3220g_setcursor(row, col);
      stm3220g_gramselect();
      readsetup(&accum);
      *dest++ = readgram(&accum);

      /* Decrement to next column */

      col++;
    }
#endif

  return OK;
}

/**************************************************************************************
 * Name:  stm3220g_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int stm3220g_getvideoinfo(FAR struct lcd_dev_s *dev,
                                 FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  lcdvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
          g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  stm3220g_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int stm3220g_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  lcdvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  stm3220g_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int stm3220g_getpower(struct lcd_dev_s *dev)
{
  lcdvdbg("power: %d\n", 0);
  return g_lcddev.power;
}

/**************************************************************************************
 * Name:  stm3220g_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int stm3220g_poweroff(void)
{
  /* Turn the display off */

  stm3220g_writereg(LCD_REG_7, 0); 

  /* Remember the power off state */

  g_lcddev.power = 0;
  return OK;
}

/**************************************************************************************
 * Name:  stm3220g_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int stm3220g_setpower(struct lcd_dev_s *dev, int power)
{
  lcdvdbg("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
      /* Then turn the display on */

#if !defined(CONFIG_STM32_ILI9320_DISABLE) || !defined(CONFIG_STM32_ILI9325_DISABLE)
      stm3220g_writereg(LCD_REG_7, 0x0173);
#endif
      g_lcddev.power = power;
    }
  else
    {
      /* Turn the display off */

      stm3220g_poweroff();
    }

  return OK;
}

/**************************************************************************************
 * Name:  stm3220g_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int stm3220g_getcontrast(struct lcd_dev_s *dev)
{
  lcdvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  stm3220g_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int stm3220g_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  lcdvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  stm3220g_lcdinitialize
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static inline void stm3220g_lcdinitialize(void)
{
  uint16_t id;

  /* Check LCD ID */

  id = stm3220g_readreg(LCD_REG_0);
  lcddbg("LCD ID: %04x\n", id);

  /* Check if the ID is for the STM32_ILI9320 (or ILI9321) or STM32_ILI9325 */

#if !defined(CONFIG_STM32_ILI9320_DISABLE) && !defined(CONFIG_STM32_ILI9325_DISABLE)
  if (id == ILI9320_ID || id == ILI9321_ID || id == ILI9325_ID)
#elif !defined(CONFIG_STM32_ILI9320_DISABLE) && defined(CONFIG_STM32_ILI9325_DISABLE)
  if (id == ILI9320_ID || id == ILI9321_ID)
#else /* if defined(CONFIG_STM32_ILI9320_DISABLE) && !defined(CONFIG_STM32_ILI9325_DISABLE)) */
  if (id == ILI9325_ID)
#endif
    {
      /* Save the LCD type (not actually used at for anything important) */

#if !defined(CONFIG_STM32_ILI9320_DISABLE)
# if !defined(CONFIG_STM32_ILI9325_DISABLE)
      if (id == ILI9325_ID)
        {
          g_lcddev.type = LCD_TYPE_ILI9325;
        }
      else
# endif
        {
          g_lcddev.type = LCD_TYPE_ILI9320;
          stm3220g_writereg(LCD_REG_229, 0x8000); /* Set the internal vcore voltage */
        }
#else /* if !defined(CONFIG_STM32_ILI9325_DISABLE) */
      g_lcddev.type = LCD_TYPE_ILI9325;
#endif
      lcddbg("LCD type: %d\n", g_lcddev.type);

      /* Start Initial Sequence */

      stm3220g_writereg(LCD_REG_0,   0x0001); /* Start internal OSC. */
      stm3220g_writereg(LCD_REG_1,   0x0100); /* Set SS and SM bit */
      stm3220g_writereg(LCD_REG_2,   0x0700); /* Set 1 line inversion */
      stm3220g_writereg(LCD_REG_3,   0x1030); /* Set GRAM write direction and BGR=1. */
    //stm3220g_writereg(LCD_REG_3,   0x1018); /* Set GRAM write direction and BGR=1. */
      stm3220g_writereg(LCD_REG_4,   0x0000); /* Resize register */
      stm3220g_writereg(LCD_REG_8,   0x0202); /* Set the back porch and front porch */
      stm3220g_writereg(LCD_REG_9,   0x0000); /* Set non-display area refresh cycle ISC[3:0] */
      stm3220g_writereg(LCD_REG_10,  0x0000); /* FMARK function */
      stm3220g_writereg(LCD_REG_12,  0x0000); /* RGB interface setting */
      stm3220g_writereg(LCD_REG_13,  0x0000); /* Frame marker Position */
      stm3220g_writereg(LCD_REG_15,  0x0000); /* RGB interface polarity */

      /* Power On sequence */

      stm3220g_writereg(LCD_REG_16,  0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
      stm3220g_writereg(LCD_REG_17,  0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
      stm3220g_writereg(LCD_REG_18,  0x0000); /* VREG1OUT voltage */
      stm3220g_writereg(LCD_REG_19,  0x0000); /* VDV[4:0] for VCOM amplitude */
      up_mdelay(200);                         /* Dis-charge capacitor power voltage (200ms) */

      stm3220g_writereg(LCD_REG_16,  0x17b0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
      stm3220g_writereg(LCD_REG_17,  0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
      up_mdelay(50);

      stm3220g_writereg(LCD_REG_18,  0x0139); /* VREG1OUT voltage */
      up_mdelay(50);

      stm3220g_writereg(LCD_REG_19,  0x1d00); /* VDV[4:0] for VCOM amplitude */
      stm3220g_writereg(LCD_REG_41,  0x0013); /* VCM[4:0] for VCOMH */
      up_mdelay(50);

      stm3220g_writereg(LCD_REG_32,  0x0000); /* GRAM horizontal Address */
      stm3220g_writereg(LCD_REG_33,  0x0000); /* GRAM Vertical Address */

      /* Adjust the Gamma Curve (ILI9320/1) */

#if !defined(CONFIG_STM32_ILI9320_DISABLE)
# if !defined(CONFIG_STM32_ILI9325_DISABLE)
    if (g_lcddev.type == LCD_TYPE_ILI9320)
# endif
      {
        stm3220g_writereg(LCD_REG_48,  0x0006);
        stm3220g_writereg(LCD_REG_49,  0x0101);
        stm3220g_writereg(LCD_REG_50,  0x0003);
        stm3220g_writereg(LCD_REG_53,  0x0106);
        stm3220g_writereg(LCD_REG_54,  0x0b02);
        stm3220g_writereg(LCD_REG_55,  0x0302);
        stm3220g_writereg(LCD_REG_56,  0x0707);
        stm3220g_writereg(LCD_REG_57,  0x0007);
        stm3220g_writereg(LCD_REG_60,  0x0600);
        stm3220g_writereg(LCD_REG_61,  0x020b);
      }
#endif
      /* Adjust the Gamma Curve (ILI9325) */

#if !defined(CONFIG_STM32_ILI9325_DISABLE)
# if !defined(CONFIG_STM32_ILI9320_DISABLE)
    else
# endif
      {
        stm3220g_writereg(LCD_REG_48, 0x0007);
        stm3220g_writereg(LCD_REG_49, 0x0302);
        stm3220g_writereg(LCD_REG_50, 0x0105);
        stm3220g_writereg(LCD_REG_53, 0x0206);
        stm3220g_writereg(LCD_REG_54, 0x0808);
        stm3220g_writereg(LCD_REG_55, 0x0206);
        stm3220g_writereg(LCD_REG_56, 0x0504);
        stm3220g_writereg(LCD_REG_57, 0x0007);
        stm3220g_writereg(LCD_REG_60, 0x0105);
        stm3220g_writereg(LCD_REG_61, 0x0808);
      }
#endif

      /* Set GRAM area */

      stm3220g_writereg(LCD_REG_80,  0x0000); /* Horizontal GRAM Start Address */
      stm3220g_writereg(LCD_REG_81,  0x00ef); /* Horizontal GRAM End Address */
      stm3220g_writereg(LCD_REG_82,  0x0000); /* Vertical GRAM Start Address */
      stm3220g_writereg(LCD_REG_83,  0x013f); /* Vertical GRAM End Address */
      stm3220g_writereg(LCD_REG_96,  0x2700); /* Gate Scan Line */
    //stm3220g_writereg(LCD_REG_96,  0xa700); /* Gate Scan Line(GS=1, scan direction is G320~G1) */
      stm3220g_writereg(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
      stm3220g_writereg(LCD_REG_106, 0x0000); /* Set scrolling line */

      /* Partial Display Control */

      stm3220g_writereg(LCD_REG_128, 0x0000);
      stm3220g_writereg(LCD_REG_129, 0x0000);
      stm3220g_writereg(LCD_REG_130, 0x0000);
      stm3220g_writereg(LCD_REG_131, 0x0000);
      stm3220g_writereg(LCD_REG_132, 0x0000);
      stm3220g_writereg(LCD_REG_133, 0x0000);

      /* Panel Control */

      stm3220g_writereg(LCD_REG_144, 0x0010);
      stm3220g_writereg(LCD_REG_146, 0x0000);
      stm3220g_writereg(LCD_REG_147, 0x0003);
      stm3220g_writereg(LCD_REG_149, 0x0110);
      stm3220g_writereg(LCD_REG_151, 0x0000);
      stm3220g_writereg(LCD_REG_152, 0x0000);

      /* Set GRAM write direction and BGR = 1
       *
       * I/D=01 (Horizontal : increment, Vertical : decrement)
       * AM=1 (address is updated in vertical writing direction)
       */

      stm3220g_writereg(LCD_REG_3, 0x1018);
      stm3220g_writereg(LCD_REG_7, 0);       /* Display off */
    }
  else
    {
      lcddbg("Unsupported LCD type\n");
    }
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
  lcdvdbg("Initializing\n");

  /* Configure GPIO pins and configure the FSMC to support the LCD */

  stm32_selectlcd();

  /* Configure and enable LCD */

  up_mdelay(50);
  stm3220g_lcdinitialize();

  /* Clear the display (setting it to the color 0=black) */

  stm3220g_lcdclear(0);

  /* Turn the display off */

  stm3220g_poweroff();
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
 *   Unitialize the LCD support
 *
 **************************************************************************************/

void up_lcduninitialize(void)
{
  stm3220g_poweroff();
  stm32_deselectlcd();
}

/**************************************************************************************
 * Name:  stm3220g_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the stm3220g-EVAL board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   display is cleared by simply setting all GRAM memory to the specified color.
 *
 **************************************************************************************/

void stm3220g_lcdclear(uint16_t color)
{
  uint32_t i = 0;
  
  stm3220g_setcursor(0, STM3220G_XRES-1); 
  stm3220g_gramselect();
  for (i = 0; i < STM3220G_XRES * STM3220G_YRES; i++)
    {
      LCD->value = color;
    }
}

#endif /* !CONFIG_STM32_ILI9320_DISABLE || !CONFIG_STM32_ILI9325_DISABLE */
