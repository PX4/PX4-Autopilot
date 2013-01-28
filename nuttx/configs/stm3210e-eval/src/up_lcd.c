/**************************************************************************************
 * configs/stm3210e-eval/src/up_lcd.c
 * arch/arm/src/board/up_lcd.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * With power management enhancements by:
 *
 *   Author: Diego Sanchez <dsanchez@nx-engineering.com>
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
/* This driver supports the following LCDs:
 *
 * 1. Ampire AM-240320LTNQW00H
 * 2. Orise Tech SPFD5408B
 * 3. RenesasSP R61580
 *
 * The driver dynamically selects the LCD based on the reported LCD ID value.  However,
 * code size can be reduced by suppressing support for individual LCDs using:
 *
 *   CONFIG_STM32_AM240320_DISABLE
 *   CONFIG_STM32_SPFD5408B_DISABLE
 *   CONFIG_STM32_R61580_DISABLE
 *
 * Omitting the above (or setting them to "n") enables support for the LCD.  Setting
 * any of the above to "y" will disable support for the corresponding LCD.
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
#include <nuttx/power/pm.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_internal.h"
#include "stm3210e-internal.h"

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/
/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 1
#endif

/* Backlight */

#ifndef CONFIG_LCD_BACKLIGHT
#  undef CONFIG_LCD_PWM
#endif

#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_LCD_PWM)
#  if !defined(CONFIG_STM32_TIM1)
#    warning "CONFIG_LCD_PWM requires CONFIG_STM32_TIM1"
#    undef CONFIG_LCD_PWM
#  endif
#  if defined(CONFIG_STM32_TIM1_FULL_REMAP)
#    warning "PA8 cannot be configured as TIM1 CH1 with full remap"
#    undef CONFIG_LCD_PWM
#  endif
#endif

#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_LCD_PWM)
#  if CONFIG_LCD_MAXPOWER < 2
#    warning "A larger value of CONFIG_LCD_MAXPOWER is recommended"
#  endif
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER) || CONFIG_LCD_MAXPOWER < 1
#  undef CONFIG_LCD_MAXPOWER
#  if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_LCD_PWM)
#    define CONFIG_LCD_MAXPOWER 100
#  else
#    define CONFIG_LCD_MAXPOWER 1
#  endif
#endif

#if CONFIG_LCD_MAXPOWER > 255
#  error "CONFIG_LCD_MAXPOWER must be less than 256 to fit in uint8_t"
#endif

/* PWM Frequency */

#ifndef CONFIG_LCD_PWMFREQUENCY
#  define CONFIG_LCD_PWMFREQUENCY 100
#endif

/* Check orientation */

#if defined(CONFIG_LCD_PORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_RPORTRAIT)
#    error "Cannot define both portrait and any other orientations"
#  endif
#elif defined(CONFIG_LCD_RPORTRAIT)
#  if defined(CONFIG_LCD_LANDSCAPE) || defined(CONFIG_LCD_PORTRAIT)
#    error "Cannot define both rportrait and any other orientations"
#  endif
#elif !defined(CONFIG_LCD_LANDSCAPE)
#  define CONFIG_LCD_LANDSCAPE 1
#endif

/* When reading 16-bit gram data, there may some shifts in the returned data
 * and/or there may be some colors in the incorrect posisions:
 *
 * - SPFD5408B:  There appears to be a 5-bit shift in the returned data.
 *               Red and green appear to be swapped on read-back as well
 * - R61580:     There is a 16-bit (1 pixel) shift in the returned data.
 * - AM240320:   Unknown -- assume colors are correct for now.
 */

#define SPFD5408B_RDSHIFT 5

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

#ifdef CONFIG_LCD_LANDSCAPE
#  define STM3210E_XRES       320
#  define STM3210E_YRES       240
#else
#  define STM3210E_XRES       240
#  define STM3210E_YRES       320
#endif

/* Color depth and format */

#define STM3210E_BPP          16
#define STM3210E_COLORFMT     FB_FMT_RGB16_565

/* STM3210E-EVAL LCD Hardware Definitions *********************************************/
/* LCD /CS is CE4,  Bank 4 of NOR/SRAM Bank 1~4 */

#define STM3210E_LCDBASE      ((uint32_t)(0x60000000 | 0x0c000000))
#define LCD                   ((struct lcd_regs_s *) STM3210E_LCDBASE)

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

#define SPFD5408B_ID          0x5408
#define R61580_ID             0x1580

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
# define lcddbg(format, arg...)  vdbg(format, ##arg)
#else
# define lcddbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

/* LCD type */

enum lcd_type_e
{
  LCD_TYPE_UNKNOWN = 0,
  LCD_TYPE_SPFD5408B,
  LCD_TYPE_R61580,
  LCD_TYPE_AM240320
};

/* This structure describes the LCD registers */

struct lcd_regs_s
{
  volatile uint16_t address;
  volatile uint16_t value;
};

/* This structure describes the state of this driver */

struct stm3210e_dev_s
{
  /* Publically visible device structure */

  struct lcd_dev_s dev;

#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_LCD_PWM)
  uint32_t reload;
#endif

  /* Private LCD-specific information follows */

  uint8_t  type;        /* LCD type. See enum lcd_type_e */
  uint8_t  power;       /* Current power setting */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Low Level LCD access */

static void stm3210e_writereg(uint8_t regaddr, uint16_t regval);
static uint16_t stm3210e_readreg(uint8_t regaddr);
static inline void stm3210e_gramselect(void);
static inline void stm3210e_writegram(uint16_t rgbval);
static void stm3210e_readsetup(FAR uint16_t *accum);
#ifndef CONFIG_STM32_AM240320_DISABLE
static void stm3210e_readnosetup(FAR uint16_t *accum);
#endif
static uint16_t stm3210e_readshift(FAR uint16_t *accum);
static uint16_t stm3210e_readnoshift(FAR uint16_t *accum);
static void stm3210e_setcursor(uint16_t col, uint16_t row);

/* LCD Data Transfer Methods */

static int stm3210e_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
             size_t npixels);
static int stm3210e_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
             size_t npixels);

/* LCD Configuration */

static int stm3210e_getvideoinfo(FAR struct lcd_dev_s *dev,
             FAR struct fb_videoinfo_s *vinfo);
static int stm3210e_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
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

static int stm3210e_getpower(struct lcd_dev_s *dev);
static int stm3210e_setpower(struct lcd_dev_s *dev, int power);
static int stm3210e_getcontrast(struct lcd_dev_s *dev);
static int stm3210e_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/* LCD Power Management */

#ifdef CONFIG_PM
static void stm3210e_pm_notify(struct pm_callback_s *cb, enum pm_state_e pmstate);
static int stm3210e_pm_prepare(struct pm_callback_s *cb, enum pm_state_e pmstate);
#endif

/* Initialization */

static inline void stm3210e_lcdinitialize(void);
#ifdef CONFIG_LCD_BACKLIGHT
static void stm3210e_backlight(void);
#else
#  define stm3210e_backlight()
#endif

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

static uint16_t g_runbuffer[STM3210E_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = STM3210E_COLORFMT,    /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = STM3210E_XRES,        /* Horizontal resolution in pixel columns */
  .yres    = STM3210E_YRES,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                    /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun = stm3210e_putrun,       /* Put a run into LCD memory */
  .getrun = stm3210e_getrun,       /* Get a run from LCD memory */
  .buffer = (uint8_t*)g_runbuffer, /* Run scratch buffer */
  .bpp    = STM3210E_BPP,          /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct stm3210e_dev_s g_lcddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = stm3210e_getvideoinfo,
    .getplaneinfo = stm3210e_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */
    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = stm3210e_getpower,
    .setpower     = stm3210e_setpower,
    .getcontrast  = stm3210e_getcontrast,
    .setcontrast  = stm3210e_setcontrast,
  },
};

#ifdef CONFIG_PM
static struct pm_callback_s g_lcdcb =
{
  .notify  = stm3210e_pm_notify,
  .prepare = stm3210e_pm_prepare,
};
#endif


/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name:  stm3210e_writereg
 *
 * Description:
 *   Write to an LCD register
 *
 **************************************************************************************/

static void stm3210e_writereg(uint8_t regaddr, uint16_t regval)
{
  /* Write the register address then write the register value */

  LCD->address = regaddr;
  LCD->value   = regval;
}

/**************************************************************************************
 * Name:  stm3210e_readreg
 *
 * Description:
 *   Read from an LCD register
 *
 **************************************************************************************/

static uint16_t stm3210e_readreg(uint8_t regaddr)
{
  /* Write the register address then read the register value */

  LCD->address = regaddr;
  return LCD->value;
}

/**************************************************************************************
 * Name:  stm3210e_gramselect
 *
 * Description:
 *   Setup to read or write multiple pixels to the GRAM memory
 *
 **************************************************************************************/

static inline void stm3210e_gramselect(void)
{
  LCD->address = LCD_REG_34;
}

/**************************************************************************************
 * Name:  stm3210e_writegram
 *
 * Description:
 *   Write one pixel to the GRAM memory
 *
 **************************************************************************************/

static inline void stm3210e_writegram(uint16_t rgbval)
{
  /* Write the value (GRAM register already selected) */

  LCD->value = rgbval;
}

/**************************************************************************************
 * Name:  stm3210e_readsetup / stm3210e_readnosetup
 *
 * Description:
 *   Prime the operation by reading one pixel from the GRAM memory if necessary for
 *   this LCD type.  When reading 16-bit gram data, there may be some shifts in the
 *   returned data:
 *
 *   - SPFD5408B:  There appears to be a 5-bit shift in the returned data.
 *   - R61580:     There is a 16-bit (1 pixel) shift in the returned data.
 *   - AM240320:   Unknown -- assuming no shift in the return data
 *
 **************************************************************************************/

/* Used for SPFD5408B and R61580 */

#if !defined(CONFIG_STM32_SPFD5408B_DISABLE) || !defined(CONFIG_STM32_R61580_DISABLE)
static void stm3210e_readsetup(FAR uint16_t *accum)
{
  /* Read-ahead one pixel */

  *accum  = LCD->value;
}
#endif

/* Used only for AM240320 */

#ifndef CONFIG_STM32_AM240320_DISABLE
static void stm3210e_readnosetup(FAR uint16_t *accum)
{
}
#endif

/**************************************************************************************
 * Name:  stm3210e_readshift / stm3210e_readnoshift
 *
 * Description:
 *   Read one correctly aligned pixel from the GRAM memory.  Possibly shifting the
 *   data and possibly swapping red and green components.
 *
 *   - SPFD5408B:  There appears to be a 5-bit shift in the returned data.
 *                 Red and green appear to be swapped on read-back as well
 *   - R61580:     There is a 16-bit (1 pixel) shift in the returned data.
 *                 All colors in the normal order
 *   - AM240320:   Unknown -- assuming colors are in the color order
 *
 **************************************************************************************/

/* This version is used only for the SPFD5408B.  It shifts the data by 5-bits and swaps
 * red and green
 */

#ifndef CONFIG_STM32_SPFD5408B_DISABLE
static uint16_t stm3210e_readshift(FAR uint16_t *accum)
{
  uint16_t red;
  uint16_t green;
  uint16_t blue;

  /* Read the value (GRAM register already selected) */

  uint16_t next = LCD->value;

  /* Return previous bits 0-10 as bits 6-15 and next data bits 11-15 as bits 0-5
   *
   *   xxxx xPPP PPPP PPPP
   *   NNNN Nxxx xxxx xxxx
   *
   * Assuming that SPFD5408B_RDSHIFT == 5
   */

  uint16_t value  = *accum << SPFD5408B_RDSHIFT | next >> (16-SPFD5408B_RDSHIFT);

  /* Save the value for the next time we are called */

  *accum = next;

  /* Tear the RGB655 apart. Swap read and green */

  red   = (value << (11-5)) & 0xf800; /* Move bits 5-9 to 11-15 */
  green = (value >> (10-5)) & 0x07e0; /* Move bits 10-15 to bits 5-10 */
  blue  =  value            & 0x001f; /* Blue is in the right place */

  /* And put the RGB565 back together */

  value = red | green | blue;

  /* This is wierd... If blue is zero, then red+green values are off by 0x20.
   * Except that both 0x0000 and 0x0020 can map to 0x0000.  Need to revisit
   * this!!!!!!!!!!!  I might be misinterpreting some of the data that I have.
   */

#if 0 /* REVISIT */
  if (value != 0 && blue == 0)
    {
      value += 0x20;
    }
#endif

  return value;
}
#endif

/* This version is used for the R61580 and for the AM240320.  It neither shifts nor
 * swaps colors.
 */

#if !defined(CONFIG_STM32_R61580_DISABLE) || !defined(CONFIG_STM32_AM240320_DISABLE)
static uint16_t stm3210e_readnoshift(FAR uint16_t *accum)
{
  /* Read the value (GRAM register already selected) */

  return LCD->value;
}
#endif

/**************************************************************************************
 * Name:  stm3210e_setcursor
 *
 * Description:
 *   Set the cursor position.  In landscape mode, the "column" is actually the physical
 *   Y position and the "row" is the physical X position.
 *
 **************************************************************************************/

static void stm3210e_setcursor(uint16_t col, uint16_t row)
{
  stm3210e_writereg(LCD_REG_32, row); /* GRAM horizontal address */
  stm3210e_writereg(LCD_REG_33, col); /* GRAM vertical address */
}

/**************************************************************************************
 * Name:  stm3210e_dumprun
 *
 * Description:
 *   Dump the contexts of the run buffer:
 *
 *  run     - The buffer in containing the run read to be dumped
 *  npixels - The number of pixels to dump
 *
 **************************************************************************************/

#if 0 /* Sometimes useful */
static void stm3210e_dumprun(FAR const char *msg, FAR uint16_t *run, size_t npixels)
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
 * Name:  stm3210e_putrun
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

static int stm3210e_putrun(fb_coord_t row, fb_coord_t col, FAR const uint8_t *buffer,
                       size_t npixels)
{
  FAR const uint16_t *src = (FAR const uint16_t*)buffer;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcddbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Write the run to GRAM. */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates -- Which edge of the display is the "top?" Here the edge
   * with the simplest conversion is used.
   */

  col = (STM3210E_XRES-1) - col;

  /* Set the cursor position */

  stm3210e_setcursor(col, row);

  /* Then write the GRAM data, auto-decrementing X */

  stm3210e_gramselect();
  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position (auto-decrements to the next column) */

      stm3210e_writegram(*src++);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates.  (Swap row and column.  This is done implicitly). */

  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3210e_setcursor(row, col);
      stm3210e_gramselect();
      stm3210e_writegram(*src++);

      /* Increment to next column */

      col++;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates.  (Swap row and column.  This is done implicitly).
   * Which edge of the display is the "top"?
   */

  col = (STM3210E_XRES-1) - col;
  row = (STM3210E_YRES-1) - row;

  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3210e_setcursor(row, col);
      stm3210e_gramselect();
      stm3210e_writegram(*src++);

      /* Decrement to next column */

      col--;
    }
#endif
  return OK;
}

/**************************************************************************************
 * Name:  stm3210e_getrun
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

static int stm3210e_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                       size_t npixels)
{
  FAR uint16_t *dest = (FAR uint16_t*)buffer;
  void (*readsetup)(FAR uint16_t *accum);
  uint16_t (*readgram)(FAR uint16_t *accum);
  uint16_t accum;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */

  lcddbg("row: %d col: %d npixels: %d\n", row, col, npixels);
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Configure according to the LCD type */

  switch (g_lcddev.type)
   {
#ifndef CONFIG_STM32_SPFD5408B_DISABLE
     case LCD_TYPE_SPFD5408B:
       readsetup = stm3210e_readsetup;
       readgram  = stm3210e_readshift;
       break;
#endif

#ifndef CONFIG_STM32_R61580_DISABLE
     case LCD_TYPE_R61580:
       readsetup = stm3210e_readsetup;
       readgram  = stm3210e_readnoshift;
       break;
#endif

#ifndef CONFIG_STM32_AM240320_DISABLE
     case LCD_TYPE_AM240320:
       readsetup = stm3210e_readnosetup;
       readgram  = stm3210e_readnoshift;
       break;
#endif

     default:  /* Shouldn't happen */
       return -ENOSYS;
   }

  /* Read the run from GRAM. */

#ifdef CONFIG_LCD_LANDSCAPE
  /* Convert coordinates -- Which edge of the display is the "top?" Here the edge
   * with the simplest conversion is used.
   */

  col = (STM3210E_XRES-1) - col;

  /* Set the cursor position */

  stm3210e_setcursor(col, row);

  /* Then read the GRAM data, auto-decrementing Y */

  stm3210e_gramselect();

  /* Prime the pump for unaligned read data */

  readsetup(&accum);

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position (autoincrements to the next row) */

      *dest++ = readgram(&accum);
    }
#elif defined(CONFIG_LCD_PORTRAIT)
  /* Convert coordinates (Swap row and column.  This is done implicitly). */

  /* Then read the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Read the next pixel from this position */

      stm3210e_setcursor(row, col);
      stm3210e_gramselect();
      readsetup(&accum);
      *dest++ = readgram(&accum);

      /* Increment to next column */

      col++;
    }
#else /* CONFIG_LCD_RPORTRAIT */
  /* Convert coordinates.  (Swap row and column.  This is done implicitly).
   * Whic edge of the display is the "top"?
   */

  col = (STM3210E_XRES-1) - col;
  row = (STM3210E_YRES-1) - row;

  /* Then write the GRAM data, manually incrementing Y (which is col) */

  for (i = 0; i < npixels; i++)
    {
      /* Write the next pixel to this position */

      stm3210e_setcursor(row, col);
      stm3210e_gramselect();
      readsetup(&accum);
      *dest++ = readgram(&accum);

      /* Decrement to next column */

      col--;
    }
#endif

  return OK;
}

/**************************************************************************************
 * Name:  stm3210e_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 **************************************************************************************/

static int stm3210e_getvideoinfo(FAR struct lcd_dev_s *dev,
                              FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  gvdbg("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres, g_videoinfo.nplanes);
  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  stm3210e_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 **************************************************************************************/

static int stm3210e_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  gvdbg("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/**************************************************************************************
 * Name:  stm3210e_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int stm3210e_getpower(struct lcd_dev_s *dev)
{
  gvdbg("power: %d\n", 0);
  return g_lcddev.power;
}

/**************************************************************************************
 * Name:  stm3210e_poweroff
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int stm3210e_poweroff(void)
{
  /* Turn the display off */

  stm3210e_writereg(LCD_REG_7, 0);

  /* Disable timer 1 clocking */

#if defined(CONFIG_LCD_BACKLIGHT)
# if defined(CONFIG_LCD_PWM)
  modifyreg32(STM32_RCC_APB2ENR, RCC_APB2ENR_TIM1EN, 0);
#endif

  /* Configure the PA8 pin as an output */

  stm32_configgpio(GPIO_LCD_BACKLIGHT);

  /* Turn the backlight off */

  stm32_gpiowrite(GPIO_LCD_BACKLIGHT, false);
#endif

  /* Remember the power off state */

  g_lcddev.power = 0;
  return OK;
}

/**************************************************************************************
 * Name:  stm3210e_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER: full on). On
 *   backlit LCDs, this setting may correspond to the backlight setting.
 *
 **************************************************************************************/

static int stm3210e_setpower(struct lcd_dev_s *dev, int power)
{
  gvdbg("power: %d\n", power);
  DEBUGASSERT((unsigned)power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
#if defined(CONFIG_LCD_BACKLIGHT) && defined(CONFIG_LCD_PWM)
      uint32_t frac;
      uint32_t duty;

      /* If we are coming up from the power off state, then re-configure the timer */

      if (g_lcddev.power == 0)
        {
          stm3210e_backlight();
        }

      /* Make sure that the power value is within range */

      if (power > CONFIG_LCD_MAXPOWER)
        {
          power = CONFIG_LCD_MAXPOWER;
        }

      /* Caclulate the new backlight duty.  It is a faction of the timer1
       * period based on the ration of the current power setting to the
       * maximum power setting.
       */

      frac = (power << 16) / CONFIG_LCD_MAXPOWER;
      duty = (g_lcddev.reload * frac) >> 16;
      if (duty > 0)
        {
          duty--;
        }

      putreg16((uint16_t)duty, STM32_TIM1_CCR1);
#else
      /* Turn the backlight on */

      stm32_gpiowrite(GPIO_LCD_BACKLIGHT, true);
#endif
      /* Then turn the display on */

#ifndef CONFIG_STM32_AM240320_DISABLE
#  if !defined (CONFIG_STM32_SPFD5408B_DISABLE) || !defined(CONFIG_STM32_R61580_DISABLE)
      stm3210e_writereg(LCD_REG_7, g_lcddev.type == LCD_TYPE_AM240320 ? 0x0173 : 0x0112);
#  else
      stm3210e_writereg(LCD_REG_7, 0x0173);
#  endif
#else
      stm3210e_writereg(LCD_REG_7, 0x0112);
#endif
      g_lcddev.power = power;
    }
  else
    {
      /* Turn the display off */

      stm3210e_poweroff();
    }

  return OK;
}

/**************************************************************************************
 * Name:  stm3210e_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int stm3210e_getcontrast(struct lcd_dev_s *dev)
{
  gvdbg("Not implemented\n");
  return -ENOSYS;
}

/**************************************************************************************
 * Name:  stm3210e_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static int stm3210e_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  gvdbg("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Name: stm3210e_pm_notify
 *
 * Description:
 *   Notify the driver of new power state. This callback is called after
 *   all drivers have had the opportunity to prepare for the new power state.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         strucure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   None - The driver already agreed to transition to the low power
 *   consumption state when when it returned OK to the prepare() call.
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static void stm3210e_pm_notify(struct pm_callback_s *cb , enum pm_state_e pmstate)
{
#ifdef CONFIG_LCD_PWM
  uint32_t frac;
  uint32_t duty;
#endif

  switch (pmstate)
    {
      case(PM_NORMAL):
        {
          /* Restore normal LCD operation */

#ifdef CONFIG_LCD_PWM
          frac = (g_lcddev.power << 16) / CONFIG_LCD_MAXPOWER;
          duty = (g_lcddev.reload * frac) >> 16;
          if (duty > 0)
            {
              duty--;
            }

          putreg16((uint16_t)duty, STM32_TIM1_CCR1);
#endif
        }
        break;

      case(PM_IDLE):
        {
          /* Entering IDLE mode - Reduce LCD light */

#ifdef CONFIG_LCD_PWM
          frac = (g_lcddev.power << 16) / CONFIG_LCD_MAXPOWER;
          duty = (g_lcddev.reload * frac) >> 16;
          if (duty > 0)
            {
              duty--;
            }

          /* Reduce the LCD backlight to 50% of the MAXPOWER */

          duty >>= 1;
          putreg16((uint16_t)duty, STM32_TIM1_CCR1);
#endif
        }
        break;

      case(PM_STANDBY):
        {
          /* Entering STANDBY mode - Turn display backlight off */

#ifdef CONFIG_LCD_PWM
          putreg16(0, STM32_TIM1_CCR1);
#endif
        }
        break;

      case(PM_SLEEP):
        {
          /* Entering SLEEP mode - Turn off LCD */

          if (g_lcddev.type == LCD_TYPE_AM240320)
            {
              /* Display off sequence */

              stm3210e_writereg(LCD_REG_0,  0xa0); /* White display mode setting */
              up_mdelay(10);                       /* Wait for 2 frame scan */
              stm3210e_writereg(LCD_REG_59, 0x00); /* Gate scan stop */

              /* Power off sequence */

              stm3210e_writereg(LCD_REG_30, 0x09); /* VCOM stop */
              stm3210e_writereg(LCD_REG_27, 0x0e); /* VS/VDH turn off */
              stm3210e_writereg(LCD_REG_24, 0xc0); /* CP1, CP2, CP3 turn off */
              up_mdelay(10);                       /* wait 10 ms */

              stm3210e_writereg(LCD_REG_24, 0x00); /* VR1 / VR2 off*/
              stm3210e_writereg(LCD_REG_28, 0x30); /* Step up circuit operating current stop */
              up_mdelay(10);

              stm3210e_poweroff();
              stm3210e_writereg(LCD_REG_0,  0xa0); /* White display mode setting */
              up_mdelay(10);                       /* Wait for 2 frame scan */

              stm3210e_writereg(LCD_REG_59, 0x00); /* Gate scan stop */
            }
          else
            {
              (void)stm3210e_poweroff();
            }
        }
        break;

      default:
        {
          /* Should not get here */

        }
        break;
    }
}
#endif

/****************************************************************************
 * Name: stm3210e_pm_prepare
 *
 * Description:
 *   Request the driver to prepare for a new power state. This is a warning
 *   that the system is about to enter into a new power state. The driver
 *   should begin whatever operations that may be required to enter power
 *   state. The driver may abort the state change mode by returning a
 *   non-zero value from the callback function.
 *
 * Input Parameters:
 *
 *    cb - Returned to the driver. The driver version of the callback
 *         strucure may include additional, driver-specific state data at
 *         the end of the structure.
 *
 *    pmstate - Identifies the new PM state
 *
 * Returned Value:
 *   Zero - (OK) means the event was successfully processed and that the
 *          driver is prepared for the PM state change.
 *
 *   Non-zero - means that the driver is not prepared to perform the tasks
 *              needed achieve this power setting and will cause the state
 *              change to be aborted. NOTE: The prepare() method will also
 *              be called when reverting from lower back to higher power
 *              consumption modes (say because another driver refused a
 *              lower power state change). Drivers are not permitted to
 *              return non-zero values when reverting back to higher power
 *              consumption modes!
 *
 *
 ****************************************************************************/

#ifdef CONFIG_PM
static int stm3210e_pm_prepare(struct pm_callback_s *cb , enum pm_state_e pmstate)
{
  /* No preparation to change power modes is required by the LCD driver.  We always
   * accept the state change by returning OK.
   */

  return OK;
}
#endif

/**************************************************************************************
 * Name:  stm3210e_lcdinitialize
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 **************************************************************************************/

static inline void stm3210e_lcdinitialize(void)
{
  uint16_t id;

  /* Check if the LCD is Orise Tech SPFD5408B Controller (or the compatible RenesasSP
   * R61580).
   */

  id = stm3210e_readreg(LCD_REG_0);
  lcddbg("LCD ID: %04x\n", id);

  /* Check if the ID is for the SPFD5408B */

#if !defined(CONFIG_STM32_SPFD5408B_DISABLE)
  if (id == SPFD5408B_ID)
    {
      /* Set the LCD type for the SPFD5408B */

      g_lcddev.type = LCD_TYPE_SPFD5408B;
      lcddbg("LCD type: %d\n", g_lcddev.type);

      /* Start Initial Sequence */

      stm3210e_writereg(LCD_REG_1,   0x0100); /* Set SS bit */
      stm3210e_writereg(LCD_REG_2,   0x0700); /* Set 1 line inversion */
      stm3210e_writereg(LCD_REG_3,   0x1030); /* Set GRAM write direction and BGR=1. */
      stm3210e_writereg(LCD_REG_4,   0x0000); /* Resize register */
      stm3210e_writereg(LCD_REG_8,   0x0202); /* Set the back porch and front porch */
      stm3210e_writereg(LCD_REG_9,   0x0000); /* Set non-display area refresh cycle ISC[3:0] */
      stm3210e_writereg(LCD_REG_10,  0x0000); /* FMARK function */
      stm3210e_writereg(LCD_REG_12,  0x0000); /* RGB 18-bit System interface setting */
      stm3210e_writereg(LCD_REG_13,  0x0000); /* Frame marker Position */
      stm3210e_writereg(LCD_REG_15,  0x0000); /* RGB interface polarity, no impact */

      /* Power On sequence */

      stm3210e_writereg(LCD_REG_16,  0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
      stm3210e_writereg(LCD_REG_17,  0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
      stm3210e_writereg(LCD_REG_18,  0x0000); /* VREG1OUT voltage */
      stm3210e_writereg(LCD_REG_19,  0x0000); /* VDV[4:0] for VCOM amplitude */
      up_mdelay(200);                         /* Dis-charge capacitor power voltage (200ms) */

      stm3210e_writereg(LCD_REG_17,  0x0007);  /* DC1[2:0], DC0[2:0], VC[2:0] */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_16,  0x12B0);  /* SAP, BT[3:0], AP, DSTB, SLP, STB */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_18,  0x01bd);  /* External reference voltage= Vci */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_19,  0x1400);  /* VDV[4:0] for VCOM amplitude */
      stm3210e_writereg(LCD_REG_41,  0x000e);  /* VCM[4:0] for VCOMH */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_32,  0x0000); /* GRAM horizontal Address */
      stm3210e_writereg(LCD_REG_33,  0x013f); /* GRAM Vertical Address */

      /* Adjust the Gamma Curve (SPFD5408B)*/

      stm3210e_writereg(LCD_REG_48,  0x0b0d);
      stm3210e_writereg(LCD_REG_49,  0x1923);
      stm3210e_writereg(LCD_REG_50,  0x1c26);
      stm3210e_writereg(LCD_REG_51,  0x261c);
      stm3210e_writereg(LCD_REG_52,  0x2419);
      stm3210e_writereg(LCD_REG_53,  0x0d0b);
      stm3210e_writereg(LCD_REG_54,  0x1006);
      stm3210e_writereg(LCD_REG_55,  0x0610);
      stm3210e_writereg(LCD_REG_56,  0x0706);
      stm3210e_writereg(LCD_REG_57,  0x0304);
      stm3210e_writereg(LCD_REG_58,  0x0e05);
      stm3210e_writereg(LCD_REG_59,  0x0e01);
      stm3210e_writereg(LCD_REG_60,  0x010e);
      stm3210e_writereg(LCD_REG_61,  0x050e);
      stm3210e_writereg(LCD_REG_62,  0x0403);
      stm3210e_writereg(LCD_REG_63,  0x0607);

      /* Set GRAM area */

      stm3210e_writereg(LCD_REG_80,  0x0000); /* Horizontal GRAM Start Address */
      stm3210e_writereg(LCD_REG_81,  0x00ef); /* Horizontal GRAM End Address */
      stm3210e_writereg(LCD_REG_82,  0x0000); /* Vertical GRAM Start Address */
      stm3210e_writereg(LCD_REG_83,  0x013f); /* Vertical GRAM End Address */
      stm3210e_writereg(LCD_REG_96,  0xa700); /* Gate Scan Line */
      stm3210e_writereg(LCD_REG_97,  0x0001); /* NDL, VLE, REV */
      stm3210e_writereg(LCD_REG_106, 0x0000); /* set scrolling line */

      /* Partial Display Control */

      stm3210e_writereg(LCD_REG_128, 0x0000);
      stm3210e_writereg(LCD_REG_129, 0x0000);
      stm3210e_writereg(LCD_REG_130, 0x0000);
      stm3210e_writereg(LCD_REG_131, 0x0000);
      stm3210e_writereg(LCD_REG_132, 0x0000);
      stm3210e_writereg(LCD_REG_133, 0x0000);

      /* Panel Control */

      stm3210e_writereg(LCD_REG_144, 0x0010);
      stm3210e_writereg(LCD_REG_146, 0x0000);
      stm3210e_writereg(LCD_REG_147, 0x0003);
      stm3210e_writereg(LCD_REG_149, 0x0110);
      stm3210e_writereg(LCD_REG_151, 0x0000);
      stm3210e_writereg(LCD_REG_152, 0x0000);

      /* Set GRAM write direction and BGR=1
       * I/D=01 (Horizontal : increment, Vertical : decrement)
       * AM=1 (address is updated in vertical writing direction)
       */

      stm3210e_writereg(LCD_REG_3, 0x1018);
      stm3210e_writereg(LCD_REG_7, 0);      /* Display OFF */
    }
  else
#endif

  /* Check if the ID is for the almost compatible R61580 */

#if !defined(CONFIG_STM32_R61580_DISABLE)
  if (id == R61580_ID)
    {
      /* Set the LCD type for the R61580 */

      g_lcddev.type = LCD_TYPE_R61580;
      lcddbg("LCD type: %d\n", g_lcddev.type);

      /* Start Initial Sequence */

      stm3210e_writereg(LCD_REG_0,   0x0000);
      stm3210e_writereg(LCD_REG_0,   0x0000);
      up_mdelay(100);
      stm3210e_writereg(LCD_REG_0,   0x0000);
      stm3210e_writereg(LCD_REG_0,   0x0000);
      stm3210e_writereg(LCD_REG_0,   0x0000);
      stm3210e_writereg(LCD_REG_0,   0x0000);
      stm3210e_writereg(LCD_REG_164, 0x0001);
      up_mdelay(100);
      stm3210e_writereg(LCD_REG_96,  0xa700);
      stm3210e_writereg(LCD_REG_8,   0x0808);

      /* Gamma Setting */

      stm3210e_writereg(LCD_REG_48,  0x0203);
      stm3210e_writereg(LCD_REG_49,  0x080f);
      stm3210e_writereg(LCD_REG_50,  0x0401);
      stm3210e_writereg(LCD_REG_51,  0x050b);
      stm3210e_writereg(LCD_REG_52,  0x3330);
      stm3210e_writereg(LCD_REG_53,  0x0b05);
      stm3210e_writereg(LCD_REG_54,  0x0005);
      stm3210e_writereg(LCD_REG_55,  0x0f08);
      stm3210e_writereg(LCD_REG_56,  0x0302);
      stm3210e_writereg(LCD_REG_57,  0x3033);

      /* Power Setting */

      stm3210e_writereg(LCD_REG_144, 0x0018); /* 80Hz */
      stm3210e_writereg(LCD_REG_16,  0x0530); /* BT, AP */
      stm3210e_writereg(LCD_REG_17,  0x0237); /* DC1,DC0,VC */
      stm3210e_writereg(LCD_REG_18,  0x01bf);
      stm3210e_writereg(LCD_REG_19,  0x1000); /* VCOM */
      up_mdelay(200);

      stm3210e_writereg(LCD_REG_1,   0x0100); /* Set SS bit */
      stm3210e_writereg(LCD_REG_2,   0x0200);
      stm3210e_writereg(LCD_REG_3,   0x1030); /* Set GRAM write direction and BGR=1. */
      stm3210e_writereg(LCD_REG_9,   0x0001);
      stm3210e_writereg(LCD_REG_10,  0x0008);
      stm3210e_writereg(LCD_REG_12,  0x0000); /* RGB 18-bit System interface setting */
      stm3210e_writereg(LCD_REG_13,  0xd000);
      stm3210e_writereg(LCD_REG_14,  0x0030);
      stm3210e_writereg(LCD_REG_15,  0x0000); /* RGB interface polarity, no impact */
      stm3210e_writereg(LCD_REG_32,  0x0000); /* H Start */
      stm3210e_writereg(LCD_REG_33,  0x0000); /* V Start */
      stm3210e_writereg(LCD_REG_41,  0x002e);
      stm3210e_writereg(LCD_REG_80,  0x0000); /* Horizontal GRAM Start Address */
      stm3210e_writereg(LCD_REG_81,  0x00ef); /* Horizontal GRAM End Address */
      stm3210e_writereg(LCD_REG_82,  0x0000); /* Vertical GRAM Start Address */
      stm3210e_writereg(LCD_REG_83,  0x013f); /* Vertical GRAM End Address */
      stm3210e_writereg(LCD_REG_97,  0x0001); /* NDL, VLE, REV */
      stm3210e_writereg(LCD_REG_106, 0x0000); /* set scrolling line */
      stm3210e_writereg(LCD_REG_128, 0x0000);
      stm3210e_writereg(LCD_REG_129, 0x0000);
      stm3210e_writereg(LCD_REG_130, 0x005f);
      stm3210e_writereg(LCD_REG_147, 0x0701);

      stm3210e_writereg(LCD_REG_7,   0x0000); /* Display OFF */
    }
  else
#endif
    {
#ifndef CONFIG_STM32_AM240320_DISABLE
      /* Set the LCD type for the AM240320 */

      g_lcddev.type = LCD_TYPE_AM240320;
      lcddbg("LCD type: %d\n", g_lcddev.type);

      /* Start Initial Sequence */

      stm3210e_writereg(LCD_REG_229, 0x8000); /* Set the internal vcore voltage */
      stm3210e_writereg(LCD_REG_0,   0x0001); /* Start internal OSC. */
      stm3210e_writereg(LCD_REG_1,   0x0100); /* Set SS and SM bit */
      stm3210e_writereg(LCD_REG_2,   0x0700); /* Set 1 line inversion */
      stm3210e_writereg(LCD_REG_3,   0x1030); /* Set GRAM write direction and BGR=1. */
      stm3210e_writereg(LCD_REG_4,   0x0000); /* Resize register */
      stm3210e_writereg(LCD_REG_8,   0x0202); /* Set the back porch and front porch */
      stm3210e_writereg(LCD_REG_9,   0x0000); /* Set non-display area refresh cycle ISC[3:0] */
      stm3210e_writereg(LCD_REG_10,  0x0000); /* FMARK function */
      stm3210e_writereg(LCD_REG_12,  0x0000); /* RGB interface setting */
      stm3210e_writereg(LCD_REG_13,  0x0000); /* Frame marker Position */
      stm3210e_writereg(LCD_REG_15,  0x0000); /* RGB interface polarity */

      /* Power On sequence */

      stm3210e_writereg(LCD_REG_16,  0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
      stm3210e_writereg(LCD_REG_17,  0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
      stm3210e_writereg(LCD_REG_18,  0x0000); /* VREG1OUT voltage */
      stm3210e_writereg(LCD_REG_19,  0x0000); /* VDV[4:0] for VCOM amplitude */
      up_mdelay(200);                         /* Dis-charge capacitor power voltage (200ms) */

      stm3210e_writereg(LCD_REG_16,  0x17b0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
      stm3210e_writereg(LCD_REG_17,  0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_18,  0x0139); /* VREG1OUT voltage */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_19,  0x1d00); /* VDV[4:0] for VCOM amplitude */
      stm3210e_writereg(LCD_REG_41,  0x0013); /* VCM[4:0] for VCOMH */
      up_mdelay(50);

      stm3210e_writereg(LCD_REG_32,  0x0000); /* GRAM horizontal Address */
      stm3210e_writereg(LCD_REG_33,  0x0000); /* GRAM Vertical Address */

      /* Adjust the Gamma Curve */

      stm3210e_writereg(LCD_REG_48,  0x0006);
      stm3210e_writereg(LCD_REG_49,  0x0101);
      stm3210e_writereg(LCD_REG_50,  0x0003);
      stm3210e_writereg(LCD_REG_53,  0x0106);
      stm3210e_writereg(LCD_REG_54,  0x0b02);
      stm3210e_writereg(LCD_REG_55,  0x0302);
      stm3210e_writereg(LCD_REG_56,  0x0707);
      stm3210e_writereg(LCD_REG_57,  0x0007);
      stm3210e_writereg(LCD_REG_60,  0x0600);
      stm3210e_writereg(LCD_REG_61,  0x020b);

      /* Set GRAM area */

      stm3210e_writereg(LCD_REG_80,  0x0000); /* Horizontal GRAM Start Address */
      stm3210e_writereg(LCD_REG_81,  0x00ef); /* Horizontal GRAM End Address */
      stm3210e_writereg(LCD_REG_82,  0x0000); /* Vertical GRAM Start Address */
      stm3210e_writereg(LCD_REG_83,  0x013f); /* Vertical GRAM End Address */
      stm3210e_writereg(LCD_REG_96,  0x2700); /* Gate Scan Line */
      stm3210e_writereg(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
      stm3210e_writereg(LCD_REG_106, 0x0000); /* Set scrolling line */

      /* Partial Display Control */

      stm3210e_writereg(LCD_REG_128, 0x0000);
      stm3210e_writereg(LCD_REG_129, 0x0000);
      stm3210e_writereg(LCD_REG_130, 0x0000);
      stm3210e_writereg(LCD_REG_131, 0x0000);
      stm3210e_writereg(LCD_REG_132, 0x0000);
      stm3210e_writereg(LCD_REG_133, 0x0000);

      /* Panel Control */

      stm3210e_writereg(LCD_REG_144, 0x0010);
      stm3210e_writereg(LCD_REG_146, 0x0000);
      stm3210e_writereg(LCD_REG_147, 0x0003);
      stm3210e_writereg(LCD_REG_149, 0x0110);
      stm3210e_writereg(LCD_REG_151, 0x0000);
      stm3210e_writereg(LCD_REG_152, 0x0000);

      /* Set GRAM write direction and BGR = 1
       *
       * I/D=01 (Horizontal : increment, Vertical : decrement)
       * AM=1 (address is updated in vertical writing direction)
       */

      stm3210e_writereg(LCD_REG_3, 0x1018);
      stm3210e_writereg(LCD_REG_7, 0);       /* Display off */
#else
      lcddbg("Unsupported LCD type\n");
#endif
  }
}

/**************************************************************************************
 * Name:  stm3210e_backlight
 *
 * Description:
 *   The LCD backlight is driven from PA8 which must be configured as TIM1
 *   CH1.  TIM1 must then be configured to output a clock on PA8; the duty
 *   of the clock determineds the backlight level.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_BACKLIGHT
static void stm3210e_backlight(void)
{
#ifdef CONFIG_LCD_PWM
  uint32_t prescaler;
  uint32_t reload;
  uint32_t timclk;
  uint16_t bdtr;
  uint16_t ccmr;
  uint16_t ccer;
  uint16_t cr2;

  /* Calculate the TIM1 prescaler value */

  prescaler = (STM32_PCLK2_FREQUENCY / CONFIG_LCD_PWMFREQUENCY + 65534) / 65535;
  if (prescaler < 1)
    {
      prescaler = 1;
    }
  else if (prescaler > 65536)
    {
      prescaler = 65536;
    }

  /* Calculate the TIM1 reload value */

  timclk = STM32_PCLK2_FREQUENCY / prescaler;
  reload = timclk / CONFIG_LCD_PWMFREQUENCY;

  if (reload < 1)
    {
      reload = 1;
    }
  else if (reload > 65535)
    {
      reload = 65535;
    }

  g_lcddev.reload = reload;

  /* Configure PA8 as TIM1 CH1 output */

  stm32_configgpio(GPIO_TIM1_CH1OUT);

  /* Enabled timer 1 clocking */

  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_TIM1EN);

  /* Reset timer 1 */

  modifyreg32(STM32_RCC_APB2RSTR, 0, RCC_APB2RSTR_TIM1RST);
  modifyreg32(STM32_RCC_APB2RSTR, RCC_APB2RSTR_TIM1RST, 0);

  /* Reset the Counter Mode and set the clock division */

  putreg16(0, STM32_TIM1_CR1);

  /* Set the Autoreload value */

  putreg16(reload-1, STM32_TIM1_ARR);

  /* Set the Prescaler value */

  putreg16(prescaler-1, STM32_TIM1_PSC);

  /* Generate an update event to reload the Prescaler value immediatly */

  putreg16(ATIM_EGR_UG, STM32_TIM1_EGR);

  /* Reset the Repetition Counter value */

  putreg16(0, STM32_TIM1_RCR);

  /* Set the main output enable (MOE) bit and clear the OSSI and OSSR
   * bits in the BDTR register.
   */

  bdtr  = getreg16(STM32_TIM1_BDTR);
  bdtr &= ~(ATIM_BDTR_OSSI | ATIM_BDTR_OSSR);
  bdtr |= ATIM_BDTR_MOE;
  putreg16(bdtr, STM32_TIM1_BDTR);

  /* Disable the Channel 1 */

  ccer  = getreg16(STM32_TIM1_CCER);
  ccer &= ~ATIM_CCER_CC1E;
  putreg16(ccer, STM32_TIM1_CCER);

  /* Get the TIM1 CR2 register value */

  cr2  = getreg16(STM32_TIM1_CR2);

  /* Select the Output Compare Mode Bits */

  ccmr  = getreg16(STM32_TIM1_CCMR1);
  ccmr &= ATIM_CCMR1_OC1M_MASK;
  ccmr |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC1M_SHIFT);
  ccmr |= (ATIM_CCMR_CCS_CCOUT << ATIM_CCMR1_CC1S_SHIFT);

  /* Set the power to the minimum value */

  g_lcddev.power = 0;
  putreg16(0, STM32_TIM1_CCR1);

  /* Select the output polarity level == LOW and enable */

  ccer |= (ATIM_CCER_CC1E );

  /* Reset the Output N Polarity level */

  ccer &= ~(ATIM_CCER_CC1NP|ATIM_CCER_CC1NE);

  /* Reset the Ouput Compare and Output Compare N IDLE State */

  cr2 &= ~(ATIM_CR2_OIS1|ATIM_CR2_OIS1N);

  /* Write the timer configuration */

  putreg16(cr2, STM32_TIM1_CR2);
  putreg16(ccmr, STM32_TIM1_CCMR1);
  putreg16(ccer, STM32_TIM1_CCER);

  /* Set the auto preload enable bit */

  modifyreg16(STM32_TIM1_CR1, 0, ATIM_CR1_ARPE);

  /* Enable Backlight Timer */

  ccer |= ATIM_CR1_CEN;
  putreg16(ccer, STM32_TIM1_CR1);

  /* Dump timer1 registers */

  lcddbg("APB2ENR: %08x\n", getreg32(STM32_RCC_APB2ENR));
  lcddbg("CR1:     %04x\n", getreg32(STM32_TIM1_CR1));
  lcddbg("CR2:     %04x\n", getreg32(STM32_TIM1_CR2));
  lcddbg("SMCR:    %04x\n", getreg32(STM32_TIM1_SMCR));
  lcddbg("DIER:    %04x\n", getreg32(STM32_TIM1_DIER));
  lcddbg("SR:      %04x\n", getreg32(STM32_TIM1_SR));
  lcddbg("BDTR:    %04x\n", getreg32(STM32_TIM1_BDTR));
  lcddbg("CCMR1:   %04x\n", getreg32(STM32_TIM1_CCMR1));
  lcddbg("CCMR2:   %04x\n", getreg32(STM32_TIM1_CCMR2));
  lcddbg("CCER:    %04x\n", getreg32(STM32_TIM1_CCER));
  lcddbg("CNT:     %04x\n", getreg32(STM32_TIM1_CNT));
  lcddbg("PSC:     %04x\n", getreg32(STM32_TIM1_PSC));
  lcddbg("ARR:     %04x\n", getreg32(STM32_TIM1_ARR));
  lcddbg("RCR:     %04x\n", getreg32(STM32_TIM1_RCR));
  lcddbg("CCR1:    %04x\n", getreg32(STM32_TIM1_CCR1));
  lcddbg("CCR2:    %04x\n", getreg32(STM32_TIM1_CCR2));
  lcddbg("CCR3:    %04x\n", getreg32(STM32_TIM1_CCR3));
  lcddbg("CCR4:    %04x\n", getreg32(STM32_TIM1_CCR4));
  lcddbg("DMAR:    %04x\n", getreg32(STM32_TIM1_DMAR));
#endif
}
#endif

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
#ifdef CONFIG_PM
  int ret;
#endif

  gvdbg("Initializing\n");

  /* Register to receive power management callbacks */

#ifdef CONFIG_PM
  ret = pm_register(&g_lcdcb);
  if (ret != OK)
  {
    lcddbg("ERROR: pm_register failed: %d\n", ret);
  }
#endif

  /* Configure GPIO pins and configure the FSMC to support the LCD */

  stm32_selectlcd();

  /* Configure and enable LCD */

  up_mdelay(50);
  stm3210e_lcdinitialize();

  /* Clear the display (setting it to the color 0=black) */

  stm3210e_lcdclear(0);

  /* Turn the backlight off */

  stm3210e_poweroff();
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
  stm3210e_poweroff();
  stm32_deselectlcd();
}

/**************************************************************************************
 * Name:  stm3210e_lcdclear
 *
 * Description:
 *   This is a non-standard LCD interface just for the STM3210E-EVAL board.  Because
 *   of the various rotations, clearing the display in the normal way by writing a
 *   sequences of runs that covers the entire display can be very slow.  Here the
 *   dispaly is cleared by simply setting all GRAM memory to the specified color.
 *
 **************************************************************************************/

void stm3210e_lcdclear(uint16_t color)
{
  uint32_t i = 0;

  stm3210e_setcursor(0, STM3210E_XRES-1);
  stm3210e_gramselect();
  for (i = 0; i < STM3210E_XRES * STM3210E_YRES; i++)
    {
      LCD->value = color;
    }
}

