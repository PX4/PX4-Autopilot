/************************************************************************************
 * configs/shenzhou/src/up_ssd1289.c
 * arch/arm/src/board/up_ssd1289.c
 *
 * This logic supports the connection of an SSD1289-based LCD to the Shenzhou IV
 * board.
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
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
 
/************************************************************************************
 * Included Files
 ************************************************************************************/

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

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_internal.h"
#include "shenzhou-internal.h"

#ifdef CONFIG_LCD_SSD1289

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/
/* Configuration ********************************************************************/

#ifndef CONFIG_SSD1289_WRONLY
#  warning "Only write access is supported; CONFIG_SSD1289_WRONLY should be defined"
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

/* Shenzhou LCD Hardware Definitions ************************************************/
/* Debug ****************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg         dbg
#  define lcdvdbg        vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/************************************************************************************
 * Private Type Definition
 ************************************************************************************/

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Helpers */

static void stm32_wrdata(uint16_t data);

/* Low Level LCD access */

static void stm32_select(FAR struct ssd1289_lcd_s *dev);
static void stm32_deselect(FAR struct ssd1289_lcd_s *dev);
static void stm32_index(FAR struct ssd1289_lcd_s *dev, uint8_t index);
#ifndef CONFIG_SSD1289_WRONLY
static uint16_t stm32_read(FAR struct ssd1289_lcd_s *dev);
#endif
static void stm32_write(FAR struct ssd1289_lcd_s *dev, uint16_t data);
static void stm32_backlight(FAR struct ssd1289_lcd_s *dev, int power);

/************************************************************************************
 * Private Data
 ************************************************************************************/
/* TFT LCD
 *
 * -- ---- -------------- -------------------------------------------------------------------
 * PN NAME SIGNAL         NOTES
 * -- ---- -------------- -------------------------------------------------------------------
 * 37 PB2  DATA_LE        To TFT LCD (CN13, ping 28)
 * 96 PB9  F_CS           To both the TFT LCD (CN13, pin 30) and to the W25X16 SPI FLASH
 * 34 PC5  TP_INT         JP6.  To TFT LCD (CN13) module (CN13, pin 26)
 * 65 PC8  LCD_CS         Active low: Pulled high (CN13, pin 19)
 * 66 PC9  TP_CS          Active low: Pulled high (CN13, pin 31)
 * 78 PC10 SPI3_SCK       To TFT LCD (CN13, pin 29)
 * 79 PC11 SPI3_MISO      To TFT LCD (CN13, pin 25)
 * 80 PC12 SPI3_MOSI      To TFT LCD (CN13, pin 27)
 * 58 PD11 SD_CS          Active low: Pulled high (See also TFT LCD CN13, pin 32)
 * 60 PD13 LCD_RS         To TFT LCD (CN13, pin 20)
 * 61 PD14 LCD_WR         To TFT LCD (CN13, pin 21)
 * 62 PD15 LCD_RD         To TFT LCD (CN13, pin 22)
 * 97 PE0  DB00           To TFT LCD (CN13, pin 3)
 * 98 PE1  DB01           To TFT LCD (CN13, pin 4)
 * 1  PE2  DB02           To TFT LCD (CN13, pin 5)
 * 2  PE3  DB03           To TFT LCD (CN13, pin 6)
 * 3  PE4  DB04           To TFT LCD (CN13, pin 7)
 * 4  PE5  DB05           To TFT LCD (CN13, pin 8)
 * 5  PE6  DB06           To TFT LCD (CN13, pin 9)
 * 38 PE7  DB07           To TFT LCD (CN13, pin 10)
 * 39 PE8  DB08           To TFT LCD (CN13, pin 11)
 * 40 PE9  DB09           To TFT LCD (CN13, pin 12)
 * 41 PE10 DB10           To TFT LCD (CN13, pin 13)
 * 42 PE11 DB11           To TFT LCD (CN13, pin 16)
 * 43 PE12 DB12           To TFT LCD (CN13, pin 15)
 * 44 PE13 DB13           To TFT LCD (CN13, pin 16)
 * 45 PE14 DB14           To TFT LCD (CN13, pin 17)
 * 46 PE15 DB15           To TFT LCD (CN13, pin 18)
 *
 * NOTE:  The backlight signl NC_BL (CN13, pin 24) is pulled high and not under
 * software control
 *
 * On LCD module:
 * -- -------------- -------------------------------------------------------------------
 * PN SIGNAL         NOTES
 * -- -------------- -------------------------------------------------------------------
 * 3  DB01           To LCD DB1
 * 4  DB00           To LCD DB0
 * 5  DB03           To LCD DB3
 * 6  DB02           To LCD DB2
 * 7  DB05           To LCD DB5
 * 8  DB04           To LCD DB4
 * 9  DB07           To LCD DB7
 * 10 DB06           To LCD DB6
 * 11 DB09           To LCD DB9
 * 12 DB08           To LCD DB8
 * 13 DB11           To LCD DB11
 * 14 DB10           To LCD DB10
 * 15 DB13           To LCD DB13
 * 16 DB12           To LCD DB12
 * 17 DB15           To LCD DB15
 * 18 DB14           To LCD DB14
 * 19 RS             To LCD RS
 * 20 /LCD_CS        To LCD CS
 * 21 /RD            To LCD RD
 * 22 /WR            To LCD WR
 * 23 BL_EN          (Not referenced)
 * 24 /RESET
 * 25 /INT           To Touch IC /INT
 * 26 MISO           To Touch IC DOUT; To AT45DB161B SO; To SD card DAT0
 * 27 LE             To 74HC573 that controls LCD 8-bit/16-bit mode
 * 28 MOSI           To Touch IC DIN; To AT45DB161B SI; To SD card CMD
 * 29 /DF_CS         To AT45DB161B Data Flash /CS
 * 30 SCLK           To Touch IC DCLK; To AT45DB161B SCK; To SD card CLK
 * 31 /SD_CS         To SD card /CS
 * 31 /TP_CS         To Touch IC CS
 */

/* LCD GPIO configurations */

static const uint32_t g_lcdconfig[] =
{
  GPIO_LCD_D0,  GPIO_LCD_D1,  GPIO_LCD_D2,  GPIO_LCD_D3,
  GPIO_LCD_D4,  GPIO_LCD_D5,  GPIO_LCD_D6,  GPIO_LCD_D7,
  GPIO_LCD_D8,  GPIO_LCD_D9,  GPIO_LCD_D10, GPIO_LCD_D11,
  GPIO_LCD_D12, GPIO_LCD_D13, GPIO_LCD_D14, GPIO_LCD_D15,

  GPIO_LCD_RS,  GPIO_LCD_CS,  GPIO_LCD_RD,  GPIO_LCD_WR,
  GPIO_LCD_LE,
};
#define NLCD_CONFIG (sizeof(g_lcdconfig)/sizeof(uint32_t))

/* This is the driver state structure (there is no retained state information) */

static struct ssd1289_lcd_s g_ssd1289 =
{
  .select    = stm32_select,
  .deselect  = stm32_deselect,
  .index     = stm32_index,
#ifndef CONFIG_SSD1289_WRONLY
  .read      = stm32_read,
#endif
  .write     = stm32_write,
  .backlight = stm32_backlight
};

/* The saved instance of the LCD driver */

static FAR struct lcd_dev_s *g_ssd1289drvr;

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_wrdata
 *
 * Description:
 *   Latch data on D0-D15 and toggle the WR line.
 *
 ************************************************************************************/

static void stm32_wrdata(uint16_t data)
{
  /* Latch the 16-bit LCD data and toggle the WR line */

  putreg32((uint32_t)data, LCD_DATA);
  putreg32(1, LCD_WR_CLEAR);
  putreg32(1, LCD_WR_SET);
}

/************************************************************************************
 * Name: stm32_select
 *
 * Description:
 *   Select the LCD device
 *
 ************************************************************************************/

static void stm32_select(FAR struct ssd1289_lcd_s *dev)
{
  /* Select the LCD by setting the LCD_CS low */

  putreg32(1, LCD_CS_CLEAR);
}

/************************************************************************************
 * Name: stm32_deselect
 *
 * Description:
 *   De-select the LCD device
 *
 ************************************************************************************/

static void stm32_deselect(FAR struct ssd1289_lcd_s *dev)
{
  /* De-select the LCD by setting the LCD_CS high */

  putreg32(1, LCD_CS_SET);
}

/************************************************************************************
 * Name: stm32_deselect
 *
 * Description:
 *   Set the index register
 *
 ************************************************************************************/

static void stm32_index(FAR struct ssd1289_lcd_s *dev, uint8_t index)
{
  /* Clear the RS signal */

  putreg32(1, LCD_RS_CLR);

  /* And write the index */

  stm32_wrdata((uint16_t)index);
}

/************************************************************************************
 * Name: stm32_read
 *
 * Description:
 *   Read LCD data (GRAM data or register contents)
 *
 ************************************************************************************/

#ifndef CONFIG_SSD1289_WRONLY
static uint16_t stm32_read(FAR struct ssd1289_lcd_s *dev)
{
#warning "Missing logic"
}
#endif

/************************************************************************************
 * Name: stm32_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 ************************************************************************************/

static void stm32_write(FAR struct ssd1289_lcd_s *dev, uint16_t data)
{
  /* Set the RS signal */

  putreg32(1, LCD_RS_CLR);

  /* And write the data */

  stm32_wrdata(data);
}

/************************************************************************************
 * Name: stm32_backlight
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 ************************************************************************************/

static void stm32_backlight(FAR struct ssd1289_lcd_s *dev, int power)
{
  /* There is no software control over the backlight */
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name:  up_lcdinitialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is fully
 *   initialized, display memory cleared, and the LCD ready to use, but with the power
 *   setting at 0 (full off).
 *
 ************************************************************************************/

int up_lcdinitialize(void)
{
  int i;

  /* Only initialize the driver once */

  if (!g_ssd1289drvr)
    {
      lcdvdbg("Initializing\n");

      /* Configure GPIO pins */

      for (i = 0; i < NLCD_CONFIG; i++)
        {
          stm32_configgpio(g_lcdconfig[i]);
        }

      /* Configure and enable the LCD */

      g_ssd1289drvr = ssd1289_lcdinitialize(&g_ssd1289);
      if (!g_ssd1289drvr)
        {
          lcddbg("ERROR: ssd1289_lcdinitialize failed\n");
          return -ENODEV;
        }
    }

  /* Turn the display off */

  g_ssd1289drvr->setpower(g_ssd1289drvr, 0);
  return OK;
}

/************************************************************************************
 * Name:  up_lcdgetdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This allows
 *   suport for multiple LCD devices.
 *
 ************************************************************************************/

FAR struct lcd_dev_s *up_lcdgetdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return g_ssd1289drvr;
}

/************************************************************************************
 * Name:  up_lcduninitialize
 *
 * Description:
 *   Unitialize the LCD support
 *
 ************************************************************************************/

void up_lcduninitialize(void)
{
  /* Turn the display off */

  g_ssd1289drvr->setpower(g_ssd1289drvr, 0);
}

#endif /* CONFIG_LCD_SSD1289 */
