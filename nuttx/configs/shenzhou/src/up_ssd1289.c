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

#ifndef CONFIG_STM32_FSMC
#  error "CONFIG_STM32_FSMC is required to use the LCD"
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
/* LCD /CS is CE1 ==  NOR/SRAM Bank 1
 * 
 * Bank 1 = 0x60000000 | 0x00000000
 * Bank 2 = 0x60000000 | 0x04000000
 * Bank 3 = 0x60000000 | 0x08000000
 * Bank 4 = 0x60000000 | 0x0c000000
 *
 * FSMC address bit 16 is used to distinguish command and data.  FSMC address bits
 * 0-24 correspond to ARM address bits 1-25. 
 */

#define STM32_LCDBASE ((uintptr_t)(0x60000000 | 0x00000000))
#define LCD_INDEX     (STM32_LCDBASE)
#define LCD_DATA      (STM32_LCDBASE + 0x00020000)

/* SRAM pin definitions */

#define LCD_NADDRLINES   1   /* A16 */
#define LCD_NDATALINES   16  /* D0-15 */

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
 */

#define GPIO_LCD_RESET (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|\
                        GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN6)

/* GPIO configurations unique to the LCD  */

static const uint32_t g_lcdconfig[] =
{
  /* PC6(RESET), FSMC_A16, FSMC_NOE, FSMC_NWE, and FSMC_NE1  */

  GPIO_LCD_RESET, GPIO_FSMC_A16, GPIO_FSMC_NOE, GPIO_FSMC_NWE, GPIO_FSMC_NE1
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
 * Name: stm32_select
 *
 * Description:
 *   Select the LCD device
 *
 ************************************************************************************/

static void stm32_select(FAR struct ssd1289_lcd_s *dev)
{
  /* Does not apply to this hardware */
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
  /* Does not apply to this hardware */
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
  putreg16((uint16_t)index, LCD_INDEX);
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
  return getreg16(LCD_DATA);
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
  putreg16((uint16_t)data, LCD_DATA);
}

/************************************************************************************
 * Name: stm32_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 ************************************************************************************/

static void stm32_backlight(FAR struct ssd1289_lcd_s *dev, int power)
{
#warning "Missing logic"
}

/************************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ************************************************************************************/

void stm32_selectlcd(void)
{
  /* Configure GPIO pins */

  stm32_extmemdata(LCD_NDATALINES);             /* Common data lines: D0-D15 */
  stm32_extmemgpios(g_lcdconfig, NLCD_CONFIG);  /* LCD-specific control lines */

  /* Enable AHB clocking to the FSMC */

  stm32_enablefsmc();

  /* Color LCD configuration (LCD configured as follow):
   * 
   *   - Data/Address MUX  = Disable   "FSMC_BCR_MUXEN" just not enable it.
   *   - Extended Mode     = Disable   "FSMC_BCR_EXTMOD"
   *   - Memory Type       = SRAM      "FSMC_BCR_SRAM"
   *   - Data Width        = 16bit     "FSMC_BCR_MWID16"
   *   - Write Operation   = Enable    "FSMC_BCR_WREN"
   *   - Asynchronous Wait = Disable
   */

  /* Bank1 NOR/SRAM control register configuration */

  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(FSMC_BTR_ADDSET(5) | FSMC_BTR_ADDHLD(0) | FSMC_BTR_DATAST(9) | FSMC_BTR_BUSTRUN(0) |
           FSMC_BTR_CLKDIV(0) | FSMC_BTR_DATLAT(0) | FSMC_BTR_ACCMODA, STM32_FSMC_BTR1);

  putreg32(0xffffffff, STM32_FSMC_BWTR1);

  /* Enable the bank by setting the MBKEN bit */

  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);
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
  /* Only initialize the driver once */

  if (!g_ssd1289drvr)
    {
      lcdvdbg("Initializing\n");

      /* Configure GPIO pins and configure the FSMC to support the LCD */

      stm32_selectlcd();

      /* Reset the LCD (active low) */

      stm32_gpiowrite(GPIO_LCD_RESET, false);
      up_mdelay(5);
      stm32_gpiowrite(GPIO_LCD_RESET, true);

      /* Configure and enable the LCD */

      up_mdelay(50);
      g_ssd1289drvr = ssd1289_lcdinitialize(&g_ssd1289);
      if (!g_ssd1289drvr)
        {
          lcddbg("ERROR: ssd1289_lcdinitialize failed\n");
          return -ENODEV;
        }
    }

  /* Clear the display (setting it to the color 0=black) */

#if 0 /* Already done in the driver */
  ssd1289_clear(g_ssd1289drvr, 0);
#endif

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
