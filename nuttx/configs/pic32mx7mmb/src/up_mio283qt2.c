/**************************************************************************************
 * configs/stm32fdiscover/src/up_mio283qt2.c
 * arch/arm/src/board/up_mio283qt2.c
 *
 * Interface definition for the MI0283QT-2 LCD from Multi-Inno Technology Co., Ltd.
 * This LCD is based on the Himax HX8347-D LCD controller.
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
 **************************************************************************************/
 
/**************************************************************************************
 * Included Files
 **************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/mio283qt2.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "pic32mx-internal.h"
#include "pic32mx-pmp.h"
#include "pic32mx7mmb_internal.h"

#ifdef CONFIG_LCD_MIO283QT2

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* Configuration **********************************************************************/

#ifndef CONFIG_PIC32MX_PMP
#  error "CONFIG_PIC32MX_PMP is required to use the LCD"
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

/* PIC32MX7MMB LCD Hardware Definitions ***********************************************/
/* --- ---------------------------------- -------------------- ------------------------
 * PIN CONFIGURATIONS                     SIGNAL NAME          ON-BOARD CONNECTIONS
 *     (Family Data Sheet Table 1-1)     (PIC32MX7 Schematic)
 * --- ---------------------------------- -------------------- ------------------------
 *   6 RC1/T2CK                           LCD_RST              TFT display
 *  43 PMA1/AETXD3/AN14/ERXD2/PMALH/RB14  LCD-CS#              TFT display, HDR2 pin 3
 *  77 OC3/RD2                            LCD_BLED             LCD backlight LED
 *  44 PMA0/AETXD2/AN15/CN12/ERXD3/OCFB/  LCD-RS               TFT display       
 *     PMALL/RB15
 *
 *  34 PMA13/AN10/RB10/CVREFOUT           LCD-YD               TFT display
 *  35 PMA12/AETXERR/AN11/ERXERR/RB11     LCD-XR               TFT display
 *  41 PMA11/AECRS/AN12/ERXD0/RB12        LCD-YU               TFT display
 *  42 PMA10/AECOL/AN13/ERXD1/RB13        LCD-XL               TFT display
 * 
 *  93 PMD0/RE0                           PMPD0                TFT display, HDR1 pin 18
 *  94 PMD1/RE1                           PMPD1                TFT display, HDR1 pin 17
 *  98 PMD2/RE2                           PMPD2                TFT display, HDR1 pin 16
 *  99 PMD3/RE3                           PMPD3                TFT display, HDR1 pin 15
 * 100 PMD4/RE4                           PMPD4                TFT display, HDR1 pin 14
 *   3 PMD5/RE5                           PMPD5                TFT display, HDR1 pin 13
 *   4 PMD6/RE6                           PMPD6                TFT display, HDR1 pin 12
 *   5 PMD7/RE7                           PMPD7                TFT display, HDR1 pin 11
 *  90 PMD8/C2RX/RG0                      PMPD8                TFT display, HDR1 pin 10
 *  89 PMD9/C2TX/ETXERR/RG1               PMPD9                TFT display, HDR1 pin 9
 *  88 PMD10/C1TX/ETXD0/RF1               PMPD10               TFT display, HDR1 pin 8
 *  87 PMD11/C1RX/ETXD1/RF0               PMPD11               TFT display, HDR1 pin 7
 *  79 PMD12/ETXD2/IC5/RD12               PMPD12               TFT display, HDR1 pin 6
 *  80 PMD13/CN19/ETXD3/RD13              PMPD13               TFT display, HDR1 pin 5
 *  83 PMD14/CN15/ETXEN/RD6               PMPD14               TFT display, HDR1 pin 4
 *  84 PMD15/CN16/ETXCLK/RD7              PMPD15               TFT display, HDR1 pin 3
 * 
 *  82 CN14/PMRD/RD5                      PMPRD                
 *  81 CN13/OC5/PMWR/RD4                  PMPWR                
 */

/* RC1, Reset -- Low value holds in reset */

#define GPIO_LCD_RST  (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTC|GPIO_PIN1)

/* RB14, LCD select -- Low value selects LCD */

#define GPIO_LCD_CS   (GPIO_OUTPUT|GPIO_VALUE_ONE|GPIO_PORTB|GPIO_PIN14)

/* RD2, Backlight -- Low value turns off */

#define GPIO_LCD_BLED (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTD|GPIO_PIN2)

/* RB15, RS -- High values selects data */

#define GPIO_LCD_RS   (GPIO_OUTPUT|GPIO_VALUE_ZERO|GPIO_PORTB|GPIO_PIN15)

/* Debug ******************************************************************************/

#ifdef CONFIG_DEBUG_LCD
#  define lcddbg       dbg
#  define lcdvdbg      vdbg
#else
#  define lcddbg(x...)
#  define lcdvdbg(x...)
#endif

/**************************************************************************************
 * Private Type Definition
 **************************************************************************************/

struct pic32mx7mmb_dev_s
{
  struct mio283qt2_lcd_s dev;      /* The externally visible part of the driver */
  bool                   data;     /* true=data selected */
  bool                   selected; /* true=LCD selected */
  bool                   reading;  /* true=We are in a read sequence */
  FAR struct lcd_dev_s  *drvr;     /* The saved instance of the LCD driver */
};

/**************************************************************************************
 * Private Function Protototypes
 **************************************************************************************/
/* Low Level LCD access */

static void pic32mx_select(FAR struct mio283qt2_lcd_s *dev);
static void pic32mx_deselect(FAR struct mio283qt2_lcd_s *dev);
static void pic32mx_index(FAR struct mio283qt2_lcd_s *dev, uint8_t index);
#ifndef CONFIG_MIO283QT2_WRONLY
static uint16_t pic32mx_read(FAR struct mio283qt2_lcd_s *dev);
#endif
static void pic32mx_write(FAR struct mio283qt2_lcd_s *dev, uint16_t data);
static void pic32mx_backlight(FAR struct mio283qt2_lcd_s *dev, int power);

/**************************************************************************************
 * Private Data
 **************************************************************************************/

/* This is the driver state structure (there is no retained state information) */

static struct pic32mx7mmb_dev_s g_pic32mx7mmb_lcd =
{
  {
    .select    = pic32mx_select,
    .deselect  = pic32mx_deselect,
    .index     = pic32mx_index,
#ifndef CONFIG_MIO283QT2_WRONLY
    .read      = pic32mx_read,
#endif
    .write     = pic32mx_write,
    .backlight = pic32mx_backlight
  }
};

/**************************************************************************************
 * Private Functions
 **************************************************************************************/

/**************************************************************************************
 * Name: pic32mx_command
 *
 * Description:
 *   Configure to write an LCD command
 *
 **************************************************************************************/

static void pic32mx_command(FAR struct pic32mx7mmb_dev_s *priv)
{
  /* Low selects command */

  if (priv->data)
    {
      pic32mx_gpiowrite(GPIO_LCD_RS, false);

      priv->data    = false;  /* Command, not data */
      priv->reading = false;  /* No read sequence in progress */
    }
}

/**************************************************************************************
 * Name: pic32mx_data
 *
 * Description:
 *   Configure to read or write LCD data
 *
 **************************************************************************************/

static void pic32mx_data(FAR struct pic32mx7mmb_dev_s *priv)
{
  /* Hi selects data */

  if (!priv->data)
    {
      pic32mx_gpiowrite(GPIO_LCD_RS, true);

      priv->data    = true;   /* Data, not command */
      priv->reading = false;  /* No read sequence in progress */
    }
}

/**************************************************************************************
 * Name: pic32mx_data
 *
 * Description:
 *   Wait until the PMP is no longer busy
 *
 **************************************************************************************/

static void pic32mx_busywait(void)
{
  while ((getreg32(PIC32MX_PMP_MODE) & PMP_MODE_BUSY) != 0);
}

/**************************************************************************************
 * Name: pic32mx_select
 *
 * Description:
 *   Select the LCD device
 *
 **************************************************************************************/

static void pic32mx_select(FAR struct mio283qt2_lcd_s *dev)
{
  FAR struct pic32mx7mmb_dev_s *priv = (FAR struct pic32mx7mmb_dev_s *)dev;

  /* CS low selects */

  if (!priv->selected)
    {
      pic32mx_gpiowrite(GPIO_LCD_CS, false);

      priv->selected = true;  /* LCD selected */
      priv->reading  = false; /* No read sequence in progress */
    }
}

/**************************************************************************************
 * Name: pic32mx_deselect
 *
 * Description:
 *   De-select the LCD device
 *
 **************************************************************************************/

static void pic32mx_deselect(FAR struct mio283qt2_lcd_s *dev)
{
  FAR struct pic32mx7mmb_dev_s *priv = (FAR struct pic32mx7mmb_dev_s *)dev;

  /* CS high de-selects */

  if (priv->selected)
    {
      pic32mx_gpiowrite(GPIO_LCD_CS, true);

      priv->selected = false; /* LCD not selected */
      priv->reading  = false; /* No read sequence in progress */
    }
}

/**************************************************************************************
 * Name: pic32mx_index
 *
 * Description:
 *   Set the index register
 *
 **************************************************************************************/

static void pic32mx_index(FAR struct mio283qt2_lcd_s *dev, uint8_t index)
{
  FAR struct pic32mx7mmb_dev_s *priv = (FAR struct pic32mx7mmb_dev_s *)dev;

  /* Make sure that the PMP is not busy from the last transaction.  Read data is not
   * available until the busy bit becomes zero.
   */

  pic32mx_busywait();

  /* Write the 8-bit command (on the 16-bit data bus) */

  pic32mx_command(priv);
  putreg16((uint16_t)index, PIC32MX_PMP_DIN);
}

/**************************************************************************************
 * Name: pic32mx_read
 *
 * Description:
 *   Read LCD data (GRAM data or register contents)
 *
 **************************************************************************************/

#ifndef CONFIG_MIO283QT2_WRONLY
static uint16_t pic32mx_read(FAR struct mio283qt2_lcd_s *dev)
{
  FAR struct pic32mx7mmb_dev_s *priv = (FAR struct pic32mx7mmb_dev_s *)dev;
  uint16_t data;

  /* Make sure that the PMP is not busy from the last transaction.  Read data is not
   * available until the busy bit becomes zero.
   */

  pic32mx_busywait();

  /* Read 16-bits of data */

  pic32mx_data(priv);
  data = getreg16(PIC32MX_PMP_DIN);

  /* We need to discard the first 16-bits of data that we read and re-read inorder
   * to get valid data (that is just the way that the PMP works).
   */

  if (!priv->reading)
    {
      data = getreg16(PIC32MX_PMP_DIN);
    }

  return data;
}
#endif

/**************************************************************************************
 * Name: pic32mx_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 **************************************************************************************/

static void pic32mx_write(FAR struct mio283qt2_lcd_s *dev, uint16_t data)
{
  FAR struct pic32mx7mmb_dev_s *priv = (FAR struct pic32mx7mmb_dev_s *)dev;

  /* Make sure that the PMP is not busy from the last transaction */

  pic32mx_busywait();

  /* Write 16-bits of data */

  pic32mx_data(priv);
  putreg16(data, PIC32MX_PMP_DIN);

  /* We are not in a write sequence */

  priv->reading = false;
}

/**************************************************************************************
 * Name: pic32mx_write
 *
 * Description:
 *   Write LCD data (GRAM data or register contents)
 *
 **************************************************************************************/

static void pic32mx_backlight(FAR struct mio283qt2_lcd_s *dev, int power)
{
  /* For now, we just control the backlight as a discrete.  Pulse width modulation
   * would be required to vary the backlight level.  A low value turns the backlight
   * off.
   */

  pic32mx_gpiowrite(GPIO_LCD_BLED, power > 0);
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
  uint32_t regval;

  /* Only initialize the driver once.  NOTE: The LCD GPIOs were already configured
   * by pic32mx_lcdinitialize.
   */

  if (!g_pic32mx7mmb_lcd.drvr)
    {
      lcdvdbg("Initializing\n");

      /* Hold the LCD in reset (active low)  */

      pic32mx_gpiowrite(GPIO_LCD_RST, false);

      /* Configure PMP to support the LCD */

      putreg32(0, PIC32MX_PMP_MODE);
      putreg32(0, PIC32MX_PMP_AEN);
      putreg32(0, PIC32MX_PMP_CON);

      /* Set LCD timing values, PMP master mode 2, 16-bit mode, no address
       * increment, and no interrupts.
       */

      regval = (PMP_MODE_WAITE_RD(0) | PMP_MODE_WAITM(3) | PMP_MODE_WAITB_1TPB |
                PMP_MODE_MODE_MODE2 | PMP_MODE_MODE16 | PMP_MODE_INCM_NONE |
                PMP_MODE_IRQM_NONE);
      putreg32(regval, PIC32MX_PMP_MODE);

      /* Enable the PMP  for reading and writing */

      regval = (PMP_CON_CSF_ADDR1415 | PMP_CON_PTRDEN | PMP_CON_PTWREN |
                PMP_CON_ADRMUX_NONE | PMP_CON_ON);
      putreg32(regval, PIC32MX_PMP_CON);

      /* Bring the LCD out of reset */

      up_mdelay(5);
      pic32mx_gpiowrite(GPIO_LCD_RST, true);

      /* Configure and enable the LCD */

      up_mdelay(50);
      g_pic32mx7mmb_lcd.drvr = mio283qt2_lcdinitialize(&g_pic32mx7mmb_lcd.dev);
      if (!g_pic32mx7mmb_lcd.drvr)
        {
          lcddbg("ERROR: mio283qt2_lcdinitialize failed\n");
          return -ENODEV;
        }
    }

  /* Clear the display (setting it to the color 0=black) */

#if 0 /* Already done in the driver */
  mio283qt2_clear(g_pic32mx7mmb_lcd.drvr, 0);
#endif

  /* Turn the display off */

  g_pic32mx7mmb_lcd.drvr->setpower(g_pic32mx7mmb_lcd.drvr, 0);
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
  return g_pic32mx7mmb_lcd.drvr;
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
  /* Turn the display off */

  g_pic32mx7mmb_lcd.drvr->setpower(g_pic32mx7mmb_lcd.drvr, 0);
}

#endif /* CONFIG_LCD_MIO283QT2 */

/****************************************************************************
 * Name: pic32mx_lcdinitialize
 *
 * Description:
 *   Initialize the LCD.  This function should be called early in the boot
 *   sequendce -- Even if the LCD is not enabled.  In that case we should
 *   at a minimum at least disable the LCD backlight.
 *
 ****************************************************************************/

void pic32mx_lcdinitialize(void)
{
  /* Configure all LCD discrete controls.  LCD will be left in this state:
   * 1. Held in reset,
   * 2. Not selected,
   * 3. Backlight off,
   * 4. Command selected.
   */

#ifdef CONFIG_LCD_MIO283QT2
   pic32mx_configgpio(GPIO_LCD_RST);
   pic32mx_configgpio(GPIO_LCD_CS);
   pic32mx_configgpio(GPIO_LCD_BLED);
   pic32mx_configgpio(GPIO_LCD_RS);

#else
  /* Just configure the backlight control as an output and turn off the
   * backlight for now.
   */

   pic32mx_configgpio(GPIO_LCD_BLED);
#endif
}


