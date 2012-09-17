/************************************************************************************
 * configs/hymini-stm32v/src/ssd1289.c
 * arch/arm/src/board/ssd1289.c
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Laurent Latil <laurent@latil.nom.fr>
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

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>

#include <arch/board/board.h>

#include "up_arch.h"
#include "stm32.h"
#include "stm32_internal.h"
#include "hymini_stm32v-internal.h"

#include "ssd1289.h"

/* Color depth and format */
#define LCD_BPP          16
#define LCD_COLORFMT     FB_FMT_RGB16_565

/* Display Resolution */
#if defined(CONFIG_LCD_LANDSCAPE)
#  define LCD_XRES       320
#  define LCD_YRES       240
#else
#  define LCD_XRES       240
#  define LCD_YRES       320
#endif

#define LCD_BL_TIMER_PERIOD   8999


/* Debug ******************************************************************************/
#ifdef CONFIG_DEBUG_LCD
# define lcddbg(format, arg...)  vdbg(format, ##arg)
#else
# define lcddbg(x...)
#endif

/* LCD is connected to the FSMC_Bank1_NOR/SRAM1 and NE1 is used as ship select signal */
/* RS <==> A16 */
#define LCD_REG              (*((volatile unsigned short *) 0x60000000)) /* RS = 0 */
#define LCD_RAM              (*((volatile unsigned short *) 0x60020000)) /* RS = 1 */

const uint16_t fsmc_gpios[] =
{ /* A16... A24 */
  GPIO_NPS_A16, GPIO_NPS_A17, GPIO_NPS_A18, GPIO_NPS_A19, GPIO_NPS_A20,
  GPIO_NPS_A21, GPIO_NPS_A22, GPIO_NPS_A23,

  /* D0... D15 */GPIO_NPS_D0, GPIO_NPS_D1, GPIO_NPS_D2, GPIO_NPS_D3,
  GPIO_NPS_D4, GPIO_NPS_D5, GPIO_NPS_D6, GPIO_NPS_D7, GPIO_NPS_D8, GPIO_NPS_D9,
  GPIO_NPS_D10, GPIO_NPS_D11, GPIO_NPS_D12, GPIO_NPS_D13, GPIO_NPS_D14,
  GPIO_NPS_D15,

  /* NOE, NWE  */GPIO_NPS_NOE, GPIO_NPS_NWE,

  /* NE1  */GPIO_NPS_NE1 };

#define NGPIOS (sizeof(fsmc_gpios)/sizeof(uint16_t))

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

/* Low Level methods */
void lcd_clear(uint16_t color);

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

#ifdef CONFIG_LCD_BACKLIGHT
static void lcd_backlight(void);
#else
#  define lcd_backlight()
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

static uint16_t g_runbuffer[LCD_XRES];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{ .fmt = LCD_COLORFMT, /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres = LCD_XRES, /* Horizontal resolution in pixel columns */
  .yres = LCD_YRES, /* Vertical resolution in pixel rows */
  .nplanes = 1, /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{ .putrun = lcd_putrun, /* Put a run into LCD memory */
  .getrun = lcd_getrun, /* Get a run from LCD memory */
  .buffer = (uint8_t*) g_runbuffer, /* Run scratch buffer */
  .bpp = LCD_BPP, /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct ssd1289_dev_s g_lcddev =
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
    .getcontrast = lcd_getcontrast,
    .setcontrast = lcd_setcontrast,
  },
  .power=0
};

/************************************************************************************
 * Name: stm32_extmemgpios
 *
 * Description:
 *   Initialize GPIOs for NOR or SRAM
 *
 ************************************************************************************/

static inline void stm32_extmemgpios(const uint16_t *gpios, int ngpios)
{
  int i;

  /* Configure GPIOs */
  for (i = 0; i < ngpios; i++)
  {
    stm32_configgpio(gpios[i]);
  }
}

/************************************************************************************
 * Name: stm32_enablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

#ifndef CONFIG_STM32_FSMC
#  error CONFIG_STM32_FSMC is required for LCD support
#endif

static void stm32_enablefsmc(void)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHBENR);
  regval |= RCC_AHBENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/************************************************************************************
 * Name: stm32_disablefsmc
 *
 * Description:
 *  enable clocking to the FSMC module
 *
 ************************************************************************************/

static void stm32_disablefsmc(void)
{
  uint32_t regval;

  /* Enable AHB clocking to the FSMC */

  regval  = getreg32( STM32_RCC_AHBENR);
  regval &= ~RCC_AHBENR_FSMCEN;
  putreg32(regval, STM32_RCC_AHBENR);
}

/************************************************************************************
 * Name: stm32_selectlcd
 *
 * Description:
 *   Initialize to the LCD
 *
 ************************************************************************************/

static void stm32_selectlcd(void)
{
  /* Configure new GPIO state */
  stm32_extmemgpios(fsmc_gpios, NGPIOS);

  /* Enable AHB clocking to the FSMC */
  stm32_enablefsmc();

  /* Bank1 NOR/SRAM control register configuration */
  putreg32(FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN, STM32_FSMC_BCR1);

  /* Bank1 NOR/SRAM timing register configuration */
  putreg32(
      FSMC_BTR_ADDSET(2)|FSMC_BTR_ADDHLD(0)|FSMC_BTR_DATAST(2)|FSMC_BTR_BUSTRUN(0)| FSMC_BTR_CLKDIV(0)|FSMC_BTR_DATLAT(0)|FSMC_BTR_ACCMODA,
      STM32_FSMC_BTR1);

  /* As ext mode is not active the write timing is ignored!! */
  putreg32(0xffffffff, STM32_FSMC_BWTR1);

  /* Enable the bank by setting the MBKEN bit */
  putreg32(FSMC_BCR_MBKEN | FSMC_BCR_SRAM | FSMC_BCR_MWID16 | FSMC_BCR_WREN,
      STM32_FSMC_BCR1);
}

/************************************************************************************
 * Name: stm32_deselectlcd
 *
 * Description:
 *   Disable the LCD
 *
 ************************************************************************************/
// FIXME: Check this code !!
void stm32_deselectlcd(void)
{
  /* Restore registers to their power up settings */

  putreg32(0xffffffff, STM32_FSMC_BCR4);

  /* Bank1 NOR/SRAM timing register configuration */

  putreg32(0x0fffffff, STM32_FSMC_BTR4);

  /* Disable AHB clocking to the FSMC */

  stm32_disablefsmc();
}

lcd_inline void write_cmd(unsigned short cmd)
{
  LCD_REG = cmd;
}

lcd_inline unsigned short read_data(void)
{
  return LCD_RAM;
}

lcd_inline void write_data(unsigned short data_code)
{
  LCD_RAM = data_code;
}

void write_reg(unsigned char reg_addr, unsigned short reg_val)
{
  write_cmd(reg_addr);
  write_data(reg_val);
}

unsigned short read_reg(unsigned char reg_addr)
{
  unsigned short val;
  write_cmd(reg_addr);
  val = read_data();
  return (val);
}

static inline void lcd_gramselect(void)
{
  write_cmd(0x22);
}

void lcd_setcursor(unsigned int x, unsigned int y)
{
#if defined(CONFIG_LCD_PORTRAIT) || defined (CONFIG_LCD_RPORTRAIT)
# if defined (CONFIG_LCD_RPORTRAIT)
  x = (LCD_XRES - 1) - x;
  y = (LCD_YRES - 1) - y;
# endif
  write_reg(0x4e, x);
  write_reg(0x4f, y);
#endif

#if defined(CONFIG_LCD_LANDSCAPE)
  y = (LCD_YRES - 1) - y;

  write_reg(0x4e, y);
  write_reg(0x4f, x);
#endif
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
  lcd_setcursor(col, row);

  lcd_gramselect();
  for (i = 0; i < npixels; i++)
  {
    write_data(*src++);
  }
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
  FAR uint16_t *dest = (FAR uint16_t*) buffer;
  int i;

  /* Buffer must be provided and aligned to a 16-bit address boundary */
  DEBUGASSERT(buffer && ((uintptr_t)buffer & 1) == 0);

  /* Read the run from GRAM. */
  lcd_setcursor(col, row);

  lcd_gramselect();

  /* dummy read */
  (void)read_data();

  for (i = 0; i < npixels; i++)
  {
    *dest++ = read_data();
  }
  return OK;
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
  if (g_lcddev.power == power) {
    return OK;
  }

  gvdbg("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  if (power > 0)
    {
#ifdef CONFIG_LCD_BACKLIGHT
      uint32_t duty;

      /* Calculate the new backlight duty.  It is a fraction of the timer
       * period based on the ration of the current power setting to the
       * maximum power setting.
       */
      duty = ((uint32_t)LCD_BL_TIMER_PERIOD * (uint32_t)power) / CONFIG_LCD_MAXPOWER;
      if (duty >= LCD_BL_TIMER_PERIOD)
        {
          duty = LCD_BL_TIMER_PERIOD - 1;
        }
      gvdbg("PWM duty: %d\n", duty);
      putreg16((uint16_t)duty, STM32_TIM3_CCR2);
#endif
      /* TODO turn the display on */
    }
  else
    {
      /* FIXME: Turn display off ? */
      gvdbg("Force PWM to 0\n");
      putreg16((uint16_t)0, STM32_TIM3_CCR2);
    }
  g_lcddev.power = power;
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
  /* Display control (GON=1,DTE=0,D[1:0]=01)
     D[1:0]=01 - Internal display is performed, display is off.
  */
  write_reg(0x07, 0x0021);

  /* Oscillator (OSCEN=1) */
  write_reg(0x00, 0x0001);

  /* Display control (GON=1,DTE=0,D[1:0]=11)
     D[1:0]=11 - Internal display is performed, display is on.
  */
  write_reg(0x07, 0x0023);

  /* Wait 30ms */
  up_mdelay(30);

  /* Display control (GON=1,DTE=1,D[1:0]=11) */
  write_reg(0x07, 0x0033);

  /* Power control 1
   DCT3 DCT2 DCT1 DCT0    BT2 BT1 BT0 -     DC3 DC2 DC1 DC0   AP2 AP1 AP0 -
   1     0    1    0       1   0   0  0      1   0   1   0     0   1   0  0
   DCT[3:0] - 1010: fosc/4  (step-up cycle of the step-up circuit for 8-color mode)
   BT[2:0]  - 100 (step-up factor of the step-up circuit)
   DC[3:0]  - 1010: fosc/4  (step-up cycle of the step-up circuit for 262k-color mode)
   AP[2:0]  - 010: Small to medium (amount of current from the stable-current source in internal operational amplifier circuit)
  */
  write_reg(0x03, 0xA8A4); /* or 0x0804 ?? */

  /* Power Control 2
     VRC[2:0] - 000: 5.1v (VCIX2 output voltage)
   */
  write_reg(0x0C, 0x0000);

  /* Power Control 3
     - - - -   - - - -    - - - -   VRH3 VRH2 VRH1 VRH0
     0 0 0 0   0 0 0 0    0 0 0 0    1    0    0    0
     VRH[3:0] - 1000: Vref x 2.165  (Set amplitude magnification of VLCD63)
   */
  write_reg(0x0D, 0x0008); // or 0x080C ??

  /* power control 4
     - - VCOMG VDV4  VDV3 VDV2 VDV1 VDV0  - - - -   - - - -
     0 0   1    0     1    0    0    1    0 0 0 0   0 0 0 0
     VDV[4:0] - 01001:  VLCD63 x 0.xx (Vcom amplitude)
     VCOMG - 1 (enable output voltage of VcomL - VDV[4:0])
   */
  write_reg(0x0E, 0x2900);

  /* Power Control 5
    - - - -   - - - -   nOTP - VCM5 VCM4   VCM3 VCM2 VCM1 VCM0
    0 0 0 0   0 0 0 0     1  0  1    1      1    0    0    0
    nOTP: 1  - setting of VCM[5:0] becomes valid and voltage of VcomH can be adjusted.
    VCM[5-0] - 111000:  VLCD63 x 0.xx  (amplify the VcomH voltage 0.35 to 0.99 times the VLCD63 voltage)
  */
  write_reg(0x1E, 0x00B8);

  /* Driver Output Control
     - RL REV CAD   BGR SM TB MUX8   MUX7 MUX6 MUX5 MUX4   MUX3 MUX2 MUX1 MUX0
     0 0   1   0     1  0  1   1      0    0    1    1      1    1    1    1
     RL  - 0  RL setting is ignored when display with RAM  (Dmode[1:0] = 00).
     REV - 1  Reverse source output level
     CAD - 0  Cs on Common
     BGR - 1  BGR  (Selects the order from RGB to BGR in writing 18-bit pixel data in the GDDRAM)
     SM  - 0  (Gate scan sequence)
     TB  - 1  (output shift direction of the gate driver)
     MUX[8:0] - 100111111 (319)  (number of lines for the LCD driver)
  */
  write_reg(0x01, 0x2B3F);

  /* LCD-Driving-Waveform Control
     - - - FLD   ENWS B/C EOR WSMD   NW7 NW6 NW5 NW4   NW3 NW2 NW1 NW0
     0 0 0  0     0    1   1   0      0   0   0   0     0   0   0   0
     FLD - 0  ?
     ENWS- 0  (disable WSYNC output pin  -> HiZ)
     B/C - 1  (When B/C = 1, a N-line inversion waveform is generated and alternates in a N-line equals to NW[7:0]+1)
     EOR - 1
     WSMD- 0
     NW[7:0] - 0  Specify the number of lines that will alternate at the N-line inversion setting (if B/C = 1).
  */
  /* Set FLD ? */
  write_reg(0x02, 0x0600);

  /* Exit sleep mode */
  write_reg(0x10, 0x0000);

  /*  Entry Mode
     VSMode DFM1 DFM0 TRANS   OEDef WMode DMode1 DMode0   TY1 TY0 ID1 ID0    AM LG2 LG1 LG0
       0     1    1     0      0      0     0      0       0   1   1   1     0   0   0   0
     VSMode - 0   When VSMode = 1 at DMode[1:0] = “00”, the frame frequency will be dependent on VSYNC.
     DFM[1:0] - 11 (65k color mode selected)
     TRANS - 0  (if TRANS = 1, transparent display is allowed during DMode[1:0] = “1x”)
     0EDef - 0  (display window is defined by R4Eh and R4Fh)
     WMode - 0  (Normal data bus)
     DMode[1:0] - 00  (data display from RAM data)
     TY[1:0] - 01  (type B.  Used for 262k color mode only)
     ID[1:0] - 11  (address counter auto incremented by 1, horizontal & vertical)
     AM - 0  Horizontal (direction of the address counter update after data are written to the GDDRAM)
     LG[2:0] - 000  (operation to perform according to GDRAM compare registers ?)
  */
#if defined(CONFIG_LCD_PORTRAIT)
  write_reg(0x11, 0x6070);
#elif defined(CONFIG_LCD_RPORTRAIT)
  /* ID[1:0] - 00  (address counter auto decremented by 1, horizontal & vertical) */
  write_reg(0x11, 0x6840);
#elif defined(CONFIG_LCD_LANDSCAPE)
  /* ID[1:0] - 10  (address counter auto decremented by 1, horizontal & vertical)
     AM - 1   Vertical direction
  */
  write_reg(0x11, 0x6068);
#else
#error "LCD orientation not supported"
#endif

  /* Compare register  (unused ?) */
  write_reg(0x05, 0x0000);
  write_reg(0x06, 0x0000);

  /* Horizontal Porch
     XL7 XL6 XL5 XL4 XL3 XL2 IB9 XL1   HBP7 HBP6 HBP5 HBP4 HBP3 HBP2 HBP1 HBP0
     XL[7:0] - 0xEF (239)  (Number of valid pixel per line is equal to XL[7:0] + 1)
     HBP[7:0] - 0x1C (28)  30 dot clocks  (delay period from falling edge of HSYNC signal to first valid data)
  */
  write_reg(0x16, 0xEF1C);

  /* Vertical Porch */
  write_reg(0x17, 0x0003);

  /* Display control
     - - - PT1    PT0 VLE2 VLE1 SPT    - - GON DTE    CM - D1 D0
     0 0 0 0       0   0    0    1     0 0  1   1     0  0 1  1
     VLE[2:1]: 00  (When VLE1 = 1 or VLE2 = 1, vertical scroll performed in the 1st screen by taking data VL17-0 in R41h
               register. When VLE1 = 1 and VLE2 = 1, a vertical scroll is performed in the 1st and 2nd screen by VL1[8:0] and VL2[8:0])
     SPT: 1  (When SPT = “1”, the 2-division LCD drive is performed.
     CM:  0  (8-color mode setting  0=disabled,  1=enabled)
     D[1:0]: 11
  */
  write_reg(0x07, 0x0133);

  /* Frame Cycle Control */
  write_reg(0x0B, 0x0000);

  /* Gate Scan Position */
  /* write_reg(0x0F, 0x0000); // Default value */

  /* Vertical Scroll Control */
  write_reg(0x41, 0x0000);
  write_reg(0x42, 0x0000);

  /* 1st Screen driving position
     driving start position for the first screen in a line unit. (0)
  */
  write_reg(0x48, 0x0000);
  /* driving end position for the first screen in a line unit. (319) */
  write_reg(0x49, 0x013F);

  /* 2nd Screen driving position  (unused) */
  write_reg(0x4A, 0x0000);
  write_reg(0x4B, 0x0000);

  /* start/end positions of the window address in the horizontal direction */
  write_reg(0x44, 0xEF00);

  /* start/end positions of the window address in the vertical direction */
  /*  write_reg(0x45, 0x0000);  // Default value
      write_reg(0x46, 0x013F);  // Default value
  */

  /* Gamma Control */
  write_reg(0x0030, 0x0707);
  write_reg(0x0031, 0x0204);
  write_reg(0x0032, 0x0204);
  write_reg(0x0033, 0x0502);
  write_reg(0x0034, 0x0507);
  write_reg(0x0035, 0x0204);
  write_reg(0x0036, 0x0204);
  write_reg(0x0037, 0x0502);
  write_reg(0x003A, 0x0302);
  write_reg(0x003B, 0x0302);

  /* RAM write data mask  (unused)
      write_reg(0x23, 0x0000);  // Default value
      write_reg(0x24, 0x0000);  // Default value
  */

  /* Setting R28h as 0x0006 is required before setting R25h and R29h registers.*/

  write_reg(0x28, 0x0006);

  /* Frame Frequency Control (R25h)
     OSC3 OSC2 OSC1 OSC0  - - - -   - - - -   - - - -
       1    1    1    0   0 0 0 0   0 0 0 0   0 0 0 0
     OSC[3:0]:  1110   Corresponding frame freq: 80Hz
  */
  write_reg(0x25, 0xE000);  /* (Default value = 8000h) */
}

/**************************************************************************************
 * Name:  lcd_backlight
 *
 * Description:
 *   The LCD backlight is driven from PB.5 which must be configured as TIM3
 *   CH2.  TIM3 must then be configured to pwm output on PB.5; the duty
 *   of the clock determines the backlight level.
 *
 **************************************************************************************/

#ifdef CONFIG_LCD_BACKLIGHT

#ifndef CONFIG_STM32_TIM3_PARTIAL_REMAP
#  error CONFIG_STM32_TIM3_PARTIAL_REMAP must be set (to have TIM3 CH2 on pin B.5)
#endif

static void lcd_backlight(void)
{
  uint16_t ccmr;
  uint16_t ccer;
  uint16_t cr2;

  /* Configure PB5 as TIM3 CH2 output */
  stm32_configgpio(GPIO_TIM3_CH2OUT);

  /* Enabled timer 3 clocking */
  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM3EN);

  /* Reset timer 3 */

  modifyreg32(STM32_RCC_APB1RSTR, 0, RCC_APB1RSTR_TIM3RST);
  modifyreg32(STM32_RCC_APB1RSTR, RCC_APB1RSTR_TIM3RST, 0);

  /* Reset the Counter Mode and set the clock division */
  putreg16(0, STM32_TIM3_CR1);

  /* Set the Autoreload value */
  putreg16(LCD_BL_TIMER_PERIOD, STM32_TIM3_ARR);

  /* Set the Prescaler value */

  putreg16(0, STM32_TIM3_PSC);

  /* Generate an update event to reload the Prescaler value immediatly */

  putreg16(ATIM_EGR_UG, STM32_TIM3_EGR);

  /* Disable the Channel 2 */

  ccer  = getreg16(STM32_TIM3_CCER);
  ccer &= ~ATIM_CCER_CC2E;
  putreg16(ccer, STM32_TIM3_CCER);

  /* Get the TIM3 CR2 register value */
  cr2  = getreg16(STM32_TIM3_CR2);

  /* Select the Output Compare Mode Bits */

  ccmr  = getreg16(STM32_TIM3_CCMR1);
  ccmr &= ATIM_CCMR1_OC2M_MASK;
  ccmr |= (ATIM_CCMR_MODE_PWM1 << ATIM_CCMR1_OC2M_SHIFT);

  /* Set the capture compare register value (50% duty) */
  // FIXME should be set to 0  (appl needs to call setpower to change it)
  g_lcddev.power = (CONFIG_LCD_MAXPOWER + 1) / 2;
  putreg16((LCD_BL_TIMER_PERIOD + 1) / 2, STM32_TIM3_CCR2);

  /* Select the output polarity level == HIGH */
  ccer &= !ATIM_CCER_CC2P;

  /* Enable channel 2*/
  ccer |= ATIM_CCER_CC2E;

  /* Write the timer configuration */

  putreg16(ccmr, STM32_TIM3_CCMR1);
  putreg16(ccer, STM32_TIM3_CCER);

  /* Set the auto preload enable bit */

  modifyreg16(STM32_TIM3_CR1, 0, ATIM_CR1_ARPE);

  /* Enable Backlight Timer !!!!*/
  modifyreg16(STM32_TIM3_CR1, 0, ATIM_CR1_CEN);

  /* Dump timer3 registers */
  lcddbg("APB1ENR: %08x\n", getreg32(STM32_RCC_APB1ENR));
  lcddbg("CR1:     %04x\n", getreg32(STM32_TIM3_CR1));
  lcddbg("CR2:     %04x\n", getreg32(STM32_TIM3_CR2));
  lcddbg("SMCR:    %04x\n", getreg32(STM32_TIM3_SMCR));
  lcddbg("DIER:    %04x\n", getreg32(STM32_TIM3_DIER));
  lcddbg("SR:      %04x\n", getreg32(STM32_TIM3_SR));
  lcddbg("EGR:     %04x\n", getreg32(STM32_TIM3_EGR));
  lcddbg("CCMR1:   %04x\n", getreg32(STM32_TIM3_CCMR1));
  lcddbg("CCMR2:   %04x\n", getreg32(STM32_TIM3_CCMR2));
  lcddbg("CCER:    %04x\n", getreg32(STM32_TIM3_CCER));
  lcddbg("CNT:     %04x\n", getreg32(STM32_TIM3_CNT));
  lcddbg("PSC:     %04x\n", getreg32(STM32_TIM3_PSC));
  lcddbg("ARR:     %04x\n", getreg32(STM32_TIM3_ARR));
  lcddbg("CCR1:    %04x\n", getreg32(STM32_TIM3_CCR1));
  lcddbg("CCR2:    %04x\n", getreg32(STM32_TIM3_CCR2));
  lcddbg("CCR3:    %04x\n", getreg32(STM32_TIM3_CCR3));
  lcddbg("CCR4:    %04x\n", getreg32(STM32_TIM3_CCR4));
  lcddbg("CCR4:    %04x\n", getreg32(STM32_TIM3_CCR4));
  lcddbg("CCR4:    %04x\n", getreg32(STM32_TIM3_CCR4));
  lcddbg("DMAR:    %04x\n", getreg32(STM32_TIM3_DMAR));
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
  unsigned short id;

  gvdbg("Initializing\n");

  /* Configure GPIO pins and configure the FSMC to support the LCD */
  stm32_selectlcd();

  /* Delay required here */

  up_mdelay(50);

  /* Check model id */

  id=read_reg(0x0);
  if (id != SSD1289_ID) {
    /* Not a SSD1289 ? */
    gdbg("up_lcdinitialize: LCD ctrl is not a SSD1289");
    return ERROR;
  }

  /* Configure and enable LCD */

  lcd_initialize();

  /* Clear the display (setting it to the color 0=black) */

  lcd_clear(0);

  /* Configure the backlight */

  lcd_backlight();
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
  stm32_deselectlcd();
}

/**************************************************************************************
 * Name:  lcd_clear
 *
 * Description:
 *   Fill the LCD ctrl memory with given color
 *
 **************************************************************************************/

void lcd_clear(uint16_t color)
{
  uint32_t index;
  lcd_setcursor(0, 0);
  lcd_gramselect(); /* Prepare to write GRAM */
  for (index = 0; index < LCD_XRES * LCD_YRES; index++)
  {
    write_data(color);
  }
}
