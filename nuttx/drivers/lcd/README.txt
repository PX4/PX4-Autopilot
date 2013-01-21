nuttx/drivers/lcd README
========================

This is the README.txt file for the drivers/lcd/ directory.

Contents
========

  - LCD Header files
    include/nuttx/lcd/lcd.h
    struct lcd_dev_s
  - Binding LCD Drivers
  - Examples: /drivers/lcd/
  - Examples: configs/
  - graphics/

LCD Header files
================

  include/nuttx/lcd/lcd.h

      Structures and APIs needed to work with LCD drivers are provided in
      this header file.  This header file also depends on some of the same
      definitions used for the frame buffer driver as privided in
      include/nuttx/fb.h.

  struct lcd_dev_s

      Each LCD device driver must implement an instance of struct lcd_dev_s.
      That structure defines a call table with the following methods:

      - Get information about the LCD video controller configuration and the
        configuration of each LCD color plane.

        int (*getvideoinfo)(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo);
        int (*getplaneinfo)(FAR struct lcd_dev_s *dev, unsigned int planeno,
                           FAR struct lcd_planeinfo_s *pinfo);

      - The following are provided only if the video hardware supports RGB
        color mapping:

        int (*getcmap)(FAR struct lcd_dev_s *dev,
                       FAR struct fb_cmap_s *cmap);
        int (*putcmap)(FAR struct lcd_dev_s *dev,
                       FAR const struct fb_cmap_s *cmap);

      - The following are provided only if the video hardware supports a
        hardware cursor:

        int (*getcursor)(FAR struct lcd_dev_s *dev,
                         FAR struct fb_cursorattrib_s *attrib);
        int (*setcursor)(FAR struct lcd_dev_s *dev,
                         FAR struct fb_setcursor_s *settings);

      - Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
        full on). On backlit LCDs, this setting may correspond to the
        backlight setting.

        int (*getpower)(struct lcd_dev_s *dev);

      - Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
        full on). On backlit LCDs, this setting may correspond to the
        backlight setting.

        int (*setpower)(struct lcd_dev_s *dev, int power);

      - Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST) */

        int (*getcontrast)(struct lcd_dev_s *dev);

     - Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST)

        int (*setcontrast)(struct lcd_dev_s *dev, unsigned int contrast);

Binding LCD Drivers
===================

  LCD drivers are not normally directly accessed by user code, but are
  usually bound to another,  higher level device driver. In general, the
  binding sequence is:

  1. Get an instance of struct lcd_dev_s from the hardware-specific LCD
     device driver, and 
  2. Provide that instance to the initialization method of the higher
     level device driver.

Examples: /drivers/lcd/
=======================

Re-usable LCD drivers reside in the drivers/lcd directory:

  mio283qt2.c. This is a driver for the MI0283QT-2 LCD from Multi-Inno
    Technology Co., Ltd.  This LCD is based on the Himax HX8347-D LCD
    controller.

  nokia6100.c.  Supports the Nokia 6100 display with either the Philips
    PCF883 or the Epson S1D15G10 display controller.  This LCD is used
    with the Olimex LPC1766-STK (but has not been fully integrated).
  
  p14201.c.  Driver for RiT P14201 series display with SD1329 IC
    controller.  This OLED is used with older versions of the
    TI/Luminary LM3S8962 Evaluation Kit.

  ssd12989.c.  Generic LCD driver for LCDs based on the Solomon Systech
    SSD1289 LCD controller. Think of this as a template for an LCD driver
    that you will proably ahve to customize for any particular LCD
    hardware. (see also configs/hymini-stm32v/src/ssd1289.c below).

  ug-9664hswag01.c.  OLED Display Module, UG-9664HSWAG01", Univision
    Technology Inc.  Used with the LPC Xpresso and Embedded Artists
    base board.

Examples: configs/
==================

There are additional LCD drivers in the configs/<board>/src directory
that support additional LCDs.  LCD drivers in the configuration directory
if they support some differ LCD interface (such as a parallel interface)
that makes then less re-usable:

  SSD1783 Drivers:

    configs/compal_e99/src/ssd1783.c

  SSD1289 Drivers:

    configs/hymini-stm32v/src/ssd1289.c.  See also drivers/lcd/ssd1298.c
      above.
    configs/stm32f4discovery/src/up_ssd1289.c.  This examples is the
      bottom half for the SSD1289 driver at drivers/lcd/ssd1289.c
    configs/hymini-stm32v/src/ssd1289.c.  See also drivers/lcd/ssd1298.c
      above.
    configs/shenzhou/src/up_ssd1289.c

  kwikstik-k40:
  
    configs/kwikstik-k40/src/up_lcd.c.  Don't waste your time.  This is
      just a stub.

  Nokia LCD Drivers:

    configs/olimex-lpc1766stk/src/up_lcd.c.  This examples is the
      bottom half for the driver at drivers/lcd/nokia6100.c.
      This was never completedly debugged ... there are probably issues
      with that nasty 9-bit SPI interfaces.

  HX8346:

    configs/sam3u-ek/src/up_lcd.c.  The SAM3U-EK developement board features
      a TFT/Transmissive color LCD module with touch-screen, FTM280C12D,
      with integrated driver IC HX8346.

  ILI93xx and Similar:

    configs/stm3210e-eval/src/up_lcd.c. This driver supports the following
      LCDs:

      1. Ampire AM-240320LTNQW00H
      2. Orise Tech SPFD5408B
      3. RenesasSP R61580

    configs/stm3220g-eval/src/up_lcd.c and configs/stm3240g-eval/src/up_lcd.c.
      AM-240320L8TNQW00H (LCD_ILI9320 or LCD_ILI9321) and
      AM-240320D5TOQW01H (LCD_ILI9325)
    configs/shenzhou/src/up_ili93xx.c. Another ILI93xx driver.

  OLEDs:

    configs/stm32f4discovery/src/up_ug2864ambag01.c
    configs/stm32f4discovery/src/up_ug2864hsweg01.c
    configs/zp214xpa/src/up_ug2864ambag01.c

  Alphnumeric Displays:

    configs/skp16c26/src/up_lcd.c.  Untested alphanumeric LCD driver.
    configs/stm32f4discovery/src/up_lcd1602.c

graphics/
=========

  See also the usage of the LCD driver in the graphics/ directory.

