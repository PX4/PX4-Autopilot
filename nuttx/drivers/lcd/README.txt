Contents
========

This is the README.txt file for the drivers/lcd/ directory.

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

  nokia6100.c.  Supports the Nokia 6100 display with either the Philips
    PCF883 or the Epson S1D15G10 display controller.  This LCD is used
    with the Olimex LPC1766-STK (but has not been fully integrated).
  
  p14201.c.  Driver for RiT P14201 series display with SD1329 IC
    controller.  This OLED is used with older versions of the
    TI/Luminary LM3S8962 Evaluation Kit.
  
  ug-9664hswag01.c.  OLED Display Module, UG-9664HSWAG01", Univision
    Technology Inc.  Used with the LPC Xpresso and Embedded Artists
    base board.

Examples: configs/
==================

There are additional LCD drivers in the configs/<board>/src directory
that support additional LCDs.  LCD drivers in the configuration directory
if they support some differ LCD interface (such as a parallel interface)
that makes then less re-usable:

configs/sam3u-ek/src/up_lcd.c.

  configs/hymini-stm32v/src/ssd1289.c.
  
    SSD1289
  
  configs/sam3u-ek/src/up_lcd.c
  
    The SAM3U-EK developement board features a TFT/Transmissive color
    LCD module with touch-screen, FTM280C12D, with integrated driver IC
    HX8346.

  configs/stm3210e-eval/src/up_lcd.c

    This driver supports the following LCDs:

    1. Ampire AM-240320LTNQW00H
    2. Orise Tech SPFD5408B
    3. RenesasSP R61580
     
  configs/skp16c26/src/up_lcd.c.  Untest alphanumeric LCD driver.

graphics/
=========

  See also the usage of the LCD driver in the graphics/ directory.
