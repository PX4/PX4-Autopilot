/****************************************************************************
 * configs/pic32mx7mmb/src/pic32mx7mmb_internal.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ****************************************************************************/

#ifndef __CONFIGS_MIKROELEKTRONIKA_PIC32MX7MMB_SRC_PIC32MX7MMB_INTERNAL_H
#define __CONFIGS_MIKROELEKTRONIKA_PIC32MX7MMB_SRC_PIC32MX7MMB_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* The Mikroelektronika PIC32MX7 MMB has 3 user LEDs labeled LED0-2 in the
 * schematics:
 *
 * ---  ----- --------------------------------------------------------------
 * PIN  Board Notes
 * ---  ----- --------------------------------------------------------------
 * RA0  LED0  Pulled-up, low value illuminates
 * RA1  LED1  Pulled-up, low value illuminates
 * RD9  LED2  Pulled-up, low value illuminates
 * RA9  LED4  Not available for general use*, indicates MMC/SD activity
 * ---  LED5  Not controllable by software, indicates power-on
 *
 * * RA9 is also the SD chip select.  It will illuminate whenever the SD card
 *   is selected.  If SD is not used, then LED4 could also be used as a user-
 *   controlled LED.
 */

/* The Mikroelektronika PIC32MX7 MMB has a joystick:
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RB0   JOY-A   Joystick A, HDR1 pin 24   Pulled up, low value when closed
 *   RB2   JOY-C   Joystick C, HDR1 pin 22   Pulled up, low value when closed
 *   RB1   JOY-B   Joystick B, HDR1 pin 23   Pulled up, low value when closed
 *   RB3   JOY-D   Joystick D, HDR1 pin 21   Pulled up, low value when closed
 *   RA10  JOY-CP  Joystick CP, HDR1 pin 25  Pulled up, low value when closed
 */

/* LCD
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RD2   LCD-BLED Backlight Light          Low value turns off
 */

/* SPI1 and SD Card
 *
 * ------ -------- ------------------------- --------------------------------
 *  GPIO   SIGNAL  BOARD CONNECTION           NOTES
 * ------ -------- ------------------------- --------------------------------
 *   RC4   SPI1    SD card slot              SPI1 data IN
 *   RD0   SPO1    SD card slot              SPI1 data OUT
 *   RD10  SCK1    SD card slot              SD card, SPI clock
 *
 *   RA9   SD_CS#  SD card slot              SD card, SPI chip select (active low)
 *   RG6   SD_WP   SD card slot              SD card, write protect
 *   RG7   SD_CD#  SD card slot              SD card, card detect (not)
 */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Name: pic32mx_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Mikroelektronika PIC32MX7
 *   MMB board.
 *
 ************************************************************************************/

#if defined(CONFIG_PIC32MX_SPI1) || defined(CONFIG_PIC32MX_SPI2) || \
    defined(CONFIG_PIC32MX_SPI3) || defined(CONFIG_PIC32MX_SPI4)
EXTERN void weak_function pic32mx_spiinitialize(void);
#endif

/************************************************************************************
 * Name: pic32mx_ledinit
 *
 * Description:
 *   Configure on-board LEDs if LED support has been selected.
 *
 ************************************************************************************/

#ifdef CONFIG_ARCH_LEDS
EXTERN void pic32mx_ledinit(void);
#endif

/****************************************************************************
 * Name: pic32mx_lcdinitialize
 *
 * Description:
 *   Initialize the LCD.  This function should be called early in the boot
 *   sequence -- even if the LCD is not enabled.  In that case we should
 *   at a minimum at least disable the LCD backlight.
 *
 ****************************************************************************/

EXTERN void pic32mx_lcdinitialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_MIKROELEKTRONIKA_PIC32MX7MMB_SRC_PIC32MX7MMB_INTERNAL_H */
