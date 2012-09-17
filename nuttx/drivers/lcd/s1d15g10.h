/**************************************************************************************
 * drivers/lcd/s1d15g10.h
 * Definitions for the Epson S1D15G0 LCD controller
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: S1D15G0D08B000, Seiko Epson Corportation, 2002.
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

#ifndef __DRIVERS_LCD_S1D15G10_H
#define __DRIVERS_LCD_S1D15G10_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/

/* Epson S1D15G10 Command Set */

#define S1D15G10_DISON       0xaf /* Display on; Data: none */
#define S1D15G10_DISOFF      0xae /* Display off; Data: none */
#define S1D15G10_DISNOR      0xa6 /* Normal display; Data: none */
#define S1D15G10_DISINV      0xa7 /* Inverse display; Data: none */
#define S1D15G10_COMSCN      0xbb /* Common scan direction; Data: (1) common scan direction */
#define S1D15G10_DISCTL      0xca /* Display control; Data: Data: (1) CL div, F1/2 pat, (2) duty, (3) FR inverse (4) dispersion */
#define S1D15G10_SLPIN       0x95 /* Sleep in; Data: none */
#define S1D15G10_SLPOUT      0x94 /* Sleep out; Data: none */
#define S1D15G10_PASET       0x75 /* Page address set; Data: (1) start page, (2) end page */
#define S1D15G10_CASET       0x15 /* Column address set; Data: (1) start addr, (2) end addr */
#define S1D15G10_DATCTL      0xbc /* Data scan direction, etc.; Data: (1) inverse, scan dir (2) RGB, (3) gray-scale */
#define S1D15G10_RGBSET8     0xce /* 256-color position set; Data: (1-8) red tones, (9-16) green tones, (17-20) blue tones */
#define S1D15G10_RAMWR       0x5c /* Writing to memory; Data: (1) write data */
#define S1D15G10_RAMRD       0x5d /* Reading from memory; Data: (1) read data */
#define S1D15G10_PTLIN       0xa8 /* Partial display in; Data: (1) start addr, (2) end addr */
#define S1D15G10_PTLOUT      0xa9 /* Partial display out; Data: none */
#define S1D15G10_RMWIN       0xe0 /* Read and modify write; Data: none */
#define S1D15G10_RMWOUT      0xee /* End; Data: none */
#define S1D15G10_ASCSET      0xaa /* Area scroll set; Data: (1) top addr, (2) bottom addr, (3) Num blocks, (4) scroll mode */
#define S1D15G10_SCSTART     0xab /* Scroll start set; Data: (1) start block addr */
#define S1D15G10_OSCON       0xd1 /* Internal oscillation on; Data: none */
#define S1D15G10_OSCOFF      0xd2 /* Internal oscillation off; Data: none */
#define S1D15G10_PWRCTR      0x20 /* Power control; Data: (1) LCD drive power */
#define S1D15G10_VOLCTR      0x81 /* Electronic volume control; Data: (1) volume value, (2) resistance ratio */
#define S1D15G10_VOLUP       0xd6 /* Increment electronic control by 1; Data: none */
#define S1D15G10_VOLDOWN     0xd7 /* Decrement electronic control by 1; Data: none */
#define S1D15G10_TMPGRD      0x82 /* Temperature gradient set; Data: (1-14) temperature gradient */
#define S1D15G10_EPCTIN      0xcd /* Control EEPROM; Data: (1) read/write */
#define S1D15G10_EPCOUT      0xcc /* Cancel EEPROM control; Data: none */
#define S1D15G10_EPMWR       0xfc /* Write into EEPROM; Data: none */
#define S1D15G10_EPMRD       0xfd /* Read from EEPROM; Data: none */
#define S1D15G10_EPSRRD1     0x7c /* Read register 1; Data: none */
#define S1D15G10_EPSRRD2     0x7d /* Read regiser 2; Data: none */
#define S1D15G10_NOP         0x25 /* NOP intruction (0x45?); Data: none */
#define S1D15G10_STREAD      0x20 /* Status read; Data: none */

/* Display control (DISCTL) bit definitions */

#define DISCTL_PERIOD_SHIFT  (0)  /* P1: Bits 0-1, F1 and F2 drive-pattern switching period */
#define DISCTL_PERIOD_MASK   (3 << DISCTL_PERIOD_SHIFT)
#  define DISCTL_PERIOD_8    (0 << DISCTL_PERIOD_SHIFT)
#  define DISCTL_PERIOD_4    (1 << DISCTL_PERIOD_SHIFT)
#  define DISCTL_PERIOD_16   (2 << DISCTL_PERIOD_SHIFT)
#  define DISCTL_PERIOD_FLD  (3 << DISCTL_PERIOD_SHIFT)
#define DISCTL_CLDIV_SHIFT   (2)  /* P1: Bits 2-4, Clock divider */
#define DISCTL_CLDIV_MASK    (7 << DISCTL_CLDIV_SHIFT)
#  define DISCTL_CLDIV_2     (0 << DISCTL_CLDIV_SHIFT)
#  define DISCTL_CLDIV_4     (1 << DISCTL_CLDIV_SHIFT)
#  define DISCTL_CLDIV_8     (2 << DISCTL_CLDIV_SHIFT)
#  define DISCTL_CLDIV_NONE  (3 << DISCTL_CLDIV_SHIFT)

/* Power control (PWRCTR) bit definitions */

#define PWCTR_REFVOLTAGE     (1 << 0) /* P1: Bit 0, Turn on reference voltage generation circuit. */
#define PWCTR_REGULATOR      (1 << 1) /* P1: Bit 1, Turn on voltage regulator and circuit voltage follower. */
#define PWCTR_BOOSTER2       (1 << 2) /* P1: Bit 2, Turn on secondary booster/step-down circuit. */
#define PWCTR_BOOSTER1       (1 << 3) /* P1: Bit 3, Turn on primary booster circuit. */
#define PWCTR_EXTR           (1 << 4) /* P1: Bit 4, Use external resistance to adjust voltage. */

/* Data control (DATCTL) bit definitions */

#define DATCTL_PGADDR_INV    (1 << 0) /* P1: Bit 0, Inverse display of the page address. */
#define DATCTL_COLADDR_REV   (1 << 1) /* P1: Bit 1, Reverse turn of column address. */
#define DATCTL_ADDR_PGDIR    (1 << 2) /* P1: Bit 2, Address-scan direction in page (vs column) direction. */

#define DATCTL_BGR           (1 << 0) /* P2: Bit0, RGB->BGR */

#define DATCTL_8GRAY         (1)      /* P3: Bits 0-2 = 001, 8 gray-scale */
#define DATCTL_16GRAY_A      (2)      /* P3: Bits 0-2 = 010, 16 gray-scale display type A */
#define DATCTL_16GRAY_B      (4)      /* P3: Bits 0-2 = 100, 16 gray-scale display type B */

/* Status register bit definions (after reset or NOP) */

#define S1D15G10_SR_PARTIAL  (1 << 0)  /* Bit 0: Partial display */
#define S1D15G10_SR_NORMAL   (1 << 1)  /* Bit 1: Normal (vs. inverse) display */
#define S1D15G10_SR_EEPROM   (1 << 2)  /* Bit 2: EEPROM access */
#define S1D15G10_SR_DISPON   (1 << 3)  /* Bit 3: Display on */
#define S1D15G10_SR_COLSCAN  (1 << 4)  /* Bit 4: Column (vs. page) scan direction */
#define S1D15G10_SR_RMW      (1 << 5)  /* Bit 5: Read modify write */
#define S1D15G10_SR_SCROLL   (3 << 6)  /* Bits 6-7: Area scroll mode */

/* Status register bit definions (after EPSRRD1) */

#define S1D15G10_SR_VOLUME   0x3f      /* Bits 0-5: Electronic volume control values */

/* Status register bit definions (after EPSRRD2) */

#define S1D15G10_SR_RRATIO   0x07      /* Bits 0-2: Built-in resistance ratio */

#endif /* __DRIVERS_LCD_S1D15G10_H */