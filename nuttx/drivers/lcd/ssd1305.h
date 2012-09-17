/**************************************************************************************
 * drivers/lcd/ssd1305.h
 * Definitions for the Solomon Systech SSD1305 132x64 Dot Matrix OLED/PLED
 * Segment/Common Driver with C
 *
 *   Copyright (C) 2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: SSD1305.pdf, "Solomon Systech SSD1305 132x64 Dot Matrix OLED/PLED
 *             Segment/Common Driver with Controller," Solomon Systech Limited,
 *             http://www.solomon-systech.com, May, 2008.
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

#ifndef __DRIVERS_LCD_SSD1305_H
#define __DRIVERS_LCD_SSD1305_H

/**************************************************************************************
 * Included Files
 **************************************************************************************/

/**************************************************************************************
 * Pre-processor Definitions
 **************************************************************************************/
/* General Definitions ******************************************************/

#define SSD1305_COLORA              0
#define SSD1305_COLORB              1
#define SSD1305_COLORC              2
#define SSD1305_COLORD              3

/* Fundamental Commands *****************************************************/
#define SSD1305_SETCOLL             0x00 /* 0x00-0x0f: Set lower column address */
#  define SSD1305_COLL_MASK         0x0f
#define SSD1305_SETCOLH             0x10 /* 0x10-0x1f: Set higher column address */
#  define SSD1305_COLH_MASK         0x0f
#define SSD1305_ADDRMODE            0x20 /* 0x20: Set memory address mode */
#  define SSD1305_ADDRMODE_HOR      0x00 /*   Data 1: Set horizontal address mode */
#  define SSD1305_ADDRMODE_VIRT     0x01 /*   Data 1: Set virtal address mode */
#  define SSD1305_ADDRMODE_PAGE     0x02 /*   Data 1: Set page address mode */
#define SSD1305_SETCOLADDR          0x21 /* 0x21: Set column address */
                                         /*   Data 1: Column start address: 0-131 */
                                         /*   Data 2: Column end address: 0-131 */
#define SSD1305_SETPAGEADDR         0x22 /* 0x22: Set page address */
                                         /*   Data 1: Page start address: 0x00-0x7d */
                                         /*   Data 2: Page end address: 0x00-0x7d */
#define SSD1305_SETSTARTLINE        0x40 /* 0x40-7f: Set display start line */
#  define SSD1305_STARTLINE_MASK    0x3f

#define SSD1305_SETCONTRAST         0x81 /* 0x81: Set contrast control */
                                         /*   Data 1: Set 1 of 256 contrast steps */
#define SSD1305_SETBRIGHTNESS       0x82 /* 0x82: Set brightness */
                                         /*   Data 1: Set 1 of 256 contrast steps */
#define SSD1305_SETLUT              0x91 /* 0x01: Set lookup table */
                                         /*   Data 1: Pulse width: 31-63 */
                                         /*   Data 2: Color A: 31-63  */
                                         /*   Data 3: Color B: 31-63 */
                                         /*   Data 4: Color C: 31-63 */
#define SSD1305_SETBANKCOLOR1       0x92      /* 0x92: Set bank 1-16 color */
#  define SSD1305_SETBANK1(c)       (c)       /* Data 1, Bits 0-1: Bank 1 color */
#  define SSD1305_SETBANK2(c)       (c << 2)  /* Data 1, Bits 2-3: Bank 2 color */
#  define SSD1305_SETBANK3(c)       (c << 4)  /* Data 1, Bits 4-5: Bank 3 color */
#  define SSD1305_SETBANK4(c)       (c << 6)  /* Data 1, Bits 6-7: Bank 4 color */
#  define SSD1305_SETBANK5(c)       (c)       /* Data 2, Bits 0-1: Bank 5 color */
#  define SSD1305_SETBANK6(c)       (c << 2)  /* Data 2, Bits 2-3: Bank 6 color */
#  define SSD1305_SETBANK7(c)       (c << 4)  /* Data 2, Bits 4-5: Bank 7 color */
#  define SSD1305_SETBANK8(c)       (c << 6)  /* Data 2, Bits 6-7: Bank 8 color */
#  define SSD1305_SETBANK9(c)       (c)       /* Data 3, Bits 0-1: Bank 9 color */
#  define SSD1305_SETBANK10(c)      (c << 2)  /* Data 3, Bits 2-3: Bank 10 color */
#  define SSD1305_SETBANK11(c)      (c << 4)  /* Data 3, Bits 4-5: Bank 11 color */
#  define SSD1305_SETBANK12(c)      (c << 6)  /* Data 3, Bits 6-7: Bank 12 color */
#  define SSD1305_SETBANK13(c)      (c)       /* Data 4, Bits 0-1: Bank 13 color */
#  define SSD1305_SETBANK14(c)      (c << 2)  /* Data 4, Bits 2-3: Bank 14 color */
#  define SSD1305_SETBANK15(c)      (c << 4)  /* Data 4, Bits 4-5: Bank 15 color */
#  define SSD1305_SETBANK16(c)      (c << 6)  /* Data 4, Bits 6-7: Bank 16 color */
#define SSD1305_SETBANKCOLOR2       0x93      /* 0x93: Set bank 17-32 color */
#  define SSD1305_SETBANK17(c)      (c)       /* Data 1, Bits 0-1: Bank 17 color */
#  define SSD1305_SETBANK18(c)      (c << 2)  /* Data 1, Bits 2-3: Bank 18 color */
#  define SSD1305_SETBANK19(c)      (c << 4)  /* Data 1, Bits 4-5: Bank 19 color */
#  define SSD1305_SETBANK20(c)      (c << 6)  /* Data 1, Bits 6-7: Bank 20 color */
#  define SSD1305_SETBANK21(c)      (c)       /* Data 2, Bits 0-1: Bank 21 color */
#  define SSD1305_SETBANK22(c)      (c << 2)  /* Data 2, Bits 2-3: Bank 22 color */
#  define SSD1305_SETBANK23(c)      (c << 4)  /* Data 2, Bits 4-5: Bank 23 color */
#  define SSD1305_SETBANK24(c)      (c << 6)  /* Data 2, Bits 6-7: Bank 24 color */
#  define SSD1305_SETBANK25(c)      (c)       /* Data 3, Bits 0-1: Bank 25 color */
#  define SSD1305_SETBANK26(c)      (c << 2)  /* Data 3, Bits 2-3: Bank 26 color */
#  define SSD1305_SETBANK27(c)      (c << 4)  /* Data 3, Bits 4-5: Bank 27 color */
#  define SSD1305_SETBANK28(c)      (c << 6)  /* Data 3, Bits 6-7: Bank 28 color */
#  define SSD1305_SETBANK29(c)      (c)       /* Data 4, Bits 0-1: Bank 29 color */
#  define SSD1305_SETBANK30(c)      (c << 2)  /* Data 4, Bits 2-3: Bank 30 color */
#  define SSD1305_SETBANK31(c)      (c << 4)  /* Data 4, Bits 4-5: Bank 31 color */
#  define SSD1305_SETBANK32(c)      (c << 6)  /* Data 4, Bits 6-7: Bank 32 color */
#define SSD1305_MAPCOL0             0xa0 /* 0xa0: Column address 0 is mapped to SEG0 */
#define SSD1305_MAPCOL131           0xa1 /* 0xa1: Column address 131 is mapped to SEG0 */
#define SSD1305_DISPRAM             0xa4 /* 0xa4: Resume to RAM content display */
#define SSD1305_DISPENTIRE          0xa5 /* 0xa5: Entire display ON */
#define SSD1305_DISPNORMAL          0xa6 /* 0xa6: Normal display */
#define SSD1305_DISPINVERTED        0xa7 /* 0xa7: Inverse display */

#define SSD1305_SETMUX              0xa8 /* 0xa8: Set Multiplex Ratio*/
                                         /*   Data 1: MUX ratio -1: 15-63 */
#define SSD1305_DIMMODE             0xab /* 0xab: Dim mode setting */
                                         /*   Data 1: Reserverd, must be zero */
                                         /*   Data 2: Contrast for bank1: 0-255 */
                                         /*   Data 3: Brightness for color bank: 0-255 */
#define SSD1305_MSTRCONFIG          0xad /* 0xad: Master configuration */
#  define SSD1305_MSTRCONFIG_EXTVCC 0x8e /*   Data 1: Select external Vcc */
#define SSD1305_DISPONDIM           0xac /* 0xac: Display ON in dim mode */
#define SSD1305_DISPOFF             0xae /* 0xae: Display OFF (sleep mode) */
#define SSD1305_DISPON              0xaf /* 0xaf: Display ON in normal mode */
#define SSD1305_SETPAGESTART        0xb0 /* 0xb0-b7: Set page start address */
#  define SSD1305_PAGESTART_MASK    0x07
#define SSD1305_SETCOMNORMAL        0xc0 /* 0xc0: Set COM output, normal mode */
#define SSD1305_SETCOMREMAPPED      0xc8 /* 0xc8: Set COM output, remapped mode */

#define SSD1305_SETOFFSET           0xd3 /* 0xd3: Set display offset */
                                         /*   Data 1: Vertical shift by COM: 0-63 */
#define SSD1305_SETDCLK             0xd5 /* 0xd5:  Set display clock divide ratio/oscillator */
#  define SSD1305_DCLKDIV_SHIFT     (0)  /*   Data 1, Bits 0-3: DCLK divide ratio/frequency*/
#  define SSD1305_DCLKDIV_MASK      0x0f
#  define SSD1305_DCLKFREQ_SHIFT    (4)  /*   Data 1, Bits 4-7: DCLK divide oscillator frequency */
#  define SSD1305_DCLKFREQ_MASK     0xf0
#define SSD1305_SETCOLORMODE        0xd8 /* 0xd:  Set area color and low power display modes */
#  define SSD1305_COLORMODE_MONO    0x00 /*   Data 1, Bits 4-5: 00=monochrome */
#  define SSD1305_COLORMODE_COLOR   0x30 /*   Data 1, Bits 4-5: 11=area color enable */
#  define SSD1305_POWERMODE_NORMAL  0x00 /*   Data 1, Bits 0,2: 00=normal power mode */
#  define SSD1305_POWERMODE_LOW     0x05 /*   Data 1, Bits 0,2: 11=low power display mode */
#define SSD1305_SETPRECHARGE        0xd9 /* 0xd9: Set pre-charge period */
#  define SSD1305_PHASE1_SHIFT      (0)  /*   Data 1, Bits 0-3: Phase 1 period of up to 15 DCLK clocks */
#  define SSD1305_PHASE1_MASK       0x0f
#  define SSD1305_PHASE2_SHIFT      (4)  /*   Data 1, Bits 4-7: Phase 2 period of up to 15 DCLK clocks */
#  define SSD1305_PHASE2_MASK       0xf0
#define SSD1305_SETCOMCONFIG        0xda /* 0xda: Set COM configuration */
#  define SSD1305_COMCONFIG_SEQ     0x02 /*   Data 1, Bit 4: 0=Sequential COM pin configuration */
#  define SSD1305_COMCONFIG_ALT     0x12 /*   Data 1, Bit 4: 1=Alternative COM pin configuration */
#  define SSD1305_COMCONFIG_NOREMAP 0x02 /*   Data 1, Bit 5: 0=Disable COM Left/Right remap */
#  define SSD1305_COMCONFIG_REMAP   0x22 /*   Data 1, Bit 5: 1=Enable COM Left/Right remap */
#define SSD1305_SETVCOMHDESEL       0xdb /* 0xdb: Set VCOMH delselect level */
#  define SSD1305_VCOMH_x4p3        0x00 /*   Data 1: ~0.43 x Vcc */
#  define SSD1305_VCOMH_x7p7        0x34 /*   Data 1: ~0.77 x Vcc */
#  define SSD1305_VCOMH_x8p3        0x3c /*   Data 1: ~0.83 x Vcc */
#define SSD1305_ENTER_RMWMODE       0xe0 /* 0xe0: Enter the Read Modify Write mode */
#define SSD1305_NOP                 0xe3 /* 0xe3: NOP Command for no operation */
#define SSD1305_EXIT_RMWMODE        0xee /* 0xee: Leave the Read Modify Write mode */

/* Graphic Acceleration Commands ********************************************/

#define SSD1305_HSCROLL_RIGHT       0x26 /* 0x26: Right horizontal scroll */
#define SSD1305_HSCROLL_LEFT        0x27 /* 0x27: Left horizontal scroll */
                                         /*   Data 1, Bits 0-2: Column scroll offset: 0-4 */
                                         /*   Data 2, Bits 0-2: Start page address: 0-7 */
#define SSD1305_HSCROLL_FRAMES6     0x00 /*   Data 3, Bits 0-2: Timer interval, 000=6 frames */
#define SSD1305_HSCROLL_FRAMES32    0x01 /*   Data 3, Bits 0-2: Timer interval, 001=32 frames */
#define SSD1305_HSCROLL_FRAMES64    0x02 /*   Data 3, Bits 0-2: Timer interval, 010=64 frames */
#define SSD1305_HSCROLL_FRAMES128   0x03 /*   Data 3, Bits 0-2: Timer interval, 011=128 frames */
#define SSD1305_HSCROLL_FRAMES3     0x04 /*   Data 3, Bits 0-2: Timer interval, 100=3 frames */
#define SSD1305_HSCROLL_FRAMES4     0x05 /*   Data 3, Bits 0-2: Timer interval, 101=4 frames */
#define SSD1305_HSCROLL_FRAMES2     0x06 /*   Data 3, Bits 0-2: Timer interval, 110=2 frames */
                                         /*   Data 4, Bits 0-2: End page address: 0-7 */

#define SSD1305_VSCROLL_RIGHT       0x29 /* 0x26: Vertical and right horizontal scroll */
#define SSD1305_VSCROLL_LEFT        0x2a /* 0x27: Vertical and left horizontal scroll */
                                         /*   Data 1, Bits 0-2: Column scroll offset: 0-4 */
                                         /*   Data 2, Bits 0-2: Start page address: 0-7 */
#define SSD1305_VSCROLL_FRAMES6     0x00 /*   Data 3, Bits 0-2: Timer interval, 000=6 frames */
#define SSD1305_VSCROLL_FRAMES32    0x01 /*   Data 3, Bits 0-2: Timer interval, 001=32 frames */
#define SSD1305_VSCROLL_FRAMES64    0x02 /*   Data 3, Bits 0-2: Timer interval, 010=64 frames */
#define SSD1305_VSCROLL_FRAMES128   0x03 /*   Data 3, Bits 0-2: Timer interval, 011=128 frames */
#define SSD1305_VSCROLL_FRAMES3     0x04 /*   Data 3, Bits 0-2: Timer interval, 100=3 frames */
#define SSD1305_VSCROLL_FRAMES4     0x05 /*   Data 3, Bits 0-2: Timer interval, 101=4 frames */
#define SSD1305_VSCROLL_FRAMES2     0x06 /*   Data 3, Bits 0-2: Timer interval, 110=2 frames */
                                         /*   Data 4, Bits 0-2: End page address: 0-7 */
                                         /*   Data 5, Bits 0-5: Vertical scrolling offset: 0-63 */
#define SSD1305_SCROLL_STOP         0x2e /* 0x2e: Deactivate scroll */
#define SSD1305_SCROLL_START        0x2f /* 0x2f: Activate scroll */
#define SSD1305_VSCROLL_AREA        0xa3 /* 0xa3: Set vertical scroll area */
                                         /*   Data 1: Number of rows in the top fixed area */
                                         /*   Data 1: Number of rows in the scroll area */

/* Status register bit definitions ******************************************/

#define SSD1305_STATUS_DISPOFF       (1 << 6) /* Bit 6: 1=Display off */

#endif /* __DRIVERS_LCD_SSD1305_H */
