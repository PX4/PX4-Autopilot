/********************************************************************************************
 * include/nuttx/lcd/hd4478ou.h
 *
 * Definitions for the Hitachi HD44780U LCD controller (as used in the
 * LCD1602).
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ********************************************************************************************/

#ifndef __INCLUDE_NUTTX_HD4478OU_H
#define __INCLUDE_NUTTX_HD4478OU_H

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
/* Command set:
 *
 *   RS=0 R/W=0 : Command
 *   RS=0 R/W=1 : Busy/AD, Read CT (see below)
 *   RS=1 R/W=0 : Write data to CGRAM or DDRAM
 *   RS=1 R/W=0 : Read data from CGRAM or DDRAM
 */

#define HD4478OU_CLEAR            (0x01)     /* Screen Clear, Set AC to 0 */
#define HD4478OU_RETURN           (0x02)     /* DDRAM AD=0, return */
#define HD4478OU_INPUT            (0x04)     /* Set moving direction of cursor */
#  define HD4478OU_INPUT_SHIFT    (1 << 0)   /*   Shift */
#  define HD4478OU_INPUT_INCR     (1 << 1)   /*   Increment mode */
#  define HD4478OU_INPUT_DECR     (0x00)     /*   Decrement mode */
#define HD4478OU_DISPLAY          (0x08)     /* Set display, cursor, blink on/off */
#  define HD4478OU_DISPLAY_BLINK  (1 << 0)   /*   Blink on/off */
#  define HD4478OU_DISPLAY_CURSOR (1 << 1)   /*   Cursor on/off */
#  define HD4478OU_DISPLAY_ON     (1 << 2)   /*   Display on/off */
#define HD4478OU_SHIFT            (0x10)     /* Remove cursor and whole diplay */
#  define HD4478OU_SHIFT_RIGHT    (1 << 2)   /*   Shift right */
#  define HD4478OU_SHIFT_LEFT     (0x00)     /*   Shift right */
#  define HD4478OU_SHIFT_DISPLAY  (1 << 3)   /*   Display shift */
#  define HD4478OU_SHIFT_CURSOR   (0x00)     /*   Cursor shift */
#define HD4478OU_FUNC             (0x20)     /* Set DL, display line, font */
#  define HD4478OU_FUNC_F5x10     (1 << 2)   /*   5x10 Style */
#  define HD4478OU_FUNC_F5x7      (0x00)     /*   5x7 Style */
#  define HD4478OU_FUNC_N1        (1 << 3)   /*   N=2R */
#  define HD4478OU_FUNC_N0        (0x00)     /*   N=1R */
#  define HD4478OU_FUNC_DL8D      (1 << 4)   /*   DL=8D, 8-bit interface */
#  define HD4478OU_FUNC_DL4D      (0x00)     /*   DL=4D, 4-bit interface */
#define HD4478OU_CGRAM_AD(a)      (0x40|(a)) /* Set CGRAM AD, send receive data */
#define HD4478OU_DDRAM_AD(a)      (0x80|(a)) /* Set DDRAM AD, send receive data */

/* RS=0 R/W=1 : Execute internal function, read AD of CT */

#define HD4478OU_BUSY(bf,ac)      ((bf) << 7 | (ac))

/********************************************************************************************
 * Pre-processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * Public Types
 ********************************************************************************************/

/********************************************************************************************
 * Public Data
 ********************************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/********************************************************************************************
 * Public Function Prototypes
 ********************************************************************************************/

/********************************************************************************************
 * Name:  up_lcd1602_initialize
 *
 * Description:
 *   the LCD1602 is an HD4478OU-based LCD from Wave share.  This function initializes the
 *   LCD1602 hardware and registers the character driver as /dev/lcd1602.
 *
 ********************************************************************************************/

int up_lcd1602_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HD4478OU_H */
