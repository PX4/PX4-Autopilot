/****************************************************************************
 * configs/xtrs/include/trs80-m3.h
 *
 *   Copyright (C) 2008 Jacques Pelletier. All rights reserved.
 *   Author: Jacques Pelletier
 *
 * This file is a part of NuttX and hence
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

/* Information from http://www.trs-80.com */

#ifndef __TRS80_M3_H
#define __TRS80_M3_H

/* Outputs a byte to a logical device or FCB. DE = FCB and A = byte.
 * Don't confuse with CTL at 0023.
 */

#define _TRS80_M3_PUT 		0x001B

/* Outputs a control byte to a logical device or FCB. DE = FCB and A = control byte. */

#define _TRS80_M3_CTL		0x0023

/* Scan keyboard and return with accumulator containing result. DE is used. */

#define _TRS80_M3_KBDSCN	0x002B

/* Displays a character at current cursor location. */

#define _TRS80_M3_VDCHAR	0x0033

/* "Waits until printer is ready then prints character. A = ASCII character.
 * If BREAK is pressed, a return to caller is made."
 */

#define _TRS80_M3_PRCHAR	0x003B

/* Call Input a line from the keyboard. B = max length of line. HL points at buffer.
 * Buffer should be the length of B plus 1. To terminate, hit BREAK or ENTER. 
 * On exit, HL points at buffer and B = number of characters entered. 
 * Carry will be set if BREAK was pressed.
 */

#define _TRS80_M3_KBLINE	0x0040

/* "Scans the keyboard until a key is pressed. If BREAK is pressed, it is returned
 * like other keys."
 */

#define _TRS80_M3_KBWAIT	0x0049

/* "Receive a character from RS-232. No entry conditions. On exit, memory location
 * 16872 contains character received. DE is altered. This routine honors wait status."
 */

#define _TRS80_M3_RSRCV		0x0050

/* "Transmit character to RS-232. On entry, Accumulator or memory location 16880 
 * contains character. On exit, 16880 = 0 if no character sent. Wait status honored."
 */

#define _TRS80_M3_RSTX		0x0055

/* Initialize RS-232 interface. On entry, memory location 16888 = send/receive baud 
 * rate code, location 16890 = wait/don't wait switch, location 16889 = RS-232
 * characteristics switch. On exit, DE is altered. For more detail, consult Model 3
 * reference manual.
 */

#define _TRS80_M3_RSINIT	0x005A

/* This is the routine that is Basic's SET, RESET, and POINT functions. Here's how 
 * to use it. Load HL with return address and push. Load register A with one of the 
 * following: 00H = POINT, 01H = RESET, and 80H = SET. Push AF onto stack. Load A with 
 * X coordinate and push onto stack. Load A with Y coordinate and JP GRAPH.
 */

#define _TRS80_M3_GRAPH		0x0150	

/* Clear screen. */

#define _TRS80_M3_CLS		0x01C9

/* Randomize. */

#define _TRS80_M3_RANDOM	0x01D3

/* This routine turns off the cassette drive. */

#define _TRS80_M3_CSOFF		0x01F8

#ifdef TRS80_MODEL1

/* A register contains a 0 or 1 which is the cassette number. This routine defines 
 * cassette number and turns on cassette. Model I only.
 */

#define _TRS80_M3_DEFCAS	0x0212

#endif /* TRS80_MODEL1 */

/* Inputs data one byte at a time from cassette after you use CSHIN. A = the data
 * byte.
 */

#define _TRS80_M3_CSIN		0x0235

/* Outputs data one byte at a time to cassette after you use CSHWR. A = the 
 * output byte.
 */

#define _TRS80_M3_CSOUT		0x0264

/* Turns on the cassette and writes the header. */

#define _TRS80_M3_CSHWR		0x0287

/* Finds the cassette header info at the beggining of cassette file. */

#define _TRS80_M3_CSHIN		0x0296

/* Ouput character in register A; OUTSEL (409CH) selects device. See OUTSEL for
 * device values.
 */

#define _TRS80_M3_OUTCHR	0x032A

/* Displays character in A on screen at next print position. Uses AF. */

#define _TRS80_M3_DISPA		0x033A

/* Calls keyboard scan routine. */

#define _TRS80_M3_keyb_scan	0x0358

/* Reads keyboard into buffer until a carriage return is entered. 40A7H contains
 * the address of the buffer.
 */

#define _TRS80_M3_KIBUFF	0x0361

/* Same as KBLINE. See 0040. */

#define _TRS80_M3_KLINE		0x05D9

/* "Get date in ASCII format. Mod III TRSDOS, LDOS, & MULTIDOS." */

#define _TRS80_M3_GETDAT 	0x3033

/* "Get time in ASCII format. Mod III TRSDOS, LDOS, & MULTIDOS." */ 

#define _TRS80_M3_GETTIM	0x3036

#endif /* __TRS80_M3_H */
