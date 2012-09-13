/***********************************************************************
 * pxdefs.h
 * Definitions of the arguments of the oSYSIO opcode
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 ***********************************************************************/

#ifndef __PXDEFS_H
#define __PXDEFS_H

/***********************************************************************/
/* Codes for system IO calls associated with standard Pascal procedure
 * and function calls.  These must be confined to the range 0x0000
 * through 0xffff.
 */

#define xEOF            (0x0001)
#define xEOLN           (0x0002)
#define xRESET          (0x0003)
#define xREWRITE        (0x0004)

#define xREADLN         (0x0010)
#define xREAD_PAGE      (0x0011)
#define xREAD_BINARY    (0x0012)
#define xREAD_INT       (0x0013)
#define xREAD_CHAR      (0x0014)
#define xREAD_STRING    (0x0015)
#define xREAD_REAL      (0x0016)

#define xWRITELN        (0x0020)
#define xWRITE_PAGE     (0x0021)
#define xWRITE_BINARY   (0x0022)
#define xWRITE_INT      (0x0023)
#define xWRITE_CHAR     (0x0024)
#define xWRITE_STRING   (0x0025)
#define xWRITE_REAL     (0x0026)

#define MAX_XOP         (0x0027)

/***********************************************************************/
/* Codes for runtime library interfaces.  These must be confined to the
 * range 0x0000 through 0xffff.
 */

/* Get an environment string.
 *   function getent(name : string) : cstring;
 * ON INPUT:
 *   TOS(0)=length of string
 *   TOS(1)=pointer to string
 * ON RETURN:  actual parameters released
 *   TOS(0,1)=32-bit absolute address of string
 */

#define lbGETENV        (0x0000)

/* Copy pascal string to a pascal string
 *   procedure str2str(src : string; var dest : string)
 * ON INPUT:
 *   TOS(0)=address of dest string
 *   TOS(1)=length of source string
 *   TOS(2)=pointer to source string
 * ON RETURN: actual parameters released.
 */

#define lbSTR2STR       (0x0001)

/* Copy C string to a pascal string
 *   procedure cstr2str(src : cstring; var dest : string)
 * ON INPUT:
 *   TOS(0)=address of dest string
 *   TOS(1,2)=32-bit absolute address of C string
 * ON RETURN: actual parameters released
 */

#define lbCSTR2STR      (0x0002)

/* Copy pascal string to a pascal string reference
 *   procedure str2rstr(src : string; var dest : rstring)
 * ON INPUT:
 *   TOS(0)=address of dest string reference
 *   TOS(1)=length of source string
 *   TOS(2)=pointer to source string
 * ON RETURN: actual parameters released.
 */

#define lbSTR2RSTR      (0x0003)

/* Copy C string to a pascal string reference
 *   procedure cstr2str(src : cstring; var dest : string)
 * ON INPUT:
 *   TOS(0)=address of dest string reference
 *   TOS(0)=MS 16-bits of 32-bit C source string pointer
 *   TOS(1)=LS 16-bits of 32-bit C source string pointer
 * ON RETURN: actual parameters released
 */

#define lbCSTR2RSTR     (0x0004)

/* Convert a string to a numeric value
 *   procedure val(const s : string; var v; var code : word); 
 *
 * Description:
 * val() converts the value represented in the string S to a numerical
 * value, and stores this value in the variable V, which can be of type
 * Longint, Real and Byte. If the conversion isn¡Çt succesfull, then the
 * parameter Code contains the index of the character in S which
 * prevented the conversion. The string S is allowed to contain spaces
 * in the beginning.
 *
 * The string S can contain a number in decimal, hexadecimal, binary or
 * octal format, as described in the language reference.
 *
 * Errors:
 * If the conversion doesn¡Çt succeed, the value of Code indicates the
 * position where the conversion went wrong.
 *
 * ON INPUT
 *   TOS(0)=address of Code
 *   TOS(1)=address of v
 *   TOS(2)=length of source string
 *   TOS(3)=pointer to source string
 * ON RETURN: actual parameters released
 */

#define lbVAL           (0x0005)

/* Create an empty string
 *   function mkstk : string;
 * ON INPUT
 * ON RETURN
 *   TOS(0)=length of new string
 *   TOS(1)=pointer to new string
 */

#define lbMKSTK         (0x0006)

/* Replace a string with a duplicate string residing in allocated
 * string stack.
 *   function mkstkstr(name : string) : string;
 * ON INPUT
 *   TOS(0)=length of original string
 *   TOS(1)=pointer to original string
 * ON RETURN
 *  TOS(0)=length of new string
 *  TOS(1)=pointer to new string
 */

#define lbMKSTKSTR      (0x0007)

/* Replace a character with a string residing in allocated string stack.
 *   function mkstkc(c : char) : string;
 * ON INPUT
 *   TOS(0)=Character value
 * ON RETURN
 *   TOS(0)=length of new string
 *   TOS(1)=pointer to new string
 */

#define lbMKSTKC        (0x0008)

/* Concatenate a string to the end of a string.
 *   function strcat(name : string, c : char) : string;
 * ON INPUT
 *   TOS(0)=length of string
 *   TOS(1)=pointer to string
 *   TOS(2)=length of string
 *   TOS(3)=pointer to string
 * ON OUTPUT
 *   TOS(1)=new length of string
 *   TOS(2)=pointer to string
 */

#define lbSTRCAT        (0x0009)

/* Concatenate a character to the end of a string.
 *   function strcatc(name : string, c : char) : string;
 * ON INPUT
 *   TOS(0)=character to concatenate
 *   TOS(1)=length of string
 *   TOS(2)=pointer to string
 * ON OUTPUT
 *   TOS(1)=new length of string
 *   TOS(2)=pointer to string
 */

#define lbSTRCATC       (0x000a)

/* Compare two pascal strings
 *   function strcmp(name1 : string, name2 : string) : integer;
 * ON INPUT
 *   TOS(1)=length of string2
 *   TOS(2)=address of string2 data
 *   TOS(3)=length of string1
 *   TOS(4)=address of string1 data
 * ON OUTPUT
 *   TOS(0)=(-1=less than, 0=equal, 1=greater than} 
 */

#define lbSTRCMP        (0x000b)

#define MAX_LBOP        (0x000c)

#endif /* __PXDEFS_H */
