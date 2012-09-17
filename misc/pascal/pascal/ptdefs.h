/***********************************************************************
 * ptdefs.h
 * Token and Symbol Table Definitions
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __PTDEFS_H
#define __PTDEFS_H

/***********************************************************************/
/* Token Values 0-0x20 reserved for get_token identification */

#define tIDENT           0x01
#define tINT_CONST       0x02
#define tCHAR_CONST      0x03
#define tBOOLEAN_CONST   0x04
#define tREAL_CONST      0x05
#define tSTRING_CONST    0x06

#define tLE              0x07
#define tGE              0x08
#define tASSIGN          0x09
#define tSUBRANGE        0x0A

/* Token Values 0x21-0x2F (except 0x24) are for ASCII character tokens */

#define tNE              ('#')
#define SQUOTE           0x27
#define tMUL             ('*')
#define tFDIV            ('/')

/* Token Values 0x30-0x39 are spare */
/* Token Values 0x3A-0x40 are for ASCII character tokens */

#define tLT              ('<')
#define tEQ              ('=')
#define tGT              ('>')

/* Token Values 0x41-0x5A are SYMBOL TABLE definitions */

#define sPROC            0x41
#define sFUNC            0x42
#define sLABEL           0x43
#define sTYPE            0x44
#define sFILE            0x45
#define sINT             0x46
#define sBOOLEAN         0x47
#define sCHAR            0x48
#define sREAL            0x49
#define sTEXT            0x4a
#define sSTRING          0x4b /* String storage type */
#define sRSTRING         0x4c /* String reference type */
#define sSTRING_CONST    0x4d
#define sPOINTER         0x4e
#define sSCALAR          0x4f
#define sSCALAR_OBJECT   0x50
#define sSUBRANGE        0x51
#define sSET_OF          0x52
#define sARRAY           0x53
#define sRECORD          0x54
#define sRECORD_OBJECT   0x55
#define sFILE_OF         0x56
#define sVAR_PARM        0x57

/* Token Values 0x5B-0x60 (except 0x5F) are for ASCII character tokens */
/* Token Values 0x61-0x7a are SYMBOL TABLE definitions */

/* Token Values 0x7b-0x7f are for ASCII character tokens */
/* Token Value  0x7f is spare */

/* Token Values 0x80-0xef are for RESERVED WORDS */

/* Standard constants (TRUE, FALSE, MAXINT) and standard files (INPUT, OUTPUT) 
 * are hard initialized into the constant/symbol table and are transparent
 * to the compiler */

/* Reserved Words 0x80-0xaf*/

#define tAND             0x80
#define tARRAY           0x81
#define tBEGIN           0x82
#define tCASE            0x83
#define tCONST           0x84
#define tDIV             0x85
#define tDO              0x86
#define tDOWNTO          0x87
#define tELSE            0x88
#define tEND             0x89
#define tFILE            0x8a
#define tFOR             0x8b
#define tFUNCTION        0x8c
#define tGOTO            0x8d
#define tIF              0x8e
#define tIMPLEMENTATION 0x08f /* Extended pascal */
#define tIN              0x90
#define tINTERFACE       0x91 /* Extended pascal */
#define tLABEL           0x92
#define tMOD             0x93
#define tNIL             0x94
#define tNOT             0x95
#define tOF              0x96
#define tOR              0x97
#define tPACKED          0x98
#define tPROCEDURE       0x99
#define tPROGRAM         0x9a
#define tRECORD          0x9b
#define tREPEAT          0x9c
#define tSET             0x9d
#define tSHL             0x9e
#define tSHR             0x9f
#define tTHEN            0xa0
#define tTO              0xa1
#define tTYPE            0xa2
#define tUNIT            0xa3 /* Extended pascal */
#define tUNTIL           0xa4
#define tUSES            0xa5 /* Extended pascal */
#define tVAR             0xa6
#define tWHILE           0xa7
#define tWITH            0xa8

/* The following codes indicate that the token is a built-in procedure
 * or function recognized by the compiler.  An additional code will be
 * place in tknSubType by the tokenizer to indicate which built-in
 * procedure or function applies.
 */

#define tFUNC            0xb0
#define tPROC            0xb1

/***********************************************************************/
/* Codes to indentify built-in functions and procedures */

#define txNONE           0x00

/* Standard Functions 0x01-0x1f*/

#define txABS            0x01
#define txARCTAN         0x02
#define txCHR            0x03
#define txCOS            0x04
#define txEOF            0x05
#define txEOLN           0x06
#define txEXP            0x07
#define txLN             0x08
#define txODD            0x09
#define txORD            0x0a
#define txPRED           0x0b
#define txROUND          0x0c
#define txSIN            0x0d
#define txSQR            0x0e
#define txSQRT           0x0f
#define txSUCC           0x10
#define txTRUNC          0x11

/* "Less than standard" Functions 0x20-0x7f */

#define txGETENV         0x20

/* Standard Procedures 0x81-0xbf */

#define txGET            0x80
#define txNEW            0x81
#define txPACK           0x82
#define txPAGE           0x83
#define txPUT            0x84
#define txREAD           0x85
#define txREADLN         0x86
#define txRESET          0x87
#define txREWRITE        0x88
#define txUNPACK         0x89
#define txWRITE          0x8a
#define txWRITELN        0x8b

/* "Less than standard" Procedures 0xc0-0xff */

#define txVAL            0xc0

#endif /* __PTDEFS_H */

