/****************************************************************************
 * include/nuttx/ascii.h
 * ASCII Control Codes
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

#ifndef __INCLUDE_NUTTX_ASCII_H
#define __INCLUDE_NUTTX_ASCII_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* All 7-bit ASCII codes */

#define ASCII_NUL        0x00 /* Null character (^@) */
#define ASCII_SOH        0x01 /* Start of header (^A) */
#define ASCII_STX        0x02 /* Start of tex (^B) */
#define ASCII_ETX        0x03 /* End of text (^C) */
#define ASCII_EOT        0x04 /* End of transmission (^D) */
#define ASCII_ENQ        0x05 /* Enquiry (^E) */
#define ASCII_ACK        0x06 /* Acknowledge (^F) */
#define ASCII_BEL        0x07 /* Bell (^G) */
#define ASCII_BS         0x08 /* Backspace (^H) */
#define ASCII_TAB        0x09 /* Horizontal tab (^I) */
#define ASCII_LF         0x0a /* Line Feed (^J) */
#define ASCII_VT         0x0b /* Vertical tab(^K) */
#define ASCII_FF         0x0c /* Form Feed (^L) */
#define ASCII_CR         0x0d /* Carriage Return (^M) */
#define ASCII_SO         0x0e /* Shift Out (^N) */
#define ASCII_SI         0x0f /* Shift In (^O) */

#define ASCII_DLE        0x10 /* Data link escape (^P) */
#define ASCII_DC1        0x11 /* XON (^Q) */
#define ASCII_DC2        0x12 /* Device control 2, block-mode flow control (^R) */
#define ASCII_DC3        0x13 /* XOFF (^S) */
#define ASCII_DC4        0x14 /* Device control 4  (^T) */
#define ASCII_NAK        0x15 /* Negative acknowledge (^U) */
#define ASCII_SYN        0x16 /* Synchronous idle (^V) */
#define ASCII_ETB        0x17 /* End transmission block(^W) */
#define ASCII_CAN        0x18 /* Cancel line(^X) */
#define ASCII_EM         0x19 /* End of medium(^Y) */
#define ASCII_SUB        0x1a /* Substitute (^Z) */
#define ASCII_ESC        0x1b /* Escape (^[) */
#define ASCII_FS         0x1c /* File separator (^\) */
#define ASCII_GS         0x1d /* Group separator (^]) */
#define ASCII_RS         0x1e /* Record separator, block-mode terminator (^^) */
#define ASCII_US         0x1f /* Unit separator (^_) */

#define ASCII_SPACE      0x20 /* Space ( ) */
#define ASCII_EXCLAM     0x21 /* Exclamation mark (!) */
#define ASCII_QUOTE      0x22 /* Quotation mark (") */
#define ASCII_NUMBER     0x23 /* Number sign (#) */
#define ASCII_HASH       0x23 /* Hash (#) */
#define ASCII_DOLLAR     0x24 /* Dollar sign ($) */
#define ASCII_PERCENT    0x25 /* Percent sign (%) */
#define ASCII_AMPERSAND  0x26 /* Ampersand (&) */
#define ASCII_RSQUOTE    0x27 /* Closing single quote (') */
#define ASCII_APOSTROPHE 0x27 /* Apostrophe (') */
#define ASCII_LPAREN     0x28 /* Opening parenthesis (() */
#define ASCII_RPAREN     0x29 /* Closing parenthesis ()) */
#define ASCII_ASTERISK   0x2a /* Asterisk (*) */
#define ASCII_PLUS       0x2b /* Plus sign (+) */
#define ASCII_COMMA      0x2c /* Comma (,) */
#define ASCII_HYPHEN     0x2d /* Hyphen (-) */
#define ASCII_DASH       0x2d /* Dash (-) */
#define ASCII_MINUS      0x2d /* Minus sign (-) */
#define ASCII_PERIOD     0x2e /* Period (.) */
#define ASCII_SLASH      0x2f /* Forward Slash (/) */
#define ASCII_DIVIDE     0x2f /* Divide (/) */

#define ASCII_0          0x30 /* Numbers */
#define ASCII_1          0x31 /* "     " */
#define ASCII_2          0x32 /* "     " */
#define ASCII_3          0x33 /* "     " */
#define ASCII_4          0x34 /* "     " */
#define ASCII_5          0x35 /* "     " */
#define ASCII_6          0x36 /* "     " */
#define ASCII_7          0x37 /* "     " */
#define ASCII_8          0x38 /* "     " */
#define ASCII_9          0x39 /* "     " */
#define ASCII_COLON      0x3a /* Colon (:) */
#define ASCII_SEMICOLON  0x3b /* Semicolon (;) */
#define ASCII_LT         0x3c /* Less than (<) */
#define ASCII_EQUAL      0x3d /* Equal (=) */
#define ASCII_GT         0x3e /* Greater than (>) */
#define ASCII_QUESTION   0x3f /* Question mark (?) */

#define ASCII_AT         0x40 /* At sign (@) */
#define ASCII_A          0x41 /* Upper case letters */
#define ASCII_B          0x42 /* "   " "  " "     " */
#define ASCII_C          0x43 /* "   " "  " "     " */
#define ASCII_D          0x44 /* "   " "  " "     " */
#define ASCII_E          0x45 /* "   " "  " "     " */
#define ASCII_F          0x46 /* "   " "  " "     " */
#define ASCII_G          0x47 /* "   " "  " "     " */
#define ASCII_H          0x48 /* "   " "  " "     " */
#define ASCII_I          0x49 /* "   " "  " "     " */
#define ASCII_J          0x4a /* "   " "  " "     " */
#define ASCII_K          0x4b /* "   " "  " "     " */
#define ASCII_L          0x4c /* "   " "  " "     " */
#define ASCII_M          0x4d /* "   " "  " "     " */
#define ASCII_N          0x4e /* "   " "  " "     " */
#define ASCII_O          0x4f /* "   " "  " "     " */

#define ASCII_P          0x50 /* "   " "  " "     " */
#define ASCII_Q          0x51 /* "   " "  " "     " */
#define ASCII_R          0x52 /* "   " "  " "     " */
#define ASCII_S          0x53 /* "   " "  " "     " */
#define ASCII_T          0x54 /* "   " "  " "     " */
#define ASCII_U          0x55 /* "   " "  " "     " */
#define ASCII_V          0x56 /* "   " "  " "     " */
#define ASCII_W          0x57 /* "   " "  " "     " */
#define ASCII_X          0x58 /* "   " "  " "     " */
#define ASCII_Y          0x59 /* "   " "  " "     " */
#define ASCII_Z          0x5a /* "   " "  " "     " */
#define ASCII_LBRACKET   0x5b /* Left bracket ([) */
#define ASCII_BACKSLASH  0x5c /* Back slash (\) */
#define ASCII_RBRACKET   0x5d /* Right bracket (]) */
#define ASCII_CARET      0x5c /* Caret (^) */
#define ASCII_CIRCUMFLEX 0x5c /* Circumflex (^) */
#define ASCII_UNDERSCORE 0x5f /* Underscore (_) */

#define ASCII_RSQUOT     0x60 /* Closing single quote */
#define ASCII_a          0x61 /* Lower case letters */
#define ASCII_b          0x62 /* "   " "  " "     " */
#define ASCII_c          0x63 /* "   " "  " "     " */
#define ASCII_d          0x64 /* "   " "  " "     " */
#define ASCII_e          0x65 /* "   " "  " "     " */
#define ASCII_f          0x66 /* "   " "  " "     " */
#define ASCII_g          0x67 /* "   " "  " "     " */
#define ASCII_h          0x68 /* "   " "  " "     " */
#define ASCII_i          0x69 /* "   " "  " "     " */
#define ASCII_j          0x6a /* "   " "  " "     " */
#define ASCII_k          0x6b /* "   " "  " "     " */
#define ASCII_l          0x6c /* "   " "  " "     " */
#define ASCII_m          0x6d /* "   " "  " "     " */
#define ASCII_n          0x6e /* "   " "  " "     " */
#define ASCII_o          0x6f /* "   " "  " "     " */

#define ASCII_p          0x70 /* "   " "  " "     " */
#define ASCII_q          0x71 /* "   " "  " "     " */
#define ASCII_r          0x72 /* "   " "  " "     " */
#define ASCII_s          0x73 /* "   " "  " "     " */
#define ASCII_t          0x74 /* "   " "  " "     " */
#define ASCII_u          0x75 /* "   " "  " "     " */
#define ASCII_v          0x76 /* "   " "  " "     " */
#define ASCII_w          0x77 /* "   " "  " "     " */
#define ASCII_x          0x78 /* "   " "  " "     " */
#define ASCII_y          0x79 /* "   " "  " "     " */
#define ASCII_z          0x7a /* "   " "  " "     " */
#define ASCII_LBRACE     0x7b /* Left brace ({) */
#define ASCII_VERTBAR    0x7c /* Vertical bar (|) */
#define ASCII_PIPE       0x7c /* Pipe (|) */
#define ASCII_RBRACE     0x7d /* Right brace (}) */
#define ASCII_TILDE      0x7e /* Tilde (~) */
#define ASCII_DEL        0x7f /* Delete (rubout) */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_ASCII_H */
