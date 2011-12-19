/****************************************************************************
 * include/ctype.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __INCLUDE_CTYPE_H
#define __INCLUDE_CTYPE_H

/* There is no consistent ctype implementation, just a
 * smattering of functions.  Individually, they are okay, but
 * a more standard, data lookup approach would make more sense
 * if used extensively.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Function:  isspace
 *
 * Description:
 * Checks  for  white-space characters.  In the "C" and "POSIX"
 * locales, these are: space, form-feed ('\f'), newline ('\n'),
 * carriage return ('\r'), horizontal tab ('\t'), and vertical
 * tab ('\v').
 *
 ****************************************************************************/

#define isspace(c) \
  ((c) == ' '  || (c) == '\t' || (c) == '\n' || \
   (c) == '\r' || (c) == '\f' || c== '\v')

/****************************************************************************
 * Function:  isascii
 *
 * Description:
 *  Checks whether c is a 7-bit unsigned char value that
 *  fits into the ASCII character set.
 *
 ****************************************************************************/

#define isascii(c)   ((c) >= 0 && (c) <= 0x7f);

/****************************************************************************
 * Function:  isprint
 *
 * Description:
 *  Checks for a printable character (including space)
 *
 ****************************************************************************/

#define isprint(c)   ((c) >= 0x20 && (c) < 0x7f)

/****************************************************************************
 * Function:  isgraph
 *
 * Description:
 *  Checks for a printable character (excluding space)
 *
 ****************************************************************************/

#define isgraph(c)   ((c) > 0x20 && (c) < 0x7f)

/****************************************************************************
 * Function:  iscntrl
 *
 * Description:
 *  Checks for control character.
 *
 ****************************************************************************/

#define iscontrol(c) (!isprint(c))

/****************************************************************************
 * Function:  islower
 *
 * Description:
 *    Checks for an lowercase letter.
 *
 ****************************************************************************/

#define islower(c)   ((c) >= 'a' && (c) <= 'z')

/****************************************************************************
 * Function:  isupper
 *
 * Description:
 *    Checks for an uppercase letter.
 *
 ****************************************************************************/

#define isupper(c)   ((c) >= 'A' && (c) <= 'Z')

/****************************************************************************
 * Function:  isalpha
 *
 * Description:
 *    Checks for an alphabetic character
 *
 ****************************************************************************/

#define isalpha(c)   (islower(c) || isupper(c))

/****************************************************************************
 * Function:  isdigit
 *
 * Description:
 *    ANSI standard isdigit implementation.
 *
 ****************************************************************************/

#define isdigit(c)   ((c) >= '0' && (c) <= '9')

/****************************************************************************
 * Function:  isalnum
 *
 * Description:
 *    Checks for an alphanumeric character
 *
 ****************************************************************************/

#define isalnum(c)   (isalpha(c) || isdigit(c))

/****************************************************************************
 * Function:  ispunct
 *
 * Description:
 *  Checks for a printable character which is not a space
 *  or an alphanumeric character
 *
 ****************************************************************************/

#define ispunct(c)   (isgraph(c) && !isalnum(c))

/****************************************************************************
 * Function:  isxdigit
 *
 * Description:
 *   isxdigit() checks for a hexadecimal digits, i.e. one of
 *   {0-9,a-f,A-F}
 *
 ****************************************************************************/

#define isxdigit(c) \
  (((c) >= '0' && (c) <= '9') || \
   ((c) >= 'a' && (c) <= 'f') || \
   ((c) >= 'A' && (c) <= 'F'))

/****************************************************************************
 * Function:  toupper
 *
 * Description:
 *   toupper() converts the letter c to upper case, if possible.
 *
 ****************************************************************************/

#define toupper(c) \
  (((c) >= 'a' && (c) <= 'z') ? ((c) - 'a' + 'A') : (c))

/****************************************************************************
 * Function:  tolower
 *
 * Description:
 *   tolower() converts the letter c to lower case, if possible.
 *
 ****************************************************************************/

#define tolower(c) \
  (((c) >= 'A' && (c) <= 'Z') ? ((c) - 'A' + 'a') : (c))

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_CTYPE_H */
