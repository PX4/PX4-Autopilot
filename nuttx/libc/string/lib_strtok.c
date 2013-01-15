/****************************************************************************
 * libc/string/lib_strtok.c
 *
 *   Copyright (C) 2007, 2008, 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <string.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_saveptr = NULL;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: strtok
 *
 * Description:
 *    The  strtok()  function  parses  a string into a
 *    sequence of tokens.  On the first call to strtok() the
 *    string to be parsed should be specified in 'str'.  In
 *    each subsequent call that should parse the same string, 
 *    'str' should be NULL.
 *
 *    The 'delim' argument specifies a set of characters that
 *    delimit the tokens in the parsed string.  The caller
 *    may specify different strings in delim in successive
 *    calls that parse the same string.
 *
 *    Each call to strtok() returns a pointer to a null-
 *    terminated string containing the next token. This
 *    string  does not include the delimiting character.  If
 *    no more tokens are found, strtok() returns NULL.
 *
 *    A sequence of two or more contiguous delimiter
 *    characters in the parsed string is considered to be a
 *    single delimiter. Delimiter characters at the start or
 *    end of the string are ignored.  The tokens returned by
 *    strtok() are always non-empty strings.
 *
 * Return
 *    strtok() returns a pointer to the next token, or NULL
 *    if there are no more tokens.
 *
 ****************************************************************************/

char *strtok(char *str, const char *delim)
{
  return strtok_r(str, delim, &g_saveptr);
}
