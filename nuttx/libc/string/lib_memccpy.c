/****************************************************************************
 * libc/string/lib_memccpy.c
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

/****************************************************************************
 * Compilation Switches
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <string.h>

/****************************************************************************
 * Global Functions
 ****************************************************************************/


/****************************************************************************
 * Name: memccpy
 *
 * Description:
 *   The memccpy() function copies bytes from memory area s2 into s1,
 *   stopping after the first occurrence of byte c (converted to an unsigned
 *   char) is copied, or after n bytes are copied, whichever comes first. If
 *   copying takes place between objects that overlap, the behavior is
 *   undefined.
 *
 * Returned Value:
 *   The memccpy() function returns a pointer to the byte after the copy of c
 *   in s1, or a null pointer if c was not found in the first n bytes of s2.
 *
 ****************************************************************************/

FAR void *memccpy(FAR void *s1, FAR const void *s2, int c, size_t n)
{
  FAR unsigned char *pout = (FAR unsigned char*)s1;
  FAR unsigned char *pin  = (FAR unsigned char*)s2;

  /* Copy at most n bytes */

  while (n-- > 0)
    {
      /* Copy one byte */

      *pout = *pin++;

      /* Did we just copy the terminating byte c? */

      if (*pout == (unsigned char)c)
        {
          /* Yes return a pointer to the byte after the copy of c into s1 */

          return (FAR void *)pout;
        }

      /* No increment to the next destination location */

      pout++;
    }

  /* C was not found in the first n bytes of s2 */

  return NULL;
}
