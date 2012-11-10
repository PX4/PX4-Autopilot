
/****************************************************************************
 * libc/string/lib_memset.c
 *
 *   Copyright (C) 2007, 2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Can't support CONFIG_MEMSET_64BIT if the platform does not have 64-bit
 * integer types.
 */

#ifndef CONFIG_HAVE_LONG_LONG
#  undef CONFIG_MEMSET_64BIT
#endif

/****************************************************************************
 * Global Functions
 ****************************************************************************/

#ifndef CONFIG_ARCH_MEMSET
void *memset(void *s, int c, size_t n)
{
#ifdef CONFIG_MEMSET_OPTSPEED
  /* This version is optimized for speed (you could do better
   * still by exploiting processor caching or memory burst
   * knowledge.)
   */

  uintptr_t addr  = (uintptr_t)s;
  uint16_t  val16 = ((uint16_t)c << 8) | (uint16_t)c;
  uint32_t  val32 = ((uint32_t)val16 << 16) | (uint32_t)val16;
#ifdef CONFIG_MEMSET_64BIT
  uint64_t  val64 = ((uint64_t)val32 << 32) | (uint64_t)val32;
#endif

  /* Make sure that there is something to be cleared */

  if (n > 0)
    {
      /* Align to a 16-bit boundary */

      if ((addr & 1) != 0)
        {
          *(uint8_t*)addr = (uint8_t)c;
          addr += 1;
          n    -= 1;
        }

      /* Check if there are at least 16-bits left to be written */

      if (n >= 2)
        {
          /* Align to a 32-bit boundary (we know that the destination
           * address is already aligned to at least a 16-bit boundary).
           */

          if ((addr & 3) != 0)
            {
              *(uint16_t*)addr = val16;
              addr += 2;
              n    -= 2;
            }

#ifndef CONFIG_MEMSET_64BIT
          /* Loop while there are at least 32-bits left to be written */

          while (n >= 4)
            {
              *(uint32_t*)addr = val32;
              addr += 4;
              n    -= 4;
            }
#else
          /* Check if there are at least 32-bits left to be written */

          if (n >= 4)
            {
              /* Align to a 64-bit boundary (we know that the destination
               * address is already aligned to at least a 32-bit boundary).
               */

              if ((addr & 7) != 0)
                {
                  *(uint32_t*)addr = val32;
                  addr += 4;
                  n    -= 4;
                }

              /* Loop while there are at least 64-bits left to be written */

              while (n >= 8)
                {
                  *(uint64_t*)addr = val64;
                  addr += 8;
                  n    -= 8;
                }
            }
#endif
        }

#ifdef CONFIG_MEMSET_64BIT
      /* We may get here with n in the range 0..7.  If n >= 4, then we should
       * have 64-bit alignment.
       */

      if (n >= 4)
        {
          *(uint32_t*)addr = val32;
          addr += 4;
          n    -= 4;
        }
#endif

      /* We may get here under the following conditions:
       *
       *   n = 0, addr may or may not be aligned
       *   n = 1, addr is aligned to at least a 16-bit boundary
       *   n = 2, addr is aligned to a 32-bit boundary
       *   n = 3, addr is aligned to a 32-bit boundary
       */

      if (n >= 2)
        {
          *(uint16_t*)addr = val16;
          addr += 2;
          n    -= 2;
        }

      if (n >= 1)
        {
          *(uint8_t*)addr = (uint8_t)c;
        }
    }
#else
  /* This version is optimized for size */

  unsigned char *p = (unsigned char*)s;
  while (n-- > 0) *p++ = c;
#endif
  return s;
}
#endif
