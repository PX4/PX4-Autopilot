/****************************************************************************
 * libc/net/lib_inetntop.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Includes some logic extracted from hwport_ftpd, written by Jaehyuk Cho
 * <minzkn@minzkn.com> which was released under the BSD license.
 *
 *   Copyright (C) HWPORT.COM. All rights reserved.
 *   Author: JAEHYUK CHO <mailto:minzkn@minzkn.com>
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

#include <sys/socket.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <arpa/inet.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_ntop
 *
 * Description:
 *  The inet_ntop() function converts a numeric address into a text string
 *  suitable for presentation.
 *
 * Input Parameters:
 *   af   - The af argument specifies the family of the address. This can be
 *          AF_INET or AF_INET6.
 *   src  - The src argument points to a buffer holding an address of the
 *          specified type.  The address must be in network byte order.
 *   dst  - The dst argument points to a buffer where the function stores
 *          the resulting text string; it shall not be NULL.
 *   size - The size argument specifies the size of this buffer, which must
 *          be large enough to hold the text string (INET_ADDRSTRLEN
 *          characters for IPv4, INET6_ADDRSTRLEN characters for IPv6).
 *
 * Returned Value:
 *   inet_ntop() returns a pointer to the buffer containing the text string
 *   if the conversion succeeds.  Otherwise, NULL is returned and the errno
 *   is set to indicate the error.  There follow errno values may be set:
 *
 *   EAFNOSUPPORT - The af argument is invalid.
 *   ENOSPC - The size of the inet_ntop() result buffer is inadequate
 *
 ****************************************************************************/

FAR const char *inet_ntop(int af, FAR const void *src, FAR char *dst, socklen_t size)
{
  int errval;
#ifndef CONFIG_NET_IPv6
  FAR char *ptr;

  DEBUGASSERT(src && dst);

  if (af != AF_INET)
    {
      errval = EAFNOSUPPORT;
      goto errout;
    }

  if (size < INET_ADDRSTRLEN)
    {
      errval = ENOSPC;
      goto errout;
    }

  ptr = (FAR char*)src;
  sprintf(dst, "%d.%d.%d.%d", ptr[0], ptr[1], ptr[2], ptr[3]);
  return dst;
#else
  FAR const struct in6_addr *in6_addr;
  uint16_t warray[8];
  int offset;
  int entry;
  int count;
  int maxentry;
  int maxcount;
 
  DEBUGASSERT(src && dst);

  if (af != AF_INET6)
    {
      errval = EAFNOSUPPORT;
      goto errout;
    }

  if (size < INET6_ADDRSTRLEN)
    {
      errval = ENOSPC;
      goto errout;
    }

  in6_addr = (FAR const struct in6_addr *)src;
  entry    = -1;
  maxentry = -1;
  maxcount = 0;
  offset   = 0;

  while (offset < 8)
    {
      warray[offset] = ntohs(in6_addr->s6_addr16[offset]);
      if (warray[offset] == 0)
        {
          entry = offset;
          count = 1;
          offset++;

          while (offset < 8)
            {
              warray[offset] = ntohs(in6_addr->s6_addr16[offset]);
              if (warray[offset] != 0)
                {
                  break;
                }
              offset++;
              count++;
            }

          if (count > maxcount)
            {
              maxentry = entry;
              maxcount = count;
            }
        }
      offset++;
    }

  offset = 0;
  dst[0] = '\0';

  while (offset < 8)
    {
      if (offset == maxentry)
        {
          size   -= snprintf(&dst[strlen(dst)], size, ":");
          offset += maxcount;
          if (offset >= 8)
            {
              size -= snprintf(&dst[strlen(dst)], size, ":");
            }
        }
      else
        {
          if (offset > 0)
            {
              size -= snprintf(&dst[strlen(dst)], size, ":");
            }

          size -= snprintf(&dst[strlen(dst)], size, "%x", warray[offset]);
          offset++;
        }
    }

    return dst;
#endif

errout:
    set_errno(errval);
    memset(dst, 0, size);
    return NULL;
}
