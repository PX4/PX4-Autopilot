/****************************************************************************
 * libc/net/lib_inetntoa.c
 *
 *   Copyright (C) 2007-2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <stdio.h>
#include <arpa/inet.h>

#ifndef CONFIG_NET_IPv6

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inet_ntoa
 *
 * Description:
 *   The inet_ntoa() function converts the Internet host address in given in
 *   network byte order to a string in standard numbers-and-dots notation.
 *   The string is returned in a statically allocated buffer, which subsequent
 *   calls will overwrite.
 *
 ****************************************************************************/

#ifdef CONFIG_CAN_PASS_STRUCTS
FAR char *inet_ntoa(struct in_addr in)
{
  static char buffer[INET_ADDRSTRLEN+2];
  FAR char *ptr = (FAR char*)&in.s_addr;
  sprintf(buffer, "%d.%d.%d.%d", ptr[0], ptr[1], ptr[2], ptr[3]);
  return buffer;
}
#else
FAR char *_inet_ntoa(in_addr_t in)
{
  static char buffer[INET_ADDRSTRLEN+2];
  FAR char *ptr = (FAR char*)&in;
  sprintf(buffer, "%d.%d.%d.%d", ptr[0], ptr[1], ptr[2], ptr[3]);
  return buffer;
}
#endif
#endif /* !CONFIG_NET_IPv6 */

