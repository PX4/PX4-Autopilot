/****************************************************************************
 * examples/udp/udp-internal.h
 *
 *   Copyright (C) 2007, 2008 Gregory Nutt. All rights reserved.
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

#ifndef __EXAMPLES_UIP_INTERNAL_H
#define __EXAMPLES_UIP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_UDP_HOST
#else
# include <debug.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

#ifdef CONFIG_EXAMPLES_UDP_HOST
   /* HTONS/L macros are unique to uIP */

#  define HTONS(a)       htons(a)
#  define HTONL(a)       htonl(a)

   /* Used printf for debug output */

#  define message(...)   printf(__VA_ARGS__)

   /* Have SO_LINGER */

#else

   /* If debug is enabled, use the synchronous lowsyslog so that the
    * program output does not get disassociated in the debug output.
    */

#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif

#endif

#define PORTNO     5471

#define ASCIISIZE  (0x7f - 0x20)
#define SENDSIZE   (ASCIISIZE+1)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

extern void send_client(void);
extern void recv_server(void);

#endif /* __EXAMPLES_UIP_INTERNAL_H */
