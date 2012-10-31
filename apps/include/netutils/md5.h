/****************************************************************************
 * apps/include/netutils/md5.h
 *
 * This file is part of the NuttX RTOS:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Darcy Gong
 *
 * Reference:
 *
 *   This code implements the MD5 message-digest algorithm.
 *   The algorithm is due to Ron Rivest.  This code was
 *   written by Colin Plumb in 1993, no copyright is claimed.
 *   This code is in the public domain; do with it what you wish.
 *
 *   Equivalent code is available from RSA Data Security, Inc.
 *   This code has been tested against that, and is equivalent,
 *   except that you don't need to include two pages of legalese
 *   with every copy.
 *
 *   To compute the message digest of a chunk of bytes, declare an
 *   MD5Context structure, pass it to MD5Init, call MD5Update as
 *   needed on buffers full of bytes, and then call MD5Final, which
 *   will fill a supplied 16-byte array with the digest.
 *
 *   See README and COPYING for more details.
 *
 * And is re-released under the NuttX modified BSD license:
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the Institute nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 *   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *   ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 *   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 *   OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 *   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 *   OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 *   SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_NETUTILS_MD5_H
#define __APPS_INCLUDE_NETUTILS_MD5_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_CODECS_HASH_MD5

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct MD5Context
{
  uint32_t buf[4];
  uint32_t bits[2];
  uint8_t in[64];
};

typedef struct MD5Context MD5_CTX;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void MD5Init(struct MD5Context *context);
void MD5Update(struct MD5Context *context, unsigned char const *buf,
               unsigned len);
void MD5Final(unsigned char digest[16], struct MD5Context *context);
void MD5Transform(uint32_t buf[4], uint32_t const in[16]);

void md5_sum(const uint8_t *addr, const size_t len, uint8_t *mac);
char *md5_hash(const uint8_t *addr, const size_t len);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_CODECS_HASH_MD5 */
#endif /* __APPS_INCLUDE_NETUTILS_MD5_H */
