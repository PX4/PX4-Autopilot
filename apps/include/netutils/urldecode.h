/****************************************************************************
 * apps/include/netutils/urldecode.h
 *
 * This file is part of the NuttX RTOS:
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Darcy Gong
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_NETUTILS_URLDECODE_H
#define __APPS_INCLUDE_NETUTILS_URLDECODE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_CODECS_URLCODE_NEWMEMORY
char *url_encode(char *str);
char *url_decode(char *str);
#endif /* CONFIG_CODECS_URLCODE_NEWMEMORY */

#ifdef CONFIG_CODECS_URLCODE
char *urlencode(const char *src, const int src_len, char *dest, int *dest_len);
char *urldecode(const char *src, const int src_len, char *dest, int *dest_len);
int urlencode_len(const char *src, const int src_len);
int urldecode_len(const char *src, const int src_len);
#endif /* CONFIG_CODECS_URLCODE */

#ifdef CONFIG_CODECS_AVR_URLCODE
void urlrawdecode(char *urlbuf);
void urlrawencode(char *str,char *urlbuf);
#endif /* CONFIG_CODECS_AVR_URLCODE */

#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_URLDECODE_H */

