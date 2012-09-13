/****************************************************************************
 * netutils/thttpd/thttpd_alloc.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __NETUTILS_THTTPD_HTTDP_ALLOC_H
#define __NETUTILS_THTTPD_HTTDP_ALLOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/* Allows all memory management calls to be intercepted */

#ifdef CONFIG_THTTPD_MEMDEBUG
extern FAR void *httpd_malloc(size_t nbytes);
extern FAR void *httpd_realloc(FAR void *oldptr, size_t oldsize, size_t newsize);
extern void      httpd_free(FAR void *ptr);
extern FAR char *httpd_strdup(const char *str);
#else
#  define httpd_malloc(n)      malloc(n)
#  define httpd_realloc(p,o,n) realloc(p,n)
#  define httpd_free(p)        free(p)
#  define httpd_strdup(s)      strdup(s)
#endif

/* Helpers to support allocations in multiples of a type size */

#define NEW(t,n)               ((t*)httpd_malloc(sizeof(t)*(n)))
#define RENEW(p,t,o,n)         ((t*)httpd_realloc((void*)p, sizeof(t)*(o), sizeof(t)*(n)))

/* Helpers to implement dynamically allocated strings */

extern void httpd_realloc_str(char **pstr, size_t *maxsizeP, size_t size);

#endif /* CONFIG_THTTPD */
#endif /* __NETUTILS_THTTPD_HTTDP_ALLOC_H */
