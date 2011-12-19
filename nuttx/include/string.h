/****************************************************************************
 * include/string.h
 *
 *   Copyright (C) 2007-2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_STRING_H
#define __INCLUDE_STRING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stddef.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

EXTERN char  *strchr(const char *s, int c);
EXTERN FAR char *strdup(const char *s);
EXTERN FAR char *strndup(FAR const char *s, size_t size);
EXTERN const char *strerror(int);
EXTERN size_t strlen(const char *);
EXTERN size_t strnlen(const char *, size_t);
EXTERN char  *strcat(char *, const char *);
EXTERN char  *strncat(char *, const char *, size_t);
EXTERN int    strcmp(const char *, const char *);
EXTERN int    strncmp(const char *, const char *, size_t);
EXTERN int    strcasecmp(const char *, const char *);
EXTERN int    strncasecmp(const char *, const char *, size_t);
EXTERN char  *strcpy(char *dest, const char *src);
EXTERN char  *strncpy(char *, const char *, size_t);
EXTERN char  *strpbrk(const char *, const char *);
EXTERN char  *strchr(const char *, int);
EXTERN char  *strrchr(const char *, int);
EXTERN size_t strspn(const char *, const char *);
EXTERN size_t strcspn(const char *, const char *);
EXTERN char  *strstr(const char *, const char *);
EXTERN char  *strtok(char *, const char *);
EXTERN char  *strtok_r(char *, const char *, char **);

EXTERN void  *memset(void *s, int c, size_t n);
EXTERN void  *memcpy(void *dest, const void *src, size_t n);
EXTERN int    memcmp(const void *s1, const void *s2, size_t n);
EXTERN void  *memmove(void *dest, const void *src, size_t count);

#ifndef CONFIG_ARCH_BZERO
# define bzero(s,n) (void)memset(s,0,n)
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_STRING_H */
