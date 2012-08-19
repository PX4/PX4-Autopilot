/*
Copyright (c) 2008-2012, Troy D. Hanson   http://uthash.sourceforge.net
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* a dynamic string implementation using macros 
 * see http://uthash.sourceforge.net/utstring
 */
#ifndef UTSTRING_H
#define UTSTRING_H

#define UTSTRING_VERSION 1.9.6

#ifdef __GNUC__
#define _UNUSED_ __attribute__ ((__unused__)) 
#else
#define _UNUSED_ 
#endif

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#define oom() exit(-1)

typedef struct {
    char *d;
    size_t n; /* allocd size */
    size_t i; /* index of first unused byte */
} UT_string;

#define utstring_reserve(s,amt)                            \
do {                                                       \
  if (((s)->n - (s)->i) < (size_t)(amt)) {                 \
     (s)->d = (char*)realloc((s)->d, (s)->n + amt);        \
     if ((s)->d == NULL) oom();                            \
     (s)->n += amt;                                        \
  }                                                        \
} while(0)

#define utstring_init(s)                                   \
do {                                                       \
  (s)->n = 0; (s)->i = 0; (s)->d = NULL;                   \
  utstring_reserve(s,100);                                 \
  (s)->d[0] = '\0'; \
} while(0)

#define utstring_done(s)                                   \
do {                                                       \
  if ((s)->d != NULL) free((s)->d);                        \
  (s)->n = 0;                                              \
} while(0)

#define utstring_free(s)                                   \
do {                                                       \
  utstring_done(s);                                        \
  free(s);                                                 \
} while(0)

#define utstring_new(s)                                    \
do {                                                       \
   s = (UT_string*)calloc(sizeof(UT_string),1);            \
   if (!s) oom();                                          \
   utstring_init(s);                                       \
} while(0)

#define utstring_renew(s)                                  \
do {                                                       \
   if (s) {                                                \
     utstring_clear(s);                                    \
   } else {                                                \
     utstring_new(s);                                      \
   }                                                       \
} while(0)

#define utstring_clear(s)                                  \
do {                                                       \
  (s)->i = 0;                                              \
  (s)->d[0] = '\0';                                        \
} while(0)

#define utstring_bincpy(s,b,l)                             \
do {                                                       \
  utstring_reserve((s),(l)+1);                               \
  if (l) memcpy(&(s)->d[(s)->i], b, l);                    \
  (s)->i += l;                                               \
  (s)->d[(s)->i]='\0';                                         \
} while(0)

#define utstring_concat(dst,src)                                 \
do {                                                             \
  utstring_reserve((dst),((src)->i)+1);                          \
  if ((src)->i) memcpy(&(dst)->d[(dst)->i], (src)->d, (src)->i); \
  (dst)->i += (src)->i;                                          \
  (dst)->d[(dst)->i]='\0';                                       \
} while(0)

#define utstring_len(s) ((unsigned)((s)->i))

#define utstring_body(s) ((s)->d)

_UNUSED_ static void utstring_printf_va(UT_string *s, const char *fmt, va_list ap) {
   int n;
   va_list cp;
   while (1) {
#ifdef _WIN32
      cp = ap;
#else
      va_copy(cp, ap);
#endif
      n = vsnprintf (&s->d[s->i], s->n-s->i, fmt, cp);
      va_end(cp);

      if ((n > -1) && (n < (int)(s->n-s->i))) {
        s->i += n;
        return;
      }

      /* Else try again with more space. */
      if (n > -1) utstring_reserve(s,n+1); /* exact */
      else utstring_reserve(s,(s->n)*2);   /* 2x */
   }
}
_UNUSED_ static void utstring_printf(UT_string *s, const char *fmt, ...) {
   va_list ap;
   va_start(ap,fmt);
   utstring_printf_va(s,fmt,ap);
   va_end(ap);
}

#endif /* UTSTRING_H */
