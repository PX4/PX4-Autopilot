/****************************************************************************
 * netutils/thttpd/thttpd_alloc.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdlib.h>
#include <debug.h>
#include <errno.h>

#include "config.h"
#include "thttpd_alloc.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_THTTPD_MEMDEBUG
static int    g_nallocations = 0;
static int    g_nfreed       = 0;
static size_t g_allocated    = 0;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Generate debugging statistics */

#ifdef CONFIG_THTTPD_MEMDEBUG
void httpd_memstats(void)
{
  static struct mallinfo mm;

  ndbg("%d allocations (%lu bytes), %d freed\n", g_nallocations, (unsigned long)g_allocated, g_nfreed);

  /* Get the current memory usage */

#ifdef CONFIG_CAN_PASS_STRUCTS
  mm = mallinfo();
#else
  (void)mallinfo(&mm);
#endif
  ndbg("arena: %08x ordblks: %08x mxordblk: %08x uordblks: %08x fordblks: %08x\n",
       mm.arena, mm.ordblks, mm.mxordblk, mm.uordblks, mm.fordblks);
}
#endif

/****************************************************************************
 * Global Functions
 ****************************************************************************/

#ifdef CONFIG_THTTPD_MEMDEBUG
FAR void *httpd_malloc(size_t nbytes)
{
  void *ptr = malloc(nbytes);
  if (!ptr)
    {
      ndbg("Allocation of %d bytes failed\n", nbytes);
    }
  else
    {
      nvdbg("Allocated %d bytes at %p\n", nbytes, ptr);
      g_nallocations++;
      g_allocated += nbytes;
    }
  httpd_memstats();
  return ptr;
}
#endif

#ifdef CONFIG_THTTPD_MEMDEBUG
FAR void *httpd_realloc(FAR void *oldptr, size_t oldsize, size_t newsize)
{
  void *ptr = realloc(oldptr, newsize);
  if (!ptr)
    {
      ndbg("Re-allocation from %d to %d bytes failed\n",
           oldsize, newsize);
    }
  else
    {
      nvdbg("Re-allocated form %d to %d bytes (from %p to %p)\n",
            oldsize, newsize, oldptr, ptr);
      g_allocated += (newsize - oldsize);
    }
  httpd_memstats();
  return ptr;
}
#endif

#ifdef CONFIG_THTTPD_MEMDEBUG
void httpd_free(FAR void *ptr)
{
  free(ptr);
  g_nfreed++;
  nvdbg("Freed memory at %p\n", ptr);
  httpd_memstats();
}
#endif

#ifdef CONFIG_THTTPD_MEMDEBUG
FAR char *httpd_strdup(const char *str)
{
  FAR char *newstr = strdup(str);
  if (!newstr)
    {
      ndbg("strdup of %s failed\n", str);
    }
  else
    {
      nvdbg("strdup'ed %s\n", str);
      g_nallocations++;
      g_allocated += (strlen(str)+1);
    }
  httpd_memstats();
  return newstr;
}
#endif

/* Helpers to implement dynamically allocated strings */

void httpd_realloc_str(char **pstr, size_t *maxsize, size_t size)
{
  size_t oldsize;
  if (*maxsize == 0)
    {
      *maxsize = MAX(CONFIG_THTTPD_MINSTRSIZE, size + CONFIG_THTTPD_REALLOCINCR);
      *pstr    = NEW(char, *maxsize + 1);
    }
  else if (size > *maxsize)
    {
      oldsize  = *maxsize;
      *maxsize = MAX(oldsize * 2, size * 5 / 4);
      *pstr    = httpd_realloc(*pstr, oldsize + 1, *maxsize + 1);
    }
  else
    {
      return;
    }

  if (!*pstr)
    {
      ndbg("out of memory reallocating a string to %d bytes\n", *maxsize);
      exit(1);
    }
}

#endif /* CONFIG_THTTPD */
