/****************************************************************************
 * netutils/webserver/httpd_fs.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2001, Swedish Institute of Computer Science.
 *   All rights reserved.
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

/****************************************************************************
 * Included Header Files
 ****************************************************************************/

#include <stdint.h>

#include <apps/netutils/httpd.h>

#include "httpd.h"
#include "httpd_fsdata.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef NULL
#define NULL 0
#endif /* NULL */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#include "httpd_fsdata.c"

#if CONFIG_NETUTILS_HTTPDFSSTATS
static uint16_t count[HTTPD_FS_NUMFILES];
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint8_t httpd_fs_strcmp(const char *str1, const char *str2)
{
  int i;

  i = 0;
  for (;;)
    {
      if (str2[i] == 0 || str1[i] == '\r' || str1[i] == '\n')
        {
          return 0;
        }

      if (str1[i] != str2[i])
        {
          return 1;
        }

      i++;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int httpd_fs_open(const char *name, struct httpd_fs_file *file)
{
#if CONFIG_NETUTILS_HTTPDFSSTATS
  uint16_t i = 0;
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
  struct httpd_fsdata_file_noconst *f;

  for(f = (struct httpd_fsdata_file_noconst *)HTTPD_FS_ROOT;
      f != NULL;
      f = (struct httpd_fsdata_file_noconst *)f->next)
    {
      if (httpd_fs_strcmp(name, f->name) == 0)
        {
          file->data = f->data;
          file->len  = f->len;
#if CONFIG_NETUTILS_HTTPDFSSTATS
          ++count[i];
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
          return 1;
        }
#if CONFIG_NETUTILS_HTTPDFSSTATS
      ++i;
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
    }
  return 0;
}

void httpd_fs_init(void)
{
#if CONFIG_NETUTILS_HTTPDFSSTATS
  uint16_t i;
  for(i = 0; i < HTTPD_FS_NUMFILES; i++)
    {
      count[i] = 0;
    }
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
}

#if CONFIG_NETUTILS_HTTPDFSSTATS
uint16_t httpd_fs_count(char *name)
{
  struct httpd_fsdata_file_noconst *f;
  uint16_t i;

  i = 0;
  for(f = (struct httpd_fsdata_file_noconst *)HTTPD_FS_ROOT;
      f != NULL;
      f = (struct httpd_fsdata_file_noconst *)f->next)
    {
      if (httpd_fs_strcmp(name, f->name) == 0)
        {
          return count[i];
        }
      ++i;
    }
  return 0;
}
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
