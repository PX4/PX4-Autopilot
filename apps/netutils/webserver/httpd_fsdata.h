/****************************************************************************
 * netutils/webserver/httpd_fsdata.h
 *
 *   Copyright (C) 2007-2009 Gregory Nutt. All rights reserved.
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

#ifndef __HTTPD_FSDATA_H__
#define __HTTPD_FSDATA_H__

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>
#include <nuttx/net/uip/uip.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct httpd_fsdata_file
{
  const struct httpd_fsdata_file *next;
  FAR const uint8_t *name;
  FAR const uint8_t *data;
  int len;
#ifdef CONFIG_NETUTILS_HTTPDFSSTATS
#if CONFIG_NETUTILS_HTTPDFSSTATS == 1
  uint16_t count;
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
};

struct httpd_fsdata_file_noconst
{
  FAR struct httpd_fsdata_file *next;
  FAR char *name;
  FAR char *data;
  int len;
#ifdef CONFIG_NETUTILS_HTTPDFSSTATS
#if CONFIG_NETUTILS_HTTPDFSSTATS == 1
  uint16_t count;
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
#endif /* CONFIG_NETUTILS_HTTPDFSSTATS */
};

#endif /* __HTTPD_FSDATA_H__ */
