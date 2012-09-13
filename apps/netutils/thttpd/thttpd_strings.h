/****************************************************************************
 * netutils/thttpd/thttpd_strings.h
 * HTTP strings
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995,1998,1999,2000,2001 by Jef Poskanzer <jef@mail.acme.com>.
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __NETUTILS_THTTPD_THTTPD_STRINGS_H
#define __NETUTILS_THTTPD_THTTPD_STRINGS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "config.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the 'root' of the Filesystem as seen by the HTTP client */

extern const char httpd_root[];

/* HTPP status */

extern const char ok200title[];
extern const char ok206title[];

extern const char err302title[];
extern const char err302form[];

extern const char err304title[];

extern const char httpd_err400title[];
extern const char httpd_err400form[];

#ifdef CONFIG_THTTPD_AUTH_FILE
extern const char err401title[];
extern const char err401form[];
#endif

extern const char err403title[];
#ifndef EXPLICIT_ERROR_PAGES
extern const char err403form[];
#endif

extern const char err404title[];
extern const char err404form[];

extern const char httpd_err408title[];
extern const char httpd_err408form[];

extern const char err500title[];
extern const char err500form[];

extern const char err501title[];
extern const char err501form[];

extern const char httpd_err503title[];
extern const char httpd_err503form[];

/* HTML strings */

extern const char html_crlf[];
extern const char html_html[];
extern const char html_endhtml[];
extern const char html_hdtitle[];
extern const char html_titlehd[];
extern const char html_body[];
extern const char html_endbody[];
extern const char html_hdr2[];
extern const char html_endhdr2[];

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Copies and decodes a string.  It's ok for from and to to be the same string. */

extern void httpd_strdecode(char *to, char *from);

/* Copies and encodes a string. */

#ifdef CONFIG_THTTPD_GENERATE_INDICES
extern void httpd_strencode(char *to, int tosize, char *from);
#endif

#endif /* CONFIG_THTTPD */
#endif /* __NETUTILS_THTTPD_THTTPD_STRINGS_H */
