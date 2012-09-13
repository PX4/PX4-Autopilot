/****************************************************************************
 * netutils/thttpd/thttpd_strings.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <string.h>
#include <ctype.h>

#include "config.h"
#include "thttpd_strings.h"

#ifdef CONFIG_THTTPD

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the 'root' of the Filesystem as seen by the HTTP client */

const char httpd_root[] = CONFIG_THTTPD_PATH;

/* HTPP status */

const char ok200title[]  = "OK";
const char ok206title[]  = "Partial Content";

const char err302title[] = "Found";
const char err302form[]  = "The actual URL is '%s'.\n";

const char err304title[] = "Not Modified";

const char httpd_err400title[]  = "Bad Request";
const char httpd_err400form[]   = "Your request has bad syntax or is inherently impossible to satisfy.\n";

#ifdef CONFIG_THTTPD_AUTH_FILE
const char err401title[] = "Unauthorized";
const char err401form[]  = "Authorization required for the URL '%s'.\n";
#endif

const char err403title[] = "Forbidden";
#ifndef EXPLICIT_ERROR_PAGES
const char err403form[]  = "You do not have permission to get URL '%s' from this server.\n";
#endif

const char err404title[] = "Not Found";
const char err404form[]  = "The requested URL '%s' was not found on this server.\n";

const char httpd_err408title[]  = "Request Timeout";
const char httpd_err408form[]   = "No request appeared within a reasonable time period.\n";

const char err500title[] = "Internal Error";
const char err500form[]  = "There was an unusual problem serving the requested URL '%s'.\n";

const char err501title[] = "Not Implemented";
const char err501form[]  = "The requested method '%s' is not implemented by this server.\n";

const char httpd_err503title[] = "Service Temporarily Overloaded";
const char httpd_err503form[]  = "The requested URL '%s' is temporarily overloaded.  Please try again later.\n";

/* HTML strings */

const char html_crlf[]      = "\r\n";
const char html_html[]      = "<HTML>\r\n";
const char html_endhtml[]   = "</HTML>\r\n";
const char html_hdtitle[]   = "<HEAD><TITLE>";
const char html_titlehd[]   = "</TITLE></HEAD>\r\n";
const char html_body[]      = "<BODY BGCOLOR=\"#99cc99\" TEXT=\"#000000\" LINK=\"#2020ff\" VLINK=\"#4040cc\">\r\n";
const char html_endbody[]   = "</BODY>\r\n";
const char html_hdr2[]      = "<H2>";
const char html_endhdr2[]   = "</H2>";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int hexit(char nibble)
{
  if (nibble >= '0' && nibble <= '9')
    {
      return nibble - '0';
    }
  else if (nibble >= 'a' && nibble <= 'f')
    {
      return nibble - 'a' + 10;
    }
  else if (nibble >= 'A' && nibble <= 'F')
    {
      return nibble - 'A' + 10;
    }
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Copies and decodes a string.  It's ok for from and to to be the same string. */

void httpd_strdecode(char *to, char *from)
{
  for (; *from != '\0'; ++to, ++from)
    {
      if (from[0] == '%' && isxdigit(from[1]) && isxdigit(from[2]))
        {
          *to = hexit(from[1]) * 16 + hexit(from[2]);
          from += 2;
        }
      else
        {
          *to = *from;
        }
    }
  *to = '\0';
}

/* Copies and encodes a string. */

#ifdef CONFIG_THTTPD_GENERATE_INDICES
static void httpd_strencode(char *to, int tosize, char *from)
{
  int tolen;

  for (tolen = 0; *from != '\0' && tolen + 4 < tosize; ++from)
    {
      if (isalnum(*from) || strchr("/_.-~", *from) != NULL)
        {
          *to = *from;
          ++to;
          ++tolen;
        }
      else
        {
          (void)sprintf(to, "%%%02x", (int)*from & 0xff);
          to += 3;
          tolen += 3;
        }
    }
  *to = '\0';
}
#endif /* CONFIG_THTTPD_GENERATE_INDICES */
#endif /* CONFIG_THTTPD */
