/****************************************************************************
 * apps/include/netutils/xmlrpc.h
 * 
 *   Copyright (C) 2012 Max Holtzberg. All rights reserved.
 *   Author: Max Holtzberg <mh@uvc.de>
 *
 * Based on the embeddable lightweight XML-RPC server code discussed
 * in the article at: http://www.drdobbs.com/web-development/\
 *    an-embeddable-lightweight-xml-rpc-server/184405364
 *
 *  Copyright (c) 2002 Cogito LLC.  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or 
 *  without modification, is hereby granted without fee provided 
 *  that the following conditions are met:
 * 
 *    1.  Redistributions of source code must retain the above 
 *        copyright notice, this list of conditions and the 
 *        following disclaimer.
 *    2.  Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the 
 *        following disclaimer in the documentation and/or other 
 *        materials provided with the distribution.
 *    3.  Neither the name of Cogito LLC nor the names of its 
 *        contributors may be used to endorse or promote products 
 *        derived from this software without specific prior 
 *        written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY COGITO LLC AND CONTRIBUTERS 'AS IS' 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED 
 * TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A 
 * PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COGITO LLC 
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARAY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF 
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/

/*
 *  Lightweight Embedded XML-RPC Server Types and Definitions
 *
 *  mtj@cogitollc.com
 *
 */

#ifndef __APPS_INCLUDE_NETUTILS_XMLRPC_H
#define __APPS_INCLUDE_NETUTILS_XMLRPC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Error definitions. */

#define XMLRPC_NO_ERROR                 (0)
#define XMLRPC_PARSE_ERROR              (-1)
#define XMLRPC_NO_SUCH_FUNCTION         (-2)
#define XMLRPC_UNEXPECTED_INTEGER_ARG   (-3)
#define XMLRPC_UNEXPECTED_BOOLEAN_ARG   (-4)
#define XMLRPC_UNEXPECTED_DOUBLE_ARG    (-5)
#define XMLRPC_UNEXPECTED_STRING_ARG    (-6)
#define XMLRPC_BAD_RESPONSE_ARG         (-7)
#define XMLRPC_INTERNAL_ERROR           (-99)

#define MAX_ARGS                        10
#define MAX_RESPONSE                    2048

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct xmlrpc_arg_s
{
  union
  {
    int i;
    char boolean;
    double d;
    char string[CONFIG_XMLRPC_STRINGSIZE+1];
  } u;
};

struct xmlrpc_s
{
  char  name[CONFIG_XMLRPC_STRINGSIZE+1];
  struct xmlrpc_arg_s arguments[MAX_ARGS];
  char  args[MAX_ARGS];
  int   argsize;
  int   arg;
  char  response[MAX_RESPONSE];
  int   error;
};

struct xmlrpc_entry_s
{
  struct xmlrpc_entry_s *next;
  int (*func)(struct xmlrpc_s*);
  char *name;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void xmlrpc_register(struct xmlrpc_entry_s *call);
int xmlrpc_parse(int sock, char *buffer);
int xmlrpc_getinteger(struct xmlrpc_s *xmlcall, int *arg);
int xmlrpc_getbool(struct xmlrpc_s *xmlcall, int *arg);
int xmlrpc_getdouble(struct xmlrpc_s *xmlcall, double *arg);
int xmlrpc_getstring(struct xmlrpc_s *xmlcall, char *arg);
int xmlrpc_buildresponse(struct xmlrpc_s *, char *, ...);

#endif /* __APPS_INCLUDE_NETUTILS_XMLRPC_H */
