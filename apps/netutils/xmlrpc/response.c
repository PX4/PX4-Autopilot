/****************************************************************************
 * apps/netutils/xmlrpc/response.c
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
 *  Lightweight Embedded XML-RPC Server Response Generator
 *
 *  mtj@cogitollc.com
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <apps/netutils/xmlrpc.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xmlrpc_insertlength(struct xmlrpc_s * xmlcall)
{
  int len, digit, xdigit = 1000, i = 0;
  char *temp;

  temp = strstr(xmlcall->response, "<?xml");
  len = strlen(temp);

  temp = strstr(xmlcall->response, "xyza");

  do
    {
      digit = (len / xdigit);
      len -= (digit * xdigit);
      xdigit /= 10;

      if ((digit == 0) && (xdigit > 1))
        temp[i++] = ' ';
      else
        temp[i++] = (0x30 + digit);
    }
  while (i < 4);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int xmlrpc_getinteger(struct xmlrpc_s * xmlcall, int *arg)
{
  if ((xmlcall == NULL) || (arg == NULL))
    {
      return XMLRPC_INTERNAL_ERROR;
    }

  if ((xmlcall->arg < xmlcall->argsize) &&
      (xmlcall->args[xmlcall->arg] == 'i'))
    {
      *arg = xmlcall->arguments[xmlcall->arg++].u.i;
      return 0;
    }

  return XMLRPC_UNEXPECTED_INTEGER_ARG;
}

int xmlrpc_getbool(struct xmlrpc_s * xmlcall, int *arg)
{
  if ((xmlcall == NULL) || (arg == NULL))
    {
      return XMLRPC_INTERNAL_ERROR;
    }

  if ((xmlcall->arg < xmlcall->argsize) &&
      (xmlcall->args[xmlcall->arg] == 'b'))
    {
      *arg = xmlcall->arguments[xmlcall->arg++].u.i;
      return 0;
    }

  return XMLRPC_UNEXPECTED_BOOLEAN_ARG;
}

int xmlrpc_getdouble(struct xmlrpc_s * xmlcall, double *arg)
{
  if ((xmlcall == NULL) || (arg == NULL))
    {
      return XMLRPC_INTERNAL_ERROR;
    }

  if ((xmlcall->arg < xmlcall->argsize) &&
      (xmlcall->args[xmlcall->arg] == 'd'))
    {
      *arg = xmlcall->arguments[xmlcall->arg++].u.d;
      return 0;
    }

  return XMLRPC_UNEXPECTED_DOUBLE_ARG;
}

int xmlrpc_getstring(struct xmlrpc_s* xmlcall, char *arg)
{
  if ((xmlcall == NULL) || (arg == NULL))
    {
      return XMLRPC_INTERNAL_ERROR;
    }

  if ((xmlcall->arg < xmlcall->argsize) &&
      (xmlcall->args[xmlcall->arg] == 's'))
    {
      strcpy(arg, xmlcall->arguments[xmlcall->arg++].u.string);
      return 0;
    }

  return XMLRPC_UNEXPECTED_STRING_ARG;
}

int xmlrpc_buildresponse(struct xmlrpc_s* xmlcall, char *args, ...)
{
  va_list argp;
  int i, ret = 0, index = 0, close = 0;
  double d;
  char *s;
  int isStruct = 0;

  if ((xmlcall == NULL) || (args == NULL))
    {
      return -1;
    }

  strcpy(xmlcall->response, "HTTP/1.1 200 OK\n"
         "Connection: close\n"
         "Content-length: xyza\n"
         "Content-Type: text/xml\n"
         "Server: Lightweight XMLRPC\n\n"
         "<?xml version=\"1.0\"?>\n" "<methodResponse>\n");

  if (xmlcall->error)
    {
      strcat(&xmlcall->response[strlen(xmlcall->response)], "  <fault>\n");
    }
  else
    {
      strcat(&xmlcall->response[strlen(xmlcall->response)],
             "  <params><param>\n");
    }

  va_start(argp, args);

  while (args[index])
    {
      if (isStruct)
        {
          if ((args[index] != '{') && (args[index] != '}'))
            {
              sprintf(&xmlcall->response[strlen(xmlcall->response)],
                      "  <member>\n");
              sprintf(&xmlcall->response[strlen(xmlcall->response)],
                      "    <name>%s</name>\n", va_arg(argp, char *));
              close = 1;
            }
        }

      switch (args[index])
        {
        case '{':
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "  <value><struct>\n");
          isStruct = 1;
          break;

        case '}':
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "  </struct></value>\n");
          isStruct = 0;
          break;

        case 'i':
          i = va_arg(argp, int);
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "    <value><int>%d</int></value>\r\n", i);
          break;

        case 'b':
          i = va_arg(argp, int);
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "    <value><boolean>%d</boolean></value>\r\n", i);
          break;

        case 'd':
          d = va_arg(argp, double);
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "    <value><double>%f</double></value>\r\n", d);
          break;

        case 's':
          s = va_arg(argp, char *);
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "    <value><string>%s</string></value>\r\n", s);
          break;

        default:
          return (XMLRPC_BAD_RESPONSE_ARG);
          break;

        }

      if (close)
        {
          sprintf(&xmlcall->response[strlen(xmlcall->response)],
                  "  </member>\n");
          close = 0;
        }

      index++;
    }

  va_end(argp);

  if (xmlcall->error)
    {
      strcat(&xmlcall->response[strlen(xmlcall->response)], "  </fault>\r\n");
    }
  else
    {
      strcat(&xmlcall->response[strlen(xmlcall->response)],
             "  </param></params>\r\n");
    }

  if (ret == 0)
    {
      strcat(&xmlcall->response[strlen(xmlcall->response)],
             "</methodResponse>\r\n");

      xmlrpc_insertlength(xmlcall);
    }
  else
    {
      xmlcall->response[0] = 0;
    }

  return ret;
}
