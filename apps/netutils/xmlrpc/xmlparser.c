/****************************************************************************
 * apps/netutils/xmlrpc/xmlparser.c
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
 *  Lightweight Embedded XML-RPC Server XML Parser
 *
 *  mtj@cogitollc.com
 *
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <apps/netutils/xmlrpc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TAG        0
#define VALUE      1
#define DONE       2

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct xmlrpc_s g_xmlcall;
static char g_data[CONFIG_XMLRPC_STRINGSIZE+1];
static struct xmlrpc_entry_s *g_entries = NULL;

static const char *errorStrings[] =
{
  /* 0 */ "Internal error (unknown)",
  /* 1 */ "Parse Error...",
  /* 2 */ "Function not found...",
  /* 3 */ "Unexpected Integer Argument...",
  /* 4 */ "Unexpected Boolean Argument...",
  /* 5 */ "Unexpected Double Argument...",
  /* 6 */ "Unexpected String Argument...",
  /* 7 */ "Bad Response Argument..."
};

#define MAX_ERROR_CODE  (sizeof(errorStrings)/sizeof(char *))

struct parsebuf_s
{
  char *buf;
  int len;
  int index;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int xmlrpc_call(struct xmlrpc_s * call)
{
  int ret = XMLRPC_NO_SUCH_FUNCTION;
  struct xmlrpc_entry_s *entry = g_entries;

  while (entry != NULL)
    {
      if (strcmp(call->name, entry->name) == 0)
        {
          ret = entry->func(call);
          break;
        }
      else
        {
          entry = entry->next;
        }
    }

  return ret;
}

static int xmlrpc_getelement(struct parsebuf_s * pbuf, char *data, int dataSize)
{
  int j = 0;
  int ret = XMLRPC_NO_ERROR;

  while (!isprint(pbuf->buf[pbuf->index]))
    {
      pbuf->index++;
    }

  if (pbuf->index >= pbuf->len)
    {
      return DONE;
    }

  if (pbuf->buf[pbuf->index] == '<')
    {
      ret = TAG;
    }
  else
    {
      ret = VALUE;
    }

  data[j++] = pbuf->buf[pbuf->index++];

  while (j < dataSize)
    {
      if (pbuf->buf[pbuf->index] == '>')
        {
          data[j++] = pbuf->buf[pbuf->index++];
          break;
        }
      else if ((pbuf->buf[pbuf->index] == '\n') ||
               (pbuf->buf[pbuf->index] == '<'))
        {
          break;
        }
      else
        {
          data[j++] = pbuf->buf[pbuf->index++];
          if (j >= dataSize)
            ret = XMLRPC_PARSE_ERROR;
        }
    }

  data[j] = 0;
  return ret;
}

static int xmlrpc_parseparam(struct parsebuf_s * pbuf)
{
  int type;

  /* Next, we need a <value> tag */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if (!((type == TAG) && (!strncmp(g_data, "<value>", 7))))
    {
      return XMLRPC_PARSE_ERROR;
    }

  /* Now we get a variable tag, the type of the value */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if (type != TAG)
    {
      return XMLRPC_PARSE_ERROR;
    }

  if (!strncmp(g_data, "<i4>", 4))
    {
      g_xmlcall.args[g_xmlcall.argsize] = 'i';
    }
  else if (!strncmp(g_data, "<int>", 5))
    {
      g_xmlcall.args[g_xmlcall.argsize] = 'i';
    }
  else if (!strncmp(g_data, "<boolean>", 9))
    {
      g_xmlcall.args[g_xmlcall.argsize] = 'b';
    }
  else if (!strncmp(g_data, "<double>", 8))
    {
      g_xmlcall.args[g_xmlcall.argsize] = 'd';
    }
  else if (!strncmp(g_data, "<string>", 8))
    {
      g_xmlcall.args[g_xmlcall.argsize] = 's';
    }
  else
    {
      return XMLRPC_PARSE_ERROR;
    }

  /* Now, parse the actual value */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if (type != VALUE)
    {
      return XMLRPC_PARSE_ERROR;
    }

  switch (g_xmlcall.args[g_xmlcall.argsize])
    {
    case 'i':
    case 'b':
      g_xmlcall.arguments[g_xmlcall.argsize].u.i = atoi(g_data);
      break;
    case 'd':
      g_xmlcall.arguments[g_xmlcall.argsize].u.d = atof(g_data);
      break;
    case 's':
      strcpy(g_xmlcall.arguments[g_xmlcall.argsize].u.string, g_data);
      break;
    default:
      return XMLRPC_PARSE_ERROR;
    }

  g_xmlcall.argsize++;

  /* Now we close out the tag, starting with the type */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if (!((type == TAG) && (!strncmp(g_data, "</", 2))))
    {
      return XMLRPC_PARSE_ERROR;
    }

  /* Next, look for the </value> close */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if (!((type == TAG) && (!strncmp(g_data, "</value>", 8))))
    {
      return XMLRPC_PARSE_ERROR;
    }

  /* Finally, close out the </param> tag */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if (!((type == TAG) && (!strncmp(g_data, "</param>", 8))))
    {
      return XMLRPC_PARSE_ERROR;
    }

  return XMLRPC_NO_ERROR;
}

static int xmlrpc_parseparams(struct parsebuf_s * pbuf)
{
  int type, ret = XMLRPC_PARSE_ERROR;

  /* First, look for the params tag */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if ((type == TAG) && (!strncmp(g_data, "<params>", 8)))
    {
      while (1)
        {
          /* Get next tag */

          type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
          if ((type == TAG) && (!strncmp(g_data, "<param>", 7)))
            {
              ret = xmlrpc_parseparam(pbuf);
            }
          else if ((type == TAG) && (!strncmp(g_data, "</params>", 9)))
            {
              return XMLRPC_NO_ERROR;
            }
          else
            {
              return XMLRPC_PARSE_ERROR;
            }
        }
    }

  return ret;
}

static int xmlrpc_parsemethod(struct parsebuf_s * pbuf)
{
  int type, ret = XMLRPC_PARSE_ERROR;

  bzero((void *)&g_xmlcall, sizeof(struct xmlrpc_s));

  /* Look for the methodName tag */

  type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if ((type == TAG) && (!strncmp(g_data, "<methodName>", 12)))
    {
      /* Get the method name for the call */

      type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
      if (type == VALUE)
        {
          /* Save the method name */

          strcpy(g_xmlcall.name, g_data);

          /* Find the closing /methodCall */

          type = xmlrpc_getelement(pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
          if ((type == TAG) && (!strncmp(g_data, "</methodName>", 13)))
            {
              /* Now, it's time to parse the parameters */

              ret = xmlrpc_parseparams(pbuf);
            }
        }
    }

  return ret;
}

static void xmlrpc_sendfault(int fault)
{
  fault = -fault;
  if (fault >= MAX_ERROR_CODE)
    {
      fault = 0;
    }

  xmlrpc_buildresponse(&g_xmlcall, "{is}",
                        "faultCode", fault, "faultString", errorStrings[fault]);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int xmlrpc_parse(int sock, char *buffer)
{
  struct parsebuf_s pbuf;
  int type;
  int ret = XMLRPC_PARSE_ERROR;

  pbuf.buf = buffer;
  pbuf.len = strlen(buffer);
  pbuf.index = 0;

  /* Parse the xml header tag */

  type = xmlrpc_getelement(&pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
  if ((type == TAG) && (!strncmp(g_data, "<?xml", 5)))
    {
      type = xmlrpc_getelement(&pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
      if ((type == TAG) && (!strncmp(g_data, "<methodCall>", 12)))
        {
          /* Parse the remaining tags within the methodCall tag */

          xmlrpc_parsemethod(&pbuf);

          /* Check for the closing /methodCall */

          type = xmlrpc_getelement(&pbuf, g_data, CONFIG_XMLRPC_STRINGSIZE);
          if ((type == TAG) && (!strncmp(g_data, "</methodCall>", 13)))
            {
              /* Successful parse, try to call a user function */

              ret = xmlrpc_call(&g_xmlcall);
            }
        }
    }

  if (ret == 0)
    {
      write(sock, g_xmlcall.response, strlen(g_xmlcall.response));
    }
  else
    {
      /* Send fault response */

      g_xmlcall.error = 1;
      xmlrpc_sendfault(ret);
      write(sock, g_xmlcall.response, strlen(g_xmlcall.response));
    }

  return ret;
}

void xmlrpc_register(struct xmlrpc_entry_s *entry)
{
  if (g_entries == NULL)
    {
      g_entries = entry;
      entry->next = NULL;
    }
  else
    {
      entry->next = g_entries;
      g_entries = entry;
    }
}
