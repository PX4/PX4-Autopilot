/****************************************************************************
 * netutils/webclient/webclient.c
 * Implementation of the HTTP client.
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* This example shows a HTTP client that is able to download web pages
 * and files from web servers. It requires a number of callback
 * functions to be implemented by the module that utilizes the code:
 * webclient_datahandler().
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef CONFIG_WEBCLIENT_HOST
#  include <nuttx/config.h>
#  include <nuttx/compiler.h>
#  include <debug.h>
#endif

#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#ifdef CONFIG_HAVE_GETHOSTBYNAME
#  include <netdb.h>
#else
#  include <apps/netutils/resolv.h>
#endif

#include <arpa/inet.h>
#include <netinet/in.h>
#include <apps/netutils/uiplib.h>
#include <apps/netutils/webclient.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define WEBCLIENT_TIMEOUT          100

#define WEBCLIENT_STATE_STATUSLINE 0
#define WEBCLIENT_STATE_HEADERS    1
#define WEBCLIENT_STATE_DATA       2
#define WEBCLIENT_STATE_CLOSE      3

#define HTTPSTATUS_NONE            0
#define HTTPSTATUS_OK              1
#define HTTPSTATUS_MOVED           2
#define HTTPSTATUS_ERROR           3

#define ISO_nl                     0x0a
#define ISO_cr                     0x0d
#define ISO_space                  0x20

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct wget_s
{
  /* Internal status */

  uint8_t state;
  uint8_t httpstatus;

  uint16_t port;     /* The port number to use in the connection */

  /* These describe the just-received buffer of data */

  FAR char *buffer;  /* user-provided buffer */
  int buflen;        /* Length of the user provided buffer */
  int offset;        /* Offset to the beginning of interesting data */
  int datend;        /* Offset+1 to the last valid byte of data in the buffer */  

  /* Buffer HTTP header data and parse line at a time */

  char line[CONFIG_WEBCLIENT_MAXHTTPLINE];
  int  ndx;

#ifdef CONFIG_WEBCLIENT_GETMIMETYPE
  char mimetype[CONFIG_WEBCLIENT_MAXMIMESIZE];
#endif
  char hostname[CONFIG_WEBCLIENT_MAXHOSTNAME];
  char filename[CONFIG_WEBCLIENT_MAXFILENAME];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_http10[]          = "HTTP/1.0";
static const char g_http11[]          = "HTTP/1.1";
#ifdef CONFIG_WEBCLIENT_GETMIMETYPE
static const char g_httpcontenttype[] = "content-type: ";
#endif
static const char g_httphost[]        = "host: ";
static const char g_httplocation[]    = "location: ";
static const char g_httpget[]         = "GET ";

static const char g_httpuseragentfields[] =
  "Connection: close\r\n"
  "User-Agent: NuttX/0.4.x (; http://www.nuttx.org/)\r\n\r\n";

static const char g_http200[]         = "200 ";
static const char g_http301[]         = "301 ";
static const char g_http302[]         = "302 ";

static const char g_httpcrnl[]        = "\r\n";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wget_strcpy
 ****************************************************************************/

static char *wget_strcpy(char *dest, const char *src)
{
  int len = strlen(src);
  memcpy(dest, src, len);
  dest[len] = '\0';
  return dest + len;
}

/****************************************************************************
 * Name: wget_resolvehost
 ****************************************************************************/

static inline int wget_resolvehost(const char *hostname, in_addr_t *ipaddr)
{
#ifdef CONFIG_HAVE_GETHOSTBYNAME

  struct hostent *he;

  nvdbg("Getting address of %s\n", hostname);
  he = gethostbyname(hostname);
  if (!he)
    {
      ndbg("gethostbyname failed: %d\n", h_errno);
      return ERROR;
    }

  nvdbg("Using IP address %04x%04x\n", (uint16_t)he->h_addr[1], (uint16_t)he->h_addr[0]);
  memcpy(ipaddr, he->h_addr, sizeof(in_addr_t));
  return OK;

#else

# ifdef CONFIG_NET_IPv6
  struct sockaddr_in6 addr;
# else
  struct sockaddr_in addr;
# endif

  /* First check if the host is an IP address. */

  if (!uiplib_ipaddrconv(hostname, (uint8_t*)ipaddr))
    {
      /* 'host' does not point to a valid address string.  Try to resolve
       *  the host name to an IP address.
       */
 
      if (resolv_query(hostname, &addr) < 0)
        {
          /* Needs to set the errno here */

          return ERROR;
        }

      /* Save the host address -- Needs fixed for IPv6 */

      *ipaddr = addr.sin_addr.s_addr;
  }
  return OK;

#endif
}

/****************************************************************************
 * Name: wget_parsestatus
 ****************************************************************************/

static inline int wget_parsestatus(struct wget_s *ws)
{
  int offset;
  int ndx;
  char *dest;

  offset = ws->offset;
  ndx    = ws->ndx;

  while (offset < ws->datend)
    {
      ws->line[ndx] = ws->buffer[offset];
      if (ws->line[ndx] == ISO_nl)
        {
          ws->line[ndx] = '\0';
          if ((strncmp(ws->line, g_http10, strlen(g_http10)) == 0) ||
              (strncmp(ws->line, g_http11, strlen(g_http11)) == 0))
            {
              dest = &(ws->line[9]);
              ws->httpstatus = HTTPSTATUS_NONE;

              /* Check for 200 OK */

              if (strncmp(dest, g_http200, strlen(g_http200)) == 0)
                {
                  ws->httpstatus = HTTPSTATUS_OK;
                }

              /* Check for 301 Moved permanently or 302 Found. Location: header line
               * will contain the new location.
               */

              else if (strncmp(dest, g_http301, strlen(g_http301)) == 0 ||
                       strncmp(dest, g_http302, strlen(g_http302)) == 0)
                {

                  ws->httpstatus = HTTPSTATUS_MOVED;
                }
            }
          else
            {
              return - ECONNABORTED;
            }

          /* We're done parsing the status line, so start parsing the HTTP headers. */

          ws->state = WEBCLIENT_STATE_HEADERS;
          break;
        }
      else
        {
          offset++;
          ndx++;
        }
    }

  ws->offset = offset;
  ws->ndx    = ndx;
  return OK;
}

/****************************************************************************
 * Name: wget_parsestatus
 ****************************************************************************/

static inline int wget_parseheaders(struct wget_s *ws)
{
  int offset;
  int ndx;

  offset = ws->offset;
  ndx    = ws->ndx;

  while (offset < ws->datend)
    {
      ws->line[ndx] = ws->buffer[offset];
      if (ws->line[ndx] == ISO_nl)
        {
          /* We have an entire HTTP header line in s.line, so
           * we parse it.
           */

          if (ndx > 0) /* Should always be true */
            {
              if (ws->line[0] == ISO_cr)
                {
                  /* This was the last header line (i.e., and empty "\r\n"), so
                   * we are done with the headers and proceed with the actual
                   * data.
                   */

                  ws->state = WEBCLIENT_STATE_DATA;
                  goto exit;
               }

              /* Truncate the trailing \r\n */

              ws->line[ndx-1] = '\0';

              /* Check for specific HTTP header fields. */

#ifdef CONFIG_WEBCLIENT_GETMIMETYPE
              if (strncasecmp(ws->line, g_httpcontenttype, strlen(g_httpcontenttype)) == 0)
                {
                  /* Found Content-type field. */

                  char *dest = strchr(ws->line, ';');
                  if (dest != NULL)
                    {
                      *dest = 0;
                   }
                  strncpy(ws->mimetype, ws->line + strlen(g_httpcontenttype), sizeof(ws->mimetype));
                }
              else
#endif
              if (strncasecmp(ws->line, g_httplocation, strlen(g_httplocation)) == 0)
                {
                  /* Parse the new HTTP host and filename from the URL.  Note that
                   * the return value is ignored.  In the event of failure, we
                   * retain the current location.
                   */

                  (void)uip_parsehttpurl(ws->line + strlen(g_httplocation), &ws->port,
                                         ws->hostname, CONFIG_WEBCLIENT_MAXHOSTNAME,
                                         ws->filename, CONFIG_WEBCLIENT_MAXFILENAME);
                  nvdbg("New hostname='%s' filename='%s'\n", ws->hostname, ws->filename);
                }
            }

          /* We're done parsing this line, so we reset the index to the start
           * of the next line.
           */

          ndx = 0;
        }
      else
        {
          ndx++;
        }
      offset++;
    }

exit:
  ws->offset = offset;
  ws->ndx    = ndx;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wget
 *
 * Description:
 *   Obtain the requested file from an HTTP server using the GET method.
 *
 *   Note: If the function is passed a host name, it must already be in
 *   the resolver cache in order for the function to connect to the web
 *   server. It is therefore up to the calling module to implement the
 *   resolver calls and the signal handler used for reporting a resolv
 *   query answer.
 *
 * Input Parameters
 *   url      - A pointer to a string containing either the full URL to
 *              the file to get (e.g., http://www.nutt.org/index.html, or
 *              http://192.168.23.1:80/index.html).
 *   buffer   - A user provided buffer to receive the file data (also
 *              used for the outgoing GET request
 *   buflen   - The size of the user provided buffer
 *   callback - As data is obtained from the host, this function is
 *              to dispose of each block of file data as it is received.
 *
 * Returned Value:
 *   0: if the GET operation completed successfully;
 *  -1: On a failure with errno set appropriately 
 *
 ****************************************************************************/
 
int wget(FAR const char *url, FAR char *buffer, int buflen,
         wget_callback_t callback, FAR void *arg)
{
  struct sockaddr_in server;
  struct wget_s ws;
  bool redirected;
  char *dest;
  int sockfd;
  int len;
  int ret = OK;

  /* Initialize the state structure */

  memset(&ws, 0, sizeof(struct wget_s));
  ws.buffer = buffer;
  ws.buflen = buflen;
  ws.port   = 80;

  /* Parse the hostname (with optional port number) and filename from the URL */

  ret = uip_parsehttpurl(url, &ws.port,
                         ws.hostname, CONFIG_WEBCLIENT_MAXHOSTNAME,
                         ws.filename, CONFIG_WEBCLIENT_MAXFILENAME);
  if (ret != 0)
    {
      ndbg("Malformed HTTP URL: %s\n", url);
      set_errno(-ret);
      return ERROR;
    }
  nvdbg("hostname='%s' filename='%s'\n", ws.hostname, ws.filename);

  /* The following sequence may repeat indefinitely if we are redirected */

  do
    {
      /* Re-initialize portions of the state structure that could have
       * been left from the previous time through the loop and should not
       * persist with the new connection.
       */

      ws.httpstatus = HTTPSTATUS_NONE;
      ws.offset     = 0;
      ws.datend     = 0;
      ws.ndx        = 0;

      /* Create a socket */

      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd < 0)
        {
          /* socket failed.  It will set the errno appropriately */

          ndbg("socket failed: %d\n", errno);
          return ERROR;
        }

      /* Get the server adddress from the host name */

      server.sin_family = AF_INET;
      server.sin_port   = htons(ws.port);
      ret = wget_resolvehost(ws.hostname, &server.sin_addr.s_addr);
      if (ret < 0)
        {
          /* Could not resolve host (or malformed IP address) */

          ndbg("Failed to resolve hostname\n");
          ret = -EHOSTUNREACH;
          goto errout_with_errno;
        }

      /* Connect to server.  First we have to set some fields in the
       * 'server' address structure.  The system will assign me an arbitrary
       * local port that is not in use.
       */

      ret = connect(sockfd, (struct sockaddr *)&server, sizeof(struct sockaddr_in));
      if (ret < 0)
        {
          ndbg("connect failed: %d\n", errno);
          goto errout;
        }

      /* Send the GET request */

      dest   = ws.buffer;
      dest   = wget_strcpy(dest, g_httpget);
      dest   = wget_strcpy(dest, ws.filename);
     *dest++ = ISO_space;
      dest   = wget_strcpy(dest, g_http10);
      dest   = wget_strcpy(dest, g_httpcrnl);
      dest   = wget_strcpy(dest, g_httphost);
      dest   = wget_strcpy(dest, ws.hostname);
      dest   = wget_strcpy(dest, g_httpcrnl);
      dest   = wget_strcpy(dest, g_httpuseragentfields);
      len    = dest - buffer;

      ret    = send(sockfd, buffer, len, 0);
      if (ret < 0)
        {
          ndbg("send failed: %d\n", errno);
          goto errout;
        }

      /* Now loop to get the file sent in response to the GET.  This
       * loop continues until either we read the end of file (nbytes == 0)
       * or until we detect that we have been redirected.
       */

      ws.state   = WEBCLIENT_STATE_STATUSLINE;
      redirected = false;
      for(;;)
        {
          ws.datend = recv(sockfd, ws.buffer, ws.buflen, 0);
          if (ws.datend < 0)
            {
              ndbg("recv failed: %d\n", errno);
              ret = ws.datend;
              goto errout_with_errno;
            }
          else if (ret == 0)
            {
              nvdbg("Connection lost\n");
              close(sockfd);            
              break;
            }

          /* Handle initial parsing of the status line */

          ws.offset = 0;
          if (ws.state == WEBCLIENT_STATE_STATUSLINE)
            {
              ret = wget_parsestatus(&ws);
              if (ret < 0)
                {
                  goto errout_with_errno;
                }
            }

          /* Parse the HTTP data */

          if (ws.state == WEBCLIENT_STATE_HEADERS)
            {
              ret = wget_parseheaders(&ws);
              if (ret < 0)
                {
                  goto errout_with_errno;
                }
            }

          /* Dispose of the data payload */

          if (ws.state == WEBCLIENT_STATE_DATA)
            {
              if (ws.httpstatus != HTTPSTATUS_MOVED)
                {
                  /* Let the client decide what to do with the received file */

                  callback(&ws.buffer, ws.offset, ws.datend, &buflen, arg);
                }
              else
                {
                  redirected = true;
                  close(sockfd);            
                  break;
                }
            }
        }
    }
  while (redirected);
  return OK;

errout_with_errno:
  set_errno(-ret);
errout:
  close(sockfd);
  return ERROR;
}
