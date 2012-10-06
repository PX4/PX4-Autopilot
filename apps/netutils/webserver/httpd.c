/****************************************************************************
 * netutils/webserver/httpd.c
 * httpd Web server
 *
 *   Copyright (C) 2007-2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a leverage of similar logic from uIP:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2004, Adam Dunkels.
 *   All rights reserved.
 *
 *   The uIP web server is a very simplistic implementation of an HTTP
 *   server. It can serve web pages and files from a read-only ROM
 *   filesystem, and provides a very small scripting language.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#ifndef CONFIG_NETUTILS_HTTPD_SINGLECONNECT
#  include <pthread.h>
#endif

#include <nuttx/net/uip/uip.h>
#include <apps/netutils/uiplib.h>
#include <apps/netutils/httpd.h>

#include "httpd.h"
#include "httpd_cgi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if !defined(CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE) && defined(CONFIG_NETUTILS_HTTPD_SENDFILE)
#  error "Script support and CONFIG_NETUTILS_HTTPD_SENDFILE are mutually exclusive"
#endif

#if defined(CONFIG_NETUTILS_HTTPD_SENDFILE) && defined(CONFIG_NETUTILS_HTTPD_MMAP)
#  error "CONFIG_NETUTILS_HTTPD_SENDFILE and CONFIG_NETUTILS_HTTPD_MMAP are mutually exclusive"
#endif

#define ISO_nl      0x0a
#define ISO_space   0x20
#define ISO_bang    0x21
#define ISO_percent 0x25
#define ISO_period  0x2e
#define ISO_slash   0x2f
#define ISO_colon   0x3a

#ifndef CONFIG_NETUTILS_HTTPD_PATH
#  define CONFIG_NETUTILS_HTTPD_PATH "/mnt"
#endif

#ifndef CONFIG_NETUTILS_HTTPD_ERRPATH
#  define CONFIG_NETUTILS_HTTPD_ERRPATH ""
#endif

/* The correct way to disable receive timeout errors is by setting the
 * timeout to zero.
 */

#ifndef CONFIG_NETUTILS_HTTPD_TIMEOUT
#  define CONFIG_NETUTILS_HTTPD_TIMEOUT 0
#endif

/* If timeouts are not enabled, then keep-alive is disabled.  This is to
 * prevent a rogue HTTP client from blocking the httpd indefinitely.
 */

#if !defined(CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE)
#  if CONFIG_NETUTILS_HTTPD_TIMEOUT == 0
#    define CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
#  endif
#endif

#if !defined(CONFIG_NETUTILS_HTTPD_SENDFILE) && !defined(CONFIG_NETUTILS_HTTPD_MMAP)
#  ifndef CONFIG_NETUTILS_HTTPD_INDEX
#    ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
#      define CONFIG_NETUTILS_HTTPD_INDEX "index.shtml"
#    else
#      define CONFIG_NETUTILS_HTTPD_INDEX "index.html"
#    endif
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int httpd_open(const char *name, struct httpd_fs_file *file)
{
#if defined(CONFIG_NETUTILS_HTTPD_SENDFILE)
  return httpd_sendfile_open(name, file);
#elif defined(CONFIG_NETUTILS_HTTPD_MMAP)
  return httpd_mmap_open(name, file);
#else
  return httpd_fs_open(name, file);
#endif
}

static int httpd_openindex(struct httpd_state *pstate)
{
  int ret;
  size_t z;

  z = strlen(pstate->ht_filename);
  if (z > 0 && pstate->ht_filename[z - 1] == '/')
    {
      pstate->ht_filename[--z] = '\0';
    }

  ret = httpd_open(pstate->ht_filename, &pstate->ht_file);
#if defined(CONFIG_NETUTILS_HTTPD_SENDFILE) || defined(CONFIG_NETUTILS_HTTPD_MMAP)
#  if defined(CONFIG_NETUTILS_HTTPD_INDEX)
  if (ret == ERROR && errno == EISDIR)
    {
      (void) snprintf(pstate->ht_filename + z, sizeof pstate->ht_filename - z, "/%s",
        CONFIG_NETUTILS_HTTPD_INDEX);

      ret = httpd_open(pstate->ht_filename, &pstate->ht_file);
    }
#  endif
#endif

  return ret;
}

static int httpd_close(struct httpd_fs_file *file)
{
#if defined(CONFIG_NETUTILS_HTTPD_SENDFILE)
  return httpd_sendfile_close(file);
#elif defined(CONFIG_NETUTILS_HTTPD_MMAP)
  return httpd_mmap_close(file);
#else
  return OK;
#endif
}

#ifdef CONFIG_NETUTILS_HTTPD_DUMPBUFFER
static void httpd_dumpbuffer(FAR const char *msg, FAR const char *buffer, unsigned int nbytes)
{
  /* CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE, and CONFIG_DEBUG_NET have to be
   * defined or the following does nothing.
   */

  nvdbgdumpbuffer(msg, (FAR const uint8_t*)buffer, nbytes);
}
#else
# define httpd_dumpbuffer(msg,buffer,nbytes)
#endif

#ifdef CONFIG_NETUTILS_HTTPD_DUMPPSTATE
static void httpd_dumppstate(struct httpd_state *pstate, const char *msg)
{
#if defined(CONFIG_DEBUG) && defined(CONFIG_DEBUG_VERBOSE) && defined(CONFIG_DEBUG_NET)
  nvdbg("[%d] pstate(%p): [%s]\n", pstate->ht_sockfd, pstate, msg);
  nvdbg("  filename:      [%s]\n", pstate->ht_filename);
  nvdbg("  htfile len:    %d\n", pstate->ht_file.len);
  nvdbg("  sockfd:        %d\n", pstate->ht_sockfd);
#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
  nvdbg("  scriptptr:     %p\n", pstate->ht_scriptptr);
  nvdbg("  scriptlen:     %d\n", pstate->ht_scriptlen);
#endif
  nvdbg("  sndlen:        %d\n", pstate->ht_sndlen);
#endif
}
#else
# define httpd_dumppstate(pstate, msg)
#endif

#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
static void next_scriptstate(struct httpd_state *pstate)
{
  char *p;
  p = strchr(pstate->ht_scriptptr, ISO_nl) + 1;
  pstate->ht_scriptlen -= (unsigned short)(p - pstate->ht_scriptptr);
  pstate->ht_scriptptr  = p;
}
#endif

#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
static int handle_script(struct httpd_state *pstate)
{
  int len;
  char *ptr;

  while (pstate->ht_file.len > 0)
    {
      /* Check if we should start executing a script */

      if (*pstate->ht_file.data == ISO_percent && *(pstate->ht_file.data + 1) == ISO_bang)
        {
          pstate->ht_scriptptr = pstate->ht_file.data + 3;
          pstate->ht_scriptlen = pstate->ht_file.len - 3;
          if (*(pstate->ht_scriptptr - 1) == ISO_colon)
            {
              if (httpd_open(pstate->ht_scriptptr + 1, &pstate->ht_file) != OK)
                {
                   return ERROR;
                }

              send(pstate->ht_sockfd, pstate->ht_file.data, pstate->ht_file.len, 0);

              httpd_close(&pstate->ht_file);
            }
          else
            {
              httpd_cgifunction f;

              f = httpd_cgi(pstate->ht_scriptptr);
              if (f != NULL)
                {
                  f(pstate, pstate->ht_scriptptr);
                }
            }

          next_scriptstate(pstate);

          /* The script is over, so we reset the pointers and continue
           * sending the rest of the file
           */

          pstate->ht_file.data = pstate->ht_scriptptr;
          pstate->ht_file.len  = pstate->ht_scriptlen;
        }
      else
        {
          /* See if we find the start of script marker in the block of HTML
           * to be sent
           */

          if (pstate->ht_file.len > HTTPD_IOBUFFER_SIZE)
            {
              len = HTTPD_IOBUFFER_SIZE;
            }
          else
            {
              len = pstate->ht_file.len;
            }

          if (*pstate->ht_file.data == ISO_percent)
            {
              ptr = strchr(pstate->ht_file.data + 1, ISO_percent);
            }
          else
            {
              ptr = strchr(pstate->ht_file.data, ISO_percent);
            }

          if (ptr != NULL && ptr != pstate->ht_file.data)
            {
              len = (int)(ptr - pstate->ht_file.data);
              if (len >= HTTPD_IOBUFFER_SIZE)
                {
                  len = HTTPD_IOBUFFER_SIZE;
                }
            }

          send(pstate->ht_sockfd, pstate->ht_file.data, len, 0);
          pstate->ht_file.data += len;
          pstate->ht_file.len  -= len;
        }
    }
  return OK;
}
#endif

static int send_chunk(struct httpd_state *pstate, const char *buf, int len)
{
  int ret;

  do
    {
      httpd_dumpbuffer("Outgoing chunk", buf, len);
      ret = send(pstate->ht_sockfd, buf, len, 0);
      if (ret < 0)
        {
          return ERROR;
        }

      buf += ret;
      len -= ret;
    }
  while (len > 0);

  return OK;
}

static int send_headers(struct httpd_state *pstate, int status, int len)
{
  const char *mime;
  const char *ptr;
  char cl[32];
  char s[128];
  int i;

  static const struct
  {
    const char *ext;
    const char *mime;
  } a[] =
  {
#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
    { "shtml", "text/html"       },
#endif
    { "html",  "text/html"       },
    { "css",   "text/css"        },
    { "txt",   "text/plain"      },
    { "js",    "text/javascript" },

    { "png",   "image/png"       },
    { "gif",   "image/gif"       },
    { "jpeg",  "image/jpeg"      },
    { "jpg",   "image/jpeg"      }
  };

  ptr = strrchr(pstate->ht_filename, ISO_period);
  if (ptr == NULL)
    {
      mime = "application/octet-stream";
    }
  else
    {
      mime = "text/plain";

      for (i = 0; i < sizeof a / sizeof *a; i++)
        {
          if (strncmp(a[i].ext, ptr + 1, strlen(a[i].ext)) == 0)
            {
              mime = a[i].mime;
              break;
            }
        }
    }

  if (len >= 0)
    {
      (void) snprintf(cl, sizeof cl, "Content-Length: %d\r\n", len);
    }
#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
  else
    {
      pstate->ht_keepalive = false;
    }
#endif

  if (status == 413)
    {
      /* TODO: here we "SHOULD" include a Retry-After header */
    }

  i = snprintf(s, sizeof s,
    "HTTP/1.0 %d %s\r\n"
#ifndef CONFIG_NETUTILS_HTTPD_SERVERHEADER_DISABLE
    "Server: uIP/NuttX http://nuttx.org/\r\n"
#endif
    "Connection: %s\r\n"
    "Content-type: %s\r\n"
    "%s"
    "\r\n",
    status,
    status >= 400 ? "Error" : "OK",
#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
    pstate->ht_keepalive ? "keep-alive" : "close",
#else
    "close",
#endif
    mime,
    len >= 0 ? cl : "");

  return send_chunk(pstate, s, i);
}

static int httpd_senderror(struct httpd_state *pstate, int status)
{
  int ret;
  char msg[10 + 1];

  nvdbg("[%d] sending error '%d'\n", pstate->ht_sockfd, status);

  if (status < 400 || status >= 600)
    {
      status = 500;
    }

#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
  if (status != 404)
    {
      pstate->ht_keepalive = false;
    }
#endif

  (void) snprintf(pstate->ht_filename, sizeof pstate->ht_filename,
    "%s/%d.html",
    CONFIG_NETUTILS_HTTPD_ERRPATH, status);

  ret = httpd_openindex(pstate);

  if (send_headers(pstate, status, ret == OK ? pstate->ht_file.len : sizeof msg - 1) != OK)
    {
      return ERROR;
    }

  if (ret != OK)
    {
      (void) snprintf(msg, sizeof msg, "Error %d\n", status);

      ret = send_chunk(pstate, msg, sizeof msg - 1);
    }
  else
    {
#ifdef CONFIG_NETUTILS_HTTPD_SENDFILE
      ret = httpd_sendfile_send(pstate->ht_sockfd, &pstate->ht_file);
#else
      ret = send_chunk(pstate, pstate->ht_file.data, pstate->ht_file.len);
#endif

      (void) httpd_close(&pstate->ht_file);
    }

  return ret;
}

static int httpd_sendfile(struct httpd_state *pstate)
{
#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
  char *ptr;
#endif
  int ret = ERROR;

  pstate->ht_sndlen = 0;

  nvdbg("[%d] sending file '%s'\n", pstate->ht_sockfd, pstate->ht_filename);

#ifdef CONFIG_NETUTILS_HTTPD_CGIPATH
  {
    httpd_cgifunction f;

    f = httpd_cgi(pstate->ht_filename);
    if (f != NULL)
      {
#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
        pstate->ht_keepalive = false;
#endif
        f(pstate, pstate->ht_filename);

        return OK;
      }
  }
#endif

  if (httpd_openindex(pstate) != OK)
    {
      ndbg("[%d] '%s' not found\n", pstate->ht_sockfd, pstate->ht_filename);
      return httpd_senderror(pstate, 404);
    }

#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
  ptr = strchr(pstate->ht_filename, ISO_period);
  if (ptr != NULL &&
      strncmp(ptr, ".shtml", strlen(".shtml")) == 0)
    {
#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
      pstate->ht_keepalive = false;
#endif
      if (send_headers(pstate, 200, -1) != OK)
        {
           goto done;
        }

      ret = handle_script(pstate);

      goto done;
    }
#endif

  if (send_headers(pstate, pstate->ht_file.len == 0 ? 204 : 200, pstate->ht_file.len) != OK)
    {
      goto done;
    }

#ifdef CONFIG_NETUTILS_HTTPD_SENDFILE
      ret = httpd_sendfile_send(pstate->ht_sockfd, &pstate->ht_file);
#else
      ret = send_chunk(pstate, pstate->ht_file.data, pstate->ht_file.len);
#endif

done:

  (void)httpd_close(&pstate->ht_file);

  return ret;
}

static inline int httpd_parse(struct httpd_state *pstate)
{
  char *o;

  enum
  {
    STATE_METHOD,
    STATE_HEADER,
    STATE_BODY
  } state;

  state = STATE_METHOD;
  o = pstate->ht_buffer;

  do
    {
      char *start;
      char *end;

      if (o == pstate->ht_buffer + sizeof pstate->ht_buffer)
        {
          ndbg("[%d] ht_buffer overflow\n");
          return 413;
        }

      {
        ssize_t r;

        r = recv(pstate->ht_sockfd, o,
          sizeof pstate->ht_buffer - (o - pstate->ht_buffer), 0);
        if (r == 0)
          {
            ndbg("[%d] connection lost\n", pstate->ht_sockfd);
            return ERROR;
          }

#if CONFIG_NETUTILS_HTTPD_TIMEOUT > 0
        if (r == -1 && errno == EWOULDBLOCK)
          {
            ndbg("[%d] recv timeout\n");
            return 408;
          }
#endif
        if (r == -1)
          {
            ndbg("[%d] recv failed: %d\n", pstate->ht_sockfd, errno);
            return 400;
          }

        o += r;
      }

      /* Here o marks the end of the total block currently awaiting processing.
       * There may be multiple lines in a block; next we deal with each in turn.
       */

      for (start = pstate->ht_buffer;
           (end = memchr(start, '\r', o - start)), end != NULL;
           start = end)
        {
          *end = '\0';
          end++;

          /* Here start and end are a single line within the current block */

          httpd_dumpbuffer("Incoming HTTP line", start, end - start);

          if (*end != '\n')
            {
              ndbg("[%d] expected CRLF\n");
              return 400;
            }

          end++;

          switch (state)
          {
          char *v;

          case STATE_METHOD:
            if (0 != strncmp(start, "GET ", 4))
              {
                ndbg("[%d] method not supported\n");
                return 501;
              }

            start += 4;
            v = start + strcspn(start, " ");

            if (0 != strcmp(v, " HTTP/1.0") && 0 != strcmp(v, " HTTP/1.1"))
              {
                ndbg("[%d] HTTP version not supported\n");
                return 505;
              }

            /* TODO: url decoding */

            if (v - start >= sizeof pstate->ht_filename)
              {
                ndbg("[%d] ht_filename overflow\n");
                return 414;
              }

            *v = '\0';
            (void) strcpy(pstate->ht_filename, start);
            state = STATE_HEADER;
            break;

          case STATE_HEADER:
            if (*start == '\0')
              {
                state = STATE_BODY;
                break;
              }

            v = start + strcspn(start, ":");
            if (*v != '\0')
              {
                *v = '\0', v++;
                v += strspn(v, ": ");
              }

            if (*start == '\0' || *v == '\0')
              {
                ndbg("[%d] header parse error\n");
                return 400;
              }

            nvdbg("[%d] Request header %s: %s\n", pstate->ht_sockfd, start, v);

            if (0 == strcasecmp(start, "Content-Length") && 0 != atoi(v))
              {
                ndbg("[%d] non-zero request length\n");
                return 413;
              }
#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
            else if (0 == strcasecmp(start, "Connection") && 0 == strcasecmp(v, "keep-alive"))
              {
                pstate->ht_keepalive = true;
              }
#endif
            break;

          case STATE_BODY:
            /* Not implemented */
            break;
          }
       }

      /* Shuffle down for the next block */

      memmove(pstate->ht_buffer, start, o - start);
      o -= (start - pstate->ht_buffer);
    }
  while (state != STATE_BODY);

#if !defined(CONFIG_NETUTILS_HTTPD_SENDFILE) && !defined(CONFIG_NETUTILS_HTTPD_MMAP)
  if (0 == strcmp(pstate->ht_filename, "/"))
    {
      strncpy(pstate->ht_filename, "/" CONFIG_NETUTILS_HTTPD_INDEX, strlen("/" CONFIG_NETUTILS_HTTPD_INDEX));
    }
#endif

  nvdbg("[%d] Filename: %s\n", pstate->ht_sockfd, pstate->ht_filename);

  return 200;
}

/****************************************************************************
 * Name: httpd_handler
 *
 * Description:
 *   Each time a new connection to port 80 is made, a new thread is created
 *   that begins at this entry point.  There should be exactly one argument
 *   and it should be the socket descriptor (+1).
 *
 ****************************************************************************/

static void *httpd_handler(void *arg)
{
  struct httpd_state *pstate = (struct httpd_state *)malloc(sizeof(struct httpd_state));
  int                 sockfd = (int)arg;
  int                 ret    = ERROR;

  nvdbg("[%d] Started\n", sockfd);

  /* Verify that the state structure was successfully allocated */

  if (pstate)
    {
      int status;

      /* Re-initialize the thread state structure */

      memset(pstate, 0, sizeof(struct httpd_state));
      pstate->ht_sockfd = sockfd;

#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
      do
        {
          pstate->ht_keepalive = false;
#endif
          /* Then handle the next httpd command */

          status = httpd_parse(pstate);
          if (status >= 400)
            {
              ret = httpd_senderror(pstate, status);
            }
          else
            {
              ret = httpd_sendfile(pstate);
            }

#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
        }
      while (pstate->ht_keepalive);
#endif

      /* End of command processing -- Clean up and exit */

      free(pstate);
    }

  /* Exit the task */

  nvdbg("[%d] Exitting\n", sockfd);
  close(sockfd);
  return NULL;
}

#ifdef CONFIG_NETUTILS_HTTPD_SINGLECONNECT
static void single_server(uint16_t portno, pthread_startroutine_t handler, int stacksize)
{
  struct sockaddr_in myaddr;
  socklen_t addrlen;
  int listensd;
  int acceptsd;
#ifdef CONFIG_NET_HAVE_SOLINGER
  struct linger ling;
#endif
#if CONFIG_NETUTILS_HTTPD_TIMEOUT > 0
  struct timeval tv;
#endif

  listensd = uip_listenon(portno);
  if (listensd < 0)
    {
      return;
    }

  /* Begin serving connections */

  for (;;)
    {
      addrlen = sizeof(struct sockaddr_in);
      acceptsd = accept(listensd, (struct sockaddr*)&myaddr, &addrlen);

      if (acceptsd < 0)
        {
          ndbg("accept failure: %d\n", errno);
          break;;
        }

      nvdbg("Connection accepted -- serving sd=%d\n", acceptsd);

      /* Configure to "linger" until all data is sent when the socket is closed */

#ifdef CONFIG_NET_HAVE_SOLINGER
      ling.l_onoff  = 1;
      ling.l_linger = 30;     /* timeout is seconds */
      if (setsockopt(acceptsd, SOL_SOCKET, SO_LINGER, &ling, sizeof(struct linger)) < 0)
        {
          close(acceptsd);
          ndbg("setsockopt SO_LINGER failure: %d\n", errno);
          break;;
        }
#endif

#if CONFIG_NETUTILS_HTTPD_TIMEOUT > 0
      /* Set up a receive timeout */

      tv.tv_sec  = CONFIG_NETUTILS_HTTPD_TIMEOUT;
      tv.tv_usec = 0;
      if (setsockopt(acceptsd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(struct timeval)) < 0)
        {
          close(acceptsd);
          ndbg("setsockopt SO_RCVTIMEO failure: %d\n", errno);
          break;;
        }
#endif

      /* Handle the request. This blocks until complete. */

      (void) httpd_handler((void*)acceptsd);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: httpd_listen
 *
 * Description:
 *   This is the main processing thread for the webserver.  It never returns
 *   unless an error occurs
 *
 ****************************************************************************/

int httpd_listen(void)
{
  /* Execute httpd_handler on each connection to port 80 */

#ifdef CONFIG_NETUTILS_HTTPD_SINGLECONNECT
  single_server(HTONS(80), httpd_handler, CONFIG_NETUTILS_HTTPDSTACKSIZE);
#else
  uip_server(HTONS(80), httpd_handler, CONFIG_NETUTILS_HTTPDSTACKSIZE);
#endif

  /* the server accept loop only returns on errors */

  return ERROR;
}

/****************************************************************************
 * Name: httpd_init
 *
 * Description:
 *   This function initializes the web server and should be called at system
 *   boot-up.
 *
 ****************************************************************************/

void httpd_init(void)
{
#if !defined(CONFIG_NETUTILS_HTTPD_MMAP) && !defined(CONFIG_NETUTILS_HTTPD_SENDFILE)
  httpd_fs_init();
#endif
}
