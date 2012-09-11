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
#include <pthread.h>
#include <errno.h>
#include <debug.h>

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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char g_httpcontenttypebinary[] = "Content-type: application/octet-stream\r\n\r\n";
static const char g_httpcontenttypecss[]    = "Content-type: text/css\r\n\r\n";
static const char g_httpcontenttypegif[]    = "Content-type: image/gif\r\n\r\n";
static const char g_httpcontenttypehtml[]   = "Content-type: text/html\r\n\r\n";
static const char g_httpcontenttypejpg[]    = "Content-type: image/jpeg\r\n\r\n";
static const char g_httpcontenttypeplain[]  = "Content-type: text/plain\r\n\r\n";
static const char g_httpcontenttypepng[]    = "Content-type: image/png\r\n\r\n";

#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
static const char g_httpextensionshtml[]    = ".shtml";
#endif
static const char g_httpextensionhtml[]     = ".html";
static const char g_httpextensioncss[]      = ".css";
static const char g_httpextensionpng[]      = ".png";
static const char g_httpextensiongif[]      = ".gif";
static const char g_httpextensionjpg[]      = ".jpg";

static const char g_http404path[]           = "/404.html";
#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
static const char g_httpindexpath[]         = "/index.shtml";
#else
static const char g_httpindexpath[]         = "/index.html";
#endif

static const char g_httpcmdget[]            = "GET ";

static const char g_httpheader200[]         =
  "HTTP/1.0 200 OK\r\n"
#ifndef CONFIG_NETUTILS_HTTPD_SERVERHEADER_DISABLE
  "Server: uIP/1.0 http://www.sics.se/~adam/uip/\r\n"
#endif
  "Connection: close\r\n";

static const char g_httpheader404[]         =
   "HTTP/1.0 404 Not found\r\n"
#ifndef CONFIG_NETUTILS_HTTPD_SERVERHEADER_DISABLE
   "Server: uIP/1.0 http://www.sics.se/~adam/uip/\r\n"
#endif
   "Connection: close\r\n";

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
  nvdbg("  scriptptr:     %p\n", pstate->ht_scriptptr);
  nvdbg("  scriptlen:     %d\n", pstate->ht_scriptlen);
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

static int httpd_addchunk(struct httpd_state *pstate, const char *buffer, int len)
{
  int newlen;
  int chunklen;
  int ret;

  do
    {
      /* Determine the size of the next chunk so that it fits into the buffer */

      newlen = pstate->ht_sndlen + len;
      if (newlen > HTTPD_IOBUFFER_SIZE)
        {
          newlen   = HTTPD_IOBUFFER_SIZE;
          chunklen = HTTPD_IOBUFFER_SIZE - pstate->ht_sndlen;
        }
      else
        {
          chunklen = len;
        }

      nvdbg("[%d] sndlen=%d len=%d newlen=%d chunklen=%d\n",
            pstate->ht_sockfd, pstate->ht_sndlen, len, newlen, chunklen);

      /* Copy that chunk into the send buffer */

      memcpy(&pstate->ht_buffer[pstate->ht_sndlen], buffer, chunklen);

      if (newlen >= HTTPD_IOBUFFER_SIZE)
        {
          /* The buffer is full.. Send what we have and reset to send again */

          httpd_dumpbuffer("Outgoing buffer", pstate->ht_buffer, newlen);
          ret = send(pstate->ht_sockfd, pstate->ht_buffer, newlen, 0);
          if (ret < 0)
            {
              return ret;
            }

          newlen = 0;
        }

      pstate->ht_sndlen = newlen;
      len              -= chunklen;
      buffer           += chunklen;
    }
  while (len > 0);

  return OK;
}

static int httpd_flush(struct httpd_state *pstate)
{
  int ret = 0;

  if (pstate->ht_sndlen > 0)
    {
      httpd_dumpbuffer("Outgoing buffer", pstate->ht_buffer, pstate->ht_sndlen);
      ret = send(pstate->ht_sockfd, pstate->ht_buffer, pstate->ht_sndlen, 0);
      if (ret >= 0)
        {
          pstate->ht_sndlen = 0;
        }
    }
  return ret;
}

static int send_headers(struct httpd_state *pstate, const char *statushdr, int len)
{
  char *ptr;
  int ret;
  nvdbg("HEADER\n");
  ret = httpd_addchunk(pstate, statushdr, len);
  if (ret < 0)
    {
      return ret;
    }

  ptr = strrchr(pstate->ht_filename, ISO_period);
  if (ptr == NULL)
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypebinary, strlen(g_httpcontenttypebinary));
    }
  else if (strncmp(g_httpextensionhtml, ptr, strlen(g_httpextensionhtml)) == 0
#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
        || strncmp(g_httpextensionshtml, ptr, strlen(g_httpextensionshtml)) == 0
#endif
       )
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypehtml, strlen(g_httpcontenttypehtml));
    }
  else if (strncmp(g_httpextensioncss, ptr, strlen(g_httpextensioncss)) == 0)
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypecss, strlen(g_httpcontenttypecss));
    }
  else if (strncmp(g_httpextensionpng, ptr, strlen(g_httpextensionpng)) == 0)
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypepng, strlen(g_httpcontenttypepng));
    }
  else if (strncmp(g_httpextensiongif, ptr, strlen(g_httpextensiongif)) == 0)
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypegif, strlen(g_httpcontenttypegif));
    }
  else if (strncmp(g_httpextensionjpg, ptr, strlen(g_httpextensionjpg)) == 0)
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypejpg, strlen(g_httpcontenttypejpg));
    }
  else
    {
      ret = httpd_addchunk(pstate, g_httpcontenttypeplain, strlen(g_httpcontenttypeplain));
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
        f(pstate, pstate->ht_filename);

        ret = OK;
        goto done;
      }
  }
#endif

  if (httpd_open(pstate->ht_filename, &pstate->ht_file) != OK)
    {
      ndbg("[%d] '%s' not found\n", pstate->ht_sockfd, pstate->ht_filename);
      memcpy(pstate->ht_filename, g_http404path, strlen(g_http404path));
      if (httpd_open(g_http404path, &pstate->ht_file) != OK)
        {
          return ERROR;
        }

      if (send_headers(pstate, g_httpheader404, strlen(g_httpheader404)) == OK)
        {
#ifdef CONFIG_NETUTILS_HTTPD_SENDFILE
          ret = httpd_sendfile_send(pstate->ht_sockfd, &pstate->ht_file);
#else
          ret = httpd_addchunk(pstate, pstate->ht_file.data, pstate->ht_file.len);
#endif
        }
    }
  else
    {
      if (send_headers(pstate, g_httpheader200, strlen(g_httpheader200)) == OK)
        {
          if (httpd_flush(pstate) < 0)
            {
              ret = ERROR;
            }
          else
            {
#ifndef CONFIG_NETUTILS_HTTPD_SCRIPT_DISABLE
              ptr = strchr(pstate->ht_filename, ISO_period);
              if (ptr != NULL &&
                  strncmp(ptr, g_httpextensionshtml, strlen(g_httpextensionshtml)) == 0)
                {
                  ret = handle_script(pstate);
                }
              else
#endif
                {
#ifdef CONFIG_NETUTILS_HTTPD_SENDFILE
                  ret = httpd_sendfile_send(pstate->ht_sockfd, &pstate->ht_file);
#else
                  ret = httpd_addchunk(pstate, pstate->ht_file.data, pstate->ht_file.len);
#endif
                }
            }
        }
    }

  (void)httpd_close(&pstate->ht_file);

done:

  /* Send anything remaining in the buffer */

  if (ret == OK && pstate->ht_sndlen > 0)
    {
      if (httpd_flush(pstate) < 0)
        {
          ret = ERROR;
        }
    }

  return ret;
}

static inline int httpd_cmd(struct httpd_state *pstate)
{
  ssize_t recvlen;
  int i;

  /* Get the next HTTP command.  We will handle only GET */

  recvlen = recv(pstate->ht_sockfd, pstate->ht_buffer, HTTPD_IOBUFFER_SIZE, 0);
  if (recvlen < 0)
    {
      ndbg("[%d] recv failed: %d\n", pstate->ht_sockfd, errno);
      return ERROR;
    }
  else if (recvlen == 0)
    {
      ndbg("[%d] connection lost\n", pstate->ht_sockfd);
      return ERROR;
    }
  httpd_dumpbuffer("Incoming buffer", pstate->ht_buffer, recvlen);

  /*  We will handle only GET */

  if (strncmp(pstate->ht_buffer, g_httpcmdget, strlen(g_httpcmdget)) != 0)
    {
      ndbg("[%d] Unsupported command\n", pstate->ht_sockfd);
      return ERROR;
    }

  /* Get the name of the file to provide */

  if (pstate->ht_buffer[4] != ISO_slash)
    {
      ndbg("[%d] Missing path\n", pstate->ht_sockfd);
      return ERROR;
    }
  else if (pstate->ht_buffer[5] == ISO_space)
    {
      strncpy(pstate->ht_filename, g_httpindexpath, strlen(g_httpindexpath));
    }
  else
    {
      for (i = 0;
           i < (HTTPD_MAX_FILENAME-1) && pstate->ht_buffer[i+4] != ISO_space;
           i++)
        {
          pstate->ht_filename[i] = pstate->ht_buffer[i+4];
        }
        pstate->ht_filename[i]='\0';
    }
  nvdbg("[%d] Filename: %s\n", pstate->ht_sockfd, pstate->ht_filename);

  /* Then send the file */

  return httpd_sendfile(pstate);
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
      /* Re-initialize the thread state structure */

      memset(pstate, 0, sizeof(struct httpd_state));
      pstate->ht_sockfd = sockfd;

      /* Then handle the next httpd command */

      ret = httpd_cmd(pstate);

      /* End of command processing -- Clean up and exit */

      free(pstate);
    }

  /* Exit the task */

  nvdbg("[%d] Exitting\n", sockfd);
  close(sockfd);
  return NULL;
}

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

  uip_server(HTONS(80), httpd_handler, CONFIG_NETUTILS_HTTPDSTACKSIZE);

  /* uip_server only returns on errors */

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
