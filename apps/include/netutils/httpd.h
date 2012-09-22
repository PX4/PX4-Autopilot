/****************************************************************************
 * apps/include/netutils/httpd.h
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based on uIP which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@sics.se>
 *   Copyright (c) 2001-2005, Adam Dunkels.
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

#ifndef __APPS_INCLUDE_NETUTILS_HTTPD_H
#define __APPS_INCLUDE_NETUTILS_HTTPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/net/uip/uip.h>

#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef __cplusplus
#  define EXTERN extern "C"
extern "C" {
#else
#  define EXTERN extern
#endif

/* As threads are created to handle each request, a stack must be allocated
 * for the thread.  Use a default if the user provided no stacksize.
 */

#ifndef  CONFIG_NETUTILS_HTTPDSTACKSIZE
#  define CONFIG_NETUTILS_HTTPDSTACKSIZE 4096
#endif

#ifndef  CONFIG_NETUTILS_HTTPDFSSTATS
#  define CONFIG_NETUTILS_HTTPDFSSTATS
#endif

#ifndef CONFIG_NETUTILS_HTTPDFILESTATS
#  define CONFIG_NETUTILS_HTTPDFILESTATS
#endif

#ifndef  CONFIG_NET_STATISTICS
#  undef CONFIG_NETUTILS_HTTPDNETSTATS
#endif

/* For efficiency reasons, the size of the IO buffer should be a multiple
 * of the TCP MSS value.  Also, the current design requires that the IO
 * buffer be sufficiently large to contain the entire GET request.
 */

#define HTTPD_IOBUFFER_SIZE (3*UIP_TCP_MSS)

/* This is the maximum size of a file path */

#if defined(CONFIG_NETUTILS_HTTPD_MMAP) || defined(CONFIG_NETUTILS_HTTPD_SENDFILE)
#define HTTPD_MAX_FILENAME PATH_MAX
#else
#define HTTPD_MAX_FILENAME  20
#endif

/****************************************************************************
 * Public types
 ****************************************************************************/

struct httpd_fs_file
{
  char *data;
  int len;
#if defined(CONFIG_NETUTILS_HTTPD_MMAP) || defined(CONFIG_NETUTILS_HTTPD_SENDFILE)
  int fd;
#endif
};

struct httpd_state
{
  char     ht_buffer[HTTPD_IOBUFFER_SIZE];  /* recv() buffer */
  char     ht_filename[HTTPD_MAX_FILENAME]; /* filename from GET command */
#ifndef CONFIG_NETUTILS_HTTPD_KEEPALIVE_DISABLE
  bool     ht_keepalive;                    /* Connection: keep-alive */
#endif
  struct httpd_fs_file ht_file;             /* Fake file data to send */
  int      ht_sockfd;                       /* The socket descriptor from accept() */
  char    *ht_scriptptr;
  uint16_t ht_scriptlen;
  uint16_t ht_sndlen;
};

struct httpd_fsdata_file
{
  const struct httpd_fsdata_file *next;
  FAR const uint8_t *name;
  FAR const uint8_t *data;
  int len;
#ifdef CONFIG_NETUTILS_HTTPDFSSTATS
  uint16_t count;
#endif
};

struct httpd_fsdata_file_noconst
{
  FAR struct httpd_fsdata_file *next;
  FAR char *name;
  FAR char *data;
  int len;
#ifdef CONFIG_NETUTILS_HTTPDFSSTATS
  uint16_t count;
#endif
};

typedef void (*httpd_cgifunction)(struct httpd_state *, char *);

struct httpd_cgi_call
{
  struct httpd_cgi_call *next;
  const char *name;
  httpd_cgifunction function;
};

/* HTTPD CGI function declaration
 *
 * Description:
 *   This macro is used for declaring a HTTPD CGI function. This function is
 *   then added to the list of HTTPD CGI functions with the httpd_cgi_register()
 *   function.
 
 * Input Paramters:
 *
 *   name     The C variable name of the function
 *   str      The string name of the function, used in the script file
 *   function A pointer to the function that implements it
 */

#define HTTPD_CGI_CALL(name, str, function) \
static void function(struct httpd_state *, char *); \
static struct httpd_cgi_call name = {NULL, str, function}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

EXTERN void httpd_init(void);
EXTERN int httpd_listen(void);
EXTERN void httpd_cgi_register(struct httpd_cgi_call *cgi_call);
EXTERN uint16_t httpd_fs_count(char *name);

EXTERN const struct httpd_fsdata_file g_httpdfs_root[];
EXTERN const int g_httpd_numfiles;

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_HTTPD_H */
