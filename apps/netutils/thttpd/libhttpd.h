/****************************************************************************
 * netutils/thttpd/libhttpd.h
 * HTTP Protocol Library Definitions
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

#ifndef __NETUTILS_THTTPD_LIBHTTPD_H
#define __NETUTILS_THTTPD_LIBHTTPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <stdint.h>
#include <stdbool.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <time.h>

#include "config.h"
#ifdef CONFIG_THTTPD

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A few convenient defines. */

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif
#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

/* Enable special instrumentation to track down "400 Bad Request" problems */

#undef CONFIG_THTTPD_BADREQUEST /* Define to enable "Bad Request" instrumentation */

#ifdef CONFIG_THTTPD_BADREQUEST
#  if !defined(CONFIG_DEBUG_VERBOSE) || !defined(CONFIG_DEBUG_NET)
#    undef CONFIG_THTTPD_BADREQUEST
#  else
#    define BADREQUEST(s) nvdbg("Bad Request: \"%s\"\n", s)
#  endif
#endif

#ifndef CONFIG_THTTPD_BADREQUEST
#  undef  BADREQUEST
#  define BADREQUEST(s)
#endif

/* Enable special instrumentation to track down "501 Not Implemented" problems */

#undef CONFIG_THTTPD_NOTIMPLEMENTED /* Define to enable "Not Implemented" instrumentation */

#ifdef CONFIG_THTTPD_NOTIMPLEMENTED
#  if !defined(CONFIG_DEBUG_VERBOSE) || !defined(CONFIG_DEBUG_NET)
#    undef CONFIG_THTTPD_NOTIMPLEMENTED
#  else
#    define NOTIMPLEMENTED(s) nvdbg("Not Implemented: \"%s\"\n", s)
#  endif
#endif

#ifndef CONFIG_THTTPD_NOTIMPLEMENTED
#  undef  NOTIMPLEMENTED
#  define NOTIMPLEMENTED(s)
#endif

/* Enable special instrumentation to track down "500 Internal Error" problems */

#undef CONFIG_THTTPD_INTERNALERROR /* Define to enable "Internal Error" instrumentation */

#ifdef CONFIG_THTTPD_INTERNALERROR
#  if !defined(CONFIG_DEBUG_VERBOSE) || !defined(CONFIG_DEBUG_NET)
#    undef CONFIG_THTTPD_INTERNALERROR
#  else
#    define INTERNALERROR(s) nvdbg("Internal Error: \"%s\"\n", s)
#  endif
#endif

#ifndef CONFIG_THTTPD_INTERNALERROR
#  undef  INTERNALERROR
#  define INTERNALERROR(s)
#endif

/* Methods */

#define METHOD_UNKNOWN 0
#define METHOD_GET 1
#define METHOD_HEAD 2
#define METHOD_POST 3

/* States for checked_state. */

#define CHST_FIRSTWORD  0
#define CHST_FIRSTWS    1
#define CHST_SECONDWORD 2
#define CHST_SECONDWS   3
#define CHST_THIRDWORD  4
#define CHST_THIRDWS    5
#define CHST_LINE       6
#define CHST_LF         7
#define CHST_CR         8
#define CHST_CRLF       9
#define CHST_CRLFCR     10
#define CHST_BOGUS      11

#define GC_FAIL 0
#define GC_OK 1
#define GC_NO_MORE 2

#define GR_NO_REQUEST 0
#define GR_GOT_REQUEST 1
#define GR_BAD_REQUEST 2

/****************************************************************************
 * Public Type Definitions
 ****************************************************************************/

/* A multi-family sockaddr. */

#ifdef  CONFIG_NET_IPv6
typedef struct sockaddr_in6 httpd_sockaddr;
#else
typedef struct sockaddr_in httpd_sockaddr;
#endif

/* A server. */

typedef struct
{
  char *hostname;
  int   cgi_count;
  int   listen_fd;
} httpd_server;

/* A connection. */

typedef struct
{
  int initialized;
  httpd_server *hs;
  httpd_sockaddr client_addr;
  char *read_buf;
  size_t read_size, read_idx, checked_idx;
  int checked_state;
  int method;
  off_t bytes_to_send;
  off_t bytes_sent;
  char *encodedurl;
  char *decodedurl;
  char *protocol;
  char *origfilename;
  char *expnfilename;
  char *encodings;
  char *pathinfo;
  char *query;
  char *referer;
  char *useragent;
  char *accept;
  char *accepte;
  char *acceptl;
  char *cookie;
  char *contenttype;
  char *reqhost;
  char *hdrhost;
  char *hostdir;
  char *authorization;
  char *remoteuser;
  size_t maxdecodedurl, maxorigfilename, maxexpnfilename, maxencodings,
    maxpathinfo, maxquery, maxaccept, maxaccepte, maxreqhost, maxhostdir,
    maxremoteuser, maxresponse;
#ifdef CONFIG_THTTPD_TILDE_MAP2
  char *altdir;
  size_t maxaltdir;
#endif
  time_t if_modified_since, range_if;
  size_t contentlength;
  char *type;                  /* not malloc()ed */
#ifdef CONFIG_THTTPD_VHOST
  char *vhostname;             /* not malloc()ed */
#endif
  bool mime_flag;
  bool one_one;                /* HTTP/1.1 or better */
  bool got_range;
  bool tildemapped;            /* this connection got tilde-mapped */
  bool keep_alive;
  bool should_linger;
  int conn_fd;                 /* Connection to the client */
  int file_fd;                 /* Descriptor for open, outgoing file */
  off_t range_start;           /* File range start from Range= */
  off_t range_end;             /* File range end from Range= */
  struct stat sb;

  /* This is the I/O buffer that is used to buffer portions of outgoing files */

  uint16_t buflen;             /* Index to first valid data in buffer */
  uint8_t buffer[CONFIG_THTTPD_IOBUFFERSIZE];
} httpd_conn;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initializes.  Does the socket(), bind(), and listen().   Returns an
 * httpd_server* which includes a socket fd that you can select() on.
 * Return (httpd_server*) 0 on error.
 */

extern FAR httpd_server *httpd_initialize(FAR httpd_sockaddr *sa);

/* Call to unlisten/close socket(s) listening for new connections. */

extern void httpd_unlisten(httpd_server *hs);

/* Call to shut down. */

extern void httpd_terminate(httpd_server *hs);

/* When a listen fd is ready to read, call this.  It does the accept() and
 * returns an httpd_conn* which includes the fd to read the request from and
 * write the response to.  Returns an indication of whether the accept()
 * failed, succeeded, or if there were no more connections to accept.
 *
 * In order to minimize malloc()s, the caller passes in the httpd_conn.
 * The caller is also responsible for setting initialized to zero before the
 * first call using each different httpd_conn.
 */

extern int httpd_get_conn(httpd_server *hs, int listen_fd, httpd_conn *hc);

/* Checks whether the data in hc->read_buf constitutes a complete request
 * yet.  The caller reads data into hc->read_buf[hc->read_idx] and advances
 * hc->read_idx.  This routine checks what has been read so far, using
 * hc->checked_idx and hc->checked_state to keep track, and returns an
 * indication of whether there is no complete request yet, there is a
 * complete request, or there won't be a valid request due to a syntax error.
 */

extern int httpd_got_request(httpd_conn *hc);

/* Parses the request in hc->read_buf.  Fills in lots of fields in hc,
 * like the URL and the various headers.
 *
 * Returns -1 on error.
 */

extern int httpd_parse_request(httpd_conn *hc);

/* Starts sending data back to the client.  In some cases (directories,
 * CGI programs), finishes sending by itself - in those cases, hc->file_fd
 * is negative.  If there is more data to be sent, then hc->file_fd is a file
 * stream for the file to send.  If you don't have a current timeval
 * handy just pass in 0.
 *
 * Returns -1 on error.
 */

extern int httpd_start_request(httpd_conn *hc, struct timeval *nowP);

/* Actually sends any buffered response text. */

extern void httpd_write_response(httpd_conn *hc);

/* Call this to close down a connection and free the data. */

extern void httpd_close_conn(httpd_conn *hc);

/* Call this to de-initialize a connection struct and *really* free the
 * mallocced strings.
 */

extern void httpd_destroy_conn(httpd_conn *hc);

/* Send an error message back to the client. */

extern void httpd_send_err(httpd_conn *hc, int status, const char *title,
                           const char *extraheads, const char *form, const char *arg);

/* Generate a string representation of a method number. */

extern const char *httpd_method_str(int method);

/* Format a network socket to a string representation. */

extern char *httpd_ntoa(httpd_sockaddr * saP);

/* Set NDELAY mode on a socket. */

extern void httpd_set_ndelay(int fd);

/* Clear NDELAY mode on a socket. */

extern void httpd_clear_ndelay(int fd);

/* Read to requested buffer, accounting for interruptions and EOF */

extern int httpd_read(int fd, const void *buf, size_t nbytes);

/* Write the buffer completely, accounting for interruptions */

extern int httpd_write(int fd, const void *buf, size_t nbytes);

#endif /* CONFIG_THTTPD */
#endif /* __NETUTILS_THTTPD_LIBHTTPD_H */

