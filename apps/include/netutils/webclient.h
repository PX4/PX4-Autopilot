/****************************************************************************
 *  apps/include/netutils/webclient.h
 * Header file for the HTTP client
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Based remotely on the uIP webclient which also has a BSD style license:
 *
 *   Author: Adam Dunkels <adam@dunkels.com>
 *   Copyright (c) 2002, Adam Dunkels.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
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

#ifndef __APPS_INCLUDE_NETUTILS_WEBCLIENT_H
#define __APPS_INCLUDE_NETUTILS_WEBCLIENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#ifndef CONFIG_WEBCLIENT_HOST
#  include <nuttx/config.h>
#endif
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_WEBCLIENT_MAXHTTPLINE
#  define CONFIG_WEBCLIENT_MAXHTTPLINE 200
#endif

#ifndef CONFIG_WEBCLIENT_MAXMIMESIZE
#  define CONFIG_WEBCLIENT_MAXMIMESIZE 32
#endif

#ifndef CONFIG_WEBCLIENT_MAXHOSTNAME
#  define CONFIG_WEBCLIENT_MAXHOSTNAME 40
#endif

#ifndef CONFIG_WEBCLIENT_MAXFILENAME
#  define CONFIG_WEBCLIENT_MAXFILENAME 100
#endif

/****************************************************************************
 * Public types
 ****************************************************************************/

/* wget calls a user provided function of the follwoing type to process
 * each received chuck of the incoming file data.  If the system has a file
 * system, then it may just write the data to a file.  Or it may buffer the
 * file in memory.  To facilitate this latter case, the caller may modify
 * the buffer address in this callback by writing to buffer and buflen. This
 * may be used, for example, to implement double buffering.
 *
 * Input Parameters:
 *   buffer - A pointer to a pointer to a buffer.  If the callee wishes to
 *       change the buffer address, it may do so in the callback by writing
 *       to buffer.
 *   offset - Offset to the beginning of valid data in the buffer.  Offset
 *       is used to skip over any HTTP header info that may be at the
 *       beginning of the buffer.
 *   datend - The end+1 offset of valid data in the buffer.  The total number
 *       of valid bytes is datend - offset.
 *   buflen - A pointer to the length of the buffer.  If the callee wishes
 *       to change the size of the buffer, it may write to buflen.
 */

typedef void (*wget_callback_t)(FAR char **buffer, int offset,
                                int datend, FAR int *buflen, FAR void *arg);

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#ifdef WGET_USE_URLENCODE
EXTERN char *web_post_str(FAR char *buffer, int *size, FAR char *name,
                          FAR char *value);
EXTERN char *web_posts_str(FAR char *buffer, int *size, FAR char **name,
                           FAR char **value, int len);
EXTERN int web_post_strlen(FAR char *name, FAR char *value);
EXTERN int web_posts_strlen(FAR char **name, FAR char **value, int len);
#endif

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
 *   arg      - User argument passed to callback.
 *
 * Returned Value:
 *   0: if the GET operation completed successfully;
 *  -1: On a failure with errno set appropriately 
 *
 ****************************************************************************/

EXTERN int wget(FAR const char *url, FAR char *buffer, int buflen,
                wget_callback_t callback, FAR void *arg);


EXTERN int wget_post(FAR const char *url, FAR const char *posts,
                     FAR char *buffer, int buflen, wget_callback_t callback,
                     FAR void *arg);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_WEBCLIENT_H */
