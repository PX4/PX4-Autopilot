/****************************************************************************
 * apps/include/netutils/ftpd.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __APPS_INCLUDE_NETUTILS_FTPD_H
#define __APPS_INCLUDE_NETUTILS_FTPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Required configuration settings:  Of course TCP networking support is
 * required.  But here are a couple that are less obvious:
 *
 *   CONFIG_DISABLE_PTHREAD - pthread support is required
 *   CONFIG_DISABLE_POLL - poll() support is required
 *
 * Other FTPD configuration options thay may be of interest:
 *
 *   CONFIG_FTPD_VENDORID - The vendor name to use in FTP communications.
 *     Default: "NuttX"
 *   CONFIG_FTPD_SERVERID - The server name to use in FTP communications.
 *     Default: "NuttX FTP Server"
 *   CONFIG_FTPD_CMDBUFFERSIZE - The maximum size of one command.  Default:
 *     128 bytes.
 *   CONFIG_FTPD_DATABUFFERSIZE - The size of the I/O buffer for data
 *     transfers.  Default: 512 bytes.
 *   CONFIG_FTPD_WORKERSTACKSIZE - The stacksize to allocate for each
 *     FTP daemon worker thread.  Default:  2048 bytes.
 */

#ifdef CONFIG_DISABLE_PTHREAD
#  error "pthread support is required (CONFIG_DISABLE_PTHREAD=n)"
#endif

#ifdef CONFIG_DISABLE_POLL
#  error "poll() support is required (CONFIG_DISABLE_POLL=n)"
#endif

#ifndef CONFIG_FTPD_VENDORID
#  define CONFIG_FTPD_VENDORID "NuttX"
#endif

#ifndef CONFIG_FTPD_SERVERID
#  define CONFIG_FTPD_SERVERID "NuttX FTP Server"
#endif

#ifndef CONFIG_FTPD_CMDBUFFERSIZE
#  define CONFIG_FTPD_CMDBUFFERSIZE 128
#endif

#ifndef CONFIG_FTPD_DATABUFFERSIZE
#  define CONFIG_FTPD_DATABUFFERSIZE 512
#endif

#ifndef CONFIG_FTPD_WORKERSTACKSIZE
#  define CONFIG_FTPD_WORKERSTACKSIZE 2048
#endif

/* Interface definitions ****************************************************/

#define FTPD_ACCOUNTFLAG_NONE    (0)
#define FTPD_ACCOUNTFLAG_ADMIN   (1 << 0)
#define FTPD_ACCOUNTFLAG_SYSTEM  (1 << 1)
#define FTPD_ACCOUNTFLAG_GUEST   (1 << 2)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This "handle" describes the FTP session */

typedef FAR void *FTPD_SESSION;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftpd_open
 *
 * Description:
 *   Create an instance of the FTPD server and return a handle that can be
 *   used to run the server.
 *
 * Input Parameters:
 *    None
 *
 * Returned Value:
 *   On success, a non-NULL handle is returned that can be used to reference
 *   the server instance.
 *
 ****************************************************************************/

EXTERN FTPD_SESSION ftpd_open(void);

/****************************************************************************
 * Name: ftpd_adduser
 *
 * Description:
 *   Add one FTP user.
 *
 * Input Parameters:
 *    handle - A handle previously returned by ftpd_open
 *    accountflags - The characteristics of this user (see FTPD_ACCOUNTFLAGS_*
 *      definitions above).
 *    user - The user login name. May be NULL indicating that no login is
 *      required.
 *    passwd - The user password.  May be NULL indicating that no password
 *      is required.
 *    home - The user home directory. May be NULL.
 *
 * Returned Value:
 *   Zero is returned on success.  A negated errno value is return on
 *   failure.
 *
 ****************************************************************************/

EXTERN int ftpd_adduser(FTPD_SESSION handle, uint8_t accountflags,
                        FAR const char *user, FAR const char *passwd,
                        FAR const char *home);

/****************************************************************************
 * Name: ftpd_session
 *
 * Description:
 *   Execute the FTPD server.  This thread does not return until either (1)
 *   the timeout expires with no connection, (2) some other error occurs, or
 *   (2) a connection was accepted and an FTP worker thread was started to
 *   service the session.  Each call to ftpd_session creates on session.
 *
 * Input Parameters:
 *   handle - A handle previously returned by ftpd_open
 *   timeout - A time in milliseconds to wait for a connection. If this
 *     time elapses with no connected, the -ETIMEDOUT error will be returned.
 *
 * Returned Value:
 *   Zero is returned if the FTP worker was started.  On failure, a negated
 *   errno value is returned to indicate why the servier terminated.
 *   -ETIMEDOUT indicates that the user-provided timeout elapsed with no
 *   connection.
 *
 ****************************************************************************/

EXTERN int ftpd_session(FTPD_SESSION handle, int timeout);

/****************************************************************************
 * Name: ftpd_close
 *
 * Description:
 *   Close and destroy the handle created by ftpd_open.
 *
 * Input Parameters:
 *   handle - A handle previously returned by ftpd_open
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

EXTERN void ftpd_close(FTPD_SESSION handle);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __APPS_INCLUDE_NETUTILS_FTPD_H */
