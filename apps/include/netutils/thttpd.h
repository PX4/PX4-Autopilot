/****************************************************************************
 *  apps/include/netutils/thttpd.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __APPS_INCLUDE_NETUTILS_THTTPD_H
#define __APPS_INCLUDE_NETUTILS_THTTPD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/binfmt/symtab.h>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* These values must be provided by the user before the THTTPD task daemon
 * is started:
 *
 * g_thttpdsymtab:  A symbol table describing all of the symbols exported
 *   from the base system.  These symbols are used to bind address references
 *   in CGI programs to NuttX.
 * g_nsymbols:  The number of symbols in g_thttpdsymtab[].
 *
 * (See examples/nxflat and examples/thttpd for examples of how such a symbol
 *  table may be created.)
 */

EXTERN FAR const struct symtab_s *g_thttpdsymtab;
EXTERN int                        g_thttpdnsymbols;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: thttpd_main
 *
 * Description:
 *   This function is the entrypoint into the THTTPD server.  It does not
 *   return.  It may be called, the normal mechanism for starting the server
 *   is:
 *
 *   1) Set is g_thttpdsymtab and g_thttpdnsymbols.  The user is required
 *      to provide a symbol table to use for binding CGI programs (if CGI
 *      is enabled.  See examples/nxflat and examples/thttpd for examples of
 *      how such a symbol table may be created.)
 *   2) Call task_create() to start thttpd_main()
 *
 ****************************************************************************/

EXTERN int thttpd_main(int argc, char **argv);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __APPS_INCLUDE_NETUTILS_THTTPD_H */
