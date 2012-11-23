/************************************************************************
 * libc/stdlib/lib_abort.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012 Gregory Nutt. All rights reserved.
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <pthread.h>

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Global Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/

/************************************************************************
 * Private Function Prototypes
 ************************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: Abort
 *
 * Description:
 *   The abort() first unblocks the SIGABRT signal, and then raises that
 *   signal for the calling process. This results in the abnormal
 *   termination of the process unless the SIGABRT signal is caught and
 *   the signal handler does not return.
 *
 *   If the abort() function causes process termination, all open
 *   streams are closed and flushed.
 *
 *   If the SIGABRT signal is ignored, or caught by a handler that
 *   returns, the abort() function will still terminate the process.
 *   It does this by restoring the default disposition for SIGABRT and
 *   then raising the signal for a second time.
 *
 * Input parameters:
 *   None
 *
 * Returned Value:
 *  This function does not return,
 *
 ************************************************************************/

void abort(void)
{
  /* NuttX does not support standard signal functionality (like the
   * behavior of the SIGABRT signal).  So no attempt is made to provide
   * a conformant version of abort() at this time.  This version does not
   * signal the calling thread all.
   *
   * Note that pthread_exit() is called instead of exit().  That is because
   * we do no know if abort was called from a pthread or a normal thread
   * (we could find out, of course).  If abort() is called from a non-pthread,
   * then pthread_exit() should fail and fall back to call exit() anyway.
   *
   * If exit() is called (either below or via pthread_exit()), then exit()
   * will flush and close all open files and terminate the thread.  If this
   * function was called from a pthread, then pthread_exit() will complete
   * any joins, but will not flush or close any streams.
   */

#ifdef CONFIG_DISABLE_PTHREAD
  exit(EXIT_FAILURE);
#else
  pthread_exit(NULL);
#endif
}
