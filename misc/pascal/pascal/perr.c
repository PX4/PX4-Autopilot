/**********************************************************************
 * perr.c
 * Error Handlers
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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
 **********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "config.h"
#include "keywords.h"
#include "pasdefs.h"
#include "pedefs.h"

#include "pas.h"
#include "ptkn.h"
#include "perr.h"
#if CONFIG_DEBUG
# include "ptbl.h"
#endif

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

#if CONFIG_DEBUG
#define DUMPTABLES dumpTables()
#else
#define DUMPTABLES
#endif

/**********************************************************************
 * Private Variables
 **********************************************************************/

static const char fmtErrNoToken[] =
   "Line %d:%04ld Error %02x Token %02x\n";
static const char fmtErrWithToken[] =
   "Line %d:%04ld Error %02x Token %02x (%s)\n";
static const char fmtErrAbort[] =
   "Fatal Error %d -- Compilation aborted\n";

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void printError(uint16_t errcode);

/***********************************************************************/

void errmsg(char *fmt, ...)
{
  char buffer[1024];
  va_list ap;

  /* Get the full string */

  va_start(ap, fmt);
  (void)vsprintf(buffer, fmt, ap);

  /* Then output the string to stderr, the err file, and the list file */

  fputs(buffer, stderr);
  fputs(buffer, errFile);
  fputs(buffer, lstFile);

  va_end(ap);
}

/***********************************************************************/

void warn(uint16_t errcode)
{
   TRACE(lstFile,"[warn:%04x]", errcode);

   /* Write error record to the error and list files */

   printError(errcode);

   /* Increment the count of warning */

   warn_count++;
} /* end warn */

/***********************************************************************/

void error(uint16_t errcode)
{
   TRACE(lstFile,"[error:%04x]", errcode);

#if CONFIG_DEBUG
   fatal(errcode);
#else
   /* Write error record to the error and list files */

   printError(errcode);

   /* Check if err_count has been execeeded the max allowable */

   if ((++err_count) > MAX_ERRORS)
     {
       fatal(eCOUNT);
     }
#endif

} /* end error */

/***********************************************************************/

void fatal(uint16_t errcode)
{
   TRACE(lstFile,"[fatal:%04x]", errcode);

   /* Write error record to the error and list files */

   printError( errcode );

   /* Dump the tables (if CONFIG_DEBUG) */

   DUMPTABLES;

   /* And say goodbye */

   printf(fmtErrAbort, errcode);
   fprintf(lstFile, fmtErrAbort, errcode);

   exit(1);

} /* end fatal */

/***********************************************************************/

static void printError(uint16_t errcode)
{
   /* Write error record to the error and list files */

   if ((tkn_strt) && (tkn_strt < stringSP))
     {
       fprintf (errFile, fmtErrWithToken,
                FP->include, FP->line, errcode, token, tkn_strt);
       fprintf (lstFile, fmtErrWithToken,
                FP->include, FP->line, errcode, token, tkn_strt);
       stringSP = tkn_strt; /* Clean up string stack */
     } /* end if */
   else
     {
       fprintf (errFile, fmtErrNoToken,
                FP->include, FP->line, errcode, token);
       fprintf (lstFile, fmtErrNoToken,
                FP->include, FP->line, errcode, token);
     } /* end else */
} /* end printError */

/***********************************************************************/

