/**********************************************************************
 *  pjopt.c
 *  Branch Optimizations
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

#include "keywords.h"
#include "pdefs.h"
#include "pinsn16.h"

#include "popt.h"
#include "polocal.h"
#include "pjopt.h"

/**********************************************************************/

int16_t BranchOptimize (void)
{
  int16_t nchanges = 0;
  register int16_t i;

  TRACE(stderr, "[BranchOptimize]");

  /* At least two pcodes are need to perform branch optimizations */

  i = 0;
  while (i < nops-1)
    {
      switch (pptr[i]->op)
        {
        case oNOT :
          switch (pptr[i+1]->op)
            {
            case oJEQUZ :
              pptr[i+1]->op = oJNEQZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJEQUZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oNEG :
          switch (pptr[i+1]->op)
            {
            case oJLTZ  :
              pptr[i+1]->op = oJGTZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJGTEZ :
              pptr[i+1]->op = oJLTEZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJGTZ  :
              pptr[i+1]->op = oJLTZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJLTEZ :
              pptr[i+1]->op = oJGTEZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oEQU  :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oNEQ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJNEQ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJEQU;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oNEQ  :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oEQU;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJEQU;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJNEQ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oLT   :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oGTE;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJGTE;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJLT;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oGTE  :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oLT;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJLT;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJGTE;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oGT   :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oLTE;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJLTE;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJGT;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oLTE :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oGT;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJGT;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJLTE;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oEQUZ  :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oNEQZ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJNEQZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJEQUZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oNEQZ  :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oEQUZ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
            case oJNEQZ :
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oLTZ   :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oGTEZ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJGTEZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJLTZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oGTEZ  :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oLTZ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJLTZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJGTEZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oGTZ   :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oLTEZ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJLTEZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJGTZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        case oLTEZ :
          switch (pptr[i+1]->op)
            {
            case oNOT :
              pptr[i]->op = oGTZ;
              deletePcode(i+1);
              nchanges++;
              break;

            case oJEQUZ :
              pptr[i+1]->op = oJGTZ;
              deletePcode(i);
              nchanges++;
              break;

            case oJNEQZ :
              pptr[i+1]->op = oJLTEZ;
              deletePcode(i);
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
          break;

        default     :
          i++;
          break;
        } /* end switch */
    } /* end while */
  return (nchanges);

} /* end BranchOptimize */

/**********************************************************************/

