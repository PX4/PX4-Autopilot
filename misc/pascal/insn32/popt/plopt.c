/**********************************************************************
 * plopt.c
 * Load/Store Optimizations
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
#include "pinsn32.h"

#include "popt.h"
#include "polocal.h"
#include "plopt.h"

/**********************************************************************/

int LoadOptimize(void)
{
  uint32_t val;
  int nchanges = 0;
  register int i;

  TRACE(stderr, "[LoadOptimize]");

  /* At least two pcodes are need to perform Load optimizations */

  i = 0;
  while (i < nops-1)
    {
      switch (GETOP(pptr[i]))
        {
          /* Eliminate duplicate loads */

        case oLDSH   :
          if ((GETOP(pptr[i+1])  == oLDSH) &&
              (GETARG(pptr[i+1]) == GETARG(pptr[i])))
            {
              PUTOP(pptr[i+1], oDUP);
              PUTARG(pptr[i+1], 0);
              nchanges++;
              i += 2;
            } /* end if */
          else i++;
          break;

          /* Convert loads indexed by a constant to unindexed loads */

        case oPUSH  :
          /* Get the index value */

          val = (int32_t)GETARG(pptr[i]);

          /* If the following instruction is a load, add the constant
           * index value to the address and switch the opcode to the
           * unindexed form.
           */

          if (GETOP(pptr[i+1]) == oLDSXH) 
            {
              PUTOP(pptr[i+1], oLDSH);
              val += GETARG(pptr[i+1]);
              PUTARG(pptr[i+1], val);
              deletePcode (i);
              nchanges++;
            } /* end if */
          else if (GETOP(pptr[i+1]) == oLASX)
            {
              PUTOP(pptr[i+1], oLAS);
              val += GETARG(pptr[i+1]);
              PUTARG(pptr[i+1], val);
              deletePcode (i);
              nchanges++;
            } /* end else if */
          else if (GETOP(pptr[i+1]) == oLDSXB)
            {
              PUTOP(pptr[i+1], oLDSB);
              val += GETARG(pptr[i+1]);
              PUTARG(pptr[i+1], val);
              deletePcode (i);
              nchanges++;
            } /* end if */
          else if (GETOP(pptr[i+1]) == oLDSXM)
            {
              PUTOP(pptr[i+1], oLDSM);
              val += GETARG(pptr[i+1]);
              PUTARG(pptr[i+1], val);
              deletePcode (i);
              nchanges++;
            } /* end if */
          else i++;
          break;

        default     :
          i++;
          break;
        } /* end switch */
    } /* end while */
  return (nchanges);
} /* end LoadOptimize */

/**********************************************************************/
int StoreOptimize (void)
{
  uint32_t val;
  int nchanges = 0;
  register int i;

  TRACE(stderr, "[StoreOptimize]");

  /* At least two pcodes are need to perform the following Store */
  /* optimizations */

  i = 0;
  while (i < nops-1)
    {
      switch (GETOP(pptr[i]))
        {
          /* Eliminate store followed by load */

        case oSTSH :
          if ((GETOP(pptr[i+1])  == oLDSH) &&
              (GETARG(pptr[i+1]) == GETARG(pptr[i])))
            {
              PUTOP(pptr[i+1], oSTSH);
              PUTOP(pptr[i], oDUP);
              PUTARG(pptr[i], 0);
              nchanges++;
              i += 2;
            } /* end if */
          else i++;
          break;

          /* Convert stores indexed by a constant to unindexed stores */
        case oPUSH :
          /* Get the index value */

          val = (int32_t)GETARG(pptr[i]);

          /* If the following instruction is a store, add the constant
           * index value to the address and switch the opcode to the
           * unindexed form.
           */

          if (i < nops-2)
            {
              if (GETOP(pptr[i+2]) == oSTSXH)
                {
                  PUTOP(pptr[i+2], oSTSH);
                  val += GETARG(pptr[i+2]);
                  PUTARG(pptr[i+2], val);
                  deletePcode (i);
                  nchanges++;
                } /* end if */
              else if (GETOP(pptr[i+2]) == oSTSXB)
                {
                  PUTOP(pptr[i+2], oSTSB);
                  val += GETARG(pptr[i+2]);
                  PUTARG(pptr[i+2], val);
                  deletePcode (i);
                  nchanges++;
                } /* end if */
              else i++;
            } /* end if */
          else i++;
          break;

        default     : 
          i++;
          break;
        } /* end switch */
    } /* end while */

  return (nchanges);

} /* end StoreOptimize */

/**********************************************************************/

