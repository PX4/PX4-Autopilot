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
#include "pinsn16.h"

#include "popt.h"
#include "polocal.h"
#include "plopt.h"

/**********************************************************************/

int16_t LoadOptimize(void)
{
  uint16_t val;
  int16_t  nchanges = 0;
  register int16_t i;

  TRACE(stderr, "[LoadOptimize]");

  /* At least two pcodes are need to perform Load optimizations */

  i = 0;
  while (i < nops-1)
    {
      switch (pptr[i]->op)
        {
          /* Eliminate duplicate loads */

        case oLDSH   :
          if ((pptr[i+1]->op   == oLDSH) &&
              (pptr[i+1]->arg1 == pptr[i]->arg1) &&
              (pptr[i+1]->arg2 == pptr[i]->arg2))
            {
              pptr[i+1]->op   = oDUPH;
              pptr[i+1]->arg1 = 0;
              pptr[i+1]->arg2 = 0;
              nchanges++;
              i += 2;
            } /* end if */
          else i++;
          break;

          /* Convert loads indexed by a constant to unindexed loads */

        case oPUSH  :
        case oPUSHB  :
          /* Get the index value */

          if (pptr[i]->op == oPUSH)
            {
              val = pptr[i]->arg2;
            }
          else
            {
              val = pptr[i]->arg1;
            }

          /* If the following instruction is a load, add the constant
           * index value to the address and switch the opcode to the
           * unindexed form.
           */

          if (pptr[i+1]->op == oLDSXH) 
            {
              pptr[i+1]->op = oLDSH;
              pptr[i+1]->arg2 += val;
              deletePcode (i);
              nchanges++;
            } /* end if */
          else if (pptr[i+1]->op == oLASX)
            {
              pptr[i+1]->op = oLAS;
              pptr[i+1]->arg2 += val;
              deletePcode (i);
              nchanges++;
            } /* end else if */
          else if (pptr[i+1]->op == oLDSXB)
            {
              pptr[i+1]->op = oLDSB;
              pptr[i+1]->arg2 += val;
              deletePcode (i);
              nchanges++;
            } /* end if */
          else if (pptr[i+1]->op == oLDSXM)
            {
              pptr[i+1]->op = oLDSM;
              pptr[i+1]->arg2 += val;
              deletePcode (i);
              nchanges++;
            } /* end if */
          else if (val < 256)
            {
              pptr[i]->op   = oPUSHB;
              pptr[i]->arg1 = val;
              pptr[i]->arg2 = 0;
              i++;
            } /* end else if */
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
int16_t StoreOptimize (void)
{
  uint16_t val;
  int16_t  nchanges = 0;
  register int16_t i;

  TRACE(stderr, "[StoreOptimize]");

  /* At least two pcodes are need to perform the following Store */
  /* optimizations */

  i = 0;
  while (i < nops-1)
    {
      switch (pptr[i]->op)
        {
          /* Eliminate store followed by load */

        case oSTSH :
          if ((pptr[i+1]->op   == oLDSH) &&
              (pptr[i+1]->arg1 == pptr[i]->arg1) &&
              (pptr[i+1]->arg2 == pptr[i]->arg2))
            {
              pptr[i+1]->op = oSTSH;
              pptr[i]->op   = oDUPH;
              pptr[i]->arg1 = 0;
              pptr[i]->arg2 = 0;
              nchanges++;
              i += 2;
            } /* end if */
          else i++;
          break;

          /* Convert stores indexed by a constant to unindexed stores */
       case oPUSH :
          /* Get the index value */

          if (pptr[i]->op == oPUSH)
            {
              val = pptr[i]->arg2;
            }
          else
            {
              val = pptr[i]->arg1;
            }

          /* If the following instruction is a store, add the constant
           * index value to the address and switch the opcode to the
           * unindexed form.
           */

          if (i < nops-2)
            {
              if (pptr[i+2]->op == oSTSXH)
                {
                  pptr[i+2]->op = oSTSH;
                  pptr[i+2]->arg2 += pptr[i]->arg2;
                  deletePcode (i);
                  nchanges++;
                } /* end if */
              else if (pptr[i+2]->op == oSTSXB)
                {
                  pptr[i+2]->op = oSTSB;
                  pptr[i+2]->arg2 += pptr[i]->arg2;
                  deletePcode (i);
                  nchanges++;
                } /* end if */
              else i++;
            } /* end if */
          else i++;
          break;

        case oPUSHB :
          if (i < nops-2)
            {
              if (pptr[i+2]->op == oSTSXB)
                {
                  pptr[i+2]->op = oSTSB;
                  pptr[i+2]->arg2 += pptr[i]->arg2;
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

