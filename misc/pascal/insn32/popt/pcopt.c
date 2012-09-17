/**********************************************************************
 * pcopt.c
 * Constant Expression Optimizations
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

#include "paslib.h"
#include "popt.h"
#include "polocal.h"
#include "pcopt.h"

/**********************************************************************/

int unaryOptimize(void)
{
  register uint32_t temp;
  register int i;
  int nchanges = 0;

  TRACE(stderr, "[unaryOptimize]");

  /* At least two pcodes are need to perform unary optimizations */

  i = 0;
  while (i < nops-1)
    {
      /* Check for a constant value being pushed onto the stack */

      if (GETOP(pptr[i]) == oPUSH)
        {
          switch (GETOP(pptr[i+1]))
            {
              /* Delete unary operators on constants */
            case oNEG   :
              PUTARG(pptr[i], -GETARG(pptr[i]));
              deletePcode(i+1);
              nchanges++;
              break;

            case oABS   :
              if ((int32_t)GETARG(pptr[i]) < 0)
                PUTARG(pptr[i], -(int32_t)GETARG(pptr[i]));
              deletePcode(i+1);
              nchanges++;
              break;

            case oINC   :
              PUTARG(pptr[i], GETARG(pptr[i]) + 1);
              deletePcode(i+1);
              nchanges++;
              break;

            case oDEC   :
              PUTARG(pptr[i], GETARG(pptr[i]) - 1);
              deletePcode(i+1);
              nchanges++;
              break;

            case oNOT   :
              PUTARG(pptr[i], ~GETARG(pptr[i]));
              PUTARG(pptr[i], ~(GETARG(pptr[i])));
              deletePcode(i+1);
              nchanges++;
              break;

              /* Simplify binary operations on constants */

            case oADD :
              if (GETARG(pptr[i]) == 0)
                {
                  deletePcodePair(i, (i+1));
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i+1], oINC);
                  deletePcode(i);
                  nchanges++;
                } /* end else if */
              else if (GETARG(pptr[i]) == ARGONES)
                {
                  PUTOP(pptr[i+1], oDEC);
                  deletePcode(i);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oSUB :
              if (GETARG(pptr[i]) == 0)
                {
                  deletePcodePair(i, (i+1));
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i+1], oDEC);
                  deletePcode(i);
                  nchanges++;
                } /* end else if */
              else if (GETARG(pptr[i]) == ARGONES)
                {
                  PUTOP(pptr[i+1], oINC);
                  deletePcode(i);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oMUL :
            case oDIV :
              temp = 0;
              switch (GETARG(pptr[i]))
                {
                case 1 :
                  deletePcodePair(i, (i+1));
                  nchanges++;
                  break;
                case 16384 : temp++;
                case  8192 : temp++;
                case  4096 : temp++;
                case  2048 : temp++;
                case  1024 : temp++;
                case   512 : temp++;
                case   256 : temp++;
                case   128 : temp++;
                case    64 : temp++;
                case    32 : temp++;
                case    16 : temp++;
                case     8 : temp++;
                case     4 : temp++;
                case     2 : temp++;
                  PUTARG(pptr[i], temp);
                  if (GETOP(pptr[i+1]) == oMUL)
                    PUTOP(pptr[i+1], oSLL);
                  else
                    PUTOP(pptr[i+1], oSRA);
                  nchanges++;
                  i++;
                  break;

                default :
                  i++;
                  break;
                } /* end switch */
              break;

            case oSLL :
            case oSRL :
            case oSRA :
            case oOR  :
              if (GETARG(pptr[i]) == 0)
                {
                  deletePcodePair(i, (i+1));
                  nchanges++;
                } /* end if */
              else i++;
              break;

            case oAND :
              if (GETARG(pptr[i]) == ARGONES)
                {
                  deletePcodePair(i, (i+1));
                  nchanges++;
                } /* end if */
              else i++;
              break;

              /* Delete comparisons of constants to zero */

            case oEQUZ  :
              if (GETARG(pptr[i]) == 0)
                PUTARG(pptr[i], -1);
              else
                PUTARG(pptr[i], 0);
              deletePcode(i+1);
              nchanges++;
              break;

            case oNEQZ  :
              if (GETARG(pptr[i]) != 0)
                PUTARG(pptr[i], -1);
              else
                PUTARG(pptr[i], 0);
              deletePcode(i+1);
              nchanges++;
              break;

            case oLTZ   :
              if ((int32_t)GETARG(pptr[i]) < 0)
                PUTARG(pptr[i], -1);
              else
                PUTARG(pptr[i], 0);
              deletePcode(i+1);
              nchanges++;
              break;
                              
            case oGTEZ  :
              if ((int32_t)GETARG(pptr[i]) >= 0)
                PUTARG(pptr[i], -1);
              else
                PUTARG(pptr[i], 0);
              deletePcode(i+1);
              nchanges++;
              break;

            case oGTZ   :
              if (GETARG(pptr[i]) > 0)
                PUTARG(pptr[i], -1);
              else
                PUTARG(pptr[i], 0);
              deletePcode(i+1);
              nchanges++;
              break;
                                                                                         
            case oLTEZ :
              if (GETARG(pptr[i]) <= 0)
                PUTARG(pptr[i], -1);
              else
                PUTARG(pptr[i], 0);
              deletePcode(i+1);
              nchanges++;
              break;

              /*  Simplify comparisons with certain constants */

            case oEQU   :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1],oEQUZ);
                  deletePcode(i);
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i], oDEC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oEQUZ);
                  nchanges++;
                } /* end else if */
              else if ((int32_t)GETARG(pptr[i]) == -1)
                {
                  PUTOP(pptr[i], oINC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oEQUZ);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oNEQ   :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1], oNEQZ);
                  deletePcode(i);
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i], oDEC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oNEQZ);
                  nchanges++;
                } /* end else if */
              else if ((int32_t)GETARG(pptr[i]) == -1)
                {
                  PUTOP(pptr[i], oINC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oNEQZ);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oLT    :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1], oLTZ);
                  deletePcode(i);
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i], oDEC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oLTZ);
                  nchanges++;
                } /* end else if */
              else if ((int32_t)GETARG(pptr[i]) == -1)
                {
                  PUTOP(pptr[i], oINC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oLTZ);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oGTE   :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1], oGTEZ);
                  deletePcode(i);
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i], oDEC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oGTEZ);
                  nchanges++;
                } /* end else if */
              else if ((int32_t)GETARG(pptr[i]) == -1)
                {
                  PUTOP(pptr[i], oINC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oGTEZ);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oGT    :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1], oGTZ);
                  deletePcode(i);
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i], oDEC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oGTZ);
                  nchanges++;
                } /* end else if */
              else if ((int32_t)GETARG(pptr[i]) == -1)
                {
                  PUTOP(pptr[i], oINC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oGTZ);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

            case oLTE   :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1], oLTEZ);
                  deletePcode(i);
                  nchanges++;
                } /* end if */
              else if (GETARG(pptr[i]) == 1)
                {
                  PUTOP(pptr[i], oDEC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oLTEZ);
                  nchanges++;
                } /* end else if */
              else if ((int32_t)GETARG(pptr[i]) == -1)
                {
                  PUTOP(pptr[i], oINC);
                  PUTARG(pptr[i], 0);
                  PUTOP(pptr[i+1], oLTEZ);
                  nchanges++;
                } /* end else if */
              else i++;
              break;

              /* Simplify or delete condition branches on constants */

            case oJEQUZ :
              if (GETARG(pptr[i]) == 0)
                {
                  PUTOP(pptr[i+1], oJMP);
                  deletePcode(i);
                } /* end if */
              else 
                deletePcodePair(i, (i+1));
              nchanges++;
              break;

            case oJNEQZ :
              if (GETARG(pptr[i]) != 0)
                {
                  PUTOP(pptr[i+1], oJMP);
                  deletePcode(i);
                } /* end if */
              else 
                deletePcodePair(i, (i+1));
              nchanges++;
              break;

            case oJLTZ  :
              if ((int32_t)GETARG(pptr[i]) < 0)
                {
                  PUTOP(pptr[i+1], oJMP);
                  deletePcode(i);
                } /* end if */
              else 
                deletePcodePair(i, (i+1));
              nchanges++;
              break;

            case oJGTEZ :
              if ((int32_t)GETARG(pptr[i]) >= 0)
                {
                  PUTOP(pptr[i+1], oJMP);
                  deletePcode(i);
                } /* end if */
              else 
                deletePcodePair(i, (i+1));
              nchanges++;
              break;

            case oJGTZ  :
              if (GETARG(pptr[i]) > 0)
                {
                  PUTOP(pptr[i+1], oJMP);
                  deletePcode(i);
                } /* end if */
              else 
                deletePcodePair(i, (i+1));
              nchanges++;
              break;

            case oJLTEZ :
              if (GETARG(pptr[i]) <= 0)
                {
                  PUTOP(pptr[i+1], oJMP);
                  deletePcode(i);
                } /* end if */
              else 
                deletePcodePair(i, (i+1));
              nchanges++;
              break;

            default     :
              i++;
              break;
            } /* end switch */
        } /* end if */

          /* Delete multiple modifications of DSEG pointer */

      else if (GETOP(pptr[i]) == oINDS)
        {
          if (GETOP(pptr[i+1]) == oINDS)
            {
              PUTARG(pptr[i], GETARG(pptr[i] + GETARG(pptr[i+1])));
              deletePcode(i+1);
            } /* end if */
          else i++;
        } /* end else if */
      else i++;
    } /* end while */

  return (nchanges);

} /* end unaryOptimize */

/**********************************************************************/

int binaryOptimize(void)
{
  register int i;
  int nchanges = 0;

  TRACE(stderr, "[binaryOptimize]");

  /* At least two pcodes are needed to perform the following binary */
  /* operator optimizations */

  i = 0;
  while (i < nops-2)
    {
      if (GETOP(pptr[i]) == oPUSH)
        {
          if (GETOP(pptr[i+1]) == oPUSH)
            {
              switch (GETOP(pptr[i+2]))
                {
                case oADD :
                  PUTARG(pptr[i], GETARG(pptr[i]) + GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSUB :
                  PUTARG(pptr[i], GETARG(pptr[i]) - GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oMUL :
                  PUTARG(pptr[i], GETARG(pptr[i]) * GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oDIV :
                  PUTARG(pptr[i], GETARG(pptr[i]) / (int32_t)GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oMOD :
                  PUTARG(pptr[i], GETARG(pptr[i]) % GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSLL :
                  PUTARG(pptr[i], GETARG(pptr[i]) << GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSRL :
                  PUTARG(pptr[i], GETARG(pptr[i]) >> GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSRA :
                  PUTARG(pptr[i], (int32_t)GETARG(pptr[i]) >> GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oOR  :
                  PUTARG(pptr[i], GETARG(pptr[i]) | GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oAND :
                  PUTARG(pptr[i], GETARG(pptr[i]) & GETARG(pptr[i+1]));
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oEQU :
                  if (GETARG(pptr[i]) == GETARG(pptr[i+1]))
                    PUTARG(pptr[i], -1);
                  else
                    PUTARG(pptr[i], 0);
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oNEQ :
                  if (GETARG(pptr[i]) != GETARG(pptr[i+1]))
                    PUTARG(pptr[i], -1);
                  else
                    PUTARG(pptr[i], 0);
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oLT  :
                  if ((int32_t)GETARG(pptr[i]) < (int32_t)GETARG(pptr[i+1]))
                    PUTARG(pptr[i], -1);
                  else
                    PUTARG(pptr[i], 0);
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oGTE :
                  if ((int32_t)GETARG(pptr[i]) >= (int32_t)GETARG(pptr[i+1]))
                    PUTARG(pptr[i], -1);
                  else
                    PUTARG(pptr[i], 0);
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oGT  :
                  if ((int32_t)GETARG(pptr[i]) > (int32_t)GETARG(pptr[i+1]))
                    PUTARG(pptr[i], -1);
                  else
                    PUTARG(pptr[i], 0);
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oLTE :
                  if ((int32_t)GETARG(pptr[i]) <= (int32_t)GETARG(pptr[i+1]))
                    PUTARG(pptr[i], -1);
                  else
                    PUTARG(pptr[i], 0);
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                default   :
                  i++;
                  break;
                } /* end switch */
            } /* end if */

          /* A single (constant) pcode is sufficient to perform the */
          /* following binary operator optimizations */

          else if ((GETOP(pptr[i+1]) == oLDSH) || (GETOP(pptr[i+1]) == oLDSB) ||
                   (GETOP(pptr[i+1]) == oLAS)  || (GETOP(pptr[i+1]) == oLAC))
            {
              switch (GETOP(pptr[i+2]))
                {
                case oADD :
                  if (GETARG(pptr[i]) == 0)
                    {
                      deletePcodePair(i, (i+2));
                      nchanges++;
                    } /* end if */
                  else if (GETARG(pptr[i]) == 1)
                    {
                      PUTOP(pptr[i+2], oINC);
                      deletePcode(i);
                      nchanges++;
                    } /* end else if */
                  else if (GETARG(pptr[i]) == ARGONES)
                    {
                      PUTOP(pptr[i+2], oDEC);
                      deletePcode(i);
                      nchanges++;
                    } /* end else if */
                  else i++;
                  break;

                case oSUB :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i], oNEG);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oMUL :
                  {
                    int32_t stmp32 = 0;
                    switch (GETARG(pptr[i]))
                      {
                      case 1 :
                        deletePcodePair(i, (i+2));
                        nchanges++;
                        break;
                      case 16384 : stmp32++;
                      case  8192 : stmp32++;
                      case  4096 : stmp32++;
                      case  2048 : stmp32++;
                      case  1024 : stmp32++;
                      case   512 : stmp32++;
                      case   256 : stmp32++;
                      case   128 : stmp32++;
                      case    64 : stmp32++;
                      case    32 : stmp32++;
                      case    16 : stmp32++;
                      case     8 : stmp32++;
                      case     4 : stmp32++;
                      case     2 : stmp32++;
                        PUTOP(pptr[i],    GETOP(pptr[i+1]));
                        PUTARG(pptr[i],   GETARG(pptr[i+1]));
                        PUTOP(pptr[i+1],  oPUSH);
                        PUTARG(pptr[i+1], stmp32);
                        PUTOP(pptr[i+2],  oSLL);
                        nchanges++;
                        i++;
                        break;

                      default :
                        i++;
                        break;
                      }
                  }
                  break;

                case oOR  :
                  if (GETARG(pptr[i]) == 0)
                    {
                      deletePcodePair(i, (i+2));
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oAND :
                  if (GETARG(pptr[i]) == ARGONES)
                    {
                      deletePcodePair(i, (i+2));
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oEQU :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i+2], oEQUZ);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oNEQ :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i+2], oNEQZ);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oLT  :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i+2], oGTEZ);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oGTE :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i+2], oLTZ);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oGT  :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i+2], oLTEZ);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oLTE :
                  if (GETARG(pptr[i]) == 0)
                    {
                      PUTOP(pptr[i+2], oGTZ);
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                default   :
                  i++;
                  break;

                } /* end switch */
            } /* end else if */
          else i++;
        } /* end if */

      /* Misc improvements on binary operators */

      else if (GETOP(pptr[i]) == oNEG)
        {
          /* Negation followed by add is subtraction */

          if (GETOP(pptr[i+1]) == oADD)
            {
              PUTOP(pptr[i+1], oSUB);
              deletePcode(i);
              nchanges++;
            }

          /* Negation followed by subtraction is addition */

          else if (GETOP(pptr[i]) == oSUB)
            {
              PUTOP(pptr[i+1], oADD);
              deletePcode(i);
              nchanges++;
            }
          else i++;
        }
      else i++;
    } /* end while */

  return (nchanges);

} /* end binaryOptimize */

/**********************************************************************/
