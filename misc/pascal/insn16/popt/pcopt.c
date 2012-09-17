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
#include "pinsn16.h"

#include "paslib.h"
#include "popt.h"
#include "polocal.h"
#include "pcopt.h"

/**********************************************************************/

int16_t unaryOptimize(void)
{
  int16_t nchanges = 0;
  register uint16_t temp;
  register int16_t i;

  TRACE(stderr, "[unaryOptimize]");

  /* At least two pcodes are need to perform unary optimizations */

   i = 0;
   while (i < nops-1)
     {
       /* Check for a constant value being pushed onto the stack */

       if ((pptr[i]->op == oPUSH) || (pptr[i]->op == oPUSHB))
         {
           /* Turn the oPUSHB into an oPUSH op (temporarily) */

           if (pptr[i]->op == oPUSHB)
             {
               pptr[i]->op   = oPUSH;
               pptr[i]->arg2 = pptr[i]->arg1;
               pptr[i]->arg1 = 0;
             } /* end if */

           switch (pptr[i+1]->op)
             {
               /* Delete unary operators on constants */
             case oNEG   :
               pptr[i]->arg2 = -(pptr[i]->arg2);
               deletePcode(i+1);
               nchanges++;
               break;

             case oABS   :
               if (signExtend16(pptr[i]->arg2) < 0)
                 pptr[i]->arg2 = -signExtend16(pptr[i]->arg2);
               deletePcode(i+1);
               nchanges++;
               break;

             case oINC   :
               (pptr[i]->arg2)++;
               deletePcode(i+1);
               nchanges++;
               break;

             case oDEC   :
               (pptr[i]->arg2)--;
               deletePcode(i+1);
               nchanges++;
               break;

             case oNOT   :
               pptr[i]->arg2 = ~(pptr[i]->arg2);
               deletePcode(i+1);
               nchanges++;
               break;

               /* Simplify binary operations on constants */

             case oADD :
               if (pptr[i]->arg2 == 0)
                 {
                   deletePcodePair(i, (i+1));
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i+1]->op = oINC;
                   deletePcode(i);
                   nchanges++;
                 } /* end else if */
               else if (pptr[i]->arg2 == (uint16_t)-1)
                 {
                   pptr[i+1]->op = oDEC;
                   deletePcode(i);
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oSUB :
               if (pptr[i]->arg2 == 0)
                 {
                   deletePcodePair(i, (i+1));
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i+1]->op = oDEC;
                   deletePcode(i);
                   nchanges++;
                 } /* end else if */
               else if (pptr[i]->arg2 == (uint16_t)-1)
                 {
                   pptr[i+1]->op = oINC;
                   deletePcode(i);
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oMUL :
             case oDIV :
               temp = 0;
               switch (pptr[i]->arg2)
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
                   pptr[i]->arg2 = temp;
                   if (pptr[i+1]->op == oMUL)
                     pptr[i+1]->op = oSLL;
                   else
                     pptr[i+1]->op = oSRA;
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
               if (pptr[i]->arg2 == 0)
                 {
                   deletePcodePair(i, (i+1));
                   nchanges++;
                 } /* end if */
               else i++;
               break;

             case oAND :
               if (pptr[i]->arg2 == 0xffff)
                 {
                   deletePcodePair(i, (i+1));
                   nchanges++;
                 } /* end if */
               else i++;
               break;

               /* Delete comparisons of constants to zero */

             case oEQUZ  :
               if (pptr[i]->arg2 == 0) pptr[i]->arg2 = -1;
               else pptr[i]->arg2 = 0;
               deletePcode(i+1);
               nchanges++;
               break;

             case oNEQZ  :
               if (pptr[i]->arg2 != 0)
                 pptr[i]->arg2 = -1;
               else
                 pptr[i]->arg2 = 0;
               deletePcode(i+1);
               nchanges++;
               break;

             case oLTZ   :
               if (signExtend16(pptr[i]->arg2) < 0)
                 pptr[i]->arg2 = -1;
               else
                 pptr[i]->arg2 = 0;
               deletePcode(i+1);
               nchanges++;
               break;

             case oGTEZ  :
               if (signExtend16(pptr[i]->arg2) >= 0)
                 pptr[i]->arg2 = -1;
               else
                 pptr[i]->arg2 = 0;
               deletePcode(i+1);
               nchanges++;
               break;

             case oGTZ   :
               if (pptr[i]->arg2 > 0) pptr[i]->arg2 = -1;
               else pptr[i]->arg2 = 0;
               deletePcode(i+1);
               nchanges++;
               break;

             case oLTEZ :
               if (pptr[i]->arg2 <= 0) pptr[i]->arg2 = -1;
               else pptr[i]->arg2 = 0;
               deletePcode(i+1);
               nchanges++;
               break;

               /*  Simplify comparisons with certain constants */

             case oEQU   :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oEQUZ;
                   deletePcode(i);
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i]->op   = oDEC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oEQUZ;
                   nchanges++;
                 } /* end else if */
               else if (signExtend16(pptr[i]->arg2) == -1)
                 {
                   pptr[i]->op   = oINC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oEQUZ;
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oNEQ   :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oNEQZ;
                   deletePcode(i);
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i]->op   = oDEC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oNEQZ;
                   nchanges++;
                 } /* end else if */
               else if (signExtend16(pptr[i]->arg2) == -1)
                 {
                   pptr[i]->op   = oINC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oNEQZ;
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oLT    :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oLTZ;
                   deletePcode(i);
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i]->op   = oDEC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oLTZ;
                   nchanges++;
                 } /* end else if */
               else if (signExtend16(pptr[i]->arg2) == -1)
                 {
                   pptr[i]->op   = oINC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oLTZ;
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oGTE   :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oGTEZ;
                   deletePcode(i);
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i]->op   = oDEC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oGTEZ;
                   nchanges++;
                 } /* end else if */
               else if (signExtend16(pptr[i]->arg2) == -1)
                 {
                   pptr[i]->op   = oINC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oGTEZ;
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oGT    :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oGTZ;
                   deletePcode(i);
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i]->op   = oDEC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oGTZ;
                   nchanges++;
                 } /* end else if */
               else if (signExtend16(pptr[i]->arg2) == -1)
                 {
                   pptr[i]->op   = oINC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oGTZ;
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

             case oLTE   :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oLTEZ;
                   deletePcode(i);
                   nchanges++;
                 } /* end if */
               else if (pptr[i]->arg2 == 1)
                 {
                   pptr[i]->op   = oDEC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oLTEZ;
                   nchanges++;
                 } /* end else if */
               else if (signExtend16(pptr[i]->arg2) == -1)
                 {
                   pptr[i]->op   = oINC;
                   pptr[i]->arg2 = 0;
                   pptr[i+1]->op = oLTEZ;
                   nchanges++;
                 } /* end else if */
               else i++;
               break;

         /* Simplify or delete condition branches on constants */

             case oJEQUZ :
               if (pptr[i]->arg2 == 0)
                 {
                   pptr[i+1]->op = oJMP;
                   deletePcode(i);
                 } /* end if */
               else 
                 deletePcodePair(i, (i+1));
               nchanges++;
               break;

             case oJNEQZ :
               if (pptr[i]->arg2 != 0)
                 {
                   pptr[i+1]->op = oJMP;
                   deletePcode(i);
                 } /* end if */
               else 
                 deletePcodePair(i, (i+1));
               nchanges++;
               break;

             case oJLTZ  :
               if (signExtend16(pptr[i]->arg2) < 0)
                 {
                   pptr[i+1]->op = oJMP;
                   deletePcode(i);
                 } /* end if */
               else 
                 deletePcodePair(i, (i+1));
               nchanges++;
               break;

             case oJGTEZ :
               if (signExtend16(pptr[i]->arg2) >= 0)
                 {
                   pptr[i+1]->op = oJMP;
                   deletePcode(i);
                 } /* end if */
               else 
                 deletePcodePair(i, (i+1));
               nchanges++;
               break;

             case oJGTZ  :
               if (pptr[i]->arg2 > 0)
                 {
                   pptr[i+1]->op = oJMP;
                   deletePcode(i);
                 } /* end if */
               else 
                 deletePcodePair(i, (i+1));
               nchanges++;
               break;

             case oJLTEZ :
               if (pptr[i]->arg2 <= 0)
                 {
                   pptr[i+1]->op = oJMP;
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

           /* If the oPUSH instruction is still there, see if we can now */
           /* represent it with an oPUSHB instruction */

           if ((pptr[i]->op == oPUSH) && (pptr[i]->arg2 < 256))
             {
               pptr[i]->op   = oPUSHB;
               pptr[i]->arg1 = pptr[i]->arg2;
               pptr[i]->arg2 = 0;
             } /* end if */
         } /* end if */

       /* Delete multiple modifications of DSEG pointer */

       else if (pptr[i]->op == oINDS)
         {
           if (pptr[i+1]->op == oINDS)
             {
               pptr[i]->arg2 += pptr[i+1]->arg2;
               deletePcode(i+1);
             } /* end if */
           else i++;
         } /* end else if */
       else i++;
     } /* end while */

   return (nchanges);

} /* end unaryOptimize */

/**********************************************************************/

int16_t binaryOptimize(void)
{
  int16_t nchanges = 0;
  register int16_t stmp16;
  register int16_t i;

  TRACE(stderr, "[binaryOptimize]");

  /* At least two pcodes are needed to perform the following binary */
  /* operator optimizations */

  i = 0;
  while (i < nops-2)
    {
      if ((pptr[i]->op == oPUSH) || (pptr[i]->op == oPUSHB))
        {
          if ((pptr[i+1]->op == oPUSH) || (pptr[i+1]->op == oPUSHB))
            {
              /* Turn the oPUSHBs into an oPUSHs op (temporarily) */

              if (pptr[i]->op == oPUSHB)
                {
                  pptr[i]->op   = oPUSH;
                  pptr[i]->arg2 = pptr[i]->arg1;
                  pptr[i]->arg1 = 0;
                } /* end if */

              if (pptr[i+1]->op == oPUSHB)
                {
                  pptr[i+1]->op   = oPUSH;
                  pptr[i+1]->arg2 = pptr[i+1]->arg1;
                  pptr[i+1]->arg1 = 0;
                } /* end if */

              switch (pptr[i+2]->op)
                {
                case oADD :
                  pptr[i]->arg2 += pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSUB :
                  pptr[i]->arg2 -= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oMUL :
                  pptr[i]->arg2 *= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oDIV :
                  stmp16 = pptr[i]->arg2 / signExtend16(pptr[i+1]->arg2);
                  pptr[i]->arg2 = stmp16;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oMOD :
                  pptr[i]->arg2 %= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSLL :
                  pptr[i]->arg2 <<= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSRL :
                  pptr[i]->arg2 >>= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oSRA :
                  stmp16 = (((int16_t)pptr[i]->arg2) >> pptr[i+1]->arg2);
                  pptr[i]->arg2 = (uint16_t)stmp16;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oOR  :
                  pptr[i]->arg2 |= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oAND :
                  pptr[i]->arg2 &= pptr[i+1]->arg2;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oEQU :
                  if (pptr[i]->arg2 == pptr[i+1]->arg2) pptr[i]->arg2 = -1;
                  else pptr[i]->arg2 = 0;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oNEQ :
                  if ((int16_t)pptr[i]->arg2 != (int16_t)pptr[i+1]->arg2)
                    pptr[i]->arg2 = -1;
                  else
                    pptr[i]->arg2 = 0;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oLT  :
                  if ((int16_t)pptr[i]->arg2 < (int16_t)pptr[i+1]->arg2)
                    pptr[i]->arg2 = -1;
                  else
                    pptr[i]->arg2 = 0;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oGTE :
                  if ((int16_t)pptr[i]->arg2 >= (int16_t)pptr[i+1]->arg2)
                    pptr[i]->arg2 = -1;
                  else
                    pptr[i]->arg2 = 0;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oGT  :
                  if ((int16_t)pptr[i]->arg2 > (int16_t)pptr[i+1]->arg2)
                    pptr[i]->arg2 = -1;
                  else
                    pptr[i]->arg2 = 0;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                case oLTE :
                  if ((int16_t)pptr[i]->arg2 <= (int16_t)pptr[i+1]->arg2)
                    pptr[i]->arg2 = -1;
                  else
                    pptr[i]->arg2 = 0;
                  deletePcodePair((i+1), (i+2));
                  nchanges++;
                  break;

                default   :
                  i++;
                  break;
                } /* end switch */

              /* If the oPUSH instruction is still there, see if we can now */
              /* represent it with an oPUSHB instruction */

              if (pptr[i] && (pptr[i]->op == oPUSH) && (pptr[i]->arg2 < 256))
                {
                  pptr[i]->op   = oPUSHB;
                  pptr[i]->arg1 = pptr[i]->arg2;
                  pptr[i]->arg2 = 0;
                } /* end if */

              if (pptr[i+1] && (pptr[i+1]->op == oPUSH) && (pptr[i+1]->arg2 < 256))
                {
                  pptr[i+1]->op   = oPUSHB;
                  pptr[i+1]->arg1 = pptr[i+1]->arg2;
                  pptr[i+1]->arg2 = 0;
                } /* end if */
            } /* end if */

          /* A single (constant) pcode is sufficient to perform the */
          /* following binary operator optimizations */

          else if ((pptr[i+1]->op == oLDSH) || (pptr[i+1]->op == oLDSB) ||
                   (pptr[i+1]->op == oLAS)  || (pptr[i+1]->op == oLAC))
            {
              /* Turn the oPUSHB into a oPUSH op (temporarily) */

              if (pptr[i]->op == oPUSHB)
                {
                  pptr[i]->op   = oPUSH;
                  pptr[i]->arg2 = pptr[i]->arg1;
                  pptr[i]->arg1 = 0;
                } /* end if */

              switch (pptr[i+2]->op)
                {
                case oADD :
                  if (pptr[i]->arg2 == 0)
                    {
                      deletePcodePair(i, (i+2));
                      nchanges++;
                    } /* end if */
                  else if (pptr[i]->arg2 == 1)
                    {
                      pptr[i+2]->op = oINC;
                      deletePcode(i);
                      nchanges++;
                    } /* end else if */
                  else if (pptr[i]->arg2 == (uint16_t)-1)
                    {
                      pptr[i+2]->op = oDEC;
                      deletePcode(i);
                      nchanges++;
                    } /* end else if */
                  else i++;
                  break;

                case oSUB :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i]->op = oNEG;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oMUL :
                  stmp16 = 0;
                  switch (pptr[i]->arg2)
                    {
                    case 1 :
                      deletePcodePair(i, (i+2));
                      nchanges++;
                      break;
                    case 16384 : stmp16++;
                    case  8192 : stmp16++;
                    case  4096 : stmp16++;
                    case  2048 : stmp16++;
                    case  1024 : stmp16++;
                    case   512 : stmp16++;
                    case   256 : stmp16++;
                    case   128 : stmp16++;
                    case    64 : stmp16++;
                    case    32 : stmp16++;
                    case    16 : stmp16++;
                    case     8 : stmp16++;
                    case     4 : stmp16++;
                    case     2 : stmp16++;
                      pptr[i]->op     = pptr[i+1]->op;
                      pptr[i]->arg1   = pptr[i+1]->arg1;
                      pptr[i]->arg2   = pptr[i+1]->arg2;
                      pptr[i+1]->op   = oPUSH;
                      pptr[i+1]->arg1 = 0;
                      pptr[i+1]->arg2 = stmp16;
                      pptr[i+2]->op   = oSLL;
                      nchanges++;
                      i++;
                      break;

                    default :
                      i++;
                      break;
                    } /* end switch */
                  break;

                case oOR  :
                  if (pptr[i]->arg2 == 0)
                    {
                      deletePcodePair(i, (i+2));
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oAND :
                  if (pptr[i]->arg2 == 0xffff)
                    {
                      deletePcodePair(i, (i+2));
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oEQU :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i+2]->op = oEQUZ;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oNEQ :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i+2]->op = oNEQZ;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oLT  :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i+2]->op = oGTEZ;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oGTE :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i+2]->op = oLTZ;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oGT  :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i+2]->op = oLTEZ;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                case oLTE :
                  if (pptr[i]->arg2 == 0)
                    {
                      pptr[i+2]->op = oGTZ;
                      deletePcode(i);
                      nchanges++;
                    } /* end if */
                  else i++;
                  break;

                default   :
                  i++;
                  break;

                } /* end switch */

              /* If the oPUSH instruction is still there, see if we can now */
              /* represent it with an oPUSHB instruction */

              if ((pptr[i]->op == oPUSH) && (pptr[i]->arg2 < 256))
                {
                  pptr[i]->op   = oPUSHB;
                  pptr[i]->arg1 = pptr[i]->arg2;
                  pptr[i]->arg2 = 0;
                } /* end if */
            } /* end else if */
          else i++;
        } /* end if */

      /* Misc improvements on binary operators */

      else if (pptr[i]->op == oNEG)
        {
          /* Negation followed by add is subtraction */

          if (pptr[i+1]->op == oADD)
            {
               pptr[i+1]->op = oSUB;
               deletePcode(i);
               nchanges++;
            }

          /* Negation followed by subtraction is addition */

          else if (pptr[i]->op == oSUB)
            {
               pptr[i+1]->op = oADD;
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

