/**********************************************************************
 * polocal.c
 * P-Code Local Optimizer
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
#include "podefs.h"
#include "pinsn16.h"

#include "pofflib.h"
#include "paslib.h"
#include "pinsn.h"
#include "pcopt.h"
#include "plopt.h"
#include "pjopt.h"
#include "polocal.h"

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void initPTable        (void);
static void putPCodeFromTable (void);
static void setupPointer      (void);

/**********************************************************************
 * Global Variables
 **********************************************************************/

OPTYPE  ptable [WINDOW];                /* Pcode Table */
OPTYPE *pptr   [WINDOW];                /* Valid Pcode Pointers */

int16_t nops    = 0;                    /* No. Valid Pcode Pointers */
int16_t end_out = 0;                    /* 1 = oEND pcode has been output */

/**********************************************************************
 * Private Variables
 **********************************************************************/

static poffHandle_t     myPoffHandle;         /* Handle to POFF object */
static poffProgHandle_t myPoffProgHandle;/* Handle to temporary POFF object */

/**********************************************************************
 * Global Functions
 **********************************************************************/

/***********************************************************************/

void localOptimization(poffHandle_t poffHandle,
                       poffProgHandle_t poffProgHandle)
{
  int16_t nchanges;

  TRACE(stderr, "[pass2]");

  /* Save the handles for use by other, private functions */

  myPoffHandle     = poffHandle;
  myPoffProgHandle = poffProgHandle;

  /* Initialization */

  initPTable();

  /* Outer loop traverse the file op-code by op-code until the oEND P-Code
   * has been output.  NOTE:  it is assumed throughout that oEND is the 
   * final P-Code in the program data section.
   */

  while (!(end_out))
    {
      /* The inner loop optimizes the buffered P-Codes until no further
       * changes can be made.  Then the outer loop will advance the buffer
       * by one P-Code
       */

      do
        {
          nchanges  = unaryOptimize ();
          nchanges += binaryOptimize();
          nchanges += BranchOptimize();
          nchanges += LoadOptimize();
          nchanges += StoreOptimize();
        } while (nchanges);

      putPCodeFromTable();
    }
}

/***********************************************************************/

void deletePcode(int16_t delIndex)
{
  TRACE(stderr, "[deletePcode]");

  pptr[delIndex]->op   = oNOP;
  pptr[delIndex]->arg1 = 0;
  pptr[delIndex]->arg2 = 0;
  setupPointer();
}

/**********************************************************************/

void deletePcodePair(int16_t delIndex1, int16_t delIndex2)
{
  TRACE(stderr, "[deletePcodePair]");

  pptr[delIndex1]->op   = oNOP;
  pptr[delIndex1]->arg1 = 0;
  pptr[delIndex1]->arg2 = 0;
  pptr[delIndex2]->op   = oNOP;
  pptr[delIndex2]->arg1 = 0;
  pptr[delIndex2]->arg2 = 0;
  setupPointer();
}

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static void putPCodeFromTable(void)
{
  register int16_t i;

  TRACE(stderr, "[putPCodeFromTable]");

  /* Transfer all buffered P-Codes (except NOPs) to the optimized file */
  do
    {
      if ((ptable[0].op != oNOP) && !(end_out))
        {
          (void)poffAddTmpProgByte(myPoffProgHandle, ptable[0].op);

          if (ptable[0].op & o8)  
            (void)poffAddTmpProgByte(myPoffProgHandle, ptable[0].arg1);

          if (ptable[0].op & o16)
            { 
              (void)poffAddTmpProgByte(myPoffProgHandle,
                                       (ptable[0].arg2 >> 8));
              (void)poffAddTmpProgByte(myPoffProgHandle,
                                       (ptable[0].arg2 & 0xff));
            }

          end_out =(ptable[0].op == oEND);
        }

      /* Move all P-Codes down one slot */

      for (i = 1; i < WINDOW; i++)
        {
          ptable[i-1].op   = ptable[i].op ;
          ptable[i-1].arg1 = ptable[i].arg1;
          ptable[i-1].arg2 = ptable[i].arg2;
        }

      /* Then fill the end slot with a new P-Code from the input file */

      insn_GetOpCode(myPoffHandle, &ptable[WINDOW-1]);

    } while (ptable[0].op == oNOP);
  setupPointer();
}

/**********************************************************************/

static void setupPointer(void)
{
  register int16_t pindex;

  TRACE(stderr, "[setupPointer]");

  for (pindex = 0; pindex < WINDOW; pindex++)
    pptr[pindex] = (OPTYPE *) NULL;

  nops = 0; 
  for (pindex = 0; pindex < WINDOW; pindex++)
    {
      switch (ptable[pindex].op)
        {
          /* Terminate list when a break from sequential logic is
           * encountered
           */

        case oRET   :
        case oEND   :
        case oJMP   :
        case oLABEL :
        case oPCAL  :
          return;

          /* Terminate list when a condition break from sequential logic is
           * encountered but include the conditional branch in the list
           */

        case oJEQUZ :
        case oJNEQZ :
        case oJLTZ  :
        case oJGTEZ :
        case oJGTZ  :
        case oJLTEZ :
          pptr[nops] = &ptable[pindex];
          nops++;
          return;

          /* Skip over NOPs and comment class pcodes */

        case oNOP   :
        case oLINE  :
          break;

          /* Include all other pcodes in the optimization list and continue */

        default     :
          pptr[nops] = &ptable[pindex];
          nops++;
        }
    }
}

/**********************************************************************/

static void initPTable(void)
{
  register int16_t i;

  TRACE(stderr, "[intPTable]");

  /* Skip over leading pcodes.  NOTE:  assumes executable begins after
   * the first oLABEL pcode
   */

  do
    {
      insn_GetOpCode(myPoffHandle, &ptable[0]);

      (void)poffAddTmpProgByte(myPoffProgHandle, ptable[0].op);

      if (ptable[0].op & o8)  
        {
          (void)poffAddTmpProgByte(myPoffProgHandle, ptable[0].arg1);
        }

      if (ptable[0].op & o16)
        { 
          (void)poffAddTmpProgByte(myPoffProgHandle, (ptable[0].arg2 >> 8));
          (void)poffAddTmpProgByte(myPoffProgHandle, (ptable[0].arg2 & 0xff));
        } /* end if */
    }
  while ((ptable[0].op != oLABEL) && (ptable[0].op != oEND));

  /* Fill the pcode window and setup pointers to working section */

  for (i = 0; i < WINDOW; i++)
    {
      insn_GetOpCode(myPoffHandle, &ptable[i]);
    }
  setupPointer();
}

/***********************************************************************/
