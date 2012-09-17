/***************************************************************
 * pcfunc.c
 * Standard Function operating on constant values
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
 ***************************************************************/

/***************************************************************
 * Included Files
 ***************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "podefs.h"
#include "pfdefs.h"
#include "pedefs.h"
#include "pxdefs.h"

#include "pas.h"
#include "pexpr.h"
#include "pfunc.h"
#include "ptkn.h"
#include "perr.h"

/***************************************************************
 * Private Function Prototypes
 ***************************************************************/

/* Standard Pascal Functions */

static void constantAbsFunc(void);    /* Integer absolute value */
static void constantPredFunc(void);
static void constantOrdFunc(void);    /* Convert scalar to integer */
static void constantSqrFunc(void);
static void constantRealFunc(uint8_t fpCode);
static void constantSuccFunc(void);
static void constantOddFunc(void);
static void constantChrFunc(void);
static void constantReal2IntFunc(int kind);
static void isOrdinalConstant(void);

/***************************************************************/
/* Process a standard Pascal function call */

void builtInFunctionOfConstant(void)
{
  TRACE(lstFile,"[builtInFunctionFactor]");

  /* Is the token a function? */

  if (token == tFUNC)
    {
      /* Yes, process it procedure according to the extended token type */

      switch (tknSubType)
        {
          /* Functions which return the same type as their argument */
        case txABS :
          constantAbsFunc();
          break;
        case txSQR :
          constantSqrFunc();
          break;
        case txPRED :
          constantPredFunc();
          break;
        case txSUCC :
          constantSuccFunc();
          break;

          /* Functions returning INTEGER with REAL arguments */

        case txROUND :
          constantReal2IntFunc(fpROUND);
          break; 
        case txTRUNC :
          constantReal2IntFunc(fpTRUNC);
          break;

          /* Functions returning CHARACTER with INTEGER arguments. */

        case txCHR :
          constantChrFunc();
          break;

          /* Function returning integer with scalar arguments */

        case txORD :
          constantOrdFunc();
          break;

          /* Functions returning BOOLEAN */
        case txODD :
          constantOddFunc();
          break;

          /* Functions returning REAL with REAL/INTEGER arguments */

        case txSQRT :
          constantRealFunc(fpSQRT);
          break;
        case txSIN :
          constantRealFunc(fpSIN);
          break;
        case txCOS :
          constantRealFunc(fpCOS);
          break;
        case txARCTAN :
          constantRealFunc(fpATAN);
          break;
        case txLN :
          constantRealFunc(fpLN);
          break;
        case txEXP :
          constantRealFunc(fpEXP);
          break;

        case txGETENV : /* Non-standard C library interfaces */
        case txEOLN :
        case txEOF :
        default :
          error(eINVALIDPROC);
          break;
        }
    }
}

/**********************************************************************/

static void constantAbsFunc(void)
{
   TRACE(lstFile,"[constantAbsFunc]");

   /* FORM:  ABS (<simple integer/real expression>) */

   checkLParen();
   constantExpression();

   if (constantToken == tINT_CONST)
     {
       if (constantInt < 0)
         constantInt = -constantInt;
     }
   else if (constantToken == tREAL_CONST)
     {
       if (constantReal < 0)
         constantReal = -constantInt;
     }
   else
      error(eINVARG);

   checkRParen();
}

/**********************************************************************/

static void constantOrdFunc(void)
{
   TRACE(lstFile,"[constantOrdFunc]");

   /* FORM:  ORD (<scalar type>) */

   checkLParen();
   constantExpression();
   isOrdinalConstant();
   checkRParen();
}

/**********************************************************************/

static void constantPredFunc(void)
{
   TRACE(lstFile,"[constantPredFunc]");

   /* FORM:  PRED (<simple integer expression>) */

   checkLParen();
   constantExpression();
   isOrdinalConstant();
   constantInt--;
   checkRParen();
}

/**********************************************************************/

static void constantSqrFunc(void)
{
   TRACE(lstFile,"[constantSqrFunc]");

   /* FORM:  SQR (<simple integer OR real expression>) */

   checkLParen();
   constantExpression();
   if (constantToken == tINT_CONST)
     {
       constantInt *= constantInt;
     }
   else if (constantToken == tREAL_CONST)
     {
       constantReal *= constantReal;
     }
   else
     {
       error(eINVARG);
     }

   checkRParen();
}

/**********************************************************************/

static void constantRealFunc(uint8_t fpOpCode)
{
   TRACE(lstFile,"[constantRealFunc]");

   /* FORM:  <function identifier> (<real/integer expression>) */

   checkLParen();
   constantExpression();
   if (constantToken == tINT_CONST)
     constantReal = (double)constantInt;
   else
     error(eINVARG);

   checkRParen();
}

/**********************************************************************/

static void constantSuccFunc(void)
{
   TRACE(lstFile,"[constantSuccFunc]");

   /* FORM:  SUCC (<simple integer expression>) */

   checkLParen();
   constantExpression();
   isOrdinalConstant();
   constantInt++;
   checkRParen();
}

/***********************************************************************/

static void constantOddFunc(void)
{
   TRACE(lstFile,"[constantOddFunc]");

   /* FORM:  ODD (<simple integer expression>) */

   checkLParen();
   constantExpression();
   isOrdinalConstant();
   constantInt &= 1;
   expression(exprAnyOrdinal, NULL);
   checkRParen();
}

/***********************************************************************/
/* Process the standard chr function */

static void constantChrFunc(void)
{
   TRACE(lstFile,"[constantCharFunc]");

   /* Form:  chr(integer expression).
    *
    * char(val) is only defined if there exists a character ch such
    * that ord(ch) = val.  If this is not the case, we will simply
    * let the returned value exceed the range of type char. */

   checkLParen();
   constantExpression();
   if (constantToken == tINT_CONST)
     {
       constantToken = tCHAR_CONST;
     }
   else
     {
    error(eINVARG);
     }

   checkRParen();
}

/***********************************************************************/

static void constantReal2IntFunc(int kind)
{
    error(eNOTYET);
}

/***********************************************************************/

static void isOrdinalConstant(void)
{
  if ((constantToken == tINT_CONST) || /* integer value */
      (constantToken == tCHAR_CONST)    || /* character value */
      (constantToken == tBOOLEAN_CONST))
    return;
  else
    error(eINVARG);
}

/***********************************************************************/

