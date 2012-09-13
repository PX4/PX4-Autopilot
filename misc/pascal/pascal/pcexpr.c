/***************************************************************
 * pexpr.c
 * Constant expression evaluation
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
#include <string.h>
#include <math.h>

#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "pedefs.h"

#include "keywords.h"
#include "pas.h"
#include "pstm.h"
#include "pexpr.h"
#include "pfunc.h"
#include "ptkn.h"
#include "perr.h"

/***************************************************************
 * Pre-processor Definitions
 ***************************************************************/

#define ADDRESS_DEREFERENCE 0x01
#define ADDRESS_FACTOR      0x02
#define INDEXED_FACTOR      0x04
#define VAR_PARM_FACTOR     0x08

#define intTrunc(x) ((x) & (~(sINT_SIZE)))

#define isRelationalOperator(t) \
   (((t) == tEQ) || ((t) == tNE) || \
    ((t) == tLT) || ((t) == tLE) || \
    ((t) == tGT) || ((t) == tGE) || \
    ((t) == tIN))

#define isRelationalType(t) \
   (((t) == tINT_CONST) || ((t) == tCHAR_CONST) || \
    (((t) == tBOOLEAN_CONST) || ((t) == tREAL_CONST)))

#define isAdditiveType(t) \
   (((t) == tINT_CONST) || ((t) == tREAL_CONST))

#define isMultiplicativeType(t) \
   (((t) == tINT_CONST) || ((t) == tREAL_CONST))

#define isLogicalType(t) \
   (((t) == tINT_CONST) || ((t) == tBOOLEAN_CONST))

/***************************************************************
 * Private Type Declarations
 ***************************************************************/

/***************************************************************
 * Private Function Prototypes
 ***************************************************************/

static void constantSimpleExpression(void);
static void constantTerm(void);
static void constantFactor(void);

/***************************************************************
 * Global Variables
 ***************************************************************/

int     constantToken;
int32_t constantInt;
double  constantReal;
char   *constantStart;

/***************************************************************
 * Private Variables
 ***************************************************************/

/***************************************************************/
/* Evaluate a simple expression of constant values */

void constantExpression(void)
{
  TRACE(lstFile,"[constantExpression]");

  /* Get the value of a simple constant expression */

  constantSimpleExpression();

  /* Is it followed by a relational operator? */

  if (isRelationalOperator(token) && isRelationalType(constantToken))
    {
      int simple1        = constantToken;
      int32_t simple1Int = constantInt;
      double simple1Real = constantReal;
      int operator       = token;

      /* Get the second simple expression */

      constantSimpleExpression();
      if (simple1 != constantToken)
        {
          /* Handle the case where the 1st argument is REAL and the
           * second is INTEGER. */

          if ((simple1 == tREAL_CONST) && (constantToken == tINT_CONST))
            {
              simple1Real = (double)simple1Int;
              simple1     = tREAL_CONST;
            }

          /* Handle the case where the 1st argument is Integer and the
           * second is REAL. */

          else if ((simple1 == tINT_CONST) && (constantToken == tREAL_CONST))
            {
              constantReal = (double)constantInt;
            }

          /* Allow the case of <scalar type> IN <set type>
           * Otherwise, the two terms must agree in type
           * -- NOT YET implemented.
           */

          else
            {
              error(eEXPRTYPE);
            }
        }

      /* Generate the comparison by type */

      switch (simple1)
        {
        case tINT_CONST :
        case tCHAR_CONST :
        case tBOOLEAN_CONST :
          switch (operator)
            {
            case tEQ :
              constantInt = (simple1Int == constantInt);
              break;
            case tNE :
              constantInt = (simple1Int != constantInt);
              break;
            case tLT :
              constantInt = (simple1Int < constantInt);
              break;
            case tLE :
              constantInt = (simple1Int <= constantInt);
              break;
            case tGT :
              constantInt = (simple1Int > constantInt);
              break;
            case tGE :
              constantInt = (simple1Int >= constantInt);
              break;
            case tIN :
              /* Not yet */
            default  :
              error(eEXPRTYPE);
              break;
            }
          break;

        case tREAL_CONST:
          switch (operator)
            {
            case tEQ :
              constantInt = (simple1Real == constantReal);
              break;
            case tNE :
              constantInt = (simple1Real != constantReal);
              break;
            case tLT :
              constantInt = (simple1Real < constantReal);
              break;
            case tLE :
              constantInt = (simple1Real <= constantReal);
              break;
            case tGT :
              constantInt = (simple1Real > constantReal);
              break;
            case tGE :
              constantInt = (simple1Real >= constantReal);
              break;
            case tIN :
              /* Not yet */
            default :
              error(eEXPRTYPE);
              break;
            }
          break;

        default :
          error(eEXPRTYPE);
          break;
        }

      /* The type resulting from these operations becomes BOOLEAN */

      constantToken = tBOOLEAN_CONST;
    }
}

/***************************************************************/
/* Process Simple Expression */

static void constantSimpleExpression(void)
{
  int16_t unary = ' ';
  int     term;
  int32_t termInt;
  double  termReal;

  TRACE(lstFile,"[constantSimpleExpression]");

  /* FORM: [+|-] <term> [{+|-} <term> [{+|-} <term> [...]]] */
  /* get +/- unary operation */

  if ((token == '+') || (token == '-'))
    {
      unary = token;
      getToken();
    }

  /* Process first (non-optional) term and apply unary operation */

  constantTerm();
  term = constantToken;
  if ((unary != ' ') && !isAdditiveType(term))
    {
      error(eINVSIGNEDCONST);
    }
  else if (unary == '-')
    {
      termInt  = -constantInt;
      termReal = -constantReal;
    }
  else
    {
      termInt  = constantInt;
      termReal = constantReal;
    }

  /* Process subsequent (optional) terms and binary operations */

  for (;;)
    {
      int operator;

      /* Check for binary operator */

      if ((((token == '+') || (token == '-')) )&& isAdditiveType(term))
        operator = token;
      else if ((token == tOR) && isLogicalType(term))
        operator = token;
      else
        break;

      /* Get the 2nd term */

      getToken();
      constantTerm();

      /* Before generating the operation, verify that the types match.
       * Perform automatic type conversion from INTEGER to REAL as
       * necessary.
       */

      if (term != constantToken)
        {
          /* Handle the case where the 1st argument is REAL and the
           * second is INTEGER. */

          if ((term == tREAL_CONST) && (constantToken == tINT_CONST))
            {
              constantReal = (double)constantInt;
              constantToken = tREAL_CONST;
            }

          /* Handle the case where the 1st argument is Integer and the
           * second is REAL. */

          else if ((term == tINT_CONST) && (constantToken == tREAL_CONST))
            {
              termReal = (double)termInt;
              term = tREAL_CONST;
            }

          /* Otherwise, the two terms must agree in type */

          else
            {
              error(eTERMTYPE);
            }
        } /* end if */


      /* Perform the selected binary operation */

      switch (term)
        {
        case tINT_CONST :
          if (operator == '+')
            {
              termInt += constantInt;
            }
          else
            {
              termInt -= constantInt;
            }
          break;

        case tREAL_CONST :
          if (operator == '+')
            {
              termReal += constantReal;
            }
          else
            {
              termReal -= constantReal;
            }
          break;

        case tBOOLEAN_CONST :
          termInt |= constantInt;
          break;

        default :
          error(eEXPRTYPE);
          break;
        }
    }

  constantToken = term;
  constantInt   = termInt;
  constantReal  = termReal;
}

/***************************************************************/
/* Evaluate a TERM */

void constantTerm(void)
{
  int     operator;
  int     factor;
  int32_t factorInt;
  double  factorReal;

  TRACE(lstFile,"[constantTerm]");

  /* FORM:  <factor> [<operator> <factor>[<operator><factor>[...]]] */

  constantFactor();
  factor     = constantToken;
  factorInt  = constantInt;
  factorReal = constantReal;
  for (;;) {
    /* Check for binary operator */

    if (((token == tMUL) || (token == tMOD)) &&
        (isMultiplicativeType(factor)))
      operator = token;
    else if (((token == tDIV) || (token == tSHL) || (token == tSHR)) &&
             (factor == tINT_CONST))
      operator = token;
    else if ((token == tFDIV) && (factor == tREAL_CONST))
      operator = token;
#if 0
    else if ((token == tFDIV) && (factor == tINT_CONST))
      {
        factorReal = (double)factorInt;
        factor     = tREAL_CONST;
        operator  = token;
      }
#endif
    else if ((token == tAND) && isLogicalType(factor))
      operator = token;
    else
      {
        constantToken = factor;
        constantInt   = factorInt;
        constantReal  = factorReal;
        break;
      }

    /* Get the next factor */

    getToken();
    constantFactor();

    /* Before generating the operation, verify that the types match.
     * Perform automatic type conversion from INTEGER to REAL as
     * necessary.
     */

    if (factor != constantToken)
      {
        /* Handle the case where the 1st argument is REAL and the
         * second is INTEGER. */

        if ((factor == tREAL_CONST) && (constantToken == tINT_CONST))
          {
            constantReal = (double)constantInt;
          }
         
        /* Handle the case where the 1st argument is Integer and the
         * second is REAL. */

        else if ((factor == tINT_CONST) && (constantToken == tREAL_CONST))
          {
            factorReal = (double)factorInt;
            factor = tREAL_CONST;
          }

        /* Otherwise, the two factors must agree in type */

        else
          {
            error(eFACTORTYPE);
          }
      } /* end if */

    /* Generate code to perform the selected binary operation */

    switch (operator)
      {
      case tMUL :
        if (factor == tINT_CONST)
          factorInt *= constantInt;
        else if (factor == tREAL_CONST)
          factorReal *= constantReal;
        else
          error(eFACTORTYPE);
        break;

      case tDIV :
        if (factor == tINT_CONST)
          factorInt /= constantInt;
        else
          error(eFACTORTYPE);
        break;

      case tFDIV :
        if (factor == tREAL_CONST)
          factorReal /= constantReal;
        else
          error(eFACTORTYPE);
        break;

      case tMOD :
        if (factor == tINT_CONST)
          factorInt %= constantInt;
        else if (factor == tREAL_CONST)
          factorReal = fmod(factorReal, constantReal);
        else
          error(eFACTORTYPE);
        break;

      case tAND :
        if ((factor == tINT_CONST) || (factor == tBOOLEAN_CONST))
          factorInt &= constantInt;
        else
          error(eFACTORTYPE);
        break;

      case tSHL :
        if (factor == tINT_CONST)
          factorInt <<= constantInt;
        else
          error(eFACTORTYPE);
        break;

      case tSHR :
        if (factor == tINT_CONST)
          factorInt >>= constantInt;
        else
          error(eFACTORTYPE);
        break;

      }
  }
}

/***************************************************************/
/* Process a FACTOR */

static void constantFactor(void)
{
  TRACE(lstFile,"[constantFactor]");

  /* Process by token type */

  switch (token)
    {
    case tINT_CONST :
    case tBOOLEAN_CONST :
    case tCHAR_CONST :
      constantToken = token;
      constantInt   = tknInt;
      getToken();
      break;

    case tREAL_CONST     :
      constantToken = token;
      constantReal  = tknReal;
      getToken();
      break;

    case tSTRING_CONST :
      constantToken = token;
      constantStart = tkn_strt;
      getToken();
      break;

      /* Highest Priority Operators */

    case tNOT:
      getToken();
      constantFactor();
      if ((constantToken != tINT_CONST) && (constantToken != tBOOLEAN_CONST))
        error(eFACTORTYPE);
      constantInt = ~constantInt;
      break;

      /* Built-in function? */

    case tFUNC:
      builtInFunctionOfConstant();
      break;

      /* Hmmm... Try the standard functions */

    default :
      error(eINVFACTOR);
      break;
    }
} 
