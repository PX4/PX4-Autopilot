/***************************************************************
 * pfunc.c
 * Standard Functions
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

#include <stdint.h>
#include <stdio.h>

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
#include "pgen.h"  /* for pas_Generate*() */
#include "ptkn.h"
#include "pinsn.h"
#include "perr.h"

/***************************************************************
 * Private Function Prototypes
 ***************************************************************/

/* Standard Pascal Functions */

static exprType absFunc    (void);    /* Integer absolute value */
static exprType predFunc   (void);
static void     ordFunc    (void);    /* Convert scalar to integer */
static exprType sqrFunc    (void);
static void     realFunc   (uint8_t fpCode);
static exprType succFunc   (void);
static void     oddFunc    (void);
static void     chrFunc    (void);
static void     fileFunc   (uint16_t opcode);

/* Enhanced Pascal functions */

/* Non-standard C-library interface functions */

static exprType getenvFunc (void);    /* Get environment string value */

/***************************************************************
 * Public Functions
 ***************************************************************/

void primeBuiltInFunctions(void)
{
}

/***************************************************************/
/* Process a standard Pascal function call */

exprType builtInFunction(void)
{
  exprType funcType = exprUnknown;

  TRACE(lstFile,"[builtInFunction]");

  /* Is the token a function? */

  if (token == tFUNC)
    {
      /* Yes, process it procedure according to the extended token type */

      switch (tknSubType)
        {
          /* Functions which return the same type as their argument */
        case txABS :
          funcType = absFunc();
          break;
        case txSQR :
          funcType = sqrFunc();
          break;
        case txPRED :
          funcType = predFunc();
          break;
        case txSUCC :
          funcType = succFunc();
          break;

        case txGETENV : /* Non-standard C library interfaces */
          funcType = getenvFunc();
          break;

          /* Functions returning INTEGER with REAL arguments */

        case txROUND :
          getToken();                          /* Skip over 'round' */
          expression(exprReal, NULL);
          pas_GenerateFpOperation(fpROUND);
          funcType = exprInteger;
          break; 
        case txTRUNC :
          getToken();                          /* Skip over 'trunc' */
          expression(exprReal, NULL);
          pas_GenerateFpOperation(fpTRUNC);
          funcType = exprInteger;
          break;

          /* Functions returning CHARACTER with INTEGER arguments. */

        case txCHR :
          chrFunc();
          funcType = exprChar;
          break;

          /* Function returning integer with scalar arguments */

        case txORD :
          ordFunc();
          funcType = exprInteger;
          break;

          /* Functions returning BOOLEAN */
        case txODD :
          oddFunc();
          funcType = exprBoolean;
          break;
        case txEOF :
          fileFunc(xEOF);
          funcType = exprBoolean;
          break;
        case txEOLN :
          fileFunc(xEOLN);
          funcType = exprBoolean;
          break;

          /* Functions returning REAL with REAL/INTEGER arguments */

        case txSQRT :
          realFunc(fpSQRT);
          funcType = exprReal;
          break;
        case txSIN :
          realFunc(fpSIN);
          funcType = exprReal;
          break;
        case txCOS :
          realFunc(fpCOS);
          funcType = exprReal;
          break;
        case txARCTAN :
          realFunc(fpATAN);
          funcType = exprReal;
          break;
        case txLN :
          realFunc(fpLN);
          funcType = exprReal;
          break;
        case txEXP :
          realFunc(fpEXP);
          funcType = exprReal;
          break;

        default :
          error(eINVALIDPROC);
          break;
        } /* end switch */
    } /* end if */

  return funcType;

} /* end builtInFunction */

void checkLParen(void)
{
   getToken();                          /* Skip over function name */
   if (token != '(') error(eLPAREN);    /* Check for '(' */
   else getToken();
}

void checkRParen(void)
{
   if (token != ')') error(eRPAREN);    /* Check for ')') */
   else getToken();
}

/***************************************************************
 * Private Functions
 ***************************************************************/

static exprType absFunc(void)
{
   exprType absType;

   TRACE(lstFile,"[absFunc]");

   /* FORM:  ABS (<simple integer/real expression>) */

   checkLParen();

   absType = expression(exprUnknown, NULL);
   if (absType == exprInteger)
      pas_GenerateSimple(opABS);
   else if (absType == exprReal)
      pas_GenerateFpOperation(fpABS);
   else
      error(eINVARG);

   checkRParen();
   return absType;

} /* end absFunc */

/**********************************************************************/

static void ordFunc(void)
{
   TRACE(lstFile,"[ordFunc]");

   /* FORM:  ORD (<scalar type>) */

   checkLParen();
   expression(exprAnyOrdinal, NULL);     /* Get any ordinal type */
   checkRParen();

} /* end ordFunc */

/**********************************************************************/

static exprType predFunc(void)
{
   exprType predType;

   TRACE(lstFile,"[predFunc]");

   /* FORM:  PRED (<simple integer expression>) */

   checkLParen();

   /* Process any ordinal expression */

   predType = expression(exprAnyOrdinal, NULL);
   checkRParen();
   pas_GenerateSimple(opDEC);
   return predType;

} /* end predFunc */

/**********************************************************************/

static exprType sqrFunc(void)
{
   exprType sqrType;

   TRACE(lstFile,"[sqrFunc]");

/* FORM:  SQR (<simple integer OR real expression>) */

   checkLParen();

   sqrType = expression(exprUnknown, NULL); /* Process any expression */
   if (sqrType == exprInteger) {

     pas_GenerateSimple(opDUP);
     pas_GenerateSimple(opMUL);

   } /* end if */
   else if (sqrType == exprReal)
     pas_GenerateFpOperation(fpSQR);

   else
     error(eINVARG);

   checkRParen();
   return sqrType;

} /* end sqrFunc */

/**********************************************************************/
static void realFunc (uint8_t fpOpCode)
{
   exprType realType;

   TRACE(lstFile,"[realFunc]");

   /* FORM:  <function identifier> (<real/integer expression>) */

   checkLParen();

   realType = expression(exprUnknown, NULL); /* Process any expression */
   if (realType == exprInteger)
     pas_GenerateFpOperation((fpOpCode | fpARG1));
   else if (realType == exprReal)
     pas_GenerateFpOperation(fpOpCode);
   else
     error(eINVARG);

   checkRParen();

} /* end realFunc */

/**********************************************************************/

static exprType succFunc(void)
{
   exprType succType;

   TRACE(lstFile,"[succFunc]");

   /* FORM:  SUCC (<simple integer expression>) */

   checkLParen();

   /* Process any ordinal expression */

   succType = expression(exprAnyOrdinal, NULL);

   checkRParen();
   pas_GenerateSimple(opINC);
   return succType;

} /* end succFunc */

/***********************************************************************/

static void oddFunc(void)
{
   TRACE(lstFile,"[oddFunc]");

   /* FORM:  ODD (<simple integer expression>) */

   checkLParen();

   /* Process any ordinal expression */

   expression(exprAnyOrdinal, NULL);
   checkRParen();
   pas_GenerateDataOperation(opPUSH, 1);
   pas_GenerateSimple(opAND);
   pas_GenerateSimple(opNEQZ);

} /* end oddFunc */

/***********************************************************************/
/* Process the standard chr function */

static void chrFunc(void)
{
   TRACE(lstFile,"[charFactor]");

   /* Form:  chr(integer expression).
    *
    * char(val) is only defined if there exists a character ch such
    * that ord(ch) = val.  If this is not the case, we will simply
    * let the returned value exceed the range of type char. */

   checkLParen();
   expression(exprInteger, NULL);
   checkRParen();

} /* end chrFunc */

/****************************************************************************/
/* EOF/EOLN function */

static void fileFunc(uint16_t opcode)
{
   TRACE(lstFile,"[fileFunc]");

   /* FORM: EOF|EOLN (<file number>) */

   checkLParen();
   if (token !=  sFILE)
     {
       error(eFILE);
     }
   else
     {
       pas_GenerateDataOperation(opINDS, sBOOLEAN_SIZE);
       pas_GenerateIoOperation(opcode, tknPtr->sParm.fileNumber);
       getToken();
       checkRParen();
     } /* end else */

} /* end fileFunc */

/**********************************************************************/
/* C library getenv interface */

static exprType getenvFunc(void)
{
  exprType stringType;

  TRACE(lstFile, "[getenvFunc]");

  /* FORM:  <string_var> = getenv(<string>) */

  checkLParen();

  /* Get the string expression representing the environment variable
   * name.
   */

  stringType = expression(exprString, NULL);

  /* Two possible kinds of strings could be returned.
   * Anything else other then 'exprString' would be an error (but
   * should happen).
   */

  if ((stringType != exprString) && (stringType != exprStkString))
    {
      error(eINVARG);
    }

  pas_BuiltInFunctionCall(lbGETENV);
  checkRParen();
  return exprCString;
}            

/***********************************************************************/
