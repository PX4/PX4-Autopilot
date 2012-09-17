/***********************************************************************
 * pexpr.h
 * External Declarations associated with pexpr.c
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
 ***********************************************************************/

#ifndef __PEXPR_H
#define __PEXPR_H

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdint.h>

/***********************************************************************
 * Type Definitions
 ***********************************************************************/

typedef enum exprEnum
{
  exprUnknown = 0,    /* TOS value unknown */
  exprAnyOrdinal,     /* TOS = any ordinal type */
  exprAnyString,      /* TOS = any string type */

  exprInteger,        /* TOS = integer value */
  exprReal,           /* TOS = real value */
  exprChar,           /* TOS = character value */
  exprBoolean,        /* TOS = boolean(integer) value */
  exprScalar,         /* TOS = scalar(integer) value */
  exprString,         /* TOS = variable length string reference */
  exprStkString,      /* TOS = reference to string on string stack */
  exprCString,        /* TOS = pointer to C string */
  exprSet,            /* TOS = set(integer) value */
  exprArray,          /* TOS = array */
  exprRecord,         /* TOS = record */

  exprIntegerPtr,     /* TOS = pointer to integer value */
  exprRealPtr,        /* TOS = pointer to a real value */
  exprCharPtr,        /* TOS = pointer to a character value */
  exprBooleanPtr,     /* TOS = pointer to a boolean value */
  exprScalarPtr,      /* TOS = pointer to a scalar value */
  exprSetPtr,         /* TOS = pointer to a set value */
  exprArrayPtr,       /* TOS = pointer to an array */
  exprRecordPtr       /* TOS = pointer to a record */
} exprType;

/***********************************************************************
 * Global Variables
 ***********************************************************************/

extern int     constantToken;
extern int32_t constantInt;
extern double  constantReal;
extern char   *constantStart;

/***********************************************************************
 * Global Function Protypes
 ***********************************************************************/

extern exprType expression ( exprType findExprType, STYPE *typePtr );
extern exprType varParm    ( exprType varExprType, STYPE *typePtr );
extern void     arrayIndex ( int32_t size );
extern exprType getExprType( STYPE *sType );

extern void constantExpression(void);

#endif /* __PEXPR_H */
