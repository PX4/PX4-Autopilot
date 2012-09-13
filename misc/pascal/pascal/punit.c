/**********************************************************************
 * punit.c
 * Parse a pascal unit file
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
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>

#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "podefs.h"
#include "pedefs.h"
#include "poff.h"      /* FHT_ definitions */

#include "pas.h"       /* for globals */
#include "pblck.h"     /* for block(), constantDefinitionGroup(), etc. */
#include "pgen.h"      /* for pas_Generate*() */
#include "ptkn.h"      /* for getToken() */
#include "ptbl.h"      /* for addFile() */
#include "pofflib.h"   /* For poff*() functions*/
#include "perr.h"      /* for error() */
#include "pprgm.h"     /* for usesSection() */
#include "punit.h"

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

#define intAlign(x)     (((x) + (sINT_SIZE-1)) & (~(sINT_SIZE-1)))

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

static void interfaceSection          (void);
static void exportedProcedureHeading  (void);
static void exportedFunctionHeading   (void);

/***********************************************************************
 * Global Functions
 ***********************************************************************/
/* This function is called only main() when the first token parsed out
 * the specified file is 'unit'.  In this case, we are parsing a unit file
 * and generating a unit binary.
 */

void unitImplementation(void)
{
  char   *saveTknStart = tkn_strt;

  TRACE(lstFile, "[unitImplementation]");

  /* FORM: unit =
   *       unit-heading ';' interface-section implementation-section
   *       init-section '.'
   * FORM: unit-heading = 'unit' identifer
   * FORM: interface-section =
   *       'interface' [ uses-section ] interface-declaration
   * FORM: implementation-section =
   *       'implementation' [ uses-section ] declaration-group
   * FORM: init-section =
   *       'initialization statement-sequence
   *       ['finalization' statement-sequence] 'end' |
   *       compound-statement | 'end'
   *
   * On entry, the 'unit' keyword has already been parsed.  The
   * current token should point to the identifier following unit.
   */

  /* Skip over the unit identifier (the caller has already verified
   * that we are processing the correct unit).
   */

  if (token != tIDENT) error(eIDENT);

  /* Set a UNIT indication in the output poff file header */

  poffSetFileType(poffHandle, FHT_UNIT, 0, tkn_strt);
  poffSetArchitecture(poffHandle, FHA_PCODE);

  /* Discard the unit name and get the next token */

  stringSP = saveTknStart;
  getToken();

  /* Skip over the semicolon separating the unit-heading from the
   * interface-section.
   */

  if (token != ';') error(eSEMICOLON);
  else getToken();

  /* Verify that the interface-section is present
   * FORM: interface-section =
   *       'interface' [ uses-section ] interface-declaration
   */

  interfaceSection();

  /* Verify that the implementation section is present
   * FORM: implementation-section =
   *       'implementation' [ uses-section ] declaration-group
   */

  if (token != tIMPLEMENTATION) error(eIMPLEMENTATION);
  else getToken();

  FP->section = eIsImplementationSection;

  /* Check for the presence of an optional uses-section */

  if (token == tUSES)
    {
      /* Process the uses-section */

      getToken();
      usesSection();
    }

  /* Now, process the declaration-group
   *
   * FORM: implementation-section =
   *       'implementation' [ uses-section ] declaration-group
   * FORM: init-section =
   *       'initialization statement-sequence
   *       ['finalization' statement-sequence] 'end' |
   *       compound-statement | 'end'
   */

  declarationGroup(0);

  /* Process the init-section
   * FORM: init-section =
   *       'initialization statement-sequence
   *       ['finalization' statement-sequence] 'end' |
   *       compound-statement | 'end'
   *
   * Not yet... for now, we only require the 'end'
   */

  FP->section = eIsInitializationSection;
  if (token != tEND) error(eEND);
  else getToken();

  FP->section = eIsOtherSection;

  /* Verify that the unit file ends with a period */

  if (token != '.') error(ePERIOD);
}

/***********************************************************************/
/* This logic is called from usersSection after any a uses-section is
 * encountered in any file at any level.  In this case, we are only
 * going to parse the interface section from the unit file.
 */

void unitInterface(void)
{
  int32_t savedDStack  = dstack;
  TRACE(lstFile, "[unitInterface]");

  /* FORM: unit =
   *       unit-heading ';' interface-section implementation-section
   *       init-section
   * FORM: unit-heading = 'unit' identifer
   *
   * On entry, the 'unit' keyword has already been parsed.  The
   * current token should point to the identifier following unit.
   */

  /* Skip over the unit identifier (the caller has already verified
   * that we are processing the correct unit).
   */

  if (token != tIDENT) error(eIDENT);
  else getToken();

  /* Skip over the semicolon separating the unit-heading from the
   * interface-section.
   */

  if (token != ';') error(eSEMICOLON);
  else getToken();

  /* Process the interface-section
   * FORM: interface-section =
   *       'interface' [ uses-section ] interface-declaration
   */

  interfaceSection();

  /* Verify that the implementation section is present
   * FORM: implementation-section =
   *       'implementation' [ uses-section ] declaration-group
   */

  if (token != tIMPLEMENTATION) error(eIMPLEMENTATION);

  /* Then just ignore the rest of the file.  We'll let the compilation
   * of the unit file check the correctness of the implementation.
   */

  FP->section = eIsOtherSection;

  /* If we are generating a program binary, then all variables declared
   * by this logic a bonafide.  But if are generating UNIT binary, then
   * all variables declared as imported with a relative stack offset.
   * In this case, we must release any data stack allocated in this
   * process.
   */

  dstack = savedDStack;
}

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void interfaceSection(void)
{
  int16_t saveNSym   = nsym;          /* Save top of symbol table */
  int16_t saveNConst = nconst;        /* Save top of constant table */

  TRACE(lstFile, "[interfaceSection]");

  /*  FORM: interface-section =
   *       'interface' [ uses-section ] interface-declaration
   *
   * On entry, the unit-heading keyword has already been parsed.  The
   * current token should point to the identifier following unit.
   */

  if (token != tINTERFACE) error(eINTERFACE);
  else getToken();

  FP->section = eIsInterfaceSection;

  /* Check for the presence of an optional uses-section */

  if (token == tUSES)
    {
      /* Process the uses-section */

      getToken();
      usesSection();
    }

  /* Process the interface-declaration
   *
   * FORM: interface-declaration =
   *       [ constant-definition-group ] [ type-definition-group ]
   *       [ variable-declaration-group ] exported-heading
   */

   /* Process optional constant-definition-group.
    * FORM: constant-definition-group =
    *       'const' constant-definition ';' { constant-definition ';' }
    */

   if (token == tCONST)
     {
       const_strt = saveNConst;        /* Limit search to present level */
       getToken();                     /* Get identifier */
       const_strt = 0;

       /* Process constant-definition.
        * FORM: constant-definition = identifier '=' constant
        */

       constantDefinitionGroup();

     } /* end if */

   /* Process type-definition-group
    * FORM: type-definition-group =
    *       'type' type-definition ';' { type-definition ';' }
    */

   if (token == tTYPE)
     {
       const_strt = saveNConst;        /* Limit search to present level */
       sym_strt   = saveNSym;
       getToken();                     /* Get identifier */
       const_strt = 0;
       sym_strt   = 0;

       /* Process the type-definitions in the type-definition-group
        * FORM: type-definition = identifier '=' type-denoter
        */

       typeDefinitionGroup();
     } /* end if */

   /* Process the optional variable-declaration-group
    * FORM: variable-declaration-group =
    *       'var' variable-declaration { ';' variable-declaration }
    */

   if (token == tVAR)
     {
       const_strt = saveNConst;        /* Limit search to present level */
       sym_strt   = saveNSym;
       getToken();                     /* Get identifier */
       const_strt = 0;
       sym_strt   = 0;

       /* Process the variable declarations
        * FORM: variable-declaration = identifier-list ':' type-denoter
        * FORM: identifier-list = identifier { ',' identifier }
        */

       variableDeclarationGroup();
     } /* end if */

   /* Process the exported-heading
    *
    * FORM: exported-heading = 
    *       procedure-heading ';' [ directive ] |
    *       function-heading ';' [ directive ]
    */

   for (;;)
     {
       /* FORM: function-heading =
        *       'function' function-identifier [ formal-parameter-list ]
        *        ':' result-type
        */

       if (token == tFUNCTION)
         {
           const_strt = saveNConst;    /* Limit search to present level */
           sym_strt   = saveNSym;
           getToken();                 /* Get identifier */
           const_strt = 0;
           sym_strt   = 0;

           /* Process the interface declaration */

           exportedFunctionHeading();
         } /* end if */

       /* FORM: procedure-heading =
        *       'procedure' procedure-identifier [ formal-parameter-list ]
        */

       else if (token == tPROCEDURE)
         {
           const_strt = saveNConst;    /* Limit search to present level */
           sym_strt   = saveNSym;
           getToken();                 /* Get identifier */
           const_strt = 0;
           sym_strt   = 0;

           /* Process the interface declaration */

           exportedProcedureHeading();
         } /* end else if */
       else break;
     } /* end for */

   /* We are finished with the interface section */

   FP->section = eIsOtherSection;
}

/* Process Procedure Declaration Block */

static void exportedProcedureHeading(void)
{
   uint16_t procLabel = ++label;
   char    *saveChSp;
   STYPE   *procPtr;
   register int i;

   TRACE(lstFile,"[exportedProcedureHeading]");

   /* FORM: procedure-heading =
    *       'procedure' identifier [ formal-parameter-list ]
    * FORM: procedure-identifier = identifier
    *
    * On entry, token refers to token AFTER the 'procedure' reserved
    * word.
    */

   /* Process the procedure-heading */

   if (token != tIDENT)
     {
       error (eIDENT);
       return;
     } /* endif */

   procPtr = addProcedure(tkn_strt, sPROC, procLabel, 0, NULL);

   /* Mark the procedure as external */

   procPtr->sParm.p.flags |= SPROC_EXTERNAL;

   /* Save the string stack pointer so that we can release all
    * formal parameter strings later.  Then get the next token.
    */

   saveChSp = stringSP;
   getToken();

   /* NOTE:  The level associated with the PROCEDURE symbol is the level
    * At which the procedure was declared.  Everything declare within the
    * PROCEDURE is at the next level
    */

   level++;

   /* Process parameter list */

   (void)formalParameterList(procPtr);

   if (token !=  ';') error (eSEMICOLON);
   else getToken();

   /* If we are compiling a program or unit that "imports" the 
    * procedure then generate the appropriate symbol table entries 
    * in the output file to support relocation when the external
    * procedure is called.
    */

   if (includeIndex > 0)
     {
       pas_GenerateProcImport(procPtr);
     }

   /* Destroy formal parameter names */

   for (i = 1; i <= procPtr->sParm.p.nParms; i++)
     {
       procPtr[i].sName = NULL;
     }
   stringSP = saveChSp;

   /* Drop the level back to where it was */

   level--;

} /* end exportedProcedureHeading */

/***************************************************************/
/* Process Function Declaration Block */

static void exportedFunctionHeading(void)
{
   uint16_t funcLabel = ++label;
   int16_t  parameterOffset;
   char    *saveChSp;
   STYPE   *funcPtr;
   register int i;

   TRACE(lstFile,"[exportedFunctionHeading]");

   /* FORM: function-declaration =
    *       function-heading ';' directive |
    *       function-heading ';' function-block
    * FORM: function-heading =
    *       'function' function-identifier [ formal-parameter-list ]
    *       ':' result-type
    *
    * On entry token should lrefer to the function-identifier.
    */

   /* Verify function-identifier */

   if (token != tIDENT)
     {
       error (eIDENT);
       return;
     } /* endif */

   funcPtr = addProcedure(tkn_strt, sFUNC, funcLabel, 0, NULL);

   /* Mark the procedure as external */

   funcPtr->sParm.p.flags |= SPROC_EXTERNAL;

   /* NOTE:  The level associated with the FUNCTION symbol is the level
    * At which the procedure was declared.  Everything declare within the
    * PROCEDURE is at the next level
    */

   level++;

   /* Save the string stack pointer so that we can release all
    * formal parameter strings later.  Then get the next token.
    */

   saveChSp = stringSP;
   getToken();

   /* Process parameter list */

   parameterOffset = formalParameterList(funcPtr);

   /* Verify that the parameter list is followed by a colon */

   if (token !=  ':') error (eCOLON);
   else getToken();

   /* Get function type, return value type/size and offset to return value */

   if (token == sTYPE)
     {
       /* The offset to the return value is the offset to the last
        * parameter minus the size of the return value (aligned to
        * multiples of size of INTEGER).
        */

       parameterOffset        -= tknPtr->sParm.t.rsize;
       parameterOffset         = intAlign(parameterOffset);

       /* Save the TYPE for the function */

       funcPtr->sParm.p.parent = tknPtr;

       /* Skip over the result-type token */

       getToken();
     } /* end if */
   else
     {
       error(eINVTYPE);
     }

   /* Verify the final semicolon */

   if (token !=  ';') error (eSEMICOLON);
   else getToken();

   /* If we are compiling a program or unit that "imports" the 
    * function then generate the appropriate symbol table entries 
    * in the output file to support relocation when the external
    * function is called.
    */

   if (includeIndex > 0)
     {
       pas_GenerateProcImport(funcPtr);
     }

   /* Destroy formal parameter names and the function return value name */

   for (i = 1; i <= funcPtr->sParm.p.nParms; i++)
     {
       funcPtr[i].sName = ((char *) NULL);
     }
   stringSP = saveChSp;

   /* Restore the original level */

   level--;

} /* end exportedFunctionHeading */
