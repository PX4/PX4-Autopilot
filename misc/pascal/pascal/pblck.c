/***************************************************************
 * pblck.c
 * Process a Pascal Block
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

#include <stdio.h>
#include <string.h>

#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "pedefs.h"
#include "podefs.h"

#include "pas.h"
#include "pblck.h"
#include "pexpr.h"
#include "pstm.h"
#include "pgen.h"
#include "ptkn.h"
#include "ptbl.h"
#include "pinsn.h"
#include "perr.h"

/***************************************************************
 * Private Definitions
 ***************************************************************/

/* This macro implements a test for:
 * FORM:  unsigned-constant = integer-number | real-number |
 *       character-literal | string-literal | constant-identifier |
 *       'nil'
 */

#define isConstant(x) \
        (  ((x) == tINT_CONST) \
        || ((x) == tBOOLEAN_CONST) \
        || ((x) == tCHAR_CONST) \
        || ((x) == tREAL_CONST) \
        || ((x) == sSCALAR_OBJECT))

#define isIntAligned(x) (((x) & (sINT_SIZE-1)) == 0)
#define intAlign(x)     (((x) + (sINT_SIZE-1)) & (~(sINT_SIZE-1)))

/***************************************************************
 * Private Function Prototypes
 ***************************************************************/

static void   pas_DeclareLabel          (void);
static void   pas_DeclareConst          (void);
static STYPE *pas_DeclareType           (char *typeName);
static STYPE *pas_DeclareOrdinalType    (char *typeName);
static STYPE *pas_DeclareVar            (void);
static void   pas_DeclareFile           (void);
static void   pas_ProcedureDeclaration  (void);
static void   pas_FunctionDeclaration   (void);

static void   pas_SetTypeSize           (STYPE *typePtr, bool allocate);
static STYPE *pas_TypeIdentifier        (bool allocate);
static STYPE *pas_TypeDenoter           (char *typeName, bool allocate);
static STYPE *pas_NewComplexType        (char *typeName);
static STYPE *pas_NewOrdinalType        (char *typeName);
static STYPE *pas_OrdinalTypeIdentifier (bool allocate);
static STYPE *pas_GetArrayType          (void);
static STYPE *pas_DeclareRecord         (char *recordName);
static STYPE *pas_DeclareField          (STYPE *recordPtr);
static STYPE *pas_DeclareParameter      (bool pointerType);
static bool   pas_IntAlignRequired      (STYPE *typePtr);

/***************************************************************
 * Private Global Variables
 ***************************************************************/

static int32_t g_nParms;
static int32_t g_dwVarSize;                      

/***************************************************************
 * Public Functions
 ***************************************************************/
/* Process BLOCK.  This function implements:
 *
 * block = declaration-group compound-statement
 *
 * Where block can appear in the followinging:
 *
 * function-block = block
 * function-declaration =
 *     function-heading ';' directive |
 *     function-heading ';' function-block
 *
 * procedure-block = block
 * procedure-declaration =
 *     procedure-heading ';' directive |
 *     procedure-heading ';' procedure-block
 *
 * program = program-heading ';' [ uses-section ] block '.'
 */

void block()
{
  uint16_t beginLabel   = ++label;     /* BEGIN label */
  int32_t saveDStack   = dstack;      /* Save DSEG size */
  char   *saveStringSP = stringSP;    /* Save top of string stack */
  int16_t saveNSym     = nsym;        /* Save top of symbol table */
  int16_t saveNConst   = nconst;      /* Save top of constant table */
  register int16_t i;

  TRACE(lstFile,"[block]");

  /* When we enter block at level zero, then we must be at the
   * entry point to the program.  Save the entry point label
   * in the POFF file.
   */

  if ((level == 0) && (FP0->kind == eIsProgram))
    {
      poffSetEntryPoint(poffHandle, label);
    }

  /* Init size of the new DSEG */

  dstack = 0;

  /* FORM: block = declaration-group compound-statement
   * Process the declaration-group
   *
   * declaration-group =
   *     label-declaration-group |
   *     constant-definition-group |
   *     type-definition-group |
   *     variable-declaration-group |
   *     function-declaration  |
   *     procedure-declaration
   */

  declarationGroup(beginLabel);

  /* Process the compound-statement
   *
   * FORM: compound-statement = 'begin' statement-sequence 'end'
   */

  /* Verify that the compound-statement begins with BEGIN */

  if (token != tBEGIN)
    {
      error (eBEGIN);
    }

  /* It may be necessary to jump around some local functions to
   * get to the main body of the block.  If any jumps are generated,
   * they will come to the beginLabel emitted here.
   */

  pas_GenerateDataOperation(opLABEL, (int32_t)beginLabel);

  /* Since we don't know for certain how we got here, invalidate
   * the level stack pointer (LSP).  This is, of course, only 
   * meaningful on architectures that implement an LSP.
   */

  pas_InvalidateCurrentStackLevel();

  /* Then emit the compoundStatement itself */

  if (dstack)
    {
      pas_GenerateDataOperation(opINDS, (int32_t)dstack);
    }

  compoundStatement();

  if (dstack)
    {
      pas_GenerateDataOperation(opINDS, -(int32_t)dstack);
    }

  /* Make sure all declared labels were defined in the block */

  verifyLabels(saveNSym);

  /* Re-initialize file table -- clear files defined in this level */

  for (i = 0; i <= MAX_FILES; i++)
    {
      if ((files [i].defined) && (files [i].flevel >= level)) {
        files [i].defined = 0;
        files [i].flevel  = 0;
        files [i].ftype   = 0;
        files [i].faddr   = 0;
        files [i].fsize   = 0;
      }
    }

  /* "Pop" declarations local to this block */

  dstack   = saveDStack;               /* Restore old DSEG size */
  stringSP = saveStringSP;             /* Restore top of string stack */
  nsym     = saveNSym;                 /* Restore top of symbol table */
  nconst   = saveNConst;               /* Restore top of constant table */
}

/***************************************************************/
/* Process declarative-part */

void declarationGroup(int32_t beginLabel)
{
  int16_t notFirst   = 0;             /* Init count of nested procs */
  int16_t saveNSym   = nsym;          /* Save top of symbol table */
  int16_t saveNConst = nconst;        /* Save top of constant table */

  TRACE(lstFile,"[declarationGroup]");

  /* FORM: declarative-part = { declaration-group }
   * FORM: declaration-group =
   *       label-declaration-group | constant-definition-group |
   *       type-definition-group   | variable-declaration-group |
   *       function-declaration    | procedure-declaration
   */

  /* Process label-declaration-group.
   * FORM: label-declaration-group = 'label' label { ',' label } ';'
   */

  if (token == tLABEL) pas_DeclareLabel();

  /* Process constant-definition-group.
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
    }

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
    }

  /* Process variable-declaration-group
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
    }

  /* Process procedure/function-declaration(s) if present 
   * FORM: function-declaration =
   *       function-heading ';' directive |
   *       function-heading ';' function-block
   * FORM: procedure-declaration =
   *       procedure-heading ';' directive |
   *       procedure-heading ';' procedure-block
   *
   * NOTE:  a JMP to the executable body of this block is generated
   * if there are nested procedures and this is not level=0
   */

  for (;;)
    {
      /* FORM: function-heading =
       *       'function' identifier [ formal-parameter-list ] ':' result-type
       */

      if (token == tFUNCTION)
        {
          /* Check if we need to put a jump around the function */

          if ((beginLabel > 0) && !(notFirst) && (level > 0))
            {
              pas_GenerateDataOperation(opJMP, (int32_t)beginLabel);
            }

          /* Get the procedure-identifier */

          const_strt = saveNConst;    /* Limit search to present level */
          sym_strt   = saveNSym;
          getToken();                 /* Get identifier */
          const_strt = 0;
          sym_strt   = 0;

          /* Define the function */

          pas_FunctionDeclaration();
          notFirst++;                 /* No JMP next time */
        }

      /* FORM: procedure-heading =
       *       'procedure' identifier [ formal-parameter-list ]
       */

      else if (token == tPROCEDURE)
        {
          /* Check if we need to put a jump around the function */

          if ((beginLabel > 0) && !(notFirst) && (level > 0))
            {
              pas_GenerateDataOperation(opJMP, (int32_t)beginLabel);
            }

          /* Get the procedure-identifier */

          const_strt = saveNConst;    /* Limit search to present level */
          sym_strt   = saveNSym;
          getToken();                 /* Get identifier */
          const_strt = 0;
          sym_strt   = 0;

          /* Define the procedure */

          pas_ProcedureDeclaration();
          notFirst++;                 /* No JMP next time */
        }
      else break;
    }
}

/***************************************************************/

void constantDefinitionGroup(void)
{
  /* Process constant-definition-group.
   * FORM: constant-definition-group =
   *       'const' constant-definition ';' { constant-definition ';' }
   * FORM: constant-definition = identifier '=' constant
   *
   * On entry, token should point to the identifier of the first
   * constant-definition.
   */

  for (;;)
    {
      if (token == tIDENT)
        {
          pas_DeclareConst();
          if (token != ';') break;
          else getToken();
        }
      else break;
    }
}

/***************************************************************/

void typeDefinitionGroup(void)
{
  char   *typeName;

  /* Process type-definition-group
   * FORM: type-definition-group =
   *       'type' type-definition ';' { type-definition ';' }
   * FORM: type-definition = identifier '=' type-denoter
   *
   * On entry, token refers to the first identifier (if any) of
   * the type-definition list.
   */

  for (;;)
    {
      if (token == tIDENT)
        {
          /* Save the type identifier */

          typeName = tkn_strt;
          getToken();

          /* Verify that '=' follows the type identifier */

          if (token != '=') error (eEQ);
          else getToken();

          (void)pas_DeclareType(typeName);
          if (token != ';') break;
          else getToken();

        }
      else break;
    }
}

/***************************************************************/

void variableDeclarationGroup(void)
{
   /* Process variable-declaration-group
    * FORM: variable-declaration-group =
    *       'var' variable-declaration { ';' variable-declaration }
    * FORM: variable-declaration = identifier-list ':' type-denoter
    * FORM: identifier-list = identifier { ',' identifier }
    *
    * Only entry, token holds the first identfier (if any) of the
    * variable-declaration list.
    */

  for (;;)
    {
      if (token == tIDENT)
        {
          (void)pas_DeclareVar();
          if (token != ';') break;
          else getToken();
        }
      else if (token == sFILE)
        {
          pas_DeclareFile();
          if (token != ';') break;
          else getToken();
        }
      else break;
    }
}

/***************************************************************/
/* Process formal-parameter-list */

int16_t formalParameterList(STYPE *procPtr)
{
  int16_t parameterOffset;
  int16_t i;
  bool    pointerType;

  TRACE(lstFile,"[formalParameterList]");

  /* FORM: formal-parameter-list =
   *       '(' formal-parameter-section { ';' formal-parameter-section } ')'
   * FORM: formal-parameter-section =
   *       value-parameter-specification |
   *       variable-parameter-specification |
   *       procedure-parameter-specification |
   *       function-parameter-specification
   * FORM: value-parameter-specification =
   *       identifier-list ':' type-identifier
   * FORM: variable-parameter-specification =
   *       'var' identifier-list ':' type-identifier
   *
   * On entry token should refer to the '(' at the beginning of the
   * (optional) formal parameter list.
   */

  g_nParms = 0;

  /* Check if the formal-parameter-list is present.  It is optional in
   * all contexts in which this function is called.
   */

  if (token == '(')
    {
      /* Process each formal-parameter-section */

      do
        {
          getToken();

          /* Check for variable-parameter-specification */

          if (token == tVAR)
            {
              pointerType = 1;
              getToken();
            } 
          else pointerType = 0;

          /* Process the common part of the variable-parameter-specification
           * and the value-parameter specification.
           * NOTE that procedure-parameter-specification and
           * function-parameter-specification are not yet supported.
           */

          (void)pas_DeclareParameter(pointerType);

        }
      while (token == ';');

      /* Verify that the formal parameter list terminates with a
       * right parenthesis.
       */

      if (token != ')') error (eRPAREN);
      else getToken();

    }

  /* Save the number of parameters found in sPROC/sFUNC symbol table entry */

  procPtr->sParm.p.nParms = g_nParms;

  /* Now, calculate the parameter offsets from the size of each parameter */

  parameterOffset = -sRETURN_SIZE;
  for (i = g_nParms; i > 0; i--)
    {
      /* The offset to the next parameter is the offset to the previous
       * parameter minus the size of the new parameter (aligned to
       * multiples of size of INTEGER).
       */

      parameterOffset -= procPtr[i].sParm.v.size;
      parameterOffset  = intAlign(parameterOffset);
      procPtr[i].sParm.v.offset = parameterOffset;
    }

  return parameterOffset;
}

/***************************************************************
 * Private Functions
 ***************************************************************/
/* Process LABEL block */

static void pas_DeclareLabel(void)
{
   char   *labelname;                   /* Label symbol table name */

   TRACE(lstFile,"[pas_DeclareLabel]");

   /* FORM:  LABEL <integer>[,<integer>[,<integer>][...]]]; */

   do
     {
       getToken();
       if ((token == tINT_CONST) && (tknInt >= 0))
         {
           labelname = stringSP;
           (void)sprintf (labelname, "%ld", tknInt);
           while (*stringSP++);
           (void)addLabel(labelname, ++label);
           getToken();
         }
       else error(eINTCONST);
     }
   while (token == ',');

   if (token != ';') error (eSEMICOLON);
   else getToken();
}

/***************************************************************/
/* Process constant definition:
 * FORM: constant-definition = identifier '=' constant
 * FORM: constant = [ sign ] integer-number |
 *                  [ sign ] real-number |
 *                  [ sign ] constant-identifier |
 *                           character-literal |
 *                           string-literal
 */

static void pas_DeclareConst(void)
{
  char *const_name;

  TRACE(lstFile,"[pas_DeclareConst]");

  /* FORM:  <identifier> = <numeric constant|string>
   * NOTE:  Only integer constants are supported
   */

  /* Save the name of the constant */

  const_name = tkn_strt;

  /* Verify that the name is followed by '=' and get the
   * following constant value.
   */

  getToken();
  if (token != '=') error (eEQ);
  else getToken();

  /* Handle constant expressions */

  constantExpression();

  /* Add the constant to the symbol table based on the type of
   * the constant found following the '= [ sign ]'
   */

  switch (constantToken)
    {
    case tINT_CONST :
    case tCHAR_CONST :
    case tBOOLEAN_CONST :
    case sSCALAR_OBJECT :
      (void)addConstant(const_name, constantToken, &constantInt, NULL);
      break;

    case tREAL_CONST :
      (void)addConstant(const_name, constantToken, (int32_t*)&constantReal, NULL);
      break;

    case tSTRING_CONST :
      {
        uint32_t offset = poffAddRoDataString(poffHandle, constantStart);
        (void)addStringConst(const_name, offset, strlen(constantStart));
      }
      break;

    default :
      error(eINVCONST);
    }
}

/***************************************************************/
/* Process TYPE declaration */

static STYPE *pas_DeclareType(char *typeName)
{
  STYPE *typePtr;

  TRACE(lstFile,"[pas_DeclareType]");

  /* This function processes the type-denoter in
   * FORM: type-definition = identifier '=' type-denoter
   * FORM: array-type = 'array' '[' index-type-list ']' 'of' type-denoter
   */

  /* FORM: type-denoter = type-identifier | new-type
   * FORM: new-type = new-ordinal-type | new-complex-type
   */

  typePtr = pas_NewComplexType(typeName);
  if (typePtr == NULL)
    {
      /* Check for Simple Types */

      typePtr = pas_DeclareOrdinalType(typeName);
      if (typePtr == NULL)
        {
          error(eINVTYPE);
        }
    }

  return typePtr;
}

/***************************************************************/
/* Process a simple TYPE declaration */

static STYPE *pas_DeclareOrdinalType(char *typeName)
{
  STYPE *typePtr;
  STYPE *typeIdPtr;

  /* Declare a new ordinal type */

  typePtr = pas_NewOrdinalType(typeName);

  /* Otherwise, declare a type equivalent to a previously defined type
   * NOTE: the following logic is incomplete.  Its is only good for
   * sKind == sType
   */

  if (typePtr == NULL)
     {
       typeIdPtr = pas_TypeIdentifier(1);
       if (typeIdPtr)
         {
           typePtr = addTypeDefine(typeName, typeIdPtr->sParm.t.type,
                                    g_dwVarSize, typeIdPtr);
         }
     }

   return typePtr;
}

/***************************************************************/
/* Process VAR declaration */

static STYPE *pas_DeclareVar(void)
{
  STYPE *varPtr;
  STYPE *typePtr;
  char  *varName;

  TRACE(lstFile,"[pas_DeclareVar]");

  /* FORM: variable-declaration = identifier-list ':' type-denoter
   * FORM: identifier-list = identifier { ',' identifier }
   */

  typePtr  = NULL;

  /* Save the current identifier */

  varName = tkn_strt;
  getToken();

  /* A comma indicates that there is another indentifier int the
   * identifier-list
   */

  if (token == ',')
    {
      /* Yes ..Process the next identifer in the indentifier list
       * via recursion
       */

      getToken();
      if (token != tIDENT) error(eIDENT);
      else typePtr = pas_DeclareVar();
    }
  else
    {
      /* No.. verify that the identifer-list is followed by ';' */

      if (token != ':') error(eCOLON);
      else getToken();

      /* Process the type-denoter */

      typePtr = pas_TypeDenoter(varName, 1);
      if (typePtr == NULL)
        {
          error(eINVTYPE);
        }
    }

  if (typePtr)
    {
      uint8_t varType = typePtr->sParm.t.type;

      /* Determine if alignment to INTEGER boundaries is necessary */

      if ((!isIntAligned(dstack)) && (pas_IntAlignRequired(typePtr)))
        dstack = intAlign(dstack);

      /* Add the new variable to the symbol table */

      varPtr = addVariable(varName, varType, dstack, g_dwVarSize, typePtr);

      /* If the variable is declared in an interface section at level zero,
       * then it is a candidate to imported or exported.
       */

      if ((!level) && (FP->section == eIsInterfaceSection))
        {
          /* Are we importing or exporting the interface?
           *
           * PROGRAM EXPORTS:
           * If we are generating a program binary (i.e., FP0->kind ==
           * eIsProgram) then the variable memory allocation must appear
           * on the initial stack allocation; therefore the variable
           * stack offset myst be exported by the program binary.
           *
           * UNIT IMPORTS:
           * If we are generating a unit binary (i.e., FP0->kind ==
           * eIsUnit), then we are importing the level 0 stack offset
           * from the main program.
           */

          if (FP0->kind == eIsUnit)
            {
              /* Mark the symbol as external and replace the absolute
               * offset with this relative offset.
               */

              varPtr->sParm.v.flags  |= SVAR_EXTERNAL;
              varPtr->sParm.v.offset  = dstack - FP->dstack;

              /* IMPORT the symbol; assign an offset relative to
               * the dstack at the beginning of this file
               */
 
              pas_GenerateStackImport(varPtr);
            }
          else /* if (FP0->kind == eIsProgram) */
            {
              /* EXPORT the symbol */

              pas_GenerateStackExport(varPtr);
            }
        }

      /* In any event, bump the stack offset to include space for 
       * this new symbol.  The 'bumped' stack offset will be the
       * offset for the next variable that is declared.
       */

      dstack += g_dwVarSize;
    }

  return typePtr;
}

/***************************************************************/
/* Process VAR FILE OF declaration */

static void pas_DeclareFile(void)
{
   int16_t fileNumber = tknPtr->sParm.fileNumber;
   STYPE *filePtr;

   TRACE(lstFile,"[pas_DeclareFile]");

   /* FORM:  <file identifier> : FILE OF <type> */
   /* OR:    <file identifier> : <FILE OF type identifier> */
   if (!(fileNumber)) error(eINVFILE);
   else if (files [fileNumber].defined) error(eDUPFILE);
   else {

     /* Skip over the <file identifier> */
     getToken();

     /* Verify that a colon follows the <file identifier> */
     if (token != ':') error (eCOLON);
     else getToken();

     /* Make sure that the data stack is aligned to INTEGER boundaries */
     dstack = intAlign(dstack);

     /* FORM:  <file identifier> : FILE OF <type> */
     if (token == sFILE_OF) {

       files[fileNumber].defined = -1;
       files[fileNumber].flevel  = level;
       files[fileNumber].ftype   = tknPtr->sParm.t.type;
       files[fileNumber].faddr   = dstack;
       files[fileNumber].fsize   = tknPtr->sParm.t.asize;
       dstack += (tknPtr->sParm.t.asize);
       getToken();

     }

     /* FORM:  <file identifier> : <FILE OF type identifier> */
     else {
       if (token != tFILE) error (eFILE);
       else getToken();
       if (token != tOF) error (eOF);
       else getToken();

       filePtr = pas_TypeIdentifier(1);
       if (filePtr) {

         files[fileNumber].defined = -1;
         files[fileNumber].flevel  = level;
         files[fileNumber].ftype   = filePtr->sParm.t.type;
         files[fileNumber].faddr   = dstack;
         files[fileNumber].fsize   = g_dwVarSize;
         dstack += g_dwVarSize;

       }
     }
   }
}

/***************************************************************/
/* Process Procedure Declaration Block */

static void pas_ProcedureDeclaration(void)
{
   uint16_t procLabel = ++label;
   char    *saveStringSP;
   STYPE   *procPtr;
   register int i;

   TRACE(lstFile,"[pas_ProcedureDeclaration]");

   /* FORM: procedure-declaration =
    *       procedure-heading ';' directive |
    *       procedure-heading ';' procedure-block
    * FORM: procedure-heading =
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
     }

   /* Add the procedure to the symbol table */

   procPtr = addProcedure(tkn_strt, sPROC, procLabel, 0, NULL);

   /* Save the string stack pointer so that we can release all
    * formal parameter strings later.  Then get the next token.
    */

   saveStringSP = stringSP;
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

   /* If we are here then we know that we are either in a program file
    * or the 'implementation' part of a unit file (see punit.c -- At present,
    * the procedure declarations of the 'interface' section of a unit file
    * follow a different path).  In the latter case (only), we should export
    * every procedure declared at level zero.
    */

   if ((level == 1) && (FP->kind == eIsUnit))
     {
       /* EXPORT the procedure symbol. */

       pas_GenerateProcExport(procPtr);
     }

   /* Save debug information about the procedure */

   pas_GenerateDebugInfo(procPtr, 0);

   /* Process block */

   pas_GenerateDataOperation(opLABEL, (int32_t)procLabel);
   block();

   /* Destroy formal parameter names */

   for (i = 1; i <= procPtr->sParm.p.nParms; i++)
     {
       procPtr[i].sName = NULL;
     }

   stringSP = saveStringSP;

   /* Generate exit from procedure */

   pas_GenerateSimple(opRET);
   level--;

   /* Verify that END terminates with a semicolon */

   if (token !=  ';') error (eSEMICOLON);
   else getToken();
}

/***************************************************************/
/* Process Function Declaration Block */

static void pas_FunctionDeclaration(void)
{
   uint16_t funcLabel = ++label;
   int16_t parameterOffset;
   char    *saveStringSP;
   STYPE   *funcPtr;
   STYPE   *valPtr;
   STYPE   *typePtr;
   char    *funcName;
   register int i;

   TRACE(lstFile,"[pas_FunctionDeclaration]");

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
     }

   funcPtr = addProcedure(tkn_strt, sFUNC, funcLabel, 0, NULL);

   /* NOTE:  The level associated with the FUNCTION symbol is the level
    * At which the procedure was declared.  Everything declare within the
    * PROCEDURE is at the next level
    */

   level++;

   /* Save the string stack pointer so that we can release all
    * formal parameter strings later.  Then get the next token.
    */

   funcName = tkn_strt;
   saveStringSP = stringSP;
   getToken();

   /* Process parameter list */

   parameterOffset = formalParameterList(funcPtr);

   /* Verify that the parameter list is followed by a colon */

   if (token !=  ':') error (eCOLON);
   else getToken();

   /* Declare the function return value variable.  This variable has
    * the same name as the function itself.  We fill the variable
    * symbol descriptor with bogus information now (but we fix it
    * below).
    */

   valPtr  = addVariable(funcName, sINT, 0, sINT_SIZE, NULL);

   /* Get function type, return value type/size and offset to return value */

   typePtr = pas_TypeIdentifier(0);
   if (typePtr) {

     /* The offset to the return value is the offset to the last
      * parameter minus the size of the return value (aligned to
      * multiples of size of INTEGER).
      */

     parameterOffset        -= g_dwVarSize;
     parameterOffset         = intAlign(parameterOffset);

     /* Save the TYPE for the function return value local variable */

     valPtr->sKind           = typePtr->sParm.t.rtype;
     valPtr->sParm.v.offset  = parameterOffset;
     valPtr->sParm.v.size    = g_dwVarSize;
     valPtr->sParm.v.parent  = typePtr;

     /* Save the TYPE for the function */

     funcPtr->sParm.p.parent = typePtr;

     /* If we are here then we know that we are either in a program file
      * or the 'implementation' part of a unit file (see punit.c -- At present,
      * the function declarations of the 'interface' section of a unit file
      * follow a different path).  In the latter case (only), we should export
      * every function declared at level zero.
      */

     if ((level == 1) && (FP->kind == eIsUnit))
       {
         /* EXPORT the function symbol. */

         pas_GenerateProcExport(funcPtr);
       }
   }
   else
     error(eINVTYPE);

   /* Save debug information about the function */

   pas_GenerateDebugInfo(funcPtr, g_dwVarSize);

   /* Process block */

   if (token !=  ';') error (eSEMICOLON);
   else getToken();

   pas_GenerateDataOperation(opLABEL, (int32_t)funcLabel);
   block();

   /* Destroy formal parameter names and the function return value name */

   for (i = 1; i <= funcPtr->sParm.p.nParms; i++)
     {
       funcPtr[i].sName = ((char *) NULL);
     }

   valPtr->sName = ((char *) NULL);
   stringSP = saveStringSP;

   /* Generate exit from procedure/function */

   pas_GenerateSimple(opRET);
   level--;

   /* Verify that END terminates with a semicolon */

   if (token !=  ';') error (eSEMICOLON);
   else getToken();
}

/***************************************************************/
/* Determine the size value to use with this type */

static void pas_SetTypeSize(STYPE *typePtr, bool allocate)
{
  TRACE(lstFile,"[pas_SetTypeSize]");

  /* Check for type-identifier */

  g_dwVarSize = 0;

  if (typePtr != NULL)
    {
      /* If allocate is true, then we want to return the size of
       * the type that we would use if we are going to allocate
       * an instance on the stack.
       */

      if (allocate)
        {
          /* Could it be a storage size value (such as is used for
           * the enhanced pascal string type?).  In an weak attempt to
           * be compatible with everyone in the world, we will allow
           * either '[]' or '()' to delimit the size specification.
           */

          if (((token == '[') || (token == '(')) &&
              ((typePtr->sParm.t.flags & STYPE_VARSIZE) != 0))
            {
              uint16_t term_token;
              uint16_t errcode;

              /* Yes... we need to parse the size from the input stream.
               * First, determine which token will terminate the size
               * specification.
               */

              if (token == '(')
                {
                  term_token = ')';    /* Should end with ')' */
                  errcode = eRPAREN;   /* If not, this is the error */
                }
              else
                {
                  term_token = ']';    /* Should end with ']' */
                  errcode = eRBRACKET; /* If not, this is the error */
                }

              /* Now, parse the size specification */

              /* We expect the size to consist of a single integer constant.
               * We should support any constant integer expression, but this
               * has not yet been implemented.
               */

              getToken();
              if (token != tINT_CONST) error(eINTCONST);
              /* else if (tknInt <= 0) error(eINVCONST); see below */
              else if (tknInt <= 2) error(eINVCONST);
              else
                {
                  /* Use the value of the integer constant for the size
                   * the allocation.  NOTE:  There is a problem here in
                   * that for the sSTRING type, it wants the first 2 bytes
                   * for the string length.  This means that the actual
                   * length is real two less than the specified length.
                   */

                  g_dwVarSize = tknInt;
                }

              /* Verify that the correct token terminated the size
               * specification.  This could be either ')' or ']'
               */

              getToken();
              if (token != term_token) error(errcode);
              else getToken();
            }
          else
            {
              /* Return the fixed size of the allocated instance of
               * this type */

              g_dwVarSize = typePtr->sParm.t.asize;
            }
        }

      /* If allocate is false, then we want to return the size of
       * the type that we would use if we are going to refer to
       * a reference on the stack.  This is really non-standard
       * and is handle certain optimatizations where we cheat and
       * pass some types by reference rather than by value.  The
       * enhanced pascal string type is the only example at present.
       */

      else
        {
          /* Return the size to a clone, reference to an instance */

          g_dwVarSize = typePtr->sParm.t.rsize;
        }
    }
}

/***************************************************************/
/* Verify that the next token is a type identifer
 * NOTE:  This function modifies the global variable g_dwVarSize
 * as a side-effect
 */

static STYPE *pas_TypeIdentifier(bool allocate)
{
  STYPE *typePtr = NULL;

  TRACE(lstFile,"[pas_TypeIdentifier]");

  /* Check for type-identifier */

  if (token == sTYPE)
    {
      /* Return a reference to the type token. */

      typePtr = tknPtr;
      getToken();

      /* Return the size value associated with this type */

      pas_SetTypeSize(typePtr, allocate);
    }

  return typePtr;
}

/***************************************************************/

static STYPE *pas_TypeDenoter(char *typeName, bool allocate)
{
  STYPE *typePtr;

  TRACE(lstFile,"[pas_TypeDenoter]");

  /* FORM: type-denoter = type-identifier | new-type
   *
   * Check for type-identifier
   */

  typePtr = pas_TypeIdentifier(allocate);
  if (typePtr != NULL)
    {
      /* Return the type identifier */

      return typePtr;
    }

  /* Check for new-type
   * FORM: new-type = new-ordinal-type | new-complex-type
   */

  /* Check for new-complex-type */

  typePtr = pas_NewComplexType(typeName);
  if (typePtr == NULL)
    {
      /* Check for new-ordinal-type */

      typePtr = pas_NewOrdinalType(typeName);
    }

  /* Return the size value associated with this type */

  pas_SetTypeSize(typePtr, allocate);

  return typePtr;
}

/***************************************************************/
/* Declare is new ordinal type */

static STYPE *pas_NewOrdinalType(char *typeName)
{
  STYPE *typePtr = NULL;

  /* Declare a new-ordinal-type
   * FORM: new-ordinal-type = enumerated-type | subrange-type
   */

  /* FORM: enumerated-type = '(' enumerated-constant-list ')' */

   if (token == '(')
     {
       int32_t nObjects;
       nObjects = 0;
       typePtr = addTypeDefine(typeName, sSCALAR, sINT_SIZE, NULL);

       /* Now declare each instance of the scalar */

       do {
         getToken();
         if (token != tIDENT) error(eIDENT);
         else
           {
             (void)addConstant(tkn_strt, sSCALAR_OBJECT, &nObjects, typePtr);
             nObjects++;
             getToken();
           }
       } while (token == ',');

       /* Save the number of objects associated with the scalar type (the
        * maximum ORD is nObjects - 1). */

       typePtr->sParm.t.maxValue = nObjects - 1;

       if (token != ')') error(eRPAREN);
       else getToken();

     }

   /* Declare a new subrange type
    * FORM: subrange-type = constant '..' constant
    * FORM: constant =
    *    [ sign ] integer-number |  [ sign ] real-number |
    *    [ sign ] constant-identifier | character-literal | string-literal
    *
    * Case 1: <constant> is INTEGER
    */

   else if (token == tINT_CONST)
     {
       /* Create the new INTEGER subrange type */

       typePtr = addTypeDefine(typeName, sSUBRANGE, sINT_SIZE, NULL);
       typePtr->sParm.t.subType  = sINT;
       typePtr->sParm.t.minValue = tknInt;
       typePtr->sParm.t.maxValue = MAXINT;

       /* Verify that ".." separates the two constants */

       getToken();
       if (token != tSUBRANGE) error(eSUBRANGE);
       else getToken();

       /* Verify that the ".." is following by an INTEGER constant */

       if ((token != tINT_CONST) || (tknInt < typePtr->sParm.t.minValue))
         error(eSUBRANGETYPE);
       else
         {
           typePtr->sParm.t.maxValue = tknInt;
           getToken();
         }
     }

   /* Case 2: <constant> is CHAR */

   else if (token == tCHAR_CONST)
     {
       /* Create the new CHAR subrange type */

       typePtr = addTypeDefine(typeName, sSUBRANGE, sCHAR_SIZE, NULL);
       typePtr->sParm.t.subType  = sCHAR;
       typePtr->sParm.t.minValue = tknInt;
       typePtr->sParm.t.maxValue = MAXCHAR;

       /* Verify that ".." separates the two constants */

       getToken();
       if (token != tSUBRANGE) error(eSUBRANGE);
       else getToken();

       /* Verify that the ".." is following by a CHAR constant */

       if ((token != tCHAR_CONST) || (tknInt < typePtr->sParm.t.minValue))
         error(eSUBRANGETYPE);
       else
         {
           typePtr->sParm.t.maxValue = tknInt;
           getToken();
         }
     }

   /* Case 3: <constant> is a SCALAR type */

   else if (token == sSCALAR_OBJECT)
     {
       /* Create the new SCALAR subrange type */

       typePtr = addTypeDefine(typeName, sSUBRANGE, sINT_SIZE, tknPtr);
       typePtr->sParm.t.subType  = token;
       typePtr->sParm.t.minValue = tknInt;
       typePtr->sParm.t.maxValue = MAXINT;

       /* Verify that ".." separates the two constants */

       getToken();
       if (token != tSUBRANGE) error(eSUBRANGE);
       else getToken();

       /* Verify that the ".." is following by a SCALAR constant of the same
        * type as the one which preceded it
        */

       if ((token != sSCALAR_OBJECT) ||
           (tknPtr != typePtr->sParm.t.parent) ||
           (tknPtr->sParm.c.val.i < typePtr->sParm.t.minValue))
         error(eSUBRANGETYPE);
       else
         {
           typePtr->sParm.t.maxValue = tknPtr->sParm.c.val.i;
           getToken();
         }
     }

   return typePtr;
}

/***************************************************************/

static STYPE *pas_NewComplexType(char *typeName)
{
  STYPE *typePtr = NULL;
  STYPE *typeIdPtr;

  TRACE(lstFile,"[pas_TypeDenoter]");

  /* FORM: new-complex-type = new-structured-type | new-pointer-type */

  switch (token)
    {
      /* FORM: new-pointer-type = '^' domain-type | '@' domain-type */

    case '^'      :
      getToken();
      typeIdPtr = pas_TypeIdentifier(1);
      if (typeIdPtr)
        {
          typePtr = addTypeDefine(typeName, sPOINTER, g_dwVarSize, typeIdPtr);
        }
      else
        {
          error(eINVTYPE);
        }
      break;

      /* FORM: new-structured-type =
       *     [ 'packed' ] array-type | [ 'packed' ] record-type |
       *     [ 'packed' ] set-type   | [ 'packed' ] file-type |
       *     [ 'packed' ] list-type  | object-type | string-type
       */

      /* PACKED Types */

    case tPACKED :
      error (eNOTYET);
      getToken();
      if (token != tARRAY) break;
      /* Fall through to process PACKED ARRAY type */

      /* Array Types
       * FORM: array-type = 'array' [ index-type-list ']' 'of' type-denoter
       */

    case tARRAY :
      getToken();
      typeIdPtr = pas_GetArrayType();
      if (typeIdPtr)
        {
          typePtr = addTypeDefine(typeName, sARRAY, g_dwVarSize, typeIdPtr);
        }
      else
        {
          error(eINVTYPE);
        }
      break;

      /* RECORD Types
       * FORM: record-type = 'record' field-list 'end'
       */

    case tRECORD :
      getToken();
      typePtr = pas_DeclareRecord(typeName);
      break;

      /* Set Types
       *
       * FORM: set-type = 'set' 'of' ordinal-type
       */

    case tSET :

      /* Verify that 'set' is followed by 'of' */

      getToken();
      if (token != tOF) error (eOF);
      else getToken();

      /* Verify that 'set of' is followed by an ordinal-type
       * If not, then declare a new one with no name
       */

      typeIdPtr = pas_OrdinalTypeIdentifier(1);
      if (typeIdPtr)
        getToken();
      else
        typeIdPtr = pas_DeclareOrdinalType(NULL);

      /* Verify that the ordinal-type is either a scalar or a 
       * subrange type.  These are the only valid types for 'set of'
       */

      if ((typeIdPtr) &&
          ((typeIdPtr->sParm.t.type == sSCALAR) ||
           (typeIdPtr->sParm.t.type == sSUBRANGE)))
        {
          /* Declare the SET type */

          typePtr = addTypeDefine(typeName, sSET_OF,
                                  typeIdPtr->sParm.t.asize, typeIdPtr);

          if (typePtr)
            {
              int16_t nObjects;

              /* Copy the scalar/subrange characteristics for convenience */

              typePtr->sParm.t.subType  = typeIdPtr->sParm.t.type;
              typePtr->sParm.t.minValue = typeIdPtr->sParm.t.minValue;
              typePtr->sParm.t.maxValue = typeIdPtr->sParm.t.minValue;

              /* Verify that the number of objects associated with the
               * scalar or subrange type will fit into an integer
               * representation of a set as a bit-string.
               */

              nObjects = typeIdPtr->sParm.t.maxValue
                - typeIdPtr->sParm.t.minValue + 1;
              if (nObjects > BITS_IN_INTEGER)
                {
                  error(eSETRANGE);
                  typePtr->sParm.t.maxValue = typePtr->sParm.t.minValue
                    + BITS_IN_INTEGER - 1;
                }
            }
        }
      else
        error(eSET);
      break;
 
      /* File Types
       * FORM: file-type = 'file' 'of' type-denoter
       */

      /* FORM: file-type = 'file' 'of' type-denoter */

    case tFILE :

      /* Make sure that 'file' is followed by 'of' */

      getToken();
      if (token != tOF) error (eOF);
      else getToken();

      /* Get the type-denoter */

      typeIdPtr = pas_TypeDenoter(NULL,1);
      if (typeIdPtr)
        {
          typePtr = addTypeDefine(typeName, sFILE_OF, g_dwVarSize, typeIdPtr);
          if (typePtr)
            {
              typePtr->sParm.t.subType = typeIdPtr->sParm.t.type;
            }
        }
      else
        {
          error(eINVTYPE);
        }
      break;

      /* FORM: string-type = pascal-string-type | c-string-type
       * FORM: pascal-string-type = 'string' [ max-string-length ]
       */
    case sSTRING :
      error (eNOTYET);
      getToken();
      break;

      /* FORM: list-type = 'list' 'of' type-denoter */
      /* FORM: object-type = 'object' | 'class' */
    default :
      break;

   }

  return typePtr;
}

/***************************************************************/
/* Verify that the next token is a type identifer
 */

static STYPE *pas_OrdinalTypeIdentifier(bool allocate)
{
  STYPE *typePtr;

  TRACE(lstFile,"[pas_OrdinalTypeIdentifier]");

  /* Get the next type from the input stream */

  typePtr = pas_TypeIdentifier(allocate);

  /* Was a type encountered? */

  if (typePtr != NULL)
    {
      switch (typePtr->sParm.t.type)
        {
          /* Check for an ordinal type (verify this list!) */

        case sINT :
        case sBOOLEAN :
        case sCHAR :
        case sSCALAR :
        case sSUBRANGE:
          /* If it is an ordinal type, then just return the
           * type pointer.
           */

          break;
        default :
          /* If not, return NULL */

          typePtr = NULL;
          break;
        }
    }
  return typePtr;
}

/***************************************************************/
/* get array type argument for TYPE block or variable declaration */

static STYPE *pas_GetArrayType(void)
{
   STYPE *typePtr = NULL;

   TRACE(lstFile,"[pas_GetArrayType]");

   /* FORM: array-type = 'array' '[' index-type-list ']' 'of' type-denoter */
   /* FORM: [PACKED] ARRAY [<integer>] OF type-denoter
    * NOTE: Bracketed value is the array size!  NONSTANDARD! */

   g_dwVarSize = 0;

   /* Verify that the index-type-list is preceded by '[' */

   if (token != '[') error (eLBRACKET);
   else
     {
       /* FORM: index-type-list = index-type { ',' index-type }
        * FORM: index-type = ordinal-type
        */

       getToken();
       if (token != tINT_CONST) error (eINTCONST);
       else
         {
           g_dwVarSize = tknInt;
           getToken();

           /* Verify that the index-type-list is followed by ']' */

           if (token != ']') error (eRBRACKET);
           else getToken();

           /* Verify that 'of' precedes the type-denoter */

           if (token != tOF) error (eOF);
           else getToken();

           /* We have the array size in elements, not get the type and convert
            * the size for the type found
            */

           typePtr = pas_DeclareType(NULL);
           if (typePtr)
             {
               g_dwVarSize *= typePtr->sParm.t.asize;
             }
         }
     }

   return typePtr;
}

/***************************************************************/

static STYPE *pas_DeclareRecord(char *recordName)
{
  STYPE *recordPtr;
  int16_t recordOffset;
  int recordCount, symbolIndex;

  TRACE(lstFile,"[pas_DeclareRecord]");

  /* FORM: record-type = 'record' field-list 'end' */

  /* Declare the new RECORD type */

  recordPtr = addTypeDefine(recordName, sRECORD, 0, NULL);

  /* Then declare the field-list associated with the RECORD
   * FORM: field-list =
   *       [
   *         fixed-part [ ';' ] variant-part [ ';' ] |
   *         fixed-part [ ';' ] |
   *         variant-part [ ';' ] |
   *       ]
   *
   * Process the fixed-part first.
   * FORM: fixed-part = record-section { ';' record-section }
   * FORM: record-section = identifier-list ':' type-denoter
   * FORM: identifier-list = identifier { ',' identifier }
   */

  for (;;)
    {
      /* Terminate parsing of the fixed-part when we encounter
       * 'case' indicating the beginning of the variant part of
       * the record.  If there is no fixed-part, then 'case' will
       * appear immediately.
       */

      if (token == tCASE) break;

      /* We now expect to see and indentifier representating the
       * beginning of the next fixed field.
       */

      (void)pas_DeclareField(recordPtr);

      /* If the field declaration terminates with a semicolon, then
       * we expect to see another <fixed part> declaration in the
       * record.
       */

      if (token == ';')
        {
          /* Skip over the semicolon and process the next fixed
           * field declaration.
           */

          getToken();

          /* We will treat this semi colon as optional.  If we
           * hit 'end' or 'case' after the semicolon, then we
           * will terminate the fixed part with no complaint.
           */

          if ((token == tEND) || (token == tCASE))
            break;
        }

      /* If there is no semicolon after the field declaration,
       * then 'end' or 'case' is expected.  This will be verified
       * below.
       */

      else break;
    }

  /* Get the total size of the RECORD type and the offset of each
   * field within the RECORD.
   */

  for (recordOffset = 0, symbolIndex = 1, recordCount = 0;
       recordCount < recordPtr->sParm.t.maxValue;
       symbolIndex++)
    {
      /* We know that 'maxValue' sRECORD_OBJECT symbols follow the sRECORD
       * type declaration.  However, these may not be sequential due to the
       * possible declaration of sTYPEs associated with each field.
       */

      if (recordPtr[symbolIndex].sKind == sRECORD_OBJECT)
        {
          /* Align the recordOffset (if necessary) */

          if ((!isIntAligned(recordOffset)) &&
              (pas_IntAlignRequired(recordPtr[symbolIndex].sParm.r.parent)))
            recordOffset = intAlign(recordOffset);

          /* Save the offset associated with this field, and determine the
           * offset to the next field (if there is one)
           */

          recordPtr[symbolIndex].sParm.r.offset = recordOffset;
          recordOffset += recordPtr[symbolIndex].sParm.r.size;
          recordCount++;
        }
    } 

  /* Update the RECORD entry for the total size of all fields */

  recordPtr->sParm.t.asize = recordOffset;

  /* Now we are ready to process the variant-part.
   * FORM: variant-part = 'case' variant-selector 'of' variant-body
   */

  if (token == tCASE)
    {
      int16_t variantOffset;
      uint16_t maxRecordSize;

      /* Skip over the 'case' */

      getToken();

      /* Check for variant-selector
       * FORM: variant-selector = [ identifier ':' ] ordinal-type-identifer
       */

      if (token != tIDENT) error(eRECORDDECLARE);

      /* Add a variant-selector to the fixed-part of the record */ 

      else
        {
          STYPE *typePtr;
          char  *fieldName;

          /* Save the field name */

          fieldName = tkn_strt;
          getToken();

          /* Verify that the identifier is followed by a colon */

          if (token != ':') error(eCOLON);
          else getToken();

          /* Get the ordinal-type-identifier */

          typePtr = pas_OrdinalTypeIdentifier(1);
          if (!typePtr) error(eINVTYPE);
          else
            {
              STYPE *fieldPtr;

              /* Declare a <field> with this <identifier> as its name */

              fieldPtr = addField(fieldName, recordPtr);

              /* Increment the number of fields in the record */

              recordPtr->sParm.t.maxValue++;

              /* Copy the size of field from the sTYPE entry into the
               * <field> type entry.  NOTE:  This element is not essential
               * since it  can be obtained from the parent type pointer
               */

              fieldPtr->sParm.r.size = typePtr->sParm.t.asize;

              /* Save a pointer back to the parent field type */

              fieldPtr->sParm.r.parent = typePtr;

              /* Align the recordOffset (if necessary) */

              if ((!isIntAligned(recordOffset)) &&
                  (pas_IntAlignRequired(typePtr)))
                recordOffset = intAlign(recordOffset);

              /* Save the offset associated with this field, and determine
               * the offset to the next field (if there is one)
               */

              fieldPtr->sParm.r.offset = recordOffset;
              recordOffset += recordPtr[symbolIndex].sParm.r.size;
            }
        }

      /* Save the offset to the start of the variant portion of the RECORD */

      variantOffset = recordOffset;
      maxRecordSize = recordOffset;

      /* Skip over the 'of' following the variant selector */

      if (token != tOF) error(eOF);
      else getToken();

      /* Loop to process the variant-body
       * FORM: variant-body =
       *       variant-list [ [ ';' ] variant-part-completer ] |
       *       variant-part-completer
       * FORM: variant-list = variant { ';' variant }
       * FORM: variant-part-completer = ( 'otherwise' | 'else' ) ( field-list )
       */

      for (;;)
        {
          /* Now process each variant where:
           * FORM: variant = case-constant-list ':' '(' field-list ')'
           * FORM: case-constant-list = case-specifier { ',' case-specifier }
           * FORM: case-specifier = case-constant [ '..' case-constant ]
           */

          /* Verify that the case selector begins with a case-constant.
           * Note that subrange case-specifiers are not yet supported.
           */

          if (!isConstant(token))
            {
              error(eINVCONST);
              break;
            }

          /* Just consume the <case selector> for now -- Really need to
           * verify that each constant is of the same type as the type
           * identifier (or the type associated with the tag) in the CASE
           */

          do
            {
              getToken();
              if (token == ',') getToken();
            }
          while (isConstant(token));

          /* Make sure a colon separates case-constant-list from the
           * field-list
           */

          if (token == ':') getToken();
          else error(eCOLON);

          /* The field-list must be enclosed in parentheses */

          if (token == '(') getToken();
          else error(eLPAREN);

          /* Special case the empty variant <field list> */

          if (token != ')')
            {
              /* Now process the <field list> for the variant.  This works
               * just like the field list of the fixed part, except the
               * offset is reset for each variant.
               * FORM: field-list =
               *       [
               *         fixed-part [ ';' ] variant-part [ ';' ] |
               *         fixed-part [ ';' ] |
               *         variant-part [ ';' ] |
               *       ]
               */

              for (;;)
                {
                  /* We now expect to see and indentifier representating the
                   * beginning of the next variablefield.
                   */

                  (void)pas_DeclareField(recordPtr);

                  /* If the field declaration terminates with a semicolon,
                   * then we expect to see another <variable part>
                   * declaration in the record.
                   */

                  if (token == ';')
                    {
                      /* Skip over the semicolon and process the next
                       * variable field declaration.
                       */

                      getToken();

                      /* We will treat this semi colon as optional.  If we
                       * hit 'end' after the semicolon, then we will
                       * terminate the fixed part with no complaint.
                       */

                      if (token == tEND)
                        break;
                    }
                  else break;
                }

              /* Get the total size of the RECORD type and the offset of each
               * field within the RECORD.
               */

              for (recordOffset = variantOffset;
                   recordCount < recordPtr->sParm.t.maxValue;
                   symbolIndex++)
                {
                  /* We know that 'maxValue' sRECORD_OBJECT symbols follow
                   * the sRECORD type declaration.  However, these may not
                   * be sequential due to the possible declaration of sTYPEs
                   * associated with each field.
                   */

                  if (recordPtr[symbolIndex].sKind == sRECORD_OBJECT)
                    {
                      /* Align the recordOffset (if necessary) */

                      if ((!isIntAligned(recordOffset)) &&
                          (pas_IntAlignRequired(recordPtr[symbolIndex].sParm.r.parent)))
                        recordOffset = intAlign(recordOffset);

                      /* Save the offset associated with this field, and
                       * determine the offset to the next field (if there
                       * is one)
                       */

                      recordPtr[symbolIndex].sParm.r.offset = recordOffset;
                      recordOffset += recordPtr[symbolIndex].sParm.r.size;
                      recordCount++;
                    }
                }

              /* Check if this is the largest variant that we have found
               * so far
               */

              if (recordOffset > maxRecordSize)
                maxRecordSize = recordOffset;
            } 

          /* Verify that the <field list> is enclosed in parentheses */

          if (token == ')') getToken();
          else error(eRPAREN);

          /* A semicolon at this position means that another <variant>
           * follows.  Keep looping until all of the variants have been
           * processed (i.e., no semi-colon)
           */

          if (token == ';') getToken();
          else break;
        }

      /* Update the RECORD entry for the maximum size of all variants */

      recordPtr->sParm.t.asize = maxRecordSize;
    }

  /* Verify that the RECORD declaration terminates with END */

  if (token != tEND) error(eRECORDDECLARE);
  else getToken();

  return recordPtr;
}

/***************************************************************/

static STYPE *pas_DeclareField(STYPE *recordPtr)
{
   STYPE *fieldPtr = NULL;
   STYPE *typePtr;

   TRACE(lstFile,"[pas_DeclareField]");

   /* Declare one record-section with a record.
    * FORM: record-section = identifier-list ':' type-denoter
    * FORM: identifier-list = identifier { ',' identifier }
    */

   if (token != tIDENT) error(eIDENT);
   else {

     /* Declare a <field> with this <identifier> as its name */

     fieldPtr = addField(tkn_strt, recordPtr);
     getToken();

     /* Check for multiple fields of this <type> */

     if (token == ',') {

       getToken();
       typePtr = pas_DeclareField(recordPtr);

     }
     else {

       if (token != ':') error(eCOLON);
       else getToken();

       /* Use the existing type or declare a new type with no name */

       typePtr = pas_TypeDenoter(NULL, 1);
     }

     recordPtr->sParm.t.maxValue++;
     if (typePtr) {

       /* Copy the size of field from the sTYPE entry into the <field> */
       /* type entry.  NOTE:  This element is not essential since it */
       /* can be obtained from the parent type pointer */

       fieldPtr->sParm.r.size     = typePtr->sParm.t.asize;

       /* Save a pointer back to the parent field type */

       fieldPtr->sParm.r.parent   = typePtr;

     }
   }

   return typePtr;
}
 
/***************************************************************/
/* Process VAR/value Parameter Declaration */
/* NOTE:  This function increments the global variable g_nParms */
/* as a side-effect */

static STYPE *pas_DeclareParameter(bool pointerType)
{
   int16_t varType = 0;
   STYPE  *varPtr;
   STYPE  *typePtr;

   TRACE(lstFile,"[pas_DeclareParameter]");

   /* FORM:
    * <identifier>[,<identifier>[,<identifier>[...]]] : <type identifier>
    */

   if (token != tIDENT) error (eIDENT);
   else
     {
       varPtr = addVariable(tkn_strt, sINT, 0, sINT_SIZE, NULL);
       getToken();

       if (token == ',')
         {
           getToken();
           typePtr = pas_DeclareParameter(pointerType);
         }
       else
         {
           if (token != ':') error (eCOLON);
           else getToken();
           typePtr = pas_TypeIdentifier(0);
         }

       if (pointerType)
         {
           varType = sVAR_PARM;
           g_dwVarSize = sPTR_SIZE;
         }
       else
         {
           varType = typePtr->sParm.t.rtype;
         }

       g_nParms++;
       varPtr->sKind          = varType;
       varPtr->sParm.v.size   = g_dwVarSize;
       varPtr->sParm.v.parent = typePtr;
     }

   return typePtr;
}

/***************************************************************/

static bool pas_IntAlignRequired(STYPE *typePtr)
{
  bool returnValue = false;

  /* Type CHAR and ARRAYS of CHAR do not require alignment (unless
   * they are passed as value parameters).  Otherwise, alignment
   * to type INTEGER boundaries is required.
   */

  if (typePtr)
    {
      if (typePtr->sKind == sCHAR)
        {
          returnValue = true;
        }
      else if (typePtr->sKind == sARRAY)
        {
          typePtr = typePtr->sParm.t.parent;
          if ((typePtr) && (typePtr->sKind == sCHAR))
            {
              returnValue = true;
            }
        }
    }

  return returnValue;
}

/***************************************************************/
