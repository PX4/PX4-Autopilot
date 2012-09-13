/***************************************************************
 * ptbl.c
 * Table Management Package
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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "pedefs.h"

#include "pas.h"
#include "ptbl.h"
#include "perr.h"

/***************************************************************
 * Private Function Prototypes
 ***************************************************************/

static STYPE *addSymbol(char *name, int16_t type);

/***************************************************************
 * Public Variables
 ***************************************************************/

STYPE *parentInteger = NULL;
STYPE *parentString  = NULL;

/***************************************************************
 * Private Variables
 ***************************************************************/
/* NOTES in the following:
 * (1) Standard Pascal reserved word
 * (2) Standard Pascal Function
 * (3) Standard Pascal Procedure
 * (4) Extended (or non-standard) Pascal reserved word
 * (5) Extended (or non-standard) Pascal function
 * (6) Extended (or non-standard) Pascal procedure
 */

static const RTYPE rsw[] =                        /* Reserved word list */
{
  {"ABS",            tFUNC,           txABS},     /* (2) */
  {"AND",            tAND,            txNONE},    /* (1) */
  {"ARCTAN",         tFUNC,           txARCTAN},  /* (2) */
  {"ARRAY",          tARRAY,          txNONE},    /* (1) */
  {"BEGIN",          tBEGIN,          txNONE},    /* (1) */
  {"CASE",           tCASE,           txNONE},    /* (1) */
  {"CHR",            tFUNC,           txCHR},     /* (2) */
  {"CONST",          tCONST,          txNONE},    /* (1) */
  {"COS",            tFUNC,           txCOS},     /* (2) */
  {"DIV",            tDIV,            txNONE},    /* (1) */
  {"DO",             tDO,             txNONE},    /* (1) */
  {"DOWNTO",         tDOWNTO,         txNONE},    /* (1) */
  {"ELSE",           tELSE,           txNONE},    /* (1) */
  {"END",            tEND,            txNONE},    /* (1) */
  {"EOF",            tFUNC,           txEOF},     /* (2) */
  {"EOLN",           tFUNC,           txEOLN},    /* (2) */
  {"EXP",            tFUNC,           txEXP},     /* (2) */
  {"FILE",           tFILE,           txNONE},    /* (1) */
  {"FOR",            tFOR,            txNONE},    /* (1) */
  {"FUNCTION",       tFUNCTION,       txNONE},    /* (1) */
  {"GET",            tPROC,           txGET},     /* (3) */
  {"GETENV",         tFUNC,           txGETENV},  /* (5) */
  {"GOTO",           tGOTO,           txNONE},    /* (1) */
  {"IF",             tIF,             txNONE},    /* (1) */
  {"IMPLEMENTATION", tIMPLEMENTATION, txNONE},    /* (4) */
  {"IN",             tIN,             txNONE},    /* (1) */
  {"INTERFACE",      tINTERFACE,      txNONE},    /* (4) */
  {"LABEL",          tLABEL,          txNONE},    /* (1) */
  {"LN",             tFUNC,           txLN},      /* (2) */
  {"MOD",            tMOD,            txNONE},    /* (1) */
  {"NEW",            tPROC,           txNEW},     /* (3) */
  {"NOT",            tNOT,            txNONE},    /* (1) */
  {"ODD",            tFUNC,           txODD},     /* (2) */
  {"OF",             tOF,             txNONE},    /* (1) */
  {"OR",             tOR,             txNONE},    /* (1) */
  {"ORD",            tFUNC,           txORD},     /* (2) */
  {"PACK",           tPROC,           txPACK},    /* (3) */
  {"PACKED",         tPACKED,         txNONE},    /* (1) */
  {"PAGE",           tPROC,           txPAGE},    /* (3) */
  {"PRED",           tFUNC,           txPRED},    /* (2) */
  {"PROCEDURE",      tPROCEDURE,      txNONE},    /* (1) */
  {"PROGRAM",        tPROGRAM,        txNONE},    /* (1) */
  {"PUT",            tPROC,           txPUT},     /* (3) */
  {"READ",           tPROC,           txREAD},    /* (3) */
  {"READLN",         tPROC,           txREADLN},  /* (3) */
  {"RECORD",         tRECORD,         txNONE},    /* (1) */
  {"REPEAT",         tREPEAT,         txNONE},    /* (1) */
  {"RESET",          tPROC,           txRESET},   /* (3) */
  {"REWRITE",        tPROC,           txREWRITE}, /* (3) */
  {"ROUND",          tFUNC,           txROUND},   /* (2) */
  {"SET",            tSET,            txNONE},    /* (1) */
  {"SHL",            tSHL,            txNONE},    /* (4) */
  {"SHR",            tSHR,            txNONE},    /* (4) */
  {"SIN",            tFUNC,           txSIN},     /* (2) */
  {"SQR",            tFUNC,           txSQR},     /* (2) */
  {"SQRT",           tFUNC,           txSQRT},    /* (2) */
  {"SUCC",           tFUNC,           txSUCC},    /* (2) */
  {"THEN",           tTHEN,           txNONE},    /* (1) */
  {"TO",             tTO,             txNONE},    /* (1) */
  {"TRUNC",          tFUNC,           txTRUNC},   /* (2) */
  {"TYPE",           tTYPE,           txNONE},    /* (1) */
  {"UNIT",           tUNIT,           txNONE},    /* (4) */
  {"UNPACK",         tPROC,           txUNPACK},  /* (3) */
  {"UNTIL",          tUNTIL,          txNONE},    /* (1) */
  {"USES",           tUSES,           txNONE},    /* (4) */
  {"VAL",            tPROC,           txVAL},     /* (6) */
  {"VAR",            tVAR,            txNONE},    /* (1) */
  {"WHILE",          tWHILE,          txNONE},    /* (1) */
  {"WITH",           tWITH,           txNONE},    /* (1) */
  {"WRITE",          tPROC,           txWRITE},   /* (3) */
  {"WRITELN",        tPROC,           txWRITELN}, /* (3) */
  {NULL,             0,               txNONE}     /* List terminator */
};

static STYPE *symbolTable;             /* Symbol Table */

/**************************************************************/

const RTYPE *findReservedWord (char *name)
{
  register const RTYPE *ptr;           /* Point into reserved word list */
  register int16_t cmp;                 /* 0=equal; >0=past it */

  for (ptr = rsw; (ptr->rname); ptr++) /* Try each each reserved word */
    {
      cmp = strcmp(ptr->rname, name);  /* Check if names match */
      if (!cmp)                        /* Check if names match */
        return ptr;                    /* Return pointer to entry if match */
      else if (cmp > 0)                /* Exit early if we are past it */
        break;
    } /* end for */

  return (RTYPE*)NULL;                 /* return NULL pointer if no match */

} /* fnd findReservedWord */  

/***************************************************************/

STYPE *findSymbol (char *inName)
{
   register int16_t i;                /* loop index */

   for (i=nsym-1; i>=sym_strt; i--)
     if (symbolTable[i].sName)
       if (!strcmp(symbolTable[i].sName, inName))
         return &symbolTable[i];
   return (STYPE*)NULL;

} /* end findSymbol */

/***************************************************************/

static STYPE *addSymbol(char *name, int16_t type)
{
   TRACE(lstFile,"[addSymbol]");

   /* Check for Symbol Table overflow */
   if (nsym >= MAX_SYM) {

     fatal(eOVF);
     return (STYPE *)NULL;

   } /* end if */
   else {

     /* Clear all elements of the symbol table entry */
     memset(&symbolTable[nsym], 0, sizeof(STYPE));

     /* Set the elements which are independent of sKind */
     symbolTable[nsym].sName  = name;
     symbolTable[nsym].sKind  = type;
     symbolTable[nsym].sLevel = level;

     return &symbolTable[nsym++];

   } /* end else */

} /* end addSymbol */

/***************************************************************/

STYPE *addTypeDefine(char *name, uint8_t type, uint16_t size, STYPE *parent)
{
   STYPE *typePtr;

   TRACE(lstFile,"[addTypeDefine]");

   /* Get a slot in the symbol table */

   typePtr = addSymbol(name, sTYPE);
   if (typePtr)
     {
       /* Add the type definition to the symbol table
        * NOTES:
        * 1. The minValue and maxValue fields (for scalar and subrange)
        *    types must be set external to this function
        * 2. For most variables, allocated size/type (rsize/rtype) and
        *    the clone size/type are the same.  If this is not the case,
        *    external logic will need to clarify this as well.
        * 3. We assume that there are no special flags associated with
        *    the type.
        */

       typePtr->sParm.t.type     = type;
       typePtr->sParm.t.rtype    = type;
       typePtr->sParm.t.flags    = 0;
       typePtr->sParm.t.asize    = size;
       typePtr->sParm.t.rsize    = size;
       typePtr->sParm.t.parent   = parent;

     } /* end if */

   /* Return a pointer to the new constant symbol */

   return typePtr;

} /* end addTypeDefine */

/***************************************************************/

STYPE *addConstant(char *name, uint8_t type, int32_t *value, STYPE *parent)
{
   STYPE *constPtr;

   TRACE(lstFile,"[addConstant]");

   /* Get a slot in the symbol table */
   constPtr = addSymbol(name, type);
   if (constPtr) {

     /* Add the value of the constant to the symbol table */
     if (type == tREAL_CONST)
       constPtr->sParm.c.val.f = *((double*) value);
     else
       constPtr->sParm.c.val.i = *value;
     constPtr->sParm.c.parent = parent;

   } /* end if */

   /* Return a pointer to the new constant symbol */

   return constPtr;

} /* end addConstant */

/***************************************************************/

STYPE *addStringConst(char *name, uint32_t offset, uint32_t size)
{
  STYPE *stringPtr;

  TRACE(lstFile,"[addStringConst]");

  /* Get a slot in the symbol table */

  stringPtr = addSymbol(name, sSTRING_CONST);
  if (stringPtr)
    {
      /* Add the value of the constant to the symbol table */

      stringPtr->sParm.s.offset = offset;
      stringPtr->sParm.s.size   = size;
    } /* end if */

  /* Return a pointer to the new string symbol */

  return stringPtr;

} /* end addString */

/***************************************************************/

STYPE *addFile(char *name, uint16_t fileNumber)
{
   STYPE *filePtr;

   TRACE(lstFile,"[addFile]");

   /* Get a slot in the symbol table */
   filePtr = addSymbol(name, sFILE);
   if (filePtr) {

     /* Add the fileNumber to the symbol table */
     filePtr->sParm.fileNumber = fileNumber;

   } /* end if */

   /* Return a pointer to the new file symbol */

   return filePtr;

} /* end addFile */

/***************************************************************/

STYPE *addProcedure(char *name, uint8_t type, uint16_t label,
                    uint16_t nParms, STYPE *parent)
{
   STYPE *procPtr;

   TRACE(lstFile,"[addProcedure]");

   /* Get a slot in the symbol table */
   procPtr = addSymbol(name, type);
   if (procPtr)
     {
       /* Add the procedure/function definition to the symbol table */

       procPtr->sParm.p.label    = label;
       procPtr->sParm.p.nParms   = nParms;
       procPtr->sParm.p.flags    = 0;
       procPtr->sParm.p.symIndex = 0;
       procPtr->sParm.p.parent   = parent;
     } /* end if */

   /* Return a pointer to the new procedure/function symbol */

   return procPtr;

} /* end addProcedure */

/***************************************************************/

STYPE *addVariable(char *name, uint8_t type, uint16_t offset,
                   uint16_t size, STYPE *parent)
{
  STYPE *varPtr;

  TRACE(lstFile,"[addVariable]");

  /* Get a slot in the symbol table */

  varPtr = addSymbol(name, type);
  if (varPtr)
    {
      /* Add the variable to the symbol table */

      varPtr->sParm.v.offset   = offset;
      varPtr->sParm.v.size     = size;
      varPtr->sParm.v.flags    = 0;
      varPtr->sParm.v.symIndex = 0;
      varPtr->sParm.v.parent   = parent;
    } /* end if */

  /* Return a pointer to the new variable symbol */

  return varPtr;

} /* end addFile */

/***************************************************************/

STYPE *addLabel(char *name, uint16_t label)
{
  STYPE *labelPtr;

  TRACE(lstFile,"[addLabel]");

  /* Get a slot in the symbol table */

  labelPtr = addSymbol(name, sLABEL);
  if (labelPtr)
    {
      /* Add the label to the symbol table */

      labelPtr->sParm.l.label = label;
      labelPtr->sParm.l.unDefined = true;
    } /* end if */

  /* Return a pointer to the new label symbol */

  return labelPtr;

} /* end addFile */

/***************************************************************/

STYPE *addField(char *name, STYPE *record)
{
   STYPE *fieldPtr;

   TRACE(lstFile,"[addField]");

   /* Get a slot in the symbol table */
   fieldPtr = addSymbol(name, sRECORD_OBJECT);
   if (fieldPtr) {

     /* Add the field to the symbol table */
     fieldPtr->sParm.r.record = record;

   } /* end if */

   /* Return a pointer to the new variable symbol */

   return fieldPtr;

} /* end addField */

/***************************************************************/

void primeSymbolTable(unsigned long symbolTableSize)
{
  int32_t trueValue   = -1;
  int32_t falseValue  =  0;
  int32_t maxintValue = MAXINT;
  STYPE *typePtr;
  register int16_t i;

  TRACE(lstFile,"[primeSymbolTable]");

  /* Allocate and initialize symbol table */

  symbolTable = malloc(symbolTableSize * sizeof(STYPE));
  if (!symbolTable)
    {
      fatal(eNOMEMORY);
    }

  nsym = 0;

  /* Add the standard constants to the symbol table */

  (void)addConstant("TRUE",   tBOOLEAN_CONST, &trueValue,   NULL);
  (void)addConstant("FALSE",  tBOOLEAN_CONST, &falseValue,  NULL);
  (void)addConstant("MAXINT", tINT_CONST,     &maxintValue, NULL);
  (void)addConstant("NIL",    tNIL,           &falseValue,  NULL);

  /* Add the standard types to the symbol table */

  typePtr = addTypeDefine("INTEGER", sINT, sINT_SIZE, NULL);
  if (typePtr)
    {
      parentInteger             = typePtr;
      typePtr->sParm.t.minValue = MININT;
      typePtr->sParm.t.maxValue = MAXINT;
    } /* end if */

  typePtr = addTypeDefine("BOOLEAN", sBOOLEAN, sBOOLEAN_SIZE, NULL);
  if (typePtr)
    {
      typePtr->sParm.t.minValue = falseValue;
      typePtr->sParm.t.maxValue = trueValue;
    } /* end if */

  typePtr = addTypeDefine("REAL", sREAL, sREAL_SIZE, NULL);

  typePtr = addTypeDefine("CHAR", sCHAR, sCHAR_SIZE, NULL);
  if (typePtr)
    {
      typePtr->sParm.t.minValue = MINCHAR;
      typePtr->sParm.t.maxValue = MAXCHAR;
    } /* end if */

  typePtr = addTypeDefine("TEXT", sFILE_OF, sCHAR_SIZE, NULL);
  if (typePtr)
    {
      typePtr->sParm.t.subType  = sCHAR;
      typePtr->sParm.t.minValue = MINCHAR;
      typePtr->sParm.t.maxValue = MAXCHAR;
    } /* end if */

  /* Add some enhanced Pascal standard" types to the symbol table
   *
   * string is represent by a 256 byte memory regions consisting of
   * one byte for the valid string length plus 255 bytes for string
   * storage
   */

  typePtr = addTypeDefine("STRING", sSTRING, sSTRING_SIZE, NULL);
  if (typePtr)
    {
      parentString              = typePtr;
      typePtr->sParm.t.rtype    = sRSTRING;
      typePtr->sParm.t.subType  = sCHAR;
      typePtr->sParm.t.rsize    = sRSTRING_SIZE;
      typePtr->sParm.t.flags    = STYPE_VARSIZE;
      typePtr->sParm.t.minValue = MINCHAR;
      typePtr->sParm.t.maxValue = MAXCHAR;
    } /* end if */

  /* Add the standard files to the symbol table */

  (void)addFile("INPUT",   0);
  (void)addFile("OUTPUT",  0);

  /* Initialize files table */

  for (i = 0; i <= MAX_FILES; i++)
    {
      files [i].defined = 0;
      files [i].flevel  = 0;
      files [i].ftype   = 0;
      files [i].faddr   = 0;
      files [i].fsize   = 0;
    } /* end for */
} /* end primeSymbolTable */

/***************************************************************/

void verifyLabels(int32_t symIndex)
{
   register int16_t i;                 /* loop index */

   for (i=symIndex; i < nsym; i++)
     if ((symbolTable[i].sKind == sLABEL)
     &&  (symbolTable[i].sParm.l.unDefined))
       error (eUNDEFLABEL);
} /* end verifyLabels */

/***************************************************************/

#if CONFIG_DEBUG
const char noName[] = "********";
void dumpTables(void)
{
  register int16_t i;

  fprintf(lstFile,"\nSYMBOL TABLE:\n");
  fprintf(lstFile,"[  Addr  ]     NAME KIND LEVL\n");

  for (i = 0; i < nsym; i++)
    {
      fprintf(lstFile,"[%08lx] ", (uint32_t)&symbolTable[i]);

      if (symbolTable[i].sName)
        fprintf(lstFile, "%8s", symbolTable[i].sName);
      else
        fprintf(lstFile, "%8s", noName);

      fprintf(lstFile," %04x %04x ",
              symbolTable[i].sKind,
              symbolTable[i].sLevel);

      switch (symbolTable[i].sKind)
        {
          /* Constants */

        case tINT_CONST :
        case tCHAR_CONST :
        case tBOOLEAN_CONST :
        case tNIL :
        case sSCALAR :
          fprintf(lstFile, "val=%ld parent=[%08lx]\n",
                  symbolTable[i].sParm.c.val.i,
                  (unsigned long)symbolTable[i].sParm.c.parent);
          break;
        case tREAL_CONST :
          fprintf(lstFile, "val=%f parent=[%08lx]\n",
                  symbolTable[i].sParm.c.val.f,
                  (unsigned long)symbolTable[i].sParm.c.parent);
          break;

          /* Types */

        case sTYPE  :
          fprintf(lstFile,
                  "type=%02x rtype=%02x subType=%02x flags=%02x "
                  "asize=%ld rsize=%ld minValue=%ld maxValue=%ld "
                  "parent=[%08lx]\n",
                  symbolTable[i].sParm.t.type,
                  symbolTable[i].sParm.t.rtype,
                  symbolTable[i].sParm.t.subType,
                  symbolTable[i].sParm.t.flags,
                  symbolTable[i].sParm.t.asize,
                  symbolTable[i].sParm.t.rsize,
                  symbolTable[i].sParm.t.minValue,
                  symbolTable[i].sParm.t.maxValue,
                  (unsigned long)symbolTable[i].sParm.t.parent);
          break;

          /* Procedures/Functions */

          /* Procedures and Functions */

        case sPROC :
        case sFUNC :
          fprintf(lstFile,
                  "label=L%04x nParms=%d flags=%02x parent=[%08lx]\n",
                  symbolTable[i].sParm.p.label,
                  symbolTable[i].sParm.p.nParms,
                  symbolTable[i].sParm.p.flags,
                  (unsigned long)symbolTable[i].sParm.p.parent);
          break;

          /* Labels */

        case sLABEL :
          fprintf(lstFile, "label=L%04x unDefined=%d\n",
                  symbolTable[i].sParm.l.label,
                  symbolTable[i].sParm.l.unDefined);
          break;

          /* Files */

        case sFILE :
          fprintf(lstFile, "fileNumber=%d\n",
                  symbolTable[i].sParm.fileNumber);
          break;

          /* Variables */

        case sINT :
        case sBOOLEAN :
        case sCHAR  :
        case sREAL :
        case sTEXT :
        case sARRAY :
        case sPOINTER :
        case sVAR_PARM :
        case sRECORD :
        case sFILE_OF :
          fprintf(lstFile, "offset=%ld size=%ld flags=%02x parent=[%08lx]\n",
                  symbolTable[i].sParm.v.offset,
                  symbolTable[i].sParm.v.size,
                  symbolTable[i].sParm.v.flags,
                  (unsigned long)symbolTable[i].sParm.v.parent);
          break;

          /* Record objects */

        case sRECORD_OBJECT :
          fprintf(lstFile,
                  "offset=%ld size=%ld record=[%08lx] parent=[%08lx]\n",
                  symbolTable[i].sParm.r.offset,
                  symbolTable[i].sParm.r.size,
                  (unsigned long)symbolTable[i].sParm.r.record,
                  (unsigned long)symbolTable[i].sParm.r.parent);
          break;

          /* Constant strings */

        case sSTRING_CONST :
          fprintf(lstFile, "offset=%04lx size=%ld\n",
                  symbolTable[i].sParm.s.offset,
                  symbolTable[i].sParm.s.size);
          break;

        default :
          fprintf(lstFile, "Unknown sKind\n");
          break;
     
        } /* end switch */
    } /* end for */

} /* end dumpTables */
#endif

/***************************************************************/

