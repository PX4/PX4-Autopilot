/***********************************************************************
 * pascal/pasdefs.h
 * General definitions for the Pascal Compiler/Optimizer
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

#ifndef __PASDEFS_H
#define __PASDEFS_H

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>   /* for FILE */
#include <config.h>
#include "pdefs.h"   /* Common definitions */

/***********************************************************************
 * Pre-processor Definitions
 ***********************************************************************/

/* Size Parameters -- some of these can be overridden from the
 * command line.
 */

#define MAX_SYM           (4096)
#define MAX_STRINGS       (65536)
#define MAX_INCL           3      /* Max number of nested include files */
#define MAX_FILES          8      /* Max number of opened files */
#define FNAME_SIZE         40     /* Max size file name */
#define MAX_INCPATHES      8      /* Max number of include pathes */

/* Bit values for the 'flags' field of the symType_t, symProc_t, and
 * symVar_t (see below)
 */

#define STYPE_VARSIZE      0x01   /* Type has variable size */
#define SPROC_EXTERNAL     0x01   /* Proc/func. is defined externally */
#define SVAR_EXTERNAL      0x01   /* Variable is defined externally */

/***********************************************************************
 * Public Enumeration Types
 ***********************************************************************/

/* This enumeration identies what kind of binary object we are creating
 * with the compilation.  At present, we may be generating either a 
 * program binary or a unit binary.
 */

enum fileKind_e
{
  eIsProgram = 0,
  eIsUnit
};
typedef enum fileKind_e fileKind_t;

/* This enumeration determines what part of a file that we are
 * processing now.
 */

enum fileSection_e
{
  eIsOtherSection = 0,      /* Unspecified part of the file */
  eIsProgramSection,        /* Any part of a program file */
  eIsInterfaceSection,      /* INTERFACE section of a unit file */
  eIsImplementationSection, /* IMPLEMENTATION section of a unit file */
  eIsInitializationSection, /* INITIALIZATION section of a unit file */
};
typedef enum fileSection_e fileSection_t;

/***********************************************************************
 * Public Structure/Types
 ***********************************************************************/

/* Reserved word table entry */

struct R
{
  char   *rname;         /* pointer to name in string stack */
  uint8_t rtype;         /* reserved word type */
  uint8_t subtype;       /* reserved word extended type */
};
typedef struct R RTYPE;

/* Symbol table entry */

struct symType_s         /* for sKind = sTYPE */
{
  uint8_t   type;        /* specific type */
  uint8_t   rtype;       /* reference to type */
  uint8_t   subType;     /* constant type for subrange types */
  uint8_t   flags;       /* flags to customize a type (see above) */
  uint32_t  asize;       /* size of allocated instances of this type */
  uint32_t  rsize;       /* size of reference to an instances of this type */
  int32_t   minValue;    /* minimum value taken subrange */
  int32_t   maxValue;    /* maximum value taken by subrange or scalar */
  struct S *parent;      /* pointer to parent type */
};
typedef struct symType_s symType_t;

struct symConst_s        /* for sKind == constant type */
{
  union
  {
    double  f;           /* real value */
    int32_t i;           /* integer value */
  } val;
  struct S *parent;      /* pointer to parent type */
};
typedef struct symConst_s symConst_t;

struct symStringConst_s  /* for sKind == sSTRING_CONST */
{
  uint32_t offset;       /* RO data section offset of string */
  uint32_t size;         /* length of string in bytes */
};
typedef struct symStringConst_s symStringConst_t;

struct symVarString_s    /* for sKind == sSTRING */
{
  uint16_t label;        /* label at string declaration */
  uint16_t size;         /* valid length of string in bytes */
  uint16_t alloc;        /* max length of string in bytes */
};
typedef struct symVarString_s symVarString_t;

struct symLabel_s        /* for sKind == sLABEL */
{
  uint16_t label;        /* label number */
  bool     unDefined;    /* set false when defined */
};
typedef struct symLabel_s symLabel_t;

struct symVar_s          /* for sKind == type identifier */
{
  int32_t   offset;      /* Data stack offset */
  uint32_t  size;        /* Size of variable */
  uint8_t   flags;       /* flags to customize a variable (see above) */
  uint32_t  symIndex;    /* POFF symbol table index (if undefined) */
  struct S *parent;      /* pointer to parent type */
};
typedef struct symVar_s symVar_t;

struct symProc_s         /* for sKind == sPROC or sFUNC */
{
  uint16_t  label;       /* entry point label */
  uint16_t  nParms;      /* number of parameters that follow */
  uint8_t   flags;       /* flags to customize a proc/func (see above) */
  uint32_t  symIndex;    /* POFF symbol table index (if undefined) */
  struct S *parent;      /* pointer to parent type (sFUNC only) */
};
typedef struct symProc_s symProc_t;

struct symRecord_s       /* for sKind == sRECORD_OBJECT */
{
  uint32_t  size;        /* size of this field */
  uint32_t  offset;      /* offset into the RECORD */
  struct S *record;      /* pointer to parent sRECORD type */
  struct S *parent;      /* pointer to parent field type */
};
typedef struct symRecord_s symRecord_t;

struct S
{
  char     *sName;       /* pointer to name in string stack */
  uint8_t   sKind;       /* kind of symbol */
  uint8_t   sLevel;      /* static nesting level */
  union
  {
    symType_t        t;          /* for type definitions */
    symConst_t       c;          /* for constants */
    symStringConst_t s;          /* for strings of constant size*/
    symVarString_t   vs;         /* for strings of variable size*/
    uint16_t         fileNumber; /* for files */
    symLabel_t       l;          /* for labels */
    symVar_t         v;          /* for variables */
    symProc_t        p;          /* for functions & procedures */
    symRecord_t      r;          /* for files of RECORDS */
  } sParm;
};
typedef struct S STYPE;

/* WITH structure */

struct W
{
  uint8_t   level;       /* static nesting level */
  bool      pointer;     /* true if offset is to pointer to RECORD */
  bool      varParm;     /* true if VAR param (+pointer) */
  int32_t   offset;      /* Data stack offset */
  uint16_t  index;       /* RECORD offset (if pointer) */
  STYPE    *parent;      /* pointer to parent RECORD type */
};
typedef struct W WTYPE;

/* File table record */

struct F
{
  int16_t defined;
  int16_t flevel;
  int16_t ftype;
  int32_t faddr;
  int16_t fsize;
};
typedef struct F FTYPE;

/* This structure captures the parsing state of the compiler for a particular
 * file.  Since multiple, nested files can be processed, this represents
 * only level in the "stack" of nested files.
 */

struct fileState_s
{
  /* These fields are managed by the higher level parsing logic
   *
   * stream    - Stream pointer the input stream associated with this
   *             file.
   * kind      - Kind of file we are processing.  If include > 0,
   *             this should be eIsUnit.
   * section   - This is the part of the program that we are parsing
   *             now.
   * dstack    - Level zero dstack offset at the time the unit was
   *             included.  This is used to convert absolute program
   *             stack offsets into relative unit stack offsets.
   * include   - Is a unique number that identifies the file.  In
   *             POFF ouput file, this would be the index to the
   *             entry in the .files section.
   */

  FILE          *stream;
  fileKind_t     kind;
  fileSection_t  section;
  int32_t        dstack;
  int16_t        include;

  /* These fields are managed by the tokenizer.  These are all
   * initialized by primeTokenizer().
   *
   * buffer[]  - Holds the current input line
   * line      - Is the line number in this file for the current line
   * cp        - Is the current pointer into buffer[]
   */

  uint32_t       line;
  unsigned char *cp;
  unsigned char  buffer[LINE_SIZE + 1];
};
typedef struct fileState_s fileState_t;

#endif /* __PASDEFS_H */
