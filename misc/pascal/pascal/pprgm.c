/**********************************************************************
 * pas.c
 * main - process PROGRAM
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

#include "pas.h"       /* for globals + openNestedFile */
#include "pblck.h"     /* for block() */
#include "pgen.h"      /* for pas_Generate*() */
#include "ptkn.h"      /* for getToken() */
#include "ptbl.h"      /* for addFile() */
#include "pofflib.h"   /* For poff*() functions*/
#include "paslib.h"    /* for extension() */
#include "perr.h"      /* for error() */
#include "punit.h"     /* for unit() */
#include "pprgm.h"

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * Public Functions
 ***********************************************************************/

void program(void)
{
  char *pgmname = NULL;

  TRACE(lstFile, "[program]");

  /* FORM: program = program-heading ';' [uses-section ] block '.'
   * FORM: program-heading = 'program' identifier [ '(' identifier-list ')' ]
   *
   * On entry, 'program' has already been identified and token refers to
   * the next token after 'program'
   */

  if (token != tIDENT) error(eIDENT);      /* Verify <program name> */
  else
    {
      pgmname = tkn_strt;                  /* Save program name */
      getToken();
    } /* end else */

  /* Process optional file list (allow re-declaration of INPUT & OUTPUT) */

  if (token == '(')
    {
      do
        {
          getToken();
          if (token == tIDENT)
            {
              if ((++nfiles) > MAX_FILES) fatal(eOVF);
              (void)addFile(tkn_strt, nfiles);
              stringSP = tkn_strt;
              getToken();
            } /* end if */
          else if ((token == sFILE) && !(tknPtr->sParm.fileNumber))
            getToken();
          else
            error(eIDENT);
        }
      while (token == ',');
      if (token != ')') error(eRPAREN);
      else getToken();
    } /* End if */

  /* Make sure that a semicolon follows the program-heading */

  if (token != ';') error(eSEMICOLON);
  else getToken();

  /* Set the POFF file header type */

  poffSetFileType(poffHandle, FHT_PROGRAM, nfiles, pgmname);
  poffSetArchitecture(poffHandle, FHA_PCODE);

  /* Discard the program name string */

  stringSP = pgmname;

  /* Process the optional 'uses-section'
   * FORM: uses-section = 'uses' [ uses-unit-list ] ';'
   */

  if (token == tUSES)
    {
      getToken();
      usesSection();
    }

  /* Process the block */

  block();
  if (token !=  '.') error(ePERIOD);
  pas_GenerateSimple(opEND);
} /* end program */

/***********************************************************************/

void usesSection(void)
{
  uint16_t saveToken;
  char defaultUnitFileName[FNAME_SIZE + 1];
  char *unitFileName = NULL;
  char *saveTknStrt;
  char *unitName;

  TRACE(lstFile, "[usesSection]");

  /* FORM: uses-section = 'uses' [ uses-unit-list ] ';'
   * FORM: uses-unit-list = unit-import {';' uses-unit-list }
   * FORM: unit-import = identifier ['in' non-empty-string ]
   *
   * On entry, token will point to the token just after
   * the 'uses' reservers word.
   */

  while (token == tIDENT)
    {
      /* Save the unit name identifier and skip over the identifier */

      unitName = tkn_strt;
      getToken();

      /* Check for the optional 'in' */

      saveTknStrt = tkn_strt;
      if (token == tIN)
        {
          /* Skip over 'in' and verify that a string constant representing
           * the file name follows.
           */

          getToken();
          if (token != tSTRING_CONST) error(eSTRING);
          else
            {
              /* Save the unit file name and skip to the
               * next token.
               */

              unitFileName = tkn_strt;
              saveTknStrt = tkn_strt;
              getToken();
            }
        }

      /* In any event, make sure that we have a non-NULL unit
       * file name.
       */

      if (!unitFileName)
        {
          /* Create a default filename */

          (void)extension(unitName, ".pas", defaultUnitFileName, 1);
          unitFileName = defaultUnitFileName;
        }

      /* Open the unit file */

      saveToken   = token;
      openNestedFile(unitFileName);
      FP->kind    = eIsUnit;
      FP->section = eIsOtherSection;

      /* Verify that this is a unit file */

      if (token != tUNIT) error(eUNIT);
      else getToken();

      /* Release the file name from the string stack */

      stringSP = saveTknStrt;

      /* Verify that the file provides the unit that we are looking
       * for (only one unit per file is supported)
       */

      if (token != tIDENT) error(eIDENT);
      else if (strcmp(unitName, tkn_strt) != 0) error(eUNITNAME);

      /* Parse the interface from the unit file (token must refer
       * to the unit name on entry into unit().
       */

      unitInterface();
      closeNestedFile();

      /* Verify the terminating semicolon */

      token = saveToken;
      if (token !=  ';') error(eSEMICOLON);
      else getToken();
    }
}

/***********************************************************************/
