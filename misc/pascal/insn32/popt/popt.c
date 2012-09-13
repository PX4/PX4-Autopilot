/**********************************************************************
 * popt.c
 * P-Code Optimizer Main Logic
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "keywords.h"
#include "podefs.h"
#include "paslib.h"
#include "pofflib.h"

#include "pinsn.h"
#include "popt.h"
#include "psopt.h"
#include "polocal.h"
#include "pfopt.h"

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void showUsage         (const char *progname, int errcode);
static void readPoffFile      (const char *filename);
static void pass1             (void);
static void pass2             (void);
static void pass3             (void);
static void writePoffFile     (const char *filename);

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

static poffHandle_t poffHandle; /* Handle to POFF object */
static int no_resolve = 0;

/**********************************************************************
 * Global Functions
 **********************************************************************/

/***********************************************************************/

int main(int argc, char *argv[], char *envp[])
{
  const char *outfilename;
  TRACE(stderr, "[main]");
  int option;

  /* Process command line argruments */

  while ((option = getopt(argc, argv, "rh")) > 0)
  {
    switch (option)
      {
      case 'r' :
        no_resolve++;
	break;
      case 'h' :
	showUsage(argv[0], 0);
      default:
	fprintf(stderr, "Unrecognized option\n");
	showUsage(argv[0], -1);
      }
  }

  /* Check for existence of filename argument */

  if (optind != argc - 1)
    {
      printf("Filename required at end of command line.\n");
      showUsage(argv[0], -1);
    } /* end if */

  /* Read the POFF file into memory */

  outfilename = argv[optind];
  readPoffFile(outfilename);

  /* Performs pass1 optimization */

  pass1();

  /* Performs pass2 optimization */

  insn_ResetOpCodeRead(poffHandle);
  pass2();

  if (!no_resolve)
    {
      /* Create final section offsets and relocation entries */

      insn_ResetOpCodeRead(poffHandle);
      pass3();
    }

  /* Write the POFF file */

  writePoffFile(outfilename);
  return 0;

} /* End main */

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static void showUsage(const char *progname, int errcode)
{
  fprintf(stderr, "USAGE:\n");
  fprintf(stderr, "  %s -h\n", progname);
  fprintf(stderr, "  %s [-r] <poff-filename>\n", progname);
  fprintf(stderr, "WHERE:\n");
  fprintf(stderr, "  -r:  Disables label resolution (default: labels resolved)\n");
  fprintf(stderr, "  -h:  Shows this message\n");
  exit(errcode);
}

/***********************************************************************/

static void readPoffFile(const char *filename)
{
  char    objname [FNAME_SIZE+1];
  FILE   *objFile;
  int     errcode;

  TRACE(stderr, "[readPoffFile]");

  /* Open the pass1 POFF object file -- Use .o1 extension */

  (void)extension(filename, "o1", objname, 1);
  if (!(objFile = fopen(objname, "rb")))
    {
      printf("Error Opening %s\n", objname);
      exit(1);
    } /* end if */

  /* Get a handle to a POFF input object */

  poffHandle = poffCreateHandle();
  if (!poffHandle)
    {
      printf("Could not get POFF handle\n");
      exit(1);
    } /* end if */

  /* Read the POFF file into memory */

  errcode = poffReadFile(poffHandle, objFile);
  if (errcode != 0)
    {
      printf("Could not read POFF file, errcode=0x%02x\n", errcode);
      exit(1);
    }

  /* Close the input file */

  fclose(objFile);
} /* end pass1 */

/***********************************************************************/

static void pass1(void)
{
  poffProgHandle_t poffProgHandle; /* Handle to temporary POFF object */

  TRACE(stderr, "[pass1]");

  /* Create a handle to a temporary object to store new POFF program
   * data.
   */

  poffProgHandle = poffCreateProgHandle();
  if (!poffProgHandle)
    {
      printf("Could not get POFF handle\n");
      exit(1);
    } /* end if */

  /* Clean up garbage left from the wasteful string stack logic */

  stringStackOptimize(poffHandle, poffProgHandle);

  /* Replace the original program data with the new program data */

  poffReplaceProgData(poffHandle, poffProgHandle);

  /* Release the temporary POFF object */

  poffDestroyProgHandle(poffProgHandle);
} /* end pass1 */

/***********************************************************************/

static void pass2(void)
{
  poffProgHandle_t poffProgHandle; /* Handle to temporary POFF object */

  TRACE(stderr, "[pass2]");

  /* Create a handle to a temporary object to store new POFF program
   * data.
   */

  poffProgHandle = poffCreateProgHandle();
  if (!poffProgHandle)
    {
      printf("Could not get POFF handle\n");
      exit(1);
    } /* end if */

  /* Perform Local Optimizatin Initialization */

  localOptimization(poffHandle, poffProgHandle);

  /* Replace the original program data with the new program data */

  poffReplaceProgData(poffHandle, poffProgHandle);

  /* Release the temporary POFF object */

  poffDestroyProgHandle(poffProgHandle);
} /* end pass2 */

/***********************************************************************/

static void pass3 (void)
{
  poffProgHandle_t poffProgHandle; /* Handle to temporary POFF object */
  TRACE(stderr, "[pass3]");

  /* Create a handle to a temporary object to store new POFF program
   * data.
   */

  poffProgHandle = poffCreateProgHandle();
  if (!poffProgHandle)
    {
      printf("Could not get POFF handle\n");
      exit(1);
    } /* end if */

  /* Finalize program section, create relocation and line number
   * sections.
   */

  optFinalize(poffHandle, poffProgHandle);

  /* Release the temporary POFF object */

  poffDestroyProgHandle(poffProgHandle);
}

/***********************************************************************/

static void writePoffFile(const char *filename)
{
  char    optname [FNAME_SIZE+1];
  FILE   *optFile;

  TRACE(stderr, "[writePoffFile]");

  /* Open optimized p-code file -- Use .o extension */

  (void)extension(filename, "o", optname, 1);
  if (!(optFile = fopen(optname, "wb")))
    {
      printf("Error Opening %s\n", optname);
      exit(1);
    } /* end if */

  /* Then write the new POFF file */

  poffWriteFile(poffHandle, optFile);

  /* Destroy the POFF object */

  poffDestroyHandle(poffHandle);

  /* Close the files used on writePoffFile */

  (void)fclose(optFile);
} /* end writePoffFile */

/***********************************************************************/
