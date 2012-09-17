/**********************************************************************
 * regm.c
 * Convert 32-bit pcode defintions to a register model
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
#include <unistd.h>

#include "keywords.h"
#include "pdefs.h"
#include "pedefs.h"
#include "paslib.h"
#include "pofflib.h"
#include "perr.h"

#include "regm.h"
#include "regm_tree.h"
#include "regm_pass1.h"
#include "regm_pass2.h"

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void         regm_ShowUsage(const char *progname, int errcode);
static int          regm_CheckPoffFile(poffHandle_t hPoff);
static poffHandle_t regm_ReadPoffFile(const char *filename);
static void         regm_Pass3(poffHandle_t hPoff);
static void         regm_Pass4(poffHandle_t hPoff);
static void         regm_Pass5(poffHandle_t hPoff);
static void         regm_WritePoffFile(poffHandle_t hPoff,
                                       const char *filename);

/**********************************************************************
 * Public Variables
 **********************************************************************/

int vRegmDebug = 0;

/**********************************************************************
 * Private Variables
 **********************************************************************/

/**********************************************************************
 * Private Functions
 **********************************************************************/

/***********************************************************************/

static void regm_ShowUsage(const char *progname, int errcode)
{
  fprintf(stderr, "USAGE:\n");
  fprintf(stderr, "  %s -h\n", progname);
  fprintf(stderr, "  %s [-d] <poff-filename>\n", progname);
  fprintf(stderr, "WHERE:\n");
  fprintf(stderr, "  -d:  Enables debug output\n");
  fprintf(stderr, "  -h:  Shows this message\n");
  exit(errcode);
}

/***********************************************************************/

static poffHandle_t regm_ReadPoffFile(const char *filename)
{
  poffHandle_t hPoff;
  char  objname [FNAME_SIZE+1];
  FILE *objFile;
  int   errcode;

  TRACE(stderr, "[regm_ReadPoffFile]");

  /* Open the optimized POFF object file -- Use the .o extension */

  (void)extension(filename, "o", objname, 1);
  if (!(objFile = fopen(objname, "rb")))
    {
      printf("Error Opening %s\n", objname);
      exit(1);
    }

  /* Get a handle to a POFF input object */

  hPoff = poffCreateHandle();
  if (!hPoff)
    {
      printf("Could not get POFF handle\n");
      exit(1);
    }

  /* Read the POFF file into memory */

  errcode = poffReadFile(hPoff, objFile);
  if (errcode != 0)
    {
      printf("Could not read POFF file, errcode=0x%02x\n", errcode);
      exit(1);
    }

  /* Close the input file */

  fclose(objFile);
  return hPoff;
}

/***********************************************************************/

static int regm_CheckPoffFile(poffHandle_t hPoff)
{
  uint8_t fileArch = poffGetArchitecture(hPoff);
  if (fileArch != FHA_PCODE_INSN32)
    {
      fprintf(stderr, "ERROR: File is not 32-bit pcode (%d)\n",
              fileArch);
      return -1;
    }
  return 0;
}

/***********************************************************************/
/* Pass3: Perform local optimization */

static void regm_Pass3(poffHandle_t hPoff)
{
  TRACE(stderr, "[regm_Pass3]");
}

/***********************************************************************/
/* Pass 4: Fixup register usage, force to use a fixed number of registers
 * (arguments, static registers, volatile registers, special registers),
 * and add logic to handle large immediate values.
 */

static void regm_Pass4(poffHandle_t hPoff)
{
  TRACE(stderr, "[regm_Pass4]");
}

/***********************************************************************/
/* Pass 5: Fixup BL and B offsets, section headers, relocation entries,
 * symbol values
 */

static void regm_Pass5(poffHandle_t hPoff)
{
  TRACE(stderr, "[regm_Pass5]");
}

/***********************************************************************/

static void regm_WritePoffFile(poffHandle_t hPoff, const char *filename)
{
#if 0
  char    rexname [FNAME_SIZE+1];
  FILE   *rexFile;

  TRACE(stderr, "[regm_WritePoffFile]");

  /* Open optimized p-code file -- Use .o extension */

  (void)extension(filename, ".rex", rexname, 1);
  if (!(rexFile = fopen(rexname, "wb")))
    {
      printf("Error Opening %s\n", rexname);
      exit(1);
    }

  /* Then write the new POFF file */

  poffWritePoffFile(hPoff, rexFile);

  /* Destroy the POFF object */

  poffDestroyHandle(hPoff);

  /* Close the files used on regm_WritePoffFile */

  (void)fclose(rexFile);
#endif
}

/**********************************************************************
 * Public Functions
 **********************************************************************/

/***********************************************************************/

int main(int argc, char *argv[], char *envp[])
{
  const char *outfilename;
  poffHandle_t hPoff;
  TRACE(stderr, "[main]");
  int option;

  /* Process command line argruments */

  while ((option = getopt(argc, argv, "dh")) > 0)
    {
      switch (option)
        {
        case 'd' :
          vRegmDebug++;
          break;
        case 'h' :
          regm_ShowUsage(argv[0], 0);
        default:
          fprintf(stderr, "Unrecognized option\n");
          regm_ShowUsage(argv[0], -1);
        }
    }

  /* Check for existence of filename argument */

  if (optind != argc - 1)
    {
      fprintf(stderr, "Filename required at end of command line.\n");
      regm_ShowUsage(argv[0], -1);
    }

  /* Read the POFF file into memory */

  outfilename = argv[optind];
  hPoff = regm_ReadPoffFile(outfilename);

  /* Verify that it is the kind of file that we can handle */

  if (regm_CheckPoffFile(hPoff) != 0)
    {
      fprintf(stderr, "File is not the correct type\n");
      regm_ShowUsage(argv[0], -1);
    }

  /* Read the debug info into a more usable structure.  And release
   * the debug info in the POFF object (we will be replacing it later)
   */

  poffReadDebugFuncInfoTable(hPoff);
  poffDiscardDebugFuncInfo(hPoff);

  /* Pass 1: Break the POFF program data into sections and buffer in
   * a tree structure.
   */

  regm_InitTree();
  regm_Pass1(hPoff);
  regm_DumpTree();

  /* We can eliminate the buffered pcode data because we have
   * a copy in the tree and we will be replacing it before we
   * save the file.
   */

  poffReleaseProgData(hPoff);

  /* Pass 2: Convert the buffered pcode to the basic register model
   * with an indefinite number of registers (arguments, general, and
   * special registers) and with 32-bit immediate size.
   */

  regm_Pass2(hPoff);

  /* Pass3: Perform local optimization */

  regm_Pass3(hPoff);

  /* Pass 4: Fixup register usage, force to use a fixed number of registers
   * (arguments, static registers, volatile registers, special registers),
   * and add logic to handle large immediate values.
   */

  regm_Pass4(hPoff);

  /* Pass 5: Fixup BL and B offsets, section headers, relocation entries,
   * symbol values
   */

  regm_Pass5(hPoff);

  /* Replace the debug info in the POFF object with the debug info
   * we have modified.  We no longer need the buffered debug info after
   * this point.
   */

  poffReplaceDebugFuncInfo(hPoff);
  poffReleaseDebugFuncInfoTable();

  /* Write the REGM POFF file */

  poffSetArchitecture(hPoff, FHA_REGM_INSN32);
  regm_WritePoffFile(hPoff, outfilename);
  return 0;
}

/***********************************************************************/

void regm_ProgSeek(poffHandle_t hPoff, uint32_t dwOffset)
{
  insn_ResetOpCodeRead(hPoff);
  if (poffProgSeek(hPoff, dwOffset) < 0)
    {
      fatal(eSEEKFAIL);
    }
}

/***********************************************************************/

