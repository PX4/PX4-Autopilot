/**********************************************************************
 * plist.c
 * POFF file lister
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
#include <ctype.h>
#include <string.h>

#define _GNU_SOURCE
#include <getopt.h>

#include "keywords.h"
#include "pdefs.h"
#include "podefs.h"
#include "pinsn16.h"
#include "pedefs.h"

#include "pofflib.h"

#include "paslib.h"
#include "pinsn.h"

/**********************************************************************
 * Definitions
 **********************************************************************/

#define MAX_STRING 80

/**********************************************************************
 * Private Data
 **********************************************************************/

static char *poffFileName      = NULL;
static int  showFileHeader     = 0;
static int  showSectionHeaders = 0;
static int  showSymbols        = 0;
static int  showRelocs         = 0;
static int  disassemble        = 0;

/**********************************************************************
 * Private Constant Data
 **********************************************************************/

static const struct option long_options[] =
{
  {"all",              0, NULL, 'a'},
  {"file-header",      0, NULL, 'h'},
  {"section-headers",  0, NULL, 'S'},
  {"symbols",          0, NULL, 's'},
  {"relocs",           0, NULL, 'r'},
  {"disassemble",      0, NULL, 'd'},
  {"help",             0, NULL, 'H'},
  {NULL,               0, NULL, 0}
};

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

static void showUsage       (const char *progname);
static void parseArgs       (int argc, char **argv);
static void dumpProgramData (poffHandle_t poffHandle);

/**********************************************************************
 * Global Functions
 **********************************************************************/

int main (int argc, char *argv[], char *envp[])
{
  FILE   *object;                  /* Object file pointer */
  poffHandle_t poffHandle;         /* Handle for POFF object */
  char    fileName[FNAME_SIZE+1];  /* Object file name */
  uint16_t errCode;                /* See pedefs.h */

  /* Parse the command line arguments */

  parseArgs(argc, argv);

  /* Open source POFF file -- Use .o or command line extension, if supplied */

  (void) extension (poffFileName, "o", fileName, 0);
  if (!(object = fopen (fileName, "rb")))
    {
      printf ("Error opening %s\n", fileName);
      exit (1);
    } /* end if */

  /* Read the POFF file */

  poffHandle = poffCreateHandle();
  if (poffHandle == NULL)
    {
      printf ("Could not get POFF handler\n");
      exit (1);
    }

  errCode = poffReadFile(poffHandle, object);
  if (errCode != eNOERROR)
    {
      printf ("Could not read POFF file\n");
      exit (1);
    }

  /* Dump the File Header */

  if (showFileHeader)
    {
      poffDumpFileHeader(poffHandle, stdout);
    }

  /* Dump the Section Headers */

  if (showSectionHeaders)
    {
      poffDumpSectionHeaders(poffHandle, stdout);
    }

  /* Dump the symbol table */

  if (showSymbols)
    {
      poffDumpSymbolTable(poffHandle, stdout);
    }

  /* Dump the relocation table */

  if (showRelocs)
    {
      poffDumpRelocTable(poffHandle, stdout);
    }

  /* Dump the program data section -- Main Loop */

  if (disassemble)
    {
      dumpProgramData(poffHandle);
    }

  /* Close files and release objects */

  poffDestroyHandle(poffHandle);
  (void)fclose(object);
  return 0;
} /* end main */

/**********************************************************************
 * Private Functions
 **********************************************************************/

static void showUsage(const char *progname)
{
  fprintf(stderr, "Usage:\n");
  fprintf(stderr, "  %s [options] <poff-filename>\n",
          progname);
  fprintf(stderr, "options:\n");
  fprintf(stderr, "  -a --all              Equivalent to: -h -S -s -r -d\n");
  fprintf(stderr, "  -h --file-header      Display the POFF file header\n");
  fprintf(stderr, "  -S --section-headers  Display the sections' header\n");
  fprintf(stderr, "  -s --symbols          Display the symbol table\n");
  fprintf(stderr, "  -r --relocs           Display the relocations\n");
  fprintf(stderr, "  -d --disassemble      Display disassembled text\n");
  fprintf(stderr, "  -H --help             Display this information\n");
  exit(1);
}

/***********************************************************************/

static void parseArgs(int argc, char **argv)
{
  int option_index;
  int c;

  /* Check for existence of filename argument */

  if (argc < 2)
    {
      fprintf(stderr, "ERROR: POFF filename required\n");
      showUsage(argv[0]);
    } /* end if */

  /* Parse the command line options */

  do
    {
      c = getopt_long (argc, argv, "ahSsrdH",
                       long_options, &option_index);
      if (c != -1)
        {
          switch (c)
            {
            case 'a' :
              showFileHeader     = 1;
              showSectionHeaders = 1;
              showSymbols        = 1;
              showRelocs         = 1;
              disassemble        = 1;
              break;

            case 'h' :
              showFileHeader     = 1;
              break;

            case 'S' :
              showSectionHeaders = 1;
              break;

            case 's' :
              showSymbols        = 1;
              break;

            case 'r' :
              showRelocs         = 1;
              break;

            case 'd' :
              disassemble        = 1;
              break;

            case 'H' :
              showUsage(argv[0]);
              break;

            default:
              /* Shouldn't happen */

              fprintf(stderr, "ERROR: Unrecognized option\n");
              showUsage(argv[0]);
            }
        }
    }
  while (c != -1);

  /* Get the name of the p-code file(s) from the last argument(s) */

  if (optind != argc-1)
    {
      fprintf(stderr, "ERROR: POFF filename required as final argument\n");
      showUsage(argv[0]);
    }

  /* Save the POFF file name */

  poffFileName = argv[argc-1];
}

/***********************************************************************/

static void dumpProgramData(poffHandle_t poffHandle)
{
  poffLibLineNumber_t *lastln;     /* Previous line number reference */
  poffLibLineNumber_t *ln;         /* Current line number reference */
  uint32_t pc;                     /* Program counter */
  OPTYPE  op;                      /* Opcode */
  int     opSize;                  /* Size of the opcode */
  int     inch;                    /* Input char */

  /* Read the line number entries from the POFF file */

  poffReadLineNumberTable(poffHandle);

  /* Dump the program data section -- DumpProgramData Loop */

  pc     = 0;
  lastln = NULL;

  while ((inch = poffGetProgByte(poffHandle)) != EOF)
    {
      /* Get opcode arguments (if any) */

      op.op   = (uint8_t) inch;
      op.arg1 = 0;
      op.arg2 = 0;
      opSize  = 1;

      if (op.op & o8)
        {
          op.arg1 = poffGetProgByte(poffHandle);
          opSize += 1;
        }

      if (op.op & o16 )
        {
          op.arg2  = poffGetProgByte(poffHandle) << 8;
          op.arg2 |= poffGetProgByte(poffHandle);
          opSize  += 2;
        } /* end if */

      /* Find the line number associated with this line */

      ln = poffFindLineNumber(pc);
      if ((ln) && (ln != lastln))
        {
          /* Print the line number line */

          printf("\n%s:%ld\n", ln->filename, ln->lineno);

          /* This will suppress reporting the same line number
           * repeatedly.
           */

          lastln = ln;
        }

      /* Print the address then the opcode on stdout */

      fprintf(stdout, "%08lx ", pc);
      insn_DisassemblePCode(stdout, &op);

      /* Bump the PC to the next address */

      pc += opSize;

    } /* end while */

  /* Release buffers associated with line number information */

  poffReleaseLineNumberTable();

} /* end dumpProgramData */

/***********************************************************************/
