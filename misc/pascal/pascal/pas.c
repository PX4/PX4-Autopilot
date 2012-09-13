/**********************************************************************
 * pas.c
 * Main process
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

#define _GNU_SOURCE
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <signal.h>
#include <errno.h>

#include "config.h"
#include "keywords.h"
#include "pasdefs.h"
#include "ptdefs.h"
#include "podefs.h"
#include "pedefs.h"

#include "pas.h"
#include "paslib.h"   /* For extension */
#include "pproc.h"    /* For primeBuiltInProcedures */
#include "pfunc.h"    /* For primeBuiltInFunctions */
#include "ptkn.h"     /* For primeTokenizer */
#include "ptbl.h"     /* For primeSymbolTable */
#include "pofflib.h"  /* For poffInitializeForOutput() */
#include "poff.h"     /* For POFF definitions */
#include "pprgm.h"    /* for program() */
#include "punit.h"    /* for unit() */
#include "perr.h"     /* for error() */

/**********************************************************************
 * Pre-processor Definitions
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

/* Unitialized Global Data */

uint16_t    token;                   /* Current token */
uint16_t    tknSubType;              /* Extended token type */
int32_t     tknInt;                  /* Integer token value */
double      tknReal;                 /* Real token value */
STYPE      *tknPtr;                  /* Pointer to symbol token*/
WTYPE       withRecord;              /* RECORD used with WITH statement */
FTYPE       files[MAX_FILES+1];      /* File Table */
fileState_t fileState[MAX_INCL];     /* State of all open files */

/* sourceFileName : Program name from command line
 * includePath[] : Pathes to search when including file
 */

char       *sourceFileName;
char       *includePath[MAX_INCPATHES];

poffHandle_t poffHandle;             /* Handle for POFF object */

FILE       *poffFile;                /* Pass1 POFF output file */
FILE       *lstFile;                 /* List File pointer */
FILE       *errFile;                 /* Error file pointer */

/* Initialized Global Data */

int16_t     level        = 0;        /* Static nesting level */
int16_t     includeIndex = 0;        /* Include file index */
int16_t     nIncPathes   = 0;        /* Number pathes in includePath[] */
uint16_t    label        = 0;        /* Last label number */
int16_t     nsym         = 0;        /* Number symbol table entries */
int16_t     nconst       = 0;        /* Number constant table entries */
int16_t     sym_strt     = 0;        /* Symbol search start index */
int16_t     const_strt   = 0;        /* Constant search start index */
int16_t     err_count    = 0;        /* Error counter */
int16_t     nfiles       = 0;        /* Program file counter */
int32_t     warn_count   = 0;        /* Warning counter */
int32_t     dstack       = 0;        /* data stack size */

/**********************************************************************
 * Private Type Definitions
 **********************************************************************/

struct outFileDesc_s
{
  const char *extension;
  const char *flags;
  FILE      **stream;
};
typedef struct outFileDesc_s outFileDesc_t;

/**********************************************************************
 * Private Variables
 **********************************************************************/

static const outFileDesc_t outFiles[] =
{
  { "o1",  "wb", &poffFile },      /* Pass 1 POFF object file */
#if LSTTOFILE
  { "lst", "w",  &lstFile },       /* List file */
#endif
  { "err", "w",  &errFile },       /* Error file */
  { NULL,  NULL }                  /* (terminates list */
};

static const char *programName;

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

static void closeFiles(void);
static void openOutputFiles(void);
static void showUsage(void);
static void parseArguments(int argc, char **argv);

/***********************************************************************
 * Private Functions
 ***********************************************************************/

static void closeFiles(void)
{
  const outFileDesc_t *outFile;

  /* Close input source files */

  for(; includeIndex >= 0; includeIndex--)
    {
      if (FP->stream)
        {
          (void)fclose(FP->stream);
          FP->stream = NULL;
        }
    }

  /* Close output files */

  for (outFile = outFiles; outFile->extension; outFile++)
    {
      if (*outFile->stream)
        {
          (void)fclose(*outFile->stream);
          *outFile->stream = NULL;
        }
    }
}

/***********************************************************************/

static void openOutputFiles(void)
{
  const outFileDesc_t *outFile;
  char tmpname[FNAME_SIZE+1];

  /* Open output files */

  for (outFile = outFiles; outFile->extension; outFile++)
    {
      /* Generate an output file name from the source file
       * name and an extension associated with the output file.
       */

      (void)extension(sourceFileName, outFile->extension, tmpname, 1);
      *outFile->stream = fopen(tmpname, outFile->flags);
      if (*outFile->stream == NULL)
        {
          fprintf(stderr, "Could not open output file '%s': %s\n",
                  tmpname, strerror(errno));
          showUsage(); 
        }
    }
}

/***********************************************************************/

static void signalHandler(int signo)
{
#ifdef  _GNU_SOURCE
  fprintf(errFile, "Received signal: %s\n", strsignal(signo));
  fprintf(lstFile, "Received signal: %s\n", strsignal(signo));
#else
  fprintf(errFile, "Received signal %d\n", signo);
  fprintf(lstFile, "Received signal %d\n", signo);
#endif
  closeFiles();
  error(eRCVDSIGNAL);
  exit(1);
}

/***********************************************************************/

static void primeSignalHandlers(void)
{
  (void)signal(SIGHUP,  signalHandler);
  (void)signal(SIGINT,  signalHandler);
  (void)signal(SIGQUIT, signalHandler);
  (void)signal(SIGILL,  signalHandler);
  (void)signal(SIGABRT, signalHandler);
  (void)signal(SIGSEGV, signalHandler);
  (void)signal(SIGTERM, signalHandler);
}

/***********************************************************************/

static void showUsage(void)
{
  fprintf(stderr, "USAGE:\n");
  fprintf(stderr, "  %s [options] <filename>\n", programName);
  fprintf(stderr, "[options]\n");
  fprintf(stderr, "  -I<include-path>\n");
  fprintf(stderr, "    Search in <include-path> for additional file\n");
  fprintf(stderr, "    A maximum of %d pathes may be specified\n",
          MAX_INCPATHES);
  fprintf(stderr, "    (default is current directory)\n");
  closeFiles();
  exit(1);
} /* end showUsage */

/***********************************************************************/

static void parseArguments(int argc, char **argv)
{
  int i;

  programName = argv[0];

  /* Check for existence of at least the filename argument */

  if (argc < 2)
    {
      fprintf(stderr, "Invalid number of arguments\n");
      showUsage(); 
    }

  /* Parse any optional command line arguments */

  for (i = 1; i < argc-1; i++)
    {
      char *ptr = argv[i];
      if (ptr[0] == '-')
        {
          switch (ptr[1])
            {
            case 'I' :
              if (nIncPathes >= MAX_INCPATHES)
                {
                  fprintf(stderr, "Unrecognized [option]\n");
                  showUsage(); 
                }
              else
                {
                  includePath[nIncPathes] = &ptr[2];
                  nIncPathes++;
                }
              break;
            default:
              fprintf(stderr, "Unrecognized [option]\n");
              showUsage(); 
            }
        }
      else
        {
          fprintf(stderr, "Unrecognized [option]\n");
          showUsage(); 
        }
    }

  /* Extract the Pascal program name from the command line */

  sourceFileName = argv[argc-1];
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

int main(int argc, char *argv[])
{
  char filename [FNAME_SIZE+1];

  /* Parse command line arguments */

  parseArguments(argc, argv);

  /* Open all output files */

  openOutputFiles();

#if !LSTTOFILE
  lstFile = stdout;
#endif

  /* Open source file -- Use .PAS or command line extension, if supplied */

  (void)extension(sourceFileName, "PAS", filename, 0);
  fprintf(errFile, "%01x=%s\n", FP->include, filename);

  memset(FP, 0, sizeof(fileState_t));
  FP->stream = fopen(filename, "r");
  if (!FP->stream)
    {
      errmsg("Could not open source file '%s': %s\n",
             filename, strerror(errno));
      showUsage(); 
    }
   
  /* Initialization */

  primeSignalHandlers();
  primeSymbolTable(MAX_SYM);
  primeBuiltInProcedures();
  primeBuiltInFunctions();
  primeTokenizer(MAX_STRINGS);

  /* Initialize the POFF object */

  poffHandle = poffCreateHandle();
  if (poffHandle == NULL)
    fatal(eNOMEMORY);

  /* Save the soure file name in the POFF output file */

  FP->include = poffAddFileName(poffHandle, filename);

  /* Define standard input/output file characteristics */

  files[0].defined = -1;
  files[0].flevel  = level;
  files[0].ftype   = sCHAR;
  files[0].faddr   = dstack;
  files[0].fsize   = sCHAR_SIZE;
  dstack          += sCHAR_SIZE;

  /* We need the following in order to calculate relative stack positions. */

  FP->dstack       = dstack;

  /* Indicate that no WITH statement has been processed */

  memset(&withRecord, 0, sizeof(WTYPE));

  /* Process the pascal program
   *
   * FORM: pascal = program | unit
   * FORM: program = program-heading ';' [uses-section ] block '.'
   * FORM: program-heading = 'program' identifier [ '(' identifier-list ')' ]
   * FORM: unit = unit-heading ';' interface-section implementation-section init-section
   * FORM: unit-heading = 'unit' identifer
   */

  getToken();
  if (token == tPROGRAM)
    { 
      /* Compile a pascal program */

      FP->kind    = eIsProgram;
      FP->section = eIsProgramSection;
      getToken();
      program();
    }
  else if (token == tUNIT)
    {
      /* Compile a pascal unit */

      FP->kind    = eIsUnit;
      FP->section = eIsOtherSection;
      getToken();
      unitImplementation();
    }
  else
    {
      /* Expected 'program' or 'unit' */

      error(ePROGRAM);
    }

  /* Dump the symbol table content (debug only) */

#if CONFIG_DEBUG
  dumpTables();
#endif

  /* Write the POFF output file */

  poffWriteFile(poffHandle, poffFile);
  poffDestroyHandle(poffHandle);

  /* Close all output files */

  closeFiles();

  /* Write Closing Message */

  if (warn_count > 0)
    {
      printf("  %ld Warnings Issued\n", warn_count);
    } /* end if */

  if (err_count > 0)
    {
      printf("  %d Errors Detected\n\n", err_count);
      return -1;
    } /* end if */

  return 0;

} /* end main */

/***********************************************************************/

void openNestedFile(const char *fileName)
{
  fileState_t *prev = FP;
  char fullpath[FNAME_SIZE + 1];
  int i;

  /* Make sure we can handle another nested file */

  if (++includeIndex >= MAX_INCL) fatal(eOVF);
  else
    {
      /* Clear the file state structure for the new include level */

      memset(FP, 0, sizeof(fileState_t));

      /* Try all source include pathes until we find the file or
       * until we exhaust the include path list.
       */

      for (i = 0; ; i++)
        {
          /* Open the nested file -- try all possible pathes or
           * until we successfully open the file.
           */

          /* The final path that we will try is the current directory */

          if (i == nIncPathes)
            {
              sprintf(fullpath, "./%s", fileName);
            }
          else
            {
              sprintf(fullpath, "%s/%s", includePath[i], fileName);
            }

          FP->stream = fopen (fullpath, "rb");
          if (!FP->stream)
            {
              /* We failed to open the file.  If there are no more
               * include pathes to examine (including the current directory),
               * then error out.  This is fatal.  Otherwise, continue
               * looping.
               */

              if (i == nIncPathes)
                {
                  errmsg("Failed to open '%s': %s\n",
                         fileName, strerror(errno));
                  fatal(eINCLUDE);
                  break; /* Won't get here */
                }
            } /* end else if */
          else
            break;
        }

      /* Setup the newly opened file */

      fprintf(errFile, "%01x=%s\n", FP->include, fullpath);
      FP->include = poffAddFileName(poffHandle, fullpath);

      /* The caller may change this, but the default behavior is
       * to inherit the kind and section of the including file
       * and the current data stack offset.
       */

      FP->kind    = prev->kind;
      FP->section = prev->section;
      FP->dstack  = dstack;

      rePrimeTokenizer();

      /* Get the first token from the file */

      getToken();
    } /* end else */
}

/***********************************************************************/

void closeNestedFile(void)
{
  if (FP->stream)
    {
      (void)fclose(FP->stream);
      includeIndex--;
    }
}

/***********************************************************************/
