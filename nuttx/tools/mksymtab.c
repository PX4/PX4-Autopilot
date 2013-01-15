/****************************************************************************
 * tools/mksymtab.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "csvparser.h"

/****************************************************************************
 * Definitions
 ****************************************************************************/

#define MAX_HEADER_FILES 500
#define SYMTAB_NAME      "g_symtab"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *g_hdrfiles[MAX_HEADER_FILES];
static int nhdrfiles;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void show_usage(const char *progname)
{
  fprintf(stderr, "USAGE: %s <cvs-file> <symtab-file>\n\n", progname);
  fprintf(stderr, "Where:\n\n");
  fprintf(stderr, "  <cvs-file>   : The path to the input CSV file\n");
  fprintf(stderr, "  <symtab-file>: The path to the output symbol table file\n");
  fprintf(stderr, "  -d           : Enable debug output\n");
  exit(EXIT_FAILURE);
}

static bool check_hdrfile(const char *hdrfile)
{
  int i;

  for (i = 0; i < nhdrfiles; i++)
    {
      if (strcmp(g_hdrfiles[i], hdrfile) == 0)
        {
          return true;
        }
    }

  return false;
}

static void add_hdrfile(const char *hdrfile)
{
  if (hdrfile && strlen(hdrfile) > 0)
    {
      if (!check_hdrfile(hdrfile))
        {
          if (nhdrfiles > MAX_HEADER_FILES)
            {
              fprintf(stderr, "ERROR:  Too man header files.  Increase MAX_HEADER_FILES\n");
              exit(EXIT_FAILURE);
            }

          g_hdrfiles[nhdrfiles] = strdup(hdrfile);
          nhdrfiles++;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *csvpath;
  char *symtab;
  char *nextterm;
  char *finalterm;
  char *ptr;
  bool cond;
  FILE *instream;
  FILE *outstream;
  int ch;
  int i;

  /* Parse command line options */

  g_debug = false;

  while ((ch = getopt(argc, argv, ":d")) > 0)
    {
      switch (ch)
        {
          case 'd' :
            g_debug = true;
            break;

          case '?' :
            fprintf(stderr, "Unrecognized option: %c\n", optopt);
            show_usage(argv[0]);

          case ':' :
            fprintf(stderr, "Missing option argument, option: %c\n", optopt);
            show_usage(argv[0]);

           break;
            fprintf(stderr, "Unexpected option: %c\n", ch);
            show_usage(argv[0]);
        }
    }

  if (optind >= argc)
    {
       fprintf(stderr, "Missing <cvs-file> and <symtab-file>\n");
       show_usage(argv[0]);          
    }

  csvpath = argv[optind];
  optind++;

  if (optind >= argc)
    {
       fprintf(stderr, "Missing <symtab-file>\n");
       show_usage(argv[0]);      
    }

  symtab = argv[optind];
  optind++;

  if (optind < argc)
    {
       fprintf(stderr, "Unexpected garbage at the end of the line\n");
       show_usage(argv[0]);      
    }    

  /* Open the CSV file for reading */

  instream = fopen(csvpath, "r");
  if (!instream)
    {
      fprintf(stderr, "open %s failed: %s\n", csvpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Open the Symbol table file for writing */

  outstream = fopen(symtab, "w");
  if (!outstream)
    {
      fprintf(stderr, "open %s failed: %s\n", csvpath, strerror(errno));
      exit(EXIT_FAILURE);
    }

  /* Get all of the header files that we need to include */

  while ((ptr = read_line(instream)) != NULL)
    {
      /* Parse the line from the CVS file */

      int nargs = parse_csvline(ptr);
      if (nargs < PARM1_INDEX)
        {
          fprintf(stderr, "Only %d arguments found: %s\n", nargs, g_line);
          exit(EXIT_FAILURE);
        }

      /* Add the header file to the list of header files we need to include */

      add_hdrfile(g_parm[HEADER_INDEX]);
    }

  /* Back to the beginning */

  rewind(instream);

  /* Output up-front file boilerplate */

  fprintf(outstream, "/* %s: Auto-generated symbol table.  Do not edit */\n\n", symtab);
  fprintf(outstream, "#include <nuttx/config.h>\n");
  fprintf(outstream, "#include <nuttx/binfmt/symtab.h>\n\n");

  /* Output all of the require header files */

  for (i = 0; i < nhdrfiles; i++)
    {
      fprintf(outstream, "#include <%s>\n", g_hdrfiles[i]);
    }

  /* Now the symbol table itself */

  fprintf(outstream, "\nstruct symtab_s %s[] =\n", SYMTAB_NAME);
  fprintf(outstream, "{\n");

  /* Parse each line in the CVS file */

  nextterm  = "";
  finalterm = "";

  while ((ptr = read_line(instream)) != NULL)
    {
      /* Parse the line from the CVS file */

      int nargs = parse_csvline(ptr);
      if (nargs < PARM1_INDEX)
        {
          fprintf(stderr, "Only %d arguments found: %s\n", nargs, g_line);
          exit(EXIT_FAILURE);
        }

      /* Output any conditional compilation */

      cond = (g_parm[COND_INDEX] && strlen(g_parm[COND_INDEX]) > 0);
      if (cond)
        {
          fprintf(outstream, "%s#if %s\n", nextterm, g_parm[COND_INDEX]);
          nextterm  = "";
        }

      /* Output the symbol table entry */

      fprintf(outstream, "%s  { \"%s\", (FAR const void *)%s }",
              nextterm, g_parm[NAME_INDEX], g_parm[NAME_INDEX]);

      if (cond)
        {
          nextterm  = ",\n#endif\n";
          finalterm = "\n#endif\n";
        }
      else
        {
          nextterm  = ",\n";
          finalterm = "\n";
        }
    }

  fprintf(outstream, "%s};\n\n", finalterm);
  fprintf(outstream, "#define NSYMBOLS (sizeof(%s) / sizeof (struct symtab_s))\n", SYMTAB_NAME);

  /* Close the CSV and symbol table files and exit */

  fclose(instream);
  fclose(outstream);
  return EXIT_SUCCESS;
}
