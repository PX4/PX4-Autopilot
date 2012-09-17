/***********************************************************************
 * xflat/tools/mknxflat.c
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Modified from ldelflib (see http://xflat.org):
 *
 *   Copyright (c) 2002, 2006, Cadenux, LLC.  All rights reserved.
 *   Copyright (c) 2002, 2006, Gregory Nutt.  All rights reserved.
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

/***********************************************************************
 * Included Files
 ***********************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <bfd.h>

#include "nxflat.h"
#include "arch/arch.h"

/***********************************************************************
 * Definitions
 ***********************************************************************/

#define dbg(format, arg...) \
    if (verbose) printf(format, ## arg)

#define BSF_GLOBL_FUNC (BSF_GLOBAL|BSF_FUNCTION)
#define BSF_WEAK_FUNC  (BSF_WEAK|BSF_FUNCTION)
#define BSF_DEFINED    (BSF_LOCAL|BSF_GLOBAL)

#define IS_GLOBL_FUNC(x) ((((x)->flags)&(BSF_GLOBL_FUNC))==(BSF_GLOBL_FUNC))
#define IS_WEAK_FUNC(x)  ((((x)->flags)&(BSF_WEAK_FUNC))==(BSF_WEAK_FUNC))
#define IS_DEFINED(x)    ((((x)->flags)&(BSF_DEFINED))!=0)
#define IS_OBJECT(x)     ((((x)->flags)&(BSF_OBJECT))!=0)
#define IS_WEAK(x)       ((((x)->flags)&(BSF_WEAK))!=0)

#define MAX_EXPORT_NAMES    1024

/***********************************************************************
 * Private Types
 ***********************************************************************/

typedef int (*symfunc_type) (asymbol * sym, void *arg);
typedef int (*namefunc_type) (const char *name, void *arg);

/***********************************************************************
 * Private Variables
 ***********************************************************************/

/* Command line settings (counters but treated like booleans) */

static int verbose = 0;
static int weak_imports = 0;
static int dsyms = 0;

/* Characteristics of things */
static int calls_nonreturning_functions = 0;

/* Sizes of things */

static long number_of_symbols = 0;
static long number_undefined = 0;

/* Names of things */

static const char *program_name = NULL;
static const char *bfd_filename = NULL;
static const char *out_filename = NULL;

/* The symbol table. */

static asymbol **symbol_table = NULL;

/* handle to included file */

static FILE *include_stream = NULL;
static char token[1024];

static int counter;

/***********************************************************************
 * Private constant data
 ***********************************************************************/

/* All of the strings defining the generated file are here: */

#include "arch/dyncall_skeleton.def"

static const char dyn_symbol_prefix[] = "__dyn";
#define DYN_SYMBOL_PREFIX_LEN 5

/* This is the list of names of libc and libpthread functions that
 * do not return.  These may require some special handling -- at a
 * minimum, they must tie up resources that can only be released
 * when the function returns.
 */

static const char *const nonreturners[] = {
  "abort",                      /* Never returns */
  "exit",                       /* Never returns */
  "_exit",                      /* Never returns */
  "longjmp",                    /* Never returns */
  "_longjmp",                   /* Never returns */
  "pthread_exit",               /* Never returns */
  "siglongjmp",                 /* Never returns */
  NULL                          /* End of list */
};

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * show_usage
 ***********************************************************************/

static void show_usage(void)
{
  fprintf(stderr, "Usage: %s [options] <bfd-filename>\n\n", program_name);
  fprintf(stderr, "Where options are one or more of the following.  Note\n");
  fprintf(stderr, "that a space is always required between the option and\n");
  fprintf(stderr, "any following arguments.\n\n");
  fprintf(stderr, "  -d Use dynamic symbol table. [symtab]\n");
  fprintf(stderr, "  -f <cmd-filename>\n");
  fprintf(stderr, "      Take next commands from <cmd-filename> [cmd-line]\n");
  fprintf(stderr, "  -o <out-filename>\n");
  fprintf(stderr, "     Output to <out-filename> [stdout]\n");
  fprintf(stderr, "  -v Verbose output [no output]\n");
  fprintf(stderr, "  -w Import weakly declared functions, i.e., weakly\n");
  fprintf(stderr, "     declared functions are expected to be provided at\n");
  fprintf(stderr, "     load-time [not imported]\n");
  fprintf(stderr, "\n");
  exit(1);
}

/***********************************************************************
 * get_symbols
 ***********************************************************************/

static asymbol **get_symbols(bfd * abfd, long *num)
{
  long storage_needed;
  asymbol **symbol_table;
  long number_of_symbols;

  if (dsyms)
    storage_needed = bfd_get_dynamic_symtab_upper_bound(abfd);
  else
    storage_needed = bfd_get_symtab_upper_bound(abfd);

  if (storage_needed < 0)
    abort();

  if (storage_needed == 0)
    return NULL;

  symbol_table = (asymbol **) malloc(storage_needed);

  if (dsyms)
    number_of_symbols = bfd_canonicalize_dynamic_symtab(abfd, symbol_table);
  else
    number_of_symbols = bfd_canonicalize_symtab(abfd, symbol_table);
  if (number_of_symbols < 0)
    abort();

  *num = number_of_symbols;
  return symbol_table;
}

/***********************************************************************
 * traverse_undefined_functions
 ***********************************************************************/

static int traverse_undefined_functions(void *arg, symfunc_type fn)
{
  int i;

  for (i = 0; i < number_of_symbols; i++)
    {
      /* Check if it is undefined and not an object. I have found that symbol
       * typing can be misleading: Imported functions are not marked as
       * BSF_FUNCTION; weakly defined objects are listed as undefined objects.
       * Conclusion: We will error if a object is truly undefined! */

      if ((symbol_table[i]->value == 0) &&
          (!IS_DEFINED(symbol_table[i])) && (!IS_OBJECT(symbol_table[i])))
        {
          /* Is is imported as a "weak" symbol? If so, we will process the
           * symbol only if we were requested to do so from the command line. */

          if ((!IS_WEAK(symbol_table[i])) || (weak_imports > 0))
            {
              /* Yes, process the symbol */

              if (fn(symbol_table[i], arg) != 0)
                {
                  /* If the function returns a non-zero value, then we
                   * terminate the traversal and return a non-zero value also. */

                  return 1;
                }
            }
        }
    }

  /* Return 0 meaning that all undefined symbols were examined successfully. */

  return 0;
}

/***********************************************************************
 * put_string
 ***********************************************************************/

static void put_string(int fd, const char *string)
{
  ssize_t bytes_available = strlen(string);
  ssize_t bytes_written = write(fd, string, bytes_available);
  if (bytes_written < 0)
    {
      fprintf(stderr,
              "Failed to write %ld bytes of string to output, errno=%d\n",
              (long)bytes_available, errno);
      exit(5);
    }
  else if (bytes_written != bytes_available)
    {
      fprintf(stderr, "Only wrote %ld of %ld bytes of string to output\n",
              (long)bytes_written, (long)bytes_available);
      exit(6);
    }
}

/***********************************************************************
 * does_not_return_name/sym
 ***********************************************************************/

static int does_not_return_name(const char *func_name)
{
  int i;

  /* Check every name in the list of (usually) non-returning function */

  for (i = 0; nonreturners[i] != NULL; i++)
    {
      /* Is this function name in the list */

      if (strcmp(func_name, nonreturners[i]) == 0)
        {
          /* Yes, return true now. */

          return 1;
        }
    }

  /* Its not in the list, return false */

  return 0;
}

static int does_not_return_sym(asymbol * sym, void *arg)
{
  const char *func_name = sym->name;
  if (func_name)
    return does_not_return_name(func_name);
  else
    return 0;
}

/***********************************************************************
 * check_for_nonreturning_functions
 ***********************************************************************/

static void check_for_nonreturning_functions(void)
{
  calls_nonreturning_functions =
    traverse_undefined_functions(NULL, does_not_return_sym);
}

/***********************************************************************
 * count_undefined
 ***********************************************************************/

static int count_undefined(asymbol * sym, void *arg)
{
  number_undefined++;
  return 0;
}

/***********************************************************************
 * put_dynimport_decl
 ***********************************************************************/

static int put_dynimport_decl(asymbol * sym, void *arg)
{
  char dynimport_decl[1024];
  const char *func_name = sym->name;
  int fd = (int)arg;

  /* Put the declaration for the dynamic info structure */
  if (func_name)
    {
      sprintf(dynimport_decl, dynimport_decl_format,
              MKINFODECLARGS(func_name, counter));
      put_string(fd, dynimport_decl);
      counter++;
    }
  return 0;
}

/***********************************************************************
 * put_dynimport_array
 ***********************************************************************/

static int put_dynimport_array(asymbol * sym, void *arg)
{
  char dynimport_array[1024];
  const char *func_name = sym->name;
  int fd = (int)arg;

  /* Create the dynimport_array */

  if (func_name)
    {
      sprintf(dynimport_array, dynimport_array_format,
              MKINFOARGS(func_name, counter));
      put_string(fd, dynimport_array);
      counter++;
    }
  return 0;
}

/***********************************************************************
 * put_nxflat_import
 ***********************************************************************/

static int put_nxflat_import(asymbol * sym, void *arg)
{
  char thunk[4096];
  const char *func_name = sym->name;
  int fd = (int)arg;

  if (func_name)
    {
      /* Create the thunk */

      if (does_not_return_name(func_name) != 0)
        {
          /* The special case for functions that may not return */

          sprintf(thunk, nonreturning_dyncall_format, MKCALLARGS(func_name, counter));
        }
      else
        {
          /* The normal case */

          sprintf(thunk, dyncall_format, MKCALLARGS(func_name, counter));
        }

      put_string(fd, thunk);
      counter++;
    }
  return 0;
}

/***********************************************************************
 * put_all_nxflat_import
 ***********************************************************************/

static void put_all_nxflat_import(int fd)
{
  if (number_undefined > 0)
    {
      /* Put all of the declarations for the dynimport structures together. */

      put_string(fd, dynimport_decl_prologue);
      counter = 0;
      (void)traverse_undefined_functions((void *)fd, put_dynimport_decl);

      /* Put all of the dynimport structures together as an array */

      put_string(fd, dynimport_array_prologue);
      counter = 0;
      (void)traverse_undefined_functions((void *)fd, put_dynimport_array);
      put_string(fd, dynimport_array_epilogue);

      /* Put all of the dyncall logic together */

      put_string(fd, dyncall_decl_prologue);
      counter = 0;
      (void)traverse_undefined_functions((void *)fd, put_nxflat_import);
    }
}

/***********************************************************************
 * put_import_name
 ***********************************************************************/

static int put_import_name(asymbol * sym, void *arg)
{
  char import_name[512];
  const char *func_name = sym->name;
  int fd = (int)arg;

  /* Create the import_name */

  if (func_name)
    {
      sprintf(import_name, import_name_strtab_format,
              MKIMPSTRTABARG(func_name, counter));
      put_string(fd, import_name);
      counter++;
    }
  return 0;
}

/***********************************************************************
 * put_import_name_strtab
 ***********************************************************************/

static void inline put_import_name_strtab(int fd)
{
  if (number_undefined > 0)
    {
      counter = 0;
      put_string(fd, import_name_strtab_prologue);
      (void)traverse_undefined_functions((void *)fd, put_import_name);
    }
}

/***********************************************************************
 * put_file_epilogue
 ***********************************************************************/

static void inline put_file_epilogue(int fd)
{
  put_string(fd, file_epilogue);
}

/***********************************************************************
 * put_file_prologue
 ***********************************************************************/

static void inline put_file_prologue(int fd)
{
  put_string(fd, file_prologue);

  if (number_undefined > 0)
    {
      put_string(fd, import_prologue);
    }
}

/***********************************************************************
 * get_file_token
 ***********************************************************************/

#define ISSPACE(c) \
((c==' ')||(c=='\f')||(c=='\n')||(c=='\r')||(c=='\t')||(c=='\v'))

#define ISTERMINATOR(c) (ISSPACE(c)||(c==EOF))

static int get_file_token(FILE * in_stream)
{
  int i;
  int c;

  /* Skip over leading whitespace */

  do
    c = getc(in_stream);
  while ISSPACE
  (c);

  if (c == EOF)
    return EOF;

  /* Add the token to the buffer. Copy characters until the buffer is full, or 
   * a terminator is encountered. */

  for (i = 0; ((i < 1023) && !ISTERMINATOR(c)); i++, c = getc(in_stream))
    {
      token[i] = (char)c;
    }

  /* Handle the string truncation case. */

  token[i] = '\0';
  while (!ISTERMINATOR(c))
    c = getc(in_stream);

  /* Return success.  On next entry, we will get the next character after the
   * terminator.  If the terminator was EOF, we should get EOF again. */

  return 0;
}

/***********************************************************************
 * get_token
 ***********************************************************************/

static char *get_token(int *argno, int argc, char **argv)
{
  char *retval = NULL;

  if (include_stream)
    {
      if (get_file_token(include_stream) == EOF)
        {
          fclose(include_stream);
          include_stream = NULL;
          retval = get_token(argno, argc, argv);
        }
      else
        {
          retval = strdup(token);
        }
    }
  else if (*argno >= argc)
    {
      retval = NULL;
    }
  else
    {
      retval = argv[*argno];
      (*argno)++;
    }
  return retval;
}

/***********************************************************************
 * get_arg
 ***********************************************************************/

static char *get_arg(int *argno, int argc, char **argv)
{
  char *retval;

  /* Get the next argument */

  retval = get_token(argno, argc, argv);
  if ((retval == NULL) || (retval[0] == '-'))
    {
      fprintf(stderr, "Option requires an argument\n\n");
      show_usage();
    }
  return retval;
}

/***********************************************************************
 * get_opt
 ***********************************************************************/

static int get_opt(int *argno, int argc, char **argv)
{
  char *opt;
  int len;
  int retval = -1;

  /* Get the next argument */

  opt = get_token(argno, argc, argv);
  if (opt != NULL)
    {
      /* It must be of length 2 and start with a '-' */

      len = strlen(opt);
      if ((len == 2) && (opt[0] == '-'))
        {
          retval = (int)opt[1];
        }
      else
        {
          fprintf(stderr, "%s Unrecognized option\n\n", opt);
          show_usage();
        }
    }
  return retval;
}

/***********************************************************************
 * parse_args
 ***********************************************************************/

static void parse_args(int argc, char **argv)
{
  int argno = 1;
  int opt;

  /* Save our name (for show_usage) */

  program_name = argv[0];

  if (argc < 2)
    {
      fprintf(stderr, "ERROR:  Missing required arguments\n\n");
      show_usage();
    }

  /* Get the name of the input BFD file. This is always the last thing in the
   * argument list.  We decrement argc so that the parsing logic will not look
   * at it. */

  bfd_filename = argv[argc - 1];
  argc--;

  /* Get miscellaneous options from the command line. */

  while ((opt = get_opt(&argno, argc, argv)) != -1)
    {
      switch (opt)
        {
        case 'd':
          dsyms++;
          break;

        case 'f':
          {
            char *filename = get_arg(&argno, argc, argv);
            if (include_stream)
              {
                fprintf(stderr, "Cannot use -f from within a cmd-file\n\n");
                show_usage();
              }
            else
              {
                include_stream = fopen(filename, "r");
                if (!include_stream)
                  {
                    fprintf(stderr, "Could not open cmd-file %s\n\n", filename);
                    show_usage();
                  }
              }
          }
          break;

        case 'o':
          out_filename = get_arg(&argno, argc, argv);
          break;

        case 'v':
          verbose++;
          break;

        case 'w':
          weak_imports++;
          break;

        default:
          fprintf(stderr, "%s Unknown option\n\n", argv[0]);
          show_usage();
          break;
        }
    }
}

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************
 * main
 ***********************************************************************/

int main(int argc, char **argv, char **envp)
{
  bfd *bf;
  int out_fd = 0;

  /* Get the input arguments */

  parse_args(argc, argv);

  /* Make sure that we can option the BFD file */

  dbg("Opening BFD file: %s\n", bfd_filename);
  if (!(bf = bfd_openr(bfd_filename, 0)))
    {
      fprintf(stderr, "Failed to open BFD file: %s, errno=%d\n",
              bfd_filename, errno);
      exit(2);
    }

  dbg("Checking format\n");
  if (bfd_check_format(bf, bfd_object) == 0)
    {
      fprintf(stderr, "BFD file %s is not an object file\n", bfd_filename);
      exit(3);
    }

  dbg("Loading symbol table from BFD file %s\n", bfd_filename);
  symbol_table = get_symbols(bf, &number_of_symbols);

  /* Count the number of undefined function symbols */

  (void)traverse_undefined_functions(NULL, count_undefined);

  /* Check if the module calls any non-returning functions (like exit).  These
   * will require some additional setup. */

  check_for_nonreturning_functions();

  /* Make sure that we can open the output file if one is specified. If no
   * out_filename is specified, we'll use stdout. */

  if (out_filename)
    {
      out_fd = open(out_filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);
      if (out_fd < 0)
        {
          fprintf(stderr, "Failed to open output file: %s, errno=%d\n",
                  out_filename, errno);
          exit(4);
        }
    }

  /* Output the thunk file in three pieces: 1. The constant file prologue 2.
   * Library path information (if any) 3. Library file name information (if
   * any) 4. Exported symbole information (if any) 5. Imported symbole
   * information (if any) 6. The constant file epilogue. */

  put_file_prologue(out_fd);
  put_import_name_strtab(out_fd);
  put_all_nxflat_import(out_fd);
  put_file_epilogue(out_fd);

  if (out_fd > 0)
    close(out_fd);
  exit(0);
}
