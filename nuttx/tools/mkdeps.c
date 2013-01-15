/****************************************************************************
 * tools/mkdeps.c
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

#include <sys/stat.h>

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <ctype.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX_BUFFER  (4096)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum slashmode_e
{
  MODE_FSLASH  = 0,
  MODE_BSLASH  = 1,
  MODE_DBLBACK = 2
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_cc       = NULL;
static char *g_cflags   = NULL;
static char *g_files    = NULL;
static char *g_altpath  = NULL;
static int   g_debug    = 0;
static bool  g_winnative = false;
#ifdef HAVE_WINPATH
static bool  g_winpath  = false;
static char *g_topdir   = NULL;
#endif

static char g_command[MAX_BUFFER];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

 /* MinGW does not seem to provide strtok_r */

#ifndef HAVE_STRTOK_R
static char *MY_strtok_r(char *str, const char *delim, char **saveptr)
{
  char *pbegin;
  char *pend = NULL;

  /* Decide if we are starting a new string or continuing from
   * the point we left off.
   */

  if (str)
    {
      pbegin = str;
    }
  else if (saveptr && *saveptr)
    {
      pbegin = *saveptr;
    }
  else
    {
      return NULL;
    }

  /* Find the beginning of the next token */

  for (;
       *pbegin && strchr(delim, *pbegin) != NULL;
       pbegin++);

  /* If we are at the end of the string with nothing
   * but delimiters found, then return NULL.
   */

  if (!*pbegin)
    {
      return NULL;
    }

  /* Find the end of the token */

  for (pend = pbegin + 1;
       *pend && strchr(delim, *pend) == NULL;
       pend++);

  /* pend either points to the end of the string or to
   * the first delimiter after the string.
   */

  if (*pend)
    {
      /* Turn the delimiter into a null terminator */

      *pend++ = '\0';
    }

  /* Save the pointer where we left off and return the
   * beginning of the token.
   */

  if (saveptr)
    {
      *saveptr = pend;
    }
  return pbegin;
}

#define strtok_r MY_strtok_r
#endif

static void append(char **base, char *str)
{
  char *oldbase;
  char *newbase;
  int alloclen;

  oldbase = *base;
  if (!oldbase)
    {
      newbase = strdup(str);      
      if (!newbase)
        {
          fprintf(stderr, "ERROR: Failed to strdup %s\n", str);
          exit(EXIT_FAILURE);
        }
    }
  else
    {
      alloclen = strlen(oldbase) + strlen(str) + 2;
      newbase = (char *)malloc(alloclen);
      if (!newbase)
        {
          fprintf(stderr, "ERROR: Failed to allocate %d bytes\n", alloclen);
          exit(EXIT_FAILURE);
        }
 
      snprintf(newbase, alloclen, "%s %s\n", oldbase, str);
      free(oldbase);
   }

  *base = newbase;
}

static void show_usage(const char *progname, const char *msg, int exitcode)
{
  if (msg)
    {
      fprintf(stderr, "\n");
      fprintf(stderr, "%s:\n", msg);
    }

  fprintf(stderr, "\n");
  fprintf(stderr, "%s  [OPTIONS] CC -- CFLAGS -- file [file [file...]]\n",
          progname);
  fprintf(stderr, "\n");
  fprintf(stderr, "Where:\n");
  fprintf(stderr, "  CC\n");
  fprintf(stderr, "    A variable number of arguments that define how to execute the compiler\n");
  fprintf(stderr, "  CFLAGS\n");
  fprintf(stderr, "    The compiler compilation flags\n");
  fprintf(stderr, "  file\n");
  fprintf(stderr, "    One or more C files whose dependencies will be checked.  Each file is expected\n");
  fprintf(stderr, "    to reside in the current directory unless --dep-path is provided on the command line\n");
  fprintf(stderr, "\n");
  fprintf(stderr, "And [OPTIONS] include:\n");
  fprintf(stderr, "  --dep-debug\n");
  fprintf(stderr, "    Enable script debug\n");
  fprintf(stderr, "  --dep-path <path>\n");
  fprintf(stderr, "    Do not look in the current directory for the file.  Instead, look in <path> to see\n");
  fprintf(stderr, "    if the file resides there.  --dep-path may be used multiple times to specify\n");
  fprintf(stderr, "    multiple alternative location\n");
  fprintf(stderr, "  --winnative\n");
  fprintf(stderr, "    By default, a POSIX-style environment is assumed (e.g., Linux, Cygwin, etc.)  This option is\n");
  fprintf(stderr, "    inform the tool that is working in a pure Windows native environment.\n");
#ifdef HAVE_WINPATH
  fprintf(stderr, "  --winpaths <TOPDIR>\n");
  fprintf(stderr, "    This option is useful when using a Windows native toolchain in a POSIX environment (such\n");
  fprintf(stderr, "    such as Cygwin).  In this case, will CC generates dependency lists using Windows paths\n");
  fprintf(stderr, "    (e.g., C:\\blablah\\blabla).  This switch instructs the script to use 'cygpath' to convert\n");
  fprintf(stderr, "    the Windows paths to Cygwin POSIXE paths.\n");
#endif
  fprintf(stderr, "  --help\n");
  fprintf(stderr, "    Shows this message and exits\n");
  exit(exitcode);
}

static void parse_args(int argc, char **argv)
{
  char *args = NULL;
  int argidx;

  /* Accumulate CFLAGS up to "--" */

  for (argidx = 1; argidx < argc; argidx++)
    {
      if (strcmp(argv[argidx], "--") == 0)
        {
          g_cc = g_cflags;
          g_cflags = args;
          args = NULL;
        }
      else if (strcmp(argv[argidx], "--dep-debug") == 0)
        {
          g_debug++;
        }
      else if (strcmp(argv[argidx], "--dep-path") == 0)
        {
          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --dep-path", EXIT_FAILURE);
            }

          if (args)
            {
              append(&args, argv[argidx]);
            }
          else
            {
              append(&g_altpath, argv[argidx]);              
            }
        }
      else if (strcmp(argv[argidx], "--winnative") == 0)
        {
          g_winnative = true;
        }
#ifdef HAVE_WINPATH
      else if (strcmp(argv[argidx], "--winpath") == 0)
        {
          g_winpath = true;
          if (g_topdir)
            {
              free(g_topdir);
            }

          argidx++;
          if (argidx >= argc)
            {
              show_usage(argv[0], "ERROR: Missing argument to --winpath", EXIT_FAILURE);
            }

          g_topdir = strdup(argv[argidx]);
        }
#endif
      else if (strcmp(argv[argidx], "--help") == 0)
        {
          show_usage(argv[0], NULL, EXIT_SUCCESS);
        }
      else
        {
          append(&args, argv[argidx]);
        }
    }

  /* The final thing accumulated is the list of files */

  g_files = args;

  /* If no paths were specified, then look in the current directory only */

  if (!g_altpath)
    {
      g_altpath = strdup(".");
    }

  if (g_debug)
    {
      fprintf(stderr, "SELECTIONS\n");
      fprintf(stderr, "  CC             : [%s]\n", g_cc ? g_cc : "(None)");
      fprintf(stderr, "  CFLAGS         : [%s]\n", g_cflags ? g_cflags : "(None)");
      fprintf(stderr, "  FILES          : [%s]\n", g_files ? g_files : "(None)");
      fprintf(stderr, "  PATHS          : [%s]\n", g_altpath ? g_altpath : "(None)");
#ifdef HAVE_WINPATH
      fprintf(stderr, "  Windows Paths  : [%s]\n", g_winpath ? "TRUE" : "FALSE");
      if (g_winpath)
        {
          fprintf(stderr, "  TOPDIR         : [%s]\n", g_topdir);
        }
#endif
      fprintf(stderr, "  Windows Native : [%s]\n", g_winnative ? "TRUE" : "FALSE");
    }

  /* Check for required paramters */

  if (!g_cc)
    {
      show_usage(argv[0], "ERROR: No compiler specified", EXIT_FAILURE);
    }

  if (!g_files)
    {
      /* Don't report an error -- this happens normally in some configurations */

      printf("# No files specified for dependency generataion\n");
      exit(EXIT_SUCCESS);
    }

#ifdef HAVE_WINPATH
  if (g_winnative && g_winpath)
    {
      show_usage(argv[0], "ERROR: Both --winnative and --winpapth makes no sense", EXIT_FAILURE);
    }
#endif
}

static void do_dependency(const char *file, char separator)
{
  static const char moption[] = " -M ";
  struct stat buf;
  char *alloc;
  char *altpath;
  char *path;
  char *lasts;
  int cmdlen;
  int pathlen;
  int filelen;
  int totallen;
  int ret;

  /* Copy the compiler into the command buffer */

  cmdlen = strlen(g_cc);
  if (cmdlen >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: Compiler string is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, g_cc);
      exit(EXIT_FAILURE);
    }

  strcpy(g_command, g_cc);

  /* Copy " -M " */

  cmdlen += strlen(moption);
  if (cmdlen >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: Option string is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, moption);
      exit(EXIT_FAILURE);
    }

  strcat(g_command, moption);

  /* Copy the CFLAGS into the command buffer */

  cmdlen += strlen(g_cflags);
  if (cmdlen >= MAX_BUFFER)
    {
      fprintf(stderr, "ERROR: CFLAG string is too long [%d/%d]: %s\n",
              cmdlen, MAX_BUFFER, g_cflags);
      exit(EXIT_FAILURE);
    }

  strcat(g_command, g_cflags);

  /* Add a space */

  g_command[cmdlen] = ' ';
  cmdlen++;
  g_command[cmdlen] = '\0';

  /* Make a copy of g_altpath. We need to do this because at least the version
   * of strtok_r above does modifie it.
   */

  alloc = strdup(g_altpath);
  if (!alloc)
    {
      fprintf(stderr, "ERROR: Failed to strdup paths\n");
      exit(EXIT_FAILURE);
    }

  altpath = alloc;

  /* Try each path.  This loop will continue until each path has been tried
   * (failure) or until stat() finds the file
   */

  while ((path = strtok_r(altpath, " ", &lasts)) != NULL)
    {
      /* Create a full path to the file */

      pathlen = strlen(path);
      totallen = cmdlen + pathlen;
      if (totallen >= MAX_BUFFER)
        {
          fprintf(stderr, "ERROR: Path is too long [%d/%d]: %s\n",
                  totallen, MAX_BUFFER, path);
          exit(EXIT_FAILURE);
        }

      strcpy(&g_command[cmdlen], path);

      if (g_command[totallen] != '\0')
        {
          fprintf(stderr, "ERROR: Missing NUL terminator\n");
          exit(EXIT_FAILURE);
        }

       if (g_command[totallen-1] != separator)
        {
          g_command[totallen] = separator;
          g_command[totallen+1] = '\0';
          pathlen++;
          totallen++;
        }

       filelen = strlen(file);
       totallen += filelen;
       if (totallen >= MAX_BUFFER)
         {
          fprintf(stderr, "ERROR: Path+file is too long [%d/%d]\n",
                  totallen, MAX_BUFFER);
          exit(EXIT_FAILURE);
         }
        
       strcat(g_command, file);

      /* Check that a file actually exists at this path */

      if (g_debug)
        {
          fprintf(stderr, "Trying path=%s file=%s fullpath=%s\n",
                  path, file, &g_command[cmdlen]);
        }

      ret = stat(&g_command[cmdlen], &buf);
      if (ret < 0)
        {
          altpath = NULL;
          continue;
        }

      if (!S_ISREG(buf.st_mode))
        {
          fprintf(stderr, "ERROR: File %s exists but is not a regular file\n",
                  &g_command[cmdlen]);
          exit(EXIT_FAILURE);
        }

      /* Okay.. we have.  Create the dependency.  One a failure to start the
       * compiler, system() will return -1;  Otherwise, the returned value
       * from the compiler is in WEXITSTATUS(ret).
       */
 
      ret = system(g_command);
#ifdef WEXITSTATUS
      if (ret < 0 || WEXITSTATUS(ret) != 0)
        {
          if (ret < 0)
            {
              fprintf(stderr, "ERROR: system failed: %s\n", strerror(errno));
            }
          else
            {
              fprintf(stderr, "ERROR: %s failed: %d\n", g_cc, WEXITSTATUS(ret));
            }

          fprintf(stderr, "       command: %s\n", g_command);
          exit(EXIT_FAILURE);
        }
#else
      if (ret < 0)
        {
          fprintf(stderr, "ERROR: system failed: %s\n", strerror(errno));
          fprintf(stderr, "       command: %s\n", g_command);
          exit(EXIT_FAILURE);
        }
#endif
 
      /* We don't really know that the command succeeded... Let's assume that it did */
 
      free(alloc);
      return;
    }

   printf("# ERROR: File \"%s\" not found at any location\n", file);
   exit(EXIT_FAILURE);
}

/* Convert a Cygwin path to a Windows path */

#ifdef HAVE_WINPATH
static char *cywin2windows(const char *str, const char *append, enum slashmode_e mode)
{
  static const char cygdrive[] = "/cydrive";
  const char *src = src;
  char *dest;
  char *newpath;
  char *allocpath = NULL;
  int srclen = strlen(str);
  int alloclen = 0;
  int drive = 0;
  int lastchar;

  /* Skip any leading whitespace */

  while (isspace(*str)) str++;

  /* Were we asked to append something? */

  if (append)
    {
      char *tmp;
 
      alloclen = sizeof(str) + sizeof(append) + 1;
      allocpath = (char *)malloc(alloclen);
      if (!allocpath)
        {
          fprintf(stderr, "ERROR: Failed to allocate %d bytes\n", alloclen);
          exit(EXIT_FAILURE);
        }

      snprintf(allocpath, alloclen, "%s/%s", str, append);
    }

  /* Looking for path of the form /cygdrive/c/bla/bla/bla */

  if (strcasecmp(src, cygdrive) == 0)
    {
      int cygsize = sizeof(cygdrive);
      if (src[cygsize] == '/')
        {
          cygsize++;
          srclen -= cygsize;
          src += cygsize;

          if (srclen <= 0)
            {
              fprintf(stderr, "ERROR: Unhandled path: \"%s\"\n", str);
              exit(EXIT_FAILURE);
            }

          drive = toupper(*src);
          if (drive < 'A' || drive > 'Z')
            {
              fprintf(stderr, "ERROR: Drive charager: \"%s\"\n", str);
              exit(EXIT_FAILURE);
            }

          srclen--;
          src++;
          alloclen = 2;
        }
    }

  /* Determine the size of the new path */

  alloclen += sizeof(src) + 1;
  if (mode == MODE_DBLBACK)
    {
      const char *tmpptr;
      for (tmpptr = src; *tmpptr; tmpptr++)
        {
          if (*tmpptr == '/') alloclen++;
        }
    }

  /* Allocate memory for the new path */

  newpath = (char *)malloc(alloclen);
  if (!newpath)
    {
      fprintf(stderr, "ERROR: Failed to allocate %d bytes\n", alloclen);
      exit(EXIT_FAILURE);
    }

  dest = newpath;

  /* Copy the drive character */

  if (drive)
    {
      *dest++ = drive;
      *dest++ = ':';
    }

  /* Copy each character from the source, making modifications for foward
   * slashes as required.
   */

  lastchar = '\0';
  for (; *src; src++)
    {
      if (mode != MODE_FSLASH && *src == '/')
        {
          if (lastchar != '/')
            {
              *dest++ = '\\';
              if (mode == MODE_DBLBACK)
                {
                  *dest++ = '\\';
                }
            }
        }
      else
        {
          *dest++ = *src;
        }

      lastchar = *src;
    }

  *dest++ = '\0';
  if (allocpath)
    {
      free(allocpath);
    }
  return dest;
}
#endif

#ifdef HAVE_WINPATH
static void do_winpath(char *file)
{
  /* The file is in POSIX format.  CC expects Windows format to generate the
   * dependencies, but GNU make expect the resulting dependencies to be back
   * in POSIX format.  What a mess!
   */

  char *path = cywin2windows(g_topdir, file, MODE_FSLASH);

  /* Then get the dependency and perform conversions on it to make it
   * palatable to the Cygwin make.
   */
#warning "Missing logic"

  free(path);
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv, char **envp)
{
  char *lasts;
  char *files;
  char *file;

  /* Parse command line parameters */

  parse_args(argc, argv);

  /* Then generate dependencies for each path on the command line.  NOTE
   * strtok_r will clobber the files list.  But that is okay because we are
   * only going to traverse it once.
   */

  files = g_files;
  while ((file = strtok_r(files, " ", &lasts)) != NULL)
    {
      /* Check if we need to do path conversions for a Windows-natvie tool
       * being using in a POSIX/Cygwin environment.
       */

#ifdef HAVE_WINPATH
      if (g_winpath)
        {
          do_winpath(file);
        }
      else
#endif
        {
          do_dependency(file, g_winnative ? '\\' : '/');
        }

      files = NULL;
    }

  return EXIT_SUCCESS;
}
