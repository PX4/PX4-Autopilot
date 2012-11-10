/****************************************************************************
 * libc/unistd/lib_getopt.c
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#include <nuttx/config.h>

#include <stdbool.h>
#include <unistd.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

FAR char *optarg; /* Optional argument following option */
int optind = 1;   /* Index into argv */
int optopt = '?'; /* unrecognized option character */

/****************************************************************************
 * Private Variables
 ****************************************************************************/

static FAR char *g_optptr       = NULL;
static bool      g_binitialized = false;

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: getopt
 *
 * Description: getopt() parses command-line arguments.  Its arguments argc
 *   and argv are the argument count and array as passed to the main()
 *   function on program invocation.  An element of argv that starts with
 *   '-' is an option element. The characters of this element (aside from
 *   the initial '-') are option characters. If getopt() is called repeatedly,
 *   it returns successively each of the option characters from each of the
 *   option elements.
 *
 *   If getopt() finds another option character, it returns that character,
 *   updating the external variable optind and a static variable nextchar so
 *   that the next call to getopt() can resume the scan with the following
 *   option character or argv-element.
 *
 *   If there are no more option characters, getopt() returns -1. Then optind
 *   is the index in argv of the first argv-element that is not an option.
 *
 *   The 'optstring' argument is a string containing the legitimate option
 *   characters. If such a character is followed by a colon, this indicates
 *   that the option requires an argument.  If an argument is required for an
 *   option so getopt() places a pointer to the following text in the same
 *   argv-element, or the text of the following argv-element, in optarg.
 *
 *   NOTES:
 *   1. opterr is not supported and this implementation of getopt() never
 *      printfs error messages.
 *   2. getopt is NOT threadsafe!
 *   3. This version of getopt() does not reset global variables until
 *      -1 is returned.  As a result, your command line parsing loops
 *      must call getopt() repeatedly and continue to parse if other
 *      errors are returned ('?' or ':') until getopt() finally returns -1.
 *     (You can also set optind to -1 to force a reset).
 *
 * Return: If an option was successfully found, then getopt() returns the
 *   option character. If all command-line options have been parsed, then
 *   getopt() returns -1.  If getopt() encounters an option character that
 *   was not in optstring, then '?' is returned. If getopt() encounters an
 *   option with a missing argument, then the return value depends on the
 *   first character in optstring: if it is ':', then ':' is returned;
 *   otherwise '?' is returned.
 *
 ****************************************************************************/

int getopt(int argc, FAR char *const argv[], FAR const char *optstring)
{
  if (argv && optstring && argc > 1)
    {
      int noarg_ret = '?';
      char *optchar;

      /* The inital value of optind is 1.  If getopt() is called again in the
       * program, optind must be reset to some value <= 1.
       */

      if (optind < 1 || !g_binitialized)
        {
          optind         = 1;     /* Skip over the program name */
          g_optptr       = NULL;  /* Start at the beginning of the first argument */
          g_binitialized = true;  /* Now we are initialized */
        }

      /* If the first character of opstring s ':', then ':' is in the event of
       * a missing argument. Otherwise '?' is returned.
       */

      if (*optstring == ':')
        {
           noarg_ret = ':';
           optstring++;
        }

      /* Are we resuming in the middle, or at the end of a string of arguments?
       * g_optptr == NULL means that we are started at the beginning of argv[optind];
       * *g_optptr == \0 means that we are starting at the beginning of optind+1
       */

      while (!g_optptr || !*g_optptr)
        {
          /* We need to start at the beginning of the next argv. Check if we need
           * to increment optind
           */

          if (g_optptr)
            {
              /* Yes.. Increment it and check for the case where where we have
               * processed everything in the argv[] array.
               */

              optind++;
            }

          /* Check for the end of the argument list */

          g_optptr = argv[optind];
          if (!g_optptr)
            {
              /* There are no more arguments, we are finished */

              g_binitialized = false;
              return ERROR;
            }

          /* We are starting at the beginning of argv[optind].  In this case, the
           * first character must be '-'
           */

          if (*g_optptr != '-')
            {
              /* The argument does not start with '-', we are finished */

              g_binitialized = false;
              return ERROR;
            }

          /* Skip over the '-' */

          g_optptr++;
        }

        /* Special case handling of "-" and "-:" */

        if (!*g_optptr)
          {
             optopt = '\0'; /* We'll fix up g_optptr the next time we are called */
             return '?';
          }

        /* Handle the case of "-:" */

        if (*g_optptr == ':')
          {
            optopt = ':';
            g_optptr++;
            return '?';
          }

        /* g_optptr now points at the next option and it is not something crazy.
         * check if the option is in the list of valid options.
         */

        optchar = strchr(optstring, *g_optptr);
        if (!optchar)
          {
            /* No this character is not in the list of valid options */

            optopt = *g_optptr;
            g_optptr++;
            return '?';
          }

        /* Yes, the character is in the list of valid options.  Does it have an
         * required argument?
         */

        if (optchar[1] != ':')
          {
            /* No, no arguments. Just return the character that we found */

            g_optptr++;
            return *optchar;
          }

        /* Yes, it has a required argument.  Is the required argument
         * immediately after the command in this same argument?
         */

        if (g_optptr[1] != '\0')
          {
              /* Yes, return a pointer into the current argument */

              optarg = &g_optptr[1];
              optind++;
              g_optptr = NULL;
              return *optchar;
          }

        /* No.. is the optional argument the next argument in argv[] ? */

        if (argv[optind+1] && *argv[optind+1] != '-')
          {
            /* Yes.. return that */

            optarg = argv[optind+1];
            optind += 2;
            g_optptr = NULL;
            return *optchar;
          }

        /* No argument was supplied */

        optarg = NULL;
        optopt = *optchar;
        optind++;
        return noarg_ret;
    }

  g_binitialized = false;
  return ERROR;
}
