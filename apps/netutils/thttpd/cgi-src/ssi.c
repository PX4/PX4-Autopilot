/****************************************************************************
 * netutils/thttpd/cgi-src/ssi.c
 * Server-side-includes CGI program
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Derived from the file of the same name in the original THTTPD package:
 *
 *   Copyright © 1995 by Jef Poskanzer <jef@mail.acme.com>.
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <nuttx/regex.h>

#include "config.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define ST_GROUND    0
#define ST_LESSTHAN  1
#define ST_BANG      2
#define ST_MINUS1    3
#define ST_MINUS2    4

#define SF_BYTES     0
#define SF_ABBREV    1

#define DI_CONFIG    0
#define DI_INCLUDE   1
#define DI_ECHO      2
#define DI_FSIZE     3
#define DI_FLASTMOD  4

#define BUFFER_SIZE  512
#define TIMEFMT_SIZE 80
#define MAX_TAGS     32

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void read_file(FILE *instream, char *vfilename, char *filename);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char *g_url;
static char  g_timeformat[TIMEFMT_SIZE];
static char  g_iobuffer1[BUFFER_SIZE];
static char  g_iobuffer2[BUFFER_SIZE];
static char *g_tags[MAX_TAGS];
static int   g_sizefmt;
static struct stat g_sb;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void internal_error(char *reason)
{
  char *title = "500 Internal Error";

  (void)printf("\
<HTML><HEAD><TITLE>%s</TITLE></HEAD>\n\
<BODY><H2>%s</H2>\n\
Something unusual went wrong during a server-side-includes request:\n\
<BLOCKQUOTE>\n\
%s\n\
</BLOCKQUOTE>\n\
</BODY></HTML>\n", title, title, reason);
}

static void not_found(char *filename)
{
  char *title = "404 Not Found";

  (void)printf("\
<HTML><HEAD><TITLE>%s</TITLE></HEAD>\n\
<BODY><H2>%s</H2>\n\
The requested server-side-includes filename, %s,\n\
does not seem to exist.\n\
</BODY></HTML>\n", title, title, filename);
}

static void not_found2(char *directive, char *tag, char *filename)
{
  char *title = "Not Found";

  (void)printf("\
<HR><H2>%s</H2>\n\
The filename requested in a %s %s directive, %s,\n\
does not seem to exist.\n\
<HR>\n", title, directive, tag, filename);
}

static void not_permitted(char *directive, char *tag, char *val)
{
  char *title = "Not Permitted";

  (void)printf("\
<HR><H2>%s</H2>\n\
The filename requested in the %s %s=%s directive\n\
may not be fetched.\n\
<HR>\n", title, directive, tag, val);
}

static void unknown_directive(char *filename, char *directive)
{
  char *title = "Unknown Directive";

  (void)printf("\
<HR><H2>%s</H2>\n\
The requested server-side-includes filename, %s,\n\
tried to use an unknown directive, %s.\n\
<HR>\n", title, filename, directive);
}

static void unknown_tag(char *filename, char *directive, char *tag)
{
  char *title = "Unknown Tag";

  (void)printf("\
<HR><H2>%s</H2>\n\
The requested server-side-includes filename, %s,\n\
tried to use the directive %s with an unknown tag, %s.\n\
<HR>\n", title, filename, directive, tag);
}

static void unknown_value(char *filename, char *directive, char *tag, char *val)
{
  char *title = "Unknown Value";

  (void)printf("\
<HR><H2>%s</H2>\n\
The requested server-side-includes filename, %s,\n\
tried to use the directive %s %s with an unknown value, %s.\n\
<HR>\n", title, filename, directive, tag, val);
}

static int get_filename(char *vfilename, char *filename,
                         char *directive, char *tag, char *val, char *fn,
                         int fnsize)
{
  char *cp;
  int vl;
  int fl;

  /* Used for the various commands that accept a file name. These commands
   * accept two tags: virtual Gives a virtual path to a document on the
   * server. file Gives a pathname relative to the current directory. ../ 
   * cannot be used in this pathname, nor can absolute paths be used.
   */

  vl = strlen(vfilename);
  fl = strlen(filename);

  if (strcmp(tag, "virtual") == 0)
    {
      if (strstr(val, "../") != (char *)0)
        {
          not_permitted(directive, tag, val);
          return -1;
        }

      /* Figure out root using difference between vfilename and filename. */

      if (vl > fl || strcmp(vfilename, &filename[fl - vl]) != 0)
        {
          return -1;
        }
 
      if (fl - vl + strlen(val) >= fnsize)
        {
          return -1;
        }

      (void)strncpy(fn, filename, fl - vl);
      (void)strcpy(&fn[fl - vl], val);
    }
  else if (strcmp(tag, "file") == 0)
    {
      if (val[0] == '/' || strstr(val, "../") != (char *)0)
        {
          not_permitted(directive, tag, val);
          return -1;
        }
      if (fl + 1 + strlen(val) >= fnsize)
        {
          return -1;
        }

      (void)strcpy(fn, filename);
      cp = strrchr(fn, '/');
      if (cp == (char *)0)
        {
          cp = &fn[strlen(fn)];
          *cp = '/';
        }
      (void)strcpy(++cp, val);
    }
  else
    {
      unknown_tag(filename, directive, tag);
      return -1;
    }
  return 0;
}

static int check_filename(char *filename)
{
  static int inited = 0;
  static char *cgi_pattern;
#ifdef CONFIG_AUTH_FILE
  struct stat sb;
  char *dirname;
  char *authname;
  char *cp;
  int fnl;
  int r;
#endif

  if (!inited)
    {
      /* Get the cgi pattern. */

      cgi_pattern = getenv("CGI_PATTERN");
#ifdef CGI_PATTERN
      if (cgi_pattern == (char *)0)
        {
          cgi_pattern = CGI_PATTERN;
        }
#endif /* CGI_PATTERN */
      inited = 1;
    }

  /* ../ is not permitted. */

  if (strstr(filename, "../") !=NULL)
    {
      return 0;
    }

  /* Ensure that we are not reading a basic auth password file. */

#ifdef CONFIG_AUTH_FILE
  fnl = strlen(filename);
  if (strcmp(filename, CONFIG_AUTH_FILE) == 0 ||
      (fnl >= sizeof(CONFIG_AUTH_FILE) &&
       strcmp(&filename[fnl - sizeof(CONFIG_AUTH_FILE) + 1], CONFIG_AUTH_FILE) == 0 &&
       filename[fnl - sizeof(CONFIG_AUTH_FILE)] == '/'))
    {
      return 0;
    }

  /* Check for an auth file in the same directory.  We can't do an actual **
   * auth password check here because CGI programs are not given the **
   * authorization header, for security reasons.  So instead we just **
   * prohibit access to all auth-protected files.
   */

  dirname = strdup(filename);
  if (dirname == (char *)0)
    {
      /* out of memory */

      return 0;
    }

  cp = strrchr(dirname, '/');
  if (cp == (char *)0)
    {
      (void)strcpy(dirname, ".");
    }
  else
    {
      *cp = '\0';
    }

  authname = malloc(strlen(dirname) + 1 + sizeof(CONFIG_AUTH_FILE));
  if (!authname)
    {
      /* out of memory */

      free(dirname);
      return 0;
    }

  (void)sprintf(authname, "%s/%s", dirname, CONFIG_AUTH_FILE);
  r = stat(authname, &sb);

  free(dirname);
  free(authname);

  if (r == 0)
    {
      return 0;
    }
#endif /* CONFIG_AUTH_FILE */

  /* Ensure that we are not reading a CGI file. */

  if (cgi_pattern != (char *)0 && match(cgi_pattern, filename))
    {
      return 0;
    }
  return 1;
}

static void show_time(time_t t, int gmt)
{
  struct tm *tmP;

  if (gmt)
    {
      tmP = gmtime(&t);
    }
  else
    {
      tmP = localtime(&t);
    }

  if (strftime(g_iobuffer2, BUFFER_SIZE, g_timeformat, tmP) > 0)
    {
      (void)puts(g_iobuffer2);
    }
}

static void show_size(off_t size)
{
  switch (g_sizefmt)
    {
    case SF_BYTES:
      (void)printf("%ld", (long)size);  /* spec says should have commas */
      break;

    case SF_ABBREV:
      if (size < 1024)
        {
          (void)printf("%ld", (long)size);
        }
      else if (size < 1024)
        {
          (void)printf("%ldK", (long)size / 1024L);
        }
      else if (size < 1024 * 1024)
        {
          (void)printf("%ldM", (long)size / (1024L * 1024L));
        }
      else
        {
          (void)printf("%ldG", (long)size / (1024L * 1024L * 1024L));
        }
      break;
    }
}

static void do_config(FILE *instream, char *vfilename, char *filename,
                       char *directive, char *tag, char *val)
{
  /* The config directive controls various aspects of the file parsing. **
   * There are two valid tags: g_timeformat Gives the server a new format to
   * use when providing dates.  This is a string compatible with the
   * strftime library call. g_sizefmt Determines the formatting to be used
   * when displaying the size of a file.  Valid choices are bytes, for a
   * formatted byte count (formatted as 1,234,567), or abbrev for an
   * abbreviated version displaying the number of kilobytes or megabytes the 
   * file occupies.
   */

  if (strcmp(tag, "g_timeformat") == 0)
    {
      (void)strncpy(g_timeformat, val, TIMEFMT_SIZE - 1);
      g_timeformat[TIMEFMT_SIZE - 1] = '\0';
    }
  else if (strcmp(tag, "g_sizefmt") == 0)
    {
      if (strcmp(val, "bytes") == 0)
        {
          g_sizefmt = SF_BYTES;
        }
      else if (strcmp(val, "abbrev") == 0)
        {
          g_sizefmt = SF_ABBREV;
        }
      else
        {
          unknown_value(filename, directive, tag, val);
        }
    }
  else
    {
      unknown_tag(filename, directive, tag);
    }
}

static void do_include(FILE *instream, char *vfilename, char *filename,
                        char *directive, char *tag, char *val)
{
  FILE *instream2;
  int ret;

  /* Inserts the text of another document into the parsed document. */

  ret = get_filename(vfilename, filename, directive, tag, val, g_iobuffer1, BUFFER_SIZE);
  if (ret < 0)
    {
      return;
    }

  if (!check_filename(g_iobuffer1))
    {
      not_permitted(directive, tag, g_iobuffer1);
      return;
    }

  instream2 = fopen(g_iobuffer1, "r");
  if (instream2 == (FILE *) 0)
    {
      not_found2(directive, tag, g_iobuffer1);
      return;
    }

  if (strcmp(tag, "virtual") == 0)
    {
      if (strlen(val) <BUFFER_SIZE)
        {
          (void)strcpy(g_iobuffer2, val);
        }
      else
        {
          (void)strcpy(g_iobuffer2, g_iobuffer1);    /* same size, has to fit */
        }
    }
  else
    {
      if (strlen(vfilename) + 1 + strlen(val) < BUFFER_SIZE)
        {
          char *cp;
          (void)strcpy(g_iobuffer2, vfilename);
          cp = strrchr(g_iobuffer2, '/');
          if (cp == (char *)0)
            {
              cp = &g_iobuffer2[strlen(g_iobuffer2)];
              *cp = '/';
            }
          (void)strcpy(++cp, val);
        }
      else
        {
          (void)strcpy(g_iobuffer2, g_iobuffer1);    /* same size, has to fit */
        }
    }

  read_file(instream2, g_iobuffer2, g_iobuffer1);
  (void)fclose(instream2);
}

static void do_echo(FILE *instream, char *vfilename, char *filename, 
                     char *directive, char *tag, char *val)
{
  char *cp;

  /* Prints the value of one of the include variables.  Any dates are
   * printed subject to the currently configured g_timeformat.  The only valid
   * tag is var, whose value is the name of the variable you wish to echo.
   */

  if (strcmp(tag, "var") != 0)
    {
      unknown_tag(filename, directive, tag);
    }
  else
    {
      if (strcmp(val, "DOCUMENT_NAME") == 0)
        {
          /* The current filename. */

          (void)puts(filename);
        }
      else if (strcmp(val, "DOCUMENT_URI") == 0)
        {
          /* The virtual path to this file (such as /~robm/foo.shtml). */

          (void)puts(vfilename);
        }
      else if (strcmp(val, "QUERY_STRING_UNESCAPED") == 0)
        {
          /* The unescaped version of any search query the client sent. */

          cp = getenv("QUERY_STRING");
          if (cp != (char *)0)
            {
              (void)puts(cp);
            }
        }
      else if (strcmp(val, "DATE_LOCAL") == 0)
        {
          struct timeval tm;

          /* The current date, local time zone. */

          gettimeofday(&tm, NULL);
          show_time(tm.tv_sec, 0);
        }
      else if (strcmp(val, "DATE_GMT") == 0)
        {
          struct timeval tm;

          /* Same as DATE_LOCAL but in Greenwich mean time. */

          gettimeofday(&tm, NULL);
          show_time(tm.tv_sec, 1);
        }
      else if (strcmp(val, "LAST_MODIFIED") == 0)
        {
          /* The last modification date of the current document. */

          if (fstat(fileno(instream), &g_sb) >= 0)
            {
              show_time(g_sb.st_mtime, 0);
            }
        }
      else
        {
          /* Try an environment variable. */

          cp = getenv(val);
          if (cp == (char *)0)
            {
              unknown_value(filename, directive, tag, val);
            }
          else
            {
              (void)puts(cp);
            }
        }
    }
}

static void do_fsize(FILE *instream, char *vfilename, char *filename,
                      char *directive, char *tag, char *val)
{
  int ret;

  /* Prints the size of the specified file. */

  ret = get_filename(vfilename, filename, directive, tag, val, g_iobuffer1, BUFFER_SIZE);
  if (ret < 0)
    {
      return;
    }

  if (stat(g_iobuffer1, &g_sb) < 0)
    {
      not_found2(directive, tag, g_iobuffer1);
      return;
    }

  show_size(g_sb.st_size);
}

static void do_flastmod(FILE *instream, char *vfilename, char *filename,
                         char *directive, char *tag, char *val)
{
  int ret;

  /* Prints the last modification date of the specified file. */

  ret = get_filename(vfilename, filename, directive, tag, val, g_iobuffer1, BUFFER_SIZE);
  if (ret < 0)
    {
      return;
    }

  if (stat(g_iobuffer1, &g_sb) < 0)
    {
      not_found2(directive, tag, g_iobuffer1);
      return;
    }
  show_time(g_sb.st_mtime, 0);
}

static void parse(FILE *instream, char *vfilename, char *filename, char *str)
{
  char *directive;
  char *cp;
  int ntags;
  int dirn;
  int i;
  char *val;

  directive = str;
  directive += strspn(directive, " \t\n\r");

  ntags = 0;
  cp = directive;
  for (;;)
    {
      cp = strpbrk(cp, " \t\n\r\"");
      if (cp == (char *)0)
        {
          break;
        }

      if (*cp == '"')
        {
          cp = strpbrk(cp + 1, "\"");
          cp++;
          if (*cp == '\0')
            {
              break;
            }
        }

      *cp++ = '\0';
      cp += strspn(cp, " \t\n\r");
      if (*cp == '\0')
        {
          break;
        }

      if (ntags < MAX_TAGS)
        {
          g_tags[ntags++] = cp;
        }
    }

  if (strcmp(directive, "config") == 0)
    {
      dirn = DI_CONFIG;
    }
  else if (strcmp(directive, "include") == 0)
    {
      dirn = DI_INCLUDE;
    }
  else if (strcmp(directive, "echo") == 0)
    {
      dirn = DI_ECHO;
    }
  else if (strcmp(directive, "fsize") == 0)
    {
      dirn = DI_FSIZE;
    }
  else if (strcmp(directive, "flastmod") == 0)
    {
      dirn = DI_FLASTMOD;
    }
  else
    {
      unknown_directive(filename, directive);
      return;
    }

  for (i = 0; i < ntags; ++i)
    {
      if (i > 0)
        {
          putchar(' ');
        }

      val = strchr(g_tags[i], '=');
      if (val == (char *)0)
        {
          val = "";
        }
      else
        {
          *val++ = '\0';
        }

      if (*val == '"' && val[strlen(val) - 1] == '"')
        {
          val[strlen(val) - 1] = '\0';
          ++val;
        }

      switch (dirn)
        {
        case DI_CONFIG:
          do_config(instream, vfilename, filename, directive, g_tags[i], val);
          break;

        case DI_INCLUDE:
          do_include(instream, vfilename, filename, directive, g_tags[i], val);
          break;

        case DI_ECHO:
          do_echo(instream, vfilename, filename, directive, g_tags[i], val);
          break;

        case DI_FSIZE:
          do_fsize(instream, vfilename, filename, directive, g_tags[i], val);
          break;

        case DI_FLASTMOD:
          do_flastmod(instream, vfilename, filename, directive, g_tags[i], val);
          break;
        }
    }
}

static void slurp(FILE *instream, char *vfilename, char *filename)
{
  int state;
  int ich;
  int i;

  /* Now slurp in the rest of the comment from the input file. */

  i     = 0;
  state = ST_GROUND;
  while ((ich = getc(instream)) != EOF)
    {
      switch (state)
        {
        case ST_GROUND:
          if (ich == '-')
            {
              state = ST_MINUS1;
            }
          break;

        case ST_MINUS1:
          if (ich == '-')
            {
              state = ST_MINUS2;
            }
          else
            {
              state = ST_GROUND;
            }
          break;

        case ST_MINUS2:
          if (ich == '>')
            {
              g_iobuffer1[i - 2] = '\0';
              parse(instream, vfilename, filename, g_iobuffer1);
              return;
            }
          else if (ich != '-')
            {
              state = ST_GROUND;
            }
          break;
        }

      if (i < BUFFER_SIZE - 1)
        {
          g_iobuffer1[i++] = (char)ich;
        }
    }
}

static void read_file(FILE *instream, char *vfilename, char *filename)
{
  int ich;
  int state;

  /* Copy it to output, while running a state-machine to look for SSI
   * directives.
   */

  state = ST_GROUND;
  while ((ich = getc(instream)) != EOF)
    {
      switch (state)
        {
        case ST_GROUND:
          if (ich == '<')
            {
              state = ST_LESSTHAN;
              continue;
            }
          break;

        case ST_LESSTHAN:
          if (ich == '!')
            {
              state = ST_BANG;
              continue;
            }
          else
            {
              state = ST_GROUND;
              putchar('<');
            }
          break;

        case ST_BANG:
          if (ich == '-')
            {
              state = ST_MINUS1;
              continue;
            }
          else
            {
              state = ST_GROUND;
              (void)puts("<!");
            }
          break;

        case ST_MINUS1:
          if (ich == '-')
            {
              state = ST_MINUS2;
              continue;
            }
          else
            {
              state = ST_GROUND;
              (void)puts("<!-");
            }
          break;

        case ST_MINUS2:
          if (ich == '#')
            {
              slurp(instream, vfilename, filename);
              state = ST_GROUND;
              continue;
            }
          else
            {
              state = ST_GROUND;
              (void)puts("<!--");
            }
          break;
        }

      putchar((char)ich);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, char **argv)
{
  FILE *instream;
  char *script_name;
  char *path_info;
  char *path_translated;
  int err = 0;

  /* Default formats. */

  (void)strcpy(g_timeformat, "%a %b %e %T %Z %Y");
  g_sizefmt = SF_BYTES;

  /* The MIME type has to be text/html. */

  (void)puts("Content-type: text/html\n\n");

  /* Get the name that we were run as. */

  script_name = getenv("SCRIPT_NAME");
  if (!script_name)
    {
      internal_error("Couldn't get SCRIPT_NAME environment variable.");
      return 1;
    }

  /* Append the PATH_INFO, if any, to get the full URL. */

  path_info = getenv("PATH_INFO");
  if (!path_info)
    {
      path_info = "";
    }

  g_url = (char*)malloc(strlen(script_name) + strlen(path_info) + 1);
  if (!g_url)
    {
      internal_error("Out of memory.");
      return 2;
    }
  (void)sprintf(g_url, "%s%s", script_name, path_info);

  /* Get the name of the file to parse. */

  path_translated = getenv("PATH_TRANSLATED");
  if (!path_translated)
    {
      internal_error("Couldn't get PATH_TRANSLATED environment variable.");
      err = 3;
      goto errout_with_g_url;
    }

  if (!check_filename(path_translated))
    {
      not_permitted("initial", "PATH_TRANSLATED", path_translated);
      err = 4;
      goto errout_with_g_url;
    }

  /* Open it. */

  instream = fopen(path_translated, "r");
  if (!instream)
    {
      not_found(path_translated);
      err = 5;
      goto errout_with_g_url;
    }

  /* Read and handle the file. */

  read_file(instream, path_info, path_translated);

  (void)fclose(instream);

errout_with_g_url:
  free(g_url);
  return err;
}
