/****************************************************************************
 * apps/system/install/install.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Author: Uros Platise <uros.platise@isotel.eu>
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
#include <nuttx/progmem.h>
#include <sys/stat.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACTION_INSTALL              0x01
#define ACTION_REMOVE               0x00
#define ACTION_REINSTALL            0x03
#define ACTION_INSUFPARAM           0x80

#define INSTALL_PROGRAMBLOCKSIZE    1024

/****************************************************************************
 * Private data
 ****************************************************************************/

static const char *install_help =
    "Installs XIP program into flash and creates a start-up script in the\n"
    "destination directory.\n\n"
    "Usage:\t%s [options] source-file.xip destination-directory\n\n"
    "Example:\n\t%s --stack 1024 /sdcard/demo.xip /usr/bin\n\n"
    "Options:\n"
    "\t--stack <required_stack_space>\n"
    "\t--priority <priority>\n"
    "\t--remove <dest-file>\tRemoves installed application\n"
    "\t--force\t\t\tReplaces existing installation\n"
    "\t--start <page>\t\tInstalls application at or after <page>\n"
    "\t--margin <pages>\tLeave some free space after the kernel (default 16)\n";

static const char *install_script_text =
    "# XIP stacksize=%x priority=%x size=%x\n";

static const char *install_script_exec =
    "exec 0x%x\n";

/****************************************************************************
 * Private functions
 ****************************************************************************/

static int install_getstartpage(int startpage, int pagemargin, int desiredsize)
{
  uint16_t page = 0, stpage = 0xffff;
  uint16_t pagesize = 0;
  int      maxlen = -1;
  int      maxlen_start = 0xffff;
  int      status;

  for (status=0, page=0; status >= 0; page++)
    {
      status   = up_progmem_ispageerased(page);
      pagesize = up_progmem_pagesize(page);

      /* Is this beginning of new free space section */

      if (status == 0)
        {
          if (stpage == 0xffff) stpage = page;
        }
      else if (status != 0)
        {
          if (stpage != 0xffff)
            {
              if ((page - stpage) > maxlen)
                {
                  if (maxlen==-1)
                    {
                      /* First time found sth? */

                      stpage += pagemargin;
                      maxlen = 0;
                    }

                  if(stpage < startpage)
                    {
                      stpage = startpage;
                    }

                  if (page > stpage)
                    {
                      maxlen = page - stpage;
                      maxlen_start = stpage;
                    }

                  if (maxlen*pagesize >= desiredsize)
                    {
                      /* printf("Found page at %d ... %d\n", stpage, page); */
                      return maxlen_start*pagesize;
                    }
                }

              stpage = 0xffff;
            }
        }
    }

  /* Requested space is not available */

  return -1;
}

static int install_programflash(int startaddr, const char *source)
{
  int  status;
  int  count;
  int  totalsize = 0;
  char *buf;
  FILE *fp;

  if ((buf = malloc(INSTALL_PROGRAMBLOCKSIZE)) == NULL)
    {
      return -ENOMEM;
    }

  if ((fp = fopen(source, "r")))
    {
      do
        {
          count = fread(buf, 1, INSTALL_PROGRAMBLOCKSIZE, fp);

          if ((status = up_progmem_write(startaddr, buf, count)) < 0)
            {
              totalsize = status;
              break;
            }

          startaddr += count;
          totalsize += count;
        }
      while(count);
    }
  else
    {
      totalsize = -errno;
    }

  fclose(fp);
  free(buf);

  return totalsize;
}

static void install_getscriptname(char *scriptname, const char *progname, const char *destdir)
{
  const char * progonly;

  /* I.e. as /usr/bin */

  strcpy(scriptname, destdir);

  /* extract from i.e. /sdcard/demo -> /demo, together with / */

  progonly = strrchr(progname, '/');
  strcat(scriptname, progonly);
}

static int install_getprogsize(const char *progname)
{
  struct stat fileinfo;

  if (stat(progname, &fileinfo) < 0)
    {
      return -1;
    }

  return fileinfo.st_size;
}

static int install_alreadyexists(const char *scriptname)
{
  FILE *fp;

  if ((fp = fopen(scriptname, "r")) == NULL)
    {
      return 0;
    }

  fclose(fp);
    return 1;
}

static int install_createscript(int addr, int stacksize, int progsize,
                                int priority, const char *scriptname)
{
  FILE *fp;

  if ((fp = fopen(scriptname, "w+")) == NULL)
    {
      return -errno;
    }

  fprintf(fp, install_script_text, stacksize, priority, progsize);
  fprintf(fp, install_script_exec, addr);

  fflush(fp);
  fclose(fp);

  return 0;
}

static int install_getlasthexvalue(FILE *fp, char delimiter)
{
  char buf[128];
  char *p;

  if (fgets(buf, 127, fp))
    {
      if ((p = strrchr(buf, delimiter)))
        {
          return strtol(p+1, NULL, 16);
        }
    }

  return -1;
}

static int install_remove(const char *scriptname)
{
  FILE    *fp;
  int      progsize, addr, freedsize;
  uint16_t page;
  int      status = 0;

  /* Parse script */

  if ((fp = fopen(scriptname, "r")))
    {
      progsize  = install_getlasthexvalue(fp,'=');
      addr      = install_getlasthexvalue(fp,' ');
      freedsize = progsize;
    }
  else
    {
      return -errno;
    }

  fclose(fp);

  /* Remove pages */

  if (progsize <= 0 || addr <= 0)
    {
      return -EIO;
    }

  do
    {
      if ((page = up_progmem_getpage(addr)) < 0)
        {
          status = -page;
          break;
        }

      if (up_progmem_erasepage(page) < 0)
        {
          status = -page;
          break;
        }

      addr += up_progmem_pagesize(page);
      progsize -= up_progmem_pagesize(page);

    }
  while(progsize > 0);

  if (status < 0)
    {
      return status;
    }

  /* Remove script file */

  if (unlink(scriptname) < 0)
    {
      return -errno;
    }

  return freedsize;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

int install_main(int argc, char *argv[])
{
  int i;
  int progsize;
  int scrsta;
  int stacksize       = 4096;
  int priority        = SCHED_PRIORITY_DEFAULT;
  int pagemargin      = 16;
  int startpage       = 0;
  int startaddr       = 0;
  int action          = ACTION_INSTALL;
  char scriptname[128];

  /* Supported? */

  if (!up_progmem_isuniform())
    {
      fprintf(stderr, "Error: install supports uniform organization only.\n");
      return -1;
    }

  /* Parse arguments */

  for (i=1; i<argc; i++)
    {
      if (argv[i][0]=='-' && argv[i][1]=='-' && i<=argc)
        {
          if (strcmp(argv[i]+2, "stack")==0)
            {
              stacksize = atoi(argv[++i]);
            }
          else if (strcmp(argv[i]+2, "priority")==0)
            {
              priority = atoi(argv[++i]);
            }
          else if (strcmp(argv[i]+2, "start")==0)
            {
              startpage = atoi(argv[++i]);
            }
          else if (strcmp(argv[i]+2, "margin")==0)
            {
              pagemargin = atoi(argv[++i]);
            }
          else if (strcmp(argv[i]+2, "remove")==0)
            {
              action = ACTION_REMOVE;
            }
          else if (strcmp(argv[i]+2, "force")==0)
            {
              action = ACTION_REINSTALL;
            }
          else fprintf(stderr, "Unknown option: %s\n", argv[i]);
        }
      else
        {
          break;
        }
    }

  /* Do the job */

  switch(action & 1)
    {
      case ACTION_REMOVE:
        if (i > argc-1)
          {
            action = ACTION_INSUFPARAM;
            break;  /* are there sufficient parameters */
          }

        if ((scrsta=install_remove(argv[i])) < 0)
          {
            fprintf(stderr, "Could not remove program: %s\n", strerror(-scrsta));
            return -1;
          }

        printf("Removed %s and freed %d bytes\n", argv[i], scrsta);
          return 0;

      case ACTION_INSTALL:
        if (i > argc-2)
          {
            action = ACTION_INSUFPARAM;
            break;  /* are there sufficient parameters */
          }

        install_getscriptname(scriptname, argv[i], argv[i+1]);

        /* script-exists? */

        if (install_alreadyexists(scriptname) == 1)
          {
            if (action != ACTION_REINSTALL)
              {
                fprintf(stderr, "Program with that name already exists.\n");
                return -EEXIST;
              }

            if ((scrsta = install_remove(scriptname)) < 0)
              {
                fprintf(stderr, "Could not remove program: %s\n", strerror(-scrsta));
                return -1;
              }

            printf("Replacing %s\n", scriptname);
          }

        startaddr = install_getstartpage(startpage, pagemargin, install_getprogsize(argv[i]));
        if (startpage < 0)
          {
            fprintf(stderr, "Not enough memory\n");
            return -ENOMEM;
          }

        if ((progsize = install_programflash(startaddr, argv[i])) <= 0)
          {
            fprintf(stderr, "Error writing program memory: %s\n"
                    "Note: Flash pages are not released, so you may try again and program will be\n"
                    "      written in other pages.\n", strerror(-progsize));
            return -EIO;
          }

        if ((scrsta = install_createscript(startaddr, stacksize, progsize,
                                             priority, scriptname)) < 0)
          {
            fprintf(stderr, "Error writing program script at %s: %s\n",
                    argv[i+1], strerror(-scrsta));
            return -EIO;
          }

        printf("Installed application of size %d bytes to program memory [%xh - %xh].\n",
                progsize, startaddr, startaddr + progsize);
        return 0;
    }

  fprintf(stderr, install_help, argv[0], argv[0]);
  return -1;
}
