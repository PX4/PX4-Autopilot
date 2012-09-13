/****************************************************************************
 * apps/graphics/tiff/tiff_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <errno.h>

#include <apps/tiff.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* This is a simple unit test for the TIFF creation library at apps/graphic/tiff.
 * It is configured to work in the Linux user-mode simulation and has not been
 * tested in any other environment.  Since the example also depends on some
 * other logic to mount a file system, currently it will only work as an NSH
 * built-on, i.e., if the following is defined:
 *
 *   CONFIG_NSH_BUILTIN_APPS=y
 *   CONFIG_EXAMPLES_TIFF_BUILTIN=y
 *
 * Other configuration options:
 *
 *  CONFIG_EXAMPLES_TIFF_OUTFILE - Name of the resulting TIFF file
 *  CONFIG_EXAMPLES_TIFF_TMPFILE1/2 - Names of two temporaries files that
 *    will be used in the file creation.
 */

#ifndef CONFIG_EXAMPLES_TIFF_OUTFILE
#  define CONFIG_EXAMPLES_TIFF_OUTFILE "/tmp/result.tif"
#endif

#ifndef CONFIG_EXAMPLES_TIFF_TMPFILE1
#  define CONFIG_EXAMPLES_TIFF_TMPFILE1 "/tmp/tmpfile1.dat"
#endif

#ifndef CONFIG_EXAMPLES_TIFF_TMPFILE2
#  define CONFIG_EXAMPLES_TIFF_TMPFILE2 "/tmp/tmpfile2.dat"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiff_main
 *
 * Description:
 *   TIFF unit test.
 *
 ****************************************************************************/

int tiff_main(int argc, char *argv[])
{
  struct tiff_info_s info;
  uint8_t strip[3*256];
  uint8_t *ptr;
  int green;
  int blue;
  int ret;

  /* Configure the interface structure */

  memset(&info, 0, sizeof(struct tiff_info_s));
  info.outfile   = CONFIG_EXAMPLES_TIFF_OUTFILE;
  info.tmpfile1  = CONFIG_EXAMPLES_TIFF_TMPFILE1;
  info.tmpfile2  = CONFIG_EXAMPLES_TIFF_TMPFILE2;
  info.colorfmt  = FB_FMT_RGB24;
  info.rps       = 1;
  info.imgwidth  = 256;
  info.imgheight = 256;
  info.iobuffer  = (uint8_t *)malloc(300);
  info.iosize    = 300;

  /* Initialize the TIFF library */

  ret = tiff_initialize(&info);
  if (ret < 0)
    {
      printf("tiff_initialize() failed: %d\n", ret);
      exit(1);
    }

  /* Add each strip to the TIFF file */

  for (green = 0, ptr = strip; green < 256; green++)
    {
      ptr = strip;
      for (blue = 0; blue < 256; blue++)
        {
          *ptr++ = (green + blue) >> 1;
          *ptr++ = green;
          *ptr++ = blue;          
        }

      ret = tiff_addstrip(&info, strip);
      if (ret < 0)
        {
          printf("tiff_addstrip() #%d failed: %d\n", green, ret);
          exit(1);
        }
    }

  /* Then finalize the TIFF file */

  ret = tiff_finalize(&info);
  if (ret < 0)
    {
      printf("tiff_finalize() failed: %d\n", ret);
      exit(1);
    }
  return 0;
}
