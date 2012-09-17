/****************************************************************************
 * examples/lcdrw/lcdrw_main.c
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

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>

#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* Most of the NX configuration settings are probbably *not* needed by this
 * example.  But, presumeably you are using NX too and so the checks might
 * be good for you.
 */

#ifndef CONFIG_NX
#  error "CONFIG_NX must be defined to use this test"
#endif

#ifndef CONFIG_NX_LCDDRIVER
#  error "CONFIG_NX_LCDDRIVER must be defined to use this test"
#endif

#ifndef CONFIG_EXAMPLES_LCDRW_BPP
#  define CONFIG_EXAMPLES_LCDRW_BPP 16
#endif

#if CONFIG_EXAMPLES_LCDRW_BPP != 16
#  error "Currently only RGB565 is supported -- feel free to extend"
#endif

#ifdef CONFIG_NX_DISABLE_16BPP
#  error "CONFIG_NX_DISABLE_16BPP disables 16-bit support"
#endif

#ifndef CONFIG_EXAMPLES_LDCRW_DEVNO
#  define CONFIG_EXAMPLES_LDCRW_DEVNO 0
#endif

#ifndef CONFIG_EXAMPLES_LDCRW_XRES
#  define CONFIG_EXAMPLES_LDCRW_XRES 240
#endif

#ifndef CONFIG_EXAMPLES_LDCRW_YRES
#  define CONFIG_EXAMPLES_LDCRW_YRES 320
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lcdrw_instance_s
{
  /* LCD device handle and planeinfo */

  FAR struct lcd_dev_s *dev;
  struct lcd_planeinfo_s pinfo;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: lcdrw_initialize
 ****************************************************************************/

static inline int lcdrw_initialize(FAR struct lcdrw_instance_s *inst)
{
  int ret;

  /* Initialize the LCD device */

  printf("screens_initialize: Initializing LCD\n");
  ret = up_lcdinitialize();
  if (ret < 0)
    {
      fprintf(stderr, "screens_initialize: up_lcdinitialize failed: %d\n", -ret);
      return ret;
    }

  /* Get the device instance. */

  printf("Get LCD instance\n");
  inst->dev = up_lcdgetdev(CONFIG_EXAMPLES_LDCRW_DEVNO);
  if (!inst->dev)
    {
      fprintf(stderr, "up_lcdgetdev failed, devno=%d\n", CONFIG_EXAMPLES_LDCRW_DEVNO);
      return ret;
    }

  /* Turn the LCD on at 75% power.  This should not be necessary. */

  (void)inst->dev->setpower(inst->dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Get the planeinfo structure */

  ret = inst->dev->getplaneinfo(inst->dev, 0, &inst->pinfo);
  if (ret < 0)
    {
      fprintf(stderr, "getplaneinfo failed: %d\n", ret);
    }
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lcdrw_main
 ****************************************************************************/

int lcdrw_main(int argc, char *argv[])
{
  struct lcdrw_instance_s inst;
  nxgl_coord_t row;
  nxgl_coord_t col;
  uint16_t value;
  uint32_t offset;
  FAR uint16_t *ptr;
  int ret;

  /* Initialize the LCD driver */

  ret = lcdrw_initialize(&inst);
  if (ret < 0)
    {
      exit(1);
    }

  /* Loop, writing all possible values to the LCD */

  value = 0;
  for (row = 0; row < CONFIG_EXAMPLES_LDCRW_YRES; row++)
    {
      /* Create a dummy row.  The important thing is to try all
       * bit combinations in a predictable way.
       */
  
       ptr = (FAR uint16_t*)inst.pinfo.buffer;
       for (col = 0; col < CONFIG_EXAMPLES_LDCRW_XRES; col++)
         {
           *ptr++ = value++;
         }

      /* Write the row to the LCD */

      ret = inst.pinfo.putrun(row, 0, inst.pinfo.buffer,
                              CONFIG_EXAMPLES_LDCRW_XRES);
      if (ret < 0)
        {
          fprintf(stderr, "putrun failed: %d\n", ret);
          exit(1);
        }
    }

  /* Print a header */

  printf("       ");
  for (col = 0; col < 15; col++)
    {
      printf("---%x ", col);
    }
  printf("---f\n");

   /* Then read each line back from the LCD. */

  offset = 0;
  for (row = 0; row < CONFIG_EXAMPLES_LDCRW_YRES; row++)
    {
      /* Read the row */
 
      ret = inst.pinfo.getrun(row, 0, inst.pinfo.buffer,
                              CONFIG_EXAMPLES_LDCRW_XRES);
      if (ret < 0)
        {
          fprintf(stderr, "getrun failed: %d\n", ret);
          exit(1);
        }

      /* Then dump the row to the display */

      ptr = (FAR uint16_t*)inst.pinfo.buffer;
      for (col = 0; col < CONFIG_EXAMPLES_LDCRW_XRES; col++)
        {
          if ((offset & 15) == 0)
            {
              printf("%06x ", offset);
            }

          value = *ptr++;
          offset++;

          if ((offset & 15) == 0)
            {
              printf("%04x\n", value);
            }
          else
            {
              printf("%04x ", value);
            }
        }
    }
  fflush(stdout);

  return 0;
}

