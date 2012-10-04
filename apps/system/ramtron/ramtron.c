/****************************************************************************
 * apps/system/ramtron/ramtron.c
 *
 *   Copyright (C) 2011 Uros Platise. All rights reserved.
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
 *
 *   Authors: Uros Platise <uros.platise@isotel.eu>
 *            Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>
#include <string.h>

#include <nuttx/spi.h>
#include <nuttx/mtd.h>

FAR struct mtd_dev_s *ramtron_initialize(FAR struct spi_dev_s *dev);

int ramtron_start(int spino)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  int retval;

  /* Get the SPI port */
  
  spi = up_spiinitialize(spino);
  if (!spi)
    {
      printf("RAMTRON: Failed to initialize SPI%d\n", spino);
      return -ENODEV;
    }

  printf("RAMTRON: Initialized SPI%d\n", spino);

  mtd = (struct mtd_dev_s *)ramtron_initialize(spi);
  if (!mtd)
    {
      printf("RAMTRON: Device not found\n");
      return -ENODEV;
    }

    printf("RAMTRON: FM25V10 of size 128 kB\n");
  //printf("RAMTRON: %s of size %d B\n", ramtron_getpart(mtd), ramtron_getsize(mtd) );

  retval = ftl_initialize(0, mtd);
  printf("RAMTRON: FTL Initialized (returns with %d)\n", retval);

  return OK;
}


int ramtron_main(int argc, char *argv[])
{
  int spino;
  
  if (argc == 3)
    {
      spino = atoi(argv[2]);
    
      if (!strcmp(argv[1], "start"))
        {
        return ramtron_start(spino);
        }
    }
  
  /* todo: write protect */

  printf("%s: <start> <spino>\n", argv[0]);
  return -1;
}
