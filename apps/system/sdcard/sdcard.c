/****************************************************************************
 * apps/system/sdcard/sdcard.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#ifdef CONFIG_STM32_SDIO
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

FAR struct sdio_dev_s *sdio_initialize(int slotno);
void sdio_mediachange(FAR struct sdio_dev_s *dev, bool cardinslot);

// TODO get the structure out from the slot number
static FAR struct sdio_dev_s *sdio = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Create device device for the SDIO-based MMC/SD block driver */

static int sdcard_start(int slotno)
{
  int ret;

  /* First, get an instance of the SDIO interface */

  sdio = sdio_initialize(slotno);
  if (!sdio)
    {
      printf("SDIO: Failed to initialize slot %d\n", slotno);
      return -ENODEV;
    }

  printf("SDIO: Initialized slot %d\n", slotno);

  /* Now bind the SPI interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(slotno, sdio);
  if (ret != OK)
    {
      printf("SDIO: Failed to bind to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  printf("SDIO: Successfully bound to the MMC/SD driver\n");

  /* Then let's guess and say that there is a card in the slot.  I need to check to
   * see if the VSN board supports a GPIO to detect if there is a card in
   * the slot.
   */
  sdio_mediachange(sdio, true);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sdcard_main(int argc, char *argv[])
{
  int slotno = 0;

  if (argc >= 2) {

      /* The 3rd argument is expected to be a slot number, if given */

      if (argc==3)
        {
          slotno = atoi(argv[2]);
        }

      /* Commands */

      if (!strcmp(argv[1], "start"))
        {
          return sdcard_start(slotno);
        }
      else if (!strcmp(argv[1], "stop"))
        {
          fprintf(stderr, "Not implemented yet\n");
        }
      else if (!strcmp(argv[1], "insert"))
        {
          if (sdio)
            {
              sdio_mediachange(sdio, true);
              return OK;
            }
        }
      else if (!strcmp(argv[1], "eject"))
        {
          if (sdio)
            {
              sdio_mediachange(sdio, false);
              return OK;
            }
        }
      else if (!strcmp(argv[1], "status"))
        {
          printf("SDcard #%d Status:\n", slotno);
#ifndef CONFIG_MMCSD_HAVECARDDETECT
          printf("\t - Without SDcard detect capability\n");
#endif
          return 0;
        }
    }

  printf("%s: <start|stop|insert|eject|status> {slotno}\n", argv[0]);
  return -1;
}
