/****************************************************************************
 * pload.c
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include "keywords.h"
#include "pedefs.h"
#include "pofflib.h"
#include "perr.h"
#include "pexec.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct pexec_s *pload(const char *filename, paddr_t varsize, paddr_t strsize)
{
  struct pexec_attr_s attr;
  struct pexec_s *st;
  poffHandle_t phandle;
  FILE        *exe;
  uint16_t     err;
  uint8_t      ftype;
  uint8_t      farch;

  /* Create a handle to contain the executable data */

  phandle = poffCreateHandle();
  if (phandle == NULL) fatal(eNOMEMORY);

  /* Open the executable file */

  if (!(exe = fopen(filename, "rb")))
    {
      dbg("ERROR: Error opening '%s': %d\n", filename, errno);
      goto errout_with_handle;
    }

  /* Load the POFF file into memory */

  err = poffReadFile(phandle, exe);
  if (err != eNOERROR)
    {
      dbg("ERROR: Could not read %s: %d\n", filename, err);
      goto errout_with_file;
    }

  /* Verify that the file is a pascal executable */

  ftype = poffGetFileType(phandle);
  if (ftype != FHT_EXEC)
    {
      dbg("ERROR: File is not a pascal executable: %d\n", ftype);
      goto errout_with_file;
    }

  farch = poffGetArchitecture(phandle);
  if (farch != FHA_PCODE_INSN16)
    {
      dbg("ERROR: File is not 16-bit pcode: %d\n", farch);
      goto errout_with_file;
    }

  /* Initialize the attribute structure */

  attr.varsize = varsize;
  attr.strsize = strsize;

  /* Extract the program entry point from the pascal executable */

  attr.entry = poffGetEntryPoint(phandle);

  /* Close the POFF file */

  (void)fclose(exe);

  /* Extract the program data from the POFF image */

  attr.maxpc = poffExtractProgramData(phandle, &attr.ispace);

  /* Extract the read-only data from the POFF image */

  attr.rosize = poffExtractRoData(phandle, &attr.rodata);

  /* Destroy the POFF image */

  poffDestroyHandle(phandle);

  /* Initialize the p-code interpreter */

  st = pexec_init(&attr);
  if (!st && attr.ispace)
    {
      /* Initialization failed, discard the allocated I-Space */

      free(st->ispace);
    }

  /* Discard the allocated RO data in any event */

  if (attr.rodata)
    {
      free(attr.rodata);
    }
  return st;

 errout_with_file:
  (void)fclose(exe);

 errout_with_handle:
  poffDestroyHandle(phandle);
  return NULL;
}
