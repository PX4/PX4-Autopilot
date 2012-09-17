/**********************************************************************
 *  pfproghandle.c
 *  POFF temporary progdata support
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
 **********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "keywords.h"  /* Standard types */
#include "pedefs.h"    /* Pascal error codes */

#include "pfprivate.h" /* POFF private definitions */
#include "pofflib.h"   /* Public interfaces */

/**********************************************************************
 * Definitions
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/***********************************************************************
 * Private Function Prototypes
 ***********************************************************************/

/***********************************************************************
 * Private Functions
 ***********************************************************************/

/***********************************************************************
 * Public Functions
 ***********************************************************************/

/***********************************************************************/
/* Set all global data structures to a known state */

poffProgHandle_t poffCreateProgHandle(void)
{
  poffProgInfo_t *poffProgInfo;

  /* Create a new POFF handle */

  poffProgInfo = (poffProgInfo_t*)malloc(sizeof(poffProgInfo_t));
  if (poffProgInfo != NULL)
    {
      /* Set everthing to zero */

      memset(poffProgInfo, 0, sizeof(poffProgInfo_t));
    }
  return poffProgInfo;
}

/***********************************************************************/

void poffDestroyProgHandle(poffProgHandle_t handle)
{
  /* Free all of the allocated, in-memory data */

  poffResetProgHandle(handle);

  /* Free the container */

  free(handle);
}

/***********************************************************************/

void poffResetProgHandle(poffProgHandle_t handle)
{
  poffProgInfo_t *poffProgInfo = (poffProgInfo_t*)handle;

  /* Free all of the allocated, in-memory data */

  if (poffProgInfo->progSectionData)
    {
      free(poffProgInfo->progSectionData);
    }

  /* Reset everything to the initial state */

  poffProgInfo->progSectionData  = NULL;
  poffProgInfo->progSectionSize  = 0;
  poffProgInfo->progSectionAlloc = 0;
}

/***********************************************************************/
