/**********************************************************************
 * pgetopcode.c
 * P-Code access utilities
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
 **********************************************************************/

/**********************************************************************
 * Included Files
 **********************************************************************/

#include <stdint.h>

#include "keywords.h"
#include "podefs.h"
#include "pinsn32.h"

#include "paslib.h"
#include "pofflib.h"
#include "pinsn.h"

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

static int16_t g_bEndIn  = 0;  /* 1 = oEND pcode or EOF received */

/**********************************************************************
 * Private Variables
 **********************************************************************/

/**********************************************************************
 * Private Functions
 **********************************************************************/

/**********************************************************************
 * Global Functions
 **********************************************************************/

/**********************************************************************/

uint32_t insn_GetOpCode(poffHandle_t handle, OPTYPE *ptr)
{
  uint32_t opsize = 1;
  int c;

  TRACE(stderr, "[insn_GetOpCode]");

  /* If we are not already at the EOF, read the next character from
   * the input stream.
   */

  if (!g_bEndIn)
    c = poffGetProgByte(handle);
  else
    c = EOF;

  /* Check for end of file.  We may have previously parsed oEND which
   * is a 'logical' end of file for a pascal program (but not a unit)
   * or we may be at the physical end of the file wihout encountering
   * oEND (typical for a UNIT file).
   */

  if ((g_bEndIn) || (c == EOF))
    {
      ptr->op = oEND;
    }
  else
    {
      ptr->op = c;
      g_bEndIn = (c == oEND);

      if (c & o32)
        {
          uint8_t *pb = (uint8_t*)&ptr->arg;
          pb[opB1]  = poffGetProgByte(handle);
          pb[opB2]  = poffGetProgByte(handle);
          pb[opB3]  = poffGetProgByte(handle);
          pb[opB4]  = poffGetProgByte(handle);
          opsize   += 4;
        }
    }

  return opsize;
}

/**********************************************************************/

void insn_ResetOpCodeRead(poffHandle_t handle)
{
  poffResetAccess(handle);
  g_bEndIn = 0;
}

/***********************************************************************/
