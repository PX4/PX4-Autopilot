/**********************************************************************
 * paddtmpopcode
 * P-Code access utilities
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

#include "keywords.h"
#include "podefs.h"
#include "pinsn16.h"

#include "paslib.h"
#include "pofflib.h"
#include "pinsn.h"

/**********************************************************************
 * Private Function Prototypes
 **********************************************************************/

/**********************************************************************
 * Global Variables
 **********************************************************************/

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

void insn_AddTmpOpCode(poffProgHandle_t progHandle, OPTYPE *ptr)
{
  /* Write the opcode which is always present */

  (void)poffAddTmpProgByte(progHandle, ptr->op);

  /* Write the 8-bit argument if present */

  if (ptr->op & o8)  
    {
      (void)poffAddTmpProgByte(progHandle, ptr->arg1);
    }

  /* Write the 16-bit argument if present */

  if (ptr->op & o16)
    { 
      (void)poffAddTmpProgByte(progHandle, (ptr->arg2 >> 8));
      (void)poffAddTmpProgByte(progHandle, (ptr->arg2 & 0xff));
    }
}

/**********************************************************************/

void insn_ResetTmpOpCodeWrite(poffProgHandle_t progHandle)
{
  poffResetProgHandle(progHandle);
}

/***********************************************************************/
