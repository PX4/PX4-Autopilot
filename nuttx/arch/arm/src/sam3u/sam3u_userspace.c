/****************************************************************************
 * arch/arm/src/common/sam3u_userspace.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <arch/board/user_map.h>

#ifdef CONFIG_NUTTX_KERNEL

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam3u_userspace
 *
 * Description:
 *   For the case of the separate user-/kernel-space build, perform whatever
 *   platform specific initialization of the user memory is required.
 *   Normally this just means initializing the user space .data and .bss
 *   segements.
 *
 ****************************************************************************/

void sam3u_userspace(void)
{
  uint8_t *src;
  uint8_t *dest;
  uint8_t *end;

  /* Clear all of user-space .bss */

  DEBUGASSERT((uintptr_t)CONFIG_USER_DATADESTSTART <= CONFIG_USER_DATADESTEND);

  dest = (uint8_t*)CONFIG_USER_BSSSTART;
  end  = (uint8_t*)CONFIG_USER_BSSEND;

  while (dest != end)
    {
      *dest++ = 0;
    }

  /* Initialize all of user-space .data */

  DEBUGASSERT((uintptr_t)CONFIG_USER_DATADESTSTART <= CONFIG_USER_DATADESTEND);

  src  = (uint8_t*)CONFIG_USER_DATASOURCE;
  dest = (uint8_t*)CONFIG_USER_DATADESTSTART;
  end  = (uint8_t*)CONFIG_USER_DATADESTEND;

  while (dest != end)
    {
      *dest++ = *src++;
    }
}

#endif /* CONFIG_NUTTX_KERNEL */

