/****************************************************************************
 * arch/arm/src/common/sam3u_mpuinit.c
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
#include <arch/board/user_map.h>
#include "mpu.h"

#ifdef CONFIG_NUTTX_KERNEL

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

#ifndef MAX
#  define MAX(a,b) a > b ? a : b
#endif

#ifndef MIN
#  define MIN(a,b) a < b ? a : b
#endif

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
 * Name: sam3u_mpuinitialize
 *
 * Description:
 *   Configure the MPU to permit user-space access to only restricted SAM3U
 *   resources.
 *
 ****************************************************************************/

void sam3u_mpuinitialize(void)
{
  uintptr_t datastart = MIN(ONFIG_USER_DATADESTSTART, CONFIG_USER_BSSSTART);
  uintptr_t dataend   = MAX(ONFIG_USER_DATADESTEND,   CONFIG_USER_BSSEND);

  DEBUGASSERT(CONFIG_USER_TEXTEND >= CONFIG_USER_TEXTSTART && dataend >= datastart);

	@echo "#define C     0x`grep \" _stext\"       $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h
	@echo "#define        0x`grep \" _etext$\"      $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h
	@echo "#define CONFIG_USER_DATASOURCE    0x`grep \" _eronly$\"    $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h
	@echo "#define C 0x`grep \" _sdata$\"     $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h
	@echo "#define CONFIG_USER_   0x`grep \" _edata$\"     $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h
	@echo "#define       0x`grep \" _sbss\"       $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h
	@echo "#define CONFIG_USER_        0x`grep \" _ebss$\"      $(TOPDIR)/User.map | cut -d' ' -f1`" >> $(BOARD_INCLUDE)/user_map.h

  /* Show MPU information */

  mpu_showtype();

  /* Configure user flash and SRAM space */

  mpu_userflash(CONFIG_USER_TEXTSTART, CONFIG_USER_TEXTEND - CONFIG_USER_TEXTSTART);
  mpu_userintsram(datastart, dataend - datastart);

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

/****************************************************************************
 * Name: sam3u_mpuheap
 *
 * Description:
 *  Map a heap region (probably needs to extension to handle external SRAM).
 *
 ****************************************************************************/

void sam3u_mpuheap(uintptr_t start, size_t size)
{
  mpu_userintsram(start, size);
}

#endif /* CONFIG_NUTTX_KERNEL */

