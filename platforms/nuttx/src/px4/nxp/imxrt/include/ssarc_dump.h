/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#pragma once

#define SSARC_DUMP_GETDESC_IOCTL _DIOC(0x0000) /* Returns a progmem_s */
#define SSARC_DUMP_CLEAR_IOCTL _DIOC(0x0010) /* Erases flash sector */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

struct ssarc_s {
	struct timespec lastwrite;
	int fileno;
	int len;
	int flags;
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Function: ssarc_dump_initialize
 *
 * Description:
 *   Initialize the Progmem dump driver
 *
 * Input Parameters:
 *   devpath - the path to instantiate the files.
 *   sizes   - Pointer to a any array of file sizes to create
 *             the last entry should be 0
 *             A size of -1 will use all the remaining spaces
 *
 * Returned Value:
 *   Number of files created on success; Negated errno on failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ssarc_dump_initialize(char *devpath, int *sizes);

/****************************************************************************
 * Function: ssarc_dump_savepanic
 *
 * Description:
 *   Saves the panic context in a previously allocated BBSRAM file
 *
 * Parameters:
 *   fileno  - the value returned by the ioctl SSARC_DUMP_GETDESC_IOCTL
 *   context - Pointer to a any array of bytes to save
 *   length  - The length of the data pointed to byt context
 *
 * Returned Value:
 *   Length saved or negated errno.
 *
 * Assumptions:
 *
 ****************************************************************************/

int ssarc_dump_savepanic(int fileno, uint8_t *context, int length);

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */
