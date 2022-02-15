/****************************************************************************
 *
 *   Copyright (C) 2020 Technology Innovation Institute. All rights reserved.
 *                 Author: Jukka Laitinen <jukkax@ssrc.tii.ae>
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

#pragma once

/* Encode the px4 boardctl ioctls in the following way:
 * the highest 4-bits identifies the boardctl's used by this if
 * the next 4-bits identifies the module which handles the ioctl
 * the low byte identiefies the actual IOCTL within the module
 */

#define _BOARDCTLIOCBASE		(0x4000)
#define IOCTL_IDX_TO_BASE(x)	((((x) & 0xF) << 8) | _BOARDCTLIOCBASE)
#define IOCTL_BASE_TO_IDX(x)	(((x) & 0x0F00) >> 8)

#define _ORBIOCDEVBASE				IOCTL_IDX_TO_BASE(0)
#define _HRTIOCBASE				IOCTL_IDX_TO_BASE(1)
#define _CRYPTOIOCBASE			IOCTL_IDX_TO_BASE(2)
#define _BUILTINIOCBASE			IOCTL_IDX_TO_BASE(3)
#define MAX_IOCTL_PTRS 4

/* The BUILTINIOCLAUNCH IOCTL is used to launch kernel side modules
 * from the user side code
 */

#define BUILTINIOCLAUNCH               (_PX4_IOC(_BUILTINIOCBASE, 1))

typedef struct builtinioclaunch {
	int argc;
	char **argv;
	int ret;
} builtinioclaunch_t;

typedef int (*ioctl_ptr_t)(unsigned int, unsigned long);

__BEGIN_DECLS

/* Function to initialize or reset the interface */
void kernel_ioctl_initialize(void);

/* Function to register a px4 boardctl handler */
int px4_register_boardct_ioctl(unsigned base, ioctl_ptr_t func);

__END_DECLS
