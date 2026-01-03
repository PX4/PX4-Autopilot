/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

/**
 * @file nuttx_stubs.c
 *
 * Stubs for optional NuttX functions that may be referenced but not always provided
 */

#include <nuttx/config.h>
#include <stdlib.h>
#include <stdbool.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Block driver unregister */
__attribute__((weak))
int unregister_blockdriver(const char *path)
{
	return -1;
}

/* NSH console main */
__attribute__((weak))
int nsh_consolemain(int argc, char **argv)
{
	return -1;
}

/* NSH system with controlling TTY */
__attribute__((weak))
int nsh_system_ctty(const char *cmd)
{
	return -1;
}

/* Network library functions */
__attribute__((weak))
int netlib_obtain_ipv4addr(const char *ifname, void *addr)
{
	return -1;
}

/* Board hardfault init */
__attribute__((weak))
int board_hardfault_init(int rl, bool enable)
{
	return 0;
}
