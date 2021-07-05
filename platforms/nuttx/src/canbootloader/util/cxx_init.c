/****************************************************************************
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#include <sched.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>
#include <syslog.h>

#ifdef CONFIG_HAVE_CXXINITIALIZE
#include "cxx_init.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type defines one entry in initialization array */

typedef CODE void (*initializer_t)(void);

/****************************************************************************
 * External References
 ****************************************************************************/

/* _sinit and _einit are symbols exported by the linker script that mark the
 * beginning and the end of the C++ initialization section.
 */

extern initializer_t _sinit;
extern initializer_t _einit;

/* _stext and _etext are symbols exported by the linker script that mark the
 * beginning and the end of text.
 */

extern uintptr_t _stext;
extern uintptr_t _etext;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxx_initialize
 *
 * Description:
 *   If C++ and C++ static constructors are supported, then this function
 *   must be provided by board-specific logic in order to perform
 *   initialization of the static C++ class instances.
 *
 *   This function should then be called in the application-specific
 *   user_start logic in order to perform the C++ initialization.  NOTE
 *   that no component of the core NuttX RTOS logic is involved; this
 *   function definition only provides the 'contract' between application
 *   specific C++ code and platform-specific toolchain support.
 *
 ****************************************************************************/

void cxx_initialize(void)
{
	static int inited = 0;

	if (inited == 0) {
		initializer_t *initp;

		sinfo("_sinit: %p _einit: %p _stext: %p _etext: %p\n",
		      &_sinit, &_einit, &_stext, &_etext);

		/* Visit each entry in the initialization table */

		for (initp = &_sinit; initp != &_einit; initp++) {
			initializer_t initializer = *initp;
			sinfo("initp: %p initializer: %p\n", initp, initializer);

			/* Make sure that the address is non-NULL and lies in the text
			 * region defined by the linker script.  Some toolchains may put
			 * NULL values or counts in the initialization table.
			 */

			if ((FAR void *)initializer >= (FAR void *)&_stext &&
			    (FAR void *)initializer < (FAR void *)&_etext) {
				sinfo("Calling %p\n", initializer);
				initializer();
			}
		}

		inited = 1;
	}
}

#endif
