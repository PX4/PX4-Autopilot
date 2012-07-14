/****************************************************************************
 * mm/mm_environment.h
 *
 *   Copyright (C) 2007-2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __MM_MM_ENVIRONMENT_H
#define __MM_MM_ENVIRONMENT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* The platform configuratioin file will not be included when the memory
 * manager is built for the host-based test harness.
 */

#ifndef MM_TEST
#  include <nuttx/config.h>
#  include <nuttx/compiler.h>
#  include <sys/types.h>
#  include <stdlib.h>
#  include <string.h>
#  include <debug.h>
#  include <errno.h>
#  include <assert.h>
#  include <nuttx/mm.h>
#else
#  include <sys/types.h>
#  include <string.h>
#  include <assert.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Special definitions used when the memory mnager is built for the host-
 * based test harness.
 */

#ifdef MM_TEST

/* Fake NuttX dependencies */

# define FAR                        /* Normally in compiler.h */
# define CONFIG_CPP_HAVE_VARARGS 1  /* Normally in compiler.h */
# define CONFIG_MM_REGIONS 2        /* Normally in config.h */
# undef  CONFIG_MM_SMALL            /* Normally in config.h */
# define CONFIG_CAN_PASS_STRUCTS 1  /* Normally in config.h */
# undef  CONFIG_SMALL_MEMORY        /* Normally in config.h */

extern void mm_addregion(FAR void *heapstart, size_t heapsize);

/* Use the real system errno */

# define mm_errno errno

/* When built for the test harness, we change the names of the exported
 * functions so that they do not collide with the host libc names.
 */

# define malloc    mm_malloc
# define memalign  mm_memalign
# define realloc   mm_realloc
# define zalloc    mm_zalloc
# define calloc    mm_calloc
# define free      mm_free

/* Use normal libc assertion functions */

# undef ASSERT
# define ASSERT(e)      assert(e)
# undef DEBUGASSERT
# define DEBUGASSERT(e) assert(e)

/* Debug macros are always on */

# define CONFIG_DEBUG

# undef mdbg
# define mdbg(format, arg...) printf(format, ##arg)
# undef mvdg
# define mvdbg(format, arg...) printf(format, ##arg)

#else
# define mm_errno get_errno()
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#endif /* __MM_MM_ENVIRONMENT_H */
