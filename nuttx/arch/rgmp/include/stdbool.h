/****************************************************************************
 * arch/rgmp/include/stdbool.h
 *
 *   Copyright (C) 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_RGMP_INCLUDE_STDBOOL_H
#define __ARCH_RGMP_INCLUDE_STDBOOL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* bool, true, and false must be provided as macros so that they can be
 * redefined by the application if necessary.
 *
 * NOTE: Under C99 'bool' is required to be defined to be the intrinsic type
 * _Bool.  However, in this NuttX context, we need backward compatibility
 * to pre-C99 standards where _Bool is not an intrinsic type.  Hence, we
 * use _Bool8 as the underlying type.
 */

#define true  1
#define false 0

#define __bool_true_false_are_defined 1

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* A byte is the smallest address memory element (at least in architectures
 * that do not support bit banding).  The requirement is only that type _Bool
 * be large enough to hold the values 0 and 1.  We select uint8_t to minimize
 * the RAM footprint of the executable.
 *
 * NOTE: We can't actually define the type _Bool here.  Under C99 _Bool is
 * an intrinsic type and cannot be the target of a typedef.  However, in this
 * NuttX context, we also need backward compatibility to pre-C99 standards
 * where _Bool is not an intrinsic type.  We work around this by using _Bool8
 * as the underlying type.
 */

typedef uint8_t _Bool8;

#endif /* __ARCH_RGMP_INCLUDE_STDBOOL_H */
