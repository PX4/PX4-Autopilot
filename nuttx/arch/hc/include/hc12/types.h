/****************************************************************************
 * arch/hc/include/hc12/types.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

/* This file should never be included directed but, rather, only indirectly
 * through sys/types.h
 */

#ifndef __ARCH_HC_INCLUDE_HC12_TYPES_H
#define __ARCH_HC_INCLUDE_HC12_TYPES_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Type Declarations
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* These are the sizes of the standard integer types.  NOTE that these type
 * names have a leading underscore character.  This file will be included
 * (indirectly) by include/stdint.h and typedef'ed to the final name without
 * the underscore character.  This roundabout way of doings things allows
 * the stdint.h to be removed from the include/ directory in the event that
 * the user prefers to use the definitions provided by their toolchain header
 * files
 */

typedef signed char        _int8_t;
typedef unsigned char      _uint8_t;
typedef signed short       _int16_t;
typedef unsigned short     _uint16_t;

/* Normally, mc68hc1x code is compiled with the -mshort option
 * which results in a 16-bit integer.  If -mnoshort is defined
 * then an integer is 32-bits.  GCC will defined __INT__ accordingly:
 */

# if __INT__ == 16
typedef signed long        _int32_t;
typedef unsigned long      _uint32_t;
#else
typedef signed int         _int32_t;
typedef unsigned int       _uint32_t;
#endif

typedef signed long long   _int64_t;
typedef unsigned long long _uint64_t;
#define __INT64_DEFINED

/* A pointer is two bytes */

typedef signed short       _intptr_t;
typedef unsigned short     _uintptr_t;

/* This is the size of the interrupt state save returned by irqsave()*/

typedef unsigned int       irqstate_t;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_HC_INCLUDE_HC12_TYPES_H */
