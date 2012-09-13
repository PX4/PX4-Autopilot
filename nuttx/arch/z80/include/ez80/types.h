/****************************************************************************
 * arch/z80/include/ez80/types.h
 * include/arch/chip/types.h
 *
 *   Copyright (C) 2008-2009 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_Z80_INCLUDE_EZ80_TYPES_H
#define __ARCH_Z80_INCLUDE_EZ80_TYPES_H

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
 *
 * These are the sizes of the types supported by the ZiLOG compiler:
 *
 *   int    - 24-bits
 *   short  - 16-bits
 *   long   - 32-bits
 *   char   - 8-bits
 *   float  - 32-bits
 */

typedef signed char        _int8_t;
typedef unsigned char      _uint8_t;

typedef signed short       _int16_t;
typedef unsigned short     _uint16_t;

typedef signed int         _int24_t;
typedef unsigned int       _uint24_t;
#define __INT24_DEFINED

typedef signed long        _int32_t;
typedef unsigned long      _uint32_t;

/* A pointer is 2 or 3 bytes, depending upon if the ez80 is in z80
 * compatibility mode or not
 *
 *   Z80 mode - 16 bits
 *   ADL mode - 24 bits
 */

#ifdef CONFIG_EZ80_Z80MODE
typedef signed short       _intptr_t;
typedef unsigned short     _uintptr_t;
#else
typedef signed int         _intptr_t;
typedef unsigned int       _uintptr_t;
#endif

/* This is the size of the interrupt state save returned by irqsave().
 * It holds the AF regiser pair + a zero pad byte
 */

typedef _uint24_t          irqstate_t;

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Global Function Prototypes
 ****************************************************************************/

#endif /* __ARCH_Z80_INCLUDE_EZ80_TYPES_H */
