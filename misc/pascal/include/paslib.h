/***************************************************************************
 * include/paslib.h
 * External Declarations associated with paslib
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
 ***************************************************************************/

#ifndef __PASLIB_H
#define __PASLIB_H

/***************************************************************************
 * Included Files
 ***************************************************************************/

#include "keywords.h"
#include "stdint.h"
#include "stdbool.h"
#include "pdefs.h"
#include "pofflib.h"

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

/* POFF file is always big-endian */

#ifdef CONFIG_ENDIAN_BIG
# undef  CONFIG_POFF_SWAPNEEDED
# define poff16(val) (val)
# define poff32(val) (val)
#else
# define CONFIG_POFF_SWAPNEEDED 1
# define poff16(val) poffSwap16(val)
# define poff32(val) poffSwap32(val)
#endif

/***************************************************************************
 * Global Function Prototypes
 ***************************************************************************/

/* File name extension helper */

extern bool     extension(const char *inName, const char *ext, char *outName,
                          bool force_default);

/* Math helpers */

extern int32_t  signExtend16(uint16_t arg16);
extern int32_t  signExtend25(uint32_t arg25);

/* Endian-ness helpers */

extern uint16_t poffSwap16(uint16_t val);
extern uint32_t poffSwap32(uint32_t val);

/***************************************************************************
 * Global Variables
 ***************************************************************************/

#endif /* __PASLIB_H */

