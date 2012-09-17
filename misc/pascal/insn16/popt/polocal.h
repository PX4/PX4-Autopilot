/***************************************************************************
 * polocal.h
 * External Declarations associated with polocal.c
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

#ifndef __POLOCAL_H
#define __POLOCAL_H

/***************************************************************************
* Included Files
****************************************************************************/

#include <stdint.h>
#include "keywords.h"
#include "pdefs.h"
#include "pofflib.h"

/***************************************************************************
* Definitions
****************************************************************************/

#define WINDOW             10           /* size of optimization window */

/***************************************************************************
* Global Function Prototypes
****************************************************************************/

extern void localOptimization(poffHandle_t poffHandle,
                              poffProgHandle_t poffProgHandle);
extern void deletePcode      (int16_t delIndex);
extern void deletePcodePair  (int16_t delIndex1, int16_t delIndex2);

/***************************************************************************
 * Global Variables
 ****************************************************************************/

extern OPTYPE  ptable [WINDOW];        /* Pcode Table */
extern OPTYPE *pptr   [WINDOW];        /* Valid Pcode Pointers */

extern int16_t nops;                   /* No. Valid Pcode Pointers */
extern int16_t end_out;                /* 1 = oEND pcode has been output */

#endif /* __PLOCAL_H */
