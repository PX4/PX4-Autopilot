/****************************************************************************
 * NxWidgets/nxwm/include/nxwmconfig.hxx
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
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

#ifndef __INCLUDE_NXWMGLYPHS_HXX
#define __INCLUDE_NXWMGLYPHS_HXX

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include "nxconfig.hxx"
#include "glyphs.hxx"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Bitmap Glyphs
 ****************************************************************************/

#if defined(__cplusplus)

namespace NxWM
{
  extern const struct NXWidgets::SRlePaletteBitmap g_calculatorBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_calibrationBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_cmdBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_minimizeBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_nshBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_playBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_startBitmap;
  extern const struct NXWidgets::SRlePaletteBitmap g_stopBitmap;
}

#endif // __cplusplus
#endif // __INCLUDE_NXWMGLYPHS_HXX
