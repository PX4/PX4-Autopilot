/****************************************************************************
 * NxWidgets/libnxwidgets/src/glyph_radiobuttonon.cxx
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
 ****************************************************************************
 *
 * Portions of this package derive from Woopsi (http://woopsi.org/) and
 * portions are original efforts.  It is difficult to determine at this
 * point what parts are original efforts and which parts derive from Woopsi.
 * However, in any event, the work of  Antony Dzeryn will be acknowledged
 * in most NxWidget files.  Thanks Antony!
 *
 *   Copyright (c) 2007-2011, Antony Dzeryn
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the names "Woopsi", "Simian Zombie" nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Antony Dzeryn ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Antony Dzeryn BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/fb.h>
#include <nuttx/rgbcolors.h>

#include "nxconfig.hxx"
#include "cbitmap.hxx"
#include "glyphs.hxx"

#if CONFIG_NXWIDGETS_BPP != 8 // No support for 8-bit color format

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Bitmap Data
 ****************************************************************************/

using namespace NXWidgets;

#if CONFIG_NXWIDGETS_BPP == 16
#  define COLOR_FMT FB_FMT_RGB16_565
#  define RGB16_TRANSP 0x0000
#  define RGB16_COLOR  RGBTO16(216,196,96)

static const uint16_t g_radioButtonOnGlyph[] =
{
  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,
  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_DARKRED, RGB16_DARKRED, RGB16_DARKRED, RGB16_DARKRED, RGB16_TRANSP,  RGB16_TRANSP,
  RGB16_TRANSP,  RGB16_DARKRED, RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_WHITE,   RGB16_TRANSP,
  RGB16_DARKRED, RGB16_TRANSP,  RGB16_TRANSP,  RGB16_COLOR,   RGB16_COLOR,   RGB16_TRANSP,  RGB16_TRANSP,  RGB16_WHITE,
  RGB16_DARKRED, RGB16_TRANSP,  RGB16_COLOR,   RGB16_COLOR,   RGB16_COLOR,   RGB16_COLOR,   RGB16_TRANSP,  RGB16_WHITE,
  RGB16_DARKRED, RGB16_TRANSP,  RGB16_COLOR,   RGB16_COLOR,   RGB16_COLOR,   RGB16_COLOR,   RGB16_TRANSP,  RGB16_WHITE,
  RGB16_DARKRED, RGB16_TRANSP,  RGB16_TRANSP,  RGB16_COLOR,   RGB16_COLOR,   RGB16_TRANSP,  RGB16_TRANSP,  RGB16_WHITE,
  RGB16_TRANSP,  RGB16_DARKRED, RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_WHITE,   RGB16_TRANSP,
  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_WHITE,   RGB16_WHITE,   RGB16_WHITE,   RGB16_WHITE,   RGB16_TRANSP,  RGB16_TRANSP,
  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,  RGB16_TRANSP,
};

#elif CONFIG_NXWIDGETS_BPP == 24 || CONFIG_NXWIDGETS_BPP == 32
#  define COLOR_FMT FB_FMT_RGB24
#  define RGB24_TRANSP 0x00000000
#  define RGB24_COLOR  RGBTO24(216,196,96)

static const uint32_t g_radioButtonOnGlyph[] =
{
  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,
  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_DARKRED, RGB24_DARKRED, RGB24_DARKRED, RGB24_DARKRED, RGB24_TRANSP,  RGB24_TRANSP,
  RGB24_TRANSP,  RGB24_DARKRED, RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_WHITE,   RGB24_TRANSP,
  RGB24_DARKRED, RGB24_TRANSP,  RGB24_TRANSP,  RGB24_COLOR,   RGB24_COLOR,   RGB24_TRANSP,  RGB24_TRANSP,  RGB24_WHITE,
  RGB24_DARKRED, RGB24_TRANSP,  RGB24_COLOR,   RGB24_COLOR,   RGB24_COLOR,   RGB24_COLOR,   RGB24_TRANSP,  RGB24_WHITE,
  RGB24_DARKRED, RGB24_TRANSP,  RGB24_COLOR,   RGB24_COLOR,   RGB24_COLOR,   RGB24_COLOR,   RGB24_TRANSP,  RGB24_WHITE,
  RGB24_DARKRED, RGB24_TRANSP,  RGB24_TRANSP,  RGB24_COLOR,   RGB24_COLOR,   RGB24_TRANSP,  RGB24_TRANSP,  RGB24_WHITE,
  RGB24_TRANSP,  RGB24_DARKRED, RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_WHITE,   RGB24_TRANSP,
  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_WHITE,   RGB24_WHITE,   RGB24_WHITE,   RGB24_WHITE,   RGB24_TRANSP,  RGB24_TRANSP,
  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,  RGB24_TRANSP,
};

#else
#  warning "Other pixel depths not yet supported"
#endif

/****************************************************************************
 * Public Bitmap Structure Defintions
 ****************************************************************************/

const struct SBitmap NXWidgets::g_radioButtonOn =
{
  CONFIG_NXWIDGETS_BPP,              // bpp    - Bits per pixel
  COLOR_FMT,                         // fmt    - Color format
  8,                                 // width  - Width in pixels
  10,                                // height - Height in rows
  (8*CONFIG_NXWIDGETS_BPP + 7) / 8,  // stride - Width in bytes
  g_radioButtonOnGlyph               // data   - Pointer to the beginning of pixel data
};

#endif // CONFIG_NXWIDGETS_BPP != 8
