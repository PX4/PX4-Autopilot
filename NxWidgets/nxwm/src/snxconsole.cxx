/********************************************************************************************
 * NxWidgets/nxwm/src/snxconsole.cxx
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
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
 
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/nx/nxglib.h>
#include <nuttx/fb.h>
#include <nuttx/rgbcolors.h>

#include "nxwmconfig.hxx"
#include "crlepalettebitmap.hxx"
#include "cnxconsole.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

#define BITMAP_NROWS      26
#define BITMAP_NCOLUMNS   25
#define BITMAP_NLUTCODES  2

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_nshNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR, CONFIG_NXWM_DEFAULT_FOREGROUNDCOLOR
};

static const uint32_t g_nshSelectedLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR, CONFIG_NXWM_DEFAULT_SELECTEDFOREGROUNDCOLOR
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_nshNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR, CONFIG_NXWM_DEFAULT_FOREGROUNDCOLOR
};

static const uint16_t g_nshSelectedLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR, CONFIG_NXWM_DEFAULT_SELECTEDFOREGROUNDCOLOR
};

/* 8-bit color lookups.  NOTE:  This is really dumb!  The lookup index is 8-bits and it used
 * to lookup an 8-bit value.  There is no savings in that!  It would be better to just put
 * the 8-bit color/greyscale value in the run-length encoded image and save the cost of these
 * pointless lookups.  But these pointless lookups do make the logic compatible with the
 * 16- and 24-bit types.
 */

#elif CONFIG_NXWIDGETS_BPP == 8

/* RGB8 (332) Colors or 8-bit greyscale */

static const nxgl_mxpixel_t g_nshNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR, CONFIG_NXWM_DEFAULT_FOREGROUNDCOLOR
};

static const nxgl_mxpixel_t g_nshSelectedLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_SELECTEDBACKGROUNDCOLOR, CONFIG_NXWM_DEFAULT_SELECTEDFOREGROUNDCOLOR
};

#else
# error "Unsupport pixel format"
#endif

static const struct SRlePaletteBitmapEntry g_nshRleEntries[] =
{
  {11, 0}, {4, 1}, {10, 0},                                        /* Row 0 */
  {7, 0}, {1, 1}, {2, 0}, {5, 1}, {2, 0}, {1, 1}, {7, 0},          /* Row 1 */
  {6, 0}, {3, 1}, {2, 0}, {4, 1}, {2, 0}, {2, 1}, {6, 0},          /* Row 2 */
  {5, 0}, {4, 1}, {2, 0}, {4, 1}, {2, 0}, {4, 1}, {4, 0},          /* Row 3 */
  {6, 0}, {3, 1}, {2, 0}, {4, 1}, {2, 0}, {3, 1}, {5, 0},          /* Row 4 */
  {3, 0}, {1, 1}, {2, 0}, {3, 1}, {2, 0}, {4, 1}, {1, 0}, {4, 1},  /* Row 5 */
  {2, 0}, {1, 1}, {2, 0},
  {2, 0}, {3, 1}, {1, 0}, {3, 1}, {2, 0}, {4, 1}, {1, 0}, {4, 1},  /* Row 6 */
  {1, 0}, {3, 1}, {1, 0},
  {1, 0}, {4, 1}, {2, 0}, {3, 1}, {1, 0}, {4, 1}, {1, 0}, {3, 1},  /* Row 7 */
  {2, 0}, {3, 1}, {1, 0},
  {1, 0}, {5, 1}, {1, 0}, {3, 1}, {1, 0}, {4, 1}, {1, 0}, {3, 1},  /* Row 8 */
  {1, 0}, {4, 1}, {1, 0},
  {2, 0}, {4, 1}, {1, 0}, {3, 1}, {1, 0}, {3, 1}, {2, 0}, {2, 1},  /* Row 9 */
  {2, 0}, {4, 1}, {1, 0},
  {3, 0}, {3, 1}, {2, 0}, {2, 1}, {1, 0}, {3, 1}, {2, 0}, {2, 1},  /* Row 10 */
  {1, 0}, {4, 1}, {2, 0},
  {2, 1}, {2, 0}, {3, 1}, {1, 0}, {2, 1}, {2, 0}, {2, 1}, {2, 0},  /* Row 11 */
  {2, 1}, {1, 0}, {3, 1}, {2, 0}, {1, 1},
  {3, 1}, {1, 0}, {3, 1}, {1, 0}, {3, 1}, {1, 0}, {2, 1}, {1, 0},  /* Row 12 */
  {2, 1}, {2, 0}, {3, 1}, {1, 0}, {2, 1},
  {4, 1}, {1, 0}, {3, 1}, {1, 0}, {2, 1}, {1, 0}, {2, 1}, {1, 0},  /* Row 13 */
  {2, 1}, {1, 0}, {3, 1}, {1, 0}, {3, 1},
  {4, 1}, {2, 0}, {2, 1}, {1, 0}, {2, 1}, {1, 0}, {2, 1}, {1, 0},  /* Row 14 */
  {2, 1}, {1, 0}, {2, 1}, {2, 0}, {3, 1},
  {1, 0}, {4, 1}, {2, 0}, {1, 1}, {1, 0}, {2, 1}, {1, 0}, {2, 1},  /* Row 15 */
  {1, 0}, {1, 1}, {1, 0}, {2, 1}, {2, 0}, {3, 1}, {1, 0},
  {2, 0}, {4, 1}, {1, 0}, {2, 1}, {1, 0}, {1, 1}, {1, 0}, {2, 1},  /* Row 16 */
  {1, 0}, {1, 1}, {1, 0}, {2, 1}, {1, 0}, {4, 1}, {1, 0},
  {3, 0}, {4, 1}, {1, 0}, {1, 1}, {1, 0}, {1, 1}, {1, 0}, {6, 1},  /* Row 17 */
  {1, 0}, {4, 1}, {2, 0},
  {5, 0}, {2, 1}, {1, 0}, {9, 1}, {1, 0}, {3, 1}, {4, 0},          /* Row 18 */
  {6, 0}, {14, 1}, {5, 0},                                         /* Row 19 */
  {5, 0}, {16, 1}, {4, 0},                                         /* Row 20 */
  {4, 0}, {18, 1}, {3, 0},                                         /* Row 21 */
  {4, 0}, {18, 1}, {3, 0},                                         /* Row 22 */
  {4, 0}, {18, 1}, {3, 0},                                         /* Row 23 */
  {4, 0}, {18, 1}, {3, 0},                                         /* Row 24 */
  {5, 0}, {16, 1}, {4, 0},                                         /* Row 25 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct SRlePaletteBitmap NxWM::g_nshBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels 
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_nshNormalLut,      //          Index 0: Unselected LUT
    g_nshSelectedLut,    //          Index 1: Selected LUT
  },
  g_nshRleEntries        // data   - Pointer to the beginning of the RLE data
};
