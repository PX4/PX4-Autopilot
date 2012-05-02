/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_start.cxx
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

#include "crlepalettebitmap.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

#define BITMAP_NROWS     25
#define BITMAP_NCOLUMNS  25
#define BITMAP_NLUTCODES 34

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_startLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0x6db66d, 0x006d00, 0x246d00, 0x6d9249, 0xdbffb6, 0xb6b692, 0x246d24, 0x249224,  /* Codes 1-8 */
  0x49b649, 0x6db649, 0x499249, 0xdbffdb, 0x92b692, 0xb6dbb6, 0x6ddb6d, 0x92b66d,  /* Codes 9-16 */
  0x6d926d, 0x92db92, 0x49926d, 0x49b624, 0x92db6d, 0xb6ffb6, 0xffffff, 0x496d49,  /* Codes 17-24 */
  0x92ff92, 0x24b624, 0x496d24, 0xb6db92, 0x499224, 0x249200, 0x92dbb6, 0x004900,  /* Codes 25-32 */
  0x009200                                                                         /* Code 33 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_startLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0x6dad, 0x0360, 0x2360, 0x6c89, 0xdff6, 0xb5b2, 0x2364, 0x2484, 0x4da9, 0x6da9,  /* Codes 0-10 */
  0x4c89, 0xdffb, 0x95b2, 0xb6d6, 0x6ecd, 0x95ad, 0x6c8d, 0x96d2, 0x4c8d, 0x4da4,  /* Codes 11-20 */
  0x96cd, 0xb7f6, 0xffff, 0x4b69, 0x97f2, 0x25a4, 0x4b64, 0xb6d2, 0x4c84, 0x2480,  /* Codes 21-30 */
  0x96d6, 0x0240, 0x0480                                                           /* Codes 31-33 */
};

/* 8-bit color lookups.  NOTE:  This is really dumb!  The lookup index is 8-bits and it used
 * to lookup an 8-bit value.  There is no savings in that!  It would be better to just put
 * the 8-bit color/greyscale value in the run-length encoded image and save the cost of these
 * pointless lookups.  But these p;ointless lookups do make the logic compatible with the
 * 16- and 24-bit types.
 */

#elif CONFIG_NXWIDGETS_BPP == 8
#  ifdef CONFIG_NXWIDGETS_GREYSCALE

/* 8-bit Greyscale */

static const uint8_t g_startLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0x97, 0x3f, 0x4a, 0x7e, 0xeb, 0xb1, 0x4e, 0x64, 0x88, 0x93, 0x73, 0xf0, 0xa7,    /* Codes 1-13 */
  0xcb, 0xad, 0xa2, 0x82, 0xbc, 0x77, 0x84, 0xb8, 0xe0, 0xff, 0x5e, 0xd1, 0x79,    /* Codes 14-26 */
  0x59, 0xc7, 0x6f, 0x60, 0xc0, 0x2a, 0x55                                         /* Codes 27-33 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_startLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0x75, 0x0c, 0x2c, 0x71, 0xde, 0xb6, 0x2c, 0x30, 0x55, 0x75, 0x51, 0xdf, 0x96,    /* Codes 1-13 */
  0xba, 0x79, 0x95, 0x71, 0x9a, 0x51, 0x54, 0x99, 0xbe, 0xff, 0x4d, 0x9e, 0x34,    /* Codes 14-26 */
  0x4c, 0xba, 0x50, 0x30, 0x9a, 0x08, 0x10                                         /* Codes 27-33 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_startRleEntries[] =
{
  {  7,   0}, {  1,   1}, {  3,   2}, {  1,   3}, {  4,   2}, {  1,   4}, {  1,   5}, {  7,   0},  /* Row 0 */
  {  5,   0}, {  1,   6}, {  2,   7}, {  1,   8}, {  1,   9}, {  2,   1}, {  1,  10}, {  2,   9},  /* Row 1 */
  {  1,  11}, {  2,   7}, {  1,   1}, {  1,  12}, {  5,   0},
  {  4,   0}, {  1,  13}, {  1,   7}, {  1,   3}, {  1,   9}, {  8,   1}, {  1,   9}, {  1,   3},  /* Row 2 */
  {  1,   7}, {  1,  11}, {  1,  14}, {  4,   0},
  {  3,   0}, {  1,  13}, {  1,   7}, {  1,   3}, {  1,   9}, {  1,  15}, {  4,   1}, {  1,  16},  /* Row 3 */
  {  3,   1}, {  1,  15}, {  1,   9}, {  1,   8}, {  1,   7}, {  1,  17}, {  4,   0},
  {  2,   0}, {  1,  14}, {  1,  11}, {  1,   3}, {  1,   9}, {  1,  15}, {  4,   1}, {  1,  18},  /* Row 4 */
  {  1,  14}, {  1,  13}, {  1,   1}, {  1,  15}, {  1,  10}, {  1,   1}, {  1,  10}, {  2,   7},
  {  1,  13}, {  3,   0},
  {  2,   0}, {  1,  19}, {  1,   7}, {  1,  20}, {  1,  10}, {  2,   1}, {  1,  21}, {  1,   1},  /* Row 5 */
  {  1,  18}, {  1,  22}, {  1,  23}, {  1,  14}, {  1,   1}, {  2,  15}, {  1,   1}, {  1,  10},
  {  1,   9}, {  1,   2}, {  1,  11}, {  1,  14}, {  2,   0},
  {  1,   0}, {  1,  14}, {  1,   7}, {  1,   8}, {  1,   9}, {  1,  10}, {  1,   1}, {  1,  18},  /* Row 6 */
  {  1,  23}, {  1,   1}, {  1,  15}, {  1,  14}, {  1,  23}, {  1,  14}, {  1,   1}, {  2,  23},
  {  1,   1}, {  2,   9}, {  1,   8}, {  1,   2}, {  1,   4}, {  2,   0},
  {  1,   0}, {  1,  24}, {  1,   2}, {  1,   8}, {  1,   9}, {  1,   1}, {  1,  13}, {  2,  23},  /* Row 7 */
  {  1,   1}, {  1,  21}, {  1,  22}, {  1,  23}, {  1,  14}, {  1,   1}, {  1,  25}, {  1,  23},
  {  1,  14}, {  1,   1}, {  1,   9}, {  1,  26}, {  1,   2}, {  1,   7}, {  1,  14}, {  1,   0},
  {  1,  22}, {  1,   7}, {  1,   8}, {  2,   9}, {  1,   1}, {  1,   5}, {  1,  23}, {  1,  18},  /* Row 8 */
  {  1,   9}, {  1,   1}, {  1,  14}, {  1,  23}, {  1,  14}, {  1,  11}, {  1,   1}, {  1,  22},
  {  1,  23}, {  1,  21}, {  1,   9}, {  1,  26}, {  1,   8}, {  1,   7}, {  1,   1}, {  1,   0},
  {  1,  13}, {  1,   7}, {  2,   8}, {  1,   9}, {  1,  18}, {  1,  23}, {  1,  14}, {  2,   9},  /* Row 9 */
  {  1,   1}, {  1,  14}, {  1,  23}, {  1,  14}, {  1,  11}, {  1,   9}, {  1,   1}, {  1,  23},
  {  1,  14}, {  1,  10}, {  2,   8}, {  1,   2}, {  1,  24}, {  1,   0},
  {  1,   1}, {  1,   7}, {  2,   8}, {  1,   9}, {  1,  14}, {  1,  23}, {  1,   1}, {  2,   9},  /* Row 10 */
  {  1,   1}, {  1,  14}, {  1,  23}, {  1,  18}, {  1,  11}, {  2,   9}, {  1,  22}, {  1,  23},
  {  1,   1}, {  2,   8}, {  1,   3}, {  1,  27}, {  1,  14},
  {  1,  11}, {  1,   2}, {  2,   8}, {  1,  10}, {  2,  23}, {  1,   9}, {  2,  20}, {  1,   1},  /* Row 11 */
  {  1,  14}, {  1,  23}, {  1,  18}, {  1,  11}, {  2,   9}, {  1,  18}, {  1,  23}, {  1,   1},
  {  2,   8}, {  2,   7}, {  1,  14},
  {  1,  11}, {  1,   2}, {  2,   8}, {  1,  10}, {  1,  23}, {  1,  22}, {  1,   9}, {  2,  26},  /* Row 12 */
  {  1,   1}, {  1,  14}, {  1,  23}, {  1,  18}, {  1,  11}, {  2,   9}, {  1,  18}, {  1,  23},
  {  1,   1}, {  2,   8}, {  2,   7}, {  1,  13},
  {  1,   1}, {  1,   3}, {  2,   8}, {  1,   9}, {  1,  23}, {  1,  22}, {  1,   9}, {  2,   8},  /* Row 13 */
  {  1,   1}, {  1,  22}, {  1,  23}, {  1,  18}, {  2,   8}, {  1,  11}, {  1,  18}, {  1,  23},
  {  1,   1}, {  2,   8}, {  2,   7}, {  1,  14},
  {  1,  13}, {  1,   7}, {  2,   8}, {  1,  11}, {  1,  18}, {  1,  23}, {  1,   1}, {  2,   8},  /* Row 14 */
  {  1,  10}, {  1,  28}, {  1,  22}, {  1,  21}, {  2,   8}, {  1,  11}, {  1,  22}, {  1,  23},
  {  1,   9}, {  2,   8}, {  1,   7}, {  1,  24}, {  1,   0},
  {  1,  14}, {  1,   7}, {  3,   8}, {  1,   1}, {  1,  23}, {  1,  14}, {  1,  11}, {  1,   8},  /* Row 15 */
  {  1,  11}, {  1,  10}, {  1,   1}, {  1,   9}, {  1,   8}, {  1,  29}, {  1,   1}, {  1,  12},
  {  1,  23}, {  1,  11}, {  2,   8}, {  1,   7}, {  1,  11}, {  1,   0},
  {  1,   0}, {  1,   7}, {  1,   3}, {  2,   8}, {  1,   9}, {  1,  28}, {  1,  23}, {  1,  18},  /* Row 16 */
  {  1,  11}, {  5,   8}, {  1,   9}, {  1,  22}, {  1,  23}, {  1,  10}, {  2,   8}, {  1,  30},
  {  1,   7}, {  1,  13}, {  1,   0},
  {  1,   0}, {  1,  17}, {  1,   7}, {  3,   8}, {  1,   1}, {  2,  23}, {  1,  31}, {  1,  15},  /* Row 17 */
  {  2,  26}, {  1,   9}, {  1,  21}, {  1,  22}, {  1,  23}, {  1,  13}, {  1,  29}, {  2,   8},
  {  1,   2}, {  1,   8}, {  2,   0},
  {  2,   0}, {  1,  24}, {  1,   2}, {  3,   8}, {  1,   9}, {  1,  22}, {  4,  23}, {  1,  14},  /* Row 18 */
  {  2,  23}, {  1,   1}, {  1,  29}, {  2,   8}, {  2,   7}, {  1,  13}, {  2,   0},
  {  2,   0}, {  1,  13}, {  1,   7}, {  1,   3}, {  3,   8}, {  1,   9}, {  1,   1}, {  1,  18},  /* Row 19 */
  {  1,  14}, {  2,  23}, {  1,  18}, {  1,  10}, {  4,   8}, {  1,   2}, {  1,  17}, {  3,   0},
  {  3,   0}, {  1,  17}, {  1,   2}, {  1,   7}, {  3,   8}, {  1,  29}, {  1,  11}, {  1,  10},  /* Row 20 */
  {  1,   1}, {  1,   9}, {  1,  11}, {  4,   8}, {  1,   2}, {  1,  27}, {  1,  14}, {  3,   0},
  {  3,   0}, {  1,  14}, {  1,  11}, {  2,   3}, { 10,   8}, {  1,  30}, {  1,  32}, {  1,   7},  /* Row 21 */
  {  1,  13}, {  4,   0},
  {  4,   0}, {  1,  14}, {  1,  24}, {  1,   7}, {  1,   3}, {  8,   8}, {  1,   7}, {  1,  32},  /* Row 22 */
  {  1,  27}, {  1,  13}, {  5,   0},
  {  5,   0}, {  1,  14}, {  1,  17}, {  2,   7}, {  2,   2}, {  2,  33}, {  1,   3}, {  3,   7},  /* Row 23 */
  {  1,  24}, {  1,   6}, {  6,   0},
  {  6,   0}, {  1,  14}, {  1,  13}, {  1,   4}, {  1,   7}, {  4,   2}, {  1,   7}, {  1,  11},  /* Row 24 */
  {  1,  13}, {  1,  14}, {  7,   0},
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_startBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels 
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_startLut,          //          Index 0: Unselected LUT
    g_startLut,          //          Index 1: Selected LUT
  },
  g_startRleEntries      // data   - Pointer to the beginning of the RLE data
};
