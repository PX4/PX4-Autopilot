/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_minimize.cxx
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
#define BITMAP_NLUTCODES 25

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_minimizeLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 9 */
  0x2449b6, 0x24496d, 0x244992, 0x242492, 0x496ddb, 0x0024db, 0x0024b6, 0x002492,  /* Codes 1-8 */
  0x0000b6, 0x2424b6, 0x0024ff, 0x0000db, 0x4949db, 0x496db6, 0x246db6, 0x4949b6,  /* Codes 9-17 */
  0x2449db, 0xb6dbff, 0xb6b6db, 0xdbdbff, 0xdbffff, 0x496dff, 0x246dff, 0x4949ff   /* Codes 17-24 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_minimizeLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0x2256, 0x224d, 0x2252, 0x2132, 0x4b7b, 0x013b, 0x0136, 0x0132, 0x0016, 0x2136,  /* Codes 1-10 */
  0x013f, 0x001b, 0x4a5b, 0x4b76, 0x2376, 0x4a56, 0x225b, 0xb6df, 0xb5bb, 0xdedf,  /* Codes 11-20 */
  0xdfff, 0x4b7f, 0x237f, 0x4a5f                                                   /* Codes 21-24 */
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

static const uint8_t g_minimizeLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0x4a, 0x42, 0x46, 0x30, 0x6e, 0x2e, 0x29, 0x25, 0x14, 0x34, 0x32, 0x18, 0x59,    /* Codes 1-13 */
  0x6a, 0x5f, 0x55, 0x4e, 0xd4, 0xba, 0xdf, 0xf4, 0x72, 0x67, 0x5d                 /* Codes 14-24 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_minimizeLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0x2a, 0x29, 0x2a, 0x26, 0x4f, 0x07, 0x06, 0x06, 0x02, 0x26, 0x07, 0x03, 0x4b,    /* Codes 1-13 */
  0x4e, 0x2e, 0x4a, 0x2b, 0xbb, 0xb7, 0xdb, 0xdf, 0x4f, 0x2f, 0x4b                 /* Codes 14-24 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_minimizeRleEntries[] =
{
  { 25,   0},                                                                                      /* Row 0 */
  { 25,   0},                                                                                      /* Row 1 */
  {  2,   0}, {  1,   1}, {  3,   2}, {  1,   3}, {  1,   4}, {  3,   3}, {  2,   4}, {  4,   3},  /* Row 2 */
  {  1,   4}, {  1,   3}, {  2,   2}, {  1,   1}, {  1,   5}, {  2,   0},
  {  2,   0}, {  1,   2}, {  3,   6}, {  8,   7}, {  2,   8}, {  3,   7}, {  2,   8}, {  1,   4},  /* Row 3 */
  {  1,   1}, {  2,   0},
  {  2,   0}, {  1,   4}, { 11,   6}, {  3,   7}, {  1,   6}, {  4,   7}, {  1,   2}, {  2,   0},  /* Row 4 */
  {  2,   0}, {  1,   7}, { 11,   6}, {  3,   7}, {  1,   6}, {  4,   7}, {  1,   3}, {  2,   0},  /* Row 5 */
  {  2,   0}, {  1,   4}, {  9,   6}, { 10,   7}, {  1,   1}, {  2,   0},                          /* Row 6 */
  {  2,   0}, {  1,   4}, {  9,   6}, {  2,   7}, {  1,   8}, {  7,   7}, {  1,   1}, {  2,   0},  /* Row 7 */
  {  2,   0}, {  1,   4}, {  8,   6}, {  4,   7}, {  1,   8}, {  6,   7}, {  1,   1}, {  2,   0},  /* Row 8 */
  {  2,   0}, {  1,   4}, {  9,   6}, {  8,   7}, {  1,   9}, {  1,   7}, {  1,   1}, {  2,   0},  /* Row 9 */
  {  2,   0}, {  1,  10}, {  9,   6}, {  1,   7}, {  1,   6}, {  6,   7}, {  1,   9}, {  1,   7},  /* Row 10 */
  {  1,   1}, {  2,   0},
  {  2,   0}, {  1,  10}, {  7,   6}, {  1,   7}, {  2,   6}, {  6,   7}, {  2,   9}, {  1,   7},  /* Row 11 */
  {  1,  10}, {  2,   0},
  {  2,   0}, {  1,  10}, {  6,   6}, {  3,   7}, {  1,   6}, {  6,   7}, {  3,   9}, {  1,  10},  /* Row 12 */
  {  2,   0},
  {  2,   0}, {  1,  10}, {  4,   6}, {  8,   7}, {  3,   9}, {  1,   7}, {  3,   9}, {  1,  10},  /* Row 13 */
  {  2,   0},
  {  2,   0}, {  1,   4}, {  1,   7}, {  1,   6}, {  1,   7}, {  2,   6}, {  2,   7}, {  3,   9},  /* Row 14 */
  {  1,   7}, {  6,   9}, {  1,   7}, {  1,   9}, {  1,   4}, {  2,   0},
  {  2,   0}, {  1,  10}, {  2,   6}, {  1,   7}, {  1,   6}, {  3,   7}, {  2,   9}, {  1,   7},  /* Row 15 */
  {  6,   9}, {  1,   7}, {  1,   6}, {  1,   9}, {  1,  10}, {  2,   0},
  {  2,   0}, {  1,  10}, {  1,  11}, {  2,   6}, {  5,   7}, {  1,   6}, {  3,   7}, {  3,   9},  /* Row 16 */
  {  2,   7}, {  2,   9}, {  1,  10}, {  2,   0},
  {  2,   0}, {  1,  10}, {  3,  11}, {  5,   6}, {  3,   9}, {  5,   7}, {  1,   6}, {  1,   7},  /* Row 17 */
  {  1,   9}, {  1,  10}, {  2,   0},
  {  2,   0}, {  1,  10}, {  1,   6}, {  1,  12}, {  1,  13}, {  1,  14}, {  1,   1}, {  2,  15},  /* Row 18 */
  {  1,   1}, {  1,  14}, {  2,  16}, {  3,  14}, {  1,   1}, {  1,  15}, {  1,  17}, {  2,   9},
  {  1,   4}, {  2,   0},
  {  2,   0}, {  1,   4}, {  2,  10}, {  1,  18}, { 13,   0}, {  1,  18}, {  1,  10}, {  1,   4},  /* Row 19 */
  {  1,   2}, {  2,   0},
  {  2,   0}, {  1,   4}, {  1,  10}, {  1,   9}, {  1,  19}, {  3,  20}, {  2,  21}, {  7,  20},  /* Row 20 */
  {  1,  21}, {  1,  18}, {  1,  13}, {  1,   4}, {  1,   2}, {  2,   0},
  {  2,   0}, {  1,   1}, {  3,   6}, {  1,  17}, {  2,  22}, {  1,   5}, {  1,  23}, {  2,  22},  /* Row 21 */
  {  3,   5}, {  2,  22}, {  1,   5}, {  1,  22}, {  1,  24}, {  1,   7}, {  1,   1}, {  2,   0},
  {  2,   0}, {  1,   5}, {  1,   1}, {  1,   6}, {  1,   7}, {  2,  10}, {  3,   7}, {  2,   6},  /* Row 22 */
  {  6,   7}, {  2,  10}, {  1,   1}, {  1,   5}, {  2,   0},
  { 25,   0},                                                                                      /* Row 23 */
  { 25,   0}                                                                                       /* Row 24 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_minimizeBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels 
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_minimizeLut,       //          Index 0: Unselected LUT
    g_minimizeLut,       //          Index 1: Selected LUT
  },
  g_minimizeRleEntries  // data   - Pointer to the beginning of the RLE data
};
