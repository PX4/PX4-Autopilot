/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_calibration.cxx
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

#define BITMAP_NROWS     21
#define BITMAP_NCOLUMNS  21
#define BITMAP_NLUTCODES 17

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_calibrationLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                   /* Code 0 */
  0x202020, 0x404040, 0x808080, 0xfcfcfc, 0x606060, 0x9c9c9c, 0xdcdcdc,  /* Codes 1-7 */
  0xececec, 0xacacac, 0x707070, 0x303030, 0x101010, 0xcccccc, 0x505050,  /* Codes 8-15 */
  0x8c8c8c, 0xbcbcbc,                                                    /* Codes 15-16 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_calibrationLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                     /* Code 0 */
  0x2104, 0x4208, 0x8410, 0xffff, 0x630c, 0x9cf3, 0xdefb, 0xef7d, 0xad75,  /* Codes 1-9 */
  0x738e, 0x3186, 0x1082, 0xce79, 0x528a, 0x8c71, 0xbdf7                   /* Codes 10-16 */
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

static const uint8_t g_calibrationLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                     /* Code 0 */
  0x20, 0x40, 0x80, 0xfc, 0x60, 0x9c, 0xdc, 0xec, 0xac, 0x70, 0x30, 0x10,  /* Codes 1-12 */
  0xcc, 0x50, 0x8c, 0xbc,                                                  /* Codes 13-16 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_calibrationLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                     /* Code 0 */
  0x24, 0x49, 0x92, 0xff, 0x6d, 0x92, 0xdb, 0xff, 0xb6, 0x6d, 0x24, 0x00,  /* Codes 1-12 */
  0xdb, 0x49, 0x92, 0xb7                                                   /* Codes 13-16 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_calibrationRleEntries[] =
{
  { 25,   0},                                                                                      /* Row 0 */
  { 25,   0},                                                                                      /* Row 1 */
  { 12,   0}, {  1,   1}, {  1,   2}, { 11,   0},                                                  /* Row 2 */
  { 12,   0}, {  1,   3}, {  1,   4}, {  6,   0}, {  1,   1}, {  2,   5}, {  2,   0},              /* Row 3 */
  { 12,   0}, {  1,   3}, {  1,   4}, {  2,   0}, {  1,   1}, {  1,   5}, {  1,   6}, {  1,   7},  /* Row 4 */
  {  2,   4}, {  1,   7}, {  2,   0},
  { 12,   0}, {  1,   3}, {  1,   4}, {  1,   6}, {  1,   7}, {  2,   4}, {  1,   8}, {  1,   9},  /* Row 5 */
  {  1,   7}, {  1,   9}, {  3,   0},
  {  8,   0}, {  1,   1}, {  1,   5}, {  1,   6}, {  1,   7}, {  2,   4}, {  1,   8}, {  1,   9},  /* Row 6 */
  {  1,  10}, {  1,  11}, {  1,   0}, {  1,  12}, {  2,  13}, {  3,   0},
  {  4,   0}, {  1,  12}, {  1,  14}, {  1,  15}, {  1,   7}, {  2,   4}, {  1,   8}, {  1,   9},  /* Row 7 */
  {  1,  16}, {  1,   4}, {  5,   0}, {  1,  10}, {  1,   5}, {  1,   3}, {  1,   5}, {  2,   0},
  {  3,   0}, {  1,   1}, {  2,   4}, {  1,   8}, {  1,   9}, {  1,  10}, {  1,  11}, {  2,   0},  /* Row 8 */
  {  1,   3}, {  1,   4}, {  5,   0}, {  1,  13}, {  1,   0}, {  1,   1}, {  1,   9}, {  2,   0},
  {  4,   0}, {  1,  15}, {  1,   4}, {  1,   5}, {  5,   0}, {  1,   3}, {  1,   4}, {  4,   0},  /* Row 9 */
  {  1,  14}, {  1,  15}, {  2,   0}, {  1,  16}, {  1,   1}, {  1,   0},
  {  4,   0}, {  1,   5}, {  1,  10}, {  1,  13}, {  5,   0}, {  1,   3}, {  1,   4}, {  4,   0},  /* Row 10 */
  {  1,   9}, {  1,   1}, {  2,   0}, {  1,   5}, {  1,  10}, {  1,   0},
  {  4,   0}, {  1,  13}, {  1,  12}, {  1,   6}, {  1,   1}, {  4,   0}, {  1,   3}, {  1,   4},  /* Row 11 */
  {  3,   0}, {  1,   1}, {  1,  16}, {  1,   0}, {  1,   6}, {  1,  14}, {  1,  12}, {  1,  13},
  {  1,   0},
  {  3,   0}, {  1,   2}, {  1,   6}, {  1,   0}, {  1,  14}, {  1,  15}, {  4,   0}, {  1,   3},  /* Row 12 */
  {  1,   4}, {  3,   0}, {  1,   3}, {  1,   5}, {  1,   1}, {  1,   8}, {  1,   6}, {  1,   0},
  {  1,   6}, {  1,  11},
  {  3,   0}, {  1,   6}, {  1,  11}, {  2,   0}, {  1,  16}, {  4,   0}, {  1,   3}, {  1,   4},  /* Row 13 */
  {  3,   0}, {  1,  16}, {  1,   0}, {  1,   2}, {  1,   4}, {  1,  16}, {  1,   0}, {  1,   2},
  {  1,   6},
  {  2,   0}, {  1,  12}, {  1,  13}, {  1,  12}, {  1,  16}, {  1,   1}, {  1,  15}, {  1,  14},  /* Row 14 */
  {  3,   0}, {  1,   3}, {  1,   4}, {  2,   0}, {  1,  10}, {  1,  13}, {  1,   3}, {  1,   6},
  {  1,   4}, {  1,   7}, {  2,   3}, {  1,   8},
  {  2,   0}, {  1,  10}, {  1,   5}, {  1,   9}, {  1,   4}, {  1,  16}, {  1,   2}, {  1,   6},  /* Row 15 */
  {  3,   0}, {  1,   3}, {  1,   4}, {  2,   0}, {  1,  11}, {  1,  16}, {  5,   4}, {  1,   8},
  {  1,  15},
  {  2,   0}, {  1,  13}, {  1,   0}, {  3,   4}, {  1,   2}, {  1,  13}, {  1,  12}, {  2,   0},  /* Row 16 */
  {  1,   3}, {  1,   4}, {  4,   0}, {  1,  12}, {  3,   2}, {  1,  11}, {  2,   0},
  {  1,   0}, {  1,  14}, {  1,  15}, {  1,   0}, {  3,   4}, {  1,   2}, {  2,   5}, {  2,   0},  /* Row 17 */
  {  1,   3}, {  1,   4}, { 11,   0},
  {  1,   0}, {  1,  16}, {  1,  13}, {  1,  16}, {  3,   4}, {  2,  13}, {  1,   7}, {  2,   0},  /* Row 18 */
  {  1,   3}, {  1,   4}, { 11,   0},
  {  1,   0}, {  1,   1}, {  1,   6}, {  1,   7}, {  3,   4}, {  1,   7}, {  1,   6}, {  1,  11},  /* Row 19 */
  {  2,   0}, {  1,   3}, {  1,   4}, { 11,   0},
  { 12,   0}, {  1,   3}, {  1,   4}, { 11,   0},                                                  /* Row 20 */
  {  5,   0}, {  1,   2}, { 14,   4}, {  1,   3}, {  4,   0},                                      /* Row 21 */
  {  5,   0}, {  1,   2}, { 14,   4}, {  1,   3}, {  4,   0},                                      /* Row 22 */
  { 25,   0},                                                                                      /* Row 23 */
  { 25,   0},                                                                                      /* Row 24 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_calibrationBitmap =
{
  CONFIG_NXWIDGETS_BPP,    // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,    // fmt    - Color format
  BITMAP_NLUTCODES,        // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,         // width  - Width in pixels 
  BITMAP_NROWS,            // height - Height in rows
  {                        // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_calibrationLut,      //          Index 0: Unselected LUT
    g_calibrationLut,      //          Index 1: Selected LUT
  },
  g_calibrationRleEntries  // data   - Pointer to the beginning of the RLE data
};
