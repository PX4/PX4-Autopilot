/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_stop.cxx
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
#define BITMAP_NLUTCODES 11

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_stopNormalLut[BITMAP_NLUTCODES] =
{
  0xd8d8d8, 0xfc6c6c, 0xfcb4b4, 0xd80000, 0xfc0000, 0x902424, 0xfcfcfc, 0xd8fcfc,  /* Codes 0-7 */
  0xb40000, 0xb40024, 0xfcd8d8                                                     /* Codes 8-10 */
};

static const uint32_t g_stopBrightLut[BITMAP_NLUTCODES] =
{
  0xe1e1e1, 0xfc9090, 0xfcc6c6, 0xe13f3f, 0xfc3f3f, 0xab5a5a, 0xfcfcfc, 0xe1fcfc,  /* Codes 0-7 */
  0xc63f3f, 0xc63f5a, 0xfce1e1                                                     /* Codes 8-10 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_stopNormalLut[BITMAP_NLUTCODES] =
{
  0xdedb, 0xfb6d, 0xfdb6, 0xd800, 0xf800, 0x9124, 0xffff, 0xdfff, 0xb000, 0xb004,  /* Codes 0-9 */
  0xfedb                                                                           /* Codes 10-10 */
};

static const uint16_t g_stopBrightLut[BITMAP_NLUTCODES] =
{
  0xe71c, 0xfc92, 0xfe38, 0xe1e7, 0xf9e7, 0xaacb, 0xffff, 0xe7ff, 0xc1e7, 0xc1eb,  /* Codes 0-9 */
  0xff1c                                                                           /* Codes 10-10 */
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

static const uint8_t g_stopNormalLut[BITMAP_NLUTCODES] =
{
  0xd8, 0x97, 0xc9, 0x40, 0x4b, 0x44, 0xfc, 0xf1, 0x35, 0x39, 0xe2   /* Codes 0-10 */
};

static const uint8_t g_stopBrightLut[BITMAP_NLUTCODES] =
{
  0xe1, 0xb0, 0xd6, 0x6f, 0x77, 0x72, 0xfc, 0xf3, 0x67, 0x6a, 0xe9  /* Codes 0-10 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_stopNormalLut[BITMAP_NLUTCODES] =
{
  0xdb, 0xed, 0xf6, 0xc0, 0xe0, 0x84, 0xff, 0xdf, 0xa0, 0xa0, 0xfb   /* Codes 0-10 */
};

static const uint8_t g_stopBrightLut[BITMAP_NLUTCODES] =
{
  0xff, 0xf2, 0xfb, 0xe4, 0xe4, 0xa9, 0xff, 0xff, 0xc4, 0xc5, 0xff  /* Codes 0-10 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_stopRleEntries[] =
{
  {  1,   0}, {  1,   1}, { 17,   2}, {  1,   1}, {  1,   2},                                      /* Row 0 */
  {  1,   1}, {  4,   3}, {  6,   4}, {  9,   3}, {  1,   1},                                      /* Row 1 */
  {  1,   2}, {  2,   3}, {  9,   4}, {  8,   3}, {  1,   5},                                      /* Row 2 */
  {  1,   2}, {  1,   3}, {  1,   4}, {  2,   6}, {  1,   7}, {  7,   4}, {  2,   3}, {  2,   6},  /* Row 3 */
  {  1,   7}, {  2,   3}, {  1,   5},
  {  1,   2}, {  1,   3}, {  1,   4}, {  3,   6}, {  1,   7}, {  5,   4}, {  2,   3}, {  3,   6},  /* Row 4 */
  {  1,   7}, {  2,   3}, {  1,   5},
  {  1,   2}, {  3,   4}, {  3,   6}, {  1,   7}, {  4,   4}, {  1,   3}, {  3,   6}, {  1,   7},  /* Row 5 */
  {  3,   3}, {  1,   5},
  {  1,   2}, {  4,   4}, {  3,   6}, {  1,   7}, {  2,   4}, {  1,   3}, {  3,   6}, {  1,   7},  /* Row 6 */
  {  4,   3}, {  1,   5},
  {  1,   2}, {  5,   4}, {  3,   6}, {  1,   7}, {  1,   3}, {  3,   6}, {  1,   7}, {  4,   3},  /* Row 7 */
  {  1,   8}, {  1,   5},
  {  1,   2}, {  6,   4}, {  6,   6}, {  1,   7}, {  5,   3}, {  1,   8}, {  1,   5},              /* Row 8 */
  {  1,   2}, {  1,   3}, {  4,   4}, {  2,   3}, {  2,   6}, {  3,   7}, {  6,   3}, {  1,   8},  /* Row 9 */
  {  1,   5},
  {  1,   2}, {  2,   3}, {  1,   4}, {  4,   3}, {  2,   6}, {  3,   7}, {  5,   3}, {  2,   8},  /* Row 10 */
  {  1,   5},
  {  1,   2}, {  7,   3}, {  4,   6}, {  1,   7}, {  5,   3}, {  2,   8}, {  1,   5},              /* Row 11 */
  {  1,   2}, {  6,   3}, {  6,   6}, {  1,   7}, {  3,   3}, {  3,   8}, {  1,   5},              /* Row 12 */
  {  1,   2}, {  5,   3}, {  2,   6}, {  2,   7}, {  1,   3}, {  2,   6}, {  2,   7}, {  1,   3},  /* Row 13 */
  {  4,   8}, {  1,   5},
  {  1,   2}, {  4,   3}, {  3,   6}, {  1,   7}, {  3,   3}, {  3,   6}, {  1,   7}, {  4,   8},  /* Row 14 */
  {  1,   5},
  {  1,   2}, {  3,   3}, {  3,   6}, {  1,   7}, {  5,   3}, {  3,   6}, {  1,   7}, {  3,   8},  /* Row 15 */
  {  1,   5},
  {  1,   2}, {  2,   3}, {  3,   6}, {  1,   7}, {  4,   3}, {  3,   8}, {  3,   6}, {  1,   7},  /* Row 16 */
  {  2,   8}, {  1,   5},
  {  1,   2}, {  2,   3}, {  3,   7}, {  3,   3}, {  6,   8}, {  3,   7}, {  2,   8}, {  1,   5},  /* Row 17 */
  {  1,   2}, {  6,   3}, {  2,   8}, {  1,   9}, { 10,   8}, {  1,   5},                          /* Row 18 */
  {  1,   1}, {  1,   3}, { 10,   8}, {  1,   9}, {  8,   8},                                      /* Row 19 */
  {  1,  10}, {  1,   1}, { 17,   5}, {  1,   8}, {  1,   1},                                      /* Row 20 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_stopBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels 
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_stopNormalLut,     //          Index 0: Unselected LUT
    g_stopBrightLut,     //          Index 1: Selected LUT
  },
  g_stopRleEntries       // data   - Pointer to the beginning of the RLE data
};
