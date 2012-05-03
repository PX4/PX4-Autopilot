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

#define BITMAP_NROWS     25
#define BITMAP_NCOLUMNS  25
#define BITMAP_NLUTCODES 66

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_stopLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0xdbdbdb, 0xdbb6db, 0xdbb6b6, 0xdb9292, 0xdb926d, 0xdb6d6d, 0xb66d6d, 0xb64949,  /* Codes 1-8 */
  0xb62449, 0xdb246d, 0xdb4949, 0xff496d, 0xffb6b6, 0xffdbff, 0xffdbdb, 0xff9292,  /* Codes 9-16 */
  0xff6d6d, 0xdb6d49, 0xff2449, 0xff246d, 0xdb2449, 0xdbdbb6, 0xff4949, 0xff2424,  /* Codes 17-24 */
  0xffffdb, 0xdbb692, 0xdb2424, 0xff4924, 0xffb6db, 0xffdbb6, 0xff6d49, 0xffb692,  /* Codes 25-32 */
  0xdb4924, 0xb64924, 0xdbdb92, 0xb69292, 0xb62424, 0xdb496d, 0xb6926d, 0xdbffff,  /* Codes 33-40 */
  0xdbffdb, 0xb6496d, 0xdb0024, 0xdb6d92, 0xff2400, 0xb66d49, 0xdb2400, 0xb62400,  /* Codes 41-48 */
  0x922424, 0xb60000, 0x922449, 0xff92b6, 0x920000, 0xdb0049, 0xb60024, 0xdb0000,  /* Codes 49-56 */
  0x922400, 0x924949, 0x924924, 0x920024, 0xff6d92, 0x6d4924, 0x6d2400, 0x6d2424,  /* Codes 57-64 */
  0x6d0000                                                                         /* Code 65 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_stopLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0xdedb, 0xddbb, 0xddb6, 0xdc92, 0xdc8d, 0xdb6d, 0xb36d, 0xb249, 0xb129, 0xd92d,  /* Codes 1-10 */
  0xda49, 0xfa4d, 0xfdb6, 0xfedf, 0xfedb, 0xfc92, 0xfb6d, 0xdb69, 0xf929, 0xf92d,  /* Codes 11-20 */
  0xd929, 0xded6, 0xfa49, 0xf924, 0xfffb, 0xddb2, 0xd924, 0xfa44, 0xfdbb, 0xfed6,  /* Codes 21-30 */
  0xfb69, 0xfdb2, 0xda44, 0xb244, 0xded2, 0xb492, 0xb124, 0xda4d, 0xb48d, 0xdfff,  /* Codes 31-40 */
  0xdffb, 0xb24d, 0xd804, 0xdb72, 0xf920, 0xb369, 0xd920, 0xb120, 0x9124, 0xb000,  /* Codes 41-50 */
  0x9129, 0xfc96, 0x9000, 0xd809, 0xb004, 0xd800, 0x9120, 0x9249, 0x9244, 0x9004,  /* Codes 51-60 */
  0xfb72, 0x6a44, 0x6920, 0x6924, 0x6800                                           /* Codes 61-65 */
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

static const uint8_t g_stopLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0xdb, 0xc5, 0xc1, 0xa7, 0xa3, 0x8d, 0x82, 0x69, 0x53, 0x63, 0x74, 0x83, 0xcb,    /* Codes 1-13 */
  0xe9, 0xe5, 0xb2, 0x98, 0x89, 0x69, 0x6d, 0x5e, 0xd6, 0x7f, 0x65, 0xfa, 0xbc,    /* Codes 14-26 */
  0x5a, 0x7b, 0xd0, 0xe1, 0x94, 0xc7, 0x70, 0x65, 0xd2, 0x9c, 0x4f, 0x78, 0x98,    /* Codes 27-39 */
  0xf4, 0xf0, 0x6d, 0x45, 0x92, 0x61, 0x7e, 0x56, 0x4b, 0x44, 0x36, 0x49, 0xb6,    /* Codes 40-52 */
  0x2b, 0x49, 0x3a, 0x41, 0x40, 0x5e, 0x5a, 0x2f, 0x9c, 0x4f, 0x35, 0x39, 0x20     /* Codes 53-65 */

};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_stopLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXTK_BORDERCOLOR1,                                                        /* Code 0 */
  0xdb, 0xd7, 0xd6, 0xd2, 0xd1, 0xcd, 0xad, 0xa9, 0xa5, 0xc5, 0xc9, 0xe9, 0xf6,    /* Codes 1-13 */
  0xfb, 0xfb, 0xf2, 0xed, 0xcd, 0xe5, 0xe5, 0xc5, 0xda, 0xe9, 0xe4, 0xff, 0xd6,    /* Codes 14-26 */
  0xc4, 0xe8, 0xf7, 0xfa, 0xed, 0xf6, 0xc8, 0xa8, 0xda, 0xb2, 0xa4, 0xc9, 0xb1,    /* Codes 27-39 */
  0xdf, 0xdf, 0xa9, 0xc0, 0xce, 0xe4, 0xad, 0xc4, 0xa4, 0x84, 0xa0, 0x85, 0xf2,    /* Codes 40-52 */
  0x80, 0xc1, 0xa0, 0xc0, 0x84, 0x89, 0x88, 0x80, 0xee, 0x68, 0x64, 0x64, 0x60     /* Codes 53-65 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_stopRleEntries[] =
{
  { 25,   0},                                                                                      /* Row 0 */
  { 25,   0},                                                                                      /* Row 1 */
  {  2,   0}, {  1,   1}, {  1,   2}, {  1,   3}, {  1,   2}, {  2,   3}, {  2,   4}, {  1,   5},  /* Row 2 */
  {  2,   6}, {  1,   7}, {  2,   8}, {  1,   9}, {  1,  10}, {  1,  11}, {  1,   8}, {  1,  11},
  {  1,  12}, {  1,  13}, {  2,   0},
  {  2,   0}, {  1,   1}, {  1,  14}, {  2,  15}, {  1,   1}, {  1,   3}, {  1,  13}, {  2,  16},  /* Row 3 */
  {  2,  17}, {  1,  18}, {  1,  11}, {  2,  19}, {  1,  20}, {  1,  19}, {  1,  21}, {  2,  19},
  {  1,  17}, {  2,   0},
  {  2,   0}, {  1,   2}, {  1,  14}, {  2,  15}, {  1,  22}, {  1,   3}, {  1,   4}, {  2,  17},  /* Row 4 */
  {  3,  23}, {  3,  24}, {  2,  21}, {  1,  23}, {  1,  19}, {  1,  24}, {  1,   8}, {  2,   0},
  {  2,   0}, {  1,   3}, {  2,  15}, {  1,   0}, {  2,  25}, {  1,  26}, {  1,   6}, {  1,  11},  /* Row 5 */
  {  2,  23}, {  1,  24}, {  2,  27}, {  1,  28}, {  2,  15}, {  1,   0}, {  1,  17}, {  1,  23},
  {  1,   8}, {  2,   0},
  {  2,   0}, {  1,   3}, {  1,  29}, {  1,  13}, {  1,  30}, {  1,   0}, {  1,  25}, {  1,  22},  /* Row 6 */
  {  1,   5}, {  1,  11}, {  2,  27}, {  1,  24}, {  1,  27}, {  1,  31}, {  1,  32}, {  1,  25},
  {  1,   0}, {  1,  26}, {  1,  33}, {  1,  27}, {  1,  34}, {  2,   0},
  {  2,   0}, {  1,   3}, {  1,  13}, {  2,  16}, {  1,  30}, {  1,   0}, {  1,  25}, {  1,  26},  /* Row 7 */
  {  1,   8}, {  2,  21}, {  1,  27}, {  1,  11}, {  1,  26}, {  1,  25}, {  1,   0}, {  1,  35},
  {  1,  34}, {  1,  33}, {  1,  27}, {  1,   8}, {  2,   0},
  {  2,   0}, {  1,  36}, {  2,  16}, {  2,   6}, {  1,  13}, {  1,   0}, {  1,  25}, {  1,   4},  /* Row 8 */
  {  3,   8}, {  1,   4}, {  1,  25}, {  1,   0}, {  1,  13}, {  1,   8}, {  1,  37}, {  2,  24},
  {  1,  37}, {  2,   0},
  {  2,   0}, {  1,  36}, {  1,   4}, {  1,  17}, {  1,  12}, {  1,  11}, {  1,  38}, {  1,  15},  /* Row 9 */
  {  2,   0}, {  2,  36}, {  1,   7}, {  1,  15}, {  1,   0}, {  1,  15}, {  1,  23}, {  1,  27},
  {  3,  24}, {  1,  27}, {  2,   0},
  {  2,   0}, {  1,  39}, {  1,   6}, {  2,  23}, {  1,  19}, {  1,  37}, {  1,  38}, {  1,  15},  /* Row 10 */
  {  1,   0}, {  2,  40}, {  1,  41}, {  1,   0}, {  1,  29}, {  1,  42}, {  1,  43}, {  1,  27},
  {  3,  24}, {  1,  27}, {  2,   0},
  {  2,   0}, {  1,   7}, {  1,  18}, {  1,  21}, {  2,  24}, {  1,  21}, {  1,   9}, {  1,  44},  /* Row 11 */
  {  1,   0}, {  3,  40}, {  1,   0}, {  1,  44}, {  1,   9}, {  3,  27}, {  1,  45}, {  1,  24},
  {  1,  37}, {  2,   0},
  {  2,   0}, {  1,   7}, {  1,  18}, {  1,  21}, {  2,  24}, {  1,  21}, {  1,   9}, {  1,  44},  /* Row 12 */
  {  1,   0}, {  3,  40}, {  1,   0}, {  1,  44}, {  1,   9}, {  3,  27}, {  1,  45}, {  1,  24},
  {  1,  37}, {  2,   0},
  {  2,   0}, {  1,  46}, {  1,  11}, {  4,  27}, {  1,   9}, {  1,  44}, {  5,   0}, {  1,  44},  /* Row 13 */
  {  1,   9}, {  1,  27}, {  2,  37}, {  1,  47}, {  1,  27}, {  1,  37}, {  2,   0},
  {  2,   0}, {  1,   8}, {  3,  27}, {  1,  48}, {  1,  37}, {  1,   6}, {  1,  15}, {  1,   0},  /* Row 14 */
  {  3,  15}, {  1,   0}, {  1,  15}, {  1,   7}, {  1,  37}, {  1,  49}, {  1,  37}, {  1,  48},
  {  1,  27}, {  1,  37}, {  2,   0},
  {  2,   0}, {  1,   8}, {  1,  37}, {  1,  27}, {  2,  48}, {  1,  46}, {  1,  15}, {  1,   0},  /* Row 15 */
  {  1,  15}, {  3,   7}, {  1,  15}, {  1,   0}, {  1,  30}, {  1,  23}, {  1,  37}, {  2,  48},
  {  1,  27}, {  1,  37}, {  2,   0},
  {  2,   0}, {  1,  49}, {  1,  37}, {  1,  27}, {  1,  50}, {  1,  34}, {  1,  30}, {  1,   0},  /* Row 16 */
  {  1,  25}, {  1,   7}, {  2,  51}, {  1,   9}, {  1,   7}, {  1,  25}, {  1,   0}, {  1,  52},
  {  1,  27}, {  1,  53}, {  1,  47}, {  1,  27}, {  1,  49}, {  2,   0},
  {  2,   0}, {  1,  37}, {  1,  54}, {  1,  55}, {  1,   9}, {  1,  30}, {  1,   0}, {  1,  15},  /* Row 17 */
  {  1,  17}, {  1,  56}, {  2,  43}, {  1,  37}, {  1,  57}, {  1,  17}, {  1,  15}, {  1,   0},
  {  1,  30}, {  1,  58}, {  1,  53}, {  1,  37}, {  1,  58}, {  2,   0},
  {  2,   0}, {  1,   9}, {  1,  53}, {  1,  37}, {  1,  32}, {  1,   0}, {  1,  25}, {  1,   5},  /* Row 18 */
  {  1,  37}, {  1,  55}, {  2,  50}, {  1,  53}, {  1,  49}, {  1,  59}, {  1,   5}, {  1,  25},
  {  1,   0}, {  1,  16}, {  1,   9}, {  1,  60}, {  1,  49}, {  2,   0},
  {  2,   0}, {  1,  49}, {  1,  60}, {  1,  61}, {  1,   0}, {  1,  25}, {  1,  30}, {  1,  62},  /* Row 19 */
  {  1,  57}, {  1,  53}, {  2,  50}, {  1,  57}, {  1,  63}, {  1,  57}, {  1,  49}, {  1,  30},
  {  1,  15}, {  1,   0}, {  1,  61}, {  1,  37}, {  1,  49}, {  2,   0},
  {  2,   0}, {  1,  49}, {  1,  55}, {  1,  11}, {  1,   8}, {  1,  58}, {  1,  64}, {  1,  65},  /* Row 20 */
  {  1,  55}, {  1,  50}, {  2,  55}, {  1,  37}, {  1,  53}, {  1,  37}, {  1,  53}, {  1,  64},
  {  1,   8}, {  1,  11}, {  1,  21}, {  1,  60}, {  1,  51}, {  2,   0},
  {  2,   0}, {  1,  38}, {  1,  55}, {  1,  60}, {  2,  49}, {  1,  65}, {  1,  60}, {  1,  50},  /* Row 21 */
  {  1,  43}, {  3,  50}, {  1,  55}, {  2,  50}, {  1,  64}, {  1,  49}, {  2,  60}, {  1,  37},
  {  1,  38}, {  2,   0},
  {  2,   0}, {  1,  29}, {  1,  38}, {  2,  49}, {  2,  64}, {  1,  49}, {  2,  55}, {  2,  37},  /* Row 22 */
  {  3,  49}, {  1,  37}, {  1,  64}, {  1,  58}, {  2,  49}, {  1,   6}, {  1,  13}, {  2,   0},
  { 25,   0},                                                                                      /* Row 23 */
  { 25,   0}                                                                                       /* Row 24 */
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
    g_stopLut,           //          Index 0: Unselected LUT
    g_stopLut,           //          Index 1: Selected LUT
  },
  g_stopRleEntries       // data   - Pointer to the beginning of the RLE data
};
