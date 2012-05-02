/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_nsh.cxx
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
#define BITMAP_NLUTCODES 36

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_nshLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0xffffdb, 0x4949db, 0x2424db, 0x4924db, 0xdbdbff, 0x6d6ddb, 0x6d92db, 0x2424b6,  /* Codes 1-8 */
  0xffdbdb, 0xdbffff, 0x4992db, 0x24246d, 0xffffff, 0x6db6db, 0x242492, 0x4924ff,  /* Codes 9-16 */
  0x49b6db, 0x6d6db6, 0x92b6db, 0xb6dbdb, 0xb6dbff, 0x2424ff, 0x9292db, 0x9292b6,  /* Codes 17-24 */
  0x00006d, 0x9292ff, 0x240092, 0x24006d, 0x4949ff, 0x494992, 0x92b6ff, 0xb6b6db,  /* Codes 25-32 */
  0x4924b6, 0x92dbdb, 0x926ddb                                                     /* Codes 33-35 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_nshLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0xfffb, 0x4a5b, 0x213b, 0x493b, 0xdedf, 0x6b7b, 0x6c9b, 0x2136, 0xfedb, 0xdfff,  /* Codes 1-10 */
  0x4c9b, 0x212d, 0xffff, 0x6dbb, 0x2132, 0x493f, 0x4dbb, 0x6b76, 0x95bb, 0xb6db,  /* Codes 11-20 */
  0xb6df, 0x213f, 0x949b, 0x9496, 0x000d, 0x949f, 0x2012, 0x200d, 0x4a5f, 0x4a52,  /* Codes 21-30 */
  0x95bf, 0xb5bb, 0x4936, 0x96db, 0x937b                                           /* Codes 31-35 */
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

static const uint8_t g_nshLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0xfa, 0x59, 0x38, 0x43, 0xdf, 0x79, 0x8f, 0x34, 0xe5, 0xf4, 0x84, 0x2c, 0xff,    /* Codes 1-13 */
  0xa4, 0x30, 0x48, 0x99, 0x75, 0xaf, 0xcf, 0xd4, 0x3c, 0x9a, 0x96, 0x0c, 0x9e,    /* Codes 14-16 */
  0x1b, 0x17, 0x5d, 0x51, 0xb3, 0xba, 0x3f, 0xc5, 0x84                             /* Codes 27-35 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_nshLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                             /* Code 0 */
  0xff, 0x4b, 0x27, 0x47, 0xdb, 0x6f, 0x73, 0x26, 0xfb, 0xdf, 0x53, 0x25, 0xff,    /* Codes 1-13 */
  0x77, 0x26, 0x47, 0x57, 0x6e, 0x97, 0xbb, 0xbb, 0x27, 0x93, 0x92, 0x01, 0x93,    /* Codes 14-16 */
  0x22, 0x21, 0x4b, 0x4a, 0x97, 0xb7, 0x46, 0x9b, 0x8f                             /* Codes 27-35 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_nshRleEntries[] =
{
  { 11,   0}, {  1,   1}, {  1,   2}, {  1,   1}, { 11,   0},                                      /* Row 0 */
  { 11,   0}, {  3,   3}, { 11,   0},                                                              /* Row 1 */
  {  9,   0}, {  4,   3}, {  1,   4}, {  2,   3}, {  9,   0},                                      /* Row 2 */
  {  4,   0}, {  1,   5}, {  4,   0}, {  2,   3}, {  1,   6}, {  1,   7}, {  1,   8}, {  1,   4},  /* Row 3 */
  {  1,   3}, {  6,   0}, {  1,   9}, {  2,   0},
  {  3,   0}, {  1,   5}, {  1,  10}, {  1,  11}, {  2,   0}, {  1,   3}, {  1,   4}, {  1,   3},  /* Row 4 */
  {  2,  10}, {  1,  12}, {  1,   3}, {  1,   4}, {  1,   3}, {  3,   0}, {  1,  13}, {  1,   5},
  {  1,  14}, {  2,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  10}, {  1,  15}, {  1,  16}, {  2,   4}, {  1,   3},  /* Row 5 */
  {  1,  10}, {  1,  13}, {  1,  10}, {  1,  12}, {  1,   3}, {  1,   4}, {  1,   3}, {  1,   2},
  {  1,  10}, {  1,   5}, {  1,  17}, {  1,  15}, {  2,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  10}, {  1,  18}, {  1,   8}, {  2,   4}, {  1,   3},  /* Row 6 */
  {  1,  10}, {  2,  13}, {  1,  19}, {  1,  12}, {  1,   4}, {  1,   3}, {  1,  19}, {  1,  13},
  {  1,  20}, {  1,  15}, {  1,   9}, {  2,   0},
  {  3,   0}, {  1,  10}, {  2,  13}, {  1,  10}, {  1,  12}, {  1,  16}, {  1,   4}, {  1,   3},  /* Row 7 */
  {  1,  10}, {  1,   5}, {  1,  13}, {  1,  10}, {  1,  12}, {  1,   4}, {  1,   2}, {  1,  10},
  {  1,  13}, {  1,  17}, {  1,  15}, {  3,   0},
  {  3,   0}, {  1,  10}, {  2,  13}, {  1,  10}, {  1,  18}, {  1,   8}, {  1,   4}, {  1,   3},  /* Row 8 */
  {  1,  10}, {  1,  21}, {  2,  13}, {  1,  19}, {  1,  22}, {  1,  23}, {  2,  13}, {  1,  15},
  {  1,   1}, {  3,   0},
  {  2,   0}, {  1,   2}, {  2,  13}, {  1,   5}, {  1,  13}, {  1,  10}, {  1,  18}, {  1,   4},  /* Row 9 */
  {  1,   3}, {  1,  10}, {  1,   5}, {  1,  24}, {  2,  13}, {  1,  24}, {  2,  13}, {  1,  19},
  {  1,  16}, {  1,   3}, {  1,   2}, {  2,   0},
  {  2,   0}, {  1,   2}, {  2,  13}, {  1,  18}, {  2,  13}, {  1,  10}, {  1,  16}, {  1,   3},  /* Row 10 */
  {  2,  10}, {  1,  12}, {  1,  10}, {  1,  13}, {  1,  10}, {  1,  13}, {  1,  10}, {  1,  12},
  {  2,   4}, {  1,   3}, {  1,   1}, {  1,   0},
  {  1,   1}, {  1,   3}, {  1,   2}, {  2,  13}, {  1,  25}, {  1,  10}, {  1,  13}, {  1,  10},  /* Row 11 */
  {  1,  15}, {  1,   3}, {  2,  10}, {  1,  25}, {  1,  26}, {  1,  13}, {  1,  10}, {  1,  13},
  {  1,  19}, {  1,  12}, {  3,   4}, {  1,   2}, {  1,   1},
  {  1,   2}, {  1,   3}, {  1,   2}, {  2,  13}, {  1,  15}, {  1,   3}, {  1,  10}, {  1,  13},  /* Row 12 */
  {  1,  18}, {  1,  27}, {  2,  10}, {  1,  28}, {  1,  22}, {  1,  23}, {  1,  13}, {  1,  20},
  {  1,  25}, {  1,  16}, {  3,   4}, {  1,   3}, {  1,   2},
  {  1,   1}, {  1,   3}, {  1,   2}, {  2,  13}, {  1,  15}, {  1,   3}, {  1,  23}, {  1,  13},  /* Row 13 */
  {  1,  10}, {  1,  25}, {  2,  10}, {  1,  25}, {  1,  29}, {  1,  10}, {  1,  13}, {  1,  10},
  {  1,  30}, {  1,   8}, {  3,   4}, {  1,   3}, {  1,   1},
  {  2,   0}, {  1,   2}, {  2,  13}, {  1,  15}, {  1,   4}, {  1,   3}, {  2,  10}, {  1,  18},  /* Row 14 */
  {  1,   5}, {  1,  10}, {  1,  25}, {  1,  31}, {  1,  13}, {  1,  10}, {  1,  13}, {  1,  32},
  {  1,  28}, {  2,   4}, {  1,   3}, {  2,   0},
  {  2,   0}, {  1,   2}, {  2,  13}, {  1,  15}, {  1,   4}, {  1,   3}, {  1,  23}, {  1,  13},  /* Row 15 */
  {  1,   5}, {  1,  21}, {  1,  10}, {  1,  12}, {  1,  10}, {  1,  13}, {  1,   5}, {  1,  13},
  {  1,  10}, {  1,  30}, {  1,   4}, {  2,   3}, {  2,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  15}, {  2,   4}, {  1,   3}, {  2,  13}, {  2,  21},  /* Row 16 */
  {  1,  10}, {  1,  13}, {  1,   5}, {  1,   3}, {  1,  10}, {  1,  13}, {  1,  10}, {  1,  33},
  {  4,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  15}, {  3,   4}, {  1,  10}, {  1,  13}, {  2,  21},  /* Row 17 */
  {  2,  13}, {  1,  18}, {  1,   3}, {  1,   6}, {  2,  13}, {  1,  15}, {  1,   1}, {  3,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  15}, {  3,   4}, {  1,  23}, {  1,  13}, {  1,  10},  /* Row 18 */
  {  1,   5}, {  1,  13}, {  1,   5}, {  1,  28}, {  1,   4}, {  1,   3}, {  1,   5}, {  1,  13},
  {  1,  14}, {  1,  15}, {  3,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  15}, {  1,   4}, {  1,   3}, {  1,   4}, {  1,   3},  /* Row 19 */
  {  1,  10}, {  3,  13}, {  1,  18}, {  1,  15}, {  1,   4}, {  1,   3}, {  1,   6}, {  1,  10},
  {  1,  21}, {  1,  15}, {  1,   9}, {  2,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  15}, {  1,   1}, {  1,   0}, {  1,   3}, {  1,   4},  /* Row 20 */
  {  1,   3}, {  1,  10}, {  1,  13}, {  1,  18}, {  1,  15}, {  1,   4}, {  1,   2}, {  1,   1},
  {  2,   0}, {  1,  13}, {  1,  34}, {  1,  15}, {  2,   0},
  {  3,   0}, {  1,  10}, {  1,  13}, {  1,  15}, {  1,   9}, {  2,   0}, {  2,   3}, {  1,  23},  /* Row 21 */
  {  1,  20}, {  1,  25}, {  1,   4}, {  1,   3}, {  1,   1}, {  3,   0}, {  1,   5}, {  1,  14},
  {  1,  15}, {  2,   0},
  {  3,   0}, {  1,   9}, {  2,  15}, {  1,   9}, {  2,   0}, {  3,   3}, {  1,  12}, {  1,   8},  /* Row 22 */
  {  1,   3}, {  1,   2}, {  4,   0}, {  1,  14}, {  1,  15}, {  1,  20}, {  2,   0},
  { 11,   0}, {  2,   3}, {  1,   6}, {  1,   1}, { 10,   0},                                      /* Row 23 */
  { 11,   0}, {  1,   1}, {  1,  35}, {  1,   1}, { 11,   0},                                      /* Row 24 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_nshBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels 
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_nshLut,            //          Index 0: Unselected LUT
    g_nshLut,            //          Index 1: Selected LUT
  },
  g_nshRleEntries        // data   - Pointer to the beginning of the RLE data
};
