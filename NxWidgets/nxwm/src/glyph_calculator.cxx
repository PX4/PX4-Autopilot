/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_calculator.cxx
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
#define BITMAP_NCOLUMNS  24
#define BITMAP_NLUTCODES 8

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_calculatorNormalLut[BITMAP_NLUTCODES] =
{
  0xfcfcfc, 0xb8bcbc, 0xf8f8f8, 0x6890c8, 0x384c80, 0xe8e8e8, 0x646464, 0x909090  /* Codes 0-7 */
};

static const uint32_t g_calculatorBrightlLut[BITMAP_NLUTCODES] =
{
  0xffffff, 0xc9cccc, 0xf9f9f9, 0x8dabd5, 0x69789f, 0xededed, 0x8a8a8a, 0xababab  /* Codes 0-7 */
};

/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_calculatorNormalLut[BITMAP_NLUTCODES] =
{
  0xffff, 0xbdf7, 0xffdf, 0x6c99, 0x3a70, 0xef5d, 0x632c, 0x9492  /* Codes 0-7 */
};

static const uint16_t g_calculatorBrightlLut[BITMAP_NLUTCODES] =
{
  0xffff, 0xce79, 0xffdf, 0x8d5a, 0x6bd3, 0xef7d, 0x8c51, 0xad55  /* Codes 0-7 */
};

/* 8-bit color lookups.  NOTE:  This is really dumb!  The lookup index is 8-bits and it used
 * to lookup an 8-bit value.  There is no savings in that!  It would be better to just put
 * the 8-bit color/greyscale value in the run-length encoded image and save the cost of these
 * pointless lookups.  But these p;ointless lookups do make the logic compatible with the
 * 16- and 24-bit types.
 */

#elif CONFIG_NXWIDGETS_BPP == 8
#  ifdef CONFIG_NXWIDGETS_GREYSCALE

static const uint8_t g_calculatorNormalLut[BITMAP_NLUTCODES] =
{
  0xfc, 0xba, 0xf8, 0x8a, 0x4b, 0xe8, 0x64, 0x90  /* Codes 0-7 */
};

static const uint8_t g_calculatorBrightlLut[BITMAP_NLUTCODES] =
{
  0xff, 0xcb, 0xf9, 0xa6, 0x77, 0xed, 0x8a, 0xab  /* Codes 0-7 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

static const nxgl_mxpixel_t g_calculatorNormalLut[BITMAP_NLUTCODES] =
{
  0xff, 0xb7, 0xff, 0x73, 0x2a, 0xff, 0x6d, 0x92  /* Codes 0-7 */
};

static const nxgl_mxpixel_t g_calculatorBrightlLut[BITMAP_NLUTCODES] =
{
  0xff, 0xdb, 0xff, 0x97, 0x6e, 0xff, 0x92, 0xb6  /* Codes 0-7 */
};

#  endif /* CONFIG_NXWIDGETS_GREYSCALE */
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_calculatorRleEntries[] =
{
  { 23,   0}, {  1,   1},                                                                          /* Row 0 */
  {  1,   0}, { 21,   1}, {  1,   2}, {  1,   1},                                                  /* Row 1 */
  {  1,   0}, {  1,   1}, { 19,   3}, {  1,   4}, {  1,   0}, {  1,   1},                          /* Row 2 */
  {  1,   0}, {  1,   1}, { 19,   3}, {  1,   4}, {  1,   0}, {  1,   1},                          /* Row 3 */
  {  1,   0}, {  1,   1}, { 19,   3}, {  1,   4}, {  1,   0}, {  1,   1},                          /* Row 4 */
  {  1,   0}, {  1,   1}, { 19,   3}, {  1,   4}, {  1,   0}, {  1,   1},                          /* Row 5 */
  {  1,   0}, {  1,   1}, { 19,   3}, {  1,   4}, {  1,   0}, {  1,   1},                          /* Row 6 */
  {  1,   2}, {  1,   1}, { 20,   4}, {  1,   0}, {  1,   1},                                      /* Row 7 */
  {  1,   2}, {  1,   1}, {  4,   5}, {  1,   6}, {  4,   5}, {  1,   6}, {  4,   5}, {  1,   6},  /* Row 8 */
  {  4,   5}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 9 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 10 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 11 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, { 20,   6}, {  1,   0}, {  1,   1},                                      /* Row 12 */
  {  1,   0}, {  1,   1}, {  4,   5}, {  1,   6}, {  4,   5}, {  1,   6}, {  4,   5}, {  1,   6},  /* Row 13 */
  {  4,   5}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 14 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 15 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 16 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   0}, {  1,   1}, { 20,   6}, {  1,   0}, {  1,   1},                                      /* Row 17 */
  {  1,   2}, {  1,   1}, {  4,   5}, {  1,   6}, {  4,   5}, {  1,   6}, {  4,   5}, {  1,   6},  /* Row 18 */
  {  4,   5}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   2}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 19 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   2}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 20 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   2}, {  1,   1}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6},  /* Row 21 */
  {  1,   5}, {  3,   7}, {  1,   6}, {  1,   5}, {  3,   7}, {  1,   6}, {  1,   0}, {  1,   1},
  {  1,   2}, {  1,   1}, { 20,   6}, {  1,   0}, {  1,   1},                                      /* Row 22 */
  { 23,   0}, {  1,   1},                                                                          /* Row 23 */
  { 24,   1}                                                                                       /* Row 24 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_calculatorBitmap =
{
  CONFIG_NXWIDGETS_BPP,     // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,     // fmt    - Color format
  BITMAP_NLUTCODES,         // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,          // width  - Width in pixels 
  BITMAP_NROWS,             // height - Height in rows
  {                         // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_calculatorNormalLut,  //          Index 0: Unselected LUT
    g_calculatorBrightlLut, //          Index 1: Selected LUT
  },
  g_calculatorRleEntries    // data   - Pointer to the beginning of the RLE data
};
