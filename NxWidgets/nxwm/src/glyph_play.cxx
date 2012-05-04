/********************************************************************************************
 * NxWidgets/nxwm/src/glyph_play.cxx
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
#define BITMAP_NLUTCODES 6

/********************************************************************************************
 * Private Bitmap Data
 ********************************************************************************************/

using namespace NxWM;

/* RGB24 (8-8-8) Colors */

#if CONFIG_NXWIDGETS_BPP == 24 ||  CONFIG_NXWIDGETS_BPP == 32

static const uint32_t g_playNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x00d800, 0x008400, 0x006800, 0x00ac00, 0x004800,                   /* Codes 1-5 */
};

static const uint32_t g_playBrightlLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x3fe13f, 0x3fa23f, 0x3f8d3f, 0x3fc03f, 0x3f753f                    /* Codes 1-5 */
};
/* RGB16 (565) Colors (four of the colors in this map are duplicates) */

#elif CONFIG_NXWIDGETS_BPP == 16

static const uint16_t g_playNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x06c0, 0x0420, 0x0340, 0x0560, 0x0240,                             /* Codes 0-5 */
};

static const uint16_t g_playBrightlLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x3f07, 0x3d07, 0x3c67, 0x3e07, 0x3ba7,                             /* Codes 0-5 */
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

static const uint8_t g_playNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x7e, 0x4d, 0x3d, 0x64, 0x2a                                        /* Codes 1-5 */
};

static const uint8_t g_playBrightlLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x9e, 0x79, 0x6c, 0x8a, 0x5e                                        /* Codes 1-5 */
};

#  else /* CONFIG_NXWIDGETS_GREYSCALE */

/* RGB8 (332) Colors */

static const nxgl_mxpixel_t g_playNormalLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x18, 0x10, 0x0c, 0x14, 0x08                                        /* Codes 0-5 */
};

static const nxgl_mxpixel_t g_playBrightlLut[BITMAP_NLUTCODES] =
{
  CONFIG_NXWM_DEFAULT_BACKGROUNDCOLOR,                                /* Code 0 */
  0x3c, 0x34, 0x30, 0x38, 0x2c                                        /* Codes 0-5 */
};

#  endif
#else
# error "Unsupport pixel format"
#endif

static const struct NXWidgets::SRlePaletteBitmapEntry g_playRleEntries[] =
{
  { 25,   0},                                                                                      /* Row 0 */
  {  9,   0}, {  1,   1}, {  1,   2}, {  3,   3}, {  1,   2}, {  1,   1}, {  9,   0},              /* Row 1 */
  {  7,   0}, {  1,   1}, {  1,   3}, {  2,   2}, {  3,   1}, {  1,   4}, {  1,   3}, {  1,   5},  /* Row 2 */
  {  1,   2}, {  7,   0},
  {  6,   0}, {  1,   1}, {  1,   5}, {  1,   2}, {  7,   0}, {  1,   1}, {  1,   5}, {  1,   3},  /* Row 3 */
  {  6,   0},
  {  4,   0}, {  1,   1}, {  1,   2}, {  1,   5}, {  1,   4}, {  9,   0}, {  1,   1}, {  1,   2},  /* Row 4 */
  {  1,   5}, {  1,   3}, {  4,   0},
  {  3,   0}, {  1,   1}, {  1,   2}, {  1,   3}, {  1,   4}, {  1,   0}, {  1,   1}, { 10,   0},  /* Row 5 */
  {  1,   4}, {  1,   5}, {  1,   2}, {  3,   0},
  {  3,   0}, {  1,   3}, {  1,   5}, {  3,   0}, {  2,   1}, { 10,   0}, {  2,   3}, {  1,   4},  /* Row 6 */
  {  2,   0},
  {  2,   0}, {  1,   1}, {  1,   5}, {  1,   4}, {  3,   0}, {  1,   1}, {  1,   4}, {  2,   1},  /* Row 7 */
  {  8,   0}, {  1,   1}, {  1,   5}, {  1,   2}, {  2,   0},
  {  2,   0}, {  1,   3}, {  1,   2}, {  4,   0}, {  1,   1}, {  5,   4}, {  7,   0}, {  1,   1},  /* Row 8 */
  {  1,   5}, {  1,   2}, {  1,   0},
  {  1,   0}, {  1,   1}, {  1,   3}, {  1,   4}, {  4,   0}, {  1,   1}, {  7,   4}, {  6,   0},  /* Row 9 */
  {  1,   2}, {  1,   3}, {  1,   0},
  {  1,   1}, {  1,   3}, {  1,   5}, {  5,   0}, {  1,   1}, {  9,   4}, {  4,   0}, {  1,   4},  /* Row 10 */
  {  1,   2}, {  1,   4},
  {  1,   1}, {  1,   3}, {  1,   4}, {  5,   0}, {  1,   1}, { 11,   4}, {  2,   0}, {  1,   4},  /* Row 11 */
  {  1,   2}, {  1,   4},
  {  1,   1}, {  1,   5}, {  1,   1}, {  5,   0}, {  1,   4}, {  9,   2}, {  2,   3}, {  1,   5},  /* Row 12 */
  {  1,   0}, {  1,   4}, {  1,   2}, {  1,   4},
  {  1,   1}, {  1,   3}, {  1,   1}, {  5,   0}, {  1,   4}, {  7,   2}, {  3,   3}, {  1,   5},  /* Row 13 */
  {  2,   0}, {  1,   4}, {  1,   2}, {  1,   4},
  {  1,   1}, {  1,   3}, {  1,   4}, {  5,   0}, {  1,   4}, {  5,   2}, {  3,   3}, {  1,   5},  /* Row 14 */
  {  4,   0}, {  1,   2}, {  1,   3}, {  1,   4},
  {  1,   0}, {  1,   2}, {  1,   3}, {  5,   0}, {  1,   4}, {  3,   2}, {  3,   3}, {  1,   5},  /* Row 15 */
  {  5,   0}, {  1,   1}, {  1,   5}, {  1,   2}, {  1,   0},
  {  2,   0}, {  1,   5}, {  1,   4}, {  4,   0}, {  1,   4}, {  2,   2}, {  2,   3}, {  1,   5},  /* Row 16 */
  {  7,   0}, {  1,   2}, {  1,   5}, {  2,   0},
  {  2,   0}, {  1,   2}, {  1,   5}, {  1,   1}, {  3,   0}, {  1,   4}, {  2,   3}, {  1,   5},  /* Row 17 */
  {  8,   0}, {  1,   1}, {  1,   3}, {  1,   2}, {  2,   0},
  {  3,   0}, {  1,   2}, {  1,   3}, {  3,   0}, {  1,   3}, {  1,   5}, {  9,   0}, {  1,   1},  /* Row 18 */
  {  2,   3}, {  3,   0},
  {  3,   0}, {  1,   1}, {  1,   5}, {  1,   2}, {  2,   0}, {  1,   5}, {  9,   0}, {  1,   1},  /* Row 19 */
  {  1,   4}, {  1,   5}, {  4,   0},
  {  4,   0}, {  1,   1}, {  1,   5}, {  1,   3}, { 10,   0}, {  1,   1}, {  1,   3}, {  1,   5},  /* Row 20 */
  {  5,   0},
  {  6,   0}, {  1,   2}, {  1,   5}, {  1,   4}, {  6,   0}, {  1,   1}, {  1,   2}, {  1,   5},  /* Row 21 */
  {  1,   2}, {  6,   0},
  {  7,   0}, {  1,   3}, {  3,   5}, {  3,   3}, {  2,   5}, {  1,   3}, {  1,   2}, {  7,   0},  /* Row 22 */
  {  9,   0}, {  1,   4}, {  2,   2}, {  1,   3}, {  2,   2}, {  1,   4}, {  9,   0},              /* Row 23 */
  { 25,   0},                                                                                      /* Row 24 */
};

/********************************************************************************************
 * Public Bitmap Structure Defintions
 ********************************************************************************************/

const struct NXWidgets::SRlePaletteBitmap NxWM::g_playBitmap =
{
  CONFIG_NXWIDGETS_BPP,  // bpp    - Bits per pixel
  CONFIG_NXWIDGETS_FMT,  // fmt    - Color format
  BITMAP_NLUTCODES,      // nlut   - Number of colors in the lLook-Up Table (LUT)
  BITMAP_NCOLUMNS,       // width  - Width in pixels 
  BITMAP_NROWS,          // height - Height in rows
  {                      // lut    - Pointer to the beginning of the Look-Up Table (LUT)
    g_playNormalLut,     //          Index 0: Unselected LUT
    g_playBrightlLut,    //          Index 1: Selected LUT
  },
  g_playRleEntries      // data   - Pointer to the beginning of the RLE data
};
