/****************************************************************************
 * include/nuttx/rgbcolors.h
 * User-friendly RGB color definitions
 *
 *   Copyright (C) 2010-2011 Gregory Nutt. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_RGBCOLOR_H
#define __INCLUDE_NUTTX_RGBCOLOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Color Creation and Conversion Macros *************************************/
/* This macro creates RGB24  from 8:8:8 RGB */

#define RGBTO24(r,g,b) \
  ((uint32_t)((r) & 0xff) << 16 | (uint32_t)((g) & 0xff) << 8 | (uint32_t)((b) & 0xff))

/* And these macros perform the inverse transformation */

#define RBG24RED(rgb)   (((rgb) >> 16) & 0xff)
#define RBG24GREEN(rgb) (((rgb) >> 8)  & 0xff)
#define RBG24BLUE(rgb)  ( (rgb)        & 0xff)

/* This macro creates RGB16 (5:6:5) from 8:8:8 RGB:
 *
 *   R[7:3] -> RGB[15:11]
 *   G[7:2] -> RGB[10:5]
 *   B[7:3] -> RGB[4:0]
 */

#define RGBTO16(r,g,b) \
  ((((uint16_t)(r) << 8) & 0xf800) | (((uint16_t)(g) << 3) & 0x07e0) | (((uint16_t)(b) >> 3) & 0x001f))

/* And these macros perform the inverse transformation */

#define RBG16RED(rgb)   (((rgb) >> 8) & 0xf8)
#define RBG16GREEN(rgb) (((rgb) >> 3) & 0xfc)
#define RBG16BLUE(rgb)  (((rgb) << 3) & 0xf8)

/* This macro creates RGB8 (3:3:2) from 8:8:8 RGB */

#define RGBTO8(r,g,b) \
  ((((uint8_t)(r) << 5) & 0xe0) | (((uint8_t)(g) << 2) & 0x1c) | ((uint8_t)(b) & 0x03))

/* And these macros perform the inverse transformation */

#define RBG8RED(rgb)    ( (rgb)       & 0xe0)
#define RBG8GREEN(rgb)  (((rgb) << 3) & 0xe0)
#define RBG8BLUE(rgb)   (((rgb) << 6) & 0xc0)

/* This macro converts RGB24 (8:8:8) to RGB16 (5:6:5):
 *
 *   00000000 RRRRRRRR BBBBBBBB GGGGGGGG -> RRRRRBBB BBBGGGGG
 */

#define RGB24TO16(rgb24) \
  (((rgb24 >> 8) & 0xf800) | ((rgb24 >> 5) & 0x07e0) | ((rgb24 >> 3) & 0x001f))

/* This macro converts RGB16 (5:6:5) to RGB24 (8:8:8):
 *
 *   RRRRRBBB BBBGGGGG -> 00000000 RRRRRRRR BBBBBBBB GGGGGGGG
 */

#define RGB16TO24(rgb16) \
  (((rgb16 & 0xf800) << 8) | ((rgb16 & 0x07e0) << 5)  | ((rgb16 & 0x001f) << 3))

/* Standard Color Definitions ***********************************************/
/* RGB24-888: 00000000 RRRRRRRR GGGGGGGG BBBBBBBB */

#define RGB24_BLACK          0x00000000
#define RGB24_WHITE          0x00ffffff

#define RGB24_BLUE           0x000000ff
#define RGB24_GREEN          0x0000ff00
#define RGB24_RED            0x00ff0000

#define RGB24_NAVY           0x00000080
#define RGB24_DARKBLUE       0x0000008b
#define RGB24_DARKGREEN      0x00006400
#define RGB24_DARKCYAN       0x00008b8b
#define RGB24_CYAN           0x0000ffff
#define RGB24_TURQUOISE      0x0040e0d0
#define RGB24_INDIGO         0x004b0082
#define RGB24_DARKRED        0x00800000
#define RGB24_OLIVE          0x00808000
#define RGB24_GRAY           0x00808080
#define RGB24_SKYBLUE        0x0087ceeb
#define RGB24_BLUEVIOLET     0x008a2be2
#define RGB24_LIGHTGREEN     0x0090ee90
#define RGB24_DARKVIOLET     0x009400d3
#define RGB24_YELLOWGREEN    0x009acd32
#define RGB24_BROWN          0x00a52a2a
#define RGB24_DARKGRAY       0x00a9a9a9
#define RGB24_SIENNA         0x00a0522d
#define RGB24_LIGHTBLUE      0x00add8e6
#define RGB24_GREENYELLOW    0x00adff2f
#define RGB24_SILVER         0x00c0c0c0
#define RGB24_LIGHTGREY      0x00d3d3d3
#define RGB24_LIGHTCYAN      0x00e0ffff
#define RGB24_VIOLET         0x00ee82ee
#define RGB24_AZUR           0x00f0ffff
#define RGB24_BEIGE          0x00f5f5dc
#define RGB24_MAGENTA        0x00ff00ff
#define RGB24_TOMATO         0x00ff6347
#define RGB24_GOLD           0x00ffd700
#define RGB24_ORANGE         0x00ffa500
#define RGB24_SNOW           0x00fffafa
#define RGB24_YELLOW         0x00ffff00

/* RGB16-565: RRRRRGGG GGGBBBBB */

#define RGB16_BLACK          0x0000
#define RGB16_WHITE          0xffff

#define RGB16_BLUE           0x001f
#define RGB16_GREEN          0x07e0
#define RGB16_RED            0xf800

#define RGB16_NAVY           0x0010
#define RGB16_DARKBLUE       0x0011
#define RGB16_DARKGREEN      0x0320
#define RGB16_DARKCYAN       0x0451
#define RGB16_CYAN           0x07ff
#define RGB16_TURQUOISE      0x471a
#define RGB16_INDIGO         0x4810
#define RGB16_DARKRED        0x8000
#define RGB16_OLIVE          0x8400
#define RGB16_GRAY           0x8410
#define RGB16_SKYBLUE        0x867d
#define RGB16_BLUEVIOLET     0x895c
#define RGB16_LIGHTGREEN     0x9772
#define RGB16_DARKVIOLET     0x901a
#define RGB16_YELLOWGREEN    0x9e66
#define RGB16_BROWN          0xa145
#define RGB16_DARKGRAY       0xad55
#define RGB16_SIENNA         0xa285
#define RGB16_LIGHTBLUE      0xaedc
#define RGB16_GREENYELLOW    0xafe5
#define RGB16_SILVER         0xc618
#define RGB16_LIGHTGREY      0xd69a
#define RGB16_LIGHTCYAN      0xe7ff
#define RGB16_VIOLET         0xec1d
#define RGB16_AZUR           0xf7ff
#define RGB16_BEIGE          0xf7bb
#define RGB16_MAGENTA        0xf81f
#define RGB16_TOMATO         0xfb08
#define RGB16_GOLD           0xfea0
#define RGB16_ORANGE         0xfd20
#define RGB16_SNOW           0xffdf
#define RGB16_YELLOW         0xffe0

/* RGB12-444: RRRR GGGGBBBB */

#define RGB12_BLACK          0x0000
#define RGB12_WHITE          0x0fff

#define RGB12_BLUE           0x000f
#define RGB12_GREEN          0x00f0
#define RGB12_RED            0x0f00

#define RGB12_NAVY           0x0008
#define RGB12_DARKBLUE       0x0009
#define RGB12_DARKGREEN      0x0060
#define RGB12_DARKCYAN       0x0099
#define RGB12_CYAN           0x00ff
#define RGB12_TURQUOISE      0x04ed
#define RGB12_INDIGO         0x0508
#define RGB12_DARKRED        0x0800
#define RGB12_OLIVE          0x0880
#define RGB12_GRAY           0x0888
#define RGB12_SKYBLUE        0x08df
#define RGB12_BLUEVIOLET     0x093e
#define RGB12_LIGHTGREEN     0x09f9
#define RGB12_DARKVIOLET     0x090d
#define RGB12_YELLOWGREEN    0x0ad3
#define RGB12_BROWN          0x0a33
#define RGB12_DARKGRAY       0x0bbb
#define RGB12_SIENNA         0x0a53
#define RGB12_LIGHTBLUE      0x0bee
#define RGB12_GREENYELLOW    0x0bf3
#define RGB12_SILVER         0x0ccc
#define RGB12_LIGHTGREY      0x0ddd
#define RGB12_LIGHTCYAN      0x0eff
#define RGB12_VIOLET         0x0f8f
#define RGB12_AZUR           0x0fff
#define RGB12_BEIGE          0x0ffe
#define RGB12_MAGENTA        0x0f0f
#define RGB12_TOMATO         0x0f64
#define RGB12_GOLD           0x0fd0
#define RGB12_ORANGE         0x0fa0
#define RGB12_SNOW           0x0fff
#define RGB12_YELLOW         0x0ff0

/* RGB8-332: RRRGGGBB (really not enough color resolution for the following) */

#define RGB8_BLACK           0x00
#define RGB8_WHITE           0xff

#define RGB8_BLUE            0x03
#define RGB8_GREEN           0x1c
#define RGB8_RED             0xe0
#define RGB8_NAVY            0x02
#define RGB8_DARKBLUE        0x02
#define RGB8_DARKGREEN       0x0c
#define RGB8_DARKCYAN        0x16
#define RGB8_CYAN            0x1f
#define RGB8_TURQUOISE       0x5f
#define RGB8_INDIGO          0x62
#define RGB8_DARKRED         0x80
#define RGB8_OLIVE           0x90
#define RGB8_GRAY            0x92
#define RGB8_SKYBLUE         0x9f
#define RGB8_BLUEVIOLET      0xab
#define RGB8_LIGHTGREEN      0xbe
#define RGB8_DARKVIOLET      0x93
#define RGB8_YELLOWGREEN     0x9d
#define RGB8_BROWN           0xa9
#define RGB8_DARKGRAY        0xdb
#define RGB8_SIENNA          0xa9
#define RGB8_LIGHTBLUE       0xdf
#define RGB8_GREENYELLOW     0xdd
#define RGB8_SILVER          0xd9
#define RGB8_LIGHTGREY       0xd9
#define RGB8_LIGHTCYAN       0xff
#define RGB8_VIOLET          0xf3
#define RGB8_AZUR            0xff
#define RGB8_BEIGE           0xff
#define RGB8_MAGENTA         0xed
#define RGB8_TOMATO          0xfc
#define RGB8_GOLD            0xfc
#define RGB8_ORANGE          0xf8
#define RGB8_SNOW            0xff
#define RGB8_YELLOW          0xfc

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_RGBCOLOR_H */
