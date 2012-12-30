/****************************************************************************
 * include/nuttx/nx/nxfonts.h
 *
 *   Copyright (C) 2008, 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NX_NXFONTS_H
#define __INCLUDE_NUTTX_NX_NXFONTS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/
/* Select the default font.  If no fonts are selected, then a compilation error
 * is likely down the road.
 */

/* Sans serif fonts */

#if defined(CONFIG_NXFONT_SANS23X27)       /* The "legacy," tiny NuttX font */
# define NXFONT_DEFAULT FONTID_SANS23X27

#elif defined(CONFIG_NXFONT_SANS17X22)
# define NXFONT_DEFAULT FONTID_SANS17X22

#elif defined(CONFIG_NXFONT_SANS20X26)
# define NXFONT_DEFAULT FONTID_SANS20X26

#elif defined(CONFIG_NXFONT_SANS22X29)
# define NXFONT_DEFAULT FONTID_SANS22X29

#elif defined(CONFIG_NXFONT_SANS28X37)
# define NXFONT_DEFAULT FONTID_SANS28X37

#elif defined(CONFIG_NXFONT_SANS39X48)
# define NXFONT_DEFAULT FONTID_SANS39X48

/* Sans serif bold fonts */

#elif defined(CONFIG_NXFONT_SANS17X23B)
# define NXFONT_DEFAULT FONTID_SANS17X23B

#elif defined(CONFIG_NXFONT_SANS20X27B)
# define NXFONT_DEFAULT FONTID_SANS20X27B

#elif defined(CONFIG_NXFONT_SANS22X29B)
# define NXFONT_DEFAULT FONTID_SANS22X29B

#elif defined(CONFIG_NXFONT_SANS28X37B)
# define NXFONT_DEFAULT FONTID_SANS28X37B

#elif defined(CONFIG_NXFONT_SANS40X49B)
# define NXFONT_DEFAULT FONTID_SANS40X49B

/* Serif fonts */

#elif defined(CONFIG_NXFONT_SERIF22X29)
# define NXFONT_DEFAULT FONTID_SERIF22X29

#elif defined(CONFIG_NXFONT_SERIF29X37)
# define NXFONT_DEFAULT FONTID_SERIF29X37

#elif defined(CONFIG_NXFONT_SERIF38X48)
# define NXFONT_DEFAULT FONTID_SERIF38X48

/* Serif bold fonts */

#elif defined(CONFIG_NXFONT_SERIF22X28B)
# define NXFONT_DEFAULT FONTID_SERIF22X28B

#elif defined(CONFIG_NXFONT_SERIF27X38B)
# define NXFONT_DEFAULT FONTID_SERIF27X38B

#elif defined(CONFIG_NXFONT_SERIF38X49B)
# define NXFONT_DEFAULT FONTID_SERIF38X49B

/* Mono-space fonts */

#elif defined(CONFIG_NXFONT_MONO5X8)
# define NXFONT_DEFAULT FONTID_MONO5X8

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Font IDs */

enum nx_fontid_e
{
  FONTID_DEFAULT     = 0         /* The default font */

/* Monospace fonts */

#ifdef CONFIG_NXFONT_MONO5X8
  , FONTID_MONO5X8 = 18          /* The 5x8 monospace font */
#endif
  
/* Sans Serif fonts */

#ifdef CONFIG_NXFONT_SANS17X22
  , FONTID_SANS17X22 = 14        /* The 17x22 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS20X26
  , FONTID_SANS20X26 = 15        /* The 20x26 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS23X27
  , FONTID_SANS23X27 = 1         /* The 23x27 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS22X29
  , FONTID_SANS22X29 = 2         /* The 22x29 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS28X37
  , FONTID_SANS28X37 = 3         /* The 28x37 sans serif font */
#endif

#ifdef CONFIG_NXFONT_SANS39X48
  , FONTID_SANS39X48 = 4         /* The 39x48 sans serif font */
#endif

/* Sans Serif bold fonts */

#ifdef CONFIG_NXFONT_SANS17X23B
  , FONTID_SANS17X23B = 16       /* The 17x23 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS20X27B
  , FONTID_SANS20X27B = 17       /* The 20x27 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS22X29B
  , FONTID_SANS22X29B = 5        /* The 22x29 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS28X37B
  , FONTID_SANS28X37B = 6        /* The 28x37 sans bold font */
#endif

#ifdef CONFIG_NXFONT_SANS40X49B
  , FONTID_SANS40X49B = 7        /* The 40x49 sans bold font */
#endif

/* Serif fonts */

#ifdef CONFIG_NXFONT_SERIF22X29
  , FONTID_SERIF22X29 = 8        /* The 22x29 serif font */
#endif

#ifdef CONFIG_NXFONT_SERIF29X37
  , FONTID_SERIF29X37 = 9        /* The 29x37 serif font */
#endif

#ifdef CONFIG_NXFONT_SERIF38X48
  , FONTID_SERIF38X48 = 10       /* The 38x48 serif font */
#endif

/* Serif bold fonts */

#ifdef CONFIG_NXFONT_SERIF22X28B
  , FONTID_SERIF22X28B = 11      /* The 22x28 serif bold font */
#endif

#ifdef CONFIG_NXFONT_SERIF27X38B
  , FONTID_SERIF27X38B = 12      /* The 27x38 serif bold font */
#endif

#ifdef CONFIG_NXFONT_SERIF38X49B
  , FONTID_SERIF38X49B = 13      /* The 38x49 serif bold font */
#endif
};

/* This structures provides the metrics for one glyph */

struct nx_fontmetric_s
{
  uint32_t stride   : 3;      /* Width of one font row in bytes */
  uint32_t width    : 6;      /* Width of the font in bits */
  uint32_t height   : 6;      /* Height of the font in rows */
  uint32_t xoffset  : 6;      /* Top, left-hand corner X-offset in pixels */
  uint32_t yoffset  : 6;      /* Top, left-hand corner y-offset in pixels */
  uint32_t unused   : 5;
};

/* This structure binds the glyph metrics to the glyph bitmap */

struct nx_fontbitmap_s
{
  struct nx_fontmetric_s metric; /* Character metrics */
  FAR const uint8_t *bitmap;     /* Pointer to the character bitmap */
};

/* This structure describes one contiguous grouping of glyphs that
 * can be described by an array starting with encoding 'first' and
 * extending through (first + nchars - 1).
 */

struct nx_fontset_s
{
  uint8_t  first;             /* First bitmap character code */
  uint8_t  nchars;            /* Number of bitmap character codes */
  FAR const struct nx_fontbitmap_s *bitmap;
};

/* This structure describes the overall metrics of the fontset */

struct nx_font_s
{
  uint8_t  mxheight;          /* Max height of one glyph in rows */
  uint8_t  mxwidth;           /* Max width of any glyph in pixels */
  uint8_t  mxbits;            /* Max number of bits per character code */
  uint8_t  spwidth;           /* The width of a space in pixels */
};

/* Finally, this structure defines everything about the font set */

struct nx_fontpackage_s
{
  uint8_t id;                            /* The font ID */
  FAR const struct nx_font_s    metrics; /* Font set metrics */
  FAR const struct nx_fontset_s font7;   /* Fonts for 7-bit encoding */
#if CONFIG_NXFONTS_CHARBITS >= 8
  FAR const struct nx_fontset_s font8;   /* Fonts for 8-bit encoding */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: nxf_getfonthandle
 *
 * Description:
 *   Given a numeric font ID, return a handle that may be subsequently be
 *   used to access the font data sets.
 *
 * Input Parameters:
 *   fontid:  Identifies the font set to get
 *
 ****************************************************************************/

EXTERN NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid);

/****************************************************************************
 * Name: nxf_getfontset
 *
 * Description:
 *   Return information about the current font set
 *
 * Input Parameters:
 *   handle:  A font handle previously returned by nxf_getfonthandle()
 *
 ****************************************************************************/

EXTERN FAR const struct nx_font_s *nxf_getfontset(NXHANDLE handle);

/****************************************************************************
 * Name: nxf_getbitmap
 *
 * Description:
 *   Return font bitmap information for the selected character encoding.
 *
 * Input Parameters:
 *   handle:  A font handle previously returned by nxf_getfonthandle()
 *   ch:      Character code whose bitmap is requested
 *
 * Returned Value:
 *   An instance of struct nx_fontbitmap_s describing the glyph.
 *
 ****************************************************************************/

EXTERN FAR const struct nx_fontbitmap_s *
  nxf_getbitmap(NXHANDLE handle, uint16_t ch);

/****************************************************************************
 * Name: nxf_convert_*bpp
 *
 * Description:
 *   Convert the 1BPP font to a new pixel depth
 *
 * Input Parameters:
 *   dest   - The destination buffer provided by the caller.
 *   height - The max height of the returned char in rows
 *   width  - The max width of the returned char in pixels
 *   stride - The width of the destination buffer in bytes
 *   bm     - Describes the character glyph to convert
 *   color  - The color to use for '1' bits in the font bitmap
 *            (0 bits are transparent)
 *
 * Returned Value:
 *  OK on Success, ERROR: on failure with errno set appropriately.
 *  (never fails)
 *
 ****************************************************************************/

EXTERN int nxf_convert_1bpp(FAR uint8_t *dest, uint16_t height,
                            uint16_t width, uint16_t stride,
                            FAR const struct nx_fontbitmap_s *bm,
                            nxgl_mxpixel_t color);
EXTERN int nxf_convert_2bpp(FAR uint8_t *dest, uint16_t height,
                            uint16_t width, uint16_t stride,
                            FAR const struct nx_fontbitmap_s *bm,
                            nxgl_mxpixel_t color);
EXTERN int nxf_convert_4bpp(FAR uint8_t *dest, uint16_t height,
                            uint16_t width, uint16_t stride,
                            FAR const struct nx_fontbitmap_s *bm,
                            nxgl_mxpixel_t color);
EXTERN int nxf_convert_8bpp(FAR uint8_t *dest, uint16_t height,
                            uint16_t width, uint16_t stride,
                            FAR const struct nx_fontbitmap_s *bm,
                            nxgl_mxpixel_t color);
EXTERN int nxf_convert_16bpp(FAR uint16_t *dest, uint16_t height,
                             uint16_t width, uint16_t stride,
                             FAR const struct nx_fontbitmap_s *bm,
                             nxgl_mxpixel_t color);
EXTERN int nxf_convert_24bpp(FAR uint32_t *dest, uint16_t height,
                             uint16_t width, uint16_t stride,
                             FAR const struct nx_fontbitmap_s *bm,
                             nxgl_mxpixel_t color);
EXTERN int nxf_convert_32bpp(FAR uint32_t *dest, uint16_t height,
                             uint16_t width, uint16_t stride,
                             FAR const struct nx_fontbitmap_s *bm,
                             nxgl_mxpixel_t color);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXFONTS_H */
