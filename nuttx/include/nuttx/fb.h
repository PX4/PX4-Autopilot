/****************************************************************************
 * include/nuttx/fb.h
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef _INCLUDE_NUTTX_FB_H
#define _INCLUDE_NUTTX_FB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Color format definitions.  The pretty much define the color pixel processing
 * organization of the video controller.
 */

/* Monochrome Formats *******************************************************/

#define FB_FMT_Y1          0         /* BPP=1, monochrome */
#define FB_FMT_Y4          1         /* BPP=4, 4-bit uncompressed greyscale */
#define FB_FMT_Y8          2         /* BPP=8, 8-bit uncompressed greyscale */
#define FB_FMT_Y16         3         /* BPP=16, 16-bit uncompressed greyscale */
#define FB_FMT_GREY        FB_FMT_Y8 /* BPP=8 */
#define FB_FMT_Y800        FB_FMT_Y8 /* BPP=8 */

#define FB_ISMONO(f)       ((f) >= FB_FMT_Y4) && (f) <= FB_FMT_Y16)

/* RGB video formats ********************************************************/

/* Standard RGB */

#define FB_FMT_RGB1        FB_FMT_Y1   /* BPP=1 */
#define FB_FMT_RGB4        4           /* BPP=4 */
#define FB_FMT_RGB8        5           /* BPP=8 RGB palette index */
#define FB_FMT_RGB8_332    6           /* BPP=8  R=3, G=3, B=2 */
#define FB_FMT_RGB12_444   7           /* BPP=12 R=4, G=4, B=4 */
#define FB_FMT_RGB16_555   8           /* BPP=16 R=5, G=5, B=5 (1 unused bit) */
#define FB_FMT_RGB16_565   9           /* BPP=16 R=6, G=6, B=5 */
#define FB_FMT_RGB24       10          /* BPP=24 */
#define FB_FMT_RGB32       11          /* BPP=32 */

/* Run length encoded RGB */

#define FB_FMT_RGBRLE4     12          /* BPP=4 */
#define FB_FMT_RGBRLE8     13          /* BPP=8 */

/* Raw RGB */

#define FB_FMT_RGBRAW      14          /* BPP=? */

/* Raw RGB with arbitrary sample packing within a pixel. Packing and precision
 * of R, G and B components is determined by bit masks for each.
 */

#define FB_FMT_RGBBTFLD16  15          /* BPP=16 */
#define FB_FMT_RGBBTFLD24  16          /* BPP=24 */
#define FB_FMT_RGBBTFLD32  17          /* BPP=32 */
#define FB_FMT_RGBA16      18          /* BPP=16 Raw RGB with alpha */
#define FB_FMT_RGBA32      19          /* BPP=32 Raw RGB with alpha */

/* Raw RGB with a transparency field. Layout is as for stanadard RGB at 16 and
 * 32 bits per pixel but the msb in each pixel indicates whether the pixel is
 * transparent or not.
 */

#define FB_FMT_RGBT16      20          /* BPP=16 */
#define FB_FMT_RGBT32      21          /* BPP=32 */

#define FB_ISRGB(f)  ((f) >= FB_FMT_RGB1) && (f) <= FB_FMT_RGBT32)

/* Packed YUV Formats *******************************************************/

#define FB_FMT_AYUV        22          /* BPP=32  Combined YUV and alpha */
#define FB_FMT_CLJR        23          /* BPP=8   4 pixels packed into a uint32_t.
                                        * YUV 4:1:1 with l< 8 bits per YUV sample */
#define FB_FMT_CYUV        24          /* BPP=16  UYVY except that height is reversed */
#define FB_FMT_IRAW        25          /* BPP=?   Intel uncompressed YUV */
#define FB_FMT_IUYV        26          /* BPP=16  Interlaced UYVY (line order
                                        * 0,2,4,.., 1,3,5...) */
#define FB_FMT_IY41        27          /* BPP=12  Interlaced Y41P (line order
                                        * 0,2,4,.., 1,3,5...) */
#define FB_FMT_IYU2        28          /* BPP=24 */
#define FB_FMT_HDYC        29          /* BPP=16  UYVY except uses the BT709 color space  */
#define FB_FMT_UYVP        30          /* BPP=24? YCbCr 4:2:2, 10-bits per component in U0Y0V0Y1 order */
#define FB_FMT_UYVY        31          /* BPP=16  YUV 4:2:2 */
#define FB_FMT_UYNV        FB_FMT_UYVY /* BPP=16  */
#define FB_FMT_Y422        FB_FMT_UYVY /* BPP=16  */
#define FB_FMT_V210        32          /* BPP=32  10-bit 4:2:2 YCrCb */
#define FB_FMT_V422        33          /* BPP=16  Upside down version of UYVY */
#define FB_FMT_V655        34          /* BPP=16? 16-bit YUV 4:2:2 */
#define FB_FMT_VYUY        35          /* BPP=?   ATI Packed YUV Data */
#define FB_FMT_YUYV        36          /* BPP=16  YUV 4:2:2 */
#define FB_FMT_YUY2        FB_FMT_YUYV /* BPP=16  YUV 4:2:2 */
#define FB_FMT_YUNV        FB_FMT_YUYV /* BPP=16  YUV 4:2:2 */
#define FB_FMT_YVYU        37          /* BPP=16  YUV 4:2:2 */
#define FB_FMT_Y41P        38          /* BPP=12  YUV 4:1:1 */
#define FB_FMT_Y411        39          /* BPP=12  YUV 4:1:1 */
#define FB_FMT_Y211        40          /* BPP=8  */
#define FB_FMT_Y41T        41          /* BPP=12  Y41P LSB for transparency */
#define FB_FMT_Y42T        42          /* BPP=16  UYVY LSB for transparency */
#define FB_FMT_YUVP        43          /* BPP=24? YCbCr 4:2:2 Y0U0Y1V0 order */

#define FB_ISYUVPACKED(f)  ((f) >= FB_FMT_AYUV) && (f) <= FB_FMT_YUVP)

/* Packed Planar YUV Formats ************************************************/

#define FB_FMT_YVU9        44          /* BPP=9  8-bit Y followed by 8-bit 4x4 VU */
#define FB_FMT_YUV9        45          /* BPP=9? */
#define FB_FMT_IF09        46          /* BPP=9.5 YVU9 + 4x4 plane of delta relative to tframe. */
#define FB_FMT_YV16        47          /* BPP=16  8-bit Y followed by 8-bit 2x1 VU */
#define FB_FMT_YV12        48          /* BPP=12  8-bit Y followed by 8-bit 2x2 VU */
#define FB_FMT_I420        49          /* BPP=12  8-bit Y followed by 8-bit 2x2 UV */
#define FB_FMT_IYUV        FB_FMT_I420 /* BPP=12 */
#define FB_FMT_NV12        50          /* BPP=12  8-bit Y followed by an interleaved 2x2 UV */
#define FB_FMT_NV21        51          /* BPP=12  NV12 with UV reversed */
#define FB_FMT_IMC1        52          /* BPP=12  YV12 except UV planes ame stride as Y */
#define FB_FMT_IMC2        53          /* BPP=12  IMC1 except UV lines interleaved at half stride boundaries */
#define FB_FMT_IMC3        54          /* BPP=12  As IMC1 except that UV swapped */
#define FB_FMT_IMC4        55          /* BPP=12  As IMC2  except that UV swapped */
#define FB_FMT_CLPL        56          /* BPP=12  YV12 but including a level of indirection. */
#define FB_FMT_Y41B        57          /* BPP=12?  4:1:1 planar. */
#define FB_FMT_Y42B        58          /* BPP=16?  YUV 4:2:2 planar. */
#define FB_FMT_CXY1        59          /* BPP=12 */
#define FB_FMT_CXY2        60          /* BPP=16 */

#define FB_ISYUVPLANAR(f)  ((f) >= FB_FMT_AYUV) && (f) <= FB_FMT_YUVP)
#define FB_ISYUV(f)        (FB_ISYUVPACKED(f) || FB_ISYUVPLANAR(f))

/* Hardware cursor control **************************************************/

#ifdef CONFIG_FB_HWCURSOR
#define FB_CUR_ENABLE      0x01        /* Enable the cursor */
#define FB_CUR_SETIMAGE    0x02        /* Set the cursor image */
#define FB_CUR_SETPOSITION 0x04        /* Set the position of the cursor */
#define FB_CUR_SETSIZE     0x08        /* Set the size of the cursor */
#define FB_CUR_XOR         0x10        /* Use XOR vs COPY ROP on image */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* If any dimension of the display exceeds 65,536 pixels, then the following
 * type will need to change:
 */

typedef uint16_t fb_coord_t;

/* This structure describes the overall video controller */

struct fb_videoinfo_s
{
 uint8_t    fmt;          /* see FB_FMT_*  */
 fb_coord_t xres;         /* Horizontal resolution in pixel columns */
 fb_coord_t yres;         /* Vertical resolution in pixel rows */
 uint8_t    nplanes;      /* Number of color planes supported */
};

/* This structure describes one color plane.  Some YUV formats may support
 * up to 4 planes
 */

struct fb_planeinfo_s
{
  FAR void  *fbmem;       /* Start of frame buffer memory */
  uint32_t   fblen;       /* Length of frame buffer memory in bytes */
  fb_coord_t stride;      /* Length of a line in bytes */
  uint8_t    bpp;         /* Bits per pixel */
};

/* On video controllers that support mapping of a pixel palette value
 * to an RGB encoding, the following structure may be used to define
 * that mapping.
 */

#ifdef CONFIG_FB_CMAP
struct fb_cmap_s
{
 uint16_t  first;         /* Offset offset first color entry in tables */
 uint16_t  len;           /* Number of color entries  in tables */

 /* Tables of  color component.  Any may be NULL if not used */

 uint8_t *red;            /* Table of 8-bit red values */
 uint8_t *green;          /* Table of 8-bit green values */
 uint8_t *blue;           /* Table of 8-bit blue values */
#ifdef CONFIG_FB_TRANSPARENCY
 uint8_t *transp;         /* Table of 8-bit transparency */
#endif
};
#endif

/* If the video controller hardware supports a hardware cursor and
 * that hardware cursor supports user-provided images, then the
 * following structure may be used to provide the cursor image
 */

#ifdef CONFIG_FB_HWCURSOR
#ifdef CONFIG_FB_HWCURSORIMAGE
struct fb_cursorimage_s
{
 fb_coord_t     width;     /* Width of the cursor image in pixels */
 fb_coord_t     height     /* Height of the curor image in pixels */
 const uint8_t *image;     /* Pointer to image data */
};
#endif

/* The following structure defines the cursor position/size */

struct fb_cursorpos_s
{
 fb_coord_t x;             /* X position in pixels */
 fb_coord_t y;             /* Y position in rows */
};

/* If the hardware supports setting the cursor size, then this structure
 * is used to provide the size.
 */

#ifdef CONFIG_FB_HWCURSORSIZE
struct fb_cursorsize_s
{
 fb_coord_t h;             /* Height in rows */
 fb_coord_t w;             /* Width in pixels */
};
#endif

/* The following is used to get the cursor attributes */

struct fb_cursorattrib_s
{
#ifdef CONFIG_FB_HWCURSORIMAGE
  uint8_t fmt;                   /* Video format of cursor */
#endif
  struct fb_cursorpos_s  pos;    /* Current cursor position */
#ifdef CONFIG_FB_HWCURSORSIZE
  struct fb_cursorsize_s mxsize; /* Maximum cursor size */
  struct fb_cursorsize_s size;   /* Current size */
#endif
};

struct fb_setcursor_s
{
  uint8_t flags;                /* See FB_CUR_* definitions */
  struct fb_cursorpos_s pos;    /* Cursor position */
#ifdef CONFIG_FB_HWCURSORSIZE
  struct fb_cursorsize_s  size; /* Cursor size */
#endif
#ifdef CONFIG_FB_HWCURSORIMAGE
  struct fb_cursorimage_s img;  /* Cursor image */
#endif
};
#endif

/* The framebuffer "driver" under NuttX is not a driver at all, but simply
 * a driver "object" that is accessed through the following vtable:
 */

struct fb_vtable_s
{
  /* Get information about the video controller configuration and the configuration
   * of each color plane.
   */

  int (*getvideoinfo)(FAR struct fb_vtable_s *vtable, FAR struct fb_videoinfo_s *vinfo);
  int (*getplaneinfo)(FAR struct fb_vtable_s *vtable, int planeno, FAR struct fb_planeinfo_s *pinfo);

  /* The following are provided only if the video hardware supports RGB color mapping */

#ifdef CONFIG_FB_CMAP
  int (*getcmap)(FAR struct fb_vtable_s *vtable, FAR struct fb_cmap_s *cmap);
  int (*putcmap)(FAR struct fb_vtable_s *vtable, FAR const struct fb_cmap_s *cmap);
#endif
  /* The following are provided only if the video hardware supports a hardware cursor */

#ifdef CONFIG_FB_HWCURSOR
  int (*getcursor)(FAR struct fb_vtable_s *vtable, FAR struct fb_cursorattrib_s *attrib);
  int (*setcursor)(FAR struct fb_vtable_s *vtable, FAR struct fb_setcursor_s *settings);
#endif
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_fbinitialize, up_fbuninitialize, up_fbgetvplane
 *
 * Description:
 *   If an architecture supports a framebuffer, then it must provide APIs
 *   to access the framebuffer as follows:
 *
 *   up_fbinitialize   - Initialize the framebuffer video hardware
 *   up_fbgetvplane    - Return a a reference to the framebuffer object for
 *                       the specified video plane.  Most OSDs support
 *                       multiple planes of video.
 *   up_fbuninitialize - Unitialize the framebuffer support
 *
 ***************************************************************************/

EXTERN int up_fbinitialize(void);
EXTERN FAR struct fb_vtable_s *up_fbgetvplane(int vplane);
EXTERN void fb_uninitialize(void);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* _INCLUDE_NUTTX_FB_H */
