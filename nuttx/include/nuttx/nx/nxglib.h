/****************************************************************************
 * include/nuttx/nx/nxglib.h
 *
 *   Copyright (C) 2008-2011 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_NX_NXGLIB_H
#define __INCLUDE_NUTTX_NX_NXGLIB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <fixedmath.h>

#ifdef CONFIG_NX_LCDDRIVER
#  include <nuttx/lcd/lcd.h>
#else
#  include <nuttx/fb.h>
#endif

/****************************************************************************
 * Pre-processor definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_NX_NPLANES
#  define CONFIG_NX_NPLANES  1  /* Max number of color planes supported */
#endif

/* Driver Selection *********************************************************/
/* NX_DRIVERTYPE selects either the framebuffer or LCD driver;
 * NX_PLANINFO_TYPE hides the difference in the framebuffer and LCD driver
 * plane types. defines are used instead of a typedefs to avoid type
 * mismatches.
 */

#ifdef CONFIG_NX_LCDDRIVER
#  define NX_DRIVERTYPE    struct lcd_dev_s
#  define NX_PLANEINFOTYPE struct lcd_planeinfo_s
#else
#  define NX_DRIVERTYPE    struct fb_vtable_s
#  define NX_PLANEINFOTYPE struct fb_planeinfo_s
#endif

/* NXGL Macros **************************************************************/
/* Mnemonics for indices */

#define NX_TOP_NDX           (0)
#define NX_LEFT_NDX          (1)
#define NX_RIGHT_NDX         (2)
#define NX_BOTTOM_NDX        (3)

/* Handy macros */

#define ngl_min(a,b)       ((a) < (b) ? (a) : (b))
#define ngl_max(a,b)       ((a) > (b) ? (a) : (b))
#define ngl_swap(a,b,t)    do { t = a; a = b; b = t; } while (0)
#define ngl_clipl(a,mn)    ((a) < (mn) ? (mn) : (a))
#define ngl_clipr(a,mx)    ((a) > (mx) ? (mx) : (a))
#define ngl_clip(a,mx,mn)  ((a) < (mn) ? (mn) : (a) > (mx) ? (mx) : (a))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Pixels *******************************************************************/

/* The size of graphics solutions can be reduced by disabling support for
 * specific resolutions.  One thing we can do, for example, is to select
 * the smallest common pixel representation:
 */

#if !defined(CONFIG_NX_DISABLE_32BPP) || !defined(CONFIG_NX_DISABLE_24BPP)
typedef uint32_t nxgl_mxpixel_t;
#elif !defined(CONFIG_NX_DISABLE_16BPP)
typedef uint16_t nxgl_mxpixel_t;
#else
typedef uint8_t  nxgl_mxpixel_t;
#endif

/* Graphics structures ******************************************************/

/* A given coordinate is limited to the screen height an width.  If either
 * of those values exceed 32,767 pixels, then the following will have to need
 * to change:
 */

typedef int16_t nxgl_coord_t;

/* Describes a point on the display */

struct nxgl_point_s
{
  nxgl_coord_t x;         /* X position, range: 0 to screen width - 1 */
  nxgl_coord_t y;         /* Y position, range: 0 to screen height - 1 */
};

/* Describes the size of a rectangular region */

struct nxgl_size_s
{
  nxgl_coord_t w;        /* Width in pixels */
  nxgl_coord_t h;        /* Height in rows */
};

/* Describes a positioned rectangle on the display */

struct nxgl_rect_s
{
  struct nxgl_point_s pt1; /* Upper, left-hand corner */
  struct nxgl_point_s pt2; /* Lower, right-hand corner */
};

/* Describes a vector starting at pt1 and extending throug pt2 */

struct nxgl_vector_s
{
  struct nxgl_point_s pt1; /* Start position */
  struct nxgl_point_s pt2; /* End position */
};

/* Describes a run, i.e., a horizontal line.  Note that the start/end positions
 * have fractional precision.  This is necessary for good joining of trapezoids
 * when a more complex shape is decomposed into trapezoids
 */

struct nxgl_run_s
{
  b16_t        x1;        /* Left X position, range: 0 to x2 */
  b16_t        x2;        /* Right X position, range: x1 to screen width - 1 */
  nxgl_coord_t y;         /* Top Y position, range: 0 to screen height - 1 */
};

/* Describes a horizontal trapezoid on the display in terms the run at the
 * top of the trapezoid and the run at the bottom
 */

struct nxgl_trapezoid_s
{
  struct nxgl_run_s top;  /* Top run */
  struct nxgl_run_s bot;  /* bottom run */
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

/* Color conversons *********************************************************/

/****************************************************************************
 * Name: nxgl_rgb2yuv
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

EXTERN void nxgl_rgb2yuv(uint8_t r, uint8_t g, uint8_t b,
                         uint8_t *y, uint8_t *u, uint8_t *v);

/****************************************************************************
 * Name: nxgl_yuv2rgb
 *
 * Description:
 *   Convert 8-bit RGB triplet to 8-bit YUV triplet
 *
 ****************************************************************************/

EXTERN void nxgl_yuv2rgb(uint8_t y, uint8_t u, uint8_t v,
                         uint8_t *r, uint8_t *g, uint8_t *b);

/* Rasterizers **************************************************************/

/****************************************************************************
 * Name: nxgl_setpixel_*bpp
 *
 * Descripton:
 *   Draw a single pixel in graphics memory at the given position and
 *   with the given color.  This is equivalent to nxgl_fillrectangle_*bpp()
 *   with a 1x1 rectangle but is more efficient.
 *
 ****************************************************************************/

EXTERN void nxgl_setpixel_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                               FAR const struct nxgl_point_s *pos,
                               uint8_t color);
EXTERN void nxgl_setpixel_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                               FAR const struct nxgl_point_s *pos,
                               uint8_t color);
EXTERN void nxgl_setpixel_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                               FAR const struct nxgl_point_s *pos,
                               uint8_t color);
EXTERN void nxgl_setpixel_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                               FAR const struct nxgl_point_s *pos,
                               uint8_t color);
EXTERN void nxgl_setpixel_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_point_s *pos,
                                uint16_t color);
EXTERN void nxgl_setpixel_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_point_s *pos,
                                uint32_t color);
EXTERN void nxgl_setpixel_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                FAR const struct nxgl_point_s *pos,
                                uint32_t color);

/****************************************************************************
 * Name: nxgl_fillrectangle_*bpp
 *
 * Descripton:
 *   Fill a rectangle region in the graphics memory with a fixed color
 *
 ****************************************************************************/

EXTERN void nxgl_fillrectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    uint8_t color);
EXTERN void nxgl_fillrectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    uint8_t color);
EXTERN void nxgl_fillrectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    uint8_t color);
EXTERN void nxgl_fillrectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    uint8_t color);
EXTERN void nxgl_fillrectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     uint16_t color);
EXTERN void nxgl_fillrectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     uint32_t color);
EXTERN void nxgl_fillrectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     uint32_t color);

/****************************************************************************
 * Name: nxgl_getrectangle_*bpp
 *
 * Descripton:
 *   Fetch a rectangular region from graphics memory.  The source is
 *   expressed as a rectangle.
 *
 ****************************************************************************/

EXTERN void nxgl_getrectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                   FAR const struct nxgl_rect_s *rect,
                                   FAR void *dest, unsigned int deststride);
EXTERN void nxgl_getrectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                   FAR const struct nxgl_rect_s *rect,
                                   FAR void *dest, unsigned int deststride);
EXTERN void nxgl_getrectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                   FAR const struct nxgl_rect_s *rect,
                                   FAR void *dest, unsigned int deststride);
EXTERN void nxgl_getrectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                   FAR const struct nxgl_rect_s *rect,
                                   FAR void *dest, unsigned int deststride);
EXTERN void nxgl_getrectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR void *dest, unsigned int deststride);
EXTERN void nxgl_getrectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR void *dest, unsigned int deststride);
EXTERN void nxgl_getrectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR void *dest, unsigned int deststride);

/****************************************************************************
 * Name: nxglib_filltrapezoid_*bpp
 *
 * Descripton:
 *   Fill a trapezoidal region in the graphics memory with a fixed color.
 *   Clip the trapezoid to lie within a boundng box.  This is useful for
 *   drawing complex shapes that can be broken into a set of trapezoids.
 *
 ****************************************************************************/

EXTERN void nxgl_filltrapezoid_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_trapezoid_s *trap,
                                    FAR const struct nxgl_rect_s *bounds,
                                    uint8_t color);
EXTERN void nxgl_filltrapezoid_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_trapezoid_s *trap,
                                    FAR const struct nxgl_rect_s *bounds,
                                    uint8_t color);
EXTERN void nxgl_filltrapezoid_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_trapezoid_s *trap,
                                    FAR const struct nxgl_rect_s *bounds,
                                    uint8_t color);
EXTERN void nxgl_filltrapezoid_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_trapezoid_s *trap,
                                    FAR const struct nxgl_rect_s *bounds,
                                    uint8_t color);
EXTERN void nxgl_filltrapezoid_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_trapezoid_s *trap,
                                    FAR const struct nxgl_rect_s *bounds,
                                    uint16_t color);
EXTERN void nxgl_filltrapezoid_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_trapezoid_s *trap,
                                     FAR const struct nxgl_rect_s *bounds,
                                     uint32_t color);
EXTERN void nxgl_filltrapezoid_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_trapezoid_s *trap,
                                     FAR const struct nxgl_rect_s *bounds,
                                     uint32_t color);

/****************************************************************************
 * Name: nxgl_moverectangle_*bpp
 *
 * Descripton:
 *   Move a rectangular region from location to another in the
 *   framebuffer/LCD memory.  The source is expressed as a rectangle; the
 *   destination position is expressed as a point corresponding to the
 *   translation of the upper, left-hand corner.
 *
 ****************************************************************************/

EXTERN void nxgl_moverectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *rect,
                                    FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     FAR struct nxgl_point_s *offset);
EXTERN void nxgl_moverectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *rect,
                                     FAR struct nxgl_point_s *offset);

/****************************************************************************
 * Name: nxgl_copyrectangle_*bpp
 *
 * Descripton:
 *   Copy a rectangular bitmap image into the specific position in the
 *   graphics memory.
 *
 ****************************************************************************/

EXTERN void nxgl_copyrectangle_1bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_2bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_4bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_8bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                    FAR const struct nxgl_rect_s *dest,
                                    FAR const void *src,
                                    FAR const struct nxgl_point_s *origin,
                                    unsigned int srcstride);
EXTERN void nxgl_copyrectangle_16bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *dest,
                                     FAR const void *src,
                                     FAR const struct nxgl_point_s *origin,
                                     unsigned int srcstride);
EXTERN void nxgl_copyrectangle_24bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *dest,
                                     FAR const void *src,
                                     FAR const struct nxgl_point_s *origin,
                                     unsigned int srcstride);
EXTERN void nxgl_copyrectangle_32bpp(FAR NX_PLANEINFOTYPE *pinfo,
                                     FAR const struct nxgl_rect_s *dest,
                                     FAR const void *src,
                                     FAR const struct nxgl_point_s *origin,
                                     unsigned int srcstride);

/****************************************************************************
 * Name: nxgl_rectcopy
 *
 * Description:
 *   This is essentially memcpy for rectangles.  We don't do structure
 *   assignments because some compilers are not good at that.
 *
 ****************************************************************************/

EXTERN void nxgl_rectcopy(FAR struct nxgl_rect_s *dest,
                          FAR const struct nxgl_rect_s *src);

/****************************************************************************
 * Name: nxgl_rectoffset
 *
 * Description:
 *   Offset the rectangle position by the specified dx, dy values.
 *
 ****************************************************************************/

EXTERN void nxgl_rectoffset(FAR struct nxgl_rect_s *dest,
                            FAR const struct nxgl_rect_s *src,
                            nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_vectoradd
 *
 * Description:
 *   Add two 2x1 vectors and save the result to a third.
 *
 ****************************************************************************/

EXTERN void nxgl_vectoradd(FAR struct nxgl_point_s *dest,
                           FAR const struct nxgl_point_s *v1,
                           FAR const struct nxgl_point_s *v2);

/****************************************************************************
 * Name: nxgl_vectorsubtract
 *
 * Description:
 *   Add subtract vector v2 from vector v1 and return the result in vector dest
 *
 ****************************************************************************/

EXTERN void nxgl_vectsubtract(FAR struct nxgl_point_s *dest,
                              FAR const struct nxgl_point_s *v1,
                              FAR const struct nxgl_point_s *v2);

/****************************************************************************
 * Name: nxgl_rectintersect
 *
 * Description:
 *   Return the rectangle representing the intersection of the two rectangles.
 *
 ****************************************************************************/

EXTERN void nxgl_rectintersect(FAR struct nxgl_rect_s *dest,
                               FAR const struct nxgl_rect_s *src1,
                               FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_intersecting
 *
 * Description:
 *   Return true if the rectangles intersect.
 *
 ****************************************************************************/

EXTERN bool nxgl_intersecting(FAR const struct nxgl_rect_s *rect1,
                              FAR const struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectadd
 *
 * Description:
 *   Return the rectangle that contains exactly two other rectanges.
 *
 ****************************************************************************/

EXTERN void nxgl_rectadd(FAR struct nxgl_rect_s *dest,
                         FAR const struct nxgl_rect_s *src1,
                         FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_rectunion
 *
 * Description:
 *   Given two rectanges, src1 and src2, return the larger rectangle that 
 *   contains both, dest.
 *
 ****************************************************************************/

EXTERN void nxgl_rectunion(FAR struct nxgl_rect_s *dest,
                           FAR const struct nxgl_rect_s *src1,
                           FAR const struct nxgl_rect_s *src2);

/****************************************************************************
 * Name: nxgl_nonintersecting
 *
 * Description:
 *   Return the regions of rectangle rect 1 that do not intersect with
 *   rect2.  This will be four rectangles ,some of which may be
 *   degenerate (and can be picked off with nxgl_nullrect)
 *
 ****************************************************************************/

EXTERN void nxgl_nonintersecting(FAR struct nxgl_rect_s result[4],
                                 FAR const struct nxgl_rect_s *rect1,
                                 FAR const struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectoverlap
 *
 * Description:
 *   Return true if the two rectangles overlap
 *
 ****************************************************************************/

EXTERN bool nxgl_rectoverlap(FAR struct nxgl_rect_s *rect1,
                             FAR struct nxgl_rect_s *rect2);

/****************************************************************************
 * Name: nxgl_rectinside
 *
 * Description:
 *   Return true if the point pt lies within rect.
 *
 ****************************************************************************/

EXTERN bool nxgl_rectinside(FAR const struct nxgl_rect_s *rect,
                            FAR const struct nxgl_point_s *pt);

/****************************************************************************
 * Name: nxgl_rectsize
 *
 * Description:
 *   Return the size of the specified rectangle.
 *
 ****************************************************************************/

EXTERN void nxgl_rectsize(FAR struct nxgl_size_s *size,
                          FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxgl_nullrect
 *
 * Description:
 *   Return true if the area of the retangle is <= 0.
 *
 ****************************************************************************/

EXTERN bool nxgl_nullrect(FAR const struct nxgl_rect_s *rect);

/****************************************************************************
 * Name: nxgl_runoffset
 *
 * Description:
 *   Offset the run position by the specified dx, dy values.
 *
 ****************************************************************************/

EXTERN void nxgl_runoffset(FAR struct nxgl_run_s *dest,
                           FAR const struct nxgl_run_s *src,
                           nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_runcopy
 *
 * Description:
 *   This is essentially memcpy for runs.  We don't do structure assignments
 *   because some compilers are not good at that.
 *
 ****************************************************************************/

EXTERN void nxgl_runcopy(FAR struct nxgl_run_s *dest,
                         FAR const struct nxgl_run_s *src);

/****************************************************************************
 * Name: nxgl_trapoffset
 *
 * Description:
 *   Offset the trapezoid position by the specified dx, dy values.
 *
 ****************************************************************************/

EXTERN void nxgl_trapoffset(FAR struct nxgl_trapezoid_s *dest,
                            FAR const struct nxgl_trapezoid_s *src,
                            nxgl_coord_t dx, nxgl_coord_t dy);

/****************************************************************************
 * Name: nxgl_trapcopy
 *
 * Description:
 *   This is essentially memcpy for trapezoids.  We don't do structure
 *   assignments because some compilers are not good at that.
 *
 ****************************************************************************/

EXTERN void nxgl_trapcopy(FAR struct nxgl_trapezoid_s *dest,
                          FAR const struct nxgl_trapezoid_s *src);

/****************************************************************************
 * Name: nxgl_colorcopy
 *
 * Description:
 *   This is essentially memcpy for colors.  This does very little for us
 *   other than hide all of the conditional compilation for planar colors
 *   in one place.
 *
 ****************************************************************************/

EXTERN void nxgl_colorcopy(nxgl_mxpixel_t dest[CONFIG_NX_NPLANES],
                           const nxgl_mxpixel_t src[CONFIG_NX_NPLANES]);

/****************************************************************************
 * Name: nxgl_splitline
 *
 * Description:
 *   In the general case, a line with width can be represented as a
 *   parallelogram with a triangle at the top and bottom.  Triangles and
 *   parallelograms are both degenerate versions of a trapeziod.  This
 *   function breaks a wide line into triangles and trapezoids.  This
 *   function also detects other degenerate cases:
 *
 *   1. If y1 == y2 then the line is horizontal and is better represented
 *      as a rectangle.
 *   2. If x1 == x2 then the line is vertical and also better represented
 *      as a rectangle.
 *   3. If the width of the line is 1, then there are no triangles at the
 *      top and bottome (this may also be the case if the width is narrow
 *      and the line is near vertical).
 *   4. If the line is oriented is certain angles, it may consist only of
 *      the upper and lower triangles with no trapezoid in between.  In
 *      this case, 3 trapezoids will be returned, but traps[1] will be
 *      degenerate.
 *
 * Input parameters:
 *   vector - A pointer to the vector described the line to be drawn.
 *   traps  - A pointer to a array of trapezoids (size 3).
 *   rect   - A pointer to a rectangle.
 *
 * Returned value:
 *   0: Line successfully broken up into three trapezoids.  Values in
 *      traps[0], traps[1], and traps[2] are valid.
 *   1: Line successfully represented by one trapezoid. Value in traps[1]
 *      is valid.
 *   2: Line successfully represented by one rectangle. Value in rect is
 *      valid
 *  <0: On errors, a negated errno value is returned.
 *
 ****************************************************************************/

EXTERN int nxgl_splitline(FAR struct nxgl_vector_s *vector,
                          FAR struct nxgl_trapezoid_s *traps,
                          FAR struct nxgl_rect_s *rect,
                          nxgl_coord_t linewidth);

/****************************************************************************
 * Name: nxgl_circlepts
 *
 * Description:
 *   Given a description of a circle, return a set of 16 points on the
 *   circumference of the circle.  These points may then be used by
 *   nx_drawcircle() or related APIs to draw a circle outline.
 *
 * Input parameters:
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   circle - A pointer the first entry in an array of 16 points where the
 *            circle points will be returned.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

EXTERN void nxgl_circlepts(FAR const struct nxgl_point_s *center,
                           nxgl_coord_t radius,
                           FAR struct nxgl_point_s *circle);

/****************************************************************************
 * Name: nxgl_circletraps
 *
 * Description:
 *   Given a description of a a circle, return 8 trapezoids that can be
 *   used to fill the circle by nx_fillcircle() and other interfaces.
 *
 * Input parameters:
 *   center - A pointer to the point that is the center of the circle
 *   radius - The radius of the circle in pixels.
 *   circle - A pointer the first entry in an array of 8 trapezoids where
 *            the circle description will be returned.
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

EXTERN void nxgl_circletraps(FAR const struct nxgl_point_s *center,
                             nxgl_coord_t radius,
                             FAR struct nxgl_trapezoid_s *circle);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_NX_NXGLIB_H */
