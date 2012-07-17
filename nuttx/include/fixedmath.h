/****************************************************************************
 * include/fixedmath.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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

#ifndef __INCLUDE_FIXEDMATH_H
#define __INCLUDE_FIXEDMATH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdint.h>

/**************************************************************************
 * Definitions
 **************************************************************************/

/* Common numbers */

#define b8HUNDRED       0x6400                  /* 100 */
#define b8TEN           0x0a00                  /* 10 */
#define b8ONE           0x0100                  /* 1 */
#define b8HALF          0x0080                  /* 0.5 */
#define b8ONETENTH      0x001a                  /* 0.1 (acutally 0.1015625) */
#define b8ONEHUNDRTH    0x0003                  /* 0.01 (actualy 0.0117198765) */
#define b8HALFPI        0x0192                  /* 1.5703125 */
#define b8PI            0x0324                  /* 3.1406250 */
#define b8TWOPI         0x0648                  /* 6.2812500 */

#define b8MAX           0x7fff                  /* Max value of b8_t */
#define ub8MAX          0xffff                  /* Max value of rb8_t */
#define b8MIN           0x8000                  /* Min value of b8_t */
#define ub8MIN          0x0000                  /* Min value of ub8_t */

#define b16THOUSAND     0x03e80000              /* 1000 */
#define b16HUNDRED      0x00640000              /* 100 */
#define b16TEN          0x000a0000              /* 10 */
#define b16ONE          0x00010000              /* 1 */
#define b16HALF         0x00008000              /* 0.5 */
#define b16ONETENTH     0x0000199a              /* 0.1 (actually 0.100006..) */
#define b16ONEHUNDRTH   0x0000028f              /* 0.01 (actually 0.0099945..) */
#define b16ONETHOUSTH   0x00000042              /* 0.001 (actually 0.000100708..)*/
#define b16HALFPI       0x0001921f              /* 1.57078552246 */
#define b16PI           0x0003243f              /* 3.14158630371 */
#define b16TWOPI        0x0006487b              /* 6.28312683105 */

#define b16MAX          0x7fffffff              /* Max value of b16_t */
#define ub16MAX         0xffffffff              /* Max value of ub16_t */
#define b16MIN          0x80000000              /* Min value of b16_t */
#define ub16MIN         0x00000000              /* Min value of ub16_t */

#define b32MILLION      0x000f424000000000      /* 1000000 */
#define b32THOUSAND     0x000003e800000000      /* 1000 */
#define b32HUNDRED      0x0000006400000000      /* 100 */
#define b32TEN          0x0000000a00000000      /* 10 */
#define b32ONE          0x0000000100000000      /* 1 */
#define b32HALF         0x0000000080000000      /* 0.5 */
#define b32ONETENTH     0x000000001999999a      /* 0.1 */
#define b32ONEHUNDRTH   0x00000000028f5c29      /* 0.01 */
#define b32ONETHOUSTH   0x0000000000418937      /* 0.001 */
#define b32ONETENTHOU   0x0000000000068db9      /* 0.0001 */
#define b32HALFPI       0x00000001921eb9ff      /* 1.57078134990 */
#define b32PI           0x00000003243f6b4f      /* 3.14159269980 */
#define b32TWOPI        0x00000006487ae7fd      /* 6.28312539984 */

#define b32MAX          0x7fffffffffffffff      /* Max value of b16_t */
#define ub32MAX         0xffffffffffffffff      /* Max value of ub16_t */
#define b32MIN          0x8000000000000000      /* Min value of b16_t */
#define ub32MIN         0x0000000000000000      /* Min value of ub16_t */

/* Conversions between b16 and b8 *****************************************/

#define b8tob16(b)      (((b16_t)(b)) << 8)
#define ub8toub16(b)    (((ub16_t)(b)) << 8)
#define b16tob8(b)      (b8_t)(((b)+0x0080)>>8)
#define ub16toub8(b)    (ub8_t)(((b)+0x0080)>>8)

#ifdef CONFIG_HAVE_LONG_LONG
#  define b8tob32(b)    (((b32_t)(b)) << 24)
#  define ub8toub32(b)  (((ub32_t)(b)) << 24)
#  define b16tob32(b)   (((b32_t)(b)) << 16)
#  define ub16toub32(b) (((ub32_t)(b)) << 16)
#  define b32tob16(b)   (b16_t)(((b) + 0x0000000000008000)>>16)
#  define ub32toub16(b) (ub16_t)(((b) + 0x0000000000008000)>>16)
#  define b32tob8(b)    (b8_t)(((b) + 0x0000000000000080)>>8)
#endif

/* 16-bit values with 8 bits of precision *********************************/

/* Conversions */

#define b8toi(a)        ((a) >> 8)              /* Conversion to integer */
#define itob8(i)        (((b8_t)(i)) << 8)      /* Conversion from integer */
#define uitoub8(i)      (((ub8_t)(i)) << 8)     /* Conversion from unsigned integer */
#define b8tof(b)        (((float)b)/256.0)      /* Conversion to float */
#define ftob8(f)        (b8_t)(((f)*256.0))     /* Conversion from float */
#define b8trunc(a)      ((a) & 0xff00)          /* Truncate to integer b8 */
#define b8round(a)      (((a)+0x0080) & 0xff00) /* Round to integer b8 */
#define b8frac(a)       ((a) & 0x00ff)          /* Take fractional part */

/* Operators */

#define ub8inv(b)       (0x8000/((b)>>1))       /* Inversion (b8=b15/b7) */
#define b8inv(b)        (0x4000/((b)>>2))       /* Inversion (b8=b14/b6) */
#define b8addb8(a,b)    ((a)+(b))               /* Addition */
#define b8addi(a,i)     ((a)+itob8(i))          /* Add integer from b16 */
#define b8subb8(a,b)    ((a)-(b))               /* Subtraction */
#define b8subi(a,i)     ((a)-itob8(i))          /* Subtract integer from b8 */
#define b8mulb8(a,b)    b16tob8((b16_t)(a)*(b16_t)(b) /* Muliplication */
#define ub8mulub8(a,b)  ub16toub8((ub16_t)(a)*(ub16_t)(b) /* Muliplication */
#define b8muli(a,i)     ((a)*(i))               /* Simple multiplication by integer */
#define b8sqr(a)        b8mulb8(a,a)            /* Square */
#define ub8sqr(a)       ub8mulub8(a,a)          /* Square */
#define b8divb8(a,b)    b8tob16(a)/(b16_t)(b)   /* Division */
#define ub8divub8(a,b)  ub8toub16(a)/(ub16_t)(b) /* Division */
#define b8divi(a,i)     ((a)/(i))               /* Simple division by integer */
#define b8idiv(i,j)     (((i)<<8)/j)            /* Division of integer, b8 result */

/* 32-bit values with 16 bits of precision ********************************/

/* Conversions */

#define b16toi(a)       ((a) >> 16)             /* Conversion to integer */
#define itob16(i)       (((b16_t)(i)) << 16)    /* Conversion from integer */
#define uitoub16(i)     (((ub16_t)(i)) << 16)   /* Conversion from unsigned integer */
#define b16tof(b)       (((float)b)/65536.0)    /* Conversion to float */
#define ftob16(f)       (b16_t)(((f)*65536.0))  /* Conversion from float */
#define b16trunc(a)     ((a) & 0xffff0000)      /* Truncate to integer */
#define b16round(a)     (((a)+0x00008000) & 0xffff0000)
#define b16frac(a)      ((a) & 0x0000ffff)      /* Take fractional part */

/* Operators */

#define ub16inv(b)      (0x80000000/((b)>>1))   /* Inversion (b16=b31/b15) */
#define b16inv(b)       (0x40000000/((b)>>2))   /* Inversion (b16=b30/b14) */
#define b16addb16(a,b)  ((a)+(b))               /* Addition */
#define b16addi(a,i)    ((a)+itob16(i))         /* Add integer to b16 */
#define b16subb16(a,b)  ((a)-(b))               /* Subtraction */
#define b16subi(a,i)    ((a)-itob16(i))         /* Subtract integer from b16 */
#define b16muli(a,i)    ((a)*(i))               /* Simple multiplication by integer */
#define b16divi(a,i)    ((a)/(i))               /* Simple division by integer*/
#define b16idiv(i,j)    (((i)<<16)/j)           /* Division of integer, b16 result */

#ifdef CONFIG_HAVE_LONG_LONG
#  define b16mulb16(a,b)   b32tob16((b32_t)(a)*(b32_t)(b)) /* Muliplication */
#  define ub16mulub16(a,b) ub32toub16((ub32_t)(a)*(ub32_t)(b)
#  define b16sqr(a)        b16mulb16(a,a)                  /* Square */
#  define ub16sqr(a)       ub16mulub16(a,a)                /* Square */
#  define b16divb16(a,b)   b16tob32(a)/(b32_t)(b)          /* Division */
#  define ub16divub16(a,b) ub16toub32(a)/(ub32_t)(b)
#endif

/**************************************************************************
 * Public Types
 **************************************************************************/

typedef int16_t  b8_t;
typedef uint16_t ub8_t;
typedef int32_t  b16_t;
typedef uint32_t ub16_t;
#ifdef CONFIG_HAVE_LONG_LONG
typedef int64_t  b32_t;
typedef uint64_t ub32_t;
#endif

/**************************************************************************
 * Global Functions
 **************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#ifndef CONFIG_HAVE_LONG_LONG
EXTERN b16_t  b16mulb16(b16_t m1, b16_t m2);
EXTERN ub16_t ub16mulub16(ub16_t m1, ub16_t m2);
EXTERN b16_t  b16sqr(b16_t a);
EXTERN ub16_t ub16sqr(ub16_t a);
EXTERN b16_t  b16divb16(b16_t num, b16_t denom);
EXTERN ub16_t ub16divub16(ub16_t num, ub16_t denom);
#endif

EXTERN b16_t b16sin(b16_t rad);
EXTERN b16_t b16cos(b16_t rad);
EXTERN b16_t b16atan2(b16_t y, b16_t x);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __INCLUDE_FIXEDMATH_H */
