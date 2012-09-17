/****************************************************************************
 * mm/mm_internal.h
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

#ifndef __MM_MM_INTERNAL_H
#define __MM_MM_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If the MCU has a small (16-bit) address capability, then we will use
 * a smaller chunk header that contains 16-bit size/offset information.
 * We will also use the smaller header on MCUs with wider addresses if
 * CONFIG_MM_SMALL is selected.  This configuration is common with MCUs
 * that have a large FLASH space, but only a tiny internal SRAM.
 */

#ifdef CONFIG_SMALL_MEMORY
  /* If the MCU has a small addressing capability, then for the smaller
   * chunk header.
   */

#  undef  CONFIG_MM_SMALL
#  define CONFIG_MM_SMALL 1
#endif

/* Chunk Header Definitions *************************************************/
/* These definitions define the characteristics of allocator
 *
 * MM_MIN_SHIFT is used to define MM_MIN_CHUNK.
 * MM_MIN_CHUNK - is the smallest physical chunk that can
 *   be allocated.  It must be at least a large as
 *   sizeof(struct mm_freenode_s).  Larger values may
 *   improve performance slightly, but will waste memory
 *   due to quantization losses.
 *
 * MM_MAX_SHIFT is used to define MM_MAX_CHUNK
 * MM_MAX_CHUNK is the largest, contiguous chunk of memory
 *   that can be allocated.  It can range from 16-bytes to
 *   4Gb.  Larger values of MM_MAX_SHIFT can cause larger
 *   data structure sizes and, perhaps, minor performance
 *   losses.
 */

#ifdef CONFIG_MM_SMALL
#  define MM_MIN_SHIFT    4  /* 16 bytes */
#  define MM_MAX_SHIFT   15  /* 32 Kb */
#else
#  define MM_MIN_SHIFT    4  /* 16 bytes */
#  define MM_MAX_SHIFT   22  /*  4 Mb */
#endif

/* All other definitions derive from these two */

#define MM_MIN_CHUNK     (1 << MM_MIN_SHIFT)
#define MM_MAX_CHUNK     (1 << MM_MAX_SHIFT)
#define MM_NNODES        (MM_MAX_SHIFT - MM_MIN_SHIFT + 1)

#define MM_GRAN_MASK     (MM_MIN_CHUNK-1)
#define MM_ALIGN_UP(a)   (((a) + MM_GRAN_MASK) & ~MM_GRAN_MASK)
#define MM_ALIGN_DOWN(a) ((a) & ~MM_GRAN_MASK)

/* An allocated chunk is distinguished from a free chunk by
 * bit 31 of the 'preceding' chunk size.  If set, then this is
 * an allocated chunk.
 */

#ifdef CONFIG_MM_SMALL
# define MM_ALLOC_BIT    0x8000
#else
# define MM_ALLOC_BIT    0x80000000
#endif
#define MM_IS_ALLOCATED(n) \
  ((int)((struct mm_allocnode_s*)(n)->preceding) < 0))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Determine the size of the chunk size/offset type */

#ifdef CONFIG_MM_SMALL
   typedef uint16_t mmsize_t;
#  define MMSIZE_MAX 0xffff
#else
   typedef size_t mmsize_t;
#  define MMSIZE_MAX SIZE_MAX
#endif

/* This describes an allocated chunk.  An allocated chunk is
 * distinguished from a free chunk by bit 15/31 of the 'preceding' chunk
 * size.  If set, then this is an allocated chunk.
 */

struct mm_allocnode_s
{
  mmsize_t size;           /* Size of this chunk */
  mmsize_t preceding;      /* Size of the preceding chunk */
};

/* What is the size of the allocnode? */

#ifdef CONFIG_MM_SMALL
# define SIZEOF_MM_ALLOCNODE   4
#else
# define SIZEOF_MM_ALLOCNODE   8
#endif

#define CHECK_ALLOCNODE_SIZE \
  DEBUGASSERT(sizeof(struct mm_allocnode_s) == SIZEOF_MM_ALLOCNODE)

/* This describes a free chunk */

struct mm_freenode_s
{
  mmsize_t size;                   /* Size of this chunk */
  mmsize_t preceding;              /* Size of the preceding chunk */
  FAR struct mm_freenode_s *flink; /* Supports a doubly linked list */
  FAR struct mm_freenode_s *blink;
};

/* What is the size of the freenode? */

#ifdef CONFIG_MM_SMALL
#  ifdef CONFIG_SMALL_MEMORY
#     define SIZEOF_MM_FREENODE 8
#  else
#     define SIZEOF_MM_FREENODE 12
#  endif
#else
# define SIZEOF_MM_FREENODE     16
#endif

#define CHECK_FREENODE_SIZE \
  DEBUGASSERT(sizeof(struct mm_freenode_s) == SIZEOF_MM_FREENODE)

/* Normally defined in stdlib.h */

#ifdef MM_TEST
struct mallinfo
{
  int arena;    /* This is the total size of memory allocated
                 * for use by malloc in bytes. */
  int ordblks;  /* This is the number of free (not in use) chunks */
  int mxordblk; /* Size of the largest free (not in use) chunk */
  int uordblks; /* This is the total size of memory occupied by
                 * chunks handed out by malloc. */
  int fordblks; /* This is the total size of memory occupied
                 * by free (not in use) chunks.*/
};
#endif

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/* This is the size of the heap provided to mm */

extern size_t  g_heapsize;

/* This is the first and last nodes of the heap */

extern FAR struct mm_allocnode_s *g_heapstart[CONFIG_MM_REGIONS];
extern FAR struct mm_allocnode_s *g_heapend[CONFIG_MM_REGIONS];

#if CONFIG_MM_REGIONS > 1
extern int g_nregions;
#else
# define g_nregions 1
#endif

/* All free nodes are maintained in a doubly linked list.  This
 * array provides some hooks into the list at various points to
 * speed searches for free nodes.
 */

extern FAR struct mm_freenode_s g_nodelist[MM_NNODES];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef MM_TEST
FAR void  *mm_malloc(size_t);
void       mm_free(void*);
FAR void  *mm_realloc(void*, size_t);
FAR void  *mm_memalign(size_t, size_t);
FAR void  *mm_zalloc(size_t);
FAR void  *mm_calloc(size_t, size_t);
#ifdef CONFIG_CAN_PASS_STRUCTS
struct mallinfo mallinfo(void);
#else
int        mallinfo(struct mallinfo *info);
#endif
#endif

void       mm_shrinkchunk(FAR struct mm_allocnode_s *node, size_t size);
void       mm_addfreechunk(FAR struct mm_freenode_s *node);
int        mm_size2ndx(size_t size);
void       mm_seminitialize(void);
void       mm_takesemaphore(void);
void       mm_givesemaphore(void);
#ifdef MM_TEST
int        mm_getsemaphore(void);
#endif

#endif /* __MM_MM_INTERNAL_H */
