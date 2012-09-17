/****************************************************************************
 * examples/mm/mm_main.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NTEST_ALLOCS 32

/* #define STOP_ON_ERRORS do{}while(0) */
#define STOP_ON_ERRORS exit(1)

/* All other definitions derive from these two */

#define MM_MIN_SHIFT      4  /* 16 bytes */
#define MM_MIN_CHUNK     (1 << MM_MIN_SHIFT)
#define MM_GRAN_MASK     (MM_MIN_CHUNK-1)
#define MM_ALIGN_UP(a)   (((a) + MM_GRAN_MASK) & ~MM_GRAN_MASK)
#define MM_ALIGN_DOWN(a) ((a) & ~MM_GRAN_MASK)

#ifdef CONFIG_SMALL_MEMORY
# define SIZEOF_MM_ALLOCNODE   4
#else
# define SIZEOF_MM_ALLOCNODE   8
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
/* Test allocations */

static const int   alloc_sizes[NTEST_ALLOCS] =
{
  1024,     12,    962,   5692, 10254,   111,   9932,    601,
   222,   2746,      3, 124321,    68,   776,   6750,    852,
  4732,     28,    901,    480,  5011,  1536,   2011,  81647,
   646,   1646,  69179,    194,  2590,     7,    969,     70
};

static const int   realloc_sizes[NTEST_ALLOCS] =
{
    18,   3088,    963,    123,   511, 11666,   3723,     42,
  9374,   1990,   1412,      6,   592,  4088,     11,   5040,
  8663,  91255,     28,   4346,  9172,   168,    229,   4734,
 59139,    221,   7830,  30421,  1666,     4,    812,    416
};

static const int random1[NTEST_ALLOCS] =
{
    20,     11,      3,     31,     9,    29,      7,     17,
    21,      2,     26,     18,    14,    25,      0,     10,
    27,     19,     22,     28,     8,    30,     12,     15,
     4,      1,     24,      6,    16,    13,      5,     23
};

static const int random2[NTEST_ALLOCS] =
{
     2,     19,     12,     23,    30,    11,     27,      4,
    20,      7,      0,     16,    28,    15,      5,     24,
    10,     17,     25,     31,     8,    29,      3,     26,
     9,     18,     22,     13,     1,    21,     14,      6
};

static const int random3[NTEST_ALLOCS] =
{
    8,      17,      3,     18,     26,   23,     30,     11,
    12,     22,      4,     20,     25,   10,     27,      1,
    29,     14,     19,     21,      0,   31,      7,     24,
     9,     15,      2,     28,     16,    6,     13,      5
};

static const int alignment[NTEST_ALLOCS/2] =
{
    128,  2048, 131072,   8192,    32,  32768, 16384 , 262144,
    512,  4096,  65536,      8,     64,  1024,    16,       4
};

static void        *allocs[NTEST_ALLOCS];
static struct       mallinfo alloc_info;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mm_showmallinfo(void)
{
  alloc_info = mallinfo();
  printf("     mallinfo:\n");
  printf("       Total space allocated from system = %ld\n",
	 alloc_info.arena);
  printf("       Number of non-inuse chunks        = %ld\n",
	 alloc_info.ordblks);
  printf("       Largest non-inuse chunk           = %ld\n",
	 alloc_info.mxordblk);
  printf("       Total allocated space             = %ld\n",
	 alloc_info.uordblks);
  printf("       Total non-inuse space             = %ld\n",
	 alloc_info.fordblks);
}

static void do_mallocs(void **mem, const int *size, const int *seq, int n)
{
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = seq[i];
      if (!mem[j])
        {
          printf("(%d)Allocating %d bytes\n", i,  size[j]);
          mem[j] = malloc(size[j]);
          printf("(%d)Memory allocated at %p\n", i, mem[j]);
          if (mem[j] == NULL)
            {
              int allocsize = MM_ALIGN_UP(size[j] + SIZEOF_MM_ALLOCNODE);
              fprintf(stderr, "(%d)malloc failed for allocsize=%d\n", i, allocsize);
              if (allocsize > alloc_info.mxordblk)
                {
                   fprintf(stderr, "   Normal, largest free block is only %ld\n", alloc_info.mxordblk);
                }
              else
                {
                  fprintf(stderr, "   ERROR largest free block is %ld\n", alloc_info.mxordblk);
                  exit(1);
                }
            }
          else
            {
              memset(mem[j], 0xAA, size[j]);
            }

          mm_showmallinfo();
        }
    }
}

static void do_reallocs(void **mem, const int *oldsize, const int *newsize, const int *seq, int n)
{
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = seq[i];
      printf("(%d)Re-allocating at %p from %d to %d bytes\n",
	     i, mem[j], oldsize[j], newsize[j]);
      mem[j] = realloc(mem[j], newsize[j]);
      printf("(%d)Memory re-allocated at %p\n", i, mem[j]);
      if (mem[j] == NULL)
	{
          int allocsize = MM_ALIGN_UP(newsize[j] + SIZEOF_MM_ALLOCNODE);
          fprintf(stderr, "(%d)realloc failed for allocsize=%d\n", i, allocsize);
          if (allocsize > alloc_info.mxordblk)
            {
              fprintf(stderr, "   Normal, largest free block is only %ld\n", alloc_info.mxordblk);
            }
          else
            {
              fprintf(stderr, "   ERROR largest free block is %ld\n", alloc_info.mxordblk);
              exit(1);
            }
	}
      else
        {
          memset(mem[j], 0x55, newsize[j]);
        }

      mm_showmallinfo();
    }
}

static void do_memaligns(void **mem, const int *size, const int *align, const int *seq, int n)
{
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = seq[i];
      printf("(%d)Allocating %d bytes aligned to 0x%08x\n",
	     i,  size[j], align[i]);
      mem[j] = memalign(align[i], size[j]);
      printf("(%d)Memory allocated at %p\n", i, mem[j]);
      if (mem[j] == NULL)
	{
          int allocsize = MM_ALIGN_UP(size[j] + SIZEOF_MM_ALLOCNODE) + 2*align[i];
          fprintf(stderr, "(%d)memalign failed for allocsize=%d\n", i, allocsize);
          if (allocsize > alloc_info.mxordblk)
            {
              fprintf(stderr, "   Normal, largest free block is only %ld\n", alloc_info.mxordblk);
            }
          else
            {
              fprintf(stderr, "   ERROR largest free block is %ld\n", alloc_info.mxordblk);
              exit(1);
            }
	}
      else
        {
          memset(mem[j], 0x33, size[j]);
        }

      mm_showmallinfo();
    }
}

static void do_frees(void **mem, const int *size, const int *seq, int n)
{
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = seq[i];
      printf("(%d)Releasing memory at %p (size=%d bytes)\n",
	     i, mem[j], size[j]);
      free(mem[j]);
      mem[j] = NULL;

      mm_showmallinfo();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_main
 ****************************************************************************/

int mm_main(int argc, char *argv[])
{
  mm_showmallinfo();

  /* Allocate some memory */

  do_mallocs(allocs, alloc_sizes, random1, NTEST_ALLOCS);

  /* Re-allocate the memory */

  do_reallocs(allocs, alloc_sizes, realloc_sizes, random2, NTEST_ALLOCS);

  /* Release the memory */

  do_frees(allocs, realloc_sizes, random3, NTEST_ALLOCS);

  /* Allocate aligned memory */

  do_memaligns(allocs, alloc_sizes, alignment, random2, NTEST_ALLOCS/2);
  do_memaligns(allocs, alloc_sizes, alignment, &random2[NTEST_ALLOCS/2], NTEST_ALLOCS/2);

  /* Release aligned memory */

  do_frees(allocs, alloc_sizes, random1, NTEST_ALLOCS);

  printf("TEST COMPLETE\n");
  return 0;
}
