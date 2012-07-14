/************************************************************************
 * mm/mm_test.c
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
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/* Fake NuttX dependencies */

#define FAR
#define CONFIG_MM_REGIONS 2
#undef  CONFIG_MM_SMALL
#define CONFIG_CAN_PASS_STRUCTS 1
#undef  CONFIG_SMALL_MEMORY

#include "mm_internal.h"

/* Pre-processor Definitions */

#define TEST_HEAP1_SIZE 0x00080000
#define TEST_HEAP2_SIZE 0x00080000
#define NTEST_ALLOCS 32

/* #define STOP_ON_ERRORS do{}while(0) */
#define STOP_ON_ERRORS exit(1)

/************************************************************************
 * Private Data
 ************************************************************************/

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
static unsigned int g_reportedheapsize = 0;
static unsigned int g_actualheapsize = 0;

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: mm_findinfreelist
 ************************************************************************/

static int mm_findinfreelist(struct mm_freenode_s *node)
{
  struct mm_freenode_s *list;

  for(list = &g_nodelist[0];
      list;
      list = list->flink)
    {
      if (list == node)
        {
          return 1;
        }
    }

  return 0;
}

/************************************************************************
 * Name: mm_showchunkinfo
 ************************************************************************/

static void mm_showchunkinfo(void)
{
  struct mm_allocnode_s *node;
#if CONFIG_MM_REGIONS > 1
  int region;
#else
# define region 0
#endif
  int found;

  printf("     CHUNK LIST:\n");

  /* Visit each region */

#if CONFIG_MM_REGIONS > 1
  for (region = 0; region < g_nregions; region++)
#endif
    {
      /* Visit each node in each region */

      for (node = g_heapstart[region];
           node < g_heapend[region];
           node = (struct mm_allocnode_s *)((char*)node + node->size))
        {
          printf("       %p 0x%08x 0x%08x %s",
                 node, node->size, node->preceding & ~MM_ALLOC_BIT,
                 node->preceding & MM_ALLOC_BIT ? "Allocated" : "Free     ");
          found = mm_findinfreelist((struct mm_freenode_s *)node);
          if (found && (node->preceding & MM_ALLOC_BIT) != 0)
            {
              printf(" Should NOT have been in free list\n");
            }
          else if (!found && (node->preceding & MM_ALLOC_BIT) == 0)
            {
              printf(" SHOULD have been in free listT\n");
            }
          else
            {
              printf(" OK\n");
            }
        }
    }
#undef region
}

/************************************************************************
 * Name: mm_showfreelist
 ************************************************************************/

static void mm_showfreelist(void)
{
  struct mm_freenode_s *prev;
  struct mm_freenode_s *node;
  int i = 0;

  printf("     FREE NODE LIST:\n");
  for(prev = NULL, node = &g_nodelist[0];
      node;
      prev = node, node = node->flink)
    {
      /* Dump "fake" nodes in a different way */

      if (node->size == 0)
        {
          printf("       [NODE %2d]         %08x %08x %08x\n",
                 i, node->preceding, (int)node->flink, (int)node->blink);
          i++;
        }
      else
        {
          printf("       %08x %08x %08x %08x %08x\n",
                 (int)node, node->size, node->preceding, (int)node->flink, (int)node->blink);
        }

      /* Verify all backward links */

      if (node->blink != prev)
        {
          fprintf(stderr, "Backward link is wrong:  Is %p, should be %p\n",
                  node->blink, prev);
          STOP_ON_ERRORS;
        }
    }
}

/************************************************************************
 * Name: mm_showmallinfo
 ************************************************************************/

static void mm_showmallinfo(void)
{
  int sval;

  mm_showchunkinfo();
  mm_showfreelist();
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

  sval = mm_getsemaphore();
  if (sval != 1)
    {
      fprintf(stderr, "After mallinfo, semaphore count=%d, should be 1\n", sval);
      STOP_ON_ERRORS;
    }

  if (!g_reportedheapsize)
    {
      g_reportedheapsize = alloc_info.uordblks + alloc_info.fordblks;
      if (g_reportedheapsize > g_actualheapsize + 16*CONFIG_MM_REGIONS ||
          g_reportedheapsize < g_actualheapsize -16*CONFIG_MM_REGIONS)
        {
          fprintf(stderr, "Total memory %d not close to uordlbks=%d + fordblks=%d = %d\n",
                 g_actualheapsize, alloc_info.uordblks, alloc_info.fordblks, g_reportedheapsize);
          STOP_ON_ERRORS;
        }
    }
  else if (alloc_info.uordblks + alloc_info.fordblks != g_reportedheapsize)
    {
      fprintf(stderr, "Total memory %d != uordlbks=%d + fordblks=%d\n",
             g_reportedheapsize, alloc_info.uordblks, alloc_info.fordblks);
      STOP_ON_ERRORS;
    }
}

/************************************************************************
 * Name: do_mallocs
 ************************************************************************/

static void do_mallocs(void **mem, const int *size, const int *rand,
                       int n)
{
  int sval;
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = rand[i];
      if (!mem[j])
        {
          printf("(%d)Allocating %d bytes\n", i,  size[j]);
          mem[j] = mm_malloc(size[j]);
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

          sval = mm_getsemaphore();
          if (sval != 1)
            {
              fprintf(stderr, "   After malloc semaphore count=%d, should be 1\n", sval);
              STOP_ON_ERRORS;
            }

          mm_showmallinfo();
        }
    }
}

/************************************************************************
 * Name: do_reallocs
 ************************************************************************/

static void do_reallocs(void **mem, const int *oldsize,
                        const int *newsize, const int *rand, int n)
{
  int sval;
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = rand[i];
      printf("(%d)Re-allocating at %p from %d to %d bytes\n",
             i, mem[j], oldsize[j], newsize[j]);
      mem[j] = mm_realloc(mem[j], newsize[j]);
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

      sval = mm_getsemaphore();
      if (sval != 1)
        {
          fprintf(stderr, "   After realloc semaphore count=%d, should be 1\n", sval);
          STOP_ON_ERRORS;
        }

      mm_showmallinfo();
    }
}

/************************************************************************
 * Name: do_memaligns
 ************************************************************************/

static void do_memaligns(void **mem, const int *size, const int *align,
                         const int *rand, int n)
{
  int sval;
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = rand[i];
      printf("(%d)Allocating %d bytes aligned to 0x%08x\n",
             i,  size[j], align[i]);
      mem[j] = mm_memalign(align[i], size[j]);
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

      sval = mm_getsemaphore();
      if (sval != 1)
        {
          fprintf(stderr, "   After memalign semaphore count=%d, should be 1\n", sval);
          STOP_ON_ERRORS;
        }

      mm_showmallinfo();
    }
}

/************************************************************************
 * Name: do_frees
 ************************************************************************/

static void do_frees(void **mem, const int *size, const int *rand, int n)
{
  int sval;
  int i;
  int j;

  for (i = 0; i < n; i++)
    {
      j = rand[i];
      printf("(%d)Releasing memory at %p (size=%d bytes)\n",
             i, mem[j], size[j]);
      mm_free(mem[j]);
      mem[j] = NULL;

      sval = mm_getsemaphore();
      if (sval != 1)
        {
          fprintf(stderr, "   After free semaphore count=%d, should be 1\n", sval);
          STOP_ON_ERRORS;
        }

      mm_showmallinfo();
    }
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: main
 ************************************************************************/

int main(int argc, char **argv, char **envp)
{
  void *heap1_base;
  void *heap2_base;
  int i, j;

  /* Allocate a heap */

  printf("Allocating test heap #1 of %ldKb\n", TEST_HEAP1_SIZE/1024);
  heap1_base = malloc(TEST_HEAP1_SIZE);
  printf("Allocated  heap1_base=%p\n", heap1_base);
  if (heap1_base == 0)
    {
      fprintf(stderr, "Failed to allocate test heap #1\n");
      exit(1);
    }

  printf("Allocating test heap #2 of %ldKb\n", TEST_HEAP2_SIZE/1024);
  heap2_base = malloc(TEST_HEAP2_SIZE);
  printf("Allocated  heap2_base=%p\n", heap2_base);
  if (heap2_base == 0)
    {
      fprintf(stderr, "Failed to allocate test heap #2\n");
      exit(1);
    }

  /* Initialize the memory manager */

  mm_initialize(heap1_base, TEST_HEAP1_SIZE);
  g_actualheapsize = TEST_HEAP1_SIZE;
  mm_showmallinfo();

  mm_addregion(heap2_base, TEST_HEAP2_SIZE);
  g_reportedheapsize = 0;
  g_actualheapsize += TEST_HEAP2_SIZE;
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

  /* Clean up and exit */

  free(heap1_base);
  free(heap2_base);

  printf("TEST COMPLETE\n");
  return 0;
}
