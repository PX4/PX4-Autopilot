mm/README.txt
=============

This directory contains the NuttX memory management logic.  This include:

1) The standard memory management functions as prototyped in stdlib.h as
   specified in the  Base definitions volume of IEEE Std 1003.1-2001.  This
   include the files:

   o Standard Interfaces: mm_malloc.c, mm_calloc.c, mm_realloc.c,
       mm_memalign.c, mm_free.c
   o Less-Standard Interfaces: mm_zalloc.c, mm_mallinfo.c
   o Internal Implementation: mm_initialize.c mm_sem.c  mm_addfreechunk.c
       mm_size2ndx.c mm_shrinkchunk.c, mm_internal.h
   o Build and Configuration files: Kconfig, Makefile

   Memory Models:

   o Small Memory Model.  If the MCU supports only 16-bit data addressing
     then the small memory model is automatically used.  The maximum size
     of the heap is then 64K.  The small memory model can also be forced
     MCUs with wider addressing by defining CONFIG_SMALL_MEMORY in the
     NuttX configuration file.
   o Large Memory Model.  Otherwise, the allocator uses a model that
     supports a heap of up to 4G.

   This implementation uses a variable length allocator with the following
   properties:

   o Overhead:  Either 8- or 4-bytes per allocation for large and small
     models, respectively.
   o Alignment:  All allocations are aligned to 8- or 4-bytes for large
     and small models, respectively.

2) Test Program.  There is also a host-best test program that can be
   used to verify the memory manager.  These are the file:

   Makefile.test, mm_test.c, and mm_environment.h.

   Build instructions:

   make -f Makefile.test

   The executable will be built in the top-level directory as nuttx/mm_text
   (or mm_test.exe under Cygwin).

3) Granule Allocator.  A non-standard granule allocator is also available
   in this directory  The granule allocator allocates memory in units
   of a fixed sized block ("granule").  All memory is aligned to the size
   of one granule.

   The granule allocator interfaces are defined in nuttx/include/nuttx/gran.h.
   The granule allocator consists of these files in this directory:

     mm_gran.h, mm_granalloc.c, mm_grancritical.c, mm_granfree.c
     mm_graninit.c

   The granule allocator is not used anywhere within the base NuttX code
   as of this writing.  The intent of the granule allocator is to provide
   a tool to support platform-specific management of aligned DMA memory.

   NOTE: Because each granule is aligned to the granule size and each
   allocations is in units of the granule size, selection of the granule
   size is important:  Larger granules will give better performance and
   less overhead but more losses of memory due to alignment and quantization
   waste.

   The current implementation also restricts the maximum allocation size
   to 32 granules.  That restriction could be eliminated with some
   additional coding effort, but currently requires larger granule
   sizes for larger allocations.
