fs/mmap README File
===================

NuttX operates in a flat open address space and is focused on MCUs that do
support Memory Management Units (MMUs).  Therefore, NuttX generally does not
require mmap() functionality and the MCUs generally cannot support true
memory-mapped files.

However, memory mapping of files is the mechanism used by NXFLAT, the NuttX
tiny binary format, to get files into memory in order to execute them.
mmap() support is therefore required to support NXFLAT.  There are two
conditions where mmap() can be supported:

1. mmap can be used to support eXecute In Place (XIP) on random access media
   under the following very restrictive conditions:

   a. The filesystem supports the FIOC_MMAP ioctl command.  Any file
      system that maps files contiguously on the media should support
      this ioctl. (vs. file system that scatter files over the media
      in non-contiguous sectors).  As of this writing, ROMFS is the
      only file system that meets this requirement.

   b. The underlying block driver supports the BIOC_XIPBASE ioctl
      command that maps the underlying media to a randomly accessible
      address. At  present, only the RAM/ROM disk driver does this.

   Some limitations of this approach are as follows:

   a. Since no real mapping occurs, all of the file contents are "mapped"
      into memory.

   b. All mapped files are read-only.

   c. There are no access privileges.

2. If CONFIG_FS_RAMMAP is defined in the configuration, then mmap() will
   support simulation of memory mapped files by copying files whole
   into RAM.  These copied files have some of the properties of
   standard memory mapped files.  There are many, many exceptions
   exceptions, however.  Some of these include:

   a. The goal is to have a single region of memory that represents a single
      file and can be shared by many threads.  That is, given a filename a
      thread should be able to open the file, get a file descriptor, and
      call mmap() to get a memory region.  Different file descriptors opened
      with the same file path should get the same memory region when mapped.

      The limitation in the current design is that there is insufficient
      knowledge to know that these different file descriptors correspond to
      the same file.  So, for the time being, a new memory region is created
      each time that rammap() is called. Not very useful!

   b. The entire mapped portion of the file must be present in memory.
      Since it is assumed the the MCU does not have an MMU, on-demanding
      paging in of file blocks cannot be supported. Since the while mapped
      portion of the file must be present in memory, there are limitations
      in the size of files that may be memory mapped (especially on MCUs
      with no significant RAM resources).

   c. All mapped files are read-only.  You can write to the in-memory image,
      but the file contents will not change.
 
   d. There are no access privileges.

   e. Since there are no processes in NuttX, all mmap() and munmap()
      operations have immediate, global effects.  Under Linux, for example,
      munmap() would eliminate only the mapping with a process; the mappings
      to the same file in other processes would not be effected.

   f. Like true mapped file, the region will persist after closing the file
      descriptor.  However, at present, these ram copied file regions are
      *not* automatically "unmapped" (i.e., freed) when a thread is terminated.
      This is primarily because it is not possible to know how many users
      of the mapped region there are and, therefore, when would be the
      appropriate time to free the region (other than when munmap is called).

      NOTE: Note, if the design limitation of a) were solved, then it would be
      easy to solve exception d) as well.
