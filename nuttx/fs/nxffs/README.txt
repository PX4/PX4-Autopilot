NXFFS README
^^^^^^^^^^^^

This README file contains information about the implemenation of the NuttX
wear-leveling FLASH file system, NXFFS.

Contents:

  General NXFFS organization
  General operation
  Headers
  NXFFS Limitations
  Multiple Writers
  ioctls
  Things to Do

General NXFFS organization
==========================

The following example assumes 4 logical blocks per FLASH erase block.  The
actual relationship is determined by the FLASH geometry reported by the MTD
driver.

ERASE LOGICAL                   Inodes begin with a inode header.  inode may
BLOCK BLOCK       CONTENTS      be marked as "deleted," pending re-packing.
  n   4*n     --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |IIIIIIIIIIIIII| Inodes begin with a inode header
                |DDDDDDDDDDDDDD| Data block containing inode data block
                | (Inode Data) |
      4*n+1   --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |DDDDDDDDDDDDDD| Inodes may consist of multiple data blocks
                | (Inode Data) |
                |IIIIIIIIIIIIII| Next inode header
                |              | Possibly a few unused bytes at the end of a block
      4*n+2   --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |DDDDDDDDDDDDDD|
                | (Inode Data) |
      4*n+3   --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |IIIIIIIIIIIIII| Next inode header
                |DDDDDDDDDDDDDD|
                | (Inode Data) |
 n+1  4*(n+1) --+--------------+
                |BBBBBBBBBBBBBB| Logic block header
                |              | All FLASH is unused after the end of the final
                |              | inode.
              --+--------------+

General operation
=================

  Inodes are written starting at the beginning of FLASH.  As inodes are
  deleted, they are marked as deleted but not removed.  As new inodes are
  written, allocations  proceed to toward the end of the FLASH -- thus,
  supporting wear leveling by using all FLASH blocks equally.

  When the FLASH becomes full (no more space at the end of the FLASH), a
  re-packing operation must be performed:  All inodes marked deleted are
  finally removed and the remaining inodes are packed at the beginning of
  the FLASH.  Allocations then continue at the freed FLASH memory at the
  end of the FLASH.

Headers
=======
  BLOCK HEADER:
    The block header is used to determine if the block has every been
    formatted and also indicates bad blocks which should never be used.

  INODE HEADER:
    Each inode begins with an inode header that contains, among other things,
    the name of the inode, the offset to the first data block, and the
    length of the inode data.

    At present, the only kind of inode support is a file.  So for now, the
    term file and inode are interchangeable.

  INODE DATA HEADER:
    Inode data is enclosed in a data header.  For a given inode, there
    is at most one inode data block per logical block.  If the inode data
    spans more than one logical block, then the inode data may be enclosed
    in multiple data blocks, one per logical block.

NXFFS Limitations
=================

This implementation is very simple as, as a result, has several limitations
that you should be aware before opting to use NXFFS:

1. Since the files are contiguous in FLASH and since allocations always
   proceed toward the end of the FLASH, there can only be one file opened
   for writing at a time.  Multiple files may be opened for reading.

2. Files may not be increased in size after they have been closed.  The
   O_APPEND open flag is not supported.

3. Files are always written sequential.  Seeking within a file opened for
   writing will not work.

4. There are no directories, however, '/' may be used within a file name
   string providing some illusion of directories.

5. Files may be opened for reading or for writing, but not both: The O_RDWR
   open flag is not supported.

6. The re-packing process occurs only during a write when the free FLASH
   memory at the end of the FLASH is exhausted.  Thus, occasionally, file
   writing may take a long time.

7. Another limitation is that there can be only a single NXFFS volume
   mounted at any time.  This has to do with the fact that we bind to
   an MTD driver (instead of a block driver) and bypass all of the normal
   mount operations.

Multiple Writers
================

As mentioned in the limitations above, there can be only one file opened
for writing at a time.  If one thread has a file opened for writing and
another thread attempts to open a file for writing, then that second
thread will be blocked and will have to wait for the first thread to
close the file.

Such behavior may or may not be a problem for your application, depending
(1) how long the first thread keeps the file open for writing and (2) how
critical the behavior of the second thread is.  Note that writing to FLASH
can always trigger a major FLASH reorganization and, hence, there is no
way to guarantee the first condition: The first thread may have the file
open for a long time even if it only intends to write a small amount.

Also note that a deadlock condition would occur if the SAME thread
attempted to open two files for writing.  The thread would would be
blocked waiting for itself to close the first file.

ioctls
======

The file system supports to ioctls:

FIOC_REFORMAT:  Will force the flash to be erased and a fresh, empty
  NXFFS file system to be written on it.
FIOC_OPTIMIZE:  Will force immediate repacking of the file system.  This
  will increase the amount of wear on the FLASH if you use this!

Things to Do
============

- The statfs() implementation is minimal.  It whould have some calcuation
  of the f_bfree, f_bavail, f_files, f_ffree return values.
- There are too many allocs and frees.  More structures may need to be
  pre-allocated.
- The file name is always extracted and held in allocated, variable-length
  memory.  The file name is not used during reading and eliminating the
  file name in the entry structure would improve performance.
- There is a big inefficiency in reading.  On each read, the logic searches
  for the read position from the beginning of the file each time.  This
  may be necessary whenever an lseek() is done, but not in general.  Read
  performance could be improved by keeping FLASH offset and read positional
  information in the read open file structure.
- Fault tolerance must be improved.  We need to be absolutely certain that
  any FLASH errors do not cause the file system to behavior incorrectly.
- Wear leveling might be improved (?).  Files are re-packed at the front
  of FLASH as part of the clean-up operation.  However, that means the files
  that are not modified often become fixed in place at the beginning of
  FLASH.  This reduces the size of the pool moving files at the end of the
  FLASH.  As the file system becomes more filled with fixed files at the
  front of the device, the level of wear on the blocks at the end of the
  FLASH increases.
- When the time comes to reorganization the FLASH, the system may be
  inavailable for a long time.  That is a bad behavior.  What is needed,
  I think, is a garbage collection task that runs periodically so that
  when the big reorganizaiton event occurs, most of the work is already
  done.  That garbarge collection should search for valid blocks that no
  longer contain valid data.  It should pre-erase them, put them in
  a good but empty state... all ready for file system re-organization.
 


