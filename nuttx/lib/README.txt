lib
===

This directory contains numerous, small functions typically associated with
what you would expect to find in a standard C library.  The sub-directories
in this directory contain standard interface that can be executed by user-
mode programs.

Normally, NuttX is built with no protection and all threads running in kerne-
mode.  In that model, there is no real architectural distinction between
what is a kernel-mode program and what is a user-mode program; the system is
more like on multi-threaded program that all runs in kernel-mode.

But if the CONFIG_NUTTX_KERNEL option is selected, NuttX will be built into
distinct user-mode and kernel-mode sections.  In that case, most of the
code in the nuttx/ directory will run in kernel-mode with with exceptions
of (1) the user-mode "proxies" found in syscall/proxies, and (2) the
standard C library functions found in this directory.  In this build model,
it is critical to separate the user-mode OS interfaces in this way.

Sub-Directories
===============

The files in the lib/ directory are organized (mostly) according which file
in the include/ directory provides the prototype for library functions.  So
we have:

  libgen    - libgen.h
  math      - math.h and fixedmath.h
  mqueue    - pthread.h
  net       - Various network-related header files: netinet/ether.h, arpa/inet.h
  pthread   - pthread.h
  queue     - queue.h
  sched     - sched.h
  semaphore - semaphore.h
  stdio     - stdio.h
  stdlib    - stdlib.h
  string    - string.h
  time      - time.h
  unistd    - unistd.h

There is also a misc/ subdirectory that contains various internal functions
and interfaces from header files that are too few to warrant their own sub-
directory:

 misc       - Nonstandard "glue" logic, debug.h, crc32.h, dirent.h

