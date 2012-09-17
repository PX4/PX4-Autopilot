README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

4.10.1
  The ZDS-II version 4.10.2 will not compile NuttX.  It reports "internal
  errors" on one of the files, mm/mm_initialize.c.  Below is a simple work-
  around.  With this work-around in place, NuttX builds successfully with
  the 4.10.1 compiler.

    --- mm/mm_initialize.c.SAVE	2008-02-13 08:06:46.833857700 -0600
    +++ mm/mm_initialize.c	2008-02-13 08:07:26.367608900 -0600
    @@ -94,8 +94,11 @@
    {
       int i;
 
    +#if 0 /* DO NOT CHECK IN */
       CHECK_ALLOCNODE_SIZE;
       CHECK_FREENODE_SIZE;
    +#endif
 
   /* Set up global variables */

4.9.5
  This is the latest tool version listed on the ZiLOG site for the Z8F6403.
  However, it uses different compiler command line arguments.

Other Versions
  If you use any version of ZDS-II other than 4.10.1 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  two files:  (1) configs/z8encore000zco/*/setenv.sh and (2)
  configs/z8encore000zco/*/Make.defs.

Configuration Subdirectories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- src/ and include/
    These directories contain common logic for all z8encore000zco
    configurations.

Variations on the basic z8encore000zco configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh z8encore000zco/<sub-directory>
   cd <nuttx-top-directgory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available:

- ostest
    This builds the examples/ostest application for execution from FLASH.
    See examples/README.txt for information about ostest.

Check out any README.txt files in these <sub-directory>s.
