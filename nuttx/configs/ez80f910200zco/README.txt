README.txt
^^^^^^^^^^

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

Different configurations have been build for this board using ZDS-11
Versions 4.11.0 and 4.11.1.  You have to check the files */Make.defs
to see how the build is configured:  Check the definitions of
ZDSVERSION (if present) and ZDSINSTALLDIR.

NOTE:  Different versions of the ZDS-II compiler may also require
different versions of .linkcmd and .zdsproj files as well.

Version 4.11.0

  Although it compiles without error, the 4.11.0 compiler generates
  bad code on one of the files, mm/mm_initialize.c.  Below is a simple work-
  around.

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

   UPDATE:  I don't know if 4.11.1 has this same problem (I bet not since
   I submitted the bug to ZiLOG), but I have permanently worked around the
   above problem for all ZiLOG compiler.

Version 5.1.1

  On June 22, 2011 I verified that these configurations build successfully
  with the 5.1.1 ZDS-II version.  All that is required to used ZDS-II is
  to modify the Make.defs file so that the correct version is used.  That
  version should also be changed in the (optional) setenv.sh file.

  The above kludge for 4.11.0 is not required.

  I had to make additional changes to the ZDS path in Make.defs (and also
  in setenv.sh) when the 32-bit ZDS-II tools are installed on my 64-bit
  Windows 7 system.
  
Other Versions
  If you use any version of ZDS-II other than 4.11.0 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  two files:  (1) configs/ez80f910200zco/*/setenv.sh and (2)
  configs/ez80f910200zco/*/Make.defs.

Configuration Subdirectories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

src/ and include/
    These directories contain common logic for all ez80f910200zco
    configurations.

Variations on the basic ez80f910200zco configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh ez80f910200zco/<sub-directory>
   cd <nuttx-top-directgory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available:

dhcpd:
    This builds the DCHP server using the examples/dhcpd application
    (for execution from FLASH.) See examples/README.txt for information
    about the dhcpd example.

httpd:
    This builds the uIP web server example using the examples/uip application
    (for execution from FLASH). See examples/README.txt for information
    about ostest.

ostest
    This builds the examples/ostest application for execution from FLASH.
    See examples/README.txt for information about ostest.

nsh
    This configuratino builds the NuttShell (NSH).  That code can be
    found in examples/nsh.  For more information see:  examples/nsh/README.txt
    and Documentation/NuttShell.html.

nettest
    This configuration is used for testing the eZ80F91 EMAC driver.  It
    builds examples/nettest.  See examples/README.txt for more information
    about nettest.

poll
    This configuration is also used for testing the eZ80F91 EMAC driver.  It
    builds examples/poll.  See examples/README.txt for more information
    about the poll test.

Check out any README.txt files in these <sub-directory>s.
