README.txt
^^^^^^^^^^

This is the README file for the NuttX port to the ZiLog ZNEO MCU.

ZDS-II Compiler Versions
^^^^^^^^^^^^^^^^^^^^^^^^

Version 4.10.2

  The ZDS-II version 4.10.2 will not compile NuttX.  It reports "internal
  errors" on some of the files.  Upgrades to ZDS-II are available for
  download from the Zilog website: http://www.zilog.com/software/zds2.asp

Version 4.11.0

  NuttX compiles correctly with the newer 4.11.0 version of the ZDS-II
  toolchain.  However, I have found a few issues:

  - The code will not run without the -reduceopt option.  There is,
    apparently, some optimization related issue.  This issue has not
    been analyzed as of this writing.

  - Not all NuttX logic will not run with the -regvars option.  There is
    at least one failure that has been reported to ZiLOG as incident 81400.

  - The Pascal add-on interpreter includes a large switch statement and
    exposes another compiler problem.  This is reported as incident 81459.

Version 4.11.1

  As of this writing (30 September 2010), the latest release of ZDS-II for the
  ZNEO is 4.11.1.  It is unknown if this release includes fixes for incidents
  81400 and 81459 or not.  It is unknown if the code will run without -reduceopt
  either. (Basically, it compiles with 4.11.1, but is untested with that version).

Version 5.0.1

  On November 29, 2012, all of the z16f configurations were converted to use 5.0.1,
  but have not been verified on a running target.

  Paths were also updated that are specific to a 32-bit toolchain running on
  a 64 bit windows platform.  Change to a different toolchain, you will need
  to modify the versioning in Make.defs and setenv.sh; if you want to build
  on a different platform, you will need to change the path in the ZDS binaries
  in those same files.
  
Other Versions

  If you use any version of ZDS-II other than 5.0.1 or if you install ZDS-II
  at any location other than the default location, you will have to modify
  two files:  (1) configs/z16f2800100zcog/*/setenv.sh and (2)
  configs/z16f2800100zcog/*/Make.defs.  Simply edit these two files, changing
  5.0.1 to whatever.

Issues
^^^^^^

There are several, important open issues with the ZNEO port (8 as of this writing).
See the TODO file in the top-level NuttX directory.  One of these should be
mentioned here because it causes a failure to compile Nuttx:

  Description: The file drivers/mmcsd/mmcsd_sdio.c generates an internal compiler
               error like:
 
               mmcsd\mmcsd_sdio.c
               Internal Error(0503) On line 2524 of "MMCSD\MMCSD_SDIO.C"
                   File <c3>, Args(562,46)

  Status:     Open.  Recommended workaround: remove mmcsd_sdio.c from
              drivers/mmcsd/Make.defs.  There is no SDIO support for the Z16 anyway
  Priority:   Low

This is bug in ZDS-II.  It was discovered in version 4.11.0 and still exists
in version 4.11.1.

Configuration Subdirectories
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- source/ and include/
    These directories contain common logic for all z16f2800100zcog
    configurations.

Variations on the basic z8f162800100zcog configuration are maintained
in subdirectories.  To configure any specific configuration, do the
following steps:

   cd <nuttx-top-directory>/tools
   ./configure.sh z16f2800100zcog/<sub-directory>
   cd <nuttx-top-directory>
   make

Where <sub-directory> is the specific board configuration that you
wish to build.  The following board-specific configurations are
available:

- ostest
    This builds the examples/ostest application for execution from FLASH.
    See examples/README.txt for information about ostest.

- pashello

    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

Check out any README.txt files in these <sub-directory>s.
