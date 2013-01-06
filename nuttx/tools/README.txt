tools/README.txt
================

This README file addresses the contents of the NuttX tools/ directory.

The tools/ directory contains miscellaneous scripts and host C programs
that are necessary parts of the the NuttX build system.  These files
include:

README.txt
----------

  This file!

Config.mk
---------

  This file contains common definitions used by many configureation files.
  This file (along with <nuttx>/.config) must be included at the top of
  each configuration-specific Make.defs file like:

    -include $(TOPDIR)/.config
    include $(TOPDIR)/tools/Config.mk

  Subsequent logic within the configuration-specific Make.defs file may then
  override these default definitions as necessary.

configure.sh
------------

  This is a bash script that is used to configure NuttX for a given
  target board.  See configs/README.txt or Documentation/NuttxPortingGuide.html
  for a description of how to configure NuttX with this script.

discover.py
-----------

  Example script for discovering devices in the local network.
  It is the counter part to apps/netutils/discover

mkconfig.c, cfgparser.c, and cfgparser.h
----------------------------------------

  These are Cs file that are used to build mkconfig program.  The mkconfig
  program is used during the initial NuttX build.

  When you configure NuttX, you will copy a configuration file called .config
  in the top level NuttX directory (See configs/README.txt or
  Documentation/NuttxPortingGuide.html).  The first time you make NuttX,
  the top-level makefile will build the mkconfig executable from mkconfig.c
  (using Makefile.host).  The top-level Makefile will then execute the
  mkconfig program to convert the .config file in the top level directory
  into include/nuttx/config.h.  config.h is a another version of the
  NuttX configuration that can be included by C files.

cmdconfig.c
-----------

  This C file can be used to build a utility for comparing two NuttX
  configuration files.

mkexport.sh and Makefile.export
-------------------------------

  These implement part of the top-level Makefile's 'export' target.  That
  target will bundle up all of the NuttX libraries, header files, and the
  startup object into an export-able, binary NuttX distribution.  The
  Makefile.export is used only by the mkexport.sh script to parse out
  options from the top-level Make.defs file.

mkfsdata.pl
-----------

  This perl script is used to build the "fake" file system and CGI support
  as needed for the apps/netutils/webserver.  It is currently used only
  by the Makefile at apps/examples/uip.  That example serves as an example
  of how to configure the uIP webserver "fake" file system.

  NOTE:  This perl script comes from uIP and was (probably) written
  by Adam Dunkels.  uIP has a license that is compatible with NuttX.

mkversion.c, cfgparser.c, and cfgparser.h
-----------------------------------------

  This is C file that is used to build mkversion program.  The mkversion
  program is used during the initial NuttX build.

  When you build NuttX there should be a version file called .version in
  the top level NuttX directory (See Documentation/NuttxPortingGuide.html).
  The first time you make NuttX, the top-level makefile will build th
  mkversion executable from mkversion.c (using Makefile.host).  The top-
  level Makefile will then execute the mkversion program to convert the
  .version file in the top level directory into include/nuttx/version.h.
  version.h provides version information that can be included by C files.

mksyscall.c, cvsparser.c, and cvsparser.h
-----------------------------------------

  This is a C file that is used to build mksyscall program.  The mksyscall
  program is used during the initial NuttX build by the logic in the top-
  level syscall/ directory.

  If you build NuttX as a separately compiled, monolithic kernel and separate
  applications, then there is a syscall layer that is used to get from the
  user application space to the NuttX kernel space.  In the user application
  "proxies" for each of the kernel functions are provided.  The proxies have
  the same function signature as the kernel function, but only execute a
  system call.

  Within the kernel, there are "stubs" for each of the system calls.  The
  stubs receive the marshalled system call data, and perform the actually
  kernel function call (in kernel-mode) on behalf of the proxy function.

  Information about the stubs and proxies is maintained in a comma separated
  value (CSV) file in the syscall/ directory.  The mksyscall program will
  accept this CVS file as input and generate all of the required proxy or
  stub files as output.  See syscall/README.txt for additonal information.

mksymtab.c, cvsparser.c, and cvsparser.h
----------------------------------------

  This is a C file that is used to build symbol tables from common-separated
  value (CSV) files.  This tool is not used during the NuttX build, but
  can be used as needed to generate files.

  USAGE: ./mksymtab <cvs-file> <symtab-file>

  Where:

    <cvs-file>   : The path to the input CSV file
    <symtab-file>: The path to the output symbol table file
    -d           : Enable debug output

  Example:

    cd nuttx/tools
    cat ../syscall/syscall.csv ../lib/lib.csv | sort >tmp.csv
    ./mksymtab.exe tmp.csv tmp.c

pic32mx
-------

  This directory contains build tools used only for PIC32MX platforms

bdf-convert.c
-------------

  This C file is used to build the bdf-converter program.  The bdf-converter
  program be used to convert fonts in Bitmap Distribution Format (BDF)
  into fonts that can be used in the NX graphics system.

  Below are general instructions for creating and installing a new font
  in the NX graphic system:

    1. Locate a font in BDF format,
    2. Use the bdf-converter program to convert the BDF font to the NuttX
       font format.  This will result in a C header file containing
       defintions.  That header file should be installed at, for example,
       graphics/nxfonts/nxfonts_myfont.h.

  Create a new NuttX configuration variable.  For example, suppose
  you define the following variable:  CONFIG_NXFONT_MYFONT.  Then
  you would need to:
  
    3. Define CONFIG_NXFONT_MYFONT=y in your NuttX configuration file.

  A font ID number has to be assigned for each new font.  The font ID
  is defined in the file include/nuttx/nx/nxfonts.h.  Those definitions
  have to be extended to support your new font.  Look at how the font ID
  enabled by CONFIG_NXFONT_SANS23X27 is defined and add an ID for your
  new font in a similar fashion:

    4. include/nuttx/nx/nxfonts.h. Add you new font as a possible system
       default font:
 
       #if defined(CONFIG_NXFONT_SANS23X27)
       # define NXFONT_DEFAULT FONTID_SANS23X27
       #elif defined(CONFIG_NXFONT_MYFONT)
       # define NXFONT_DEFAULT FONTID_MYFONT
       #endif

       Then define the actual font ID.  Make sure that the font ID value
       is unique:
 
       enum nx_fontid_e
       {
         FONTID_DEFAULT     = 0      /* The default font */
       #ifdef CONFIG_NXFONT_SANS23X27
         , FONTID_SANS23X27 = 1      /* The 23x27 sans serif font */
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
         , FONTID_MYFONT    = 2      /* My shiny, new font */
       #endif
       ...
 
  New Add the font to the NX build system.  There are several files that
  you have to modify to to this.  Look how the build system uses the
  font CONFIG_NXFONT_SANS23X27 for examaples:

    5. nuttx/graphics/Makefile.  This file needs logic to auto-generate
       a C source file from the header file that you generated with the
       the bdf-converter program.  Notice NXFONTS_FONTID=2; this must be
       set to the same font ID value that you defined in the
       include/nuttx/nx/nxfonts.h file.

       genfontsources:
         ifeq ($(CONFIG_NXFONT_SANS23X27),y)
          @$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=1 EXTRADEFINES=$(EXTRADEFINES)
        endif
         ifeq ($(CONFIG_NXFONT_MYFONT),y)
          @$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_FONTID=2 EXTRADEFINES=$(EXTRADEFINES)
        endif

    6. nuttx/graphics/nxfonts/Make.defs.  Set the make variable NXFSET_CSRCS.
       NXFSET_CSRCS determines the name of the font C file to build when
       NXFONTS_FONTID=2:

       ifeq ($(CONFIG_NXFONT_SANS23X27),y)
       NXFSET_CSRCS    += nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(CONFIG_NXFONT_MYFONT),y)
       NXFSET_CSRCS    += nxfonts_bitmaps_myfont.c
       endif

    7. nuttx/graphics/nxfonts/Makefile.sources.  This is the Makefile used
       in step 5 that will actually generate the font C file.  So, given
       your NXFONTS_FONTID=2, it needs to determine a prefix to use for
       auto-generated variable and function names and (again) the name of
       the autogenerated file to create (this must be the same name that
       was used in nuttx/graphics/nxfonts/Make.defs):

       ifeq ($(NXFONTS_FONTID),1)
       NXFONTS_PREFIX    := g_sans23x27_
       GEN_CSRC    = nxfonts_bitmaps_sans23x27.c
       endif
       ifeq ($(NXFONTS_FONTID),2)
       NXFONTS_PREFIX    := g_myfont_
       GEN_CSRC    = nxfonts_bitmaps_myfont.c
       endif

    8. graphics/nxfonts/nxfonts_bitmaps.c.  This is the file that contains
       the generic font structures.  It is used as a "template" file by
       nuttx/graphics/nxfonts/Makefile.sources to create your customized
       font data set.

       #if NXFONTS_FONTID == 1
       #  include "nxfonts_sans23x27.h"
       #elif NXFONTS_FONTID == 2
       #  include "nxfonts_myfont.h"
       #else
       #  error "No font ID specified"
       #endif

       Where nxfonts_myfont.h is the NuttX font file that we generated in
       step 2 using the bdf-converter tool.

    9. graphics/nxfonts/nxfonts_getfont.c.  Finally, we need to extend the
       logic that does the run-time font lookups so that can find our new
       font.  The lookup function is NXHANDLE nxf_getfonthandle(enum nx_fontid_e fontid).
       The new font information needs to be added to data structures used by
       that function:
 
       #ifdef CONFIG_NXFONT_SANS23X27
       extern const struct nx_fontpackage_s g_sans23x27_package;
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       extern const struct nx_fontpackage_s g_myfont_package;
       #endif

       static FAR const struct nx_fontpackage_s *g_fontpackages[] =
       {
       #ifdef CONFIG_NXFONT_SANS23X27
       &g_sans23x27_package,
       #endif
       #ifdef CONFIG_NXFONT_MYFONT
       &g_myfont_package,
       #endif
       NULL
       };

Makefile.host
-------------

  This is the makefile that is used to make the mkconfig program from
  the mkconfig.c C file, the cmpconfig program from cmpconfig.c C file
  the mkversion program from the mkconfig.c C file, or the mksyscall
  program from the mksyscall.c file.  Usage:

  cd tools/
  make -f Makefile.host <program>

mkromfsimg.sh
-------------

  This script may be used to automate the generate of a ROMFS file system
  image.  It accepts an rcS script "template" and generates and image that
  may be mounted under /etc in the NuttX pseudo file system.

mkdeps.sh
mkdeps.bat
mkdeps.c
mknulldeps.sh
-------------

  NuttX uses the GCC compilers capabilities to create Makefile dependencies.
  The bash script mkdeps.sh is used to run GCC in order to create the
  dependencies.  If a NuttX configuration uses the GCC toolchain, its Make.defs
  file (see configs/README.txt) will include a line like:

    MKDEP = $(TOPDIR)/tools/mkdeps.sh, or
    MKDEP = $(TOPDIR)/tools/mkdeps[.exe] (See NOTE below)

  If the NuttX configuration does not use a GCC compatible toolchain, then
  it cannot use the dependencies and instead it uses mknulldeps.sh:

    MKDEP = $(TOPDIR)/tools/mknulldeps.sh

  The mknulldeps.sh is a stub script that does essentially nothing.

  NOTE:  The mk*deps.* files are undergoing change.  mkdeps.sh is a bash
  script that produces dependencies well for POSIX style hosts (e..g.,
  Linux and Cygwin).  It does not work well for mixed environments with
  a Windows toolchain running in a POSIX style environemnt (hence, the
  mknulldeps.sh script).  And, of course, cannot be used in a Windows
  nativ environment.

  [mkdeps.sh does have an option, --winpath, that purports to convert
  the dependencies generated by a Windows toolchain to POSIX format.
  However, that is not being used and mostly likely does not cover
  all of the conversion cases.]

  mkdeps.bat is a simple port of the bash script to run in a Windows
  command shell.  However, it does not work well either because some
  of the common CFLAGS use characters like '=' which are transformed
  by the CMD.exe shell.

  mkdeps.c generates mkdeps (on Linux) or mkdeps.exe (on Windows).
  However, this verison is still under-development.  It works well in
  the all POSIX environment or in the all Windows environment but also
  does not work well in mixed POSIX environment with a Windows toolchain.
  In that case, there are still issues with the conversion of things like
  'c:\Program Files' to 'c:program files' by bash.  Those issues may,
  eventually be solvable but for now continue to use mknulldeps.sh in
  that mixed environment.

define.sh
define.bat
---------

  Different compilers have different conventions for specifying pre-
  processor definitions on the compiler command line.  This bash
  script allows the build system to create create command line definitions
  without concern for the particular compiler in use.

  The define.bat script is a counterpart for use in the native Windows
  build.

incdir.sh
incdir.bat
---------

  Different compilers have different conventions for specifying lists
  of include file paths on the the compiler command line.  This incdir.sh
  bash script allows the build system to create include file paths without
  concern for the particular compiler in use.

  The incdir.bat script is a counterpart for use in the native Windows
  build.  However, there is currently only one compiler supported in
  that context:  MinGW-GCC.

link.sh
link.bat
copydir.sh
copydir.bat
unlink.sh
unlink.bat
----------

  Different file system have different capabilities for symbolic links.
  Some windows file systems have no native support for symbolic links.
  Cygwin running under windows has special links built in that work with
  all cygwin tools.  However, they do not work when Windows native tools
  are used with cygwin.  In that case something different must be done.

  If you are building under Linux or under cygwin with a cygwin tool
  chain, then your Make.defs file may have definitions like the
  following:

    DIRLINK = $(TOPDIR)/tools/link.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  The first definition is not always present because link.sh is the
  default.  link.sh is a bash script that performs a normal, Linux-style
  symbolic link;  unlink.sh is a do-it-all unlinking script.

  But if you are building under cygwin using a Windows native toolchain
  within a POSIX framework (such as Cygwin), then you will need something
  like the following in you Make.defs file:

    DIRLINK = $(TOPDIR)/tools/copydir.sh
    DIRUNLINK = (TOPDIR)/tools/unlink.sh

  copydir.sh will copy the whole directory instead of linking it.

  Finally, if you are running in a pure native Windows environment with
  a CMD.exe shell, then you will need something like this:

    DIRLINK = $(TOPDIR)/tools/copydir.bat
    DIRUNLINK = (TOPDIR)/tools/unlink.bat

  Note that this will copy directories.  ;ink.bat might also be used in
  this case.  link.bat will attempt to create a symbolic link using the
  NTFS mklink.exe command instead of copying files.  That logic, however,
  has not been verified as of this writing.

mkimage.sh
----------

  The creates a downloadable image as needed with the rrload bootloader.

indent.sh
---------

  This script can be used to indent .c and .h files in a manner similar
  to my coding NuttX coding style.  It doesn't do a really good job,
  however (see the comments at the top of the indent.sh file).

zipme.sh
--------

  I use this script to create the nuttx-xx.yy.tar.gz tarballs for
  release on SourceForge.  It is handy because it also does the
  kind of clean that you need to do to make a clean code release.
