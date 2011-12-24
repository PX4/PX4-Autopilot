README
^^^^^^

  o Installation
    - Installing Cygwin
    - Download and Unpack
    - Semi-Optional apps/ Package
    - Installation Directories with Spaces in the Path
    - A Note about Header Files
  o Configuring NuttX
  o Toolchains
    - Cross-Development Toolchains
    - NuttX Buildroot Toolchain
  o Shells
  o Building NuttX
    - Building
    - Re-building 
  o Documentation

INSTALLATION
^^^^^^^^^^^^

Installing Cygwin

  NuttX may be installed and built on a Linux system or on a Windows
  system if Cygwin is installed.  Installing Cygwin on your Windows PC
  is simple, but time consuming.  See http://www.cygwin.com/ for
  installation instructions. Basically you just need to download a
  tiny setup.exe program and it does the real, internet installation
  for you.

  Some Cygwin installation tips:
  
  1. Install at C:\cygwin

  2. Install EVERYTHING:  "Only the minimal base packages from the
     Cygwin distribution are installed by default. Clicking on categories
     and packages in the setup.exe package installation screen will
     provide you with the ability to control what is installed or updated.
     Clicking on the "Default" field next to the "All" category will
     provide you with the opportunity to install every Cygwin package.
     Be advised that this will download and install hundreds of megabytes
     to your computer."
     
     If you use the "default" installation, you will be missing many
     of the Cygwin utilities that you will need to build NuttX.  The
     build will fail in numerous places because of missing packages.

  After installing Cygwin, you will get lots of links for installed
  tools and shells.  I use the RXVT native shell.  It is fast and reliable
  and does not require you to run the Cygwin X server (which is neither
  fast nor reliable).  The rest of these instructions assume that you
  are at a bash command line prompt in either Linux or in Cygwin shell.

Download and Unpack:

  Download and unpack the NuttX tarball.  If you are reading this, then
  you have probably already done that.  After unpacking, you will end
  up with a directory called nuttx-version (where version is the NuttX
  version number). You might want to rename that directory nuttx to
  match the various instructions in the documentation and some scripts
  in the source tree.

Semi-Optional apps/ Package:

  All NuttX libraries and example code used to be in included within
  the NuttX source tree.  As of NuttX-6.0, this application code was
  moved into a separate tarball, the apps tarball.  If you are just
  beginning with NuttX, then you will want to download the versioned
  apps tarball along with the NuttX tarball.  If you already have your
  own product application directory, then you may not need the apps
  tarball.

  It is call "Semi-optional" because if you don't have some apps/
  directory, NuttX will *fail* to build!

  Download the unpack the apps tarball in the same directly where you
  unpacked the NuttX tarball.  After you unpack the apps tarball, you
  will have a new directory called apps-version (where the version
  should exactly match the version of the NuttX tarball).  Again, you
  might want to rename the directory to simply apps/ to match what
  you read in the documentation
  
  After unpacking the apps tarball, you will have two directories side
  by side like this:
  
             |
        +----+----+
        |         |
      nuttx/     apps/

  This is important because the NuttX build will expect to find the
  apps directory in that (default) location.  )That default location
  can be changed by editing your NuttX configuration file, but that
  is another story).

Installation Directories with Spaces in the Path:

  The nuttx build directory should reside in a path that contains no
  spaces in any higher level directory name.  For example, under
  Cygwin, your home directory might be formed from your first and last
  names like: "/home/First Last". That will cause strange errors when
  the make system tries to build.

  [Actually, that problem is probably not to difficult to fix.  Some
   Makefiles probably just need some paths within double quotes]

  I work around spaces in the home directory name, by creating a
  new directory that does not contain any spaces, such as /home/nuttx.
  Then I install NuttX in /home/nuttx and always build from 
  /home/nuttx/nuttx.

A Note about Header Files:

  Some toolchains are built with header files extracted from a C-library
  distribution (such as newlib).  For those toolchains, NuttX must be
  compiled without using the standard header files that are distributed
  with your toolchain.  This prevents including conflicting, incompatible
  header files (such as stdio.h).

  Certain header files, such as setjmp.h and varargs.h, may still be
  needed from your toolchain, however.  If that is the case, one solution
  is to copy those header file from your toolchain into the NuttX include
  directory.

  Also, if you prefer to use the stdint.h and stdbool.h header files from
  your toolchain, those could be copied into the include/ directory too.
  Using most other header files from your toolchain would probably cause
  errors.

CONFIGURING NUTTX
^^^^^^^^^^^^^^^^^

"Canned" NuttX configuration files are retained in:

  configs/<board-name>/<config-dir>

Where <board-name> is the name of your development board and <config-dir>.
Configuring NuttX requires only copying three files from the <config-dir>
to the directly where you installed NuttX (TOPDIR):

  Copy configs/<board-name>/<config-dir>/Make.def to ${TOPDIR}/Make.defs
    Make.defs describes the rules needed by you tool chain to compile
    and link code.  You may need to modify this file to match the
    specific needs of your toolchain.

  Copy configs/<board-name>/<config-dir>/setenv.sh to ${TOPDIR}/setenv.sh
    setenv.sh is an optional convenience file that I use to set
    the PATH variable to the toolchain binaries.  You may chose to
    use setenv.sh or not.  If you use it, then it may need to be
    modified to include the path to your toolchain binaries.

  Copy configs/<board-name>/<config-dir>/defconfig to ${TOPDIR}/.config
    The defconfig file holds the actual build configuration.  This
    file is included by all other make files to determine what is
    included in the build and what is not.  This file is also used
    to generate a C configuration header at include/nuttx/config.h.

General information about configuring NuttX can be found in:

  ${TOPDIR}/configs/README.txt
  ${TOPDIR}/configs/<board-name>/README.txt

There is a configuration script in the tools/ directory that makes this
easier.  It is used as follows:

  cd ${TOPDIR}/tools
  ./configure.sh <board-name>/<config-dir>

TOOLCHAINS
^^^^^^^^^^

Cross-Development Toolchains

  In order to build NuttX for your board, you will have to obtain a cross-
  compiler to generate code for your target CPU.  For each board,
  configuration, there is a README.txt file (at configs/<board-name>/README.txt).
  That README file contains suggestions and information about appropriate
  tools and development environments for use with your board.

  In any case, the script, setenv.sh that was deposited in the top-
  level directory when NuttX was configured should be edited to set
  the path to where you installed the toolchain.  The use of setenv.sh
  is optional but can save a lot of confusion in the future.

NuttX Buildroot Toolchain

  For many configurations, a DIY set of tools is available for NuttX.  These
  tools can be downloaded from the NuttX SourceForge file repository.  After
  unpacking the buildroot tarball, you can find instructions for building
  the tools in the buildroot/configs/README.txt file.

  Check the README.txt file in the configuration director for your board
  to see if you can use the buildroot toolchain with your board (this
  README.txt file is located in configs/<board-name>/README.txt).

  This toolchain is available for both the Linux and Cygwin development
  environments.

SHELLS
^^^^^^

  The NuttX build relies on some shell scripts.  Some are inline in the
  Makefiles and many are exectuble scripts in the tools/. directory.  The
  scripts were all developed using bash and many contain bash shell
  dependencies.

  Most of the scripts begin with #!/bin/bash to specifically select the
  bash shell.  Some still have #!/bin/sh but I haven't heard any complaints
  so these must not have bash dependencies.

  There are two shell issues that I have heard of:

  1. Linux where /bin/sh refers to an incompatible shell (like ksh or csh).

     In this case, bash is probably avaiable and the #!/bin/bash at the
     beginning of the file should do the job.  If any scripts with #!/bin/sh
     fail, try changing that ti #!/bin/bash and let me know about the change.

  2. FreeBSD with the Bourne Shell and no bash shell.

     The other, reverse case has also been reported on FreeBSD setups that
     have the Bourne shell, but not bash.  In this base, #!/bin/bash fails
     but #!/bin/sh works okay.  My recommendation in this case is to create
     a symbolic link at /bin/bash that refers to the Bourne shell.

     There may still be issues, however, with certain the bash-centric scripts
     that will require modifications.

BUILDING NUTTX
^^^^^^^^^^^^^^

Building

  NuttX builds in-place in the source tree.  You do not need to create
  any special build directories.  Assuming that your Make.defs is setup
  properly for your tool chain and that setenv.sh contains the path to where
  your cross-development tools are installed, the following steps are all that
  are required to build NuttX:

    cd ${TOPDIR}
    . ./setenv.sh
    make

  At least one configuration (eagle100) requires additional command line
  arguments on the make command.  Read ${TOPDIR}/configs/<board-name>/README.txt
  to see if that applies to your target.

Re-building 

  Re-building is normally simple -- just type make again.

  But there are some things that can "get you" when you use the Cygwin
  development environment with Windows native tools.  The native Windows
  tools do not understand Cygwin's symbolic links, so the NuttX make system
  does something weird:  It copies the configuration directories instead of
  linking to them (it could, perhaps, use the NTFS 'mklink' command, but it
  doesn't).

  A consequence of this is that you can easily get confused when you edit
  a file in one of the linked (i.e., copied) directories, re-build NuttX,
  and then not see your changes when you run the program.  That is because
  build is still using the version of the file in the copied directory, not
  your modified file! To work around this annoying behavior, do the
  following when you re-build:
   
     make clean_context all
   
  This 'make' command will remove of the copied directories, re-copy them,
  then make NuttX.

CYGWIN BUILD PROBLEMS
^^^^^^^^^^^^^^^^^^^^^

If you see strange behavior when building under Cygwin then you may have
a problem with your PATH variable.  For example, if you see failures to
locate files that are clearly present, that may mean that you are using
the wrong version of a tool.  For example, you may not be using Cygwin's
'make' program at /usr/bin/make.  Try:

    $ which make
    /usr/bin/make

When you install some toolchains (such as Yargarto or CodeSourcery tools),
they may modify your PATH variable to include a path to their binaries.
At that location, they make have GNUWin32 versions of the tools.  So you
might actually be using a version of make that does not understand Cygwin
paths.

The solution is either:

1. Edit your PATH to remove the path to the GNUWin32 tools, or
2. Put /usr/local/bin, /usr/bin, and /bin at the front of your path:

   $ export PATH=/usr/local/bin:/usr/bin:/bin:$PATH

DOCUMENTATION
^^^^^^^^^^^^^

Additional information can be found in the Documentation/ directory and
also in README files that are scattered throughout the source tree.  The
documentation is in HTML and can be access by loading the following file
into your Web browser:

  Documentation/index.html

NuttX documentation is also available online at http://www.nuttx.org.

Below is a guide to the available README files in the NuttX source tree:

nuttx
 |
 |- arch/
 |   |
 |   |- arm/
 |   |   `- src
 |   |       `- lpc214x/README.txt
 |   |- avr/
 |   |   `- README.txt
 |   |- sh/
 |   |   |- include/
 |   |   |   |-m16c/README.txt
 |   |   |   |-sh1/README.txt
 |   |   |   `-README.txt
 |   |   |- src/
 |   |   |   |-common/README.txt
 |   |   |   |-m16c/README.txt
 |   |   |   |-sh1/README.txt
 |   |   |   `-README.txt
 |   |- x86/
 |   |   |- include/
 |   |   |   `-README.txt
 |   |   `- src/
 |   |       `-README.txt
 |   `- z80/
 |   |   `- src/
 |   |       `- z80/README.txt
 |   `- README.txt
 |- configs/
 |   |- amber/
 |   |   `- README.txt
 |   |- avr32dev1/
 |   |   `- README.txt
 |   |- c5471evm/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- demo0s12ne64/
 |   |   `- README.txt
 |   |- detron/
 |   |   `- README.txt
 |   |- ea3131/
 |   |   `- README.txt
 |   |- ea3152/
 |   |   `- README.txt
 |   |- eagle100/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- ez80f910200kitg/
 |   |   |- ostest/README.txt
 |   |   `- README.txt
 |   |- ez80f910200zco/
 |   |   |- dhcpd/README.txt
 |   |   |- httpd/README.txt
 |   |   |- nettest/README.txt
 |   |   |- nsh/README.txt
 |   |   |- ostest/README.txt
 |   |   |- poll/README.txt
 |   |   `- README.txt
 |   |-  hymini-stm32v/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- kwikstik-k40/
 |   |   `- README.txt
 |   |- lm3s6965-ek/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- lm3s8962-ek/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- lpcxpresso-lpc1768/
 |   |   `- README.txt
 |   |- m68332evb/
 |   |   |- include/README.txt
 |   |   `- src/README.txt
 |   |- mbed/
 |   |   `- README.txt
 |   |- mcu123-lpc214x/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- micropendous3/
 |   |   `- README.txt
 |   |- mx1ads/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- ne63badge/
 |   |   `- README.txt
 |   |- ntosd-dm320/
 |   |   |- doc/README.txt
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- nucleus2g/
 |   |   `- README.txt
 |   |- olimex-lpc1766stk/
 |   |   `- README.txt
 |   |- olimex-lpc2378/
 |   |   |- include/README.txt
 |   |   `- README.txt
 |   |- olimex-strp711/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- pcblogic-pic32mx/
 |   |   `- README.txt
 |   |- pic32-starterkit/
 |   |   `- README.txt
 |   |- pjrc-8051/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- qemu-i486/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- rgmp/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- sam3u-ek/
 |   |   `- README.txt
 |   |- sim/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- skp16c26/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- stm3210e-eval/
 |   |   |- include/README.txt
 |   |   |- RIDE/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- stm3240g-eval/
 |   |   `- README.txt
 |   |- sure-pic32mx/
 |   |   `- README.txt
 |   |- teensy/
 |   |   `- README.txt
 |   |- twr-k60n512/
 |   |   `- README.txt
 |   |- us7032evb1/
 |   |   |- bin/README.txt
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- vsn/
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- xtrs/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- z16f2800100zcog/
 |   |   |- ostest/README.txt
 |   |   |- pashello/README.txt
 |   |   `- README.txt
 |   |- z80sim/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- z8encore000zco/
 |   |   |- ostest/README.txt
 |   |   `- README.txt
 |   |- z8f64200100kit/
 |   |   |- ostest/README.txt
 |   |   `- README.txt
 |   `- README.txt
 |- drivers/
 |   |- lcd/
 |   |   `- README.txt
 |   `- README.txt
 |- fs/
 |   |- mmap/
 |   |   `- README.txt
 |   `- nxffs/
 |       `- README.txt
 |- graphics/
 |   `- README.txt
 |- lib/
 |   `- README.txt
 |- libxx/
 |   `- README.txt
 |- syscall/
 |   `- README.txt
 `- tools/
     `- README.txt

apps
 |- examples/
 |   |- pashello/README.txt
 |   `- README.txt
 |- graphics/
 |   `- tiff/README.txt
 |- interpreters/
 |   |- ficl
 |   |  `- README.txt
 |   `- README.txt
 |- netutils/
 |   |- ftpc
 |   |  `- README.txt
 |   |- telnetd
 |   |  `- README.txt
 |   `- README.txt
 |- nshlib/
 |   `- README.txt
 |- system/
 |   |- i2c/README.txt
 |   |- free/README.txt
 |   `- install
 |      `- README.txt
 |- vsn/
 |   |- hello/README.txt
 |   |- poweroff
 |   |  `- README.txt
 |   |- ramtron
 |   |  `- README.txt
 |   |- sdcard
 |   |  `- README.txt
 |   `- sysinfo
 |      `- README.txt
 `- README.txt
