README
^^^^^^

CONTENTS
^^^^^^^^

  o AVAILABLE CONFIGURATIONS
  o GENERAL BUILD STEPS
  o FAQ
  o Cygwin GCC BUILD NOTES
  o Building GDB Under Cygwin 

AVAILABLE CONFIGURATIONS
^^^^^^^^^^^^^^^^^^^^^^^^

arm-defconfig
	Builds an ARM toolchain using gcc 3.4.6

arm7tdmi-defconfig-4.2.4
arm920t-defconfig-4.2.4
arm926t-defconfig-4.2.4
	Builds an ARM toolchain using gcc 4.2.4.  This configuration
	builds both gcc and g++.  There are three versions: one for 
	arm7tdmi (armv4t), arm920t (armv4t) and arm926t (arv5t) because
	of differences in the way that soft floating is handled in between
	the armv4t and arm5t architectures.

	NOTE: The newer versions of GCC generate new sections and can
	cause some problems for NuttX configurations developed under older
	toolchains.  In particular, arm-elf-objcopy may fail with strange
	errors.  If this occurs, try adding the following arguments to the
	arm-elf-objcopy command "-R .note -R .note.gnu.build-id -R .comment"

	This logic is several configuration Make.defs files:

	HOSTOS			=  ${shell uname -o}

	ARCHCCVERSION		= ${shell $(CC) -v 2>&1 | sed -n '/^gcc version/p' | sed -e 's/^gcc version \([0-9\.]\)/\1/g' -e 's/[-\ ].*//g' -e '1q'}
	ARCHCCMAJOR		= ${shell echo $(ARCHCCVERSION) | cut -d'.' -f1}

	ifeq ($(ARCHCCMAJOR),4)
	ifneq ($(HOSTOS),Cygwin)
	OBJCOPYARGS		= -R .note -R .note.gnu.build-id -R .comment
	endif
	endif

	This change probably applies to other architectures as well (?)

arm920t-defconfig-4.3.3
arm7tdmi-defconfig-4.3.3
	Builds an ARM toolchain using gcc 4.3.3.  These configurations
	builds both gcc and g++ for the arm7tdmi (armv4t) or the arm920t
	(armv4t).  These are udates to *-defconfig-4.2.4 (see notes above).

avr-defconfig-4.3.3
avr-defconfig-5.4.2
	Builds an AVR toolchain using gcc 4.3.3 or 4.5.2.  This configuration
	builds both gcc and g++ for the AVR (armv4t). This toolchain
	is intended to support the NuttX ATmega128 port.

cortexm3-defconfig-4.3.3
	Builds an OABI ARM toolchain for the Cortex-M3 using gcc 4.3.3.
	This configuration builds gcc, g++ and the NXFLAT toolchain.

cortexm3-defconfig-nxflat
arm926t-defconfig-nxflat
	This configuration build an NXFLAT toolchain (only) for
	use with the Cortex-M3 or ARM9 (untested on ARM9 as of this
	writing).

cortexm3-eabi-defconfig-4.5.2
	Builds an EABI ARM toolchain for the Cortex-M3 using gcc 4.5.2.
	This configuration builds gcc, g++ and the NXFLAT toolchain.

bfin-defconfig-4.2.4
	Builds an Blackfin toolchain using gcc 4.2.4

h8300_config
	Builds an H8/300 toolchain using gcc 3.4.6

i486-defconfig-4.3.3
	Builds an i486 cross development toolchain using gcc 4.3.3.  Why would
	you want such a thing?  On Linux, of course, such a thing is not needed
	because you can use the installed GCC to build i486 ELF binaries.  But
	that will not work under Cygwin!  The Cygwin toolchain (and probably
	MinGW), build DOS MZ format executables (i.e., .exe files).  That is
	probably not usable for most NuttX targets.  Instead, you should use this
	i486-elf-gcc to generate true ELF binaries under Cygwin.

m32c_defconfig_4.2.4
m32c_defconfig_4.3.3
	Build a toolchain for use with the M16C port using eith gcc 4.2.4 or 4.3.3

m68hc11-config
m68hc12-config-3.4.6
	Builds an hc11/hc12 toolchain using gcc 3.4.6 .  NOT RECOMMENDED for hcs12;
    Use m9s12x_config_3.3.6

m68hc12-config-4.3.3
	Builds an hc11/hc12 toolchain using gcc 4.3.3.NOT RECOMMENDED for hcs12;
    Use m9s12x_config_3.3.6
 
	This configuration fails to build with the following error:

	make[3]: Entering directory `blabla/buildroot/toolchain_build_m68hc12/gcc-4.3.3-build/m68hc12-elf/libgcc'
	...
	blabla/buildroot/toolchain_build_m68hc12/gcc-4.3.3/libgcc/../gcc/libgcc2.c:566: internal compiler error: in init_move_cost, at regclass.c:323
	Please submit a full bug report,
	with preprocessed source if appropriate.
	See <http://gcc.gnu.org/bugs.html> for instructions.
	make[3]: *** [_muldi3.o] Error 1
	make[3]: Leaving directory `blabla/buildroot/toolchain_build_m68hc12/gcc-4.3.3-build/m68hc12-elf/libgcc'

	Use m68hc12-config-3.4.6

m9s12x_config_3.3.6
	Builds a hcs12 toolchain using gcc 3.3.6 and extensive m9x12x-specific patches.

m68k-config
	Builds an M68K toolchain using gcc 3.4.6

sh-defconfig
	Builds an SH-1/2 toolchain using gcc 3.4.6

GENERAL BUILD STEPS
^^^^^^^^^^^^^^^^^^^

1. Configure your host machine.  You host PC should have a relatively complete
   C development environment.  I don't have a full list of the package requirements.
   The later tool chains also require GMP and MPRF development packages or the
   build will fail with errors like:

    "configure: error: Building GCC requires GMP 4.1+ and MPFR 2.3.0+. ...
     Copies of these libraries' source code can be found at their respective
     hosting sites as well as at ftp://gcc.gnu.org/pub/gcc/infrastructure/.
     See also http://gcc.gnu.org/install/prerequisites.html for additional info.
     If you obtained GMP and/or MPFR from a vendor distribution package, make
     sure that you have installed both the libraries and the header files.
     They may be located in separate packages."

   Version 4.5.x and beyond also require the MPC package.
 
   You should try your package manager for whatever Linux version you are using
   first.  The header files are normally included in versions of the packages that
   have "-devel" in the package name.

2. CD to the correct directory.

   Change to the directory just above the NuttX installation.  If <nuttx-dir> is
   where NuttX is installed, then cd to <nuttx-dir>/..

3. Get and Install the buildroot Module

   a. Using a release tarball:

     cd <nuttx-dir>/..
     Download the appropriate buildroot package.
     unpack the buildroot package
     rename the directory to buildroot

   b. Using SVN

     Check out the misc/buildroot module. SVN checkout instructions:

        svn co https://nuttx.svn.sourceforge.net/svnroot/nuttx nuttx/trunk/misc/buildroot

     Move the buildroot Source Tree and create the archive directory

        mv misc/buildroot .

   Make the archive directory:

     mkdir archive

   The <nuttx-dir>/../buildroot is where the toolchain is built;
   The <nuttx-dir>/../archive directory is where toolchain sources will be downloaded.

4. Make sure that NuttX is configured

     cd <nuttx-dir>/tools
     ./configure.sh <nuttx-configuration>

5. Configure and Make the buildroot

     cd buildroot
     cp configs/<config-file> .config
     make oldconfig
     make

   This will download the large source packages for the toolchain and build the toolchain.
   The resulting binaries will be under buildroot/build_<arch>.  There will also be a
   large build directory called something like toolchain_build_<arch>; this directory
   can be removed once the build completes successfully.

   Where <config-file> is one of the configuration files listed above and <arch> is an
   archtecture name.  Examples: build_m32c, build_arm_nofpu, etc.

FAQ
^^^

Q: What is up with errors like this:

   fatal error: ansidecl.h: No such file or directory

A: This was reported on Fedora F14.  The cause of the problem is that some host
   binutils header files were moved from the binutils RPM to the binutils-dev
   RPM.  The fix is to install the binutils-dev RPM.

Q: How do I build the NuttX toolchain under Cygwin?

A: See below...

Q: NuttX directory ../../nuttx does not exist

A: The default path to the nuttx directory is $(TOPDIR)/../../nuttx where
   TOPDIR holds the path to the buildroot directory.  If you checkout the
   entire SVN tree then that will be the correct location of the nuttx
   directory.

   If you see this error, it just means that nuttx is not in that expected,
   default location.  In that case, use 'make config' or 'make menuconfig'
   to edit the configuration.  Find the option to set the path to NuttX
   and set it to the correct location for your build environment.

Q: Some of my libraries like GMP and MPFR are in non-standard locations the
   GCC build can't file them:

     checking for correct version of mpfr.h... no
     configure: error: Building GCC requires GMP 4.1+ and MPFR 2.3.0+.

A: http://tech.groups.yahoo.com/group/nuttx/message/1160

   "I think that you can specify the path to GMP and MPFR. I think that GCC
    has some special configuration command line options to support this. I
    can't remember exactly and I don't have an unpacked version of GCC at
    hand.

   "Try this: Go to the buildroot/toolchain_build_nofpu_arm/gcc-x.x directory
    and type:

      ./configure --help

   "That should list all of the GCC configuration options. I bet you will see
    (near the bottom) some options to set the path to these tools.

   "What you will have to do then is to modify the script at:

       buildroot/toolchain/gcc/gcc-nuttx-4.x.mk

   "You will see that there are several places where $(GCC_DIR)/configure is
    invoked. I think you would have to hard code those path options into those
    configure commands."

Cygwin GCC BUILD NOTES
^^^^^^^^^^^^^^^^^^^^^^

 o Cygwin normally creates a /home directory with your Windows user name.  Unfortunately,
   that could very likely include spaces.  In that case, the Cygwin build will have
   lots of problems.  Here is how I worked around that:

   - I created a /home/buildroot directory and copied buildroot to that location
     (/home/build/buildroot/buildroot)
   - I have the archives directory at /home/buildroot/archives
   - And a symbolic link to the nuttx build directory at /home/buildroot/nuttx

   With those workarounds, the buildroot will build.  However, you will also need
   to either edit the setenv.sh file to reference this new location, or else move
   resulting build diectory.

 o On Cygwin, the buildroot 'make' command will fail with an error like:

   "...
      build/genchecksum cc1-dummy > cc1-checksum.c
      opening cc1-dummy: No such file or directory
   ..."

   This is caused because on Cygwin, host executables will be generated with the extension .exe
   and, apparently, the make variable "exeext" is set incorrectly.  A work around after the
   above occurs is:

      cd toolchain_build_<arch>/gcc-4.2.4-build/gcc	# Go to the directory where error occurred
      mv cc1-dummy.exe cc1-dummy			# Rename the executable without .exe
      rm cc1-checksum.c					# Get rid of the bad generated file

   Then resume the buildroot make:

      cd -						# Back to the buildroot make directory
      make						# Restart the build

   If you build g++, you will see another similar error:

   ...
      build/genchecksum cc1plus-dummy > cc1plus-checksum.c
      opening cc1plus-dummy: No such file or directory
   ...

   The fix is similar:

      cd toolchain_build_<arch>/gcc-4.2.4-build/gcc	# Go to the directory where error occurred
      mv cc1plus-dummy.exe cc1plus-dummy		# Rename the executable without .exe
      rm cc1plus-checksum.c				# Get rid of the bad generated file

   Then resume the buildroot make:

      cd -						# Back to the buildroot make directory
      make						# Restart the build

 o Once I had problems building the toolchain on Cygwin.  In this case, I
   would occasioinally get "Permission denied" errors will trying to configure
   the toolchain.  My hunch is that this error was caused because of failures
   to remove some temporary files (like conftest.c).  Perhaps there errors
   occurred because some other application opened those files too???  Perhaps
   a virus scanner.

   Sometimes when this occurs, the build continues to execute.  If that is
   the case, it could end-up making a bad toolchain??? In this case, you need
   to hit Control-C to stop the build.  Normally, however, the "Permission
   denied" error causes the configure script to stop.  In either case, if you
   just restart the make, the build will continue past the failure point.

   This has happened to me only while doing other intensive activities in
   windows during the toolchain build.  I suspect if you leave your PC
   mostly idle while the toolchain builds, this will not likely be a problem
   for you.

Building GDB Under Cygwin
^^^^^^^^^^^^^^^^^^^^^^^^^

   This can be tricking, but it has been done.  See this message sequence for
   http://tech.groups.yahoo.com/group/nuttx/message/726 .  Apparently there
   are some incompatibilities with Cygwin 1.7 that require an additional
   patch.  See http://old.nabble.com/-RFA--windows-nat.c%3A-Cygwin%3A-Port-to-Cygwin-1.7-td27735619.html

   A version of this patch for GDB-6.8 and Cywgin 1.7 (and above) is provided
   in this directory (Thanks to Dimiter Georgiev).  That file is called
   gdb-1_8-cygwin-1_7.patch.  This should be copied into the GDB 6.8 toolchain
   directory if it is needed:

   cp configs/gdb-1_8-cygwin-1_7.patch toolchain/gdb/6.8/.
 
