README
^^^^^^

  o Installation
    - Installing Cygwin
    - Download and Unpack
    - Semi-Optional apps/ Package
    - Installation Directories with Spaces in the Path
    - Notes about Header Files
  o Configuring NuttX
    - Instantiating "Canned" Configurations
    - NuttX Configuration Tool
    - Incompatibilities with Older Configurations
    - Converting Older Configurations to use the Configuration Tool
    - NuttX Configuration Tool under DOS
  o Toolchains
    - Cross-Development Toolchains
    - NuttX Buildroot Toolchain
  o Shells
  o Building NuttX
    - Building
    - Re-building 
    - Build Targets and Options
    - Native Windows Build
    - Installing GNUWin32
  o Cygwin Build Problems
    - Strange Path Problems
    - Window Native Toolchain Issues
  o Documentation

INSTALLATION
^^^^^^^^^^^^

Installing Cygwin
-----------------

  NuttX may be installed and built on a Linux system or on a Windows
  system if Cygwin is installed.  Installing Cygwin on your Windows PC
  is simple, but time consuming.  See http://www.cygwin.com/ for
  installation instructions. Basically you just need to download a
  tiny setup.exe program and it does the real, internet installation
  for you.

     NOTE: NuttX can also be installed and built on a native Windows
     system, but with some loss of tool functionality (see the
     discussion "Native Windows Build" below).

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
  fast nor reliable).  Unless otherwise noted, the rest of these
  instructions assume that you are at a bash command line prompt in
  either Linux or in Cygwin shell.

Download and Unpack
-------------------

  Download and unpack the NuttX tarball.  If you are reading this, then
  you have probably already done that.  After unpacking, you will end
  up with a directory called nuttx-version (where version is the NuttX
  version number). You might want to rename that directory nuttx to
  match the various instructions in the documentation and some scripts
  in the source tree.

Semi-Optional apps/ Package
---------------------------

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

Installation Directories with Spaces in the Path
------------------------------------------------

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
  /home/nuttx/nuttx-code.

Notes about Header Files
------------------------

  Other C-Library Header Files.

    Some toolchains are built with header files extracted from a C-library
    distribution (such as newlib).  These header files must *not* be used
    with NuttX because NuttX provides its own, built-in C-Library.  For
    toolchains that do include built-in header files from a foreign C-
    Library, NuttX must be compiled without using the standard header files
    that are distributed with your toolchain.  This prevents including
    conflicting, incompatible header files (such as stdio.h).

  Header Files Provided by Your Toolchain.

    Certain header files, such as setjmp.h, stdarg.h, and math.h, may still
    be needed from your toolchain and your compiler may not, however, be able
    to find these if you compile NuttX without using standard header file.
    If that is the case, one solution is to copy those header file from
    your toolchain into the NuttX include directory.

  Duplicated Header Files.

    There are also a few header files that can be found in the nuttx/include
    directory which are duplicated by the header files from your toolchain.
    stdint.h and stdbool.h are examples.  If you prefer to use the stdint.h
    and stdbool.h header files from your toolchain, those could be copied
    into the nuttx/include/ directory. Using most other header files from
    your toolchain would probably cause errors.

  math.h

    Even though you should not use a foreign C-Library, you may still need
    to use other, external libraries with NuttX.  In particular, you may
    need to use the math library, libm.a.  NuttX supports a generic, built-in
    math library that can be enabled using CONFIG_LIBM=y.  However, you may
    still want to use a higher performance external math library that has
    been tuned for your CPU.  Sometimes such such tuned math libraries are
    bundled with your toolchain.

    The math libary header file, math.h, is a then special case.  If you do
    nothing, the standard math.h header file that is provided with your
    toolchain will be used.

    If you have a custom, architecture specific math.h header file, then
    that header file should be placed at arch/<cpu>/include/math.h.  There
    is a stub math.h header file located at include/nuttx/math.h.  This stub
    header file can be used to "redirect" the inclusion to an architecture-
    specific math.h header file.  If you add an architecture specific math.h
    header file then you should also define CONFIG_ARCH_MATH_H=y in your
    NuttX Configuration file.  If CONFIG_ARCH_MATH_H is selected, then the
    top-level Makefile will copy the stub math.h header file from
    include/nuttx/matn.h to include/math.h where it will become the system
    math.h header file.  The stub math.h header file does nothing other
    than to include that archicture-specific math.h header file as the
    system math.h header file.

  float.h

    If you enable the generic, built-in math library, then that math library
    will expect your toolchain to provide the standard float.h header file.
    The float.h header file defines the properties of your floating point
    implementation.  It would always be best to use your toolchain's float.h
    header file but if none is avaiable, a default float.h header file will
    provided if this option is selected.  However, there is no assurance that
    the settings in this float.h are actually correct for your platform!

  stdarg.h

    In most cases, the correct version of stdarg.h is the version provided with your toolchain.  However, sometimes there are issues with with using your toolchains stdarg.h.  For example, it may attempt to draw in header files that do not exist in NuttX or perhaps the header files that is uses are not compatible with the NuttX header files.  In those cases, you can use an architecture-specific stdarg.h header file by defining CONFIG_ARCH_STDARG_H=y.
    See the discussion above for the math.h header.  This setting works exactly
    the same for the stdarg.h header file.

CONFIGURING NUTTX
^^^^^^^^^^^^^^^^^

Instantiating "Canned" Configurations
-------------------------------------

"Canned" NuttX configuration files are retained in:

  configs/<board-name>/<config-dir>

Where <board-name> is the name of your development board and <config-dir>.
Configuring NuttX requires only copying three files from the <config-dir>
to the directory where you installed NuttX (TOPDIR) (and sometimes one
additional file to the directory the NuttX application package (APPSDIR)):

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

  Copy configs/<board-name>/<config-dir>/appconfig to ${APPSDIR}/.config

    The appconfig file describes the applications that need to be
    built in the appliction directory (APPSDIR).  Not all configurations
    have an appconfig file.  This file is deprecated and will not be
    used with new defconfig files produced with the mconf configuration
    tool.

General information about configuring NuttX can be found in:

  ${TOPDIR}/configs/README.txt
  ${TOPDIR}/configs/<board-name>/README.txt

There is a configuration script in the tools/ directory that makes this
easier.  It is used as follows:

  cd ${TOPDIR}/tools
  ./configure.sh <board-name>/<config-dir>

There is an alternative Windows batch file that can be used in the
windows native enironment like:

  cd ${TOPDIR}\tools
  configure.bat <board-name>\<config-dir>

See tools/README.txt for more information about these scripts.

NuttX Configuration Tool
------------------------

  An automated tool is under development to support re-configuration
  of NuttX.  This tool, however, is not yet quite ready for general
  usage.

  This automated tool is based on the kconfig-frontends application
  available at http://ymorin.is-a-geek.org/projects/kconfig-frontends
  (A snapshot of this tool is also available at ../misc/tools).  This
  application provides a tool called 'mconf' that is used by the NuttX
  top-level Makefile.  The following make target is provided:

    make menuconfig

  This make target will bring up NuttX configuration menus.  The
  'menuconfig' target depends on two things:

  1. The Kconfig configuration data files that appear in almost all
     NuttX directories.  These data files are the part that is still
     under development (patches are welcome!).  The Kconfig files
     contain configuration information for the configuration settings
     relevant to the directory in which the Kconfig file resides.

     NOTE: For a description of the syntax of this configuration file,
     see ../misc/tools/kconfig-language.txt.

  2. The 'mconf' tool.  'mconf' is part of the kconfig-frontends
     package.  You can download that package from the website
     http://ymorin.is-a-geek.org/projects/kconfig-frontends or you
     can use the snapshot in ../misc/tools.

     Building may be as simple as 'configure; make; make install'
     but there may be some build complexities, especially if you
     are building under Cygwin.  See the more detailed build
     instructions at ../misc/tools/README.txt

     The 'make install' step will, by default, install the 'mconf'
     tool at /usr/local/bin/mconf.  Where ever you choose to
     install 'mconf', make certain that your PATH variable includes
     a path to that installation directory.

  The basic configuration order is "bottom-up":

    - Select the build environment,
    - Select the processor,
    - Select the board,
    - Select the supported peripherals
    - Configure the device drivers,
    - Configure the application options on top of this.

  This is pretty straight forward for creating new configurations
  but may be less intuitive for modifying existing configurations.

Incompatibilities with Older Configurations
-------------------------------------------

  ***** WARNING *****

  The old legacy, manual configurations and the new kconfig-frontends
  configurations are not 100% compatible.  Old legacy configurations
  can *not* be used with the kconfig-frontends tool:  If you run
  'make menuconfig' with a legacy configuration the resulting
  configuration will probably not be functional.

  Q: How can I tell if a configuration is a new kconfig-frontends
     configuration or an older, manual configuration?

  A: a) New kcondfig-frontends configurations will have this setting
        within the defconfig/.config file":

          CONFIG_NUTTX_NEWCONFIG=y

     b) Only old, manual configurations will have an appconfig file

Converting Older Configurations to use the Configuration Tool
-------------------------------------------------------------

  Q: How can I convert a older, manual configuration into a new,
     kconfig-frontends toolchain.

  A: 1) Look at the appconfig file:  Each application path there
        will now have to have an enabling setting.  For example,
        if the old appconfig file had:

          CONFIGURED_APPS = examples/ostest

        Then the new configuration will need:

          CONFIG_EXAMPLES_OSTEST=y

        The appconfig file can then be deleted because it will not
        be needed after the conversion.

     2) Build the cmpconfig utility at tools:

          cd tools
          make -f Makefile.host cmpconfig

     3) Perform these steps repeatedly until you are convinced that
        the configurations are the same:

        a) Repeat the following until you have account for all of the differences:

             cp configs/<board>/<condfiguration>/defconfig .config
             make menuconfig  (Just exit and save the new .config file)
             tools/cmpconfig configs/<board>/<condfiguration>/defconfig .config | grep file1
 
           The final grep will show settings in the old defconfig file that
           do not appear in the new .config file (or have a different value
           in the new .config file).  In the new configuration, you will
           probably have to enable certain groups of features.  Such
           hierarachical enabling options were not part of the older
           configuration.

        b) Then make sure these all make sense:

             tools/cmpconfig configs/<board>/<condfiguration>/defconfig .config | grep file2

           The final grep will show settings in the new .config file that
           do not appear in the older defconfig file (or have a different value
           in the new .config file).  Here you should see only the new
           hierarachical enabling options (such as CONFIG_SPI or CONFIG_MMCSD)
           plus some other internal configuration settings (like CONFIG_ARCH_HAVE_UART0).
           You will have to convince yourself that these new settings all make sense.

     4) Finally, update the configuration:

          cp .config configs/<board>/<condfiguration>/defconfig
          rm configs/<board>/<condfiguration>/appconfig

        NOTE:  You should comment out the line containing the CONFIG_APPS_DIR
        in the new defconfig file.  Why?  Because the application directory
        may reside at a different location when the configuration is installed
        at some later time.

          # CONFIG_APPS_DIR="../apps"

     5) The updated configuration can then be instantiated in the normal
        fashion:

          cd tools
          ./configure.sh <board>/<condfiguration>

        (or configure.bat for the case of the Windows native build).

        NOTE: If CONFIG_APPS_DIR is not defined in the defconfig file,
        the configure.sh script will find and add the new, correct path to
        the application directory (CONFIG_APPS_DIR) when it copies the
        defconfig file to the .config file.  This is the setting that was
        commented out in step 4.

NuttX Configuration Tool under DOS
----------------------------------

  Recent versions of NuttX support building NuttX from a native Windows
  CMD.exe shell (see "Native Windows Build" below).  But kconfig-frontends
  is a Linux tool.  There have been some successes building a Windows
  native version of the kconfig-frontends tool, but that is not ready
  for prime time.

  At this point, there are only a few options for the Windows user:

  1. You can run the configuration tool using Cygwin.  However, the
     Cygwin Makefile.win will complain so to do this will, you have
     to manually edit the .config file:

      a. Delete the line: CONFIG_WINDOWS_NATIVE=y
      b. Change the apps/ directory path, CONFIG_APPS_DIR to use Unix
         style delimiters.  For example, change "..\apps" to "../apps"

     And of course, after you use the configuration tool you need to
     restore CONFIG_WINDOWS_NATIVE=y and the correct CONFIG_APPS_DIR.

  2) You can, with some effort, run the the Cygwin mconf tool directly
     in the CMD.exe shell.  In this case, you do not have to modify the
     .config file, but there are other complexities:

      a. You need to temporarily set the Cgywin directories in the PATH
         variable then run mconf manually like:

          mconf Kconfig

         There is a Windows bacht file at tools/kconfig.bat that automates
         these steps:

         tools/kconfig menuconfig

       b. There is an issue with accessing DOS environment variables from
          the Cygwin mconf running in the CMD.exe shell.  The following
          change to the top-level Kconfig file seems to work around these
          problems:
  
          config APPSDIR
              string
          -   option env="APPSDIR"
          +   default "../apps"

TOOLCHAINS
^^^^^^^^^^

Cross-Development Toolchains
----------------------------

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
-------------------------

  For many configurations, a DIY set of tools is available for NuttX.  These
  tools can be downloaded from the NuttX SourceForge file repository.  After
  unpacking the buildroot tarball, you can find instructions for building
  the tools in the buildroot/configs/README.txt file.

  Check the README.txt file in the configuration director for your board
  to see if you can use the buildroot toolchain with your board (this
  README.txt file is located in configs/<board-name>/README.txt).

  This toolchain is available for both the Linux and Cygwin development
  environments.

  Advantages:  (1) NuttX header files are built into the tool chain,
  and (2) related support tools like NXFLAT tools and the ROMFS
  genromfs tools can be built into your toolchain.

  Disadvantages:  This tool chain is not was well supported as some other
  toolchains.  GNU tools are not my priority and so the buildroot tools
  often get behind.  For example, the is still no EABI support in the
  NuttX buildroot toolchain for ARM.

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
--------

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
-----------

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

Build Targets and Options
-------------------------

  Build Targets:
  Below is a summary of the build targets available in the top-level
  NuttX Makefile:

  all

    The default target builds the NuttX executable in the selected output
    formats.

  clean

    Removes derived object files, archives, executables, and temporary
    files, but retains the configuration and context files and directories.

  distclean

    Does 'clean' then also removes all configuration and context files.
    This essentially restores the directory structure to its original,
    unconfigured stated.

  Application housekeeping targets.  The APPDIR variable refers to the user
  application directory.  A sample apps/ directory is included with NuttX,
  however, this is not treated as part of NuttX and may be replaced with a
  different application directory.  For the most part, the application
  directory is treated like any other build directory in the Makefile script.
  However, as a convenience, the following targets are included to support
  housekeeping functions in the user application directory from the NuttX
  build directory.

  apps_clean

    Perform the clean operation only in the user application directory

  apps_distclean

    Perform the distclean operation only in the user application directory.
    The apps/.config file is preserved so that this is not a "full" distclean
    but more of a configuration "reset."

  export

    The export target will package the NuttX libraries and header files into
    an exportable package.  Caveats: (1) These needs some extension for the KERNEL
    build. (2) The logic in tools/mkexport.sh only supports GCC and, for example,
    explicitly assumes that the archiver is 'ar'

  download

    This is a helper target that will rebuild NuttX and download it to the target
    system in one step.  The operation of this target depends completely upon
    implementation of the DOWNLOAD command in the user Make.defs file.  It will
    generate an error an error if the DOWNLOAD command is not defined.

  The following targets are used internally by the make logic but can be invoked
  from the command under certain conditions if necessary.

  depend

    Create build dependencies. (NOTE:  There is currently no support for build
    dependencies under Cygwin using Windows-native toolchains.)

  context

    The context target is invoked on each target build to assure that NuttX is
    properly configured.  The basic configuration steps include creation of the
    the config.h and version.h header files in the include/nuttx directory and
    the establishment of symbolic links to configured directories.

  clean_context

    This is part of the distclean target.  It removes all of the header files
    and symbolic links created by the context target.

  Build Options:
  Of course, the value any make variable an be overriden from the make command
  line.  However, there is one particular variable assignment option that may
  be useful to you:

  V=1

    This is the build "verbosity flag."  If you specify V=1 on the make command
    line, you will see the exact commands used in the build. This can be very
    useful when adding new boards or tracking down compile time errors and
    warnings (Contributed by Richard Cochran).

Native Windows Build
--------------------

  The beginnings of a Windows native build are in place but still not full
  usable as of this writing.  The windows native build logic initiatiated
  if CONFIG_WINDOWS_NATIVE=y is defined in the NuttX configuration file:

  This build:

    - Uses all Windows style paths
    - Uses primarily Windows batch commands from cmd.exe, with
    - A few extensions from GNUWin32

  In this build, you cannot use a Cygwin or MSYS shell. Rather the build must
  be performed in a Windows CMD shell. Here is a better shell than than the
  standard issue, CMD shell:  ConEmu which can be downloaded from:
  http://code.google.com/p/conemu-maximus5/

  Build Tools.  The build still relies on some Unix-like commands.  I use
  the GNUWin32 tools that can be downloaded from http://gnuwin32.sourceforge.net/.

  Host Compiler:  I use the MingGW compiler which can be downloaded from
  http://www.mingw.org/.  If you are using GNUWin32, then it is recommended
  the you not install the optional MSYS components as there may be conflicts.

  This capability should still be considered a work in progress because:
 
  (1) It has not been verfied on all targets and tools, and
  (2) it still lacks some of the creature-comforts of the more mature environments
      (like 'make menuconfig' support.  See the section "NuttX Configuration Tool
      under DOS" above).

   There is an alternative to the setenv.sh script available for the Windows
   native environment: tools/configure.bat.  See tools/README.txt for additional
   information.

Installing GNUWin32
-------------------

  The Windows native build will depend upon a few Unix-like tools that can be
  provided either by MSYS or GNUWin32.  The GNUWin32 are available from
  http://gnuwin32.sourceforge.net/.  GNUWin32 provides ports of tools with a
  GPL or similar open source license to modern MS-Windows (Microsoft Windows
  2000 / XP / 2003 / Vista / 2008 / 7).  See
  http://gnuwin32.sourceforge.net/packages.html for a list of all of the tools
  available in the GNUWin32 package.

  The SourceForge project is located here:
  http://sourceforge.net/projects/gnuwin32/.  The project is still being
  actively supported (although some of the Windows ports have gotten very old).

  Some commercial toolchains include a subset of the GNUWin32 tools in the
  installation.  My recommendation is that you download the GNUWin32 tools
  directly from the sourceforge.net website so that you will know what you are
  using and can reproduce your build environment.

  GNUWin32 Installation Steps:

  The following steps will download and execute the GNUWin32 installer.

  1. Download GetGNUWin32-x.x.x.exe from
     http://sourceforge.net/projects/getgnuwin32/files/.  This is the
     installer.  The current version as of this writing is 0.6.3.

  2. Run the installer.

  3. Accept the license.

  4. Select the installation directory.  My recommendation is the
     directory that contains this README file (<this-directory>).

  5. After running GetGNUWin32-0.x.x.exe, you will have a new directory
     <this-directory>/GetGNUWin32

  Note the the GNUWin32 installer didn't install GNUWin32.  Instead, it
  installed another, smarter downloader.  That downloader is the GNUWin32
  package management tool developed by the Open SSL project.

  The following steps probably should be performed from inside a DOS shell.

  6. Change to the directory created by GetGNUWin32-x.x.x.exe

    cd GetGNUWin32

  7. Execute the download.bat script.  The download.bat script will download
     about 446 packages!  Enough to have a very complete Linux-like environment
     under the DOS shell.  This will take awhile.  This step only downloads
     the packages and the next step will install the packages.

     download

  8. This step will install the downloaded packages.  The argument of the
     install.bat script is the installation location.  C:\gnuwin32 is the
     standard install location:

     install C:\gnuwin32

  NOTE:  This installation step will install *all* GNUWin32 packages... far
  more than you will ever need.  If disc space is a problem for you, you might
  need to perform a manual installation of the individual ZIP files that you
  will find in the <this directory>/GetGNUWin32/packages directory.

CYGWIN BUILD PROBLEMS
^^^^^^^^^^^^^^^^^^^^^

Strange Path Problems
---------------------

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

Window Native Toolchain Issues
------------------------------

  There are many popular Windows native toolchains that may be used with NuttX.
  Examples include CodeSourcery (for Windows), devkitARM, and several vendor-
  provied toolchains.  There are several limitations with using a and Windows
  based toolchain in a Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had no effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.  The rebuild
     is not a long as you might think because there is no dependency checking
     if you are using a native Windows toolchain.  That bring us to #3:

  3. Dependencies are not made when using Windows versions of the GCC on a POSIX
     platform (i.e., Cygwin).  This is because the dependencies are generated
     using Windows paths which do not work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

     If you are building natively on Windows, then no such conflict exists
     and the best selection is:

       MKDEP                = $(TOPDIR)/tools/mkdeps.exe 

General Pre-built Toolchain Issues

  To continue with the list of "Window Native Toolchain Issues" we can add
  the following.  These, however, are really just issues that you will have
  if you use any pre-built toolchain (vs. building the NuttX toolchain from
  the NuttX buildroot package):

  There may be incompatibilities with header files, libraries, and compiler
  built-in functions at detailed below.  For the most part, these issues
  are handled in the existing make logic.  But if you are breaking new ground,
  then you may incounter these:

  4. Header Files.  Most pre-built toolchains will build with a foreign C
     library (usually newlib, but maybe uClibc or glibc if you are using a 
     Linux toolchain).  This means that the header files from the foreign
     C library will be built into the toolchain.  So if you "include <stdio.h>",
     you will get the stdio.h from the incompatible, foreign C library and
     not the nuttx stdio.h (at nuttx/include/stdio.h) that you wanted.

     This can cause really confusion in the buildds and you must always be
     sure the -nostdinc is included in the CFLAGS.  That will assure that
     you take the include files only from 

  5. Libraries.  What was said above header files applies to libraries.
     You do not want to include code from the libraries of any foreign
     C libraries built into your toolchain.  If this happens you will get
     perplexing errors about undefined sysmbols.  To avoid these errors,
     you will need to add -nostdlib to your CFLAGS flags to assure that
     you only take code from the NuttX libraries.

     This, however, may causes other issues for libraries in the toolchain
     that you do want (like libgcc.a or libm.a).  These are special-cased
     in most Makefiles, but you could still run into issues of missing
     libraries.

  6. Built-Ins.  Some compilers target a particular operating system.
     Many people would, for example, like to use the same toolchain to
     develop Linux and NuttX software.  Compilers built for other
     operating systems may generate incompatible built-in logic and,
     for this reason, -fno-builtin should also be included in your
     C flags

  And finally you may not be able to use NXFLAT.

  7. NXFLAT. If you use a pre-built toolchain, you will lose all support
     for NXFLAT.  NXFLAT is a binary format described in
     Documentation/NuttXNxFlat.html.  It may be possible to build
     standalone versions of the NXFLAT tools; there are a few examples
     of this in the misc/buildroot/configs directory.  However, it
     is possible that there could be interoperability issues with
     your toolchain since they will be using different versions of
     binutials and possibly different ABIs.

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
 |   |       |- z80/README.txt
 |   |       `- z180/README.txt, z180_mmu.txt
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
 |   |- compal_e88
 |   |   `- README.txt
 |   |- compal_e99
 |   |   `- README.txt
 |   |- demo0s12ne64/
 |   |   `- README.txt
 |   |- ea3131/
 |   |   `- README.txt
 |   |- ea3152/
 |   |   `- README.txt
 |   |- eagle100/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- ekk-lm3s9b96/
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
 |   |-  fire-stm32v2/
 |   |   `- README.txt
 |   |-  hymini-stm32v/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
 |   |   `- README.txt
 |   |- lincoln60/
 |   |   `- README.txt
 |   |- kwikstik-k40/
 |   |   `- README.txt
 |   |- lm3s6432-s2e/
 |   |   |- include/README.txt
 |   |   |- src/README.txt
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
 |   |- lpc4330-xplorer/
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
 |   |- mirtoo/
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
 |   |- p112/
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
 |   |- shenzhou/
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
 |   |- stm3220g-eval/
 |   |   `- README.txt
 |   |- stm3240g-eval/
 |   |   `- README.txt
 |   |- stm32f100rc_generic/
 |   |   `- README.txt
 |   |- stm32f4discovery/
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
 |   |- sercomm/
 |   |   `- README.txt
 |   |- syslog/
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
 |- libc/
 |   `- README.txt
 |- libxx/
 |   `- README.txt
 |- mm/
 |   `- README.txt
 |- syscall/
 |   `- README.txt
 `- tools/
     `- README.txt

apps
 |- examples/
 |   |- json/README.txt
 |   |- pashello/README.txt
 |   `- README.txt
 |- graphics/
 |   `- tiff/README.txt
 |- interpreters/
 |   |- ficl
 |   |  `- README.txt
 |   `- README.txt
 |- modbus/
 |   `- README.txt
 |- netutils/
 |   |- discover
 |   |  `- README.txt
 |   |- ftpc
 |   |  `- README.txt
 |   |- json
 |   |  `- README.txt
 |   |- telnetd
 |   |  `- README.txt
 |   `- README.txt
 |- nshlib/
 |   `- README.txt
 |- NxWidgets/
 |   `- README.txt
 |- system/
 |   |- i2c/README.txt
 |   |- free/README.txt
 |   |- install
 |   |  `- README.txt
 |   |- poweroff
 |   |  `- README.txt
 |   |- ramtron
 |   |  `- README.txt
 |   |- sdcard
 |   |  `- README.txt
 |   `- sysinfo
 |      `- README.txt
 `- README.txt

 NxWidgets
 |- Doxygen
 |   `- README.txt
 |- tools
 |   `- README.txt
 |- UnitTests
 |   `- README.txt
 `- README.txt
