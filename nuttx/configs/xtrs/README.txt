xtrs README
^^^^^^^^^^^^^

Current status
^^^^^^^^^^^^^^

The xtrs port is not operational yet; some work still needs to be done.

Contents
^^^^^^^^

  o Getting a TRS80 emulator and DOS disks
  o Loading an executable into xtrs
  o Configuring NuttX
  o Reconfiguring NuttX
  o Reconfiguring for Linux, OSX, or Cygwin
  o SDCC
  o Building the SDCC toolchain

Getting a TRS80 emulator and DOS disks
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This port uses a vintage computer based on the Z80, the TRS80.
There's a main page describing the different models of TRS80.
See: http://www.trs-80.com

An emulator for this computer is available to run TRS80 programs on a 
linux platform (http://www.tim-mann.org/xtrs.html).

Other emulators are available for other platforms. 
See http://www.trs-80.com, click on the link Emulators.

TRSDOS, LDOS and other softwares are available at:
http://discover-net.net/~dmkeil/trs80/software/trs-dos.htm

Or you can get TRSDOS 1.3 and 6.1 from this site; it's included with the emulator.
http://discover-net.net/~dmkeil/trs80/model4.htm

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using version 2.7.0 of the SDCC toolchain.

Loading an executable into xtrs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At http://www.trs-80.com click on the link: Getting a Software Onto an Emulator.

Configuring NuttX
^^^^^^^^^^^^^^^^^

  ostest

    This configuration performs a simple, minimal OS test using
    examples/ostest.  This can be configurated as follows:

       cd tools
       ./configure.sh xtrs/ostest
       cd -
       . ./setenv.sh

  nsh

    This configuration file builds NSH (examples/nsh).  This
    configuration is not functional due to issues with use of the
    simulated serial driver (see the TODO list).

    This configuration can be selected by:

       cd tools
       ./configure.sh xtrs/nsh
       cd -
       . ./setenv.sh

  pashello

    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

    This configuration is not usable because the resulting binary
    is too large for the z80 address space.

    This configuration can be selected by:

       cd tools
       ./configure.sh xtrs/pashello
       cd -
       . ./setenv.sh

Reconfiguring NuttX
^^^^^^^^^^^^^^^^^^^

These configurations all use the kconfig-frontends, mconf-based configuration
tool.  To change this configuration using that tool, you should:

  a. Build and install the mconf tool.  See nuttx/README.txt and
     misc/tools/README.txt

  b. Execute 'make menuconfig' in nuttx/ in order to start the reconfiguration
     process.

Reconfiguring for Linux, OSX, or Cygwin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

All of the z80 configurations in this this directory are set up to build in a
Windows CMD.exe shell.  This configuration requires the MinGW host compiler
and severl GNUWin32 tools (see discussion in the top-level NuttX/README.txt
file).

These configurations can be converted to run under Linux (or Cygwin or OSX),
by modifying the configuration file as follows:

  -CONFIG_HOST_WINDOWS=y
  -CONFIG_WINDOWS_NATIVE=y
  +CONFIG_HOST_LINUX=y
 
  -CONFIG_Z80_TOOLCHAIN_SDCCW=y
  +CONFIG_Z80_TOOLCHAIN_SDCCL=y

You may need to first manually change the CONFIG_APPS_DIR="..\apps" definition
in the .config file because the backslash may upset some Unix-based tools.

You cannot use the default setenv.bat in these Unix-like enviroments because
that is a Windows batch file.  Use configs/z80sim/script/setenv.sh instead.

SDCC
^^^^

These z80 configurations all use the SDCC toolchain (http://sdcc.sourceforge.net/).
Source and pre-built SDCC binaries can be downloaded from the SDCC SourceForge
site: http://sourceforge.net/projects/sdcc/files/ .  Pre-built binaries are
available for Linux, MAC OSX, and for Win32.  Various SDCC options can be
selected with:

  CONFIG_Z80_TOOLCHAIN_SDCCL=y : SDCC for Linux, MAC OSX or Cygwin (see below)
  CONFIG_Z80_TOOLCHAIN_SDCCW=y : SDCC for Win32

SDCC versions 3.2.0 or higher are recommended.

Building the SDCC toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^

You may also want to build your own SDCC toolchain.  You might want to do this,
for example, if you are running under Cygwin and want a Cygwin compatible
SDCC toolchain.

The SDCC toolchain is built with the standard configure/make/make install
sequence.  However, some special actions are required to generate libraries
compatible with this build.  First start with the usual steps

  download
  unpack
  cd sdcc
  ./configure

But before making, we need to apply a patch to the SDCC 2.6.0 source
so that the z80 assembler can handle long symbol names

  Apply sdcc-2.6.0-asz80-symlen.patch
  cd sdcc/device/lib

Then make the SDCC binaries

  cd sdcc
  make

and install SDCC:

  sudo make install

