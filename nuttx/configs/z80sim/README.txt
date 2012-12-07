z80sim README
^^^^^^^^^^^^^

This port uses a primitive, emulated Z80 and the SDCC toolchain.
The instruction set emulator can be found in the NuttX SVN at
http://svn.code.sf.net/p/nuttx/code/trunk/misc/sims/z80sim

The SDCC toolchain is available from http://sdcc.sourceforge.net/.  All
testing has been performed using version 2.6.0 of the SDCC toolchain.

Contents
^^^^^^^^

  o Configuring NuttX
  o SDCC
  o Building the SDCC toolchain
  o SDCC Update
  o Newer SDCC Versions

Configuring NuttX
^^^^^^^^^^^^^^^^^

  ostest

    This configuration performs a simple, minimal OS test using
    examples/ostest.  This can be configurated as follows:

       cd tools
       ./configure.sh z80sim/ostest
       cd -
       . ./setenv.sh

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the mconf tool.  See nuttx/README.txt and
          misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. The default setup for this configuration uses a windows native build.
       As of this writing, the native Windows build still does not work.

       This configuration was last verified sucessfully prior to the
       the configure to Kconfig/mconf tool using SDCC 2.6.0 built to run
       natively under Cygwin.

    3. This configuration can be converted to run under Linux (or Cygwin or
       OSX), by modifying the configuration file as follows:

        -CONFIG_HOST_WINDOWS=y
        -CONFIG_WINDOWS_NATIVE=y
        +CONFIG_HOST_LINUX=y
 
        -CONFIG_Z80_TOOLCHAIN_SDCCW=y
        +CONFIG_Z80_TOOLCHAIN_SDCCL=y

       You make also have to change the value of CONFIG_APPS_DIR.  You cannot
       use the default setenv.bat.  Use configs/z80sim/script/setenv.sh instead.

  nsh

    This configuration file builds NSH (examples/nsh).  This
    configuration is not functional due to issues with use of the
    simulated serial driver (see the TODO list).

    This configuration can be selected by:

       cd tools
       ./configure.sh z80sim/nsh
       cd -
       . ./setenv.sh

 pashello

    Configures to use examples/pashello for execution from FLASH
    See examples/README.txt for information about pashello.

    This configuration is not usable because the resulting binary
    is too large for the z80 address space.

    This configuration can be selected by:

       cd tools
       ./configure.sh z80sim/pashello
       cd -
       . ./setenv.sh

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

Then make the SDCC binaries

  cd sdcc
  make

and install SDCC:

  sudo make install

