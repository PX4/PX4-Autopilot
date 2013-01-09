README
^^^^^^

This is the README file for the port of NuttX to the Neuros OSD.

CONTENTS
^^^^^^^^
  - Dev vs. Production Neuros OSD v1.0 boards
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX buildroot Toolchain
  - ARM/DM320-specific Configuration Options
  - Configurations
  - Configuration Options

Dev vs. Production Neuros OSD v1.0 boards
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  This port supports both the original Neuros OSD v1.0 Dev Board.
  This port has recently been extended to V1.0 Production board (and
  that is now the default configuration). References:

    http://www.neurostechnology.com/neuros-developer-community
    http://wiki.neurostechnology.com/index.php/OSD_1.0_Developer_Home
    http://wiki.neurostechnology.com/index.php/DM320_Platform_development

  There are some differences between the Dev Board and the currently
  available commercial v1.0 Boards, most notably in the amount of memory:
  8Mb FLASH and 32Mb RAM vs. 16Mb and 64Mb as in the production board.
  See the following for more information:
    
     http://wiki.neurostechnology.com/index.php/OSD_Developer_Board_v1

  NuttX operates on the ARM9EJS of this dual core processor.  The DSP
  is available and unused.

  STATUS: This port is code complete, verified, and included in the
  NuttX 0.2.1 release.

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. Raisonance GNU toolchain, or
  4. The NuttX buildroot Toolchain (see below), or
  5. Any generic arm-none-eabi GNU toolchain.

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_DM320_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_DM320_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_DM320_DEVKITARM=y      : devkitARM under Windows
    CONFIG_DM320_BUILDROOT=y	    : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_ARM_TOOLCHAIN_GNU_EABI : Generic arm-none-eabi toolchain

  If you are not using CONFIG_DM320_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  The toolchain may also be set using the kconfig-mconf utility (make menuconfig)
  or by passing CONFIG_ARM_TOOLCHAIN=<toolchain> to make, where <toolchain> is one
  of CODESOURCERYW, CODESOURCERYL, DEVKITARM, BUILDROOT or GNU_EABI as described
  above.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Raisonance toolchains are
  Windows native toolchains.  The CodeSourcey (for Linux) and NuttX buildroot
  toolchains are Cygwin and/or Linux native toolchains. There are several limitations
  to using a Windows based toolchain in a Cygwin environment.  The three biggest are:

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

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.
 
  Generic arm-none-eabi GNU Toolchain
  -----------------------------------
  There are a number of toolchain projects providing support for ARMv4/v5
  class processors, including:

    GCC ARM Embedded
      https://launchpad.net/gcc-arm-embedded

    Summon ARM Toolchain
      https://github.com/esden/summon-arm-toolchain

    Yagarto
      http://www.yagarto.de

  Others exist for various Linux distributions, MacPorts, etc.  Any version
  based on GCC 4.6.3 or later should work.

IDEs
^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project (There is a simple RIDE project
  in the RIDE subdirectory).
  
  Makefile Build
  --------------
  Under Eclipse, it is pretty easy to set up an "empty makefile project" and
  simply use the NuttX makefile to build the system.  That is almost for free
  under Linux.  Under Windows, you will need to set up the "Cygwin GCC" empty
  makefile project in order to work with Windows (Google for "Eclipse Cygwin" -
  there is a lot of help on the internet).

  Native Build
  ------------
  Here are a few tips before you start that effort:

  1) Select the toolchain that you will be using in your .config file
  2) Start the NuttX build at least one time from the Cygwin command line
     before trying to create your project.  This is necessary to create
     certain auto-generated files and directories that will be needed.
  3) Set up include pathes:  You will need include/, arch/arm/src/dm320,
     arch/arm/src/common, arch/arm/src/arm, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/arm/up_head.S.  You may have to build the NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by the IDE.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the ARM926 GCC toolchain (if
  different from the default).

  If you have no ARM toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).

  1. You must have already configured Nuttx in <some-dir>nuttx.

     cd tools
     ./configure.sh ntosd-dm320/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack

  4. cd <some-dir>/buildroot

  5. cp configs/arm-defconfig .config OR
     cp configs/arm926t_defconfig-4.2.4 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h so that the PATH variable includes the path to the
     newly built binaries.

ARM/DM320-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_ARM926EJS=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=dm320

	CONFIG_ARCH_CHIP_name - For use in C code

	   CONFIG_ARCH_CHIP_DM320

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=ntosd-dm320

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_NTOSD_DM320 (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM.

	CONFIG_DRAM_START - The start address of installed DRAM

	CONFIG_DRAM_VSTART - The startaddress of DRAM (virtual)

	CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
	   have LEDs

	CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
	   stack. If defined, this symbol is the size of the interrupt
	   stack in bytes.  If not defined, the user task stacks will be
	  used during interrupt handling.

	CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

	CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
	   cause a 100 second delay during boot-up.  This 100 second delay
	   serves no purpose other than it allows you to calibratre
	   CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
	   the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
	   the delay actually is 100 seconds.

  DM320 specific device driver settings

	CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
	   console and ttys0 (default is the UART0).
	CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer
	CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer
	CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
	CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
	CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
	CONFIG_UARTn_2STOP - Two stop bits

  DM320 USB Configuration

	CONFIG_DM320_GIO_USBATTACH
	   GIO that detects USB attach/detach events
	CONFIG_DM320_GIO_USBDPPULLUP
	   GIO 
	CONFIG_DMA320_USBDEV_DMA
	   Enable DM320-specific DMA support
	CONFIG_DM320_GIO_USBATTACH=6

Configurations
^^^^^^^^^^^^^^

Each Neuros OSD configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh ntosd-dm320/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

nettest
^^^^^^^

This alternative configuration directory may be used to
enable networking using the OSDs DM9000A Ethernet interface.
It uses examples/nettest to excercise the TCP/IP network.

nsh
^^^

Configures the NuttShell (nsh) located at examples/nsh.  The
Configuration enables both the serial and telnetd NSH interfaces.

ostest
^^^^^^

This configuration directory, performs a simple OS test using
examples/ostest.

poll
^^^^

This configuration exercises the poll()/select() text at
examples/poll

thttpd
^^^^^^

This builds the THTTPD web server example using the THTTPD and
the examples/thttpd application.

udp
^^^

This alternative configuration directory is similar to nettest
except that is use examples/upd to exercise UDP.

uip
^^^

This configuration file demonstrates the tiny webserver
at examples/uip.

Configuration Options
^^^^^^^^^^^^^^^^^^^^^

In additional to the common configuration options listed in the
file configs/README.txt, there are other configuration options
specific to the DM320:

 CONFIG_ARCH - identifies the arch subdirectory and, hence, the
   processor architecture.
 CONFIG_ARCH_name - for use in C code.  This identifies the
   particular chip or SoC that the architecture is implemented
   in.
 CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory
 CONFIG_ARCH_CHIP_name - For use in C code
 CONFIG_ARCH_BOARD - identifies the configs subdirectory and, hence,
   the board that supports the particular chip or SoC.
 CONFIG_ENDIAN_BIG - define if big endian (default is little endian)
 CONFIG_ARCH_BOARD_name - for use in C code
 CONFIG_BOARD_LOOPSPERMSEC - for delay loops
 CONFIG_ARCH_LEDS - Use LEDs to show state.
 CONFIG_DRAM_SIZE - Describes the internal DRAM.
 CONFIG_DRAM_START - The start address of internal DRAM
 CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

DM320 specific device driver settings

 CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
   console and ttys0 (default is the UART0).
 CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
   This specific the size of the receive buffer
 CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
   being sent.  This specific the size of the transmit buffer
 CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
 CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
 CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
 CONFIG_UARTn_2STOP - Two stop bits

DM320 USB Configuration

 CONFIG_DM320_GIO_USBATTACH
   GIO that detects USB attach/detach events
 CONFIG_DM320_GIO_USBDPPULLUP
   GIO connected to D+.  Support software connect/disconnect.
 CONFIG_DMA320_USBDEV_DMA
   Enable DM320-specific DMA support

Neuros OSD Configuration Options

 CONFIG_ARCH_NTOSD_DEVBOARD - Selects the old NTOSD development board.
   The default is the production OSD board which differs in 
   several ways.
