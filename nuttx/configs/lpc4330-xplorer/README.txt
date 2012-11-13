README
======

README for NuttX port to the LPC4330-Xplorer board from NGX Technologies
featuring the NXP LPC4330FET100 MCU

Contents
========

  - LPC4330-Xplorer development board
  - Status
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - Code Red IDE/Tools
    Booting the LPCLink
    Using GDB
    Troubleshooting
    Command Line Flash Programming
    Executing from SPIFI
    USB DFU Booting
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - Serial Console
  - FPU
  - LPC4330-Xplorer Configuration Options
  - Configurations

LPC4330-Xplorer board
=====================

  Memory Map
  ----------

  Block                 Start      Length
  Name                  Address
  --------------------- ---------- ------
  RAM                   0x10000000   128K
  RAM2                  0x10080000    72K
  RAMAHB                0x20000000    32K
  RAMAHB2               0x20008000    16K
  RAMAHB3               0x2000c000    16K
  SPIFI flash           0x1e000000  4096K

  GPIO Usage:
  -----------

  GPIO                              PIN     SIGNAL NAME
  -------------------------------- ------- --------------
  gpio1[12] - LED D2                J10-20  LED1
  gpio1[11] - LED D3                J10-17  LED2
  gpio0[7]  - User Button SW2       J8-25   BTN1

  Console
  -------

  The LPC4330-Xplorer default console is the USB1 virtual COM port (VCOM).

Status
======

  This is the current status of the LPC43xx port:

  - The basic OS test configuration and the basic NSH configurations
    are present and fully verified.  This includes:  SYSTICK system time,
    pin and GPIO configuration, and serial console support.  A SPIFI
    MTD driver is also in place but requires further verification.

  - The following drivers have been copied from the LPC17xx port, but
    require integration into the LPC43xx.  This integration should
    consist of:

    - Remove LPC17xx power, clocking, and pin configuration logic.
    - Adding of clock source and frequency to the board.h file.
    - Adding of LPC43 clock connection and pin configuration logic.
 
    Within any luck, these drivers should come up very quickly:
 
    - lpc43_adc.c,
    - lpc43_dac.c,
    - lpc43_gpdma.c,
    - lpc43_i2c.c,
    - lpc43_spi.c, and
    - lpc43_ssp.c

    These LPC17xx drivers were not brought into the LPC43xx port because
    it appears the these peripherals have been completely redesigned:

    - CAN,
    - Ethernet,
    - USB device, and
    - USB host.

    The following LPC43xx peripherals are unsupported.  Some may be
    compatible with the LPC17xx, but there is no LPC17xx driver to be
    ported:

    - SD/MMC,
    - EMC,
    - USB0,
    - USB1,
    - Ethernet,
    - LCD,
    - SCT,
    - Timers 0-3
    - MCPWM,
    - QEI,
    - Alarm timer,
    - WWDT,
    - RTC,
    - Event monitor, and
    - CAN,

    For the missing drivers some of these can be leveraged from other
    MCUs that appear to support the same peripheral IP.

    - USB0 appears to be the same as the USB OTG peripheral for the
      LPC31xx.  It should be possible to drop in the LPC31xx driver
      with a small porting effort.

    - The Ethernet block looks to be based on the same IP as the
      STM32 Ethernet and, as a result, it should be possible to leverage
      the STM32 Ethernet driver with a little more effort.

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
=====================

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The Code Red GNU toolchain,
  2. The CodeSourcery GNU toolchain,
  3. The Atollic Toolchain,
  4. The devkitARM GNU toolchain,
  5. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery or devkitARM toolchain, you simply need add one of the
  following configuration options to your .config (or defconfig) file:

    CONFIG_LPC43_CODEREDW=y       : Code Red "RedSuite" under Windows
    CONFIG_LPC43_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_LPC43_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_LPC43_ATOLLIC_LITE=y   : The free, "Lite" version of Atollic toolchain under Windows
    CONFIG_LPC43_ATOLLIC_PRO=y    : The paid, "Pro" version of Atollic toolchain under Windows
    CONFIG_LPC43_DEVKITARM=y      : devkitARM under Windows
    CONFIG_LPC43_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_LPC43_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the Code Red, CodeSourcery (for Windows), Atollic and devkitARM toolchains
  are Windows native toolchains.  The CodeSourcery (for Linux) and NuttX buildroot
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

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  The CodeSourcery Toolchain (2009q1)
  -----------------------------------
  The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  The Atollic "Pro" and "Lite" Toolchain
  --------------------------------------
  One problem that I had with the Atollic toolchains is that the provide a gcc.exe
  and g++.exe in the same bin/ file as their ARM binaries.  If the Atollic bin/ path
  appears in your PATH variable before /usr/bin, then you will get the wrong gcc
  when you try to build host executables.  This will cause to strange, uninterpretable
  errors build some host binaries in tools/ when you first make.

  Also, the Atollic toolchains are the only toolchains that have built-in support for
  the FPU in these configurations.  If you plan to use the Cortex-M4 FPU, you will
  need to use the Atollic toolchain for now.  See the FPU section below for more
  information.

  The Atollic "Lite" Toolchain
  ----------------------------
  The free, "Lite" version of the Atollic toolchain does not support C++ nor
  does it support ar, nm, objdump, or objdcopy. If you use the Atollic "Lite"
  toolchain, you will have to set:

    CONFIG_HAVE_CXX=n

  In order to compile successfully.  Otherwise, you will get errors like:

    "C++ Compiler only available in TrueSTUDIO Professional"

  The make may then fail in some of the post link processing because of some of
  the other missing tools.  The Make.defs file replaces the ar and nm with
  the default system x86 tool versions and these seem to work okay.  Disable all
  of the following to avoid using objcopy:

    CONFIG_RRLOAD_BINARY=n
    CONFIG_INTELHEX_BINARY=n
    CONFIG_MOTOROLA_SREC=n
    CONFIG_RAW_BINARY=n

  devkitARM
  ---------
  The devkitARM toolchain includes a version of MSYS make.  Make sure that the
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
====

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project .
  
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
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc43xx,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/common/up_vectors.S.

Code Red IDE/Tools
^^^^^^^^^^^^^^^^^^

  Booting the LPCLink
  -------------------

  The first step is to activate the LPCLink's boot mode.  Some general
  instructions to do this are provided here:

    http://support.code-red-tech.com/CodeRedWiki/BootingLPCLink

  For my RedSuite installation path, that can be done using the following
  steps in a Cygwin bash shell:

    $ /cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/bin/Scripts/bootLPCXpresso.cmd winusb
    Booting LPC-Link with LPCXpressoWIN.enc
    Press any key to continue . . . 

  The same file logic can be found the less restrictive LPCXpresso package at:

    /cygdrive/c/nxp/LPCXpresso_4.2.3_292/lpcxpresso/bin

  (The "free" RedSuite version has a download limit of 8K; the "free" LPCXpresso
  version has a download limit of 128K).

  NOTE that the following alias is defined in the setenv.sh file and
  can be used to enter the boot mode with a simpler command:

    alias lpc43xx='${SCRIPT_BIN}/Scripts/bootLPCXpresso.cmd winusb'

  Be default, the setenv.sh scripts uses the LPCXpresso path shown above.
  Once setenv.sh has been sources, then entering boot mode becomes simply:

    $ lpc43xx
    Booting LPC-Link with LPCXpressoWIN.enc
    Press any key to continue . . . 

  Using GDB
  ---------

  The underlying debugger within Red Suite/LPCXpresso is GDB.  That GDB
  used from the command line.  The GDB configuration details for command
  line use are on Code Red Wiki:

    http://support.code-red-tech.com/CodeRedWiki/UsingGDB

  and is also summarized here (see the full Wiki for additional details
  and options).

  The Code Red Debug Driver implements the GDB "remote" protocol to allow
  connection to debug targets.  To start a debug session using GDB, use
  following steps:

    arm-none-eabi-gdb executable.axf                  : Start GDB and name the debug image
    target extended-remote | <debug driver> <options> : Start debug driver, connect to target
    load                                              : Load image and download to target

  The where <debug driver> is crt_emu_lpc18_43_nxp for LPC18xx and LPC43xx.
  Your PATH variable should be set up so that the debug driver executable
  can be found.  For my installation, the driver for the LPC18xx and LPC43xx
  is located at:

    /cygdrive/c/code_red/RedSuite_4.2.3_379/redsuite/bin/crt_emu_lpc18_43_nxp.exe, OR
    /cygdrive/c/nxp/LPCXpresso_4.2.3_292/lpcxpresso/bin/crt_emu_lpc18_43_nxp.exe

  And <options> are:

    -n set information level for the debug driver. n should be 2, 3 or 4.
        2 should be sufficient in most circumstances 

    -p<part> is the target device to connect to and you should use
        <part>=LPC4330.

    -wire=<probe> specifies the debug probe.  For LPCLink on Windows 7 use
        <probe>=winusb.  The 128K free version only supports the LPC-Link
        and RedProbe debug probes. Other JTAG interfaces are supported in
        the full version.

  Thus the correct invocation for the LPC4330 under Windows7 would be:

    target extended-remote | crt_emu_lpc18_43_nxp -2 -pLPC4330 -wire=winusb

  DDD.  This command can be used to start GDB under the graphics front-end
  DDD:

    $ ddd --debugger arm-none-eabi-gdb nuttx &

  NOTE 1:  Don't forget to put the LPCLink in boot mode as described above
  before starting GDB.  So a typical session might look like this:

    $ lpc43xx
    Booting LPC-Link with LPCXpressoWIN.enc
    Press any key to continue . . . 

    $ arm-none-eabi-gdb nuttx
    (gdb) target extended-remote | crt_emu_lpc18_43_nxp -2 -pLPC4330 -wire=winusb
    (gdb) load
    (gdb) r
    (gdb) c

  NOTE 2:  Don't forget to enable CONFIG_DEBUG_SYMBOLS=y in your NuttX
  configuration file when you build NuttX.  That option is necessary to build
  in debugging symbols.

  NOTE 3:  There are few things that NuttX has to do differently if you
  are using a debugger.  Make sure that you also set CONFIG_DEBUG=y.  Nothing
  also is needed and no debug output will be generated; but NuttX will
  use CONFIG_DEBUG=y to mean that a debugger is attached and will deal
  with certain resets and debug controls appropriately.

  So you should have:

    CONFIG_DEBUG=y
    CONFIG_DEBUG_SYMBOLS=y

  NOTE 4: Every time that you control-C out of the command line GDB, you
  leave a copy of the Code Red debugger (crt_emu_lpc18_43_nxp) running.  I
  have found that if you have these old copies of the debugger running, 
  hen strange things can happen when start yet another copy of the
  debugger (I suspect that GDB may be talking with the wrong debugger).

  If you exit GDB with quit (not control-C), it seems to clean-up okay.
  But I have taken to keeping a Process Explorer window open all of the
  time to keep track of how many of these bad processes have been created.

  NOTE 5: There is also a certain function that is causing some problems.
  The very first thing that the start-up logic does is call a function
  called lpc43_softreset() which resets most of the peripherals.  But it
  also causes some crashes... I think because the resets are causing some
  interrupts.

  I put a big delay in the soft reset logic between resetting and clearing
  pending interrupts  and that seems to help some but I am not confident
  that that is a fix.  I think that the real fix might be to just eliminated
  this lpc43_softreset() function if we determine that it is not needed.

  If you step over lpc43_softreset() after loading the coding (using the 'n'
  command), then everything seems work okay.

  Troubleshooting
  ---------------

  This page provides some troubleshooting information that you can use to
  verify that the LPCLink is working correctly:

    http://support.code-red-tech.com/CodeRedWiki/LPCLinkDiagnostics

  Command Line Flash Programming
  ------------------------------

  The LPC18xx/LPC43xx debug driver can also be used to program the LPC43xx
  flash directly from the command line.  The script flash.sh that may be
  found in the configs/lpc4330-xplorer/scripts directory can do that with
  a single command line command.

  Executing from SPIFI
  --------------------

  By default, the configurations here assume that you are executing directly
  from SRAM.

    CONFIG_BOOT_SRAM=y             : Executing in SRAM
    CONFIG_LPC43_CODEREDW=y        : Code Red under Windows

  To execute from SPIFI, you would need to set:

    CONFIG_BOOT_SPIFI=y            : Executing from SPIFI
    CONFIG_DRAM_SIZE=(128*1024)    : SRAM Bank0 size
    CONFIG_DRAM_START=0x10000000   : SRAM Bank0 base address
    CONFIG_SPIFI_OFFSET=(512*1024) : SPIFI file system offset

  To boot the LPC4330-Xplorer from SPIFI the DIP switches should be 1-OFF,
  2-ON, 3-ON, 4-ON (LOW LOW LOW HIGH in Table 19, MSB to LSB).

  If the code in flash hard faults after reset and crt_emu_lpc18_43_nxp
  can't reset the MCU, an alternative is to temporarily change switch 1
  to  ON and press the reset button so it enters UART boot mode. Then
  change it back to OFF and reset to boot again from flash.

  # Use -wire to specify the debug probe in use:
  #   (empty)       Red Probe+
  #   -wire=winusb  LPC-Link on Windows XP
  #   -wire=hid     LPC-Link on Windows Vista/ Windows 7
  # Add -g -4 for verbose output

  crt_emu_lpc18_43_nxp -wire=hid -pLPC4330 -load-base=0x14000000 
    -flash-load-exec=nuttx.bin -flash-driver=LPC1850A_4350A_SPIFI.cfx

  USB DFU Booting
  ---------------

  To be provided.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpc4330-xplorer/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  details PLUS some special instructions that you will need to follow if you
  are building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
================================

  The older, OABI buildroot toolchain is also available.  To use the OABI
  toolchain:

  1. When building the buildroot toolchain, either (1) modify the cortexm3-eabi-defconfig-4.6.3
     configuration to use EABI (using 'make menuconfig'), or (2) use an exising OABI
     configuration such as cortexm3-defconfig-4.3.3

  2. Modify the Make.defs file to use the OABI conventions:

    +CROSSDEV = arm-nuttx-elf-
    +ARCHCPUFLAGS = -mtune=cortex-m3 -march=armv7-m -mfloat-abi=soft
    +NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-gotoff.ld -no-check-sections
    -CROSSDEV = arm-nuttx-eabi-
    -ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -NXFLATLDFLAGS2 = $(NXFLATLDFLAGS1) -T$(TOPDIR)/binfmt/libnxflat/gnu-nxflat-pcrel.ld -no-check-sections

NXFLAT Toolchain
================

  If you are *not* using the NuttX buildroot toolchain and you want to use
  the NXFLAT tools, then you will still have to build a portion of the buildroot
  tools -- just the NXFLAT tools.  The buildroot with the NXFLAT tools can
  be downloaded from the NuttX SourceForge download site
  (https://sourceforge.net/projects/nuttx/files/).
 
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1768/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-nxflat .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly builtNXFLAT binaries.

Serial Console
==============

The LPC4330 Xplorer does not have RS-232 drivers or serial connectors on board.
USART0 and UART1 are available on J8 as follows:

  ------ ------ -----------------------
  SIGNAL J8 PIN   LPC4330FET100 PIN
                  (TFBGA100 package)
  ------ ------ -----------------------
  U0_TXD pin  9  F6  P6_4  U0_TXD=Alt 2
  U0_RXD pin 10  F9  P6_5  U0_RXD=Alt 2
  U1_TXD pin 13  H8  P1_13 U1_TXD=Alt 1
  U1_RXD pin 14  J8  P1_14 U1_RXD=Alt 1
  ------ ------ -----------------------

  GND  is available on J8 pin 1
  5V   is available on J8 pin 2
  VBAT is available on J8 pin 3

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the most NuttX Cortex-M4
ports.  The current LPC43xx port support only one of these options, the "Non-
Lazy Floating Point Register Save".  As a consequence, CONFIG_ARMV7M_CMNVECTOR
must be defined in *all* LPC43xx configuration files.

1. Lazy Floating Point Register Save.

   This is an untested implementation that saves and restores FPU registers
   only on context switches.  This means: (1) floating point registers are
   not stored on each context switch and, hence, possibly better interrupt
   performance.  But, (2) since floating point registers are not saved,
   you cannot use floating point operations within interrupt handlers.

   This logic can be enabled by simply adding the following to your .config
   file:

   CONFIG_ARCH_FPU=y

2. Non-Lazy Floating Point Register Save

   Mike Smith has contributed an extensive re-write of the ARMv7-M exception
   handling logic. This includes verified support for the FPU.  These changes
   have not yet been incorporated into the mainline and are still considered
   experimental.  These FPU logic can be enabled with:

   CONFIG_ARCH_FPU=y
   CONFIG_ARMV7M_CMNVECTOR=y

   You will probably also changes to the ld.script in if this option is selected.
   This should work:

   -ENTRY(_stext)
   +ENTRY(__start)         /* Treat __start as the anchor for dead code stripping */
   +EXTERN(_vectors)       /* Force the vectors to be included in the output */

CFLAGS
------

Only the Atollic toolchain has built-in support for the Cortex-M4 FPU.  You will see
the following lines in each Make.defs file:

  ifeq ($(CONFIG_LPC43_ATOLLIC_LITE),y)
    # Atollic toolchain under Windows
    ...
  ifeq ($(CONFIG_ARCH_FPU),y)
    ARCHCPUFLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard
  else
    ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
  endif
  endif

If you are using a toolchain other than the Atollic toolchain, then to use the FPU
you will also have to modify the CFLAGS to enable compiler support for the ARMv7-M
FPU.  As of this writing, there are not many GCC toolchains that will support the
ARMv7-M FPU.

As a minimum you will need to add CFLAG options to (1) enable hardware floating point
code generation, and to (2) select the FPU implementation.  You might try the same
options as used with the Atollic toolchain in the Make.defs file:

  ARCHCPUFLAGS = -mcpu=cortex-m4 -mthumb -march=armv7e-m -mfpu=fpv4-sp-d16 -mfloat-abi=hard

Configuration Changes
---------------------

Below are all of the configuration changes that I had to make to configs/stm3240g-eval/nsh2
in order to successfully build NuttX using the Atollic toolchain WITH FPU support:

  -CONFIG_ARCH_FPU=n              : Enable FPU support
  +CONFIG_ARCH_FPU=y

  -CONFIG_LPC43_CODESOURCERYW=y   : Disable the CodeSourcery toolchain
  +CONFIG_LPC43_CODESOURCERYW=n

  -CONFIG_LPC43_ATOLLIC_LITE=n    : Enable *one* the Atollic toolchains
   CONFIG_LPC43_ATOLLIC_PRO=n
  -CONFIG_LPC43_ATOLLIC_LITE=y    : The "Lite" version, OR
   CONFIG_LPC43_ATOLLIC_PRO=y     : The "Pro" version (not both)

  -CONFIG_INTELHEX_BINARY=y       : Suppress generation FLASH download formats
  +CONFIG_INTELHEX_BINARY=n       : (Only necessary with the "Lite" version)

  -CONFIG_HAVE_CXX=y              : Suppress generation of C++ code
  +CONFIG_HAVE_CXX=n              : (Only necessary with the "Lite" version)

See the section above on Toolchains, NOTE 2, for explanations for some of
the configuration settings.  Some of the usual settings are just not supported
by the "Lite" version of the Atollic toolchain.

LPC4330-Xplorer Configuration Options
=====================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc43xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC4330=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lpc4330-xplorer (for the LPC4330-Xplorer board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPC4330_XPLORER=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_DRAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x10000000

    CONFIG_ARCH_IRQPRIO - The LPC43xx supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=y

    CONFIG_ARCH_FPU - The LPC43xxx supports a floating point unit (FPU)

       CONFIG_ARCH_FPU=y
 
    CONFIG_BOOT_xxx - The startup code needs to know if the code is running
       from internal FLASH, external FLASH, SPIFI, or SRAM in order to
       initialize properly.  Note that a boot device is not specified for 
       cases where the code is copied into SRAM; those cases are all covered
       by CONFIG_BOOT_SRAM.

       CONFIG_BOOT_SRAM=y      : Running from SRAM             (0x1000:0000)
       CONFIG_BOOT_SPIFI=y     : Running from QuadFLASH        (0x1400:0000)
       CONFIG_BOOT_FLASHA=y    : Running in internal FLASHA    (0x1a00:0000)
       CONFIG_BOOT_FLASHB=y    : Running in internal FLASHA    (0x1b00:0000)
       CONFIG_BOOT_CS0FLASH=y  : Running in external FLASH CS0 (0x1c00:0000)
       CONFIG_BOOT_CS1FLASH=y  : Running in external FLASH CS1 (0x1d00:0000)
       CONFIG_BOOT_CS2FLASH=y  : Running in external FLASH CS2 (0x1e00:0000)
       CONFIG_BOOT_CS3FLASH=y  : Running in external FLASH CS3 (0x1f00:0000)

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

    Individual subsystems can be enabled:

      CONFIG_LPC43_ADC0=y
      CONFIG_LPC43_ADC1=y
      CONFIG_LPC43_ATIMER=y
      CONFIG_LPC43_CAN1=y
      CONFIG_LPC43_CAN2=y
      CONFIG_LPC43_DAC=y
      CONFIG_LPC43_EMC=y
      CONFIG_LPC43_ETHERNET=y
      CONFIG_LPC43_EVNTMNTR=y
      CONFIG_LPC43_GPDMA=y
      CONFIG_LPC43_I2C0=y
      CONFIG_LPC43_I2C1=y
      CONFIG_LPC43_I2S0=y
      CONFIG_LPC43_I2S1=y
      CONFIG_LPC43_LCD=y
      CONFIG_LPC43_MCPWM=y
      CONFIG_LPC43_QEI=y
      CONFIG_LPC43_RIT=y
      CONFIG_LPC43_RTC=y
      CONFIG_LPC43_SCT=y
      CONFIG_LPC43_SDMMC=y
      CONFIG_LPC43_SPI=y
      CONFIG_LPC43_SPIFI=y
      CONFIG_LPC43_SSP0=y
      CONFIG_LPC43_SSP1=y
      CONFIG_LPC43_TMR0=y
      CONFIG_LPC43_TMR1=y
      CONFIG_LPC43_TMR2=y
      CONFIG_LPC43_TMR3=y
      CONFIG_LPC43_USART0=y
      CONFIG_LPC43_UART1=y
      CONFIG_LPC43_USART2=y
      CONFIG_LPC43_USART3=y
      CONFIG_LPC43_USB0=y
      CONFIG_LPC43_USB1=y
      CONFIG_LPC43_USB1_ULPI=y
      CONFIG_LPC43_WWDT=y

  LPC43xx specific U[S]ART device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is the USART0).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

    CONFIG_USARTn_RS485MODE - Support LPC43xx USART0,2,3 RS485 mode
      ioctls (TIOCSRS485 and TIOCGRS485) to enable and disable
      RS-485 mode.
  
  LPC43xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC43_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC43_CAN2 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 = the number of CAN time quanta in segment 2. Default: 7

  LPC43xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC43_ETHERNET.

    CONFIG_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_PHY_AUTONEG - Enable auto-negotion
    CONFIG_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_PHY_FDUPLEX - Select full (vs. half) duplex

    CONFIG_NET_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_NET_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_NET_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_NET_PRIORITY - Ethernet interrupt priority.  The is default is
      the higest priority.
    CONFIG_NET_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG.
    CONFIG_NET_HASH - Enable receipt of near-perfect match frames.
    CONFIG_NET_MULTICAST - Enable receipt of multicast (and unicast) frames.
      Automatically set if CONFIG_NET_IGMP is selected.

  LPC43xx USB Device Configuration

    CONFIG_LPC43_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events. 
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC43_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC43_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC43_USBDEV_DMA
      Enable lpc17xx-specific DMA support
    CONFIG_LPC43_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC43_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC43xx USB Host Configuration

    CONFIG_USBHOST_OHCIRAM_SIZE
      Total size of OHCI RAM (in AHB SRAM Bank 1)
    CONFIG_USBHOST_NEDS
      Number of endpoint descriptors
    CONFIG_USBHOST_NTDS
      Number of transfer descriptors
    CONFIG_USBHOST_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_USBHOST_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_USBHOST_IOBUFSIZE
      Size of one end-user I/O buffer.  This can be zero if the
      application can guarantee that all end-user I/O buffers
      reside in AHB SRAM.

Configurations
==============

Each LPC4330-Xplorer configuration is maintained in a sudirectory and can be selected
as follow:

    cd tools
    ./configure.sh lpc4330-xplorer/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  ostest:
  ------
    This configuration of this directory performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    executing directly from SRAM.

      CONFIG_BOOT_SRAM=y             : Executing in SRAM
      CONFIG_LPC43_CODEREDW=y        : Code Red under Windows

    This configuration directory, performs a simple test of the USB host
    HID keyboard class driver using the test logic in examples/hidkbd.

    If you use the Atollic toolchain, then the FPU test can be enabled in the
    examples/ostest by adding the following your NuttX configuration file:

     -CONFIG_ARCH_FPU=n              : Enable FPU support
     +CONFIG_ARCH_FPU=y

     -CONFIG_LPC43_CODESOURCERYW=y   : Disable the CodeSourcery toolchain
     +CONFIG_LPC43_CODESOURCERYW=n

     -CONFIG_LPC43_ATOLLIC_LITE=n    : Enable *one* the Atollic toolchains
      CONFIG_LPC43_ATOLLIC_PRO=n
     -CONFIG_LPC43_ATOLLIC_LITE=y    : The "Lite" version, OR
      CONFIG_LPC43_ATOLLIC_PRO=n     : The "Pro" version (only one)

     -CONFIG_INTELHEX_BINARY=y       : Suppress generation FLASH download formats
     +CONFIG_INTELHEX_BINARY=n       : (Only necessary with the "Lite" version)

     -CONFIG_HAVE_CXX=y              : Suppress generation of C++ code
     +CONFIG_HAVE_CXX=n              : (Only necessary with the "Lite" version)

     -CONFIG_SCHED_WAITPID=y         : Enable the waitpid() API needed by the FPU test
     +CONFIG_SCHED_WAITPID=n

    The FPU test also needs to know the size of the FPU registers save area in
    bytes (see arch/arm/include/armv7-m/irq_lazyfpu.h):

     +CONFIG_EXAMPLES_OSTEST_FPUSIZE=(4*33)

  nsh:
  ----
    This configuration is the NuttShell (NSH) example at examples/nsh/
    examples/ostest.    By default, this project assumes that you are
    executing directly from SRAM.

      CONFIG_BOOT_SRAM=y             : Executing in SRAM
      CONFIG_LPC43_CODEREDW=y        : Code Red under Windows

    To execute from SPIFI, you would need to set:

      CONFIG_BOOT_SPIFI=y            : Executing from SPIFI
      CONFIG_DRAM_SIZE=(128*1024)    : SRAM Bank0 size
      CONFIG_DRAM_START=0x10000000   : SRAM Bank0 base address
      CONFIG_SPIFI_OFFSET=(512*1024) : SPIFI file system offset

    CONFIG_MM_REGIONS should also be increased if you want to other SRAM banks
    to the memory pool.

    This configuration has some special options that can be used to
    create a block device on the SPIFI FLASH.  NOTE:  CONFIG_LPC43_SPIFI=y
    must also be defined to enable SPIFI setup support:

    SPIFI device geometry:

      CONFIG_SPIFI_OFFSET - Offset the beginning of the block driver this many
        bytes into the device address space.  This offset must be an exact
        multiple of the erase block size (CONFIG_SPIFI_BLKSIZE). Default 0.
      CONFIG_SPIFI_BLKSIZE - The size of one device erase block.  If not defined
        then the driver will try to determine the correct erase block size by
        examining that data returned from spifi_initialize (which sometimes
        seems bad).

    Other SPIFI options

      CONFIG_SPIFI_SECTOR512 - If defined, then the driver will report a more
        FAT friendly 512 byte sector size and will manage the read-modify-write
        operations on the larger erase block.
      CONFIG_SPIFI_READONLY - Define to support only read-only operations.
      CONFIG_SPIFI_LIBRARY - Don't use the LPC43xx ROM routines but, instead,
        use an external library implementation of the SPIFI interface.
      CONFIG_SPIFI_VERIFY - Verify all spifi_program() operations by reading
        from the SPI address space after each write.
      CONFIG_DEBUG_SPIFI_DUMP - Debug option to dump read/write buffers.  You
        probably do not want to enable this unless you want to dig through a
        *lot* of debug output!  Also required CONFIG_DEBUG, CONFIG_DEBUG_VERBOSE,
        and CONFIG_DEBUG_FS,

    In my experience, there were some missing function pointers in the LPC43xx
    SPIFI ROM routines and the SPIFI configuration could only be built with
    CONFIG_SPIFI_LIBRARY=y.  The SPIFI library is proprietary and cannot be
    provided within NuttX open source repository; SPIFI library binaries can
    be found on the lpcware.com website.  In this build sceneario, you must
    also provide the patch to the external SPIFI library be defining the make
    variable EXTRA_LIBS in the top-level Make.defs file.  Good luck!
