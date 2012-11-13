README
^^^^^^

README file for the Microment Eagle100 NuttX port.

References:
^^^^^^^^^^

  Micromint: http://www.micromint.com/
  Luminary:  http://www.luminarymicro.com/

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the Luminary FLASH programming application was used for
  writing to FLASH and this application works only under Windows.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain, or
  3. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery or devkitARM GNU toolchain, you simply need to build the
  system as follows:

     make                         # Will build for the devkitARM toolchain
     make CROSSDEV=arm-eabi-      # Will build for the devkitARM toolchain
     make CROSSDEV=arm-none-eabi- # Will build for the CodeSourcery toolchain
     make CROSSDEV=arm-nuttx-elf- # Will build for the NuttX buildroot toolchain

  Of course, hard coding this CROSS_COMPILE value in Make.defs file will save
  some repetitive typing.

  NOTE: the CodeSourcery and devkitARM toolchains are Windows native toolchains.
  The NuttX buildroot toolchain is a Cygwin toolchain.  There are several limitations
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

       make clean_context; make CROSSDEV=arm-none-eabi-

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

     Support has been added for making dependencies with the CodeSourcery toolchain.
     That support can be enabled by modifying your Make.defs file as follows:

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.  It has been reported to me that the
  devkitARM will require an lower optimization level of -O1.  Currently all of the
  Make.def files have -O2 for devkitARM -- if you are using this toolchain, you may
  need to review these settings.

CodeSourcery on Linux
^^^^^^^^^^^^^^^^^^^^^

  If you select the CodeSourcery toolchain, the make system will assume that you
  are running a Windows version of the toolchain.  If you are running under Linux,
  the the make will probably fail.  The fix is to edit your Make.defs file and
  use something like:

    CROSSDEV = arm-none-eabi-
    WINTOOL = n
    MKDEP = $(TOPDIR)/tools/mkdeps.sh
    ARCHCPUFLAGS = -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    ARCHINCLUDES = -I. -isystem $(TOPDIR)/include
    ARCHXXINCLUDES = -I. -isystem $(TOPDIR)/include -isystem $(TOPDIR)/include/cxx
    ARCHSCRIPT = -T$(TOPDIR)/configs/$(CONFIG_ARCH_BOARD)/scripts/ld.script
    MAXOPTIMIZATION = -O2

  The values for TOPDIR is provided by the make system; the value for CONFIG_ARCH_BOARD
  is provided in your defconfig file.  'ostest' refers to the ostest/ configuration;
  this would be different for other configurations.

  For an example of a CodeSourcery-under-Linux Make.defs file, see
  configs/stm3210e-eval/nsh/Make.defs.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh eagle100/<sub-dir>

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
  details PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^

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

Ethernet-Bootloader
^^^^^^^^^^^^^^^^^^^

  Here are some notes about using the Luminary Ethernet boot-loader built
  into the Eagle-100 board.

  Built-In Application:

  - The board has no fixed IP address but uses DHCP to get an address.
    I used a D-link router; I can use a web browser to surf to the D-link
    web page to get the address assigned by 

  - Then you can use this IP address in your browser to surf to the Eagle-100
    board.  It presents several interesting pages -- the most important is
    the page called "Firmware Update".  That page includes instructions on
    how to download code to the Eagle-100.

  - After you burn the first program, you lose this application.  Then you
    will probably be better off connected directly to the Eagle-100 board
    or through a switch (The router caused problems for me during downloads).

  Using the Ethernet Bootloader:

  - You will need the "LM Flash Programmer application".  You can get that
    program from the Luminary web site.  There is a link on the LM3S6918 page.

  - Is there any documentation for using the bootloader?  Yes and No:  There
    is an application note covering the bootloader on the Luminary site, but
    it is not very informative.  The Eagle100 User's Manual has the best
    information.

  - Are there any special things I have to do in my code, other than setting 
    the origin to 0x0000:2000 (APP_START_ADDRESS)?  No.  The bootloader assumes
    that you have a vector table at that address .  The bootloader does the
    following each time it boots (after you have downloaded the first valid
    application):

    o The bootloader sets the vector table register to the APP_START_ADDRESS,
    o It sets the stack pointer to the address at APP_START_ADDRESS, and then
    o Jumps to the address at APP_START_ADDRESS+4.

  - You can force the bootloader to skip starting the application and stay
    in the update mode.  You will need to do this in order to download a new
    application.  You force the update mode by holding the user button on the
    Eagle-100 board while resetting the board.  The user button is GPIOA, pin 6
    (call FORCED_UPDATE_PIN in the bootloader code).

  - Note 1:  I had to remove my D-Link router from the configuration in order
    to use the LM Flash Programmer (the Bootloader issues BOOTP requests to
    communicate with the LM Flash Programmer, my router was responding to
    these BOOTP requests and hosing the download).  It is safer to connect
    via a switch or via an Ethernet switch.

  - Note 2:  You don't need the router's DHCPD server in the download
    configuration; the Luminary Flash Programmer has the capability of
    temporarily assigning the IP address to the Eagle-100 via BOOTP.

Eagle100-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lm3s

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LM3S6918

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=eagle100 (for the MicroMint Eagle-100 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EAGLE100

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The LM3S6918 supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=y

    CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
       have LEDs

    CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
       stack. If defined, this symbol is the size of the interrupt
        stack in bytes.  If not defined, the user task stacks will be
      used during interrupt handling.

    CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

    CONFIG_ARCH_BOOTLOADER - Configure to use the MicroMint Eagle-100
       Ethernet bootloader.

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOH and GPIOJ must be disabled because they do not exist on the LM3S6918.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_LM3S_DISABLE_GPIOA_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOB_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOC_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOD_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOE_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOF_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOG_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOH_IRQS=y
    CONFIG_LM3S_DISABLE_GPIOJ_IRQS=y
 
  LM3S6818 specific device driver settings

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

    CONFIG_SSI0_DISABLE - Select to disable support for SSI0
    CONFIG_SSI1_DISABLE - Select to disable support for SSI1
    CONFIG_SSI_POLLWAIT - Select to disable interrupt driven SSI support.
      Poll-waiting is recommended if the interrupt rate would be to
      high in the interrupt driven case.
    CONFIG_SSI_TXLIMIT - Write this many words to the Tx FIFO before
      emptying the Rx FIFO.  If the SPI frequency is high and this
      value is large, then larger values of this setting may cause
      Rx FIFO overrun errors.  Default: half of the Tx FIFO size (4).

    CONFIG_LM3S_ETHERNET - This must be set (along with CONFIG_NET)
      to build the LM3S Ethernet driver
    CONFIG_LM3S_ETHLEDS - Enable to use Ethernet LEDs on the board.
    CONFIG_LM3S_BOARDMAC - If the board-specific logic can provide
      a MAC address (via lm3s_ethernetmac()), then this should be selected.
    CONFIG_LM3S_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_LM3S_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_LM3S_ETHNOPAD - Set to suppress Tx padding
    CONFIG_LM3S_MULTICAST - Set to enable multicast frames
    CONFIG_LM3S_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_LM3S_BADCRC - Set to enable bad CRC rejection.
    CONFIG_LM3S_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Each Eagle-100 configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh eagle100/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nettest:
    This configuration directory may be used to enable networking using the
    LM3S6918's Ethernet controller. It uses examples/nettest to excercise the
    TCP/IP network.

  httpd:
    This builds the uIP web server example using the examples/uip application
    (for execution from FLASH). See examples/README.txt for information
    about ostest.

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interfaces (the telnet
    interface should also be functional, but is not enabled in this
    configuration).

  nxflat:
    This builds the NXFLAT example at apps/examples/nxfalt.

    NOTE: See note above with regard to the EABI/OABI buildroot
    toolchains.  This example can only be built using the older
    OABI toolchain.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

  thttpd:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTE: See note above with regard to the EABI/OABI buildroot
    toolchains.  This example can only be built using the older
    OABI toolchain.

By default, all of these examples are built to be used with the Luminary
Ethernet Bootloader (you can change the ld.script file in any of these
sub-directories to change that configuration).


