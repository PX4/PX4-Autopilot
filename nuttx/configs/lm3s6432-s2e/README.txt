README
^^^^^^

README for NuttX port to the Stellaris RDK-S2E Reference Design Kit and
the MDL-S2E Ethernet to Serial module.

Contents
^^^^^^^^

  Stellaris RDK-S2E Reference Design Kit
  Development Environment
  GNU Toolchain Options
  IDEs
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLFAT Toolchain
  Stellaris MDL-S2E Reference Design Configuration Options
  Configurations

Stellaris RDK-S2E Reference Design Kit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Stellaris RDK-S2E Reference Design Kit includes the following features:

 o  MDL-S2E Ethernet to serial module
 o  LM3S6432 in a 10 x 10 mm BGA package for reduced board size
 o  10/100 Mbit Ethernet port
 o  Auto MDI/MDIX cross-over correction
 o  Traffic and link indicators Serial ports
 o  UART ports include RTS/CTS for flow control
 o  UART0 has RS232 levels, transceiver runs at up to 230.4 Kbaud
 o  UART1 has CMOS/TTL levels, can run at 1.0 Mbaud

Features of the LM3S6432 Microcontroller

 o  32-bit RISC performance using ARM® Cortex™-M3 v7M architecture
    - 50-MHz operation
    - Hardware-division and single-cycle-multiplication
    - Integrated Nested Vectored Interrupt Controller (NVIC)
    - 42 interrupt channels with eight priority levels
 o  96 KB single-cycle flash
 o  32 KB single-cycle SRAM
 o  Three general-purpose 32-bit timers
 o  Integrated Ethernet MAC and PHY
 o  Two fully programmable 16C550-type UARTs
 o  Three 10-bit channels (inputs) when used as single-ended inputs
 o  Two independent integrated analog comparators
 o  One I2C module
 o  One PWM generator block
    – One 16-bit counter
    – Two comparators
    – Produces two independent PWM signals
    – One dead-band generator
 o  0 to 43 GPIOs, depending on user configuration
 o  On-chip low drop-out (LDO) voltage regulator

GPIO Usage

PIN SIGNAL            Function
--- ----------------- ---------------------------------------
 L3 PA0/U0RX          UART0 receive
 M3 PA1/U0TX          UART0 transmit
E12 PB0/U0CTS         UART0 CTS
D12 PB1/U0RTS         UART0 RTS
 L5 PA4/SPIRX         SPI receive (pin hardwired to U1RX)
 M5 PA5/SPITX         SPI transmit (pin hardwired to U1TX)
 H2 PD2/U1RX          UART1 receive
 H1 PD3/U1TX          UART1 transmit
 L4 PA3/U1CTS/SPICLK  UART1 CTS or SPI clock
 M4 PA2/U1RTS/SPISEL  UART1 RTS or SPI slave select
J11 PF0/LED1          Ethernet LED1 (green)
J12 PF1/LED0          Ethernet LED0 (yellow)
C11 PB2               Transciever #INVALID
C12 PB3               Transciever #ENABLE
 A6 PB4               Transciever ON
 B7 PB5               Transciever #OFF

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux, Mac OS X or Cygwin on Windows can be used for the development 
  environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using GCC on
  Mac OS X.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. The NuttX buildroot Toolchain (see below).

  To use a specific toolchain, you simply need to add one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_LM_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_LM_CODESOURCERYL=y   : CodeSourcery under Linux or on Mac OS X.
    CONFIG_LM_DEVKITARM=y       : devkitARM under Windows
    CONFIG_LM_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_LM_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows) and devkitARM are Windows native toolchains.
  The CodeSourcey (for Linux) and NuttX buildroot toolchains are Cygwin and/or Linux
  native toolchains. There are several limitations to using a Windows based
  toolchain in a Cygwin environment.  The three biggest are:

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

IDEs
^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.
  
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
  3) Set up include pathes:  You will need include/, arch/arm/src/lm,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lm/lm_vectors.S.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  This port was tested with tools built using summon-arm-toolchain; available
  from https://github.com/esden/summon-arm-toolchain, however the buildroot
  instructions should apply for other platforms.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lm3s6432-s2e/<sub-dir>

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

Stellaris MDL-S2E Reference Design Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lm

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LM3S6432

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lm3s6432-s2e (for the Stellaris MDL-S2E Reference Design)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LM3S6432S2E

    CONFIG_ARCH_LOOPSPERMSEC - As supplied, calibrated for correct operation
       of delay loops assuming 50MHz CPU frequency.

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

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOH and GPIOJ must be disabled because they do not exist on the LM3S6432.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint - GPIOs C-G are not pinned out on the MDL-S2E board.

    CONFIG_LM_DISABLE_GPIOA_IRQS=n
    CONFIG_LM_DISABLE_GPIOB_IRQS=n
    CONFIG_LM_DISABLE_GPIOC_IRQS=y
    CONFIG_LM_DISABLE_GPIOD_IRQS=y
    CONFIG_LM_DISABLE_GPIOE_IRQS=y
    CONFIG_LM_DISABLE_GPIOF_IRQS=y
    CONFIG_LM_DISABLE_GPIOG_IRQS=y
    CONFIG_LM_DISABLE_GPIOH_IRQS=y
    CONFIG_LM_DISABLE_GPIOJ_IRQS=y
 
  LM3S6432 specific device driver settings

    CONFIG_UARTn_DISABLE
       The TX and RX pins for UART1 share I/O pins with the TX and RX pins
       for SSI0.  To avoid conflicts, only one of SSI0 and UART1 should
       be enabled in a configuration.
    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
       console and ttys0 (default is UART1).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_UARTn_2STOP - Two stop bits

    CONFIG_SSI0_DISABLE - Select to disable support for SSI0
      The TX and RX pins for SSI0 share I/O pins with the TX and RX pins
      for UART1.  To avoid conflicts, only one of SSI0 and UART1 should
      be enabled in a configuration.
    CONFIG_SSI1_DISABLE - Select to disable support for SSI1
      Note that the LM3S6432 only has one SSI, so SSI1 should always be
      disabled.
    CONFIG_SSI_POLLWAIT - Select to disable interrupt driven SSI support.
      Poll-waiting is recommended if the interrupt rate would be to
      high in the interrupt driven case.
    CONFIG_SSI_TXLIMIT - Write this many words to the Tx FIFO before
      emptying the Rx FIFO.  If the SPI frequency is high and this
      value is large, then larger values of this setting may cause
      Rx FIFO overrun errors.  Default: half of the Tx FIFO size (4).

    CONFIG_LM_ETHERNET - This must be set (along with CONFIG_NET)
      to build the LM3S Ethernet driver
    CONFIG_LM_ETHLEDS - Enable to use Ethernet LEDs on the board.
    CONFIG_LM_BOARDMAC - This should be set in order to use the
      MAC address configured in the flash USER registers.
    CONFIG_LM_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_LM_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_LM_ETHNOPAD - Set to suppress Tx padding
    CONFIG_LM_MULTICAST - Set to enable multicast frames
    CONFIG_LM_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_LM_BADCRC - Set to enable bad CRC rejection.
    CONFIG_LM_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Each Stellaris MDL-S2E Reference Design configuration is maintained in a
sudirectory and can be selected as follow:

    cd tools
    ./configure.sh lm3s6432-s2e/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTE: As it is configured now, you MUST have a network connected.
    Otherwise, the NSH prompt will not come up because the Ethernet
    driver is waiting for the network to come up.  That is probably
    a bug in the Ethernet driver behavior!

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

    NOTE: as the configuration stands, ostest will hang during the 
    semaphore test.  This has not been debugged.
