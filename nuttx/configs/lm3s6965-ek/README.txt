README
^^^^^^

README for NuttX port to the Stellaris LMS36965 Evaluation Kit

Contents
^^^^^^^^

  Stellaris LMS36965 Evaluation Kit
  Development Environment
  GNU Toolchain Options
  IDEs
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  USB Device Controller Functions
  OLED
  Stellaris LM3S6965 Evaluation Kit Configuration Options
  Configurations

Stellaris LMS36965 Evaluation Kit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The Stellaris LM3S6965 Evaluation Board includes the following features:

 o  Stellaris LM3S6965 microcontroller with fully-integrated 10/100 embedded
    Ethernet controller
 o  Simple setup; USB cable provides serial communication, debugging, and
    power
 o  OLED graphics display with 128 x 96 pixel resolution
 o  User LED, navigation switches, and select pushbuttons
 o  Magnetic speaker
 o  LM3S6965 I/O available on labeled break-out pads
 o  Standard ARM® 20-pin JTAG debug connector with input and output modes
 o  USB interface for debugging and power supply
 o  MicroSD card slot

Features of the LM3S6965 Microcontroller

 o  32-bit RISC performance using ARM® Cortex™-M3 v7M architecture
    – 50-MHz operation
    – Hardware-division and single-cycle-multiplication
    – Integrated Nested Vectored Interrupt Controller (NVIC)
    – 42 interrupt channels with eight priority levels
 o  256 KB single-cycle flash
 o  64 KB single-cycle SRAM
 o  Four general-purpose 32-bit timers
 o  Integrated Ethernet MAC and PHY
 o  Three fully programmable 16C550-type UARTs
 o  Four 10-bit channels (inputs) when used as single-ended inputs
 o  Two independent integrated analog comparators
 o  Two I2C modules
 o  Three PWM generator blocks
    – One 16-bit counter
    – Two comparators
    – Produces two independent PWM signals
    – One dead-band generator
 o  Two QEI modules with position integrator for tracking encoder position
 o  0 to 42 GPIOs, depending on user configuration
 o  On-chip low drop-out (LDO) voltage regulator

GPIO Usage

PIN SIGNAL      EVB Function
--- ----------- ---------------------------------------
 26 PA0/U0RX    Virtual COM port receive
 27 PA1/U0TX    Virtual COM port transmit
 10 PD0/IDX0    SD card chip select
 11 PD1/PWM1    Sound
 30 PA4/SSI0RX  SD card data out
 31 PA5/SSI0TX  SD card and OLED display data in
 28 PA2/SSI0CLK SD card and OLED display clock
 22 PC7/PHB0    OLED display data/control select
 29 PA3/SSI0FSS OLED display chip select
 73 PE1/PWM5    Down switch
 74 PE2/PHB1    Left switch
 72 PE0/PWM4    Up switch
 75 PE3/PHA1    Right switch
 61 PF1/IDX1    Select switch
 47 PF0/PWM0    User LED
 23 PC6/CCP3    Enable +15 V

OLED
^^^^

  The Evaluation Kit includes an OLED graphics display. Features:

  - RiT P14201 series display
  - 128 columns by 96 rows
  - 4-bit, 16-level gray scale.
  - High-contrast (typ. 500:1)
  - Excellent brightness (120 cd/m2)
  - Fast 10 us response.

  The OLED display has a built-in controller IC with synchronous serial and
  parallel interfaces (SSD1329). Synchronous serial (SSI) is used on the EVB.
  The SSI port is shared with the microSD card slot.

  - PC7: OLED display data/control select (D/Cn)
  - PA3: OLED display chip select (CSn)

  NOTE:  Newer versions of the LM3S6965 Evaluation Kit has an OSAM 128x64x4 OLED
  display.  Some tweaks to drivers/lcd/p14201.c would be required to support that
  LCD.

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery or devkitARM, you simply need to add one of the following
  configuration options to your .config (or defconfig) file:

    CONFIG_LM3S_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_LM3S_CODESOURCERYL=y   : CodeSourcery under Linux
    CONFIG_LM3S_DEVKITARM=y       : devkitARM under Windows
    CONFIG_LM3S_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_LM3S_BUILDROOT, then you may also have to modify
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

  NOTE 3: I recently (i.e., late 2011) tried building with the CodeSourcery Windows
  toolchain.  The code worked but required 40 seconds to boot (or even until the
  status LED illuminates)!!  Know idea why. With the buildroot tools, boot time is
  a couple of seconds.

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

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lm3s6965-ek/<sub-dir>

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

USB Device Controller Functions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Device Overview

    An FT2232 device from Future Technology Devices International Ltd manages
    USB-to-serial conversion. The FT2232 is factory configured by Luminary
    Micro to implement a JTAG/SWD port (synchronous serial) on channel A and
    a Virtual COM Port (VCP) on channel B. This feature allows two simultaneous
    communications links between the host computer and the target device using
    a single USB cable. Separate Windows drivers for each function are provided
    on the Documentation and Software CD.

  Debugging with JTAG/SWD

    The FT2232 USB device performs JTAG/SWD serial operations under the control 
    of the debugger or the Luminary Flash Programmer.  It also operate as an
    In-Circuit Debugger Interface (ICDI), allowing debugging of any external
    target board.  Debugging modes:

    MODE DEBUG FUNCTION       USE                         SELECTED BY
    1    Internal ICDI        Debug on-board LM3S6965     Default Mode
                              microcontroller over USB
                              interface.
    2    ICDI out to JTAG/SWD The EVB is used as a USB    Connecting to an external
         header               to SWD/JTAG interface to    target and starting debug
                              an external target.         software. The red Debug Out
                                                          LED will be ON.
    3    In from JTAG/SWD     For users who prefer an     Connecting an external
         header               external debug interface    debugger to the JTAG/SWD
                              (ULINK, JLINK, etc.) with   header.
                              the EVB.

  Virtual COM Port

    The Virtual COM Port (VCP) allows Windows applications (such as HyperTerminal)
    to communicate with UART0 on the LM3S6965 over USB. Once the FT2232 VCP
    driver is installed, Windows assigns a COM port number to the VCP channel.

Stellaris LM3S6965 Evaluation Kit Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

       CONFIG_ARCH_CHIP_LM3S6965

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lm3s6965-ek (for the Stellaris LM3S6965 Evaluation Kit)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LM3S6965EK

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

    CONFIG_ARCH_LEDS -  Use LEDs to show state. Unique to board architecture.

    CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
       cause a 100 second delay during boot-up.  This 100 second delay
       serves no purpose other than it allows you to calibratre
       CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
       the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
       the delay actually is 100 seconds.

  There are configurations for disabling support for interrupts GPIO ports.
  GPIOJ must be disabled because it does not exist on the LM3S6918.
  Additional interrupt support can be disabled if desired to reduce memory
  footprint.

    CONFIG_LM3S_DISABLE_GPIOA_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOB_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOC_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOD_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOE_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOF_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOG_IRQS=n
    CONFIG_LM3S_DISABLE_GPIOH_IRQS=n
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
      a MAC address (via lm_ethernetmac()), then this should be selected.
    CONFIG_LM3S_ETHHDUPLEX - Set to force half duplex operation
    CONFIG_LM3S_ETHNOAUTOCRC - Set to suppress auto-CRC generation
    CONFIG_LM3S_ETHNOPAD - Set to suppress Tx padding
    CONFIG_LM3S_MULTICAST - Set to enable multicast frames
    CONFIG_LM3S_PROMISCUOUS - Set to enable promiscuous mode
    CONFIG_LM3S_BADCRC - Set to enable bad CRC rejection.
    CONFIG_LM3S_DUMPPACKET - Dump each packet received/sent to the console.

Configurations
^^^^^^^^^^^^^^

Each Stellaris LM3S6965 Evaluation Kit configuration is maintained in a
sudirectory and can be selected as follow:

    cd tools
    ./configure.sh lm3s6965-ek/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables both the serial and telnetd NSH interfaces.

    NOTES:
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary

    3. As it is configured now, you MUST have a network connected.
       Otherwise, the NSH prompt will not come up because the Ethernet
       driver is waiting for the network to come up.  That is probably
       a bug in the Ethernet driver behavior!

    4. Network File System (NFS) support can be added by setting the
      following in your configuration file:

      CONFIG_NFS=y

  nx:
    And example using the NuttX graphics system (NX).  This example
    uses the P14201 OLED driver.

    NOTES:
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

    NOTES:
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y                 : Linux (Cygwin under Windows okay too).
       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y                 : Output formats: ELF and raw binary
