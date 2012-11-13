README
^^^^^^

README for NuttX port to the Stellaris EKK-LM3S9B96 Evaluation Kit

Contents
^^^^^^^^

  Stellaris EKK-LM3S9B96 Evaluation Kit
  Development Environment
  GNU Toolchain Options
  IDEs
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  Stellaris EKK-LM3S9B96 Evaluation Kit Configuration Options
  Configurations

Stellaris EKK-LM3S9B96 Evaluation Kit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The EKK-LM3S9B96 evaluation kit provides the following features:

 o LM3S9B96 high-performance Stellaris microcontroller and large memory
    – 32-bit ARM® Cortex™-M3 core
    – 256 KB single-cycle Flash memory, 96 KB single-cycle SRAM, 23.7 KB single-cycle ROM
 o Ethernet 10/100 port with two LED indicators
 o USB 2.0 Full-Speed OTG port
 o SAFERTOS™ operating system in microcontroller ROM
 o Virtual serial communications port capability
 o Oversized board pads for GPIO access
 o User pushbutton and LED
 o Detachable ICDI board can be used for debugging other Luminary Micro boards
 o Easy to customize

Features of the LM3S9B96 Microcontroller

 o  ARM® Cortex™-M3  architecture
    – 80-MHz operation
    – ARM Cortex SysTick Timer
    – Integrated Nested Vectored Interrupt Controller (NVIC)
 o  External Peripheral Interface (EPI)
 o  256 KB single-cycle flash
 o  96 KB single-cycle SRAM
 o  Four general-purpose 32-bit timers
 o  Integrated Ethernet MAC and PHY
 o  Three fully programmable 16C550-type UARTs
 o  Two 10-bit channels (inputs) when used as single-ended inputs
 o  Three independent integrated analog comparators
 o  Two CAN modules
 o  Two I2C modules
 o  Two SSI modules
 o  Two Watchdog Timers (32-bit)
 o  Three PWM generator blocks
    – One 16-bit counter
    – Two comparators
    – Produces eight independent PWM signals
    – One dead-band generator
 o  Two QEI modules with position integrator for tracking encoder position
 o  Up to 65 GPIOs, depending on user configuration
 o  On-chip low drop-out (LDO) voltage regulator

GPIO Usage

PIN SIGNAL      EVB Function
--- ----------- ---------------------------------------
 26 PA0/U0RX      Virtual COM port receive
 27 PA1/U0TX      Virtual COM port transmit
 66 PB0/USB0ID    USBID signal from the USB-On-the-Go
 67 PB1/USB0VBUS  USB VBUS input signal from USB-OTG
 92 PB4/GPIO      User pushbutton SW2.
 80 PC0/TCK/SWCLK JTAG or SWD clock input
 79 PC1/TMS/SWDIO JTAG TMS input or SWD bidirectional signal SWDIO
 78 PC2/TDI       JTAG TDI signal input
 77 PC3/TDO/SWO   JTAG TDO output or SWD trace signal SWO output.
 10 PD0/GPIO      User LED
 60 PF2/LED1      Ethernet LED1 (yellow)
 59 PF3/LED0      Ethernet LED0 (green)
 83 PH3/USB0EPEN  USB-OTG power switch
 76 PH4/USB0PFLT  Overcurrent input status from USB-OTG power switch

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

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

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
  3) Set up include pathes:  You will need include/, arch/arm/src/lm3s,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lm3s/lm3s_vectors.S.

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
     ./configure.sh ekk-lm3s9b96/<sub-dir>

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

Stellaris EKK-LM3S9B96 Evaluation Kit Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

       CONFIG_ARCH_CHIP_LM3S9B96

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=ekk-lm3s9b96 (for the Stellaris EKK-LM3S9b96 Evaluation Kit)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_EKKLM3S9B96

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00018000 (96Kb)

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

Each Stellaris EKK-LM3S9b96 Evaluation Kit configuration is maintained in a
sudirectory and can be selected as follow:

    cd tools
    ./configure.sh ekk-lm3s9b96/<subdir>
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


