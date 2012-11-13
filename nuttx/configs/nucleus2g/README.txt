README
^^^^^^

README for NuttX port to the Nucleus 2G LPC1768 board from 2G Engineering
(http://www.2g-eng.com/)

Contents
^^^^^^^^

  2G-Engineering Nucleus Board
  Development Environment
  GNU Toolchain Options
  IDEs
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  LEDs
  Nucleus 2G Configuration Options
  Configurations

Nucleus 2G Board
^^^^^^^^^^^^^^^^

GPIO Usage

  P0[0]/RD1/TXD3/SDA1               P0[0]/CAN_RX1
  P0[1]/TD1/RXD3/SCL                P0[1]/CAN_TX1
  P0[2]/TXD0/AD0[7]                 TX0
  P0[3]/RXD0/AD0[6]                 RX0
  P0[4]                             P0[4]/CAN1_STB
  P0[5]                             P0[5]/CAN2_STB
  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     GPI/O_CS1
  P0[7]/I2STX_CLK/SCK1/MAT2[1]      SCLK1
  P0[8]/I2STX_WS/MISO1/MAT2[2]      MISO1
  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     MOSI1
  P0[10]                            P0[10]/CAN1_TERM
  P0[11]                            P0[11]/CAN2_TERM
  P0[15]/TXD1/SCK0/SCK              MMC_CLK
  P0[16]/RXD1/SSEL0/SSEL            MMC_CD
  P0[17]/CTS1/MISO0/MISO            MMC_DATA0
  P0[18]/DCD1/MOSI0/MOSI            MMC_MISO
  P0[19]/DSR1/SDA1                  GPI/O_CS2
  P0[20]/DTR1/SCL1                  GPI/O_CS3
  P0[21]/RI1/MCIPWR/RD1             P0[21]
  P0[22]/RTS1/TD1                   P0[22]
  P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]   AD0
  P0[24]/AD0[1]/I2SRX_WS/CAP3[1]    AD1
  P0[25]/AD0[2]/I2SRX_SDA/TXD3      AD2
  P0[26]/AD0[3]/AOUT/RXD3           AD3
  P0[27]/SDA0/USB_SDA               SDA
  P0[28]/SCL0                       SCL
  P0[29]/USB_D+                     USB+
  P0[30]/USB_D-                     USB-

  P1[0] - P1[17]                    Not connected
  P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] USB_LINK
  P1[19]-P[29]                      P[19]-P[29]
  P1[30]/VBUS/AD0[4]                USB_+5
  P1[31]/SCK1/AD0[5]                AD5

  P2[0]                             P2[0]/LED1_A
  P2[1]                             P2[1]/LED1_B
  P2[2]                             P2[2]/LED2_A
  P2[3]                             P2[3]/LED2_B
  P2[4]                             P2[4]
  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   232_POWERAVE
  P2[6]/PCAP1[0]/RI1/TRACECLK       232_VALID
  P2[7]/RD2/RTS1                    P2[7]/CAN_RX2
  P2[8]/TD2/TXD2                    P2[8]/CAN_TX2
  P2[9]/USB_CONNECT/RXD2            USB_CONNECT
  P2[10]/EINT0/NMI                  BOOTLOADER
  P2[11]/EINT1/I2STX_CLK            HEARTBEAT
  P2[12]/EINT2/I2STX_WS             EXTRA_LED
  P2[13]/EINT3/I2STX_SDA            5V_ENABLE

  P3[25]-P3[26]                     Not connected

  P4[28]-P4[29]                     P4[28]-P4[29]

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
  the CodeSourcery or devkitARM toolchain, you simply need add one of the
  following configuration options to your .config (or defconfig) file:

    CONFIG_LPC17_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_LPC17_CODESOURCERYL=y   : CodeSourcery under Linux
    CONFIG_LPC17_DEVKITARM=y       : devkitARM under Windows
    CONFIG_LPC17_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_LPC17_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows)and devkitARM are Windows native toolchains.
  The CodeSourcey (for Linux) and NuttX buildroot toolchains are Cygwin and/or
  Linux native toolchains. There are several limitations to using a Windows based
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
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc17xx,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc17x/lpc17_vectors.S.

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
     ./configure.sh nucleus2g/<sub-dir>

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

LEDs
^^^^

  If CONFIG_ARCH_LEDS is defined, then support for the Nucleus-2G LEDs will be
  included in the build.  See:

  - configs/nucleus2g/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/nucleus2g/src/nucleus2g_internal.h - GPIO settings for the LEDs.

  - configs/nucleus2g/src/up_leds.c - LED control logic.

  The Nucleus2G has 3 LEDs... two on the Babel CAN board and a "heartbeat" LED."
  The LEDs on the Babel CAN board are capabl of OFF/GREEN/RED/AMBER status.
  In normal usage, the two LEDs on the Babel CAN board would show CAN status, but if
  CONFIG_ARCH_LEDS is defined, these LEDs will be controlled as follows for NuttX
  debug functionality (where NC means "No Change").

  During the boot phases.  LED1 and LED2 will show boot status.

                                          /* LED1   LED2   HEARTBEAT */
    #define LED_STARTED                0  /* OFF    OFF    OFF */
    #define LED_HEAPALLOCATE           1  /* GREEN  OFF    OFF */
    #define LED_IRQSENABLED            2  /* OFF    GREEN  OFF */
    #define LED_STACKCREATED           3  /* OFF    OFF    OFF */

    #define LED_INIRQ                  4  /*  NC     NC    ON  (momentary) */
    #define LED_SIGNAL                 5  /*  NC     NC    ON  (momentary) */
    #define LED_ASSERTION              6  /*  NC     NC    ON  (momentary) */
    #define LED_PANIC                  7  /*  NC     NC    ON  (2Hz flashing) */
    #undef  LED_IDLE                      /* Sleep mode indication not supported */

  After the system is booted, this logic will no longer use LEDs 1 and 2.  They
  are then available for use the application software using lpc17_led1() and
  lpc17_led2():

    enum lpc17_ledstate_e
    {
      LPC17_LEDSTATE_OFF   = 0,
      LPC17_LEDSTATE_GREEN = 1,
      LPC17_LEDSTATE_RED   = 2,
      LPC17_LEDSTATE_AMBER = (LPC17_LEDSTATE_GREEN|LPC17_LEDSTATE_RED),
    };

    EXTERN void lpc17_led1(enum lpc17_ledstate_e state);
    EXTERN void lpc17_led2(enum lpc17_ledstate_e state);

  The heartbeat LED is illuminated during all interrupt and signal procressing.
  Normally, it will glow dimly to inicate that the LPC17xx is taking interrupts.
  On an assertion PANIC, it will flash at 2Hz.

Nucleus 2G Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc17xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC1768=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=nucleus2g (for the Nucleus 2G)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_NUCLEUS2G=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_DRAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x10000000

    CONFIG_ARCH_IRQPRIO - The LPC17xx supports interrupt prioritization

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

    Individual subsystems can be enabled:
      CONFIG_LPC17_MAINOSC=y
      CONFIG_LPC17_PLL0=y
      CONFIG_LPC17_PLL1=n
      CONFIG_LPC17_ETHERNET=n
      CONFIG_LPC17_USBHOST=n
      CONFIG_LPC17_USBOTG=n
      CONFIG_LPC17_USBDEV=n
      CONFIG_LPC17_UART0=y
      CONFIG_LPC17_UART1=n
      CONFIG_LPC17_UART2=n
      CONFIG_LPC17_UART3=n
      CONFIG_LPC17_CAN1=n
      CONFIG_LPC17_CAN2=n
      CONFIG_LPC17_SPI=n
      CONFIG_LPC17_SSP0=n
      CONFIG_LPC17_SSP1=n
      CONFIG_LPC17_I2C0=n
      CONFIG_LPC17_I2C1=n
      CONFIG_LPC17_I2S=n
      CONFIG_LPC17_TMR0=n
      CONFIG_LPC17_TMR1=n
      CONFIG_LPC17_TMR2=n
      CONFIG_LPC17_TMR3=n
      CONFIG_LPC17_RIT=n
      CONFIG_LPC17_PWM=n
      CONFIG_LPC17_MCPWM=n
      CONFIG_LPC17_QEI=n
      CONFIG_LPC17_RTC=n
      CONFIG_LPC17_WDT=n
      CONFIG_LPC17_ADC=n
      CONFIG_LPC17_DAC=n
      CONFIG_LPC17_GPDMA=n
      CONFIG_LPC17_FLASH=n

  LPC17xx specific device driver settings

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

  LPC17xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_CAN2 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 = the number of CAN time quanta in segment 2. Default: 7

  LPC17xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC17_ETHERNET.

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

  LPC17xx USB Device Configuration

    CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events. 
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC17_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC17_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC17_USBDEV_DMA
      Enable lpc17xx-specific DMA support
    CONFIG_LPC17_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC17_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC17xx USB Host Configuration (the Nucleus2G does not support USB Host)

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
^^^^^^^^^^^^^^

Each Nucleus 2G configuration is maintained in a
sudirectory and can be selected as follow:

    cd tools
    ./configure.sh nucleus2g/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial  NSH interfaces.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

  usbserial:
    This configuration directory exercises the USB serial class
    driver at examples/usbserial.  See examples/README.txt for
    more information.

  usbstorage:
    This configuration directory exercises the USB mass storage
    class driver at examples/usbstorage.  See examples/README.txt for
    more information.

