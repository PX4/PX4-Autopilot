README.txt
==========

This is the README file for the port of NuttX to the Freescale Kinetis
TWR-K60N512.  Refer to the Freescale web site for further information
about this part:
http://www.freescale.com/webapp/sps/site/prod_summary.jsp?code=TWR-K60N512-KIT

The TWR-K60N51 includes with the FreeScale Tower System which provides (among
other things) a simple UART connection.

Contents
========

  o Kinetis TWR-K60N512 Features
  o Kinetis TWR-K60N512 Pin Configuration
    - On-Board Connections
    - Connections via the General Purpose Tower Plug-in (TWRPI) Socket
    - Connections via the Tower Primary Connector Side A
    - Connections via the Tower Primary Connector Side B
    - TWR-SER Serial Board Connection
  o LEDs
  o Development Environment
  o GNU Toolchain Options
  o IDEs
  o NuttX EABI "buildroot" Toolchain
  o NuttX OABI "buildroot" Toolchain
  o NXFLAT Toolchain
  
Kinetis TWR-K60N512 Features:
=============================

  o K60N512 in 144 MAPBGA, K60N512VMD100
  o Capacitive Touch Pads
  o Integrated, Open-Source JTAG
  o SD Card Slot
  o MMA7660 3-axis accelerometer
  o Tower Plug-In (TWRPI) Socket for expansion (sensors, etc.)
  o Touch TWRPI Socket adds support for various capacitive touch boards
    (e.g. keypads, rotary dials, sliders, etc.)
  o Tower connectivity for access to USB, Ethernet, RS232/RS485, CAN, SPI,
    I²C, Flexbus, etc.
  o Plus: Potentiometer, 4 LEDs, 2 pushbuttons, infrared port

Kinetis TWR-K60N512 Pin Configuration
=====================================

On-Board Connections
-------------------- ------------------------- -------- -------------------
FEATURE              CONNECTION                PORT/PIN PIN FUNCTION
-------------------- ------------------------- -------- -------------------
OSJTAG USB-to-serial OSJTAG Bridge RX Data     PTE9     UART5_RX
Bridge               OSJTAG Bridge TX Data     PTE8     UART5_TX
SD Card Slot         SD Clock                  PTE2     SDHC0_DCLK
                     SD Command                PTE3     SDHC0_CMD
                     SD Data0                  PTE1     SDHC0_D0
                     SD Data1                  PTE0     SDHC0_D1
                     SD Data2                  PTE5     SDHC0_D2
                     SD Data3                  PTE4     SDHC0_D3
                     SD Card Detect            PTE28    PTE28
                     SD Write Protect          PTE27    PTE27
Infrared Port        IR Transmit               PTD7     CMT_IRO
                     IR Receive                PTC6     CMP0_IN0
Pushbuttons          SW1 (IRQ0)                PTA19    PTA19
                     SW2 (IRQ1)                PTE26    PTE26
                     SW3 (RESET)               RESET_b  RESET_b
Touch Pads           E1 / Touch                PTA4     TSI0_CH5
                     E2 / Touch                PTB3     TSI0_CH8
                     E3 / Touch                PTB2     TSI0_CH7
                     E4 / Touch                PTB16    TSI0_CH9
LEDs                 E1 / Orange LED           PTA11    PTA11
                     E2 / Yellow LED           PTA28    PTA28
                     E3 / Green LED            PTA29    PTA29
                     E4 / Blue LED             PTA10    PTA10
Potentiometer        Potentiometer (R71)       ?        ADC1_DM1
Accelerometer        I2C SDA                   PTD9     I2C0_SDA
                     I2C SCL                   PTD8     I2C0_SCL
                     IRQ                       PTD10    PTD10
Touch Pad / Segment  Electrode 0 (J3 Pin 3)    PTB0     TSI0_CH0
LCD TWRPI Socket     Electrode 1 (J3 Pin 5)    PTB1     TSI0_CH6
                     Electrode 2 (J3 Pin 7)    PTB2     TSI0_CH7
                     Electrode 3 (J3 Pin 8)    PTB3     TSI0_CH8
                     Electrode 4 (J3 Pin 9)    PTC0     TSI0_CH13
                     Electrode 5 (J3 Pin 10)   PTC1     TSI0_CH14
                     Electrode 6 (J3 Pin 11)   PTC2     TSI0_CH15
                     Electrode 7 (J3 Pin 12)   PTA4     TSI0_CH5
                     Electrode 8 (J3 Pin 13)   PTB16    TSI0_CH9
                     Electrode 9 (J3 Pin 14)   PTB17    TSI0_CH10
                     Electrode 10 (J3 Pin 15)  PTB18    TSI0_CH11
                     Electrode 11 (J3 Pin 16)  PTB19    TSI0_CH12
                     TWRPI ID0 (J3 Pin 17)     ?        ADC1_DP1
                     TWRPI ID1 (J3 Pin 18)     ?        ADC1_SE16

Connections via the General Purpose Tower Plug-in (TWRPI) Socket
-------------------- ------------------------- -------- -------------------
FEATURE             CONNECTION                 PORT/PIN PIN FUNCTION
-------------------- ------------------------- -------- -------------------
General Purpose      TWRPI AN0 (J4 Pin 8)       ?        ADC0_DP0/ADC1_DP3
TWRPI Socket         TWRPI AN1 (J4 Pin 9)       ?        ADC0_DM0/ADC1_DM3
                     TWRPI AN2 (J4 Pin 12)      ?        ADC1_DP0/ADC0_DP3
                     TWRPI ID0 (J4 Pin 17)      ?        ADC0_DP1
                     TWRPI ID1 (J4 Pin 18)      ?        ADC0_DM1
                     TWRPI I2C SCL (J5 Pin 3)   PTD8     I2C0_SCL
                     TWRPI I2C SDA (J5 Pin 4)   PTD9     I2C0_SDA
                     TWRPI SPI MISO (J5 Pin 9)  PTD14    SPI2_SIN
                     TWRPI SPI MOSI (J5 Pin 10) PTD13    SPI2_SOUT
                     TWRPI SPI SS (J5 Pin 11)   PTD15    SPI2_PCS0
                     TWRPI SPI CLK (J5 Pin 12)  PTD12    SPI2_SCK
                     TWRPI GPIO0 (J5 Pin 15)    PTD10    PTD10
                     TWRPI GPIO1 (J5 Pin 16)    PTB8     PTB8
                     TWRPI GPIO2 (J5 Pin 17)    PTB9     PTB9
                     TWRPI GPIO3 (J5 Pin 18)    PTA19    PTA19
                     TWRPI GPIO4 (J5 Pin 19)    PTE26    PTE26

The TWR-K60N512 features two expansion card-edge connectors that interface
to the Primary and Secondary Elevator boards in a Tower system. The Primary
Connector (comprised of sides A and B) is utilized by the TWR-K60N512 while
the Secondary Connector (comprised of sides C and D) only makes connections
to the GND pins.

Connections via the Tower Primary Connector Side A
--- -------------------- --------------------------------
PIN NAME                 USAGE
--- -------------------- --------------------------------
A7  SCL0                 PTD8
A8  SDA0                 PTD9
A9  GPIO9 / CTS1         PTC19
A10 GPIO8 / SDHC_D2      PTE5
A11 GPIO7 / SD_WP_DET    PTE27
A13 ETH_MDC              PTB1
A14 ETH_MDIO             PTB0
A16 ETH_RXDV             PTA14
A19 ETH_RXD1             PTA12
A20 ETH_RXD0             PTA13
A21 SSI_MCLK             PTE6
A22 SSI_BCLK             PTE12
A23 SSI_FS               PTE11
A24 SSI_RXD              PTE7
A25 SSI_TXD              PTE10
A27 AN3                  PGA0_DP/ADC0_DP0/ADC1_DP3
A28 AN2                  PGA0_DM/ADC0_DM0/ADC1_DM3
A29 AN1                  PGA1_DP/ADC1_DP0/ADC0_DP3
A30 AN0                  PGA1_DM/ADC1_DM0/ADC0_DM3
A33 TMR1                 PTA9
A34 TMR0                 PTA8
A35 GPIO6                PTB9
A37 PWM3                 PTA6
A38 PWM2                 PTC3
A39 PWM1                 PTC2
A40 PWM0                 PTC1
A41 RXD0                 PTE25
A42 TXD0                 PTE24
A43 RXD1                 PTC16
A44 TXD1                 PTC17
A64 CLKOUT0              PTC3
A66 EBI_AD14             PTC0
A67 EBI_AD13             PTC1
A68 EBI_AD12             PTC2
A69 EBI_AD11             PTC4
A70 EBI_AD10             PTC5
A71 EBI_AD9              PTC6
A71 EBI_R/W_b            PTC11
A72 EBI_AD8              PTC7
A73 EBI_AD7              PTC8
A74 EBI_AD6              PTC9
A75 EBI_AD5              PTC10
A76 EBI_AD4              PTD2
A77 EBI_AD3              PTD3
A78 EBI_AD2              PTD4
A79 EBI_AD1              PTD5
A80 EBI_AD0              PTD6

Connections via the Tower Primary Connector Side B
--- -------------------- --------------------------------
PIN NAME                 USAGE
--- -------------------- --------------------------------
B7  SDHC_CLK / SPI1_CLK  PTE2
B9  SDHC_D3 / SPI1_CS0_b PTE4
B10 SDHC_CMD / SPI1_MOSI PTE1
B11 SDHC_D0 / SPI1_MISO  PTE3
B13 ETH_RXER             PTA5
B15 ETH_TXEN             PTA15
B19 ETH_TXD1             PTA17
B20 ETH_TXD0             PTA16
B21 GPIO1 / RTS1         PTC18
B22 GPIO2 / SDHC_D1      PTE0
B23 GPIO3                PTE28
B24 CLKIN0               PTA18
B25 CLKOUT1              PTE26
B27 AN7                  PTB7
B28 AN6                  PTB6
B29 AN5                  PTB5
B30 AN4                  PTB4
B34 TMR2                 PTD6
B35 GPIO4                PTB8
B37 PWM7                 PTA2
B38 PWM6                 PTA1
B39 PWM5                 PTD5
B40 PWM4                 PTA7
B41 CANRX0               PTE25
B42 CANTX0               PTE24
B44 SPI0_MISO            PTD14
B45 SPI0_MOSI            PTD13
B46 SPI0_CS0_b           PTD11
B47 SPI0_CS1_b           PTD15
B48 SPI0_CLK             PTD12
B50 SCL1                 PTD8
B51 SDA1                 PTD9
B52 GPIO5 / SD_CARD_DET  PTE28
B55 IRQ_H                PTA24
B56 IRQ_G                PTA24
B57 IRQ_F                PTA25
B58 IRQ_E                PTA25
B59 IRQ_D                PTA26
B60 IRQ_C                PTA26
B61 IRQ_B                PTA27
B62 IRQ_A                PTA27
B63 EBI_ALE / EBI_CS1_b  PTD0
B64 EBI_CS0_b            PTD1
B66 EBI_AD15             PTB18
B67 EBI_AD16             PTB17
B68 EBI_AD17             PTB16
B69 EBI_AD18             PTB11
B70 EBI_AD19             PTB10
B72 EBI_OE_b             PTB19
B73 EBI_D7               PTB20
B74 EBI_D6               PTB21
B75 EBI_D5               PTB22
B76 EBI_D4               PTB23
B77 EBI_D3               PTC12
B78 EBI_D2               PTC13
B79 EBI_D1               PTC14
B80 EBI_D0               PTC15

TWR-SER Serial Board Connection
===============================

The serial board connects into the tower and then maps to the tower pins to
yet other functions (see TWR-SER.pdf).

For the serial port, the following jumpers are required:

  J15: 1-2 (default)
  J17: 1-2 (default)
  J18: 1-2 (default)
  J19: 1-2 (default)

The two connections map as follows:

  A41 RXD0  - Not connected
  A42 TXD0  - Not connected
  A43 RXD1  - ELE_RXD (connects indirectory to DB-9 connector J8)
  A44 TXD1  - ELE_TXD (connects indirectory to DB-9 connector J8)

Finally, we can conclude that:

  UART4 (PTE24/25) is not connected, and
  UART3 (PTC16/17) is associated with the DB9 connector

NOTE: UART5 is associated with OSJTAG bridge and may also be usable.

LEDs
====

The TWR-K60N100 board has four LEDs labeled D2..D4 on the board.  Usage of
these LEDs is defined in include/board.h and src/up_leds.c.  They are encoded
as follows:

    SYMBOL                Meaning                  LED1*    LED2    LED3    LED4
    -------------------   -----------------------  -------  ------- ------- ------
    LED_STARTED           NuttX has been started   ON       OFF     OFF     OFF
    LED_HEAPALLOCATE      Heap has been allocated  OFF      ON      OFF     OFF
    LED_IRQSENABLED       Interrupts enabled       ON       ON      OFF     OFF
    LED_STACKCREATED      Idle stack created       OFF      OFF     ON      OFF
    LED_INIRQ             In an interrupt**        ON       N/C     N/C     OFF
    LED_SIGNAL            In a signal handler***   N/C      ON      N/C     OFF
    LED_ASSERTION         An assertion failed      ON       ON      N/C     OFF
    LED_PANIC             The system has crashed   N/C      N/C      N/C    ON
    LED_IDLE              STM32 is is sleep mode   (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

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

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the CodeSourcery Windows toolchain.  To
  use the devkitARM or the NuttX GNU toolchain, you simply need to change the
  the following configuration options to your .config (or defconfig) file:

    CONFIG_KINETIS_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_KINETIS_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_KINETIS_DEVKITARM=y      : devkitARM under Windows
    CONFIG_KINETIS_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_KINETIS_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows) and devkitARM toolchains are
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

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
====

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
  3) Set up include pathes:  You will need include/, arch/arm/src/k40,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/kinetis/k40_vectors.S.

NuttX EABI "buildroot" Toolchain
================================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M4 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M4 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  NOTE:  The NuttX toolchain may not include optimizations for Cortex-M4 (ARMv7E-M).

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh twr-k60n512/<sub-dir>

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
  building a Cortex-M4 toolchain for Cygwin under Windows.

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

TWR-K60N512-specific Configuration Options
==========================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This sould
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM4=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=k40

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_MK60N512VMD100

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=twr-k60n512 (for the TWR-K60N512 development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_TWR_K60N512=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The Kinetis K60 supports interrupt prioritization

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

    CONFIG_KINETIS_TRACE    -- Enable trace clocking on power up.
    CONFIG_KINETIS_FLEXBUS  -- Enable flexbus clocking on power up.
      CONFIG_KINETIS_UART0    -- Support UART0
      CONFIG_KINETIS_UART1    -- Support UART1
      CONFIG_KINETIS_UART2    -- Support UART2
      CONFIG_KINETIS_UART3    -- Support UART3
      CONFIG_KINETIS_UART4    -- Support UART4
      CONFIG_KINETIS_UART5    -- Support UART5
      CONFIG_KINETIS_ENET     -- Support Ethernet (K60 only)
      CONFIG_KINETIS_RNGB     -- Support the random number generator(K60 only)
    CONFIG_KINETIS_FLEXCAN0 -- Support FlexCAN0
    CONFIG_KINETIS_FLEXCAN1 -- Support FlexCAN1
    CONFIG_KINETIS_SPI0     -- Support SPI0
    CONFIG_KINETIS_SPI1     -- Support SPI1
    CONFIG_KINETIS_SPI2     -- Support SPI2
    CONFIG_KINETIS_I2C0     -- Support I2C0
    CONFIG_KINETIS_I2C1     -- Support I2C1
    CONFIG_KINETIS_I2S      -- Support I2S
      CONFIG_KINETIS_DAC0     -- Support DAC0
      CONFIG_KINETIS_DAC1     -- Support DAC1
    CONFIG_KINETIS_ADC0     -- Support ADC0
    CONFIG_KINETIS_ADC1     -- Support ADC1
    CONFIG_KINETIS_CMP      -- Support CMP
    CONFIG_KINETIS_VREF     -- Support VREF
    CONFIG_KINETIS_SDHC     -- Support SD host controller
    CONFIG_KINETIS_FTM0     -- Support FlexTimer 0
    CONFIG_KINETIS_FTM1     -- Support FlexTimer 1
    CONFIG_KINETIS_FTM2     -- Support FlexTimer 2
    CONFIG_KINETIS_LPTIMER  -- Support the low power timer
    CONFIG_KINETIS_RTC      -- Support RTC
    CONFIG_KINETIS_SLCD     -- Support the segment LCD (K60 only)
    CONFIG_KINETIS_EWM      -- Support the external watchdog
    CONFIG_KINETIS_CMT      -- Support Carrier Modulator Transmitter
    CONFIG_KINETIS_USBOTG   -- Support USB OTG (see also CONFIG_USBHOST and CONFIG_USBDEV)
    CONFIG_KINETIS_USBDCD   -- Support the USB Device Charger Detection module
    CONFIG_KINETIS_LLWU     -- Support the Low Leakage Wake-Up Unit
    CONFIG_KINETIS_TSI      -- Support the touch screeen interface
    CONFIG_KINETIS_FTFL     -- Support FLASH
    CONFIG_KINETIS_DMA      -- Support DMA
    CONFIG_KINETIS_CRC      -- Support CRC
    CONFIG_KINETIS_PDB      -- Support the Programmable Delay Block
    CONFIG_KINETIS_PIT      -- Support Programmable Interval Timers
    CONFIG_ARMV7M_MPU       -- Support the MPU

  Kinetis interrupt priorities (Default is the mid priority)

    CONFIG_KINETIS_UART0PRIO
    CONFIG_KINETIS_UART1PRIO
    CONFIG_KINETIS_UART2PRIO
    CONFIG_KINETIS_UART3PRIO
    CONFIG_KINETIS_UART4PRIO
    CONFIG_KINETIS_UART5PRIO

    CONFIG_KINETIS_EMACTMR_PRIO
    CONFIG_KINETIS_EMACTX_PRIO
    CONFIG_KINETIS_EMACRX_PRIO
    CONFIG_KINETIS_EMACMISC_PRIO

    CONFIG_KINETIS_SDHC_PRIO

  PIN Interrupt Support

    CONFIG_GPIO_IRQ          -- Enable pin interrtup support.  Also needs
      one or more of the following:
    CONFIG_KINETIS_PORTAINTS -- Support 32 Port A interrupts
    CONFIG_KINETIS_PORTBINTS -- Support 32 Port B interrupts
    CONFIG_KINETIS_PORTCINTS -- Support 32 Port C interrupts
    CONFIG_KINETIS_PORTDINTS -- Support 32 Port D interrupts
    CONFIG_KINETIS_PORTEINTS -- Support 32 Port E interrupts

  Kinetis K60 specific device driver settings

    CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn (n=0..5) for the
      console and ttys0 (default is the UART0).
    CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_UARTn_BAUD - The configure BAUD of the UART.
    CONFIG_UARTn_BITS - The number of bits.  Must be either 8 or 8.
    CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity

  Kenetis ethernet controller settings

    CONFIG_ENET_NRXBUFFERS - Number of RX buffers.  The size of one
        buffer is determined by CONFIG_NET_BUFSIZE.  Default: 6
    CONFIG_ENET_NTXBUFFERS - Number of TX buffers.  The size of one
        buffer is determined by CONFIG_NET_BUFSIZE.  Default: 2
    CONFIG_ENET_USEMII - Usee MII mode.  Default: RMII mode.
    CONFIG_ENET_PHYADDR - PHY address
 
Configurations
==============

Each TWR-K60N512 configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh twr-k60n512/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

    CONFIG_KINETIS_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.
    Support for the board's SPI-based MicroSD card is included
    (but not passing tests as of this writing).

    NOTE: An SDHC driver is underwork and can be enabled in the NSH
    configuration for further testing be setting the following
    configuration faluesas follows:

      -CONFIG_KINETIS_SDHC=n
      +CONFIG_KINETIS_SDHC=y         # Enable the SDHC driver

      -CONFIG_GPIO_IRQ=n
      +CONFIG_GPIO_IRQ=y             # Enable GPIO interrupts

      -CONFIG_KINETIS_PORTEINTS=n
      +CONFIG_KINETIS_PORTEINTS=y    # Enable PortE GPIO interrupts

      -CONFIG_SCHED_WORKQUEUE=n
      +CONFIG_SCHED_WORKQUEUE=y      # Enable the NuttX workqueue

      -CONFIG_NSH_ARCHINIT=n
      +CONFIG_NSH_ARCHINIT=y         # Provide NSH intialization logic


