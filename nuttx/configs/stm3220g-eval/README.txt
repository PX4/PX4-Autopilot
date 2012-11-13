README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM3220G-EVAL development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - STM3220G-EVAL-specific Configuration Options
  - LEDs
  - Ethernet
  - PWM
  - CAN
  - FSMC SRAM
  - I/O Expanders
  - STM3220G-EVAL-specific Configuration Options
  - Configurations

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the Raisonance R-Link emulatator and some RIDE7 development tools
  were used and those tools works only under Windows.

GNU Toolchain Options
=====================

  Toolchain Configurations
  ------------------------
  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The Atollic Toolchain,
  3. The devkitARM GNU toolchain,
  4. Raisonance GNU toolchain, or
  5. The NuttX buildroot Toolchain (see below).

  Most testing has been conducted using the CodeSourcery toolchain for Windows and
  that is the default toolchain in most configurations.  To use the Atollic
  devkitARM, Raisonance GNU, or NuttX buildroot toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_STM32_ATOLLIC_LITE=y   : The free, "Lite" version of Atollic toolchain under Windows
    CONFIG_STM32_ATOLLIC_PRO=y    : The paid, "Pro" version of Atollic toolchain under Windows
    CONFIG_STM32_DEVKITARM=y      : devkitARM under Windows
    CONFIG_STM32_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you change the default toolchain, then you may also have to modify the PATH in
  the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows), Atollic, devkitARM, and Raisonance toolchains are
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
  3) Set up include pathes:  You will need include/, arch/arm/src/stm32,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/stm32/stm32_vectors.S.  With RIDE, I have to build NuttX
  one time from the Cygwin command line in order to obtain the pre-built
  startup object needed by RIDE.

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
     ./configure.sh stm3220g-eval/<sub-dir>

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

Ethernet
========

The Ethernet driver is configured to use the MII interface:

  Board Jumper Settings:

    Jumper  Description
    JP8     To enable MII, JP8 should not be fitted.
    JP6     2-3: Enable MII interface mode
    JP5     2-3: Provide 25 MHz clock for MII or 50 MHz clock for RMII by MCO at PA8
    SB1     Not used with MII

LEDs
====

The STM3220G-EVAL board has four LEDs labeled LD1, LD2, LD3 and LD4 on the
board.. These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related\
events as follows:

    SYMBOL               Meaning                 LED1*   LED2    LED3    LED4
    -------------------  ----------------------- ------- ------- ------- ------
    LED_STARTED          NuttX has been started  ON      OFF     OFF     OFF
    LED_HEAPALLOCATE     Heap has been allocated OFF     ON      OFF     OFF
    LED_IRQSENABLED      Interrupts enabled      ON      ON      OFF     OFF
    LED_STACKCREATED     Idle stack created      OFF     OFF     ON      OFF
    LED_INIRQ            In an interrupt**       ON      N/C     N/C     OFF
    LED_SIGNAL           In a signal handler***  N/C     ON      N/C     OFF
    LED_ASSERTION        An assertion failed     ON      ON      N/C     OFF
    LED_PANIC            The system has crashed  N/C     N/C     N/C     ON
    LED_IDLE             STM32 is is sleep mode  (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

PWM
===

The STM3220G-Eval has no real on-board PWM devices, but the board can be
configured to output a pulse train using timer output pins.  The following
pins have been use to generate PWM output (see board.h for some other
candidates):

TIM4 CH2.  Pin PD13 is used by the FSMC (FSMC_A18) and is also connected
to the Motor Control Connector (CN5) just for this purpose.  If FSMC is
not enabled, then FSMC_A18 will not be used (and will be tri-stated from
the LCD).

  CONFIGURATION:

    CONFIG_STM32_TIM4=y
    CONFIG_PWM=n
    CONFIG_PWM_PULSECOUNT=n
    CONFIG_STM32_TIM4_PWM=y
    CONFIG_STM32_TIM4_CHANNEL=2

  ACCESS:

    Daughterboard Extension Connector, CN3, pin 32
    Ground is available on CN3, pin1

  NOTE: TIM4 hardware will not support pulse counting.

TIM8 CH4:  Pin PC9 is used by the microSD card (MicroSDCard_D1) and I2S
(I2S_CKIN) but can be completely disconnected from both by opening JP16.

  CONFIGURATION:

    CONFIG_STM32_TIM8=y
    CONFIG_PWM=n
    CONFIG_PWM_PULSECOUNT=y
    CONFIG_STM32_TIM8_PWM=y
    CONFIG_STM32_TIM8_CHANNEL=4

  ACCESS:

    Daughterboard Extension Connector, CN3, pin 17
    Ground is available on CN3, pin1

CAN
===

Connector 10 (CN10) is DB-9 male connector that can be used with CAN1 or CAN2.

  JP10 connects CAN1_RX or CAN2_RX to the CAN transceiver
  JP3 connects CAN1_TX or CAN2_TX to the CAN transceiver

CAN signals are then available on CN10 pins:

  CN10 Pin 7 = CANH
  CN10 Pin 2 = CANL

Mapping to STM32 GPIO pins:

  PD0   = FSMC_D2 & CAN1_RX
  PD1   = FSMC_D3 & CAN1_TX
  PB13  = ULPI_D6 & CAN2_TX
  PB5   = ULPI_D7 & CAN2_RX

Configuration Options:

  CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
    CONFIG_STM32_CAN2 must also be defined)
  CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
    Standard 11-bit IDs.
 CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
    Default: 8
  CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
    Default: 4

  CONFIG_STM32_CAN1 - Enable support for CAN1
  CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1 is defined.
  CONFIG_STM32_CAN2 - Enable support for CAN2
  CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2 is defined.
  CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
  CONFIG_CAN_TSEG2 - the number of CAN time quanta in segment 2. Default: 7
  CONFIG_CAN_REGDEBUG - If CONFIG_DEBUG is set, this will generate an
    dump of all CAN registers.

FSMC SRAM
=========

On-board SRAM
-------------

A 16 Mbit SRAM is connected to the STM32F407IGH6 FSMC bus which shares the same
I/Os with the CAN1 bus. Jumper settings:

  JP1: Connect PE4 to SRAM as A20
  JP2: onnect PE3 to SRAM as A19

JP3 and JP10 must not be fitted for SRAM and LCD application.  JP3 and JP10
select CAN1 or CAN2 if fitted; neither if not fitted.

The on-board SRAM can be configured by setting

  CONFIG_STM32_FSMC=y
  CONFIG_STM32_FSMC_SRAM=y
  CONFIG_HEAP2_BASE=0x64000000
  CONFIG_HEAP2_SIZE=2097152
  CONFIG_MM_REGIONS=2

Configuration Options
---------------------

Internal SRAM is available in all members of the STM32 family. In addition
to internal SRAM, SRAM may also be available through the FSMC.  In order to
use FSMC SRAM, the following additional things need to be present in the
NuttX configuration file:

  CONFIG_STM32_FSMC=y        : Enables the FSMC
  CONFIG_STM32_FSMC_SRAM=y   : Indicates that SRAM is available via the
                               FSMC (as opposed to an LCD or FLASH).
  CONFIG_HEAP2_BASE          : The base address of the SRAM in the FSMC
                               address space
  CONFIG_HEAP2_SIZE          : The size of the SRAM in the FSMC
                               address space
  CONFIG_MM_REGIONS          : Must be set to a large enough value to
                               include the FSMC SRAM

SRAM Configurations
-------------------
There are 2 possible SRAM configurations:

  Configuration 1. System SRAM (only)
                   CONFIG_MM_REGIONS == 1
  Configuration 2. System SRAM and FSMC SRAM
                   CONFIG_MM_REGIONS == 2
                   CONFIG_STM32_FSMC_SRAM defined

I/O Expanders
=============

The STM3220G-EVAL has two STMPE811QTR I/O expanders on board both connected to
the STM32 via I2C1.  They share a common interrupt line: PI2.

STMPE811 U24, I2C address 0x41 (7-bit)
------ ---- ---------------- --------------------------------------------
STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
------ ---- ---------------- --------------------------------------------
  Y-        TouchScreen_Y-   LCD Connector XL
  X-        TouchScreen_X-   LCD Connector XR
  Y+        TouchScreen_Y+   LCD Connector XD
  X+        TouchScreen_X+   LCD Connector XU
  IN3       EXP_IO9
  IN2       EXP_IO10
  IN1       EXP_IO11
  IN0       EXP_IO12

STMPE811 U29, I2C address 0x44 (7-bit)
------ ---- ---------------- --------------------------------------------
STPE11 PIN  BOARD SIGNAL     BOARD CONNECTION
------ ---- ---------------- --------------------------------------------
  Y-        EXP_IO1
  X-        EXP_IO2
  Y+        EXP_IO3
  X+        EXP_IO4
  IN3       EXP_IO5
  IN2       EXP_IO6
  IN1       EXP_IO7
  IN0       EXP_IO8

STM3220G-EVAL-specific Configuration Options
============================================

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=stm32

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_STM32F207IG=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm3220g_eval (for the STM3220G-EVAL development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM3220G_EVAL=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    In addition to internal SRAM, SRAM may also be available through the FSMC.
    In order to use FSMC SRAM, the following additional things need to be
    present in the NuttX configuration file:

    CONFIG_STM32_FSMC_SRAM - Indicates that SRAM is available via the
      FSMC (as opposed to an LCD or FLASH).

    CONFIG_HEAP2_BASE - The base address of the SRAM in the FSMC address space (hex)

    CONFIG_HEAP2_SIZE - The size of the SRAM in the FSMC address space (decimal)

    CONFIG_ARCH_IRQPRIO - The STM3220xxx supports interrupt prioritization

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

    AHB1
    ----
    CONFIG_STM32_CRC
    CONFIG_STM32_BKPSRAM
    CONFIG_STM32_DMA1
    CONFIG_STM32_DMA2
    CONFIG_STM32_ETHMAC
    CONFIG_STM32_OTGHS

    AHB2
    ----
    CONFIG_STM32_DCMI
    CONFIG_STM32_CRYP
    CONFIG_STM32_HASH
    CONFIG_STM32_RNG
    CONFIG_STM32_OTGFS

    AHB3
    ----
    CONFIG_STM32_FSMC

    APB1
    ----
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_TIM5
    CONFIG_STM32_TIM6
    CONFIG_STM32_TIM7
    CONFIG_STM32_TIM12
    CONFIG_STM32_TIM13
    CONFIG_STM32_TIM14
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_SPI3
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_UART4
    CONFIG_STM32_UART5
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_I2C3
    CONFIG_STM32_CAN1
    CONFIG_STM32_CAN2
    CONFIG_STM32_DAC1
    CONFIG_STM32_DAC2
    CONFIG_STM32_PWR -- Required for RTC

    APB2
    ----
    CONFIG_STM32_TIM1
    CONFIG_STM32_TIM8
    CONFIG_STM32_USART1
    CONFIG_STM32_USART6
    CONFIG_STM32_ADC1
    CONFIG_STM32_ADC2
    CONFIG_STM32_ADC3
    CONFIG_STM32_SDIO
    CONFIG_STM32_SPI1
    CONFIG_STM32_SYSCFG
    CONFIG_STM32_TIM9
    CONFIG_STM32_TIM10
    CONFIG_STM32_TIM11

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion. Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,14
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,14
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,14, m=1,..,3
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,14
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,14, m=1,..,2

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}

  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  JTAG Enable settings (by default JTAG-DP and SW-DP are disabled):

    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM3220xxx specific device driver settings

    CONFIG_U[S]ARTn_SERIAL_CONSOLE - selects the USARTn (n=1,2,3) or UART
           m (m=4,5) for the console and ttys0 (default is the USART1).
    CONFIG_U[S]ARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_U[S]ARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_U[S]ARTn_BAUD - The configure BAUD of the UART.  Must be
    CONFIG_U[S]ARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_U[S]ARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_U[S]ARTn_2STOP - Two stop bits

    CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

    CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32_SDIO
      and CONFIG_STM32_DMA2.
    CONFIG_SDIO_PRI - Select SDIO interrupt prority.  Default: 128
    CONFIG_SDIO_DMAPRIO - Select SDIO DMA interrupt priority.
      Default:  Medium
    CONFIG_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
      4-bit transfer mode.

    CONFIG_STM32_PHYADDR - The 5-bit address of the PHY on the board
    CONFIG_STM32_MII - Support Ethernet MII interface
    CONFIG_STM32_MII_MCO1 - Use MCO1 to clock the MII interface
    CONFIG_STM32_MII_MCO2 - Use MCO2 to clock the MII interface
    CONFIG_STM32_RMII - Support Ethernet RMII interface
    CONFIG_STM32_AUTONEG - Use PHY autonegotion to determine speed and mode
    CONFIG_STM32_ETHFD - If CONFIG_STM32_AUTONEG is not defined, then this
      may be defined to select full duplex mode. Default: half-duplex
    CONFIG_STM32_ETH100MBPS - If CONFIG_STM32_AUTONEG is not defined, then this
      may be defined to select 100 MBps speed.  Default: 10 Mbps
    CONFIG_STM32_PHYSR - This must be provided if CONFIG_STM32_AUTONEG is
      defined.  The PHY status register address may diff from PHY to PHY.  This
      configuration sets the address of the PHY status register.
    CONFIG_STM32_PHYSR_SPEED - This must be provided if CONFIG_STM32_AUTONEG is
      defined.  This provides bit mask indicating 10 or 100MBps speed.
    CONFIG_STM32_PHYSR_100MBPS - This must be provided if CONFIG_STM32_AUTONEG is
      defined.  This provides the value of the speed bit(s) indicating 100MBps speed.
    CONFIG_STM32_PHYSR_MODE - This must be provided if CONFIG_STM32_AUTONEG is
      defined.  This provide bit mask indicating full or half duplex modes.
    CONFIG_STM32_PHYSR_FULLDUPLEX - This must be provided if CONFIG_STM32_AUTONEG is
      defined.  This provides the value of the mode bits indicating full duplex mode.
    CONFIG_STM32_ETH_PTP - Precision Time Protocol (PTP).  Not supported
      but some hooks are indicated with this condition.

  STM3220G-EVAL CAN Configuration

    CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
      CONFIG_STM32_CAN2 must also be defined)
    CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
      Default: 8
    CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
      Default: 4
    CONFIG_CAN_LOOPBACK - A CAN driver may or may not support a loopback
      mode for testing. The STM32 CAN driver does support loopback mode.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2 is defined.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 - the number of CAN time quanta in segment 2. Default: 7
    CONFIG_CAN_REGDEBUG - If CONFIG_DEBUG is set, this will generate an
      dump of all CAN registers.

  STM3220G-EVAL LCD Hardware Configuration

  STM32 USB OTG FS Host Driver Support

  Pre-requisites
 
   CONFIG_USBHOST      - Enable general USB host support
   CONFIG_STM32_OTGFS  - Enable the STM32 USB OTG FS block
   CONFIG_STM32_SYSCFG - Needed
 
  Options:
 
   CONFIG_STM32_OTGFS_RXFIFO_SIZE - Size of the RX FIFO in 32-bit words.
     Default 128 (512 bytes)
   CONFIG_STM32_OTGFS_NPTXFIFO_SIZE - Size of the non-periodic Tx FIFO
     in 32-bit words.  Default 96 (384 bytes)
   CONFIG_STM32_OTGFS_PTXFIFO_SIZE - Size of the periodic Tx FIFO in 32-bit
     words.  Default 96 (384 bytes)
   CONFIG_STM32_OTGFS_DESCSIZE - Maximum size of a descriptor.  Default: 128
   CONFIG_STM32_OTGFS_SOFINTR - Enable SOF interrupts.  Why would you ever
     want to do that?
   CONFIG_STM32_USBHOST_REGDEBUG - Enable very low-level register access
     debug.  Depends on CONFIG_DEBUG.
   CONFIG_STM32_USBHOST_PKTDUMP - Dump all incoming and outgoing USB
     packets. Depends on CONFIG_DEBUG.

Configurations
==============

Each STM3220G-EVAL configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh stm3220g-eval/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  dhcpd:
  -----

    This builds the DCHP server using the apps/examples/dhcpd application
    (for execution from FLASH.) See apps/examples/README.txt for information
    about the dhcpd example.  The server address is 10.0.0.1 and it serves
    IP addresses in the range 10.0.0.2 through 10.0.0.17 (all of which, of
    course, are configurable).

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows

  nettest:
  -------

    This configuration directory may be used to verify networking performance
    using the STM32's Ethernet controller. It uses apps/examples/nettest to excercise the
    TCP/IP network.

    CONFIG_EXAMPLES_NETTEST_SERVER=n                       : Target is configured as the client
    CONFIG_EXAMPLES_NETTEST_PERFORMANCE=y                  : Only network performance is verified.
    CONFIG_EXAMPLES_NETTEST_IPADDR=(10<<24|0<<16|0<<8|2)   : Target side is IP: 10.0.0.2
    CONFIG_EXAMPLES_NETTEST_DRIPADDR=(10<<24|0<<16|0<<8|1) : Host side is IP: 10.0.0.1
    CONFIG_EXAMPLES_NETTEST_CLIENTIP=(10<<24|0<<16|0<<8|1) : Server address used by which ever is client.

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    CONFIG_STM32_CODESOURCERYW=y                  : CodeSourcery under Windows
    CONFIG_NSH_DHCPC=n                            : DHCP is disabled
    CONFIG_NSH_IPADDR=(192<<24|168<<16|13<<8|161) : Target IP address 192.168.8.161
    CONFIG_NSH_DRIPADDR=(192<<24|168<<16|13<<8|1) : Host IP address 192.168.8.1

    NOTES:
    1. This example assumes that a network is connected.  During its
       initialization, it will try to negotiate the link speed.  If you have
       no network connected when you reset the board, there will be a long
       delay (maybe 30 seconds?) before anything happens.  That is the timeout
       before the networking finally gives up and decides that no network is
       available.

    2. This example supports the ADC test (apps/examples/adc) but this must
       be manually enabled by selecting:

       CONFIG_ADC=y             : Enable the generic ADC infrastructure
       CONFIG_STM32_ADC3=y      : Enable ADC3
       CONFIG_STM32_TIM1=y      : Enable Timer 1
       CONFIG_STM32_TIM1_ADC=y  : Indicate that timer 1 will be used to trigger an ADC
       CONFIG_STM32_TIM1_ADC3=y : Assign timer 1 to drive ADC3 sampling
       CONFIG_STM32_ADC3_SAMPLE_FREQUENCY=100 : Select a sampling frequency

       See also apps/examples/README.txt

       General debug for analog devices (ADC/DAC):

       CONFIG_DEBUG_ANALOG

    3. This example supports the PWM test (apps/examples/pwm) but this must
       be manually enabled by selecting eeither

       CONFIG_PWM=y                : Enable the generic PWM infrastructure
       CONFIG_PWM_PULSECOUNT=n     : Disable to support for TIM1/8 pulse counts
       CONFIG_STM32_TIM4=y         : Enable TIM4
       CONFIG_STM32_TIM4_PWM=y     : Use TIM4 to generate PWM output
       CONFIG_STM32_TIM4_CHANNEL=2 : Select output on TIM4, channel 2

       If CONFIG_STM32_FSMC is disabled, output will appear on CN3, pin 32.
       Ground is available on CN3, pin1.

       Or..

       CONFIG_PWM=y                : Enable the generic PWM infrastructure
       CONFIG_PWM_PULSECOUNT=y     : Enable to support for TIM1/8 pulse counts
       CONFIG_STM32_TIM8=y         : Enable TIM8
       CONFIG_STM32_TIM8_PWM=y     : Use TIM8 to generate PWM output
       CONFIG_STM32_TIM8_CHANNEL=4 : Select output on TIM8, channel 4

       If CONFIG_STM32_FSMC is disabled, output will appear on CN3, pin 17
       Ground is available on CN23 pin1.

       See also include/board.h and apps/examples/README.txt

       Special PWM-only debug options:

       CONFIG_DEBUG_PWM

    4. This example supports the CAN loopback test (apps/examples/can) but this
       must be manually enabled by selecting:

       CONFIG_CAN=y             : Enable the generic CAN infrastructure
       CONFIG_CAN_EXID=y or n   : Enable to support extended ID frames
       CONFIG_STM32_CAN1=y      : Enable CAN1
       CONFIG_CAN_LOOPBACK=y    : Enable CAN loopback mode

       See also apps/examples/README.txt

       Special CAN-only debug options:

       CONFIG_DEBUG_CAN
       CONFIG_CAN_REGDEBUG

    5. This example can support an FTP client.  In order to build in FTP client
       support simply uncomment the following lines in the appconfig file (before
       configuring) or in the apps/.config file (after configuring):

       #CONFIGURED_APPS += netutils/ftpc
       #CONFIGURED_APPS += examples/ftpc

    6. This example can support an FTP server.  In order to build in FTP server
       support simply uncomment the following lines in the appconfig file (before
       configuring) or in the apps/.config file (after configuring):

       #CONFIGURED_APPS += netutils/ftpd
       #CONFIGURED_APPS += examples/ftpd

       And enable poll() support in the NuttX configuration file:

       CONFIG_DISABLE_POLL=n

    7. This example supports the watchdog timer test (apps/examples/watchdog)
       but this must be manually enabled by selecting:

       CONFIG_WATCHDOG=y         : Enables watchdog timer driver support
       CONFIG_STM32_WWDG=y       : Enables the WWDG timer facility, OR
       CONFIG_STM32_IWDG=y       : Enables the IWDG timer facility (but not both)

       The WWDG watchdog is driven off the (fast) 42MHz PCLK1 and, as result,
       has a maximum timeout value of 49 milliseconds.  For WWDG watchdog, you
       should also add the fillowing to the configuration file:

       CONFIG_EXAMPLES_WATCHDOG_PINGDELAY=20
       CONFIG_EXAMPLES_WATCHDOG_TIMEOUT=49

       The IWDG timer has a range of about 35 seconds and should not be an issue.

    7. Adding LCD and graphics support:

       appconfig (apps/.config):  Enable the application configurations that you
       want to use.  Asexamples:

       CONFIGURED_APPS += examples/nx       : Pick one or more
       CONFIGURED_APPS += examples/nxhello  :
       CONFIGURED_APPS += examples/nximage  :
       CONFIGURED_APPS += examples/nxlines  :

       defconfig (nuttx/.config):

       CONFIG_STM32_FSMC=y                  : FSMC support is required for the LCD
       CONFIG_NX=y                          : Enable graphics suppport
       CONFIG_MM_REGIONS=2                  : When FSMC is enabled, so is the on-board SRAM memory region

    8. USB OTG FS Device or Host Support
 
       CONFIG_USBDEV          - Enable USB device support, OR
       CONFIG_USBHOST         - Enable USB host support (but not both)

       CONFIG_STM32_OTGFS     - Enable the STM32 USB OTG FS block
       CONFIG_STM32_SYSCFG    - Needed for all USB OTF FS support

       CONFIG_SCHED_WORKQUEUE - Worker thread support is required for the mass
                                storage class (both host and device).
       CONFIG_NSH_ARCHINIT    - Architecture specific USB initialization
                                is needed

    9. This configuration requires that jumper JP22 be set to enable RS-232 operation.

  nsh2:
  -----

    This is an alternaitve NSH configuration.  One limitation of the STM3220G-EVAL
    board is that you cannot have both a UART-based NSH console and SDIO support.
    The nsh2 differs from the nsh configuration in the following ways:

    -CONFIG_STM32_USART3=y      : USART3 is disabled
    + CONFIG_STM32_USART3=n

    -CONFIG_STM32_SDIO=n        : SDIO is enabled
    +CONFIG_STM32_SDIO=y

    Logically, these are the only differences:  This configuration has SDIO (and
    the SD card) enabled and the serial console disabled. There is ONLY a
    Telnet console!.

    There are some special settings to make life with only a Telnet

    CONFIG_SYSLOG=y - Enables the System Logging feature.
    CONFIG_RAMLOG=y - Enable the RAM-based logging feature.
    CONFIG_RAMLOG_CONSOLE=y - Use the RAM logger as the default console.
      This means that any console output from non-Telnet threads will
      go into the circular buffer in RAM.
    CONFIG_RAMLOG_SYSLOG - This enables the RAM-based logger as the
      system logger.  This means that (1) in addition to the console
      output from other tasks, ALL of the debug output will also to
      to the circular buffer in RAM, and (2) NSH will now support a
      command called 'dmesg' that can be used to dump the RAM log.

    There are a few other configuration differences as necessary to support
    this different device configuration. Just the do the 'diff' if you are
    curious.

    NOTES:
    1. See the notes for the nsh configuration.  Most also apply to the nsh2
       configuration.

    2. RS-232 is disabled, but Telnet is still available for use as a console.
       Since RS-232 and SDIO use the same pins (one controlled by JP22), RS232
       and SDIO cannot be used concurrently.

    3. This configuration requires that jumper JP22 be set to enable SDIO
       operation.  To enable MicroSD Card, which shares same I/Os with RS-232,
       JP22 is not fitted.

    4. In order to use SDIO without overruns, DMA must be used.

    5. Another SDIO/DMA issue.  This one is probably a software bug.  This is
       the bug as stated in the TODO list:

       "If you use a large I/O buffer to access the file system, then the
        MMCSD driver will perform multiple block SD transfers.  With DMA
        ON, this seems to result in CRC errors detected by the hardware
        during the transfer.  Workaround:  CONFIG_MMCSD_MULTIBLOCK_DISABLE=y"

       For this reason, CONFIG_MMCSD_MULTIBLOCK_DISABLE=y appears in the defconfig
       file.

    6. Another DMA-related concern.  I see this statement in the reference
       manual:  "The burst configuration has to be selected in order to respect
       the AHB protocol, where bursts must not cross the 1 KB address boundary
       because the minimum address space that can be allocated to a single slave
       is 1 KB. This means that the 1 KB address boundary should not be crossed
       by a burst block transfer, otherwise an AHB error would be generated,
       that is not reported by the DMA registers."

       There is nothing in the DMA driver to prevent this now.

  nxwm
  ----
    This is a special configuration setup for the NxWM window manager
    UnitTest.  The NxWM window manager can be found here:

      nuttx-code/NxWidgets/nxwm

    The NxWM unit test can be found at:

      nuttx-code/NxWidgets/UnitTests/nxwm

    Documentation for installing the NxWM unit test can be found here:

      nuttx-code/NxWidgets/UnitTests/README.txt

    Here is the quick summary of the build steps (Assuming that all of
    the required packages are available in a directory ~/nuttx-code):

    1. Intall the nxwm configuration

       $ cd ~/nuttx-code/nuttx/tools
       $ ./configure.sh stm3220g-eval/nxwm

    2. Make the build context (only)

       $ cd ..
       $ . ./setenv.sh
       $ make context
       ...

    3. Install the nxwm unit test

       $ cd ~/nuttx-code/NxWidgets
       $ tools/install.sh ~/nuttx-code/apps nxwm
       Creating symbolic link
        - To ~/nuttx-code/NxWidgets/UnitTests/nxwm
        - At ~/nuttx-code/apps/external

    4. Build the NxWidgets library

       $ cd ~/nuttx-code/NxWidgets/libnxwidgets
       $ make TOPDIR=~/nuttx-code/nuttx
       ...

    5. Build the NxWM library

       $ cd ~/nuttx-code/NxWidgets/nxwm
       $ make TOPDIR=~/nuttx-code/nuttx
       ...

    6. Built NuttX with the installed unit test as the application

       $ cd ~/nuttx-code/nuttx
       $ make

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows

  telnetd:
  --------

    A simple test of the Telnet daemon(see apps/netutils/README.txt,
    apps/examples/README.txt, and apps/examples/telnetd).  This is
    the same daemon that is used in the nsh configuration so if you
    use NSH, then you don't care about this.  This test is good for
    testing the Telnet daemon only because it works in a simpler
    environment than does the nsh configuration.
