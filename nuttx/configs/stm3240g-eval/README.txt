README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM32140G-EVAL development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX buildroot Toolchain
  - STM3240G-EVAL-specific Configuration Options
  - LEDs
  - Ethernet
  - PWM
  - CAN
  - FPU
  - STM3240G-EVAL-specific Configuration Options
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

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The Atollic Toolchain, 
  3. The devkitARM GNU toolchain,
  4. Raisonance GNU toolchain, or
  5. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the CodeSourcery toolchain for Windows.  To use
  the Atollic, devkitARM, Raisonance GNU, or NuttX buildroot toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_STM32_ATOLLIC=y        : Atollic toolchain under Windows
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

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

  NOTE 1:  The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2:  The free, "Lite" version of the Atollic toolchain does not support C++
  nor does it support ar, nm, objdump, or objdcopy. If you use the Atollic "Lite"
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

  Another problem that I had with the Atollic toolchain is that the provide a gcc.exe
  and g++.exe in the same bin/ file as their ARM binaries.  If the Atollic bin/ path
  appears in your PATH variable before /usr/bin, then you will get the wrong gcc
  when you try to build host executables.  This will cause to strange, uninterpretable
  errors build some host binaries in tools/ when you first make. Here is my
  workaround kludge.

  1. Edit the setenv.sh to put the Atollic toolchain at the beginning of the PATH
  2. Source the setenv.sh file: . ./setenv.sh.  A side effect of this is that it
     will set an environment variable called PATH_ORIG.
  3. Then go back to the original patch:  export PATH=$PATH_ORIG
  4. Then make.  The make will build all of the host executable but will fail
     when it gets to the first ARM binary.
  5. Then source setenv.sh again: . ./setenv.sh.  That will correct the PATH
     again.  When you do make again, the host executables are already made and
     now the correct PATH is in place for the ARM build.

  Also, the Atollic toolchain is the only toolchain that has built-in support for
  the FPU in these configurations.  If you plan to use the Cortex-M4 FPU, you will
  need to use the Atollic toolchain for now.  See the FPU section below for more
  information.

  NOTE 3:  The devkitARM toolchain includes a version of MSYS make.  Make sure that
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

NuttX buildroot Toolchain
=========================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh stm3240g-eval/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

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

The STM3240G-EVAL board has four LEDs labeled LD1, LD2, LD3 and LD4 on the
board.. These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related\
events as follows:

	SYMBOL				Meaning					LED1*	LED2	LED3	LED4
	-------------------	-----------------------	-------	-------	-------	------
	LED_STARTED			NuttX has been started	ON		OFF		OFF		OFF
	LED_HEAPALLOCATE	Heap has been allocated	OFF		ON		OFF		OFF
	LED_IRQSENABLED		Interrupts enabled		ON		ON		OFF		OFF
	LED_STACKCREATED	Idle stack created		OFF		OFF		ON		OFF
	LED_INIRQ			In an interrupt**		ON		N/C		N/C		OFF
	LED_SIGNAL			In a signal handler***  N/C		ON		N/C		OFF
	LED_ASSERTION		An assertion failed		ON		ON		N/C		OFF
	LED_PANIC			The system has crashed	N/C		N/C		N/C		ON
    LED_IDLE            STM32 is is sleep mode  (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

PWM
===

The STM3240G-Eval has no real on-board PWM devices, but the board can be
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

FPU
===

FPU Configuration Options
-------------------------

There are two version of the FPU support built into the STM32 port.

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

  ifeq ($(CONFIG_STM32_ATOLLIC),y)
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

  -CONFIG_ARCH_FPU=y              : Enable FPU support
  +CONFIG_ARCH_FPU=n

  -CONFIG_STM32_CODESOURCERYW=n   : Disable the CodeSourcery toolchain
  +CONFIG_STM32_CODESOURCERYW=y

  -CONFIG_STM32_ATOLLIC=y         : Enable the Atollic toolchain
  +CONFIG_STM32_ATOLLIC=n

  -CONFIG_INTELHEX_BINARY=n       : Suppress generation FLASH download formats
  +CONFIG_INTELHEX_BINARY=y

  -CONFIG_HAVE_CXX=n              : Suppress generation of C++ code
  +CONFIG_HAVE_CXX=y

See the section above on Toolchains, NOTE 2, for explanations for some of
the configuration settings.  Some of the usual settings are just not supported
by the "Lite" version of the Atollic toolchain.

STM3240G-EVAL-specific Configuration Options
============================================

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_CORTEXM4=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=stm32

	CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
	   chip:

	   CONFIG_ARCH_CHIP_STM32F407IG=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n
 
	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=stm3240g_eval (for the STM3240G-EVAL development board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_STM3240G_EVAL=y

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

	   CONFIG_DRAM_SIZE=0x00010000 (64Kb)

	CONFIG_DRAM_START - The start address of installed DRAM

	   CONFIG_DRAM_START=0x20000000

	CONFIG_DRAM_END - Last address+1 of installed RAM

	   CONFIG_DRAM_END=(CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

	CONFIG_ARCH_IRQPRIO - The STM3240xxx supports interrupt prioritization

	   CONFIG_ARCH_IRQPRIO=y

	CONFIG_ARCH_FPU - The STM3240xxx supports a floating point unit (FPU)

	   CONFIG_ARCH_FPU=y

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
	CONFIG_STM32_CCMDATARAM
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

  Timer and I2C devices may need to the following to force power to be applied
  unconditionally at power up.  (Otherwise, the device is powered when it is
  initialized).

    CONFIG_STM32_FORCEPOWER

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

  STM3240xxx specific device driver settings

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

  STM3240G-EVAL CAN Configuration

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

  STM3240G-EVAL LCD Hardware Configuration

Configurations
==============

Each STM3240G-EVAL configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh stm3240g-eval/<subdir>
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

    CONFIG_EXAMPLE_NETTEST_SERVER=n                       : Target is configured as the client
    CONFIG_EXAMPLE_NETTEST_PERFORMANCE=y                  : Only network performance is verified.
    CONFIG_EXAMPLE_NETTEST_IPADDR=(10<<24|0<<16|0<<8|2)   : Target side is IP: 10.0.0.2
    CONFIG_EXAMPLE_NETTEST_DRIPADDR=(10<<24|0<<16|0<<8|1) : Host side is IP: 10.0.0.1
    CONFIG_EXAMPLE_NETTEST_CLIENTIP=(10<<24|0<<16|0<<8|1) : Server address used by which ever is client.

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    CONFIG_STM32_CODESOURCERYW=y              : CodeSourcery under Windows
    CONFIG_NSH_DHCPC=n                        : DHCP is disabled
    CONFIG_NSH_IPADDR=(10<<24|0<<16|0<<8|2)   : Target IP address 10.0.0.2
    CONFIG_NSH_DRIPADDR=(10<<24|0<<16|0<<8|1) : Host IP address 10.0.0.1

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

    7. This configuration requires that jumper JP22 be set to enable RS-232 operation.

  nsh2:
  -----

    This is an alternaitve NSH configuration.  One limitation of the STM3240G-EVAL
    board is that you cannot have both a UART-based NSH console and SDIO support.
    The nsh2 differs from the nsh configuration in the following ways:

    -CONFIG_STM32_USART3=y      : USART3 is disabled
    + CONFIG_STM32_USART3=n

    -CONFIG_STM32_SDIO=n        : SDIO is enabled
    +CONFIG_STM32_SDIO=y

    Logically, that is the only difference:  This configuration has SDIO (and
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

    3. This configuration requires that jumper JP22 be set to enable SDIO operation.

    4. In order to use SDIO without overruns, DMA must be used.  The STM32 F4
       has 192Kb of SRAM in two banks:  112Kb of "system" SRAM located at
       0x2000:0000 and 64Kb of "TCM" SRAM located at 0x1000:0000. It appears
       that you cannot perform DMA from TCM SRAM.  The work around that I have now
       is simply to omit the 64Kb of TCM SRAM from the heap so that all memory is
       allocated from System SRAM.  This is done by setting: 
       
       CONFIG_MM_REGIONS=1

       Then DMA works fine. The downside is, of course, is that we lose 64Kb
       of precious SRAM.

    5. Another SDIO/DMA issue.  This one is probably a software bug.  This is
       the bug as stated in the TODO list:

       "If you use a large I/O buffer to access the file system, then the
        MMCSD driver will perform multiple block SD transfers.  With DMA
        ON, this seems to result in CRC errors detected by the hardware
        during the transfer.  Workaround:  Use I/O buffers less the 1024
        bytes."

       For this reason, CONFIG_FTPD_DATABUFFERSIZE=512 appears in the defconfig
       file.

    6. Another DMA-related concern.  I see this statement in the reference
       manual:  "The burst configuration has to be selected in order to respect
       the AHB protocol, where bursts must not cross the 1 KB address boundary
       because the minimum address space that can be allocated to a single slave
       is 1 KB. This means that the 1 KB address boundary should not be crossed
       by a burst block transfer, otherwise an AHB error would be generated,
       that is not reported by the DMA registers."

       There is nothing in the DMA driver to prevent this now.

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
