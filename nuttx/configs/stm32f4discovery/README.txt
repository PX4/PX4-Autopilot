README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM32F4 Discovery development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX buildroot Toolchain
  - stm32f4discovery-specific Configuration Options
  - LEDs
  - PWM
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
  2. The devkitARM GNU toolchain,
  3. Raisonance GNU toolchain, or
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the CodeSourcery toolchain for Windows.  To use
  the devkitARM, Raisonance GNU, or NuttX buildroot toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_STM32_DEVKITARM=y      : devkitARM under Windows
    CONFIG_STM32_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you change the default toolchain, then you may also have to modify the PATH in
  the setenv.h file if your make cannot find the tools.

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

     Support has been added for making dependencies with the windows-native toolchains.
     That support can be enabled by modifying your Make.defs file as follows:

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

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
     ./configure.sh stm32f4discovery/<sub-dir>

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

LEDs
====

The stm32f4discovery board has four LEDs; green, organge, red and blue on the
board.. These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related\
events as follows:

	SYMBOL				Meaning					LED1*	LED2	LED3	LED4
                                                green   orange  red     blue
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

The stm32f4discovery has no real on-board PWM devices, but the board can be
configured to output a pulse train using TIM4 CH2 on PD3.  This pin is
available next to the audio jack.

stm32f4discovery-specific Configuration Options
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

	   CONFIG_ARCH_BOARD=stm32f4discovery (for the stm32f4discovery development board)

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_STM32F4_DISCOVERY=y

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

  JTAG Enable settings (by default only SW-DP is enabled):

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

  STM3240xxx CAN Configuration

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

  STM3240xxx SPI Configuration

	CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
	  support. Non-interrupt-driven, poll-waiting is recommended if the
	  interrupt rate would be to high in the interrupt driven case.
	CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
	  Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

  STM3240xxx DMA Configuration

	CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32_SDIO
	  and CONFIG_STM32_DMA2.
	CONFIG_SDIO_PRI - Select SDIO interrupt prority.  Default: 128
	CONFIG_SDIO_DMAPRIO - Select SDIO DMA interrupt priority. 
	  Default:  Medium
	CONFIG_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
	  4-bit transfer mode.

Configurations
==============

Each stm32f4discovery configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh stm32f4discovery/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    CONFIG_STM32_CODESOURCERYL=y              : CodeSourcery under Linux / Mac OS X

    NOTES:
    1. This example supports the PWM test (apps/examples/pwm) but this must
       be manually enabled by selecting:

       CONFIG_PWM=y              : Enable the generic PWM infrastructure
       CONFIG_STM32_TIM4_PWM=y   : Use TIM4 to generate PWM output

       See also apps/examples/README.txt

       Special PWM-only debug options:

       CONFIG_DEBUG_PWM

