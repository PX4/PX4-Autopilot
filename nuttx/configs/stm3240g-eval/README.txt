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
configured to output a pulse train using TIM4 CH2.  This pin is used by
FSMC is but is also connected to the Motor Control Connector (CN5) just
for this purpose:

  PD13 FSMC_A18 / MC_TIM4_CH2 pin 33 (EnB)

FSMC must be disabled in this case!  PD13 is available at:

  Daughterboard Extension Connector, CN3, pin 32 - available
  TFT LCD Connector, CN19, pin 17 -- not available without removing the LCD.
  Motor Control Connector CN15, pin 33 -- not available unless you bridge SB14.

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
  CONFIG_CAN_FIFOSIZE - The size of the circular buffer of CAN messages.
    Default: 8
  CONFIG_CAN_NPENDINGRTR - The size of the list of pending RTR requests.
    Default: 4

  CONFIG_STM32_CAN1 - Enable support for CAN1
  CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN1 is defined.
  CONFIG_STM32_CAN2 - Enable support for CAN1
  CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_STM32_CAN2 is defined.

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
  or DAC conversion.

	CONFIG_STM32_TIM1_PWM
	CONFIG_STM32_TIM2_PWM
	CONFIG_STM32_TIM3_PWM
	CONFIG_STM32_TIM4_PWM
	CONFIG_STM32_TIM5_PWM
	CONFIG_STM32_TIM8_PWM
	CONFIG_STM32_TIM9_PWM
	CONFIG_STM32_TIM10_PWM
	CONFIG_STM32_TIM11_PWM
	CONFIG_STM32_TIM12_PWM
	CONFIG_STM32_TIM13_PWM
	CONFIG_STM32_TIM14_PWM

	CONFIG_STM32_TIM1_ADC
	CONFIG_STM32_TIM2_ADC
	CONFIG_STM32_TIM3_ADC
	CONFIG_STM32_TIM4_ADC
	CONFIG_STM32_TIM5_ADC
	CONFIG_STM32_TIM6_ADC
	CONFIG_STM32_TIM7_ADC
	CONFIG_STM32_TIM8_ADC

	CONFIG_STM32_TIM1_DAC
	CONFIG_STM32_TIM2_DAC
	CONFIG_STM32_TIM3_DAC
	CONFIG_STM32_TIM4_DAC
	CONFIG_STM32_TIM5_DAC
	CONFIG_STM32_TIM6_DAC
	CONFIG_STM32_TIM7_DAC
	CONFIG_STM32_TIM8_DAC

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

    CONFIG_STM32_CODESOURCERYW=y              : CodeSourcery under Windows
    CONFIG_NSH_DHCPC=n                        : DHCP is disabled
    CONFIG_NSH_IPADDR=(10<<24|0<<16|0<<8|2)   : Target IP address 10.0.0.2
    CONFIG_NSH_DRIPADDR=(10<<24|0<<16|0<<8|1) : Host IP address 10.0.0.1

    NOTE:  This example assumes that a network is connected.  During its
    initialization, it will try to negotiate the link speed.  If you have
    no network connected when you reset the board, there will be a long
    delay (maybe 30 seconds?) before anything happens.  That is the timeout
    before the networking finally gives up and decides that no network is
    available.
