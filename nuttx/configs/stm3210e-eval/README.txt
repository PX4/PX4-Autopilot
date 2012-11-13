README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM3210E-EVAL development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - DFU and JTAG
  - OpenOCD
  - LEDs
  - Temperature Sensor
  - RTC
  - FSMC SRAM
  - STM3210E-EVAL-specific Configuration Options
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

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery, devkitARM or Raisonance GNU toolchain, you simply need to
  add one of the following configuration options to your .config (or defconfig)
  file:

    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux
    CONFIG_STM32_DEVKITARM=y      : devkitARM under Windows
    CONFIG_STM32_RAISONANCE=y     : Raisonance RIDE7 under Windows
    CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_STM32_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

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
     ./configure.sh stm3210e-eval/<sub-dir>

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

DFU and JTAG
============

  Enbling Support for the DFU Bootloader
  --------------------------------------
  The linker files in these projects can be configured to indicate that you
  will be loading code using STMicro built-in USB Device Firmware Upgrade (DFU)
  loader or via some JTAG emulator.  You can specify the DFU bootloader by
  adding the following line:

    CONFIG_STM32_DFU=y

  to your .config file. Most of the configurations in this directory are set
  up to use the DFU loader.

  If CONFIG_STM32_DFU is defined, the code will not be positioned at the beginning
  of FLASH (0x08000000) but will be offset to 0x08003000.  This offset is needed
  to make space for the DFU loader and 0x08003000 is where the DFU loader expects
  to find new applications at boot time.  If you need to change that origin for some
  other bootloader, you will need to edit the file(s) ld.script.dfu for the
  configuration.

  The DFU SE PC-based software is available from the STMicro website,
  http://www.st.com.  General usage instructions:
  
  1. Convert the NuttX Intel Hex file (nuttx.hex) into a special DFU
     file (nuttx.dfu)... see below for details.
  2. Connect the STM3210E-EVAL board to your computer using a USB
     cable.
  3. Start the DFU loader on the STM3210E-EVAL board.  You do this by
     resetting the board while holding the "Key" button.  Windows should
     recognize that the DFU loader has been installed.
  3. Run the DFU SE program to load nuttx.dfu into FLASH.

  What if the DFU loader is not in FLASH?  The loader code is available
  inside of the Demo dirctory of the USBLib ZIP file that can be downloaded
  from the STMicro Website.  You can build it using RIDE (or other toolchains);
  you will need a JTAG emulator to burn it into FLASH the first time.

  In order to use STMicro's built-in DFU loader, you will have to get
  the NuttX binary into a special format with a .dfu extension.  The
  DFU SE PC_based software installation includes a file "DFU File Manager"
  conversion program that a file in Intel Hex format to the special DFU
  format.  When you successfully build NuttX, you will find a file called
  nutt.hex in the top-level directory.  That is the file that you should
  provide to the DFU File Manager.  You will end up with a file called
  nuttx.dfu that you can use with the STMicro DFU SE program.

  Enabling JTAG
  -------------
  If you are not using the DFU, then you will probably also need to enable
  JTAG support.  By default, all JTAG support is disabled but there NuttX
  configuration options to enable JTAG in various different ways.

  These configurations effect the setting of the SWJ_CFG[2:0] bits in the AFIO
  MAPR register.  These bits are used to configure the SWJ and trace alternate function I/Os. The SWJ (SerialWire JTAG) supports JTAG or SWD access to the
  Cortex debug port.  The default state in this port is for all JTAG support
  to be disable.

  CONFIG_STM32_JTAG_FULL_ENABLE - sets SWJ_CFG[2:0] to 000 which enables full
    SWJ (JTAG-DP + SW-DP)

  CONFIG_STM32_JTAG_NOJNTRST_ENABLE - sets SWJ_CFG[2:0] to 001 which enable
    full SWJ (JTAG-DP + SW-DP) but without JNTRST.

  CONFIG_STM32_JTAG_SW_ENABLE - sets SWJ_CFG[2:0] to 010 which would set JTAG-DP
    disabled and SW-DP enabled

  The default setting (none of the above defined) is SWJ_CFG[2:0] set to 100
  which disable JTAG-DP and SW-DP.

OpenOCD
=======

I have also used OpenOCD with the STM3210E-EVAL.  In this case, I used
the Olimex USB ARM OCD.  See the script in configs/stm3210e-eval/tools/oocd.sh
for more information.  Using the script:

1) Start the OpenOCD GDB server

   cd <nuttx-build-directory>
   configs/stm3210e-eval/tools/oocd.sh $PWD

2) Load Nuttx

   cd <nuttx-built-directory>
   arm-none-eabi-gdb nuttx
   gdb> target remote localhost:3333
   gdb> mon reset
   gdb> mon halt
   gdb> load nuttx

3) Running NuttX

   gdb> mon reset
   gdb> c

LEDs
====

The STM3210E-EVAL board has four LEDs labeled LD1, LD2, LD3 and LD4 on the
board.. These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
events as follows:

    SYMBOL           Meaning                 LED1* LED2  LED3  LED4
    ---------------- ----------------------- ----- ----- ----- -----
    LED_STARTED      NuttX has been started  ON    OFF   OFF   OFF
    LED_HEAPALLOCATE Heap has been allocated OFF   ON    OFF   OFF
    LED_IRQSENABLED  Interrupts enabled      ON    ON    OFF   OFF
    LED_STACKCREATED Idle stack created      OFF   OFF   ON    OFF
    LED_INIRQ        In an interrupt**       ON    N/C   N/C   OFF
    LED_SIGNAL       In a signal handler***  N/C   ON    N/C   OFF
    LED_ASSERTION    An assertion failed     ON    ON    N/C   OFF
    LED_PANIC        The system has crashed  N/C   N/C   N/C   ON
    LED_IDLE         STM32 is is sleep mode  (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

Temperature Sensor
==================

Support for the on-board LM-75 temperature sensor is available.  This supported
has been verified, but has not been included in any of the available the
configurations.  To set up the temperature sensor, add the following to the
NuttX configuration file

  CONFIG_I2C=y
  CONFIG_I2C_LM75=y

Then you can implement logic like the following to use the temperature sensor:

  #include <nuttx/sensors/lm75.h>
  #include <arch/board/board.h>

  ret =  stm32_lm75initialize("/dev/temp");       /* Register the temperature sensor */
  fd = open("/dev/temp", O_RDONLY);               /* Open the temperature sensor device */
  ret = ioctl(fd, SNIOC_FAHRENHEIT, 0);           /* Select Fahrenheit */
  bytesread = read(fd, buffer, 8*sizeof(b16_t));  /* Read temperature samples */

More complex temperature sensor operations are also available.  See the IOCTAL
commands enumerated in include/nuttx/sensors/lm75.h.  Also read the descriptions
of the stm32_lm75initialize() and stm32_lm75attach() interfaces in the
arch/board/board.h file (sames as configs/stm3210e-eval/include/board.h).

RTC
===

  The STM32 RTC may configured using the following settings.

    CONFIG_RTC - Enables general support for a hardware RTC. Specific
      architectures may require other specific settings.
    CONFIG_RTC_HIRES - The typical RTC keeps time to resolution of 1
      second, usually supporting a 32-bit time_t value.  In this case,
      the RTC is used to &quot;seed&quot; the normal NuttX timer and the
      NuttX timer provides for higher resoution time. If CONFIG_RTC_HIRES
      is enabled in the NuttX configuration, then the RTC provides higher
      resolution time and completely replaces the system timer for purpose of
      date and time.
      CONFIG_RTC_FREQUENCY - If CONFIG_RTC_HIRES is defined, then the
      frequency of the high resolution RTC must be provided.  If CONFIG_RTC_HIRES
      is not defined, CONFIG_RTC_FREQUENCY is assumed to be one.
    CONFIG_RTC_ALARM - Enable if the RTC hardware supports setting of an alarm. 
      A callback function will be executed when the alarm goes off

  In hi-res mode, the STM32 RTC operates only at 16384Hz.  Overflow interrupts
  are handled when the 32-bit RTC counter overflows every 3 days and 43 minutes.
  A BKP register is incremented on each overflow interrupt creating, effectively,
  a 48-bit RTC counter.

  In the lo-res mode, the RTC operates at 1Hz.  Overflow interrupts are not handled
  (because the next overflow is not expected until the year 2106.

   WARNING:  Overflow interrupts are lost whenever the STM32 is powered down.  The
   overflow interrupt may be lost even if the STM32 is powered down only momentarily.
   Therefore hi-res solution is only useful in systems where the power is always on.

FSMC SRAM
=========

The 8-Mbit SRAM is connected to the STM32 at PG10 which will be FSMC_NE3, Bank1
SRAM3.  This memory will appear at address 0x68000000.

The on-board SRAM can be configured by setting

  CONFIG_STM32_FSMC=y                         : Enables the FSMC
  CONFIG_STM32_FSMC_SRAM=y                    : Enable external SRAM support
  CONFIG_HEAP2_BASE=0x68000000                : SRAM will be located at 0x680000000
  CONFIG_HEAP2_SIZE=1048576                   : The size of the SRAM is 1Mbyte
  CONFIG_MM_REGIONS=2                         : There will be two memory regions
                                              : in the heap

STM3210E-EVAL-specific Configuration Options
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

       CONFIG_ARCH_CHIP_STM32F103ZET6

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n
 
    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=stm3210e_eval (for the STM3210E-EVAL development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_STM3210E_EVAL=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The STM32F103Z supports interrupt prioritization

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
    AHB
    ---
    CONFIG_STM32_DMA1
    CONFIG_STM32_DMA2
    CONFIG_STM32_CRC
    CONFIG_STM32_FSMC
    CONFIG_STM32_SDIO

    APB1
    ----
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_TIM5
    CONFIG_STM32_TIM6
    CONFIG_STM32_TIM7
    CONFIG_STM32_WWDG
    CONFIG_STM32_IWDG
    CONFIG_STM32_SPI2
    CONFIG_STM32_SPI4
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_UART4
    CONFIG_STM32_UART5
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_USB
    CONFIG_STM32_CAN1
    CONFIG_STM32_BKP
    CONFIG_STM32_PWR
    CONFIG_STM32_DAC1
    CONFIG_STM32_DAC2
    CONFIG_STM32_USB

    APB2
    ----
    CONFIG_STM32_ADC1
    CONFIG_STM32_ADC2
    CONFIG_STM32_TIM1
    CONFIG_STM32_SPI1
    CONFIG_STM32_TIM8
    CONFIG_STM32_USART1
    CONFIG_STM32_ADC3

  Timer and I2C devices may need to the following to force power to be applied
  unconditionally at power up.  (Otherwise, the device is powered when it is
  initialized).

    CONFIG_STM32_FORCEPOWER

  Timer devices may be used for different purposes.  One special purpose is
  to generate modulated outputs for such things as motor control.  If CONFIG_STM32_TIMn
  is defined (as above) then the following may also be defined to indicate that
  the timer is intended to be used for pulsed output modulation, ADC conversion,
  or DAC conversion.  Note that ADC/DAC require two definition:  Not only do you have
  to assign the timer (n) for used by the ADC or DAC, but then you also have to
  configure which ADC or DAC (m) it is assigned to.

    CONFIG_STM32_TIMn_PWM   Reserve timer n for use by PWM, n=1,..,8
    CONFIG_STM32_TIMn_ADC   Reserve timer n for use by ADC, n=1,..,8
    CONFIG_STM32_TIMn_ADCm  Reserve timer n to trigger ADCm, n=1,..,8, m=1,..,3
    CONFIG_STM32_TIMn_DAC   Reserve timer n for use by DAC, n=1,..,8
    CONFIG_STM32_TIMn_DACm  Reserve timer n to trigger DACm, n=1,..,8, m=1,..,2

  For each timer that is enabled for PWM usage, we need the following additional
  configuration settings:

    CONFIG_STM32_TIMx_CHANNEL - Specifies the timer output channel {1,..,4}
 
  NOTE: The STM32 timers are each capable of generating different signals on
  each of the four channels with different duty cycles.  That capability is
  not supported by this driver:  Only one output channel per timer.

  Alternate pin mappings.  The STM3210E-EVAL board requires only CAN1 remapping
  On the STM3210E-EVAL board pin PB9 is wired as TX and pin PB8 is wired as RX.
  Which then makes the proper connection through the CAN transiver SN65HVD230 
  out to the CAN D-type 9-pn male connector where pin 2 is CANL and pin 7 is CANH.

    CONFIG_STM32_TIM1_FULL_REMAP
    CONFIG_STM32_TIM1_PARTIAL_REMAP
    CONFIG_STM32_TIM2_FULL_REMAP
    CONFIG_STM32_TIM2_PARTIAL_REMAP_1
    CONFIG_STM32_TIM2_PARTIAL_REMAP_2
    CONFIG_STM32_TIM3_FULL_REMAP
    CONFIG_STM32_TIM3_PARTIAL_REMAP
    CONFIG_STM32_TIM4_REMAP
    CONFIG_STM32_USART1_REMAP
    CONFIG_STM32_USART2_REMAP
    CONFIG_STM32_USART3_FULL_REMAP
    CONFIG_STM32_USART3_PARTIAL_REMAP
    CONFIG_STM32_SPI1_REMAP
    CONFIG_STM32_SPI3_REMAP
    CONFIG_STM32_I2C1_REMAP
    CONFIG_STM32_CAN1_REMAP1
    CONFIG_STM32_CAN1_REMAP2
    CONFIG_STM32_CAN2_REMAP

  JTAG Enable settings (by default JTAG-DP and SW-DP are disabled):
    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM32F103Z specific device driver settings

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

  STM3210E-EVAL CAN Configuration

    CONFIG_CAN - Enables CAN support (one or both of CONFIG_STM32_CAN1 or
      CONFIG_STM32_CAN2 must also be defined)
    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
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

  STM3210E-EVAL LCD Hardware Configuration

    CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape"
      support. Default is this 320x240 "landscape" orientation
      (this setting is informative only... not used).
    CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait"
      orientation support.  In this orientation, the STM3210E-EVAL's
      LCD ribbon cable is at the bottom of the display. Default is
      320x240 "landscape" orientation.
    CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse
      portrait" orientation support.  In this orientation, the
      STM3210E-EVAL's LCD ribbon cable is at the top of the display.
      Default is 320x240 "landscape" orientation.
    CONFIG_LCD_BACKLIGHT - Define to support a backlight.
    CONFIG_LCD_PWM - If CONFIG_STM32_TIM1 is also defined, then an
      adjustable backlight will be provided using timer 1 to generate
      various pulse widthes.  The granularity of the settings is
      determined by CONFIG_LCD_MAXPOWER.  If CONFIG_LCD_PWM (or
      CONFIG_STM32_TIM1) is not defined, then a simple on/off backlight
      is provided.
    CONFIG_LCD_RDSHIFT - When reading 16-bit gram data, there appears
      to be a shift in the returned data.  This value fixes the offset.
      Default 5.

    The LCD driver dynamically selects the LCD based on the reported LCD
    ID value.  However, code size can be reduced by suppressing support for
    individual LCDs using:

    CONFIG_STM32_AM240320_DISABLE
    CONFIG_STM32_SPFD5408B_DISABLE
    CONFIG_STM32_R61580_DISABLE

Configurations
==============

Each STM3210E-EVAL configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh stm3210e-eval/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  buttons:
  --------

    Uses apps/examples/buttons to exercise STM3210E-EVAL buttons and
    button interrupts.
 
    CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows

  composite
  ---------

    This configuration exercises a composite USB interface consisting
    of a CDC/ACM device and a USB mass storage device.  This configuration
    uses apps/examples/composite.

  nsh and nsh2:
  ------------
    Configure the NuttShell (nsh) located at examples/nsh.

    Differences between the two NSH configurations:

    =========== ======================= ================================
                nsh                     nsh2
    =========== ======================= ================================
    Toolchain:  NuttX buildroot for     Codesourcery for Windows (1)
                Linux or Cygwin (1,2)
    ----------- ----------------------- --------------------------------
    Loader:     DfuSe                   DfuSe
    ----------- ----------------------- --------------------------------
    Serial      Debug output: USART1    Debug output: USART1
    Console:    NSH output:   USART1    NSH output:   USART1 (3)
    ----------- ----------------------- --------------------------------
    I2C         No                      I2C1
    ----------- ----------------------- --------------------------------
    microSD     Yes                     Yes
    Support
    ----------- ----------------------- --------------------------------
    FAT FS      CONFIG_FAT_LCNAME=y     CONFIG_FAT_LCNAME=y
    Config      CONFIG_FAT_LFN=n        CONFIG_FAT_LFN=y (4)
    ----------- ----------------------- --------------------------------
    Support for No                      Yes
    Built-in
    Apps
    ----------- ----------------------- --------------------------------
    Built-in    None                    apps/examples/nx
    Apps                                apps/examples/nxhello
                                        apps/examples/usbstorage (5)
                                        apps/system/i2c
    =========== ======================= ================================

    (1) You will probably need to modify nsh/setenv.sh or nsh2/setenv.sh
        to set up the correct PATH variable for whichever toolchain you
        may use.
    (2) Since DfuSe is assumed, this configuration may only work under
        Cygwin without modification.
    (3) When any other device other than /dev/console is used for a user
        interface, (1) linefeeds (\n) will not be expanded to carriage return
        / linefeeds \r\n). You will need to configure your terminal program
        to account for this. And (2) input is not automatically echoed so
        you will have to turn local echo on.
    (4) Microsoft holds several patents related to the design of
        long file names in the FAT file system.  Please refer to the
        details in the top-level COPYING file.  Please do not use FAT
        long file name unless you are familiar with these patent issues.
    (5) When built as an NSH add-on command (CONFIG_EXAMPLES_USBMSC_BUILTIN=y),
        Caution should be used to assure that the SD drive is not in use when
        the USB storage device is configured.  Specifically, the SD driver
        should be unmounted like:

        nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard # Card is mounted in NSH
        ...
        nsh> umount /mnd/sdcard                    # Unmount before connecting USB!!!
        nsh> msconn                                # Connect the USB storage device
        ...
        nsh> msdis                                 # Disconnect USB storate device
        nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard # Restore the mount

        Failure to do this could result in corruption of the SD card format.

    The nsh2 contains support for some built-in applications that can be
    enabled by make some additional minor changes:

    (1) examples/can.  The CAN test example can be enabled by changing the
        following settings in nsh2/defconfig:

        CONFIG_CAN=y             # Enable CAN "upper-half" driver support
        CONFIG_STM32_CAN1=y      # Enable STM32 CAN1 "lower-half" driver support

        The default CAN settings may need to change in your board board
        configuration:

        CONFIG_CAN_EXTID=y       # Support extended IDs
        CONFIG_CAN1_BAUD=250000  # Bit rate: 250 KHz
        CONFIG_CAN_TSEG1=12      # 80% sample point
        CONFIG_CAN_TSEG2=3
  nx:
  ---
    An example using the NuttX graphics system (NX).  This example
    focuses on general window controls, movement, mouse and keyboard
    input.

      CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
      CONFIG_LCD_RPORTRAIT=y        : 240x320 reverse portrait

  nxconsole:
  ----------
    This is yet another NSH configuration.  This NSH configuration differs
    from the other, however, in that it uses the NxConsole driver to host
    the NSH shell.

    Some of the differences in this configuration include these settings
    in the defconfig file:

    These select NX Multi-User mode:

      CONFG_NX_MULTIUSER=y
      CONFIG_DISABLE_MQUEUE=n
 
    The following definition in the defconfig file to enables the NxConsole
    driver:

      CONFIG_NXCONSOLE=y

    The appconfig file selects examples/nxconsole instead of examples/nsh:

      CONFIGURED_APPS += examples/nxconsole

    Other configuration settings:

      CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin
      CONFIG_LCD_LANDSCAPE=y        : 320x240 landscape

  nxlines:
  ------
    Another example using the NuttX graphics system (NX).   This
    example focuses on placing lines on the background in various
    orientations.

      CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
      CONFIG_LCD_RPORTRAIT=y        : 240x320 reverse portrait

  nxtext:
  ------
    Another example using the NuttX graphics system (NX).   This
    example focuses on placing text on the background while pop-up
    windows occur.  Text should continue to update normally with
    or without the popup windows present.

      CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin
      CONFIG_LCD_RPORTRAIT=y        : 240x320 reverse portrait

    NOTE:  When I tried building this example with the CodeSourcery
    tools, I got a hardfault inside of its libgcc.  I haven't
    retested since then, but beware if you choose to change the
    toolchain.

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    examples/ostest.  By default, this project assumes that you are
    using the DFU bootloader.

      CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin

  pm:
  --
    This is a configuration that is used to test STM32 power management, i.e.,
    to test that the board can go into lower and lower states of power usage
    as a result of inactivity.  This configuration is based on the nsh2
    configuration with modifications for testing power management.  This
    configuration should provide some guideline for power management in your
    STM32 application.

      CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows

    CONFIG_PM_CUSTOMINIT and CONFIG_IDLE_CUSTOM are necessary parts of the
    PM configuration:

      CONFIG_PM_CUSTOMINIT=y

    CONFIG_PM_CUSTOMINIT moves the PM initialization from arch/arm/src/stm32/stm32_pminitialiaze.c
    to configs/stm3210-eval/src/up_pm.c.  This allows us to support board-
    specific PM initialization.
    
      CONFIG_IDLE_CUSTOM=y

    The bulk of the PM activities occur in the IDLE loop.  The IDLE loop is
    special because it is what runs when there is no other task running.  Therefore
    when the IDLE executes, we can be assure that nothing else is going on; this
    is the ideal condition for doing reduced power management.

    The configuration CONFIG_IDLE_CUSTOM allows us to "steal" the normal STM32
    IDLE loop (of arch/arm/src/stm32/stm32_idle.c) and replace this with our own
    custom IDLE loop (at configs/stm3210-eval/src/up_idle.c).

    Here are some additional things to note in the configuration:

      CONFIG_PM_BUTTONS=y
    
    CONFIG_PM_BUTTONS enables button support for PM testing.  Buttons can drive
    EXTI interrupts and EXTI interrrupts can be used to wakeup for certain reduced
    power modes (STOP mode).  The use of the buttons here is for PM testing purposes
    only; buttons would normally be part the application code and CONFIG_PM_BUTTONS
    would not be defined.

      CONFIG_RTC_ALARM=y

    The RTC alarm is used to wake up from STOP mode and to transition to
    STANDBY mode.  This used of the RTC alarm could conflict with other uses of
    the RTC alarm in your application.

  RIDE
  ----
    This configuration builds a trivial bring-up binary.  It is
    useful only because it words with the RIDE7 IDE and R-Link debugger.

      CONFIG_STM32_RAISONANCE=y     : Raisonance RIDE7 under Windows

  usbserial:
  ---------
    This configuration directory exercises the USB serial class
    driver at examples/usbserial.  See examples/README.txt for
    more information.

      CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin

    USB debug output can be enabled as by changing the following
    settings in the configuration file:

      -CONFIG_DEBUG=n
      -CONFIG_DEBUG_VERBOSE=n
      -CONFIG_DEBUG_USB=n
      +CONFIG_DEBUG=y
      +CONFIG_DEBUG_VERBOSE=y
      +CONFIG_DEBUG_USB=y

      -CONFIG_EXAMPLES_USBSERIAL_TRACEINIT=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACECLASS=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER=n
      -CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS=n
      +CONFIG_EXAMPLES_USBSERIAL_TRACEINIT=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACECLASS=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACETRANSFERS=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACECONTROLLER=y
      +CONFIG_EXAMPLES_USBSERIAL_TRACEINTERRUPTS=y

    By default, the usbserial example uses the Prolific PL2303
    serial/USB converter emulation.  The example can be modified
    to use the CDC/ACM serial class by making the following changes
    to the configuration file:

      -CONFIG_PL2303=y
      +CONFIG_PL2303=n

      -CONFIG_CDCACM=n
      +CONFIG_CDCACM=y

    The example can also be converted to use the alternative
    USB serial example at apps/examples/usbterm by changing the 
    following:

      -CONFIGURED_APPS += examples/usbserial
      +CONFIGURED_APPS += examples/usbterm

    In either the original appconfig file (before configuring)
    or in the final apps/.config file (after configuring).

  usbstorage:
  ----------
    This configuration directory exercises the USB mass storage
    class driver at examples/usbstorage.  See examples/README.txt for
    more information.

      CONFIG_STM32_BUILDROOT=y      : NuttX buildroot under Linux or Cygwin

