README
======

This README discusses issues unique to NuttX configurations for the
STMicro STM32F4 Discovery development board.

Contents
========

  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI "buildroot" Toolchain
  - NuttX OABI "buildroot" Toolchain
  - NXFLAT Toolchain
  - LEDs
  - PWM
  - UARTs
  - Timer Inputs/Outputs
  - FPU
  - FSMC SRAM
  - SSD1289
  - STM32F4Discovery-specific Configuration Options
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

  All testing has been conducted using the CodeSourcery toolchain for Windows.  To use
  the Atollic, devkitARM, Raisonance GNU, or NuttX buildroot toolchain, you simply need to
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

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

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

  Also, the Atollic toolchains are the only toolchains that have built-in support for
  the FPU in these configurations.  If you plan to use the Cortex-M4 FPU, you will
  need to use the Atollic toolchain for now.  See the FPU section below for more
  information.

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
     ./configure.sh STM32F4Discovery/<sub-dir>

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

LEDs
====

The STM32F4Discovery board has four LEDs; green, organge, red and blue on the
board.. These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
events as follows:

  SYMBOL                Meaning                 LED1*    LED2     LED3     LED4
                                                green    orange   red      blue
  -------------------  -----------------------  -------  -------  -------  ------
  LED_STARTED          NuttX has been started   ON       OFF      OFF      OFF
  LED_HEAPALLOCATE     Heap has been allocated  OFF      ON       OFF      OFF
  LED_IRQSENABLED      Interrupts enabled       ON       ON       OFF      OFF
  LED_STACKCREATED     Idle stack created       OFF      OFF      ON       OFF
  LED_INIRQ            In an interrupt**        ON       N/C      N/C      OFF
  LED_SIGNAL           In a signal handler***   N/C      ON       N/C      OFF
  LED_ASSERTION        An assertion failed      ON       ON       N/C      OFF
  LED_PANIC            The system has crashed   N/C      N/C      N/C      ON
  LED_IDLE             STM32 is is sleep mode   (Optional, not used)

  * If LED1, LED2, LED3 are statically on, then NuttX probably failed to boot
    and these LEDs will give you some indication of where the failure was
 ** The normal state is LED3 ON and LED1 faintly glowing.  This faint glow
    is because of timer interupts that result in the LED being illuminated
    on a small proportion of the time.
*** LED2 may also flicker normally if signals are processed.

PWM
===

The STM32F4Discovery has no real on-board PWM devices, but the board can be
configured to output a pulse train using TIM4 CH2 on PD3.  This pin is
available next to the audio jack.

UART
====

UART/USART PINS
---------------

USART1
  CK      PA8
  CTS     PA11*
  RTS     PA12*
  RX      PA10*, PB7
  TX      PA9*, PB6*
USART2
  CK      PA4*, PD7
  CTS     PA0*, PD3
  RTS     PA1, PD4*
  RX      PA3, PD6
  TX      PA2, PD5*
USART3
  CK      PB12, PC12*, PD10
  CTS     PB13, PD11
  RTS     PB14, PD12*
  RX      PB11, PC11, PD9
  TX      PB10*, PC10*, PD8
UART4
  RX      PA1, PC11
  TX      PA0*, PC10*
UART5
  RX      PD2
  TX      PC12*
USART6
  CK      PC8, PG7**
  CTS     PG13**, PG15**
  RTS     PG12**, PG8**
  RX      PC7*, PG9**
  TX      PC6, PG14**

 * Indicates pins that have other on-board functions and should be used only
   with care (See table 5 in the STM32F4Discovery User Guide).  The rest are
   free I/O pins.
** Port G pins are not supported by the MCU
 
Default USART/UART Configuration
--------------------------------
 
USART2 is enabled in all configurations (see */defconfig).  RX and TX are
configured on pins PA3 and PA2, respectively (see include/board.h).

Timer Inputs/Outputs
====================

TIM1
  CH1     PA8, PE9
  CH2     PA9*, PE11
  CH3     PA10*, PE13
  CH4     PA11*, PE14
TIM2
  CH1     PA0*, PA15, PA5*
  CH2     PA1, PB3*
  CH3     PA2, PB10*
  CH4     PA3, PB11
TIM3
  CH1     PA6*, PB4, PC6
  CH2     PA7*, PB5, PC7*
  CH3     PB0, PC8
  CH4     PB1, PC9
TIM4
  CH1     PB6*, PD12*
  CH2     PB7, PD13*
  CH3     PB8, PD14*
  CH4     PB9*, PD15*
TIM5
  CH1     PA0*, PH10**
  CH2     PA1, PH11**
  CH3     PA2, PH12**
  CH4     PA3, PI0
TIM8
  CH1     PC6, PI5
  CH2     PC7*, PI6
  CH3     PC8, PI7
  CH4     PC9, PI2
TIM9
  CH1     PA2, PE5
  CH2     PA3, PE6
TIM10
  CH1     PB8, PF6
TIM11
  CH1     PB9*, PF7
TIM12
  CH1     PH6**, PB14
  CH2     PC15, PH9**
TIM13
  CH1     PA6*, PF8
TIM14
  CH1     PA7*, PF9

 * Indicates pins that have other on-board functions and should be used only
   with care (See table 5 in the STM32F4Discovery User Guide).  The rest are
   free I/O pins.
** Port H pins are not supported by the MCU

Quadrature Encode Timer Inputs
------------------------------

If enabled (by setting CONFIG_QENCODER=y), then quadrature encoder will
use either TIM2 or TIM8 (see nsh/defconfig).  If TIM2 is selected, the input 
pins PA15 and PA1 for CH1 and CH2, respectively).  If TIM8 is selected, then
PC6 and PI5 will be used for CH1 and CH2  (see include board.h for pin
definitions).

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

  ifeq ($(CONFIG_STM32_ATOLLIC_LITE),y)
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

FSMC SRAM
=========

On-board SRAM
-------------
The STM32F4Discovery has no on-board SRAM.  The information here is only for
reference in case you choose to add some.

Configuration Options
---------------------
Internal SRAM is available in all members of the STM32 family. The F4 family
also contains internal CCM SRAM.  This SRAM is different because it cannot
be used for DMA.  So if DMA needed, then the following should be defined
to exclude CCM SRAM from the heap:

  CONFIG_STM32_CCMEXCLUDE    : Exclude CCM SRAM from the HEAP

In addition to internal SRAM, SRAM may also be available through the FSMC.
In order to use FSMC SRAM, the following additional things need to be
present in the NuttX configuration file:

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
There are 4 possible SRAM configurations:

  Configuration 1. System SRAM (only)
                   CONFIG_MM_REGIONS == 1
                   CONFIG_STM32_FSMC_SRAM NOT defined
                   CONFIG_STM32_CCMEXCLUDE defined
  Configuration 2. System SRAM and CCM SRAM
                   CONFIG_MM_REGIONS == 2
                   CONFIG_STM32_FSMC_SRAM NOT defined
                   CONFIG_STM32_CCMEXCLUDE NOT defined
  Configuration 3. System SRAM and FSMC SRAM
                   CONFIG_MM_REGIONS == 2
                   CONFIG_STM32_FSMC_SRAM defined
                   CONFIG_STM32_CCMEXCLUDE defined
  Configuration 4. System SRAM, CCM SRAM, and FSMC SRAM
                   CONFIG_MM_REGIONS == 3
                   CONFIG_STM32_FSMC_SRAM defined
                   CONFIG_STM32_CCMEXCLUDE NOT defined

Configuration Changes
---------------------

Below are all of the configuration changes that I had to make to configs/stm3240g-eval/nsh2
in order to successfully build NuttX using the Atollic toolchain WITH FPU support:

  -CONFIG_ARCH_FPU=n              : Enable FPU support
  +CONFIG_ARCH_FPU=y

  -CONFIG_STM32_CODESOURCERYW=y   : Disable the CodeSourcery toolchain
  +CONFIG_STM32_CODESOURCERYW=n

  -CONFIG_STM32_ATOLLIC_LITE=n   : Enable *one* the Atollic toolchains
   CONFIG_STM32_ATOLLIC_PRO=n
  -CONFIG_STM32_ATOLLIC_LITE=y   : The "Lite" version
   CONFIG_STM32_ATOLLIC_PRO=n    : The "Pro" version

  -CONFIG_INTELHEX_BINARY=y       : Suppress generation FLASH download formats
  +CONFIG_INTELHEX_BINARY=n       : (Only necessary with the "Lite" version)

  -CONFIG_HAVE_CXX=y              : Suppress generation of C++ code
  +CONFIG_HAVE_CXX=n              : (Only necessary with the "Lite" version)

See the section above on Toolchains, NOTE 2, for explanations for some of
the configuration settings.  Some of the usual settings are just not supported
by the "Lite" version of the Atollic toolchain.

SSD1289
=======

I purchased an LCD display on eBay from china.  The LCD is 320x240 RGB565 and
is based on an SSD1289 LCD controller and an XPT2046 touch IC.  The pin out
from the 2x16 connect on the LCD is labeled as follows:

LCD CONNECTOR:          SSD1289 MPU INTERFACE PINS:

   +------+------+      DEN     I  Display enable pin
1  | GND  | 3V3  |  2   VSYNC   I  Frame synchronization signal
   +------+------+      HSYNC   I  Line synchronization signal
3  | D1   | D0   |  4   DOTCLK  I  Dot clock and OSC source
   +------+------+      DC      I  Data or command
5  | D3   | D2   |  6   E (~RD) I  Enable/Read strobe
   +------+------+      R (~WR) I  Read/Write strobe
7  | D5   | D4   |  8   D0-D17  IO For parallel mode, 8/9/16/18 bit interface
   +------+------+      WSYNC   O  RAM write synchronizatin output
9  | D7   | D6   | 10   ~RES    I  System reset
   +------+------+      ~CS     I  Chip select of serial interface
11 | D9   | D8   | 12   SCK     I  Clock of serial interface
   +------+------+      SDI     I  Data input in serial mode
13 | D11  | D10  | 14   SDO     O  Data output in serial moce
   +------+------+
15 | D13  | D12  | 16
   +------+------+
17 | D15  | D14  | 18
   +------+------+
19 | RS   | CS   | 20
   +------+------+
21 | RD   | WR   | 22  NOTES:
   +------+------+
23 |BL_CNT|RESET | 24  BL_CNT is the PWM backlight level control.
   +------+------+
25 |TP_RQ |TP_S0 | 26  These pins are for the touch panel: TP_REQ
   +------+------+     TP_S0, TP_SI, TP_SCX, and TP_CS
27 | NC   |TP_SI | 28
   +------+------+
29 | NC   |TP_SCX| 30
   +------+------+
31 | NC   |TP_CS | 32
   +------+------+

MAPPING TO STM32 F4:

  ---------------- -------------- ----------------------------------
   STM32 FUNCTION  LCD PIN       STM32F4Discovery PIN
  ---------------- -------------- ----------------------------------
   FSMC_D0          D0     pin 4   PD14 P1 pin 46 Conflict (Note 1)
   FSMC_D1          D1     pin 3   PD15 P1 pin 47 Conflict (Note 2)
   FSMC_D2          D2     pin 6   PD0  P2 pin 36 Free I/O
   FSMC_D3          D3     pin 5   PD1  P2 pin 33 Free I/O
   FSMC_D4          D4     pin 8   PE7  P1 pin 25 Free I/O
   FSMC_D5          D5     pin 7   PE8  P1 pin 26 Free I/O
   FSMC_D6          D6     pin 10  PE9  P1 pin 27 Free I/O
   FSMC_D7          D7     pin 9   PE10 P1 pin 28 Free I/O
   FSMC_D8          D8     pin 12  PE11 P1 pin 29 Free I/O
   FSMC_D9          D9     pin 11  PE12 P1 pin 30 Free I/O
   FSMC_D10         D10    pin 14  PE13 P1 pin 31 Free I/O
   FSMC_D11         D11    pin 13  PE14 P1 pin 32 Free I/O
   FSMC_D12         D12    pin 16  PE15 P1 pin 33 Free I/O
   FSMC_D13         D13    pin 15  PD8  P1 pin 40 Free I/O
   FSMC_D14         D14    pin 18  PD9  P1 pin 41 Free I/O
   FSMC_D15         D15    pin 17  PD10 P1 pin 42 Free I/O
   FSMC_A16         RS     pin 19  PD11 P1 pin 27 Free I/O
   FSMC_NE1         ~CS    pin 10  PD7  P2 pin 27 Free I/O
   FSMC_NWE         ~WR    pin 22  PD5  P2 pin 29 Conflict (Note 3)
   FSMC_NOE         ~RD    pin 21  PD4  P2 pin 32 Conflict (Note 4)
   PC6              RESET  pin 24  PC6  P2 pin 47 Free I/O
   Timer ouput      BL_CNT pin 23  (to be determined)
  ---------------- -------------- ----------------------------------

   1 Used for the RED LED
   2 Used for the BLUE LED
   3 Used for the RED LED and for OTG FS Overcurrent.  It may be okay to use
     for the parallel interface if PC0 is held high (or floating).  PC0 enables
     the STMPS2141STR IC power switch that drives the OTG FS host VBUS.
   4 Also the reset pin for the CS43L22 audio Codec.

NOTE:  The configuration to test this LCD configuration is available at
configs/stm32f4discover/nxlines.  As of this writing, I have not seen the
LCD working so I probaby have some things wrong.

I might need to use a bit-baning interface.  Below is the pin configurationf
of a similar LCD to support a (write-only), bit banging interface:

  LCD PIN   BOARD CONNECTION
  LEDA      5V
  VCC       5V
  RD        3.3V
  GND       GND
  DB0-7     Port C pins configured as outputs
  DB8-15    Port A pins configured as outputs
  RS        Pin configured as output
  WR        Pin configured as output
  CS        Pin configured as output
  RSET      Pin configured as output

The following summarize the bit banging oprations:

  /* Rese the LCD */
  void Reset(void)
  {
    Set RSET output
    delay
    Clear RSET output
    delay
    Set RSET output
  }

  /* Write 16-bits of whatever */
  void Write16(uint8_t ms, uint8_t ls)
  {
    Set port A to ms
    Set port B to ls

    Clear WR pin
    Set   WR pin
  }

  /* Set the index register to an LCD register address */
  void Index(uint8_t address)
  {
    Clear RS
    Write16(0, address);
  }

  /* Write data to the LCD register or GRAM memory */
  void WriteData(uin16_t data)
  {
    Set RS
    Write16(data >> 8, data & 0xff);
  }

  /* Write to a register */
  void WriteRegister(uint8_t address, uint16_t data)
  {
    Index(address);
    WriteData(data);
  }

STM32F4Discovery-specific Configuration Options
===============================================

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

       CONFIG_ARCH_CHIP_STM32F407VG=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n
 
    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=STM32F4Discovery (for the STM32F4Discovery development board)

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

    CONFIG_STM32_CCMEXCLUDE - Exclude CCM SRAM from the HEAP

    In addition to internal SRAM, SRAM may also be available through the FSMC.
    In order to use FSMC SRAM, the following additional things need to be
    present in the NuttX configuration file:

    CONFIG_STM32_FSMC_SRAM - Indicates that SRAM is available via the
      FSMC (as opposed to an LCD or FLASH).

    CONFIG_HEAP2_BASE - The base address of the SRAM in the FSMC address space (hex)

    CONFIG_HEAP2_SIZE - The size of the SRAM in the FSMC address space (decimal)

    CONFIG_ARCH_IRQPRIO - The STM32F4Discovery supports interrupt prioritization

       CONFIG_ARCH_IRQPRIO=y

    CONFIG_ARCH_FPU - The STM32F4Discovery supports a floating point unit (FPU)

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

  JTAG Enable settings (by default only SW-DP is enabled):

    CONFIG_STM32_JTAG_FULL_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
    CONFIG_STM32_JTAG_NOJNTRST_ENABLE - Enables full SWJ (JTAG-DP + SW-DP)
      but without JNTRST.
    CONFIG_STM32_JTAG_SW_ENABLE - Set JTAG-DP disabled and SW-DP enabled

  STM32F4Discovery specific device driver settings

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

  STM32F4Discovery CAN Configuration

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

  STM32F4Discovery SPI Configuration

    CONFIG_STM32_SPI_INTERRUPTS - Select to enable interrupt driven SPI
      support. Non-interrupt-driven, poll-waiting is recommended if the
      interrupt rate would be to high in the interrupt driven case.
    CONFIG_STM32_SPI_DMA - Use DMA to improve SPI transfer performance.
      Cannot be used with CONFIG_STM32_SPI_INTERRUPT.

  STM32F4Discovery DMA Configuration

    CONFIG_SDIO_DMA - Support DMA data transfers.  Requires CONFIG_STM32_SDIO
      and CONFIG_STM32_DMA2.
    CONFIG_SDIO_PRI - Select SDIO interrupt prority.  Default: 128
    CONFIG_SDIO_DMAPRIO - Select SDIO DMA interrupt priority. 
      Default:  Medium
    CONFIG_SDIO_WIDTH_D1_ONLY - Select 1-bit transfer mode.  Default:
      4-bit transfer mode.

  STM32 USB OTG FS Host Driver Support

  Pre-requisites
 
   CONFIG_USBDEV          - Enable USB device support
   CONFIG_USBHOST         - Enable USB host support
   CONFIG_STM32_OTGFS     - Enable the STM32 USB OTG FS block
   CONFIG_STM32_SYSCFG    - Needed
   CONFIG_SCHED_WORKQUEUE - Worker thread support is required
 
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

Each STM32F4Discovery configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh STM32F4Discovery/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  cxxtest:
  -------

  The C++ standard libary test at apps/examples/cxxtest configuration.  This
  test is used to verify the uClibc++ port to NuttX.  This configuration may
  be selected as follows:

    cd <nuttx-directory>/tools
    ./configure.sh sim/cxxtest

  NOTES:

  1. Before you can use this example, you must first install the uClibc++
     C++ library.  This is located outside of the NuttX source tree at
     misc/uClibc++ in SVN.  See the README.txt file for instructions on
     how to install uClibc++

  2. This configuration uses the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the mconf tool.  See nuttx/README.txt and
        misc/tools/

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

  3. Ideally, you should build with a toolchain based on GLIBC or
     uClibc++.  It you use a toolchain based on newlib, you may see
     an error like the following:

     .../lib/libsupc++.a(vterminate.o): In function `__gnu_cxx::__verbose_terminate_handler()':
     vterminate.cc:(....): undefined reference to `_impure_ptr'

     Here is a quick'n'dirty fix:

     1. Get the directory where you can find libsupc++:

        arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -print-file-name=libsupc++.a

     2. Go to that directory and save a copy of vterminate.o (in case you
        want to restore it later:

        cd <the-directory-containing-libsupc++.a>
        arm-none-eabi-ar.exe -x libsupc++.a vterminate.o

     3. Then remove vterminate.o from the library.  At build time, the
        uClibc++ package will provide a usable replacement vterminate.o.

     Steps 2 and 3 will require root privileges on most systems (not Cygwin).

     Now NuttX should link with no problem.  If you want to restore the
     vterminate.o that you removed from libsupc++, you can do that with:

       arm-none-eabi-ar.exe rcs libsupc++.a vterminate.o
        
  4. Exceptions are enabled and workking (CONFIG_UCLIBCXX_EXCEPTIONS=y)

  elf:
  ---

    This configuration derives from the ostest configuration.  It has
    been modified to us apps/examples/elf in order to test the ELF
    loader.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the mconf tool.  See nuttx/README.txt and
          misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default toolchain:

       CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows

    3. By default, this project assumes that you are *NOT* using the DFU
       bootloader.
 
    4. It appears that you cannot excute from CCM RAM.  This is why the
       following definition appears in the defconfig file:

       CONFIG_STM32_CCMEXCLUDE=y

    5. This configuration requires that you have the genromfs tool installed
       on your system and that you have the full path to the installed genromfs
       executable in PATH variable (see apps/examples/README.txt)

  ostest:
  ------
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the mconf tool.  See nuttx/README.txt and
          misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default toolchain:

       CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux / Mac OS X

    3. By default, this project assumes that you are *NOT* using the DFU
       bootloader.
 
    4. If you use the Atollic toolchain, then the FPU test can be enabled in the
      examples/ostest by adding the following your NuttX configuration file:

      -CONFIG_ARCH_FPU=n              : Enable FPU support
      +CONFIG_ARCH_FPU=y

      -CONFIG_STM32_CODESOURCERYW=y   : Disable the CodeSourcery toolchain
      +CONFIG_STM32_CODESOURCERYW=n

      -CONFIG_STM32_ATOLLIC_LITE=n   : Enable *one* the Atollic toolchains
       CONFIG_STM32_ATOLLIC_PRO=n
      -CONFIG_STM32_ATOLLIC_LITE=y   : The "Lite" version
       CONFIG_STM32_ATOLLIC_PRO=n    : The "Pro" version

      -CONFIG_INTELHEX_BINARY=y       : Suppress generation FLASH download formats
      +CONFIG_INTELHEX_BINARY=n       : (Only necessary with the "Lite" version)

      -CONFIG_HAVE_CXX=y              : Suppress generation of C++ code
      +CONFIG_HAVE_CXX=n              : (Only necessary with the "Lite" version)

      -CONFIG_SCHED_WAITPID=y         : Enable the waitpid() API needed by the FPU test
      +CONFIG_SCHED_WAITPID=n

      The FPU test also needs to know the size of the FPU registers save area in
      bytes (see arch/arm/include/armv7-m/irq_lazyfpu.h):

      -CONFIG_EXAMPLES_OSTEST_FPUSIZE=(4*33)
 
  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    Default toolchain:

    CONFIG_STM32_CODESOURCERYL=y  : CodeSourcery under Linux / Mac OS X

    NOTES:
    1. This example supports the PWM test (apps/examples/pwm) but this must
       be manually enabled by selecting:

       CONFIG_PWM=y              : Enable the generic PWM infrastructure
       CONFIG_STM32_TIM4=y       : Enable TIM4
       CONFIG_STM32_TIM4_PWM=y   : Use TIM4 to generate PWM output

       See also apps/examples/README.txt

       Special PWM-only debug options:

       CONFIG_DEBUG_PWM

    2. This example supports the Quadrature Encode test (apps/examples/qencoder)
       but this must be manually enabled by selecting:

       CONFIG_QENCODER=y         : Enable the generic Quadrature Encoder infrastructure
       CONFIG_STM32_TIM8=y       : Enable TIM8
       CONFIG_STM32_TIM2=n       : (Or optionally TIM2)
       CONFIG_STM32_TIM8_QE=y    : Use TIM8 as the quadrature encoder
       CONFIG_STM32_TIM2_QE=y    : (Or optionally TIM2)

       See also apps/examples/README.txt

       Special PWM-only debug options:

       CONFIG_DEBUG_QENCODER

    3. This example supports the watchdog timer test (apps/examples/watchdog)
       but this must be manually enabled by selecting:

       CONFIG_WATCHDOG=y         : Enables watchdog timer driver support
       CONFIG_STM32_WWDG=y       : Enables the WWDG timer facility, OR
       CONFIG_STM32_IWDG=y       : Enables the IWDG timer facility (but not both)

       The WWDG watchdog is driven off the (fast) 42MHz PCLK1 and, as result,
       has a maximum timeout value of 49 milliseconds.  for WWDG watchdog, you
       should also add the fillowing to the configuration file:

       CONFIG_EXAMPLES_WATCHDOG_PINGDELAY=20
       CONFIG_EXAMPLES_WATCHDOG_TIMEOUT=49

       The IWDG timer has a range of about 35 seconds and should not be an issue.

     4. USB Support (CDC/ACM device)

        CONFIG_STM32_OTGFS=y      : STM32 OTG FS support
        CONFIG_USBDEV=y           : USB device support must be enabled
        CONFIG_CDCACM=y           : The CDC/ACM driver must be built
        CONFIG_NSH_BUILTIN_APPS=y : NSH built-in application support must be enabled

     5. Using the USB console.

        The STM32F4Discovery NSH configuration can be set up to use a USB CDC/ACM
        (or PL2303) USB console.  The normal way that you would configure the
        the USB console would be to change the .config file like this:

        CONFIG_STM32_OTGFS=y      : STM32 OTG FS support
        CONFIG_DEV_CONSOLE=n      : Inhibit use of /dev/console by other logic
        CONFIG_USBDEV=y           : USB device support must be enabled
        CONFIG_CDCACM=y           : The CDC/ACM driver must be built
        CONFIG_CDCACM_CONSOLE=y   : Enable the CDC/ACM USB console.

        However, that configuration does not work.  It fails early probably because
        of some dependency on /dev/console before the USB connection is established.

        But there is a work around for this that works better (but has some side
        effects).  The following configuration will also create a NSH USB console
        but this version will will use /dev/console.  Instead, it will use the
        normal /dev/ttyACM0 USB serial device for the console:

        CONFIG_STM32_OTGFS=y      : STM32 OTG FS support
        CONFIG_USBDEV=y           : USB device support must be enabled
        CONFIG_CDCACM=y           : The CDC/ACM driver must be built
        CONFIG_CDCACM_CONSOLE=n   : Done use the CDC/ACM USB console.
        CONFIG_NSH_USBCONSOLE=y   : Instead use some other USB device for the console

        The particular USB device that is used is:

        CONFIG_NSH_USBCONDEV="/dev/ttyACM0"

        NOTE 1: When you first start the USB console, you have hit ENTER a few
        times before NSH starts.  The logic does this to prevent sending USB data
        before there is anything on the host side listening for USB serial input.

        Now the side effects:

        NOTE 2. When any other device other than /dev/console is used for a user
        interface, linefeeds (\n) will not be expanded to carriage return /
        linefeeds (\r\n).  You will need to set your terminal program to account
        for this.

        NOTE 3: /dev/console still exists and still refers to the serial port. So
        you can still use certain kinds of debug output (see include/debug.h, all
        of the interfaces based on lib_lowprintf will work in this configuration).

    6. USB OTG FS Host Support.  The following changes will enable support for
       a USB host on the STM32F4Discovery, including support for a mass storage
       class driver:
 
       CONFIG_USBDEV=n          - Make sure tht USB device support is disabled
       CONFIG_USBHOST=y         - Enable USB host support
       CONFIG_STM32_OTGFS=y     - Enable the STM32 USB OTG FS block
       CONFIG_STM32_SYSCFG=y    - Needed for all USB OTF FS support
       CONFIG_SCHED_WORKQUEUE=y - Worker thread support is required for the mass
                                  storage class driver.
       CONFIG_NSH_ARCHINIT=y    - Architecture specific USB initialization
                                  is needed for NSH
       CONFIG_FS_FAT=y          - Needed by the USB host mass storage class.

       With those changes, you can use NSH with a FLASH pen driver as shown
       belong.  Here NSH is started with nothing in the USB host slot:

       NuttShell (NSH) NuttX-x.yy
       nsh> ls /dev
       /dev:
        console
        null
        ttyS0

       After inserting the FLASH drive, the /dev/sda appears and can be
       mounted like this:

       nsh> ls /dev
       /dev:
        console
        null
        sda
        ttyS0
       nsh> mount -t vfat /dev/sda /mnt/stuff
       nsh> ls /mnt/stuff
       /mnt/stuff:
        -rw-rw-rw-   16236 filea.c

       And files on the FLASH can be manipulated to standard interfaces:

       nsh> echo "This is a test" >/mnt/stuff/atest.txt
       nsh> ls /mnt/stuff
       /mnt/stuff:
        -rw-rw-rw-   16236 filea.c
        -rw-rw-rw-      16 atest.txt
       nsh> cat /mnt/stuff/atest.txt
       This is a test
       nsh> cp /mnt/stuff/filea.c fileb.c
       nsh> ls /mnt/stuff
       /mnt/stuff:
        -rw-rw-rw-   16236 filea.c
        -rw-rw-rw-      16 atest.txt
        -rw-rw-rw-   16236 fileb.c

       To prevent data loss, don't forget to un-mount the FLASH drive
       before removing it:

       nsh> umount /mnt/stuff

  nxlines:
  ------
    An example using the NuttX graphics system (NX).   This example focuses on
    placing lines on the background in various orientations.

      CONFIG_STM32_CODESOURCERYW=y  : CodeSourcery under Windows
      CONFIG_LCD_LANDSCAPE=y        : 320x240 landscape orientation

    The STM32F4Discovery board does not have any graphics capability.  This
    configuration assumes that you have connected an SD1289-based LCD as
    described above under "SSD1289".  NOTE:  At present, it has not been
    proven that the STM32F4Discovery can actually drive an LCD.  There are
    some issues with how some of the dedicated FSMC pins are used on the
    boards.  This configuration may not be useful and may only serve as
    an illustration of how to build for th SSD1289 LCD.

  NOTES:

  1. As of this writing, I have not seen the LCD work!

  2. This configuration uses the mconf-based configuration tool.  To
     change this configuration using that tool, you should:

     a. Build and install the mconf tool.  See nuttx/README.txt and
        misc/tools/

     b. Execute 'make menuconfig' in nuttx/ in order to start the
        reconfiguration process.

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
