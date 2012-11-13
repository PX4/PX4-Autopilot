README
======

This README discusses issues unique to NuttX configurations for the Shenzhou
IV development board from www.armjishu.com featuring the STMicro STM32F107VCT
MCU.  As of this writing, there are five models of the Shenzhou board:

  1. Shenzhou I (STM32F103RB)
  2. Shenzhou II (STM32F103VC)
  3. Shenzhou III (STM32F103ZE)
  4. Shenzhou IV (STM32F107VC)
  5. Shenzhou king ((STM32F103ZG, core board + IO expansion board)).

Support is currently provided for the Shenzhou IV only.  Features of the
Shenzhou IV board include:

  - STM32F107VCT
  - 10/100M PHY (DM9161AEP)
  - TFT LCD Connector
  - USB OTG
  - CAN (CAN1=2)
  - USART connectos (USART1-2)
  - RS-485
  - SD card slot
  - Audio DAC (PCM1770)
  - SPI Flash (W25X16)
  - (4) LEDs (LED1-4)
  - 2.4G Wireless (NRF24L01 SPI module)
  - 315MHz Wireless (module)
  - (4) Buttons (KEY1-4, USERKEY2, USERKEY, TEMPER, WAKEUP)
  - VBUS/external +4V select
  - 5V/3.3V power conversion
  - Extension connector
  - JTAG

Contents
========

  - STM32F107VCT Pin Usage
  - Development Environment
  - GNU Toolchain Options
  - IDEs
  - NuttX EABI buildroot Toolchain
  - NuttX OABI buildroot Toolchain
  - NXFLAT Toolchain
  - Shenzhou-specific Configuration Options
  - LEDs
  - Shenzhou-specific Configuration Options
  - Configurations

STM32F107VCT Pin Usage
======================

-- ---- -------------- -------------------------------------------------------------------
PN NAME SIGNAL         NOTES
-- ---- -------------- -------------------------------------------------------------------
23 PA0  WAKEUP         Connected to KEY4.  Active low: Closing KEY4 pulls WAKEUP to ground.
24 PA1  MII_RX_CLK
        RMII_REF_CLK
25 PA2  MII_MDIO
26 PA3  315M_VT
29 PA4  DAC_OUT1       To CON5(CN14)
30 PA5  DAC_OUT2       To CON5(CN14). JP10
        SPI1_SCK       To the SD card, SPI FLASH
31 PA6  SPI1_MISO      To the SD card, SPI FLASH
32 PA7  SPI1_MOSI      To the SD card, SPI FLASH
67 PA8  MCO            To DM9161AEP PHY
68 PA9  USB_VBUS       MINI-USB-AB. JP3
        USART1_TX      MAX3232 to CN5
69 PA10 USB_ID         MINI-USB-AB. JP5
        USART1_RX      MAX3232 to CN5
70 PA11 USB_DM         MINI-USB-AB
71 PA12 USB_DP         MINI-USB-AB
72 PA13 TMS/SWDIO
76 PA14 TCK/SWCLK
77 PA15 TDI

-- ---- -------------- -------------------------------------------------------------------
PN NAME SIGNAL         NOTES
-- ---- -------------- -------------------------------------------------------------------
35 PB0  ADC_IN1        To CON5(CN14)
36 PB1  ADC_IN2        To CON5(CN14)
37 PB2  DATA_LE        To TFT LCD (CN13)
        BOOT1          JP13
89 PB3  TDO/SWO
90 PB4  TRST
91 PB5  CAN2_RX
92 PB6  CAN2_TX        JP11
        I2C1_SCL
93 PB7  I2C1_SDA
95 PB8  USB_PWR        Drives USB VBUS
96 PB9  F_CS           To both the TFT LCD (CN13) and to the W25X16 SPI FLASH
47 PB10 USERKEY        Connected to KEY2
48 PB11 MII_TX_EN      Ethernet PHY
51 PB12 I2S_WS         Audio DAC
        MII_TXD0       Ethernet PHY
52 PB13 I2S_CK         Audio DAC
        MII_TXD1       Ethernet PHY
53 PB14 SD_CD          There is confusion here.  Schematic is wrong LCD_WR is PB14.
54 PB15 I2S_DIN        Audio DAC

-- ---- -------------- -------------------------------------------------------------------
PN NAME SIGNAL         NOTES
-- ---- -------------- -------------------------------------------------------------------
15 PC0  POTENTIO_METER
16 PC1  MII_MDC        Ethernet PHY
17 PC2  WIRELESS_INT
18 PC3  WIRELESS_CE    To the NRF24L01 2.4G wireless module
33 PC4  USERKEY2       Connected to KEY1
34 PC5  TP_INT         JP6.  To TFT LCD (CN13) module
        MII_INT        Ethernet PHY
63 PC6  I2S_MCK        Audio DAC. Active low: Pulled high
64 PC7  PCM1770_CS     Audio DAC. Active low: Pulled high
65 PC8  LCD_CS         TFT LCD (CN13). Active low: Pulled high
66 PC9  TP_CS          TFT LCD (CN13). Active low: Pulled high
78 PC10 SPI3_SCK       To TFT LCD (CN13), the NRF24L01 2.4G wireless module
79 PC11 SPI3_MISO      To TFT LCD (CN13), the NRF24L01 2.4G wireless module
80 PC12 SPI3_MOSI      To TFT LCD (CN13), the NRF24L01 2.4G wireless module
7  PC13 TAMPER         Connected to KEY3
8  PC14 OSC32_IN       Y1 32.768Khz XTAL
9  PC15 OSC32_OUT      Y1 32.768Khz XTAL

-- ---- -------------- -------------------------------------------------------------------
PN NAME SIGNAL         NOTES
-- ---- -------------- -------------------------------------------------------------------
81 PD0  CAN1_RX
82 PD1  CAN1_TX
83 PD2  LED1           Active low: Pulled high
84 PD3  LED2           Active low: Pulled high
85 PD4  LED3           Active low: Pulled high
86 PD5  485_TX         Same as USART2_TX but goes to SP3485
        USART2_TX      MAX3232 to CN6
87 PD6  485_RX         Save as USART2_RX but goes to SP3485 (see JP4)
        USART2_RX      MAX3232 to CN6
88 PD7  LED4           Active low: Pulled high
        485_DIR        SP3485 read enable (not)
55 PD8  MII_RX_DV      Ethernet PHY
        RMII_CRSDV     Ethernet PHY
56 PD9  MII_RXD0       Ethernet PHY
57 PD10 MII_RXD1       Ethernet PHY
58 PD11 SD_CS          Active low: Pulled high (See also TFT LCD CN13, pin 32)
59 PD12 WIRELESS_CS    To the NRF24L01 2.4G wireless module
60 PD13 LCD_RS         To TFT LCD (CN13)
61 PD14 LCD_WR         To TFT LCD (CN13). Schematic is wrong LCD_WR is PB14.
62 PD15 LCD_RD         To TFT LCD (CN13)

-- ---- -------------- -------------------------------------------------------------------
PN NAME SIGNAL         NOTES
-- ---- -------------- -------------------------------------------------------------------
97 PE0  DB00           To TFT LCD (CN13)
98 PE1  DB01           To TFT LCD (CN13)
1  PE2  DB02           To TFT LCD (CN13)
2  PE3  DB03           To TFT LCD (CN13)
3  PE4  DB04           To TFT LCD (CN13)
4  PE5  DB05           To TFT LCD (CN13)
5  PE6  DB06           To TFT LCD (CN13)
38 PE7  DB07           To TFT LCD (CN13)
39 PE8  DB08           To TFT LCD (CN13)
40 PE9  DB09           To TFT LCD (CN13)
41 PE10 DB10           To TFT LCD (CN13)
42 PE11 DB11           To TFT LCD (CN13)
43 PE12 DB12           To TFT LCD (CN13)
44 PE13 DB13           To TFT LCD (CN13)
45 PE14 DB14           To TFT LCD (CN13)
46 PE15 DB15           To TFT LCD (CN13)

-- ---- -------------- -------------------------------------------------------------------
PN NAME SIGNAL         NOTES
-- ---- -------------- -------------------------------------------------------------------
73 N/C

12 OSC_IN              Y2 25Mhz XTAL
13 OSC_OUT             Y2 25Mhz XTAL

94 BOOT0               JP15 (3.3V or GND)
14 RESET               S5
6  VBAT                JP14 (3.3V or battery)

49 VSS_1               GND
74 VSS_2               GND
99 VSS_3               GND
27 VSS_4               GND
10 VSS_5               GND
19 VSSA                VSSA
20 VREF-               VREF-

Development Environment
=======================

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment because the development tools that I used only work under Windows.

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
  that is the default toolchain in most configurations.  To use the Atollic,
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
  Windows native toolchains.  The CodeSourcery (for Linux) and NuttX buildroot
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

NuttX EABI buildroot Toolchain
==============================

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh shenzhou/<sub-dir>

     cd ..
     make context

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-eabi-defconfig-4.6.3 .config

  6. make oldconfig

  7. make

  8. Edit nuttx/.config to select the buildroot toolchain as described above
     and below:

     -CONFIG_STM32_CODESOURCERYW=y
     +CONFIG_STM32_BUILDROOT=y

  9. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

     -export TOOLCHAIN_BIN="/cygdrive/c/Program Files (x86)/CodeSourcery/Sourcery G++ Lite/bin"
     +#export TOOLCHAIN_BIN="/cygdrive/c/Program Files (x86)/CodeSourcery/Sourcery G++ Lite/bin"

     -#export TOOLCHAIN_BIN="${WD}/../misc/buildroot/build_arm_nofpu/staging_dir/bin"
     +export TOOLCHAIN_BIN="${WD}/../misc/buildroot/build_arm_nofpu/staging_dir/bin"

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
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

The Shenzhou board has four LEDs labeled LED1, LED2, LED3 and LED4 on the
board. These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
defined.  In that case, the usage by the board port is defined in
include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
events as follows:

    SYMBOL               Meaning                 LED1*   LED2    LED3    LED4****
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
  ** The normal state is LED1 ON and LED1 faintly glowing.  This faint glow
     is because of timer interupts that result in the LED being illuminated
     on a small proportion of the time.
 *** LED2 may also flicker normally if signals are processed.
**** LED4 may not be available if RS-485 is also used. For RS-485, it will
     then indicate the RS-485 direction.

Shenzhou-specific Configuration Options
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

       CONFIG_ARCH_CHIP_STM32F107VC=y

    CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG - Enables special STM32 clock
       configuration features.

       CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG=n

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=shenzhou (for the Shenzhou development board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_SHENZHOU=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_STM32_CCMEXCLUDE - Exclude CCM SRAM from the HEAP

    CONFIG_ARCH_IRQPRIO - The STM32107xxx supports interrupt prioritization

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
    CONFIG_STM32_ETHMAC
    CONFIG_STM32_OTGFS
    CONFIG_STM32_IWDG
    CONFIG_STM32_PWR -- Required for RTC

    APB1 (low speed)
    ----------------
    CONFIG_STM32_BKP
    CONFIG_STM32_TIM2
    CONFIG_STM32_TIM3
    CONFIG_STM32_TIM4
    CONFIG_STM32_TIM5
    CONFIG_STM32_TIM6
    CONFIG_STM32_TIM7
    CONFIG_STM32_USART2
    CONFIG_STM32_USART3
    CONFIG_STM32_UART4
    CONFIG_STM32_UART5
    CONFIG_STM32_SPI2
    CONFIG_STM32_SPI3
    CONFIG_STM32_I2C1
    CONFIG_STM32_I2C2
    CONFIG_STM32_CAN1
    CONFIG_STM32_CAN2
    CONFIG_STM32_DAC1
    CONFIG_STM32_DAC2
    CONFIG_STM32_WWDG

    APB2 (high speed)
    -----------------
    CONFIG_STM32_TIM1
    CONFIG_STM32_SPI1
    CONFIG_STM32_USART1
    CONFIG_STM32_ADC1
    CONFIG_STM32_ADC2

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

  STM32107xxx specific device driver settings

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

    CONFIG_STM32_PHYADDR - The 5-bit address of the PHY on the board
    CONFIG_STM32_MII - Support Ethernet MII interface
    CONFIG_STM32_MII_MCO - Use MCO to clock the MII interface
    CONFIG_STM32_RMII - Support Ethernet RMII interface
    CONFIG_STM32_RMII_MCO - Use MCO to clock the RMII interface
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

  Shenzhou CAN Configuration

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

  Shenzhou LCD Hardware Configuration

  The LCD driver supports the following LCDs on the STM324xG_EVAL board:

    AM-240320L8TNQW00H (LCD_ILI9320 or LCD_ILI9321) OR
    AM-240320D5TOQW01H (LCD_ILI9325)

  Configuration options.

    CONFIG_LCD_LANDSCAPE - Define for 320x240 display "landscape"
      support. Default is this 320x240 "landscape" orientation
      For the Shenzhou board, the edge opposite from the row of buttons
      is used as the top of the display in this orientation.
    CONFIG_LCD_RLANDSCAPE - Define for 320x240 display "reverse
      landscape" support. Default is this 320x240 "landscape"
      orientation
      For the Shenzhou board, the edge next to the row of buttons
      is used as the top of the display in this orientation.
    CONFIG_LCD_PORTRAIT - Define for 240x320 display "portrait"
      orientation support.
    CONFIG_LCD_RPORTRAIT - Define for 240x320 display "reverse
      portrait" orientation support.
    CONFIG_LCD_RDSHIFT - When reading 16-bit gram data, there appears
      to be a shift in the returned data.  This value fixes the offset.
      Default 5.

    The LCD driver dynamically selects the LCD based on the reported LCD
    ID value.  However, code size can be reduced by suppressing support for
    individual LCDs using:

    CONFIG_STM32_ILI9320_DISABLE (includes ILI9321)
    CONFIG_STM32_ILI9325_DISABLE

  STM32 USB OTG FS Host Driver Support

  Pre-requisites
 
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

Each Shenzhou configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh shenzhou/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  nsh:
  ---
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.

    CONFIG_STM32_CODESOURCERYW=y              : CodeSourcery under Windows
    CONFIG_NSH_DHCPC=n                        : DHCP is disabled
    CONFIG_NSH_IPADDR=0x0a000002              : Target IP address 10.0.0.2
    CONFIG_NSH_DRIPADDR=0x0a000001            : Host IP address 10.0.0.1

    NOTES:
    1. This example assumes that a network is connected.  During its
       initialization, it will try to negotiate the link speed.  If you have
       no network connected when you reset the board, there will be a long
       delay (maybe 30 seconds?) before anything happens.  That is the timeout
       before the networking finally gives up and decides that no network is
       available.

    2. Enabling the ADC example:

       The only internal signal for ADC testing is the potentiometer input:

         ADC1_IN10(PC0) Potentiometer

       External signals are also available on CON5 CN14:
    
         ADC_IN8 (PB0) CON5 CN14 Pin2
         ADC_IN9 (PB1) CON5 CN14 Pin1

       The signal selection is hard-coded in configs/shenzhou/src/up_adc.c:  The
       potentiometer input (only) is selected.

       These selections will enable sampling the potentiometer input at 100Hz using
       Timer 1:

         CONFIG_ANALOG=y                        : Enable analog device support
         CONFIG_ADC=y                           : Enable generic ADC driver support
         CONFIG_ADC_DMA=n                       : ADC DMA is not supported
         CONFIG_STM32_ADC1=y                    : Enable ADC 1
         CONFIG_STM32_TIM1=y                    : Enable Timer 1
         CONFIG_STM32_TIM1_ADC=y                : Use Timer 1 for ADC
         CONFIG_STM32_TIM1_ADC1=y               : Allocate Timer 1 to ADC 1
         CONFIG_STM32_ADC1_SAMPLE_FREQUENCY=100 : Set sampling frequency to 100Hz
         CONFIG_STM32_ADC1_TIMTRIG=0            : Trigger on timer output 0
         CONFIG_STM32_FORCEPOWER=y              : Apply power to TIM1 a boot up time
         CONFIG_EXAMPLES_ADC=y                  : Enable the apps/examples/adc built-in

  nxwm
  ----
    This is a special configuration setup for the NxWM window manager
    UnitTest.  The NxWM window manager can be found here:

      nuttx-code/NxWidgets/nxwm

    The NxWM unit test can be found at:

      nuttx-code/NxWidgets/UnitTests/nxwm

    NOTE:  JP6 selects between the touchscreen interrupt and the MII
    interrupt.  It should be positioned 1-2 to enable the touchscreen
    interrupt.

    Documentation for installing the NxWM unit test can be found here:

      nuttx-code/NxWidgets/UnitTests/README.txt

    Here is the quick summary of the build steps (Assuming that all of
    the required packages are available in a directory ~/nuttx-code):

    1. Intall the nxwm configuration

       $ cd ~/nuttx-code/tools
       $ ./configure.sh shenzhou/nxwm

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
       $ make TOPDIR=~/nuttx-code
       ...

    5. Build the NxWM library

       $ cd ~/nuttx-code/NxWidgets/nxwm
       $ make TOPDIR=~/nuttx-code
       ...

    6. Built NuttX with the installed unit test as the application

       $ cd ~/nuttx-code
       $ make

    NOTE: Reading from the LCD is not currently supported by this
    configuration.  The hardware will support reading from the LCD
    and drivers/lcd/ssd1289.c also supports reading from the LCD.
    This limits some graphics capabilities.

    Reading from the LCD is not supported only because it has not
    been test.  If you get inspired to test this feature, you can
    turn the LCD read functionality on by setting:

      -CONFIG_LCD_NOGETRUN=y
      +# CONFIG_LCD_NOGETRUN is not set

      -CONFIG_NX_WRITEONLY=y
      +# CONFIG_NX_WRITEONLY is not set
 
  thttpd
  ------

    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTE: See note above with regard to the EABI/OABI buildroot
    toolchains.  This example can only be built using the older
    OABI toolchain due to incompatibilities introduced in later
    GCC releases.
