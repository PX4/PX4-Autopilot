configs/pic32mx7mmb README
===============================

This README file discusses the port of NuttX to the  Mikroelektronika PIC32MX7
Multimedia Board (MMB).  See http://www.mikroe.com/ for further information.

Contents
========

  PIC32MX795F512L Pin Out
  Toolchains
  Creating Compatible NuttX HEX files
  Serial Console
  LEDs
  PIC32MX Configuration Options
  Configurations

PIC32MX795F512L Pin Out
=======================

LEFT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
--- ---------------------------------- -------------------------- -----------------------------------------------
PIN CONFIGURATIONS                     SIGNAL NAME                ON-BOARD CONNECTIONS
    (Family Data Sheet Table 1-1)     (PIC32MX7 Schematic)
--- ---------------------------------- -------------------------- -----------------------------------------------
  1 RG15/AERXERR                       AERXERR                    LAN8720A RXERR
  2 VDD                                VCC3                       ---
  3 PMD5/RE5                           PMPD5                      TFT display, HDR1 pin 13
  4 PMD6/RE6                           PMPD6                      TFT display, HDR1 pin 12
  5 PMD7/RE7                           PMPD7                      TFT display, HDR1 pin 11
  6 RC1/T2CK                           LCD_RST                    TFT display
  7 RC2/AC2TX/T3CK                     EE_CS#                     M25P80 CS
  8 RC3/AC2RX/T4CK                     ACL_CS#                    ADXL345 CS and VCC
  9 RC4/SDI1/T5CK                      SDI1                       SPI1 data IN
 10 PMA5/CN8/ECOL/RG6/SCK2/U3RTS/U6TX  SD_WP                      SD card, write protect
 11 PMA4/CN9/ECRS/RG7/SDA4/SDI2/U3RX   SD_CD#                     SD card, card detect (not)
 12 PMA3/AECRSDV/AERXDV/CN10/ECRSDV/   AECRSDV                    LAN8720A SRS_DIV
    ERXDV/RG8/SCL4/SDO2/U3TX
 13 MCLR                               MCLR                       Debug connector
 14 PMA2/AEREFCLK/AERXCLK/CN11/        AEREFCLK                   LAN8720A INT
    EREFCLK/ERXCLK/RG9/SS2/U3CTS/
    U6RX
 15 VSS                                (grounded)                 ---
 16 VDD                                VCC3                       ---
 17 RA0/TMS                            LED-0                      LED0 (pulled up), HDR2 pin 26
 18 AERXD0/INT1/RE8                    AERXD0                     LAN8720A RXD0
 19 AERXD1/INT2/RE9                    AERXD1                     LAN8720A RXD1
 20 AN5/C1IN+/CN7/RB5/VBUSON           RB5                        HDR1 pin 28
 21 AN4/C1IN-/CN6/RB4                  CDC_CS#                    ?
 22 AN3/C2IN+/CN5/RB3                  JOY-D                      Joystick D, HDR1 pin 21
 23 AN2/C2IN-/CN4/RB2                  JOY-C                      Joystick C, HDR1 pin 22
 24 AN1/CN3/PGEC1/RB1                  JOY-B                      Joystick B, HDR1 pin 23
 25 AN0/CN2/PGED1/RB0                  JOY-A                      Joystick A, HDR1 pin 24

BOTTOM SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
--- ---------------------------------- -------------------------- -----------------------------------------------
PIN CONFIGURATIONS                     SIGNAL NAME                ON-BOARD CONNECTIONS
    (Family Data Sheet Table 1-1)     (PIC32MX7 Schematic)
--- ---------------------------------- -------------------------- -----------------------------------------------
 26 AN6/OCFA/PGEC2/RB6                 PGC2                       Debugger interface
 27 AN7/PGED2/RB7                      PGD2                       Debugger interface
 28 PMA7/AERXD2/CVREF-/RA9             SD_CD#                     SD Connector
 29 PMA6/AERXD3/CVREF+/RA10/VREF+      JOY-CP                     Joystick CP, HDR1 pin 25
 30 AVDD                               VCC3                       ---
 31 AVSS                               (grounded)                 ---
 32 AN8/C1OUT/RB8                      TEMP                       MCP9700A VOUT
 33 AN9/C2OUT/RB9                      USB-PSW                    USB soft-connect pull-up, HDR2 pin 3
 34 PMA13/AN10/RB10/CVREFOUT           LCD-YD                     TFT display
 35 PMA12/AETXERR/AN11/ERXERR/RB11     LCD-XR                     TFT display
 36 VSS                                (grounded)                 ---
 37 VDD                                P32_VDD                    ---
 38 RA1/TCK                            LED-1                      LED1 (pulled up), HDR2 pin 27
 39 AC1TX/RF13/SCK4/U2RTS/U5TX         SCK3A                      WM873ASEDS BCLK, HDR2 pin 21
 40 AC1RX/RF12/SS4/U2CTS/U5RX          LRC                        WM873ASEDS DACLRC
 41 PMA11/AECRS/AN12/ERXD0/RB12        LCD-YU                     TFT display
 42 PMA10/AECOL/AN13/ERXD1/RB13        LCD-XL                     TFT display
 43 PMA1/AETXD3/AN14/ERXD2/PMALH/RB14  LCD-CS#                    TFT display, HDR2 pin 3
 44 PMA0/AETXD2/AN15/CN12/ERXD3/OCFB/  LCD-RS                     TFT display       
    PMALL/RB15
 45 VSS                                (grounded)                 ---
 46 VDD                                P32_VDD                    ---
 47 AETXD0/CN20/RD14/SS3/U1CTS/U4RX    AETXD0                     LAN8720A TXD0
 48 AETXD1/CN21/RD15/SCK3/U1RTS/U4TX   AETXD1                     LAN8720A TXD1
 49 PMA9/CN17/RF4/SDA5/SDI4/U2RX       SDI3A                      WM873ASEDS ADCDAT, HDR2 pin 19
 50 PMA8/CN18/RF5/SCL5/SDO4/U2TX       SDO3A                      WM873ASEDS DACDAT, HDR2 pin 20

RIGHT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
--- ---------------------------------- -------------------------- -----------------------------------------------
PIN CONFIGURATIONS                     SIGNAL NAME                ON-BOARD CONNECTIONS
    (Family Data Sheet Table 1-1)     (PIC32MX7 Schematic)
--- ---------------------------------- -------------------------- -----------------------------------------------
 75 VSS                                (grounded)
 74 CN0/RC14/SOSCO/T1CK                SOSC0                      32.768kHz Oscillator
 73 CN1/RC13/SOSCI                     SOSC1                      32.768kHz Oscillator
 72 OC1/INT0/RD0/SDO1                  SDO1M                      SPI1 data out
 71 PMA14/AEMDC/EMDC/IC4/PMCS1/RD11    AEMDC                      LAN8720A MDC
 70 PMA15/IC3/PMCS2/RD10/SCK1          SCK1M                      SPI1 clock
 69 IC2/RD9/SS1                        LED-2                      LED2 (pulled up), HDR2 pin 28
 68 AEMDIO/EMDIO/IC1/RD8/RTCC          AEMDIO                     LAN8720A MDIO
 67 AETXEN/INT4/RA15/SDA1              AETXN                      LAN8720A TXEN
 66 AETXCLK/INT3/RA14/SCL1             RA14                       HDR2 pin 14
 65 VSS                                (grounded)                 ---
 64 CLKO/OSC2/RC15                                                8MHz crystal
 63 CLKI/OSC1/RC12                                                8MHz crystal
 62 VDD                                VCC3                       ---
 61 RA5/TDO                            RA5                        HDR2 pin 13
 60 RA4/TDI                            RA4                        HDR2 pin 12
 59 RA3/SDA2                           SDA2                       I2C2 SDA, 24AA01 SDA
 58 RA2/SCL2                           SCL2                       I2C2 SCL, 24AA01 SCL
 57 D+/RG2                             USBDP                      USB device
 56 D-/RG3                             USBDM                      USB device
 55 VUSB                               VCC3                       ---
 54 VBUS                               USB_DET                    USB device
 53 RF8/SCL3/SDO3/U1TX                 U1TX                       RS-232
 52 RF2/SDA3/SDI3/U1RX                 U2RX                       RS-232
 51 RF3/USBID                          USB-ID                     USB device

TOP SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
--- ---------------------------------- -------------------------- -----------------------------------------------
PIN CONFIGURATIONS                     SIGNAL NAME                ON-BOARD CONNECTIONS
    (Family Data Sheet Table 1-1)     (PIC32MX7 Schematic)
--- ---------------------------------- -------------------------- -----------------------------------------------
100 PMD4/RE4                           PMPD4                      TFT display, HDR1 pin 14
 99 PMD3/RE3                           PMPD3                      TFT display, HDR1 pin 15
 98 PMD2/RE2                           PMPD2                      TFT display, HDR1 pin 16
 97 RG13/TRD0                          TRD0                       HDR2 pin 7
 96 RG12/TRD1                          TRD1                       HDR2 pin 8
 95 RG14/TRD2                          TRD2                       HDR2 pin 9
 94 PMD1/RE1                           PMPD1                      TFT display, HDR1 pin 17
 93 PMD0/RE0                           PMPD0                      TFT display, HDR1 pin 18
 92 RA7/TRD3                           TRD3                       HDR2 pin 10
 91 RA6/TRCLK                          TRCLK                      HDR2 pin 6
 90 PMD8/C2RX/RG0                      PMPD8                      TFT display, HDR1 pin 10
 89 PMD9/C2TX/ETXERR/RG1               PMPD9                      TFT display, HDR1 pin 9
 88 PMD10/C1TX/ETXD0/RF1               PMPD10                     TFT display, HDR1 pin 8
 87 PMD11/C1RX/ETXD1/RF0               PMPD11                     TFT display, HDR1 pin 7
 86 VDD                                P32_VDD                    ---
 85 VCAP/VCORE                         (capacitor to ground)      ---
 84 PMD15/CN16/ETXCLK/RD7              PMPD15                     TFT display, HDR1 pin 3
 83 PMD14/CN15/ETXEN/RD6               PMPD14                     TFT display, HDR1 pin 4
 82 CN14/PMRD/RD5                      PMPRD                      
 81 CN13/OC5/PMWR/RD4                  PMPWR                      
 80 PMD13/CN19/ETXD3/RD13              PMPD13                     TFT display, HDR1 pin 5
 79 PMD12/ETXD2/IC5/RD12               PMPD12                     TFT display, HDR1 pin 6
 78 OC4/RD3                            RD3                        HDR2 pin 5
 77 OC3/RD2                            LCD_BLED                   LCD backlight LED
 76 OC2/RD1                            RD1                        HDR2 pin 11

Toolchains
==========

  MPLAB/C32
  ---------

  I am using the free, "Lite" version of the PIC32MX toolchain available
  for download from the microchip.com web site.  I am using the Windows
  version.  The MicroChip toolchain is the only toolchain currently
  supported in these configurations, but it should be a simple matter to
  adapt to other toolchains by modifying the Make.defs file include in
  each configuration.

  C32 Toolchain Options:

    CONFIG_PIC32MX_MICROCHIPW      - MicroChip full toolchain for Windows
    CONFIG_PIC32MX_MICROCHIPL      - MicroChip full toolchain for Linux
    CONFIG_PIC32MX_MICROCHIPW_LITE - MicroChip "Lite" toolchain for Windows
    CONFIG_PIC32MX_MICROCHIPL_LITE - MicroChip "Lite" toolchain for Linux

  NOTE:  The "Lite" versions of the toolchain does not support C++.  Also
  certain optimization levels are not supported by the "Lite" toolchain.

  MicrochipOpen
  -------------

  An alternative, build-it-yourself toolchain is available here:
  http://sourceforge.net/projects/microchipopen/ .  These tools were
  last updated circa 2010.  NOTE:  C++ support still not available
  in this toolchain.

  Building MicrochipOpen (on Linux)

  1) Get the build script from this location:

      http://microchipopen.svn.sourceforge.net/viewvc/microchipopen/ccompiler4pic32/buildscripts/trunk/

  2) Build the code using the build script, for example:

      ./build.sh -b v105_freeze

     This will check out the selected branch and build the tools.

  3) Binaries will then be available in a subdirectory with a name something like
     pic32-v105-freeze-20120622/install-image/bin (depending on the current data
     and the branch that you selected.

     Note that the tools will have the prefix, mypic32- so, for example, the
     compiler will be called mypic32-gcc.

  Pinguino mips-elf Toolchain
  ---------------------------

  Another option is the mips-elf toolchain used with the Pinguino project.  This
  is a relatively current mips-elf GCC and should provide free C++ support as
  well. This toolchain can be downloded from the Pinguino website:
  http://wiki.pinguino.cc/index.php/Main_Page#Download . There is some general
  information about using the Pinguino mips-elf toolchain in this thread:
  http://tech.groups.yahoo.com/group/nuttx/message/1821

  See also configs/mirtoo/README.txt.  There is an experimental (untested)
  configuration for the Mirtoo platform in that directory.

  MPLAB/C32 vs MPLABX/X32
  -----------------------

  It appears that Microchip is phasing out the MPLAB/C32 toolchain and replacing
  it with MPLABX and XC32.  At present, the XC32 toolchain is *not* compatible
  with the NuttX build scripts.  Here are some of the issues that I see when trying
  to build with XC32:

  1) Make.def changes:  You have to change the tool prefix:

     CROSSDEV=xc32-

  2) debug.ld/release.ld:  The like expect some things that are not present in
     the current linker scripts (or are expected with different names).  Here
     are some partial fixes:

     Rename:  kseg0_progmem to kseg0_program_mem
     Rename:  kseg1_datamem to kseg1_data_mem

  Even then, there are more warnings from the linker and some undefined symbols
  for non-NuttX code that resides in the unused Microchip libraries.  See this
  email thread at http://tech.groups.yahoo.com/group/nuttx/message/1458 for more
  information.  You will have to solve at least this undefined symbol problem if
  you want to used the XC32 toolchain.

  Windows Native Toolchains
  -------------------------
  
  NOTE:  There are several limitations to using a Windows based toolchain in a
  Cygwin environment.  The three biggest are:

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

Powering the Board
==================

  [To be provided]

Creating Compatible NuttX HEX files
===================================

  Intel Hex Format Files:
  -----------------------

    When NuttX is built it will produce two files in the top-level NuttX
    directory:

    1) nuttx - This is an ELF file, and
    2) nuttx.hex - This is an Intel Hex format file.  This is controlled by
       the setting CONFIG_INTELHEX_BINARY in the .config file.

    The PICkit tool wants an Intel Hex format file to burn into FLASH. However,
    there is a problem with the generated nutt.hex: The tool expects the nuttx.hex
    file to contain physical addresses.  But the nuttx.hex file generated from the
    top-level make will have address in the KSEG0 and KSEG1 regions.

  tools/pic32mx/mkpichex:
  ----------------------

    There is a simple tool in the NuttX tools/pic32mx directory that can be
    used to solve both issues with the nuttx.hex file.  But, first, you must
    build the tool:

      cd tools/pic32mx
      make

    Now you will have an excecutable file call mkpichex (or mkpichex.exe on
    Cygwin).  This program will take the nutt.hex file as an input, it will
    convert all of the KSEG0 and KSEG1 addresses to physical address, and
    it will write the modified file, replacing the original nuttx.hex.

    To use this file, you need to do the following things:

      . ./setenv.sh    # Source setenv.sh.  Among other this, this script
                       # will add the NuttX tools/pic32mx directory to your
                       # PATH variable
      make             # Build nuttx and nuttx.hex
      mkpichex $PWD    #  Convert addresses in nuttx.hex.  $PWD is the path
                       # to the top-level build directory.  It is the only
                       # required input to mkpichex.

Serial Console
==============

  UART1 is connected to the on-board RS-232 connector

LEDs
====

  The Mikroelektronika PIC32MX7 MMB has 3 user LEDs labeled LED0-2 in the
  schematics:

  ---  ----- ---------------------------------------------------------
  PIN  Board Notes
  ---  ----- ---------------------------------------------------------
  RA0  LED0  Pulled-up, low value illuminates
  RA1  LED1  Pulled-up, low value illuminates
  RD9  LED2  Pulled-up, low value illuminates
  RA9  LED4  Not available for general use*, indicates MMC/SD activity
  ---  LED5  Not controllable by software, indicates power-on

  * RA9 is also the SD chip select.  It will illuminate whenever the SD card
    is selected.  If SD is not used, then LED4 could also be used as a user-
    controlled LED.

 If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as follows:

                            ON                  OFF
  ------------------------- ---- ---- ---- ---- ---- ----
                            LED0 LED1 LED2 LED0 LED1 LED2
  ------------------------- ---- ---- ---- ---- ---- ----
  LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
  LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
  LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
  LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
  LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
  LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
  LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
  LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C

PIC32MX Configuration Options
=============================

  General Architecture Settings:

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
     be set to:

       CONFIG_ARCH=mips

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_MIPS=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_MIPS32=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=pic32mx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_PIC32MX795F512L=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=pic32mx7mmb

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_PIC32MX7MMB=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_DRAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0xa0000000

    CONFIG_ARCH_IRQPRIO - The PIC32MXx supports interrupt prioritization

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

    PIC32MX Configuration

      CONFIG_PIC32MX_MVEC - Select muli- vs. single-vectored interrupts

    Individual subsystems can be enabled:

       CONFIG_PIC32MX_WDT            - Watchdog timer
       CONFIG_PIC32MX_T2             - Timer 2 (Timer 1 is the system time and always enabled)
       CONFIG_PIC32MX_T3             - Timer 3
       CONFIG_PIC32MX_T4             - Timer 4
       CONFIG_PIC32MX_T5             - Timer 5
       CONFIG_PIC32MX_IC1            - Input Capture 1
       CONFIG_PIC32MX_IC2            - Input Capture 2
       CONFIG_PIC32MX_IC3            - Input Capture 3
       CONFIG_PIC32MX_IC4            - Input Capture 4
       CONFIG_PIC32MX_IC5            - Input Capture 5
       CONFIG_PIC32MX_OC1            - Output Compare 1
       CONFIG_PIC32MX_OC2            - Output Compare 2
       CONFIG_PIC32MX_OC3            - Output Compare 3
       CONFIG_PIC32MX_OC4            - Output Compare 4
       CONFIG_PIC32MX_OC5            - Output Compare 5
       CONFIG_PIC32MX_I2C1           - I2C 1
       CONFIG_PIC32MX_I2C2           - I2C 2
       CONFIG_PIC32MX_I2C3           - I2C 3
       CONFIG_PIC32MX_I2C4           - I2C 4
       CONFIG_PIC32MX_I2C5           - I2C 5
       CONFIG_PIC32MX_SPI1           - SPI 1
       CONFIG_PIC32MX_SPI2           - SPI 2
       CONFIG_PIC32MX_SPI3           - SPI 3
       CONFIG_PIC32MX_SPI4           - SPI 4
       CONFIG_PIC32MX_UART1          - UART 1
       CONFIG_PIC32MX_UART2          - UART 2
       CONFIG_PIC32MX_UART3          - UART 3
       CONFIG_PIC32MX_UART4          - UART 4
       CONFIG_PIC32MX_UART5          - UART 5
       CONFIG_PIC32MX_UART6          - UART 6
       CONFIG_PIC32MX_ADC            - ADC 1
       CONFIG_PIC32MX_PMP            - Parallel Master Port
       CONFIG_PIC32MX_CM1            - Comparator 1
       CONFIG_PIC32MX_CM2            - Comparator 2
       CONFIG_PIC32MX_RTCC           - Real-Time Clock and Calendar
       CONFIG_PIC32MX_DMA            - DMA
       CONFIG_PIC32MX_FLASH          - FLASH
       CONFIG_PIC32MX_USBDEV         - USB device
       CONFIG_PIC32MX_USBHOST        - USB host
       CONFIG_PIC32MX_CAN1           - Controller area network 1
       CONFIG_PIC32MX_CAN2           - Controller area network 2
       CONFIG_PIC32MX_ETHERNET       - Ethernet

    PIC32MX Configuration Settings
    DEVCFG0:
      CONFIG_PIC32MX_DEBUGGER - Background Debugger Enable. Default 3 (disabled). The
        value 2 enables.
      CONFIG_PIC32MX_ICESEL - In-Circuit Emulator/Debugger Communication Channel Select
        Default 1 (PG2)
      CONFIG_PIC32MX_PROGFLASHWP  - Program FLASH write protect.  Default 0xff (disabled)
      CONFIG_PIC32MX_BOOTFLASHWP - Default 1 (disabled)
      CONFIG_PIC32MX_CODEWP - Default 1 (disabled)
    DEVCFG1: (All settings determined by selections in board.h)
    DEVCFG2: (All settings determined by selections in board.h)
    DEVCFG3: 
      CONFIG_PIC32MX_USBIDO - USB USBID Selection.  Default 1 if USB enabled
        (USBID pin is controlled by the USB module), but 0 (GPIO) otherwise.
      CONFIG_PIC32MX_VBUSIO - USB VBUSON Selection (Default 1 if USB enabled
        (VBUSON pin is controlled by the USB module, but 0 (GPIO) otherwise.
      CONFIG_PIC32MX_WDENABLE - Enabled watchdog on power up.  Default 0 (watchdog
        can be enabled later by software).

    The priority of interrupts may be specified.  The value ranage of
    priority is 4-31. The default (16) will be used if these any of these
    are undefined.

       CONFIG_PIC32MX_CTPRIO         - Core Timer Interrupt
       CONFIG_PIC32MX_CS0PRIO        - Core Software Interrupt 0
       CONFIG_PIC32MX_CS1PRIO        - Core Software Interrupt 1
       CONFIG_PIC32MX_INT0PRIO       - External Interrupt 0
       CONFIG_PIC32MX_INT1PRIO       - External Interrupt 1
       CONFIG_PIC32MX_INT2PRIO       - External Interrupt 2
       CONFIG_PIC32MX_INT3PRIO       - External Interrupt 3
       CONFIG_PIC32MX_INT4PRIO       - External Interrupt 4
       CONFIG_PIC32MX_FSCMPRIO       - Fail-Safe Clock Monitor
       CONFIG_PIC32MX_T1PRIO         - Timer 1 (System timer) priority
       CONFIG_PIC32MX_T2PRIO         - Timer 2 priority
       CONFIG_PIC32MX_T3PRIO         - Timer 3 priority
       CONFIG_PIC32MX_T4PRIO         - Timer 4 priority
       CONFIG_PIC32MX_T5PRIO         - Timer 5 priority
       CONFIG_PIC32MX_IC1PRIO        - Input Capture 1
       CONFIG_PIC32MX_IC2PRIO        - Input Capture 2
       CONFIG_PIC32MX_IC3PRIO        - Input Capture 3
       CONFIG_PIC32MX_IC4PRIO        - Input Capture 4
       CONFIG_PIC32MX_IC5PRIO        - Input Capture 5
       CONFIG_PIC32MX_OC1PRIO        - Output Compare 1
       CONFIG_PIC32MX_OC2PRIO        - Output Compare 2
       CONFIG_PIC32MX_OC3PRIO        - Output Compare 3
       CONFIG_PIC32MX_OC4PRIO        - Output Compare 4
       CONFIG_PIC32MX_OC5PRIO        - Output Compare 5
       CONFIG_PIC32MX_I2C1PRIO       - I2C 1
       CONFIG_PIC32MX_I2C2PRIO       - I2C 2
       CONFIG_PIC32MX_I2C3PRIO       - I2C 3
       CONFIG_PIC32MX_I2C4PRIO       - I2C 4
       CONFIG_PIC32MX_I2C5PRIO       - I2C 5
       CONFIG_PIC32MX_SPI2PRIO       - SPI 2
       CONFIG_PIC32MX_UART1PRIO      - UART 1
       CONFIG_PIC32MX_UART2PRIO      - UART 2
       CONFIG_PIC32MX_CN             - Input Change Interrupt
       CONFIG_PIC32MX_ADCPRIO        - ADC1 Convert Done
       CONFIG_PIC32MX_PMPPRIO        - Parallel Master Port
       CONFIG_PIC32MX_CM1PRIO        - Comparator 1
       CONFIG_PIC32MX_CM2PRIO        - Comparator 2
       CONFIG_PIC32MX_FSCMPRIO       - Fail-Safe Clock Monitor
       CONFIG_PIC32MX_RTCCPRIO       - Real-Time Clock and Calendar
       CONFIG_PIC32MX_DMA0PRIO       - DMA Channel 0
       CONFIG_PIC32MX_DMA1PRIO       - DMA Channel 1
       CONFIG_PIC32MX_DMA2PRIO       - DMA Channel 2
       CONFIG_PIC32MX_DMA3PRIO       - DMA Channel 3
       CONFIG_PIC32MX_DMA4PRIO       - DMA Channel 4
       CONFIG_PIC32MX_DMA5PRIO       - DMA Channel 5
       CONFIG_PIC32MX_DMA6PRIO       - DMA Channel 6
       CONFIG_PIC32MX_DMA7PRIO       - DMA Channel 7
       CONFIG_PIC32MX_FCEPRIO        - Flash Control Event
       CONFIG_PIC32MX_USBPRIO        - USB

  PIC32MXx specific device driver settings.  NOTE:  For the Ethernet
  starter kit, there is no RS-232 connector (even with the MEB).  See
  discussion above ("") for information about how you can configure
  an external MAX2232 board to get a serial console.

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

PIC32MX specific PHY/Ethernet device driver settings

    CONFIG_PHY_KS8721 - Selects the Micrel KS8721 PHY
    CONFIG_PHY_DP83848C - Selects the National Semiconduction DP83848C PHY
    CONFIG_PHY_LAN8720 - Selects the SMSC LAN8720 PHY
    CONFIG_PHY_AUTONEG - Enable auto-negotion
    CONFIG_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_PHY_FDUPLEX - Select full (vs. half) duplex
    CONFIG_NET_NTXDESC - Configured number of Tx descriptors. Default: 2
    CONFIG_NET_NRXDESC - Configured number of Rx descriptors. Default: 4
    CONFIG_NET_PRIORITY - Ethernet interrupt priority.  The is default is
      the higest priority.
    CONFIG_NET_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG.
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG.
    CONFIG_NET_HASH - Enable receipt of near-perfect match frames.
    CONFIG_NET_MULTICAST - Enable receipt of multicast (and unicast) frames.
      Automatically set if CONFIG_NET_IGMP is selected.

  Related DEVCFG3 Configuration Settings:
    CONFIG_PIC32MX_FETHIO: Ethernet I/O Pin Selection bit:
      1 = Default Ethernet I/O Pins
      0 = Alternate Ethernet I/O Pins
    CONFIG_PIC32MX_FMIIEN: Ethernet MII Enable bit
      1 = MII enabled
      0 = RMII enabled

  PIC32MXx USB Device Configuration

  PIC32MXx USB Host Configuration (the PIC32MX does not support USB Host)

Configurations
==============

Each PIC32MX configuration is maintained in a sudirectory and can be
selected as follow:

    cd tools
    ./configure.sh pic32mx7mmb/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  ostest:
  =======
    Description.
    ------------
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

    Serial Output.
    --------------
    The OS test produces all of its test output on the serial console.
    This configuration has UART1 enabled as a serial console.

  nsh:
  ====
    Description.
    ------------
    This is the NuttShell (NSH) using the NSH startup logic at
    apps/examples/nsh.

    Serial Output.
    --------------
    The OS test produces all of its test output on the serial console.
    This configuration has UART1 enabled as a serial console.

    SD Card Support
    ---------------
    SD card support is built into this example by default:
  
       CONFIG_PIC32MX_SPI1=y
       CONFIG_NSH_ARCHINIT=y

    The SD card can be mounted from the NSH command line as follows:

       nsh> mount -t vfat /dev/mmcsd0 /mnt/sdcard
       nsh> ls -l /mnt/sdcard
       /mnt/sdcard:
        -rw-rw-rw-      16 ATEST.TXT
        -rw-rw-rw-   21170 TODO
        -rw-rw-rw-      22 ANOTHER.TXT
        -rw-rw-rw-      22 HI2148.TXT
        -rw-rw-rw-      16 HiFromNotePad.txt

    USB Configurations.
    ------------------
    USB device support is enabled by default in this configuration.
    The following settings are defined by default (and can be set
    to 'n' to disabled USB device support).
 
      CONFIG_USBDEV=y         : Enable basic USB device support
      CONFIG_PIC32MX_USBDEV=y : Enable PIC32 USB device support
      CONFIG_USBMSC=y         : USB supports a mass storage device.

    In this configuration, NSH will support the following commands:

      msconn  : Connect the mass storage device, exportint the SD
                card as the USB mass storage logical unit.
      msdis   : Disconnect the USB mass storage device
 
    NOTE: The SD card should *not* be mounted under NSH *and* exported
    by the mass storage device!!! That can result in corruption of the
    SD card format.  This is the sequence of commands that you should
    used to work the the SD card safely:

      mount -t vfat /dev/mmcsd0 /mnt/sdcard : Mount the SD card initially
      ...
      umount /mnt/sdcard   : Unmount the SD card before connecting
      msconn               : Connect the USB MSC
      ...
      msdis                : Disconnect the USB MSC
      mount -t vfat /dev/mmcsd0 /mnt/sdcard : Re-mount the SD card
      ...
      
    Other USB other device configurations can be enabled and
    included as NSH built-in built in functions.

    examples/usbterm - This option can be enabled by uncommenting
    the following line in the appconfig file:

      CONFIGURED_APPS += examples/usbterm

    And by enabling one of the USB serial devices:

      CONFIG_USBMSC=n         : Disable USB mass storage device.
      CONFIG_PL2303=y         : Enable the Prolifics PL2303 emulation
      CONFIG_CDCACM=y         : or the CDC/ACM serial driver (not both)

    examples/cdcacm -  The examples/cdcacm program can be included as an 
    function by uncommenting the following line in the appconfig file:
    
      CONFIGURED_APPS += examples/cdcacm

    and defining the following in your .config file:

      CONFIG_USBMSC=n         : Disable USB mass storage device.
      CONFIG_CDCACM=y         : Enable the CDCACM device

    Networking Configurations.
    --------------------------
    Networking is enabled by default in this configuration:

      CONFIG_NET=y              : Enable networking support
      CONFIG_PIC32MX_ETHERNET=y : Enable the PIC32 Ethernet driver
      CONFIG_NSH_TELNET=y       : Enable the Telnet NSH console (optional)

    The default configuration has:

      CONFIG_NSH_DHCPC=n                        : DHCP is disabled
      CONFIG_NSH_IPADDR=(10<<24|0<<16|0<<8|2)   : Target IP address 10.0.0.2
      CONFIG_NSH_DRIPADDR=(10<<24|0<<16|0<<8|1) : Host IP address 10.0.0.1

    This will probably need to be customized for your network.
 
    NOTES:
    1. This logic will assume that a network is connected.  During its
       initialization, it will try to negotiate the link speed.  If you have
       no network connected when you reset the board, there will be a long
       delay (maybe 30 seconds?) before anything happens.  That is the timeout
       before the networking finally gives up and decides that no network is
       available.

    2. This example can support an FTP client.  In order to build in FTP client
       support simply uncomment the following lines in the appconfig file (before
       configuring) or in the apps/.config file (after configuring):

       #CONFIGURED_APPS += netutils/ftpc
       #CONFIGURED_APPS += examples/ftpc

    3. This example can support an FTP server.  In order to build in FTP server
       support simply uncomment the following lines in the appconfig file (before
       configuring) or in the apps/.config file (after configuring):

       #CONFIGURED_APPS += netutils/ftpd
       #CONFIGURED_APPS += examples/ftpd

       And enable poll() support in the NuttX configuration file:

       CONFIG_DISABLE_POLL=n

  Using a RAM disk and the USB MSC device to the nsh configuration
  ----------------------------------------------------------------
  Here is an experimental change to examples/nsh that will create a RAM
  disk and attempt to export that RAM disk as a USB mass storage device.

  1. Changes to nuttx/.config

    a) Enable support for the PIC32 USB device

      -CONFIG_PIC32MX_USBDEV=n 
      +CONFIG_PIC32MX_USBDEV=y

    b) Enable NuttX USB device support

      -CONFIG_USBDEV=n
      +CONFIG_USBDEV=y

    c) Enable the USB MSC class driver

      -CONFIG_USBMSC=n
      +CONFIG_USBMSC=y

    d) Use a RAM disk (instead of an SD card) as the USB MSC logical unit:

      -CONFIG_EXAMPLES_USBMSC_DEVPATH1="/dev/mmcsd0"
      +CONFIG_EXAMPLES_USBMSC_DEVPATH1="/dev/ram0"

  2. Changes to nuttx/.config.

    a) Enable building of the examples/usbstorage:

      -# CONFIGURED_APPS += examples/usbstorage
      +  CONFIGURED_APPS += examples/usbstorage

  3. When NSH first comes up, you must manually create the RAM disk
     before exporting it:

    a) Create a 64Kb RAM disk at /dev/ram0:

      nsh> mkrd -s 512 128

    b) Put a FAT file system on the RAM disk:
  
      nsh> mkfatfs /dev/ram0

    b) Now the 'msconn' command will connect to the host and
       export /dev/ram0 as the USB logical unit:

      nsh> msconn

    NOTE:  This modification should be considered experimental.  IN the
    little testing I have done with it, it appears functional.  But the
    logic has not been stressed and there could still be lurking issues.
    (There is a bug associated with this configuration listed in the
    top-level TODO list).

  Adding LCD and graphics support to the nsh configuration:
  --------------------------------------------------------

    LCD support is already enabled in defconfig (nuttx/.config):

      CONFIG_NX=y                          : Enable graphics suppport
      CONFIG_PIC32MX_PMP=y                 : Enable parallel port support
      CONFIG_LCD_MIO283QT2=y               : MIO283QT2 LCD support

    But you will have to enable a specific graphics example application
    in order to see anything.
 
    appconfig (apps/.config):  Enable the application configurations that you
    want to use.  Asexamples:

      CONFIGURED_APPS += examples/nx       : Pick one or more
      CONFIGURED_APPS += examples/nxhello  :
      CONFIGURED_APPS += examples/nximage  :
      CONFIGURED_APPS += examples/nxlines  :

  Enabling touch screen support in the nsh configuaration
  -------------------------------------------------------

    In defconfig (or nuttx/.config), set:

      CONFIG_INPUT=y                       : Enable input device support
      CONFIG_SCHED_WORKQUEUE=y             : Work queue support needed

    In appconfig (or apps/.config), uncomment:

      CONFIGURED_APPS += examples/touchscreen
