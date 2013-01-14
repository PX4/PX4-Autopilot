configs/ubw32 README
====================

This README file discusses the port of NuttX to the Sparkfun UBW32 board.
This port uses the original v2.5 board which is based on the MicroChip
PIC32MX460F512L.  See http://www.sparkfun.com/products/8971.  This older
version has been replaced with this board http://www.sparkfun.com/products/9713.
See also http://www.schmalzhaus.com/UBW32/.

Contents
========

  PIC32MX460F512L Pin Out
  MAX3232 Connection
  Toolchains
  Loading NuttX with PICkit2
  LEDs
  Buttons
  PIC32MX Configuration Options
  Configurations

PIC32MX460F512L Pin Out
=======================

  PIC32MX460F512L 100-Pin TQFP (USB) Pin Out.  The mapping to the pins on
  the PCL Logic board are very simple, each pin is brought out to a connector
  label with the PIC32MX460F512L pin number.

  On board logic only manages power, crystal, and USB signals.

  LEFT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
  ---- ----------------------------  -------------------------------------------------------
  PIN  NAME                          Board Connection (omitting pins brought to J3 and J4)
  ---- ----------------------------  -------------------------------------------------------
    1  RG15
    2  Vdd
    3  PMD5/RE5
    4  PMD6/RE6                      RE6    User switch
    5  PMD7/RE7                      RE7    Program switch
    6  T2CK/RC1
    7  T3CK/RC2
    8  T4CK/RC3
    9  T5CK/SDI1/RC4
   10  SCK2/PMA5/CN8/RG6
   11  SDI2/PMA4/CN9/RG7
   12  SDO2/PMA3/CN10/RG8
   13  MCLR
   14  SS2/PMA2/CN11/RG9
   15  Vss
   16  Vdd
   17  TMS/RA0
   18  INT1/RE8
   19  INT2/RE9
   20  AN5/C1IN+/VBUSON/CN7/RB5
   21  AN4/C1IN-/CN6/RB4
   22  AN3/C2IN+/CN5/RB3
   23  AN2/C2IN-/CN4/RB2             RB1    ICSP/Debug/IO (J5) pin 6 (labeled B2)
   24  PGEC1/AN1/CN3/RB1             RB1    ICSP/Debug/IO (J5) pin 7 (labeled B1)
   25  PGED1/AN0/CN2/RB0             RB0    ICSP/Debug/IO (J5) pin 8 (labeled B0)

  BOTTOM SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
  ---- ----------------------------  -------------------------------------------------------
  PIN  NAME                          Board Connection (omitting pins brought to J3 and J4)
  ---- ----------------------------  -------------------------------------------------------
   26  PGEC2/AN6/OCFA/RB6            PGC    ICSP/Debug/IO (J5) pin 5 (labled PGC)
   27  PGED2/AN7/RB7                 PGD    ICSP/Debug/IO (J5) pin 4 (labled PGD)
   28  VREF-/CVREF-/PMA7/RA9
   29  VREF+/CVREF+/PMA6/RA10
   30  AVdd
   31  AVss
   32  AN8/C1OUT/RB8
   33  AN9/C2OUT/RB9
   34  AN10/CVREFOUT/PMA13/RB10
   35  AN11/PMA12/RB11
   36  Vss
   37  Vdd
   38  TCK/RA1
   39  U2RTS/RF13
   40  U2CTS/RF12
   41  AN12/PMA11/RB12
   42  AN13/PMA10/RB13
   43  AN14/PMALH/PMA1/RB14
   44  AN15/OCFB/PMALL/PMA0/CN12/RB15
   45  Vss
   46  Vdd
   47  U1CTS/CN20/RD14
   48  U1RTS/CN21/RD15
   49  U2RX/PMA9/CN17/RF4
   50  U2TX/PMA8/CN18/RF5

  RIGHT SIDE, TOP-TO-BOTTOM (if pin 1 is in upper left)
  ---- ----------------------------  -------------------------------------------------------
  PIN  NAME                          Board Connection (omitting pins brought to J3 and J4)
  ---- ----------------------------  -------------------------------------------------------
   75  Vss
   74  SOSCO/T1CK/CN0/RC14
   73  SOSCI/CN1/RC13
   72  SDO1/OC1/INT0/RD0
   71  IC4/PMCS1/PMA14/RD11
   70  SCK1/IC3/PMCS2/PMA15/RD10
   69  SS1/IC2/RD9
   68  RTCC/IC1/RD8
   67  SDA1/INT4/RA15
   66  SCL1/INT3/RA14
   65  Vss
   64  OSC2/CLKO/RC15
   63  OSC1/CLKI/RC12
   62  Vdd
   61  TDO/RA5
   60  TDI/RA4
   59  SDA2/RA3
   58  SCL2/RA2
   57  D+/RG2                        D+     Alternate USB (J6) pin 3 (labeled D+)
                                            USB host (JP1) pin
                                            USB Function (Mini B)
   56  D-/RG3                        D-     Alternate USB (J6) pin 2 (labeled D-)
                                            USB host (JP1) pin
                                            USB Function (Mini B)
   55  VUSB
   54  VBUS                          VBUS   Alternate USB (J6) pin 1 (labeled VBUS)
                                            USB host (JP1) pin
                                            USB Function (Mini B)
                                            Jumper JP1 for USB host functionality
   53  U1TX/RF8
   52  U1RX/RF2
   51  USBID/RF3                     USBID  Alternate USB (J6) pin 4 (labeled ID)
                                            USB Function (Mini B)
                                            Also USB LED

  TOP SIDE, LEFT-TO-RIGHT (if pin 1 is in upper left)
  ---- ----------------------------  -------------------------------------------------------
  PIN  NAME                          Board Connection (omitting pins brought to J3 and J4)
  ---- ----------------------------  -------------------------------------------------------
  100  PMD4/RE4
   99  PMD3/RE3
   98  PMD2/RE2                      RE2    LED1
   97  TRD0/RG13
   96  TRD1/RG12
   95  TRD2/RG14
   94  PMD1/RE1                      RE1    LED2
   93  PMD0/RE0                      RE0    LED3
   92  TRD3/RA7
   91  TRCLK/RA6
   90  PMD8/RG0
   89  PMD9/RG1
   88  PMD10/RF1
   87  PMD11/RF0
   86  ENVREG
   85  Vcap/Vddcore
   84  PMD15/CN16/RD7
   83  PMD14/CN15/RD6
   82  PMRD/CN14/RD5
   81  OC5/PMWR/CN13/RD4
   80  PMD13/CN19/RD13
   79  IC5/PMD12/RD12
   78  OC4/RD3
   77  OC3/RD2
   76  OC2/RD1

MAX3232 Connection
==================

  I use a tiny, MAX3232 board that I got from the eBay made by NKC
  Electronics (http://www.nkcelectronics.com/).  As of this writing, it
  is also available here: http://www.nkcelectronics.com/rs232-to-ttl-3v--55v-convert232356.html

  CTS -- Not connected
  RTS -- Not connected
  TX  -- J4 pin 31: U1TX/RF8
  RX  -- J4 pin 30: U1RX/RF2
  GND -- J4 pin 40: GND
  Vcc -- J4 pin 39: 5V
 
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

     -CROSSDEV=pic32-
     +CROSSDEV=xc32-

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

       MKDEP                = $(TOPDIR)/tools/mknulldeps.sh

Loading NuttX with PICkit2
==========================

  NOTE:  You need a PICKit3 if you plan to use the MPLAB debugger!  The PICKit2
  can, however, still be used to load programs.  Instructions for the PICKit3
  are similar.

  Intel Hex Forma Files:
  ----------------------

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
      mkpichex $PWD    # Convert addresses in nuttx.hex.  $PWD is the path
                       # to the top-level build directory.  It is the only
                       # required input to mkpichex.

LEDs
====

  ----- ----- -------------------------------------------------------------
  LABEL COLOR CONTROL
  ----- ----- -------------------------------------------------------------
  USB   Green   RF3. This could be used by software if USB is not used. 
                     Otherwise, RF3 is used as the USBID signal.
  LED1  While   RE2, Pulled up.  Low value illuminates
  LED2  Red     RE1, Pulled up.  Low value illuminates
  LED3  Yellow  RE0, Pulled up.  Low value illuminates
  PWR   Blue    Illuminated when 5V is present, not controlled by software

  If CONFIG_ARCH_LEDS is defined, then NuttX will control these LEDs as follows:

                            ON                  OFF
  ------------------------- ---- ---- ---- ---- ---- ----
                            LED1 LED2 LED3 LED1 LED2 LED3
  ------------------------- ---- ---- ---- ---- ---- ----
  LED_STARTED            0  OFF  OFF  OFF  ---  ---  ---
  LED_HEAPALLOCATE       1  ON   OFF  N/C  ---  ---  ---
  LED_IRQSENABLED        2  OFF  ON   N/C  ---  ---  ---
  LED_STACKCREATED       3  ON   ON   N/C  ---  ---  ---
  LED_INIRQ              4  N/C  N/C  ON   N/C  N/C  OFF
  LED_SIGNAL             4  N/C  N/C  ON   N/C  N/C  OFF
  LED_ASSERTION          4  N/C  N/C  ON   N/C  N/C  OFF
  LED_PANIC              5  ON   N/C  N/C  OFF  N/C  N/C

Buttons
=======

    RE6 User switch
    RE7 Program switch
 
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

       CONFIG_ARCH_CHIP_PIC32MX460F512L=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=ubw32

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_UBW32=y

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
       CONFIG_PIC32MX_SPI1           - SPI 1
       CONFIG_PIC32MX_SPI2           - SPI 2
       CONFIG_PIC32MX_UART1          - UART 1
       CONFIG_PIC32MX_UART2          - UART 2
       CONFIG_PIC32MX_ADC            - ADC 1
       CONFIG_PIC32MX_PMP            - Parallel Master Port
       CONFIG_PIC32MX_CM1            - Comparator 1
       CONFIG_PIC32MX_CM2            - Comparator 2
       CONFIG_PIC32MX_RTCC           - Real-Time Clock and Calendar
       CONFIG_PIC32MX_DMA            - DMA
       CONFIG_PIC32MX_FLASH          - FLASH
       CONFIG_PIC32MX_USBDEV         - USB device
       CONFIG_PIC32MX_USBHOST        - USB host

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
       CONFIG_PIC32MX_SPI1PRIO       - SPI 1
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
       CONFIG_PIC32MX_FCEPRIO        - Flash Control Event
       CONFIG_PIC32MX_USBPRIO        - USB

  PIC32MXx specific device driver settings

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

  PIC32MXx USB Device Configuration

  PIC32MXx USB Host Configuration (the PIC32MX does not support USB Host)

Configurations
==============

Each PIC32MX configuration is maintained in a sudirectory and can be
selected as follow:

    cd tools
    ./configure.sh ubw32/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  ostest:
  =======
    Description.
    ------------
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

    NOTES:
    ------ 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Serial Output.

       The OS test produces all of its test output on the serial console.
       This configuration has UART1 enabled as a serial console. 

       TX  -- J4 pin 31: U1TX/RF8
       RX  -- J4 pin 30: U1RX/RF2
       GND -- J4 pin 40: GND
       Vcc -- J4 pin 39: 5V
 
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

    TX  -- J4 pin 31: U1TX/RF8
    RX  -- J4 pin 30: U1RX/RF2
    GND -- J4 pin 40: GND
    Vcc -- J4 pin 39: 5V
 
    USB Configuations.
    -----------------
    Several USB device configurations can be enabled and included
    as NSH built-in built in functions.

    All USB device configurations require the following basic setup in
    your NuttX configuration file to enable USB device support:

      CONFIG_USBDEV=y         : Enable basic USB device support
      CONFIG_PIC32MX_USBDEV=y : Enable PIC32 USB device support

    examples/usbterm - This option can be enabled by uncommenting
    the following line in the appconfig file:

      CONFIGURED_APPS += examples/usbterm

    And by enabling one of the USB serial devices:

      CONFIG_PL2303=y         : Enable the Prolifics PL2303 emulation
      CONFIG_CDCACM=y         : or the CDC/ACM serial driver (not both)

    examples/cdcacm -  The examples/cdcacm program can be included as an
    function by uncommenting the following line in the appconfig file:

      CONFIGURED_APPS += examples/cdcacm

    and defining the following in your .config file:

      CONFIG_CDCACM=y         : Enable the CDCACM device
