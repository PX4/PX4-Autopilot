README
^^^^^

This is the README file for the port of NuttX to the Micropendous 3 board.
This board is develepmend by http://code.google.com/p/opendous/.  The
Micropendous 3 is based on an Atmel AT90USB646, 647, 1286 or 1287 MCU.
NuttX was ported using the AT90USB647 version.  As of this writing,
documentation for the Micropendous board is available here:
http://code.google.com/p/micropendous/wiki/Micropendous3

Contents
^^^^^^^^

  o Micropendous3 Features
  o Pin Usage
  o Atmel AVRISP mkII Connection
  o DFU Bootloader
  o Serial Console
  o Toolchains
  o Windows Native Toolchains
  o NuttX buildroot Toolchain
  o avr-libc
  o Micropendous3 Configuration Options
  o Configurations

Micropendous3 Features
^^^^^^^^^^^^^^^^^^^^^^

  o Based on the 64-pin USB AVR Microcontrollers: AT90USB646, AT90USB647,
    AT90USB1286, or AT90USB1287.
  o USB Full Speed (12Mbit/s)
  o USB Device Mode (Host mode supported with AT90USBxx7 devices)
  o 60kb (AT90USB64) or 120kb (AT90USB128) of available FLASH memory for
    your programs (4kb(AT90USB64)/8kb(AT90USB128) used by USB bootloader -
    stock Atmel or LUFA)
  o 4 kbytes SRAM and 2 kbytes of EEPROM (AT90USB64) or 8 kbytes SRAM and 4
    kbytes of EEPROM (AT90USB128)
  o External SRAM is possible.  Layout for CY7C1019D 1-Mbit SRAM (unpopulated)
  o USB powered
  o 16MHz crystal
  o 48 General Purpose IO Pins (47 with external SRAM)
  o Vcc=VBUS jumper selects whether USB VBUS or an external supply is used
    to power the board
  o RESET and HWB buttons to enable firmware loading over USB (no external
    programmer required)
  o HWB can be used as a user button
  o USB-A Plug
  o JTAG header
  o Size LxWxH (including headers): 3.15" x 0.8" x 0.6" =~ 8cm x 2cm x 1.5cm
  o Completely OpenHardware Design

Pin Usage
^^^^^^^^^

  AT90USB90128/64 TQFP64
  -- ------------------------ ---------------------------------------------
  PIN SIGNAL                  BOARD CONNECTION
  -- ------------------------ ---------------------------------------------
  (left)
  1  (INT.6/AIN.0) PE6         J3-25 E6, CY7C1019D ^CE (Unpopulated)
  2  (INT.7/AIN.1/UVcon) PE7   J3-26 E7, CY7C1019D A16 (Unpopulated)
  3  UVcc                      
  4  D-                        USB DP
  5  D+                        USB DM
  6  UGnd                      GND
  7  UCap                      GND (via cap)
  8  VBus                      USB VBUS
  9  (IUID) PE3                J3-22 E3
  10 (SS/PCINT0) PB0           J3-28 B0
  11 (PCINT1/SCLK) PB1         J3-29 B1
  12 (PDI/PCINT2/MOSI) PB2     J3-30 B2
  13 (PDO/PCINT3/MISO) PB3     J3-31 B3
  14 (PCINT4/OC.2A) PB4        J3-32 B4
  15 (PCINT5/OC.1A) PB5        J3-33 B5
  16 (PCINT6/OC.1B) PB6        J3-34 B6
  (bottom)
  17 (PCINT7/OC.0A/OC.1C) PB7  J3-35 B7
  18 (INT4/TOSC1) PE4          J3-23 E4
  19 (INT.5/TOSC2) PE5         J3-24 E5
  20 RESET                     SW1
  21 VCC                       VCC
  22 GND                       GND
  23 XTAL2                     X1
  24 XTAL1                     X1
  25 (OC0B/SCL/INT0) PD0       J3-36 D0
  26 (OC2B/SDA/INT1) PD1       J3-37 D1
  27 (RXD1/INT2) PD2           J3-38 D2
  28 (TXD1/INT3) PD3           J3-39 D3
  29 (ICP1) PD4                J3-40 D4
  30 (XCK1) PD5                J3-41 D5
  31 (T1) PD6                  J3-42 D6
  32 (T0) PD7                  J3-43 D7
  (right)
  48 PA3 (AD3)                 J3-14 A3, 74AHC573 D3, CY7C1019D |O3 (Unpopulated)
  47 PA4 (AD4)                 J3-15 A4, 74AHC573 D4, CY7C1019D |O4 (Unpopulated)
  46 PA5 (AD5)                 J3-16 A5, 74AHC573 D5, CY7C1019D |O5 (Unpopulated)
  45 PA6 (AD6)                 J3-17 A6, 74AHC573 D6, CY7C1019D |O6 (Unpopulated)
  44 PA7 (AD7)                 J3-18 A7, 74AHC573 D7, CY7C1019D |O7 (Unpopulated)
  43 PE2 (ALE/HWB)             SW-2 (pulled-up), J3-21 E2, 74AHC573 Cp
  42 PC7 (A15/IC.3/CLKO)       J3-51 C7, CY7C1019D A15 (Unpopulated)
  41 PC6 (A14/OC.3A)           J3-50 C6, CY7C1019D A14 (Unpopulated)
  40 PC5 (A13/OC.3B)           J3-49 C5, CY7C1019D A13 (Unpopulated)
  39 PC4 (A12/OC.3C)           J3-48 C4, CY7C1019D A12 (Unpopulated)
  38 PC3 (A11/T.3)             J3-47 C3, CY7C1019D A11 (Unpopulated)
  37 PC2 (A10)                 J3-46 C2, CY7C1019D A10 (Unpopulated)
  36 PC1 (A9)                  J3-45 C1, CY7C1019D A9  (Unpopulated)
  35 PC0 (A8)                  J3-44 C0, CY7C1019D A8  (Unpopulated)
  34 PE1 (RD)                  J3-20 E1, CY7C1019D ^OE (Unpopulated)
  33 PE0 (WR)                  J3-19 E0, CY7C1019D ^WE (Unpopulated)
  (top)
  64 AVCC                      (Power circuitry)
  63 GND                       GND
  62 AREF                      J3-2 AREF, (Power circuitry)
  61 PF0 (ADC0)                J3-3 F0
  60 PF1 (ADC1)                J3-4 F1
  59 PF2 (ADC2)                J3-5 F2
  58 PF3 (ADC3)                J3-6 F3
  57 PF4 (ADC4/TCK)            J3-7 F4, JTAG TCK
  56 PF5 (ADC5/TMS)            J3-8 F5, JTAG TMS
  55 PF6 (ADC6/TDO)            J3-9 F6, JTAG TD0
  54 PF7 (ADC7/TDI)            J3-20 F7, JTAG TDI
  53 GND                       GND
  52 VCC                       VCC
  51 PA0 (AD0)                 J3-11 A0, 74AHC573 D0, CY7C1019D |O0 (Unpopulated)
  50 PA1 (AD1)                 J3-12 A1, 74AHC573 D1, CY7C1019D |O1 (Unpopulated)
  49 PA2 (AD2)                 J3-13 A2, 74AHC573 D2, CY7C1019D |O2 (Unpopulated)

Atmel AVRISP mkII Connection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  ISP6PIN Header
  --------------

        1  2
  MISO  o  o VCC
   SCK  o  o MOSI
  RESET o  o GND

  Micropendous 3 JTAG (JTAG10PIN Connector)
  ------------------- ---------------------
  
      1  2                 1  2
  TCK o  o GND         TCK o  o GND
  TDO o  o VCC         TDO o  o VTref
  TMS o  o RESET       TMS o  o nSRST
  VCC o  o N/C             o  o (nTRST)
  TDI o  o GND         TDI o  o GND

  JTAGICE mkII Connection to 10-pin Header
  ------------------------------------------
  10PIN Header         6PIN Header
  -------------------- ---------------------
  Pin 1 TCK            Pin 3 SCK
  Pin 2 GND            Pin 6 GND
  Pin 3 TDO            Pin 1 MISO
  Pin 4 VTref          Pin 2 Vcc
  Pin 6 nSRT           Pin 5 Reset
  Pin 9 TDI            Pin 4 MOSI

DFU Bootloader
^^^^^^^^^^^^^^

There is also an DFU bootloader that resides in the upper 8Kb of FLASH
(unless you ERASE the flash with with ICE).  You can enter this bootloader
(if it is in FLASH) by:

Holding both the SW1 (RESET) and SW2, then releasing SW1 while continuing
to hold SW2.  SW2 connects to the PE2/HWB signal and causes a reset into
the bootloader memory region.

Then you can use FLIP to load code into FLASH (available at the Atmel Web
Site).  The DFU USB driver for the DFU bootload is available in the usb
subdirectory in the FLIP installation location.

Serial Console
^^^^^^^^^^^^^^

  A serial console is supported on an external MAX232/MAX3232 Connected
  on PD2 and PD3:

  Port D, Bit 2: RXD1, Receive Data (Data input pin for the USART1). When
    the USART1 receiver is enabled this pin is configured as an input
    regardless of the value of DDD2. When the USART forces this pin to
    be an input, the pull-up can still be controlled by the PORTD2 bit.
  Port D, Bit 3: TXD1, Transmit Data (Data output pin for the USART1).
    When the USART1 Transmitter is enabled, this pin is configured as
    an output regardless of the value of DDD3.

  AT90USB90128/64 TQFP64
  -- ------------------------ ---------------------------------------------
  PIN SIGNAL                  BOARD CONNECTION
  -- ------------------------ ---------------------------------------------
  27 (RXD1/INT2) PD2           J3-38 D2
  28 (TXD1/INT3) PD3           J3-39 D3

Toolchains
^^^^^^^^^^

There are several toolchain options.  However, testing has been performed
using *only* the NuttX buildroot toolchain described below.  Therefore,
the NuttX buildroot toolchain is the recommended choice.

The toolchain may be selected using the kconfig-mconf tool (via 'make menuconfig'),
by editing the existing configuration file (defconfig), or by overriding
the toolchain on the make commandline with CONFIG_AVR_TOOLCHAIN=<toolchain>.

The valid values for <toolchain> are BUILDROOT, CROSSPACK, LINUXGCC and WINAVR.

Buildroot:

  There is a DIY buildroot version for the AVR boards here:
  http://sourceforge.net/projects/nuttx/files/buildroot/.  See the
  following section for details on building this toolchain.

  It is assumed in some places that buildroot toolchain is available
  at ../misc/buildroot/build_avr.  Edit the setenv.sh file if
  this is not the case.

  After configuring NuttX, make sure that CONFIG_AVR_BUILDROOT=y is set in your
  .config file.

WinAVR:

  For Cygwin development environment on Windows machines, you can use
  WinAVR: http://sourceforge.net/projects/winavr/files/

  It is assumed in some places that WinAVR is installed at C:/WinAVR.  Edit the
  setenv.sh file if this is not the case.

  After configuring NuttX, make sure that CONFIG_AVR_WINAVR=y is set in your
  .config file.

  WARNING:  There is an incompatible version of cygwin.dll in the WinAVR/bin
  directory!  Make sure that the path to the correct cygwin.dll file precedes
  the path to the WinAVR binaries!

Linux:

  For Linux, there are widely available avr-gcc packages.  On Ubuntu, use:
  sudo apt-get install gcc-avr gdb-avr avr-libc

  After configuring NuttX, make sure that CONFIG_AVR_LINUXGCC=y is set in your
  .config file.

Mac OS X:

  For Mac OS X, the CrossPack for AVR toolchain is available from:

    http://www.obdev.at/products/crosspack/index.html

  This toolchain is functionally equivalent to the Linux GCC toolchain.

Windows Native Toolchains
^^^^^^^^^^^^^^^^^^^^^^^^^

  The WinAVR toolchain is a Windows native toolchain. There are several
  limitations to using a Windows native toolchain in a Cygwin environment. 
  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath'
     utility but you might easily find some new path problems.  If so, check
     out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic
     links are used in Nuttx (e.g., include/arch).  The make system works
     around these  problems for the Windows tools by copying directories
     instead of linking them.  But this can also cause some confusion for
     you:  For example, you may edit a file in a "linked" directory and find
     that your changes had no effect. That is because you are building the
     copy of the file in the "fake" symbolic directory.  If you use a
     Windows toolchain, you should get in the habit of making like this:

       make clean_context all

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This
     is because the dependencies are generated using Windows pathes which do
     not work with the Cygwin make.

       MKDEP = $(TOPDIR)/tools/mknulldeps.sh

  An additional issue with the WinAVR toolchain, in particular, is that it
  contains an incompatible version of the Cygwin DLL in its bin/ directory.
  You must take care that the correct Cygwin DLL is used.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  If NuttX buildroot toolchain source tarball cne can be downloaded from the
  NuttX SourceForge download site (https://sourceforge.net/projects/nuttx/files/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh micropendous3/<sub-dir>

     NOTE: you also must copy avr-libc header files into the NuttX include
     directory with command perhaps like:

     cp -a /cygdrive/c/WinAVR/include/avr include/.

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/avr-defconfig-4.5.2 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you
  are building a toolchain for Cygwin under Windows.

avr-libc
^^^^^^^^

Header Files

  In any case, header files from avr-libc are required:  http://www.nongnu.org/avr-libc/.
  A snapshot of avr-lib is included in the WinAVR installation. For Linux
  development platforms, avr-libc package is readily available (and would
  be installed in the apt-get command shown above).  But if you are using
  the NuttX buildroot configuration on Cygwin, then you will have to build
  get avr-libc from binaries.

Header File Installation

  The NuttX build will required that the AVR header files be available via
  the NuttX include directory.  This can be accomplished by either copying
  the avr-libc header files into the NuttX include directory:

  cp -a <avr-libc-path>/include/avr <nuttx-path>/include/.

  Or simply using a symbolic link:

  ln -s <avr-libc-path>/include/avr <nuttx-path>/include/.

Build Notes:

  It may not necessary to have a built version of avr-lib; only header files
  are required.  Bu if you choose to use the optimized libraru functions of
  the flowing point library, then you may have to build avr-lib from sources.
  Below are instructions for building avr-lib from fresh sources:

  1. Download the avr-libc package from: 

     http://savannah.nongnu.org/projects/avr-libc/

     I am using avr-lib-1.7.1.tar.bz2

  2. Upack the tarball and cd into the 
 
     tar jxf avr-lib-1.7.1.tar.bz2
     cd avr-lib-1.7.1

  3. Configure avr-lib.  Assuming that WinAVR is installed at the following
     location:

     export PATH=/cygdrive/c/WinAVR/bin:$PATH
     ./configure --build=`./config.guess` --host=avr

     This takes a *long* time.

  4. Make avr-lib.

     make

     This also takes a long time because it generates variants for nearly
     all AVR chips.

  5. Install avr-lib.

     make install

Micropendous3 Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=avr

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_AVR=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_AT90USB=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=at90usb

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip.  This should be exactly one of

       CONFIG_ARCH_CHIP_AT90USB646=y
       CONFIG_ARCH_CHIP_AT90USB647=y
       CONFIG_ARCH_CHIP_AT90USB1286=y
       CONFIG_ARCH_CHIP_AT90USB1287=y

       Depending on which Micropendous3 version you have.

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=micropendous3

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_MICROPENOUS3=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM.  One of:

       CONFIG_DRAM_SIZE=(4*1024) - (4Kb)
       CONFIG_DRAM_SIZE=(8*1024) - (8Kb)

    CONFIG_DRAM_START - The start address of installed SRAM

       CONFIG_DRAM_START=0x800100

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

      CONFIG_AVR_INT0=n
      CONFIG_AVR_INT1=n
      CONFIG_AVR_INT2=n
      CONFIG_AVR_INT3=n
      CONFIG_AVR_INT4=n
      CONFIG_AVR_INT5=n
      CONFIG_AVR_INT6=n
      CONFIG_AVR_INT7=n
      CONFIG_AVR_USBHOST=n
      CONFIG_AVR_USBDEV=n
      CONFIG_AVR_WDT=n
      CONFIG_AVR_TIMER0=n
      CONFIG_AVR_TIMER1=n
      CONFIG_AVR_TIMER2=n
      CONFIG_AVR_TIMER3=n
      CONFIG_AVR_SPI=n
      CONFIG_AVR_USART1=y
      CONFIG_AVR_ANACOMP=n
      CONFIG_AVR_ADC=n
      CONFIG_AVR_TWI=n
 
  If the watchdog is enabled, this specifies the initial timeout.  Default
  is maximum supported value.

      CONFIG_WDTO_15MS
      CONFIG_WDTO_30MS
      CONFIG_WDTO_60MS
      CONFIG_WDTO_120MS
      CONFIG_WDTO_1250MS
      CONFIG_WDTO_500MS
      CONFIG_WDTO_1S
      CONFIG_WDTO_2S
      CONFIG_WDTO_4S
      CONFIG_WDTO_8S

  AT90USB specific device driver settings

    CONFIG_USARTn_SERIAL_CONSOLE - selects the USARTn for the
       console and ttys0 (default is no serial console).
    CONFIG_USARTn_RXBUFSIZE - Characters are buffered as received.
       This specific the size of the receive buffer
    CONFIG_USARTn_TXBUFSIZE - Characters are buffered before
       being sent.  This specific the size of the transmit buffer
    CONFIG_USARTn_BAUD - The configure BAUD of the USART.  Must be
    CONFIG_USARTn_BITS - The number of bits.  Must be either 7 or 8.
    CONFIG_USARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity
    CONFIG_USARTn_2STOP - Two stop bits

Configurations
^^^^^^^^^^^^^^

Each Micropendous3 configuration is maintained in a sudirectory and can
be selected as follow:

    cd tools
    ./configure.sh micropendous3/<subdir>
    cd -
    . ./setenv.sh

NOTE: You must also copy avr-libc header files, perhaps like:

     cp -a /cygdrive/c/WinAVR/include/avr include/.

Where <subdir> is one of the following:

  hello:
    The simple apps/examples/hello "Hello, World!" example.

    FLASH/SRAM Requirements (as of 6/16/2011):

      $ avr-nuttx-elf-size nuttx
       text    data     bss     dec     hex filename
      24816     978     308   26102    65f6 nuttx

    Strings are in SRAM.
