README
^^^^^

This is the README file for the port of NuttX to the PJRC Teensy++ 2.0 board.
This board is developed by http://pjrc.com/teensy/.  The Teensy++ 2.0 is based
on an Atmel AT90USB1286 MCU.

Contents
^^^^^^^^

  o Teensy++ 2.0 Features
  o Pin Usage
  o Halfkey Bootloader
  o Serial Console
  o SD Connection
  o Toolchains
  o Windows Native Toolchains
  o NuttX buildroot Toolchain
  o avr-libc
  o Teensy++ Configuration Options
  o Configurations

Teensy++ 2.0 Features
^^^^^^^^^^^^^^^^^^^^^

  o Based on the 64-pin USB AVR Microcontroller AT90USB1286.
  o USB Full Speed (12Mbit/s)
  o USB Device Mode
  o 120kbof available FLASH memory for programs.
  o 8 kbytes SRAM and 4 kbytes of EEPROM 
  o USB powered
  o 16MHz crystal
  o 48 General Purpose IO Pins

Pin Usage
^^^^^^^^^

  AT90USB1286 TQFP64
  -- ------------------------ ---------------------------------------------
  PIN SIGNAL                  BOARD CONNECTION
  -- ------------------------ ---------------------------------------------
  (left)
  1  (INT.6/AIN.0) PE6         Pad E6
  2  (INT.7/AIN.1/UVcon) PE7   Pad E7
  3  UVcc                      (Voltage circutry)
  4  D-                        USB DP
  5  D+                        USB DM
  6  UGnd                      GND
  7  UCap                      GND (via cap)
  8  VBus                      USB VBUS
  9  (IUID) PE3                N/C
  10 (SS/PCINT0) PB0           Pad B0
  11 (PCINT1/SCLK) PB1         Pad B1
  12 (PDI/PCINT2/MOSI) PB2     Pad B2
  13 (PDO/PCINT3/MISO) PB3     Pad B3
  14 (PCINT4/OC.2A) PB4        Pad B4
  15 (PCINT5/OC.1A) PB5        Pad B5
  16 (PCINT6/OC.1B) PB6        Pad B6
  (bottom)
  17 (PCINT7/OC.0A/OC.1C) PB7  Pad B7
  18 (INT4/TOSC1) PE4          Pad E4
  19 (INT.5/TOSC2) PE5         Pad E5
  20 RESET                     Switch pulls to ground
  21 VCC                       VCC
  22 GND                       GND
  23 XTAL2                     XTAL (16MHz)
  24 XTAL1                     XTAL (16MHz)
  25 (OC0B/SCL/INT0) PD0       Pad D0
  26 (OC2B/SDA/INT1) PD1       Pad D1
  27 (RXD1/INT2) PD2           Pad D2
  28 (TXD1/INT3) PD3           Pad D3
  29 (ICP1) PD4                Pad D4
  30 (XCK1) PD5                Pad D5
  31 (T1) PD6                  Pad D6, LED
  32 (T0) PD7                  Pad D7
  (right)
  48 PA3 (AD3)                 Pad A3
  47 PA4 (AD4)                 Pad A4
  46 PA5 (AD5)                 Pad A5
  45 PA6 (AD6)                 Pad A6
  44 PA7 (AD7)                 Pad A7
  43 PE2 (ALE/HWB)             Pad ALE (Pulled down)
  42 PC7 (A15/IC.3/CLKO)       Pad C7
  41 PC6 (A14/OC.3A)           Pad C6
  40 PC5 (A13/OC.3B)           Pad C5
  39 PC4 (A12/OC.3C)           Pad C4
  38 PC3 (A11/T.3)             Pad C3
  37 PC2 (A10)                 Pad C2
  36 PC1 (A9)                  Pad C1
  35 PC0 (A8)                  Pad C0
  34 PE1 (RD)                  Pad E1
  33 PE0 (WR)                  Pad E0
  (top)
  64 AVCC                      VCC
  63 GND                       GND
  62 AREF                      Pad Ref (Capacitor to ground)
  61 PF0 (ADC0)                Pad F0
  60 PF1 (ADC1)                Pad F1
  59 PF2 (ADC2)                Pad F2
  58 PF3 (ADC3)                Pad F3
  57 PF4 (ADC4/TCK)            Pad F4
  56 PF5 (ADC5/TMS)            Pad F5
  55 PF6 (ADC6/TDO)            Pad F6
  54 PF7 (ADC7/TDI)            Pad F7
  53 GND                       GND
  52 VCC                       VCC
  51 PA0 (AD0)                 Pad A0
  50 PA1 (AD1)                 Pad A1
  49 PA2 (AD2)                 Pad A2

Halfkey Bootloader
^^^^^^^^^^^^^^^^^^

o Download the Teensy application from http://pjrc.com/teensy/loader.html
o Instructions are available for your OS at that places as well.

Summary:

1. Start Teensy
2. Press button on the Teensy board
3. Select a HEX file (File menu)
4. Select "program" (Operations menu)
5. Reboot (Operations menu).

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
  27 (RXD1/INT2) PD2           Pad D2
  28 (TXD1/INT3) PD3           Pad D3

  Plus power and ground.  There are numerous ground points and both USB 5V
  and Vcc are available.

SD Connection
^^^^^^^^^^^^^

I have the SD-ADP SD/MMC Card Adaptor from www.gravitech.com
(http://www.gravitech.us/sdcaad.html). Features: 

  o On-board 3.3V regulator 
  o Connect directly to 3.3V or 5.0V microcontroller 
  o Card detect LED
  o Includes 11-pin male header
  o Board dimension: 2.0”x1.3”

SD-ADP Pinout / SD Connection

 -- ---- ----------- -------------------------------------------------------
 J2 NAME SD CARD     DESCRIPTION
 -- ---- ----------- -------------------------------------------------------
  1 VIN   (reguator) Input power to the SD card (3.3V to 6.0V) 
  2 GND   3,6,12,13  Common (Connects to the housing of the SD socket)
  3 3V3   4 3.3V     Output voltage from the on-board 3.3V regulator (250mA) 
  4 NC    9 NC       Connect to pin 9 on the SD card (not used in SPI mode) 
  5 CS    1 DAT3/CS  Chip select *
  6 DI    2 CMD/DI   Serial input data *
  7 SCK   5 SCK      Serial clock *
  8 DO    7 DAT0/DO  Serial output data 
  9 IRQ   8 DAT1/IRQ Interrupt request, connect to pin 8 on the SD card (not used in SPI mode) 
 10 CD   10 CD       Card detect (active low) 
 11 WP   11 WP       Write protect
 -- ---- ----------- -------------------------------------------------------

  * Via a 74LCX245 level translator / buff

Teensy SPI Connection

  -- ---- -- ------------------------- -------
  J2 NAME PIN NAME                     PAD
  -- ---- -- ------------------------- -------
   1 VIN  -- Connected to USB +5V
   2 GND  -- Connected to USB GND
   3 3V3  -- Not used                  ---
   4 NC   -- Not used
   5 CS   10 (SS/PCINT0) PB0           Pad B0
   6 DI   12 (PDI/PCINT2/MOSI) PB2     Pad B2
   7 SCK  11 (PCINT1/SCLK) PB1         Pad B1
   8 DO   13 (PDO/PCINT3/MISO) PB3     Pad B3
   9 IRQ  -- Not used                  ---
  10 CD   14 (PCINT4/OC.2A) PB4        Pad B4
  11 WP   15 (PCINT5/OC.1A) PB5        Pad B5
  -- ---- -- ------------------------- -------

Toolchains
^^^^^^^^^^

There are several toolchain options.  However, testing has been performed
using *only* the NuttX buildroot toolchain described below.  Therefore,
the NuttX buildroot toolchain is the recommended choice.

The toolchain may be selected using the mconf tool (via 'make menuconfig'),
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
     ./configure.sh Teensy++/<sub-dir>

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

Teensy++ Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
       chip. 

       CONFIG_ARCH_CHIP_AT90USB1286=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=teensy

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_TEENSY=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM.  One of:

       CONFIG_DRAM_SIZE=(8*1024) - (8Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

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

  AT90USB specific USB device configuration

    CONFIG_USB_DISABLE_PADREGULATOR
    CONFIG_USB_LOWSPEED
    CONFIG_USB_NOISYVBUS

Configurations
^^^^^^^^^^^^^^

Each Teensy++ configuration is maintained in a sudirectory and can
be selected as follow:

    cd tools
    ./configure.sh teensy/<subdir>
    cd -
    . ./setenv.sh

NOTE: You must also copy avr-libc header files, perhaps like:

     cp -a /cygdrive/c/WinAVR/include/avr include/.

Where <subdir> is one of the following:

  hello:
    The simple apps/examples/hello "Hello, World!" example.

  ostest:
    This configuration directory, performs a simple OS test using
    apps/examples/ostest. NOTE:  The OS test is quite large.  In order
    to get it to fit within AVR memory constraints, it will probably be
    necessary to disable some OS features.

  usbstorage:
    This configuration directory exercises the USB mass storage
    class driver at apps/examples/usbstorage.  See apps/examples/README.txt
    for more information.  NOTE:  THIS CONFIGURATION HAS NOT YET BEEN
    DEBUGGED AND DOES NOT WORK!!!  ISSUES:  (1) THE SPI DRIVER IS UNTESTED,
    (2) THE USB DRIVER IS UNTESTED, AND (3) THE RAM USAGE MIGHT BE EXCESSIVE.

    Update 7/11:  (1) The SPI/SD driver has been verified, however, (2) I
    believe that the current teensy/usbstorage configuration uses too
    much SRAM for the system to behave sanely.  A lower memory footprint
    version of the mass storage driver will be required before this can
    be debugged
