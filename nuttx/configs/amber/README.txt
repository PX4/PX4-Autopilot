README
^^^^^

This is the README file for the port of NuttX to the Amber Web Server from
SoC Robotics (http://www.soc-robotics.com/index.htm).  The
Amber Web Server is based on an Atmel ATMega128.  As of this writing,
documentation for the Amber Web Server board is available here:

http://www.soc-robotics.com/product/Amber_Specs/Amber_Processor.html

and

http://www.soc-robotics.com/pdfs/Amber%201-5a%20Hardware%20Reference%20Guide.pdf

Contents
^^^^^^^^

  o Amber Web Server Features
  o Pin Connections
  o Atmel AVRISP mkII Connection
  o Toolchains
  o Windows Native Toolchains
  o NuttX buildroot Toolchain
  o avr-libc
  o Amber Web Server Configuration Options
  o Configurations

Amber Web Server Features
^^^^^^^^^^^^^^^^^^^^^^^^^

   o  17.56MHz ATmega128 Atmel 8bit AVR RISC Processor
   o  128Kbyte Flash
   o  64Kbyte RAM
   o  10BaseT Ethernet Port
   o  High Speed Serial Port
   o  8Ch 10bit Analog Input port
   o  16 Digital IO ports
   o  Expansion bus for daughter cards
   o  LED status indicators
   o  ISP Programming port
   o  7-14VDC input
   o  Power via Ethernet port

Pin Connections (PCB Rev 1.5a)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  -------------------- -----------------------------
  ATMega128 Pinout     Amber board connection
  -------------------- -----------------------------
  (left)
   1 PEN               Pulled-up
   2 PE0 (RXD0/PDI)    MAX202ECWED T1IN or J7-1, ISP-PDI (via 74HC5053), J5-26
   3 PE1 (TXD0/PDO)    MAX202ECWED A1OUT or J7-9, ISP-PDO (via 74HC5053), J5-25
   4 PE2 (XCK0/AIN0)   MAX202ECWED T2IN, J5-24
   5 PE3 (OC3A/AIN1)   MAX202ECWED A2OUT, J5-23
   6 PE4 (OC3B/INT4)   J5-22
   7 PE5 (OC3C/INT5)   J5-21, RTL8019AS INT 0, TP5 PE5
   8 PE6 (T3/INT6)     J5-20
   9 PE7 (ICP3/INT7)   J5-19
  10 PB0 (SS)          Pull up of SS SPI master
  11 PB1 (SCK)         J7-7, ISP_SCK (via 74HC4053) and AT45D011 SCK, J5-17
  12 PB2 (MOSI)        AT45D011 SI. J5-16
  13 PB3 (MISO)        AT45D011 SO, J5-15
  14 PB4 (OC0)         AT45D011 CS\, J5-14
  15 PB5 (OC1A)        J5-13
  16 PB6 (OC1B)        J5-12
  (bottom)
  17 PB7 (OC2/OC1C)    J5-11
  18 PG3/TOSC2         32.768KHz XTAL
  19 PG4/TOSC1         32.768KHz XTAL
  20 RESET             RESET
  21 VCC
  22 GND               GND
  23 XTAL2             14.7456MHz XTAL
  24 XTAL1             14.7456MHz XTAL
  25 PD0 (SCL/INT0)    J5-10
  26 PD1 (SDA/INT1)    J5-9
  27 PD2 (RXD1/INT2)   J5-8, MAX488CSA RO (RS-485)
  28 PD3 (TXD1/INT3)   J5-7, MAX488CSA DI (RS-485)
  29 PD4 (ICP1)        J5-6
  30 PD5 (XCK1)        J5-5
  31 PD6 (T1)          J5-4
  32 PD7 (T2)          J5-3
  (left)
  48 PA3 (AD3)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  47 PA4 (AD4)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  46 PA5 (AD5)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  45 PA6 (AD6)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  44 PA7 (AD7)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  43 PG2 (ALE)         J5-1, 74HC5730, 62246DLP-7, RTL8019AS
  42 PC7 (A15)         TP4 A15, J5-27, 74HC5730
  41 PC6 (A14)         J5-28, 74HC5730, 62246DLP-7, RTL8019AS
  40 PC5 (A13)         J5-29, 74HC5730, 62246DLP-7, RTL8019AS
  39 PC4 (A12)         J5-30, 74HC5730, 62246DLP-7, RTL8019AS
  38 PC3 (A11)         J5-31, 74HC5730, 62246DLP-7, RTL8019AS
  37 PC2 (A10)         J5-32, 74HC5730, 62246DLP-7, RTL8019AS
  36 PC1 (A9)          J5-33, 74HC5730, 62246DLP-7, RTL8019AS
  35 PC0 (A8)          J5-34, 74HC5730, 62246DLP-7, RTL8019AS
  34 PG1 (RD)          TP2 RD\, J5-52, 62246DLP-7, RTL8019AS
  33 PG0 (WR)          TP3 WR\, J5-51, 62246DLP-7, RTL8019AS
  (top)
  64 AVCC
  63 GND               GND
  62 AREF              (analog supply)
  61 PF0 (ADC0)        J6-5, PDV-P9 Light Sensor
  60 PF1 (ADC1)        J6-7, Thermister
  59 PF2 (ADC2)        J6-9, MXA2500GL Dual Axis Accesserometer, AOUTX
  58 PF3 (ADC3)        J6-11, MXA2500GL Dual Axis Accesserometer, AOUTY
  57 PF4 (ADC4/TCK)    J6-13, MXA2500GL Dual Axis Accesserometer, TOUT
  56 PF5 (ADC5/TMS)    J6-15
  55 PF6 (ADC6/TDO)    J6-17
  54 PF7 (ADC7/TDI)    J6-19
  53 GND               GND
  52 VCC
  51 PA0 (AD0)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  50 PA1 (AD1)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS
  49 PA2 (AD2)         J5-?, 74HC5730, 62246DLP-7, RTL8019AS

Switches and Jumpers
^^^^^^^^^^^^^^^^^^^^
ISP/UART0
  JP1 - DTE/DCE selection
  JP2 - 
  JP5 - 
  J11 - STK500 Enable

ADC
  JP8 - 
  JP9 - 

Networking
  JP10 - 

RS-485
  J8 - 
  J9 - 
  J10 - 

Atmel AVRISP mkII Connection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  ISP6PIN Header
  --------------

         1  2
   MISO  o  o VCC
    SCK  o  o MOSI
  RESET\ o  o GND

  (ISP10PIN Connector)
  ------------------- -------------------------
  
         1  2
    MOSI o  o Vcc   - ISP-PDI: PE0/PDI/RX0 via 74HC5053
     LED o  o GND   - ISP-PROG: J11/GND, to 74HC5053 and LED
  RESET\ o  o GND   - to 74HC505
    SCK  o  o GND   - ISP_SCK: SCK, PB0/SS\
    MISO o  o GND   - ISP-PDO: PE1/PD0/TX0 via 74HC5053

  Board Orientation

    |
    | +-----+
    | + O O |
    | + O O |
    | + O O
    | + O O |
    | + O x | PIN 1
    | +-----+
    |

  AVRISP mkII Connection to 10-pin Header
  -------------------------------------------
  10PIN Header         6PIN Header
  --------------------- ---------------------
  Pin  1 MOSI           Pin 4 MOSI
  Pin  2 Vcc            Pin 2 Vcc
  Pin  3 LED                  Controlled via J11
  Pin  4 GND            Pin 6 GND
  Pin  5 RESET\         Pin 5 RESET\
  Pin  6 GND                  N/C
  Pin  7 SCK            Pin 3 SCK
  Pin  8 GND                  N/C
  Pin  9 MISO           Pin 1 MISO
  Pin 10 GND                  N/C

Toolchains
^^^^^^^^^^

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
     ./configure.sh amber/<sub-dir>

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
     loction:

     export PATH=/cygdrive/c/WinAVR/bin:$PATH
     ./configure --build=`./config.guess` --host=avr

     This takes a *long* time.

  4. Make avr-lib.

     make

     This also takes a long time because it generates variants for nearly
     all AVR chips.

  5. Install avr-lib.

     make install

Amber Web Server Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=avr

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_AVR=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_ATMEGA=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=atmega

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_ATMEGA128=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=amber

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_AMBER=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM.  One of:

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
       CONFIG_AVR_WDT=n
       CONFIG_AVR_TIMER0=n
       CONFIG_AVR_TIMER1=n
       CONFIG_AVR_TIMER2=n
       CONFIG_AVR_TIMER3=n
       CONFIG_AVR_SPI=n
       CONFIG_AVR_USART0=y
       CONFIG_AVR_USART1=n
       CONFIG_AVR_ADC=n
       CONFIG_AVR_ANACOMP=n
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

 ATMEGA specific device driver settings

    CONFIG_USARTn_SERIAL_CONSOLE - selects the USARTn for the
       console and ttys0 (default is the USART0).
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

Each Amber Web Server configuration is maintained in a sudirectory and can
be selected as follow:

    cd tools
    ./configure.sh amber/<subdir>
    cd -
    . ./setenv.sh

NOTE: You must also copy avr-libc header files, perhaps like:

     cp -a /cygdrive/c/WinAVR/include/avr include/.

Where <subdir> is one of the following:

  hello:
    The simple apps/examples/hello "Hello, World!" example.
