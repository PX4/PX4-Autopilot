README
^^^^^^

This is the README file for the NuttX port to the Atmel AVR32DEV1 board.

Contents
^^^^^^^^

 * GPIO Pin Configuration
 * Serial Connection
 * Toolchains
 * Development Environment
 * GNU Toolchains
 * IDEs
   - Makefile Build
   - Native Build
 * AVR32 Bootloader
   - Boot Sequence
   - Link Address
   - Entering the ISP
   - BatchISP
 * Reset
 * Make Tip
 * AVR32DEV1 Configuration Options
 * Configurations

GPIO Pin Configuration
^^^^^^^^^^^^^^^^^^^^^^

The only GPIO pin usage is for LEDs (2) and Buttons (2):

  PIN 13  PA7  LED1
  PIN 14  PA8  LED2
  PIN 24  PB2  KEY1
  PIN 25  PB3  KEY2

(See configs/avr32dev/src/avr32dev_internal.h).  And also for
crystals (4), JTAG (1), and USB (1):

  PIN 30  PA11 XIN32
  PIN 31  PA12 XOUT32
  PIN 35  PA15 EVTO (JTAG)
  PIN 39  PA18 X1IN
  PIN 40  PA19 X1OUT
  PIN 61  PA26 ID (USB)

All GPIO pins are brought out through connectors J1 (PINS 33-64)
and J2 (PINS 1-32).

NOTE:  There seems to be some difference in labeling for OSC0 and
OSC1 between MCUZone.com and Atmel:

  Oscillator pinout
  -------------------------- --------------------
  QFP48 QFP64 Pad Oscillator AVR32DEV1
   PIN   PIN       PIN       LABEL
  ----- ----- ---- --------- --------------------
   30    39   PA18 XIN0      X1IN   (12MHz)
         41   PA28 XIN1      PA28   (no crystal)
   22    30   PA11 XIN32     XIN32  (32KHz)
   31    40   PA19 XOUT0     X1OUT  (12Mhz)
         42   PA29 XOUT1     PA29   (no crystal)
   23    31   PA12 XOUT32    XOUT32 (32 Khz)
  ----- ----- ---- --------- --------------------

NOTE 1: These crystal inputs/outputs are analog signals and my
assumption is that they need no pin multiplexing setting to
enable them for the external crystal function.

NOTE 2: There is no support for OSC1.

NOTE 3: There are solder pads for the 32KHz OSC32, but the
crystal is not populated on my board.  Therefore, the RTC will
have to run from the (uncalibrated) RCOSC.

Serial Connection
^^^^^^^^^^^^^^^^^

USART1 is the default USART1 used in the configuration files to
provide a serial console (of course, that can be easily changed
by editting the configuration file).  The AVR32DEV1 board has no
RS-232 drivers or connectors on board.  I use an off-board MAX232
module that I got on eBay (search for MAX232 if you want to find
one).  I connect the MAX232 board as follows:

In configs/avr32dev/include/board.h:

  #define PINMUX_USART1_RXD   PINMUX_USART1_RXD_1
  #define PINMUX_USART1_TXD   PINMUX_USART1_TXD_1

In arch/avr/src/at32uc3/at32uc3b_pinmux.h:

  #define PINMUX_USART1_RXD_1 (GPIO_PERIPH | GPIO_FUNCD | GPIO_PORTA | 17)
  #define PINMUX_USART1_TXD_1 (GPIO_PERIPH | GPIO_FUNCA | GPIO_PORTA | 23)

PA17 and PA23 are avaiable from the AVR32DEV1:

  FUNC GPIO  PIN   Header 16X2 (J1) MX232 Board
  ---- ----- ----- ---------------- ------------
  RXD  PA17  PIN37 Pin 5            PIN4 RXD (5V TTL/CMOS)
  TXD  PA23  PIN47 Pin 15           PIN3 TXD (5V TTL/CMOS)
                                    PIN2 GND
                                    PIN1 VCC (5V)

  Voltage on GPIO Pins with respect to Ground for TCK, RESET_N, PA03-PA08,
  PA11-PA12, PA18-PA19, PA28-PA31............................-0.3 to 3.6V
  Other Pins ............................................... -0.3 to 5.5V

  I get the 5V from another USB port (using the 5V power cable that normally
  provides the extra current needed by my USB IDE drive).

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Linux, Mac OS X or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchains
^^^^^^^^^^^^^^

Atmel Toolchain:

  The build logic in these directories assume that you are using the GNU
  toolchain with the Atmel patches.  The patch file, pre-patched tool
  sources,and pre-built binaries are available from the Atmel website.

    CONFIG_AVR32_AVRTOOLSW=y  # Use the windows version
    CONFIG_AVR32_AVRTOOLSL=y  # Ue the Linux version

  NOTE: The NuttX builtroot cannot be used to build the AVR32 toolchain.
  This is because the Atmel patches that add support for the AVR32 are not
  included in the NuttX buildroot.

WinAVR:

  Another option for use under Windows is WinAVR:
  http://sourceforge.net/projects/winavr/files/.  WinAVR includes the
  AVR32 toolchain as well as the AVR toolchain and various support
  libraries and header files.

AVR32 Toolchain Builder:

  A third option is to build the toolchain yourself. For OS X and Linux systems,
  this Makefile will build a complete gcc-4.4.3 toolchain:

    https://github.com/jsnyder/avr32-toolchain

  By default the toolchain installs into ${HOME}/avr-32-tools-<somedate> and
  the bin subdirectory must be added to your path before compiling.

IDEs
^^^^

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
  3) Set up include pathes:  You will need include/, arch/avr/src/at32uc3,
     arch/avr/src/common, arch/arm/src/avr, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/avr/src/avr3/up_nommuhead.S.

AVR32 Bootloader
^^^^^^^^^^^^^^^^

  Boot Sequence
  -------------
  
    "An AVR UC3 part having the bootloader programmed resets as any other
     part at 80000000h. Bootloader execution begins here. The bootloader
     first performs the boot process to know whether it should start the
     USB DFU ISP or the application. If the tested conditions indicate
     that the USB DFU ISP should be started, then execution continues in
     the bootloader area, i.e. between 80000000h and 80002000h, else
     the bootloader launches the application at 80002000h."
  
  Link Address
  ------------

  The linker scripts (ld.script) assume that you are using the DFU
  bootloader.  The bootloader resides at 0x8000:0000 and so the ld.script
  files link the application to execute after the bootloader at
  0x8000:2000. To link so that NuttX boots directly without using the
  bootloader, change the flash definition from:

    flash (rxai!w)  : ORIGIN = 0x80002000, LENGTH = 256K - 8K

  to:
    flash (rxai!w)  : ORIGIN = 0x80000000, LENGTH = 256K

  Or to use the MSC bootloader:

    flash (rxai!w)  : ORIGIN = 0x80008000, LENGTH = 256K - 32K

  Entering the ISP
  ----------------

  In order to use the USB port to download the FLASH(ISP), you need to
  use the S3(PA13) to make CPU return to boot status. In this mode, the
  on chip bootloader will run, making the ISP possible.

  BatchISP
  --------

  Unlike other Atmel parts, the AVR32 will not work with the FLIP GUI
  program.  Instead, you must use the command-line loader call BatchISP.
  If need to download FLIP from the atmel.com website, install the USB
  driver in the FLIP usb directory.  Then in the bin directory where
  you installed FLIP, you will also find batchisp.exe.

  NOTE: The AVR32DEV1 setenv.sh files will add the path to the BatchISP
  bin directory to the Cygwin PATH variable.  If you use a different
  version of FLIP or if you install FLIP in a different location, you
  will need to modify the setenv.sh files.

  Notes from "AVR32 UC3 USB DFU Bootloader" (doc7745.pdf)
  
  "To launch BatchISP, open a command prompt. Windows or Cygwin command
   prompt can be used provided that the bin folder of the FLIP installation
   directory is in the PATH (Windows’ or Cygwin’s) environment variable.
   When running BatchISP on AT32UC3xxxxx, the target part has to be specified
   with -device at32uc3xxxxx and the communication port with -hardware usb.
   Commands can then be placed after -operation. These commands are executed
   in order. BatchISP options can be placed in a text file invoked using
   -cmdfile rather than on the command line.

  "BatchISP works with an internal ISP buffer per target memory. These ISP
   buffers can be filled from several sources. All target operations (program,
   verify, read) are performed using these buffers."
 
  The following BatchISP command line will erase FLASH, write the nuttx binary
  into FLASH, and reset the AVR32.  This command line is available in the
  script config/avr32dev1/tools/doisp.sh:

     batchisp -device at32uc3b0256 -hardware usb -operation erase f memory flash \
     blankcheck loadbuffer nuttx.elf program verify start reset 0

  "BatchISP main commands available on AT32UC3xxxxx are:
  
   - ASSERT { PASS | FAIL } changes the displayed results of the following
     operations according to the expected behavior.
   - ONFAIL { ASK | ABORT | RETRY | IGNORE } changes the interactive behavior
     of BatchISP in case of failure.
   - WAIT <Nsec> inserts a pause between two ISP operations.
   - ECHO <comment> displays a message.
   - ERASE F erases internal flash contents, except the bootloader.
   - MEMORY { FLASH | SECURITY | CONFIGURATION | BOOTLOADER | SIGNATURE | USER }
     selects a target memory on which to apply the following operations.
   - ADDRANGE <addrMin> <addrMax> selects in the current target memory an
     address range on which to apply the following operations.
   - BLANKCHECK checks that the selected address range is erased.
   - FILLBUFFER <data> fills the ISP buffer with a byte value.
   - LOADBUFFER { <in_elffile> | <in_hexfile> } loads the ISP buffer from an
     input file.
   - PROGRAM programs the selected address range with the ISP buffer.
   - VERIFY verifies that the selected address range has the same contents
     as the ISP buffer.
   - READ reads the selected address range to the ISP buffer.
   - SAVEBUFFER <out_hexfile> { HEX386 | HEX86 } saves the ISP buffer to an
      output file.
   - START { RESET | NORESET } 0 starts the execution of the programmed
     application with an optional hardware reset of the target.

  "The AT32UC3xxxxx memories made available by BatchISP are:

  - FLASH: This memory is the internal flash array of the target, including the
    bootloader protected area. E.g. on AT32UC3A0512 (512-kB internal flash),
    addresses from 0 to 0x7FFFF can be accessed in this memory.
  - SECURITY: This memory contains only one byte. The least significant bit
    of this byte reflects the value of the target Security bit which can only
    be set to 1. Once set, the only accepted commands will be ERASE and START.
    After an ERASE command, all commands are accepted until the end of the
    non-volatile ISP session, even if the Security bit is set.
  - CONFIGURATION: This memory contains one byte per target general-purpose
    fuse bit.  The least significant bit of each byte reflects the value of
    the corresponding GP fuse bit.
  - BOOTLOADER: This memory contains three bytes concerning the ISP: the ISP
    version in BCD format without the major version number (always 1), the
    ISP ID0 and the ISP ID1.
  - SIGNATURE: This memory contains four bytes concerning the part: the product
    manufacturer ID, the product family ID, the product ID and the product
    revision.
  - USER: This memory is the internal flash User page of the target, with
    addresses from 0 to 0x1FF.

  "For further details about BatchISP commands, launch batchisp -h or see the
   help files installed with FLIP ..."

Reset
^^^^^

   I don't trust the reset button -- if you reset and something weird happens,
   try a full power cycle.

Make Tip
^^^^^^^^

   Because this build uses a native Windows toolchain and the native Windows
   tools do not understand Cygwin's symbolic links, the NuttX make system does
   something weird:  It copies the configuration directories instead of linking
   to them (it could, perhaps, use the NTFS 'mklink' command, but it doesn't).

   A consequence of this is that you can easily get confused when you edit
   a file in one of the "linked" directories, re-build NuttX, and then not see your
   changes when you run the program.  That is because build is still using the
   version of the file in the copied directory, not your modified file! To work
   around this annoying behavior, do the following when you re-build:
   
   make clean_context all <-- Remove and re-copy all of the directories, then make all
   doisp.sh               <-- Load the code onto the board.

AVR32DEV1 Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=avr

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_AVR=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_AVR32=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=at32uc3

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_AT32UC3B0256

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=avr32dev1 (for the AV32DEV1 board)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_AVR32DEV1

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (SRAM in this case):

       CONFIG_DRAM_SIZE=0x00010000 (64Kb)

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x20000000

    CONFIG_ARCH_IRQPRIO - The AT32UC3B0256 supports interrupt prioritization

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
  
    CONFIG_AVR32_GPIOIRQ - GPIO interrupt support
    CONFIG_AVR32_GPIOIRQSETA - Set of GPIOs on PORTA that support interrupts
    CONFIG_AVR32_GPIOIRQSETB - Set of GPIOs on PORTB that support interrupts

    CONFIG_AVR32_USARTn - Enable support for USARTn
    CONFIG_AVR32_USARTn_RS232 - Configure USARTn as an RS232 interface.
    CONFIG_AVR32_USARTn_SPI - Configure USARTn as an SPI interface.
    CONFIG_AVR32_USARTn_RS485 - Configure USARTn as an RS485 interface.
    CONFIG_AVR32_USARTn_MAN - Configure USARTn as an Manchester interface.
    CONFIG_AVR32_USARTn_MODEM - Configure USARTn as an Modem interface.
    CONFIG_AVR32_USARTn_IRDA - Configure USARTn as an IRDA interface.
    CONFIG_AVR32_USARTn_ISO786 - Configure USARTn as an ISO786 interface.

  AT32UC3B0256 specific device driver settings

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

Each Atmel AVR32DEV configuration is maintained in a sudirectory and
can be selected as follow:

    cd tools
    ./configure.sh avr32dev1/<subdir>
    cd -
    . ./setenv.sh

(Or configure.bat in a native Windows environment).

Where <subdir> is one of the following:

  nsh:
    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interface.

  ostest:
    This configuration directory, performs a simple OS test using
    examples/ostest.

    NOTE: Round-robin scheduling is disabled in this test because
    the RR test in examples/ostest declares data structures that
    are too large for the poor little uc3 SRAM.


