zp214xpa README FILE
====================

The ZP213X/4XPA board from the0.net with LPC2148 installed.
Includes support for the UG-2864AMBAG01 OLED from The0.net.

Contents
========

  o MCU Connections
  o Serial Console
  o Using OpenOCD and GDB with an FT2232 JTAG emulator
  o Configurations

MCU Connections:
================

The ZP213X/4XPA board is no more than an LPC2148, crystals,
USB device and several connectors.

  Module Socket:
  --------------
  PIN NAME  PIN NAME
   1  VBAT  56  VCC
   2  3V3   55  Vusb
   3  VREF  54  3V3
   4  P0.0  53  RESET
   5  P0.1  52  P1.31
   6  P0.2  51  P1.30
   7  P0.3  50  P1.29
   8  P0.4  49  P1.28
   9  P0.5  48  P1.27
  10  P0.6  47  P1.26
  11  P0.7  46  P1.25
  12  P0.8  45  P1.24
  13  P0.9  44  P1.23
  14  P0.10 43  P1.22
  15  P0.11 42  P1.21
  16  P0.12 41  P1.20
  17  P0.13 40  P1.19
  18  P0.14 39  P1.18
  19  P0.15 38  P1.17
  20  P0.16 37  P1.16
  21  P0.17 36  P0.31
  22  P0.18 35  P0.30
  23  P0.19 34  P0.29
  24  P0.20 33  P0.28
  25  P0.21 32  P0.27
  26  P0.22 31  P0.26
  27  P0.23 30  P0.25
  28  GND   29  GND

  JTAG Debug:
  -----------
  PIN NAME       PIN NAME
   1  VCC1        2  3V3
   3  P1.31 NTRST 4  GND
   5  P1.28 TDI   6  GND
   7  P1.30 TMS   8  GND
   9  P1.29 TCK  10  GND
  11  P1.26 RTCK 12  GND
  13  P1.27 TDO  14  GND
  15  RESET NRTS 16  GND
  17  N/C   NC0  18  GND
  19  N/C   NC1  20  GND

  Z28160 Net Module:
  ------------------
  PIN NAME      PIN NAME
   1  P0.7  /CS 10  3V3   VCC
   2  P0.4  SCK  9  P1.24 RST
   3  P0.6  SI   8  N/C   CLKOUT
   4  P0.5  SO   7  INT   P1.25
   5  GND        6  N/C   WOL

  SPI LCD:
  --------
  PIN NAME
   1  3V3   3V3
   2  VCC   5V
   3  P0.18 RESET(DO)
   4  P0.19 DI
   5  P0.20 CS
   6  P0.17 SCK
   7  P0.23 A0(RESET)
   8  N/C   LED-
   9  N/C   LED+(BL)
  10  GND   GND

  USB Interface:
  --------------
  Vusb, P0.26, P0.27

Serial Console:
===============

  Both UART0 and UART1 are always enabled.  UART0 is configured to be the
  serial console in these configurations.

  P0.0/TXD0/PWM1        Module Socket, Pin 4
  P0.1/RxD0/PWM3/EINT0  Module Socket, Pin 5

  P0.8/TXD1/PWM4/AD1.1  Module Socket, Pin 12
  P0.9/RxD1/PWM6/EINT3  Module Socket, Pin 13

LCD Interface
=============

  PIN NAME  PIN CONFIGURATION
   3  RESET P0.18/CAP1.3/MISO1/MAT1.3P0.18 RESET - General purpose output
   4  DI    P0.19/MAT1.2/MOSI1/CAP1.2P0.19 DI    - Alternate function 2
   5  CS    P0.20/MAT1.3/SSEL1/EINT3             - General purpose output
   6  SCK   P0.17/CAP1.2/SCK1/MAT1.2             - Alternate function 2
   7  A0    P0.23/VBUS                           - General purpose output

ENC29J60 Interface
==================

  PIN NAME  PIN CONFIGURATION
   1  /CS   P0.7/SSEL0/PWM2/EINT2                - General purpose output
   2  SCK   P0.4/SCK0/CAP0.1/AD0.6               - Alternate function 1
   3  SI    P0.6/MOSI0/CAP0.2/AD1.0              - Alternate function 1
   4  SO    P0.5/MISO0/MAT0.1/AD0.7              - Alternate function 1
   7  INT   P1.25/EXTIN0                         - Alternal function 1
   9  RST   P1.24/TRACECLK                       - General purpose output

Using OpenOCD and GDB with an FT2232 JTAG emulator
==================================================

  Downloading OpenOCD
  
    You can get information about OpenOCD here: http://openocd.berlios.de/web/
    and you can download it from here. http://sourceforge.net/projects/openocd/files/.
    To get the latest OpenOCD with more mature lpc214x, you have to download
    from the GIT archive.
    
      git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd

    At present, there is only the older, frozen 0.4.0 version.  These, of course,
    may have changed since I wrote this.
 
  Building OpenOCD under Cygwin:

    You can build OpenOCD for Windows using the Cygwin tools.  Below are a
    few notes that worked as of November 7, 2010.  Things may have changed
    by the time you read this, but perhaps the following will be helpful to
    you:
    
    1. Install Cygwin (http://www.cygwin.com/).  My recommendation is to install
       everything.  There are many tools you will need and it is best just to
       waste a little disk space and have everthing you need.  Everything will
       require a couple of gigbytes of disk space.

    2. Create a directory /home/OpenOCD.

    3. Get the FT2232 drivr from http://www.ftdichip.com/Drivers/D2XX.htm and
       extract it into /home/OpenOCD/ftd2xx

       $ pwd
       /home/OpenOCD
       $ ls
       CDM20802 WHQL Certified.zip
       $ mkdir ftd2xx
       $ cd ftd2xx
       $ unzip ..CDM20802\ WHQL\ Certified.zip 
       Archive:  CDM20802 WHQL Certified.zip
       ...

    3. Get the latest OpenOCD source
    
       $ pwd
       /home/OpenOCD
       $ git clone git://openocd.git.sourceforge.net/gitroot/openocd/openocd
 
       You will then have the source code in /home/OpenOCD/openocd

    4. Build OpenOCD for the FT22322 interface

       $ pwd
       /home/OpenOCD/openocd
       $ ./bootstrap 

       Jim is a tiny version of the Tcl scripting language.  It is needed
       by more recent versions of OpenOCD.  Build libjim.a using the following
       instructions:

       $ git submodule init
       $ git submodule update
       $ cd jimtcl
       $ ./configure --with-jim-ext=nvp
       $ make
       $ make install

       Configure OpenOCD:

       $ ./configure --enable-maintainer-mode --disable-werror --disable-shared \
                    --enable-ft2232_ftd2xx --with-ftd2xx-win32-zipdir=/home/OpenOCD/ftd2xx \
                    LDFLAGS="-L/home/OpenOCD/openocd/jimtcl"

        Then build OpenOCD and its HTML documentation:

        $ make
        $ make html

        The result of the first make will be the "openocd.exe" will be
        created in the folder /home/openocd/src.  The following command
        will install OpenOCD to a standard location (/usr/local/bin)
        using using this command:

        $ make install

  Helper Scripts.

    I have been using the Olimex ARM-USB-OCD JTAG debugger with the
    ZP213X/4XPA.  OpenOCD requires a configuration file.  I keep the
    one I used last here:
    
      configs/zpa214xpa/tools/olimex.cfg

    However, the "correct" configuration script to use with OpenOCD may
    change as the features of OpenOCD evolve.  So you should at least
    compare that olimex.cfg file with configuration files in
    /usr/local/share/openocd/scripts/target (or /home/OpenOCD/openocd/tcl/target).
    
    There is also a script on the tools/ directory that I use to start
    the OpenOCD daemon on my system called oocd.sh.  That script will
    probably require some modifications to work in another environment:
  
    - Possibly the value of OPENOCD_PATH and TARGET_PATH
    - It assumes that the correct script to use is the one at
      configs/zp214xpa/tools/olimex.cfg

  Starting OpenOCD

    Then you should be able to start the OpenOCD daemon like:

      configs/zp214xpa/tools/oocd.sh $PWD

    If you use the setenv.sh file, that the path to oocd.sh will be added
    to your PATH environment variabl.  So, in that case, the command simplifies
    to just:

      oocd.sh $PWD

    Where it is assumed that you are executing oocd.sh from the top-level
    directory where NuttX is installed.  $PWD will be the path to the
    top-level NuttX directory.

  Connecting GDB

    Once the OpenOCD daemon has been started, you can connect to it via
    GDB using the following GDB command:

      arm-nuttx-elf-gdb
      (gdb) target remote localhost:3333

    NOTE:  The name of your GDB program may differ.  For example, with the
    CodeSourcery toolchain, the ARM GDB would be called arm-none-eabi-gdb.

    After starting GDB, you can load the NuttX ELF file:

      (gdb) symbol-file nuttx
      (gdb) load nuttx

    NOTES:
    1. Loading the symbol-file is only useful if you have built NuttX to
       include debug symbols (by setting CONFIG_DEBUG_SYMBOLS=y in the
       .config file).
 
    OpenOCD will support several special 'monitor' commands.  These
    GDB commands will send comments to the OpenOCD monitor.  Here
    are a couple that you will need to use:
  
     (gdb) monitor reset
     (gdb) monitor halt

    NOTES:
    1. The MCU must be halted using 'mon halt' prior to loading code.
    2. Reset will restart the processor after loading code.
    3. The 'monitor' command can be abbreviated as just 'mon'.

Configurations:
===============

  Each NXP LPC214x configuration is maintained in a sudirectory and
  can be selected as follow:

    cd tools
    ./configure.sh zp214xpa/<subdir>
    cd -
    . ./setenv.sh

  Where <subdir> is one of the following:

  nsh:
  ----

    Configures the NuttShell (nsh) located at examples/nsh.  The
    Configuration enables only the serial NSH interfaces.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y             : Linux (Cygwin under Windows okay too).
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y             : Output formats: ELF and raw binary

  nxlines:
  --------

    This is the apps/examples/nxlines test using the UG_2864AMBAG01 board
    from The0.net that plugs into the "SPI LCD" connector on the ZP3X4XPA
    board.

    NOTES:

    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain:

       CONFIG_HOST_LINUX=y             : Linux (Cygwin under Windows okay too).
       CONFIG_ARM_TOOLCHAIN_GNU_EABI=y : Buildroot (arm-nuttx-elf-gcc)
       CONFIG_RAW_BINARY=y             : Output formats: ELF and raw binary

    3. Verified as of this writing (2012-12-30).
