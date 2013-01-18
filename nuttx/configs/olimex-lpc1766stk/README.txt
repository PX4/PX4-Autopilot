README
^^^^^^

README for NuttX port to the Olimex LPC1766-STK development board

Contents
^^^^^^^^

  Olimex LPC1766-STK development board
  Development Environment
  GNU Toolchain Options
  IDEs
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  LEDs
  Using OpenOCD and GDB with an FT2232 JTAG emulator
  Olimex LPC1766-STK Configuration Options
  USB Host Configuration
  Configurations

Olimex LPC1766-STK development board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  GPIO Usage:
  -----------

  GPIO                             PIN  SIGNAL NAME
  -------------------------------- ---- --------------
  P0[0]/RD1/TXD3/SDA1               46  RD1
  P0[1]/TD1/RXD3/SCL1               47  TD1
  P0[2]/TXD0/AD0[7]                 98  TXD0
  P0[3]/RXD0/AD0[6]                 99  RXD0
  P0[4]/I2SRX_CLK/RD2/CAP2[0]       81  LED2/ACC IRQ
  P0[5]/I2SRX_WS/TD2/CAP2[1]        80  CENTER
  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     79  SSEL1
  P0[7]/I2STX_CLK/SCK1/MAT2[1]      78  SCK1
  P0[8]/I2STX_WS/MISO1/MAT2[2]      77  MISO1
  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     76  MOSI1
  P0[10]/TXD2/SDA2/MAT3[0]          48  SDA2
  P0[11]/RXD2/SCL2/MAT3[1]          49  SCL2
  P0[15]/TXD1/SCK0/SCK              62  TXD1
  P0[16]/RXD1/SSEL0/SSEL            63  RXD1
  P0[17]/CTS1/MISO0/MISO            61  CTS1
  P0[18]/DCD1/MOSI0/MOSI            60  DCD1
  P0[19]/DSR1/SDA1                  59  DSR1
  P0[20]/DTR1/SCL1                  58  DTR1
  P0[21]/RI1/RD1                    57  MMC PWR
  P0[22]/RTS1/TD1                   56  RTS1
  P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]    9  BUT1
  P0[24]/AD0[1]/I2SRX_WS/CAP3[1]     8  TEMP
  P0[25]/AD0[2]/I2SRX_SDA/TXD3       7  MIC IN
  P0[26]/AD0[3]/AOUT/RXD3            6  AOUT
  P0[27]/SDA0/USB_SDA               25  USB_SDA
  P0[28]/SCL0/USB_SCL               24  USB_SCL
  P0[29]/USB_D+                     29  USB_D+
  P0[30]/USB_D-                     30  USB_D-
  P1[0]/ENET_TXD0                   95  E_TXD0
  P1[1]/ENET_TXD1                   94  E_TXD1
  P1[4]/ENET_TX_EN                  93  E_TX_EN
  P1[8]/ENET_CRS                    92  E_CRS
  P1[9]/ENET_RXD0                   91  E_RXD0
  P1[10]/ENET_RXD1                  90  E_RXD1
  P1[14]/ENET_RX_ER                 89  E_RX_ER
  P1[15]/ENET_REF_CLK               88  E_REF_CLK
  P1[16]/ENET_MDC                   87  E_MDC
  P1[17]/ENET_MDIO                  86  E_MDIO
  P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] 32  USB_UP_LED
  P1[19]/MC0A/#USB_PPWR/CAP1[1]     33  #USB_PPWR
  P1[20]/MCFB0/PWM1[2]/SCK0         34  SCK0
  P1[21]/MCABORT/PWM1[3]/SSEL0      35  SSEL0
  P1[22]/MC0B/USB_PWRD/MAT1[0]      36  USBH_PWRD
  P1[23]/MCFB1/PWM1[4]/MISO0        37  MISO0
  P1[24]/MCFB2/PWM1[5]/MOSI0        38  MOSI0
  P1[25]/MC1A/MAT1[1]               39  LED1
  P1[26]/MC1B/PWM1[6]/CAP0[0]       40  CS_UEXT
  P1[27]/CLKOUT/#USB_OVRCR/CAP0[1]  43  #USB_OVRCR
  P1[28]/MC2A/PCAP1[0]/MAT0[0]      44  P1.28
  P1[29]/MC2B/PCAP1[1]/MAT0[1]      45  P1.29
  P1[30]/VBUS/AD0[4]                21  VBUS
  P1[31]/SCK1/AD0[5]                20  AIN5
  P2[0]/PWM1[1]/TXD1                75  UP
  P2[1]/PWM1[2]/RXD1                74  DOWN
  P2[2]/PWM1[3]/CTS1/TRACEDATA[3]   73  TRACE_D3
  P2[3]/PWM1[4]/DCD1/TRACEDATA[2]   70  TRACE_D2
  P2[4]/PWM1[5]/DSR1/TRACEDATA[1]   69  TRACE_D1
  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   68  TRACE_D0
  P2[6]/PCAP1[0]/RI1/TRACECLK       67  TRACE_CLK
  P2[7]/RD2/RTS1                    66  LEFT
  P2[8]/TD2/TXD2                    65  RIGHT
  P2[9]/USB_CONNECT/RXD2            64  USBD_CONNECT
  P2[10]/#EINT0/NMI                 53  ISP_E4
  P2[11]/#EINT1/I2STX_CLK           52  #EINT1
  P2[12]/#EINT2/I2STX_WS            51  WAKE-UP
  P2[13]/#EINT3/I2STX_SDA           50  BUT2
  P3[25]/MAT0[0]/PWM1[2]            27  LCD_RST
  P3[26]/STCLK/MAT0[1]/PWM1[3]      26  LCD_BL

  Serial Console
  --------------

  The LPC1766-STK board has two serial connectors.  One, RS232_0, connects to
  the LPC1766 UART0.  This is the DB-9 connector next to the power connector.
  The other RS232_1, connect to the LPC1766 UART1.  This is he DB-9 connector
  next to the Ethernet connector.

  Simple UART1 is the more flexible UART and since the needs for a serial
  console are minimal, the more minimal UART0/RS232_0 is used for the NuttX
  system console.  Of course, this can be changed by editting the NuttX
  configuration file as discussed below.

  The serial console is configured as follows (57600 8N1):

    BAUD: 57600
    Number of Bits: 8
    Parity: None
    Stop bits: 1

  You will need to connect a monitor program (Hyperterminal, Tera Term,
  minicom, whatever) to UART0/RS232_0 and configure the serial port as
  shown above.

  NOTE: The ostest example works fine at 115200, but the other configurations
  have problems at that rate (probably because they use the interrupt driven
  serial driver).  Other LPC17xx boards with the same clocking will run at
  115200.

  LCD
  ---

  The LPC1766-STK has a Nokia 6100 132x132 LCD and either a Phillips PCF8833
  or an Epson S1D15G10 LCD controller.  The NuttX configuration may have to
  be adjusted depending on which controller is used with the LCD.  The
  "LPC1766-STK development board Users Manual" states tha the board features
  a "LCD NOKIA 6610 128x128 x12bit color TFT with Epson LCD controller."
  But, referring to a different Olimex board, "Nokia 6100 LCD Display
  Driver," Revision 1, James P. Lynch ("Nokia 6100 LCD Display Driver.pdf")
  says:
  
  "The major irritant in using this display is identifying the graphics
   controller; there are two possibilities (Epson S1D15G00 or Philips
   PCF8833). The LCD display sold by the German Web Shop Jelu has a Leadis
   LDS176 controller but it is 100% compatible with the Philips PCF8833).
   So how do you tell which controller you have? Some message boards have
   suggested that the LCD display be disassembled and the controller chip
   measured with a digital caliper – well that’s getting a bit extreme.

  "Here’s what I know. The Olimex boards have both display controllers
   possible; if the LCD has a GE-12 sticker on it, it’s a Philips PCF8833.
   If it has a GE-8 sticker, it’s an Epson controller. The older Sparkfun
   6100 displays were Epson, their web site indicates that the newer ones
   are an Epson clone. Sparkfun software examples sometimes refer to the
   Philips controller so the whole issue has become a bit murky. The
   trading companies in Honk Kong have no idea what is inside the displays
   they are selling. A Nokia 6100 display that I purchased from Hong Kong
   a couple of weeks ago had the Philips controller."

  The LCD connects to the LPC1766 via SPI and two GPIOs.  The two GPIOs are
  noted above:
  
    P1.21 is the SPI chip select, and
    P3.25 is the LCD reset
    P3.26 is PWM1 output used to control the backlight intensity.
  
  MISO0 and MOSI0 are join via a 1K ohm resistor so the LCD appears to be
  write only.

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems. Testing was performed using the Cygwin
  environment.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The CodeSourcery GNU toolchain,
  2. The devkitARM GNU toolchain,
  3. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the NuttX buildroot toolchain.  However,
  the make system is setup to default to use the devkitARM toolchain.  To use
  the CodeSourcery or devkitARM toolchain, you simply need add one of the
  following configuration options to your .config (or defconfig) file:

    CONFIG_LPC17_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_LPC17_CODESOURCERYL=y   : CodeSourcery under Linux
    CONFIG_LPC17_DEVKITARM=y       : devkitARM under Windows
    CONFIG_LPC17_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)

  If you are not using CONFIG_LPC17_BUILDROOT, then you may also have to modify
  the PATH in the setenv.h file if your make cannot find the tools.

  NOTE: the CodeSourcery (for Windows)and devkitARM are Windows native toolchains.
  The CodeSourcey (for Linux) and NuttX buildroot toolchains are Cygwin and/or
  Linux native toolchains. There are several limitations to using a Windows based
  toolchain in a Cygwin environment.  The three biggest are:

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

  NOTE 1: The CodeSourcery toolchain (2009q1) does not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

IDEs
^^^^

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
  3) Set up include pathes:  You will need include/, arch/arm/src/lpc17xx,
     arch/arm/src/common, arch/arm/src/armv7-m, and sched/.
  4) All assembly files need to have the definition option -D __ASSEMBLY__
     on the command line.

  Startup files will probably cause you some headaches.  The NuttX startup file
  is arch/arm/src/lpc17x/lpc17_vectors.S.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/buildroot/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh olimex-lpc1766stk/<sub-dir>

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
  details PLUS some special instructions that you will need to follow if you
  are building a Cortex-M3 toolchain for Cygwin under Windows.

  NOTE:  Unfortunately, the 4.6.3 EABI toolchain is not compatible with the
  the NXFLAT tools.  See the top-level TODO file (under "Binary loaders") for
  more information about this problem. If you plan to use NXFLAT, please do not
  use the GCC 4.6.3 EABI toochain; instead use the GCC 4.3.3 OABI toolchain.
  See instructions below.

NuttX OABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
^^^^^^^^^^^^^^^^

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
^^^^

  If CONFIG_ARCH_LEDS is defined, then support for the LPC1766-STK LEDs will be
  included in the build.  See:

  - configs/olimex-lpc1766stk/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/olimex-lpc1766stk/src/lpc1766stk_internal.h - GPIO settings for the LEDs.

  - configs/olimex-lpc1766stk/src/up_leds.c - LED control logic.

  The LPC1766-STK has two LEDs.  If CONFIG_ARCH_LEDS is defined, these LEDs will
  be controlled as follows for NuttX debug functionality (where NC means "No Change").
  Basically,

  LED1:
  - OFF means that the OS is still initializing. Initialization is very fast so
    if you see this at all, it probably means that the system is hanging up
    somewhere in the initialization phases.
  - ON means that the OS completed initialization.
  - Glowing means that the LPC17 is running in a reduced power mode: LED1 is
    turned off when the processor enters sleep mode and back on when it wakesup
    up.

  LED2:
  - ON/OFF toggles means that various events are happening.
  - GLowing: LED2 is turned on and off on every interrupt so even timer interrupts
    should cause LED2 to glow faintly in the normal case.
  - Flashing. If the LED2 is flashing at about 2Hz, that means that a crash
    has occurred.  If CONFIG_ARCH_STACKDUMP=y, you will get some diagnostic
    information on the console to help debug what happened.

  NOTE:  LED2 is controlled by a jumper labeled: ACC_IRQ/LED2.  That jump must be
  in the LED2 position in order to support LED2.

  LED1    LED2      Meaning
  ------- --------  --------------------------------------------------------------------
   OFF    OFF      Still initializing and there is no interrupt activity. 
                    Initialization is very fast so if you see this, it probably means
                    that the system is hung up somewhere in the initialization phases.
   OFF     Glowing  Still initializing (see above) but taking interrupts.
   OFF     ON       This would mean that (1) initialization did not complete but the
                    software is hung, perhaps in an infinite loop, somewhere inside
                    of an interrupt handler.
   OFF     Flashing Ooops!  We crashed before finishing initialization (or, perhaps
                    after initialization, during an interrupt while the LPC17xx was
                    sleeping -- see below).
 
   ON      OFF      The system has completed initialization, but is apparently not taking
                    any interrupts.
   ON      Glowing  The OS successfully initialized and is taking interrupts (but, for
                    some reason, is never entering a reduced power mode -- perhaps the
                    CPU is very busy?).
   ON      ON       This would mean that (1) the OS complete initialization, but (2)
                    the software is hung, perhaps in an infinite loop, somewhere inside
                    of a signal or interrupt handler.
   Glowing Glowing  This is also a normal healthy state: The OS successfully initialized,
                    is running in reduced power mode, but taking interrupts.  The glow
                    is very faint and you may have to dim the lights to see that LEDs are
                    active at all!  See note below.
   ON      Flashing Ooops!  We crashed sometime after initialization.

  NOTE: In glowing/glowing case, you get some good subjective information about the
  behavior of your system by looking at the level of the LED glow (or better, by
  connecting O-Scope and calculating the actual duty):
  
  1. The intensity of the glow is determined by the duty of LED on/off toggle --
     as the ON period becomes larger with respect the OFF period, the LED will
     glow more brightly.
  2. LED2 is turned ON when entering an interrupt and turned OFF when returning from
     the interrupt.  A brighter LED2 means that the system is spending more time in
     interrupt handling.
  3. LED1 is turned OFF just before the processor goes to sleep.  The processor
     sleeps until awakened by an interrupt.  LED1 is turned back ON after the
     processor is re-awakened -- actually after returning from the interrupt that
     cause the processor to re-awaken (LED1 will be off during the execution of
     that interrupt).  So a brighter LED1 means that the processor is spending
     less time sleeping.

  When my LPC1766 sits IDLE -- doing absolutely nothing but processing timer interrupts --
  I see the following:
  
  1. LED1 glows dimly due to the timer interrupts.
  2. But LED2 is even more dim!  The LED ON time excludes the time processing the
     interrupt that re-awakens the processing.  So this tells me that the LPC1766 is
     spending more time processing timer interrupts than doing any other kind of
     processing.  That, of course, makes sense if the system is truly idle and only
     processing timer interrupts.

Using OpenOCD and GDB with an FT2232 JTAG emulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  Downloading OpenOCD
  
    You can get information about OpenOCD here: http://openocd.berlios.de/web/
    and you can download it from here. http://sourceforge.net/projects/openocd/files/.
    To get the latest OpenOCD with more mature lpc17xx, you have to download
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
    LPC1766-STK (http://www.olimex.com).  OpenOCD requires a configuration
    file.  I keep the one I used last here:
    
      configs/olimex-lpc1766stk/tools/olimex.cfg

    However, the "correct" configuration script to use with OpenOCD may
    change as the features of OpenOCD evolve.  So you should at least
    compare that olimex.cfg file with configuration files in
    /usr/local/share/openocd/scripts/target (or /home/OpenOCD/openocd/tcl/target).
    As of this writing, there is no script for the lpc1766, but the
    lpc1768 configurtion can be used after changing the flash size to
    256Kb.  That is, change:

      flash bank $_FLASHNAME lpc2000 0x0 0x80000 0 0 $_TARGETNAME ...

    To:
 
      flash bank $_FLASHNAME lpc2000 0x0 0x40000 0 0 $_TARGETNAME ...
    
    There is also a script on the tools/ directory that I use to start
    the OpenOCD daemon on my system called oocd.sh.  That script will
    probably require some modifications to work in another environment:
  
    - Possibly the value of OPENOCD_PATH and TARGET_PATH
    - It assumes that the correct script to use is the one at
      configs/olimex-lpc1766stk/tools/olimex.cfg

  Starting OpenOCD

    Then you should be able to start the OpenOCD daemon like:

      configs/olimex-lpc1766stk/tools/oocd.sh $PWD

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
    2. I usually have to reset, halt, and 'load nuttx' a second time.  For
       some reason, the first time apparently does not fully program the
       FLASH.
    3. The MCU must be halted prior to loading code using 'mon reset'
       as described below.
 
    OpenOCD will support several special 'monitor' commands.  These
    GDB commands will send comments to the OpenOCD monitor.  Here
    are a couple that you will need to use:
  
     (gdb) monitor reset
     (gdb) monitor halt

    NOTES:
    1. The MCU must be halted using 'mon halt' prior to loading code.
    2. Reset will restart the processor after loading code.
    3. The 'monitor' command can be abbreviated as just 'mon'.

Olimex LPC1766-STK Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

    CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
       be set to:

       CONFIG_ARCH=arm

    CONFIG_ARCH_family - For use in C code:

       CONFIG_ARCH_ARM=y

    CONFIG_ARCH_architecture - For use in C code:

       CONFIG_ARCH_CORTEXM3=y

    CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

       CONFIG_ARCH_CHIP=lpc17xx

    CONFIG_ARCH_CHIP_name - For use in C code to identify the exact
       chip:

       CONFIG_ARCH_CHIP_LPC1766=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=olimex-lpc1766stk (for the Olimex LPC1766-STK)

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPC1766STK=y

    CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
       of delay loops

    CONFIG_ENDIAN_BIG - define if big endian (default is little
       endian)

    CONFIG_DRAM_SIZE - Describes the installed DRAM (CPU SRAM in this case):

       CONFIG_DRAM_SIZE=(32*1024) (32Kb)

       There is an additional 32Kb of SRAM in AHB SRAM banks 0 and 1.

    CONFIG_DRAM_START - The start address of installed DRAM

       CONFIG_DRAM_START=0x10000000

    CONFIG_ARCH_IRQPRIO - The LPC17xx supports interrupt prioritization

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
    
      CONFIG_LPC17_MAINOSC=y
      CONFIG_LPC17_PLL0=y
      CONFIG_LPC17_PLL1=n
      CONFIG_LPC17_ETHERNET=n
      CONFIG_LPC17_USBHOST=n
      CONFIG_LPC17_USBOTG=n
      CONFIG_LPC17_USBDEV=n
      CONFIG_LPC17_UART0=y
      CONFIG_LPC17_UART1=n
      CONFIG_LPC17_UART2=n
      CONFIG_LPC17_UART3=n
      CONFIG_LPC17_CAN1=n
      CONFIG_LPC17_CAN2=n
      CONFIG_LPC17_SPI=n
      CONFIG_LPC17_SSP0=n
      CONFIG_LPC17_SSP1=n
      CONFIG_LPC17_I2C0=n
      CONFIG_LPC17_I2C1=n
      CONFIG_LPC17_I2S=n
      CONFIG_LPC17_TMR0=n
      CONFIG_LPC17_TMR1=n
      CONFIG_LPC17_TMR2=n
      CONFIG_LPC17_TMR3=n
      CONFIG_LPC17_RIT=n
      CONFIG_LPC17_PWM=n
      CONFIG_LPC17_MCPWM=n
      CONFIG_LPC17_QEI=n
      CONFIG_LPC17_RTC=n
      CONFIG_LPC17_WDT=n
      CONFIG_LPC17_ADC=n
      CONFIG_LPC17_DAC=n
      CONFIG_LPC17_GPDMA=n
      CONFIG_LPC17_FLASH=n

  LPC17xx specific device driver settings

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

  LPC17xx specific CAN device driver settings.  These settings all
  require CONFIG_CAN:

    CONFIG_CAN_EXTID - Enables support for the 29-bit extended ID.  Default
      Standard 11-bit IDs.
    CONFIG_CAN1_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_CAN1 is defined.
    CONFIG_CAN2_BAUD - CAN1 BAUD rate.  Required if CONFIG_LPC17_CAN2 is defined.
    CONFIG_CAN1_DIVISOR - CAN1 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN2_DIVISOR - CAN2 is clocked at CCLK divided by this number.
      (the CCLK frequency is divided by this number to get the CAN clock).
      Options = {1,2,4,6}. Default: 4.
    CONFIG_CAN_TSEG1 - The number of CAN time quanta in segment 1. Default: 6
    CONFIG_CAN_TSEG2 - the number of CAN time quanta in segment 2. Default: 7

  LPC17xx specific PHY/Ethernet device driver settings.  These setting
  also require CONFIG_NET and CONFIG_LPC17_ETHERNET.

    CONFIG_PHY_KS8721 - Selects Micrel KS8721 PHY
    CONFIG_PHY_AUTONEG - Enable auto-negotion
    CONFIG_PHY_SPEED100 - Select 100Mbit vs. 10Mbit speed.
    CONFIG_PHY_FDUPLEX - Select full (vs. half) duplex

    CONFIG_NET_EMACRAM_SIZE - Size of EMAC RAM.  Default: 16Kb
    CONFIG_NET_NTXDESC - Configured number of Tx descriptors. Default: 18
    CONFIG_NET_NRXDESC - Configured number of Rx descriptors. Default: 18
    CONFIG_NET_PRIORITY - Ethernet interrupt priority.  The is default is
      the higest priority.
    CONFIG_NET_WOL - Enable Wake-up on Lan (not fully implemented).
    CONFIG_NET_REGDEBUG - Enabled low level register debug.  Also needs
      CONFIG_DEBUG.
    CONFIG_NET_DUMPPACKET - Dump all received and transmitted packets.
      Also needs CONFIG_DEBUG.
    CONFIG_NET_HASH - Enable receipt of near-perfect match frames.
    CONFIG_NET_MULTICAST - Enable receipt of multicast (and unicast) frames.
      Automatically set if CONFIG_NET_IGMP is selected.

  LPC17xx USB Device Configuration

    CONFIG_LPC17_USBDEV_FRAME_INTERRUPT
      Handle USB Start-Of-Frame events. 
      Enable reading SOF from interrupt handler vs. simply reading on demand.
      Probably a bad idea... Unless there is some issue with sampling the SOF
      from hardware asynchronously.
    CONFIG_LPC17_USBDEV_EPFAST_INTERRUPT
      Enable high priority interrupts.  I have no idea why you might want to
      do that
    CONFIG_LPC17_USBDEV_NDMADESCRIPTORS
      Number of DMA descriptors to allocate in SRAM.
    CONFIG_LPC17_USBDEV_DMA
      Enable lpc17xx-specific DMA support
    CONFIG_LPC17_USBDEV_NOVBUS
      Define if the hardware implementation does not support the VBUS signal
    CONFIG_LPC17_USBDEV_NOLED
      Define if the hardware  implementation does not support the LED output

  LPC17xx USB Host Configuration
    CONFIG_USBHOST_OHCIRAM_SIZE
      Total size of OHCI RAM (in AHB SRAM Bank 1)
    CONFIG_USBHOST_NEDS
      Number of endpoint descriptors
    CONFIG_USBHOST_NTDS
      Number of transfer descriptors
    CONFIG_USBHOST_TDBUFFERS
      Number of transfer descriptor buffers
    CONFIG_USBHOST_TDBUFSIZE
      Size of one transfer descriptor buffer
    CONFIG_USBHOST_IOBUFSIZE
      Size of one end-user I/O buffer.  This can be zero if the
      application can guarantee that all end-user I/O buffers
      reside in AHB SRAM.

USB Host Configuration
^^^^^^^^^^^^^^^^^^^^^^

The NuttShell (NSH) Nucleus 2G can be modified in order to support
USB host operations.  To make these modifications, do the following:

1. First configure to build the NSH configuration from the top-level
   NuttX directory:

   cd tools
   ./configure nucleus2g/nsh
   cd ..

2. Then edit the top-level .config file to enable USB host.  Make the
   following changes:

   CONFIG_LPC17_USBHOST=y
   CONFIG_USBHOST=y
   CONFIG_SCHED_WORKQUEUE=y

When this change is made, NSH should be extended to support USB flash
devices.  When a FLASH device is inserted, you should see a device
appear in the /dev (pseudo) directory.  The device name should be
like /dev/sda, /dev/sdb, etc.  The USB mass storage device, is present
it can be mounted from the NSH command line like:

   ls /dev
   mount -t vfat /dev/sda /mnt/flash

Files on the connect USB flash device should then be accessible under
the mountpoint /mnt/flash.

Configurations
^^^^^^^^^^^^^^

Each Olimex LPC1766-STK configuration is maintained in a
sudirectory and can be selected as follow:

    cd tools
    ./configure.sh olimex-lpc1766stk/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  ftpc:
    This is a simple FTP client shell used to exercise the capabilities
    of the FTPC library (apps/netutils/ftpc).  This example is configured
    to that it will only work as a "built-in" program that can be run from
    NSH when CONFIG_NSH_BUILTIN_APPS is defined.

    From NSH, the startup command sequence is then:

      nsh> mount -t vfat /dev/mmcsd0 /tmp # Mount the SD card at /tmp
      nsh> cd /tmp                        # cd into the /tmp directory
      nsh> ftpc xx.xx.xx.xx[:pp]          # Start the FTP client
      nfc> login <name> <password>        # Log into the FTP server
      nfc> help                           # See a list of FTP commands

    where xx.xx.xx.xx is the IP address of the FTP server and pp is an
    optional port number (default is the standard FTP port number 21).

    NOTES:

    1. Support for FAT long file names is built-in but can easily be
       removed if you are concerned about Microsoft patent issues (see the
       section "FAT Long File Names" in the top-level COPYING file).
       
       CONFIG_FS_FAT=y
       CONFIG_FAT_LCNAMES=y <-- Long file name support
       CONFIG_FAT_LFN=y
       CONFIG_FAT_MAXFNAME=32
       CONFIG_FS_NXFFS=n
       CONFIG_FS_ROMFS=n

    2. You may also want to define the following in your configuration file.
       Otherwise, you will have not feedback about what is going on:
 
       CONFIG_DEBUG=y
       CONFIG_DEBUG_VERBOSE=y
       CONFIG_DEBUG_FTPC=y

  hidkbd:
    This configuration directory, performs a simple test of the USB host
    HID keyboard class driver using the test logic in apps/examples/hidkbd.

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Default platform/toolchain: This is how the build is configured by
       be default.  These options can easily be re-confured, however.

       CONFIG_HOST_WINDOWS=y                   : Windows
       CONFIG_WINDOWS_CYGWIN=y                 : Cygwin environment on Windows
       CONFIG_ARMV7M_TOOLCHAIN_CODESOURCERYW=y : CodeSourcery under Windows

   nettest:
    This configuration directory may be used to enable networking using the
    LPC17xx's Ethernet controller. It uses apps/examples/nettest to excercise the
    TCP/IP network.

  nsh:
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.
    Support for the board's SPI-based MicroSD card is included.

    NOTE:  If you start the program with no SD card inserted, there will be
    a substantial delay. This is because there is no hardware support to sense
    whether or not an SD card is inserted.  As a result, the driver has to
    go through many retries and timeouts before it finally decides that there
    is not SD card in the slot.

    Configuration Notes:

    NOTES:
 
    1. This configuration uses the mconf-based configuration tool.  To
       change this configuration using that tool, you should:

       a. Build and install the kconfig-mconf tool.  See nuttx/README.txt
          and misc/tools/

       b. Execute 'make menuconfig' in nuttx/ in order to start the
          reconfiguration process.

    2. Uses the older, OABI, buildroot toolchain.  But that is easily
       reconfigured:

       CONFIG_ARMV7M_TOOLCHAIN_BUILDROOT=y : Buildroot toolchain
       CONFIG_ARMV7M_OABI_TOOLCHAIN=y      : Older, OABI toolchain

    3. This configuration supports a network.  You may have to change
       these settings for your network:

       CONFIG_NSH_IPADDR=0x0a000002        : IP address: 10.0.0.2
       CONFIG_NSH_DRIPADDR=0x0a000001      : Gateway:    10.0.0.1
       CONFIG_NSH_NETMASK=0xffffff00       : Netmask:    255.255.255.0

    4. This configuration supports the SPI-based MMC/SD card slot.
       FAT file system support for FAT long file names is built-in but
       can easily be removed if you are concerned about Microsoft patent
       issues (see the section "FAT Long File Names" in the top-level
       COPYING file).

       CONFIG_FAT_LFN=y                    : Enables long file name support

  nx:
    An example using the NuttX graphics system (NX).  This example uses
    the Nokia 6100 LCD driver. NOTE:  The Nokia 6100 driver does not
    work on this board as of this writing.

  ostest:
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.

  slip-httpd:
    This configuration is identical to the thttpd configuration except that
    it uses the SLIP data link layer via a serial driver instead of the
    Ethernet data link layer.  The Ethernet driver is disabled; SLIP IP
    packets are exchanged on UART1; UART0 is still the serial console.

    1. Configure and build the slip-httpd configuration.
    2. Connect to a Linux box (assuming /dev/ttyS0)
    3. Reset on the target side and attach SLIP on the Linux side:
    
       $ modprobe slip
       $ slattach -L -p slip -s 57600 /dev/ttyS0 &
 
       This should create an interface with a name like sl0, or sl1, etc.
       Add -d to get debug output.  This will show the interface name.

       NOTE: The -L option is included to suppress use of hardware flow
       control.  This is necessary because I haven't figured out how to 
       use the UART1 hardware flow control yet.

       NOTE: The Linux slip module hard-codes its MTU size to 296.  So you
       might as well set CONFIG_NET_BUFSIZE to 296 as well.

    4. After turning over the line to the SLIP driver, you must configure
       the network interface. Again, you do this using the standard
       ifconfig and route commands. Assume that we have connected to a
       host PC with address 192.168.0.101 from your target with address
       10.0.0.2. On the Linux PC you would execute the following as root:

       $ ifconfig sl0 10.0.0.1 pointopoint 10.0.0.2 up
       $ route add 10.0.0.2 dev sl0

       Assuming the SLIP is attached to device sl0.

    5. For monitoring/debugging traffic:

       $ tcpdump -n -nn -i sl0 -x -X -s 1500

    NOTE: Only UART1 supports the hardware handshake.  If hardware
    handshake is not available, then you might try the slattach option
    -L which is supposed to enable "3-wire operation."

    NOTE: This configurat only works with VERBOSE debug disabled.  For some
    reason, certain debug statements hang(?).
    
    NOTE: This example does not use UART1's hardware flow control.  UART1
    hardware flow control is partially implemented but does not behave as
    expected.  It needs a little more work.

  thttpd:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTE: See note above with regard to the EABI/OABI buildroot
    toolchains.  This example can only be built using the older
    OABI toolchain.

  usbserial:
    This configuration directory exercises the USB serial class
    driver at apps/examples/usbserial.  See apps/examples/README.txt for
    more information.

  usbstorage:
    This configuration directory exercises the USB mass storage
    class driver at apps/examples/usbstorage.  See apps/examples/README.txt
    for more information.

