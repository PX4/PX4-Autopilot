README
^^^^^^

README for NuttX port to the Embedded Artists' base board with the NXP
the LPCXpresso daughter board.

Contents
^^^^^^^^

  LCPXpresso LPC1768 Board
  Embedded Artist's Base Board
  Development Environment
  GNU Toolchain Options
  NuttX EABI "buildroot" Toolchain
  NuttX OABI "buildroot" Toolchain
  NXFLAT Toolchain
  Code Red IDE
  LEDs
  LPCXpresso Configuration Options
  Configurations

LCPXpresso LPC1768 Board
^^^^^^^^^^^^^^^^^^^^^^^^

  Pin Description                  Connector On Board       Base Board
  -------------------------------- --------- -------------- ---------------------

  P0[0]/RD1/TXD3/SDA1               J6-9     I2C E2PROM SDA TXD3/SDA1
  P0[1]/TD1/RXD3/SCL                J6-10                   RXD3/SCL1
  P0[2]/TXD0/AD0[7]                 J6-21    
  P0[3]/RXD0/AD0[6]                 J6-22    
  P0[4]/I2SRX-CLK/RD2/CAP2.0        J6-38                   CAN_RX2
  P0[5]/I2SRX-WS/TD2/CAP2.1         J6-39                   CAN_TX2
  P0[6]/I2SRX_SDA/SSEL1/MAT2[0]     J6-8                    SSEL1, OLED CS
  P0[7]/I2STX_CLK/SCK1/MAT2[1]      J6-7                    SCK1, OLED SCK
  P0[8]/I2STX_WS/MISO1/MAT2[2]      J6-6                    MISO1
  P0[9]/I2STX_SDA/MOSI1/MAT2[3]     J6-5                    MOSI1, OLED data in
  P0[10]                            J6-40                   TXD2/SDA2
  P0[11]                            J6-41                   RXD2/SCL2
  P0[15]/TXD1/SCK0/SCK              J6-13                   TXD1/SCK0
  P0[16]/RXD1/SSEL0/SSEL            J6-14                   RXD1/SSEL0
  P0[17]/CTS1/MISO0/MISO            J6-12                   MISO0
  P0[18]/DCD1/MOSI0/MOSI            J6-11                   MOSI0
  P0[19]/DSR1/SDA1                  PAD17                   N/A
  P0[20]/DTR1/SCL1                  PAD18    I2C E2PROM SCL N/A
  P0[21]/RI1/MCIPWR/RD1             J6-23                  
  P0[22]/RTS1/TD1                   J6-24    LED            
  P0[23]/AD0[0]/I2SRX_CLK/CAP3[0]   J6-15                   AD0.0
  P0[24]/AD0[1]/I2SRX_WS/CAP3[1]    J6-16                   AD0.1
  P0[25]/AD0[2]/I2SRX_SDA/TXD3      J6-17                   AD0.2
  P0[26]/AD0[3]/AOUT/RXD3           J6-18                   AD0.3/AOUT / RGB LED
  P0[27]/SDA0/USB_SDA               J6-25                   
  P0[28]/SCL0                       J6-26                   
  P0[29]/USB_D+                     J6-37                   USB_D+
  P0[30]/USB_D-                     J6-36                   USB_D-

  P1[0]/ENET-TXD0                   J6-34?  TXD0            TX-(Ethernet PHY)
  P1[1]/ENET_TXD1                   J6-35?  TXD1            TX+(Ethernet PHY)
  P1[4]/ENET_TX_EN                          TXEN            N/A
  P1[8]/ENET_CRS                            CRS_DV/MODE2    N/A
  P1[9]/ENET_RXD0                   J6-32?  RXD0/MODE0      RD-(Ethernet PHY)
  P1[10]/ENET_RXD1                  J6-33?  RXD1/MODE1      RD+(Ethernet PHY)
  P1[14]/ENET_RX_ER                         RXER/PHYAD0     N/A
  P1[15]/ENET_REF_CLK                       REFCLK          N/A
  P1[16]/ENET_MDC                           MDC             N/A
  P1[17]/ENET_MDIO                          MDIO            N/A
  P1[18]/USB_UP_LED/PWM1[1]/CAP1[0] PAD1                    N/A
  P1[19]/MC0A/USB_PPWR/N_CAP1.1     PAD2                    N/A
  P1[20]/MCFB0/PWM1.2/SCK0          PAD3                    N/A
  P1[21]/MCABORT/PWM1.3/SSEL0       PAD4                    N/A
  P1[22]/MC0B/USB-PWRD/MAT1.0       PAD5                    N/A
  P1[23]/MCFB1/PWM1.4/MISO0         PAD6                    N/A
  P1[24]/MCFB2/PWM1.5/MOSI0         PAD7                    N/A
  P1[25]/MC1A/MAT1.1                PAD8                    N/A
  P1[26]/MC1B/PWM1.6/CAP0.0         PAD9                    N/A
  P1[27]/CLKOUT/USB-OVRCR-N/CAP0.1  PAD10                   N/A
  P1[28]/MC2A/PCAP1.0/MAT0.0        PAD11                   N/A
  P1[29]/MC2B/PCAP1.1/MAT0.1        PAD12                   N/A
  P1[30]/VBUS/AD0[4]                J6-19                   AD0.4
  P1[31]/SCK1/AD0[5]                J6-20                   AD0.5

  P2[0]/PWM1.1/TXD1                 J6-42                   PWM1.1 / RGB LED / RS422 RX
  P2[1]/PWM1.2/RXD1                 J6-43                   PWM1.2 / OLED voltage / RGB LED
  P2[2]/PWM1.3/CTS1/TRACEDATA[3]    J6-44                   PWM1.3
  P2[3]/PWM1.4/DCD1/TRACEDATA[2]    J6-45                   PWM1.4
  P2[4]/PWM1.5/DSR1/TRACEDATA[1]    J6-46                   PWM1.5
  P2[5]/PWM1[6]/DTR1/TRACEDATA[0]   J6-47                   PWM1.6
  P2[6]/PCAP1[0]/RI1/TRACECLK       J6-48    
  P2[7]/RD2/RTS1                    J6-49                   OLED command/data
  P2[8]/TD2/TXD2                    J6-50    
  P2[9]/USB_CONNECT/RXD2            PAD19   USB Pullup      N/A
  P2[10]/EINT0/NMI                  J6-51    
  P2[11]/EINT1/I2STX_CLK            J6-52    
  P2[12]/EINT2/I2STX_WS             j6-53    
  P2[13]/EINT3/I2STX_SDA            J6-27                 

  P3[25]/MAT0.0/PWM1.2              PAD13                   N/A
  P3[26]/STCLK/MAT0.1/PWM1.3        PAD14                   N/A

  P4[28]/RX-MCLK/MAT2.0/TXD3        PAD15                   N/A
  P4[29]/TX-MCLK/MAT2.1/RXD3        PAD16                   N/A

Embedded Artist's Base Board
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Jumpers
-------

  There are many jumpers on the base board.  A usable combination is the
  default jumper settings WITH the two J54 jumpers both removed.  Those
  jumpers are for ISP support and will cause the board to reset.

  To use the SD, J55 must be set to provide chip select PIO1_11 signal as
  the SD slot chip select.

SD Slot
-------

       Base-board  J4/J6 LPC1768
  SD   Signal      Pin   Pin
  ---  ----------- ----- --------
  CS   PIO1_11*     55   P2.2
  DIN  PIO0_9-MOSI   5   P0.9 MOSI1
  DOUT PIO0_8-MISO   6   P0.8 MISO1
  CLK  PIO2_11-SCK   7   P0.9 SCK1
  CD   PIO2_10      52   P2.11

  These jumper settings are required:

  *J55 must be set to provide chip select PIO1_11 signal as the SD slot
   chip select.
 
USB Device
----------
 
  Base-board          J4/J6 LPC1768
  Signal              Pin   Pin
  ------------------- ----- --------
  PIO0_6-USB_CONNECT* 23    P0.21
  USB_DM              36    USB_D-
  USB_DP              37    USB_D+
  PIO0_3-VBUS_SENSE** 39    P0.5

  These jumper settings are listed for information only.  They are *not*
  required for use with NuttX and LPCXpresso.  The configurable pins
  (P0.21 and P0.5) are not used!

  *J14 must be set to permit GPIO control of the USB connect pin
 **J12 must be set to permit GPIO control of the USB vbus sense pin
   J23 is associated the LEDs used for USB support

  Here is a more detailed pin mapping:

  ---------------------------------------------+------+-----------------------------------------------
                    LPCXpresso                 | J4/6 |            Base Board
  ---------------------------------------------|      |-----------------------------------------------
  LPC1768                        Signal        |      | Signal             Connection
  ------------------------------ --------------+------+------------------- ---------------------------
  P0.29/USB-D+                   P0[29]/USB-D+ |  37  | USB_DP             USB D+
  P0.30/USB-D-                   P0[30]/USB-D- |  36  | USB_DM             USB D-
  P1.18/USB-UP-LED/PWM1.1/CAP1.0 PAD1          | N/A  | N/A                N/A
  P1.30/VBUS/AD0.4               P1[30]        |  19  | PIO1_3             (Not used on board)
  P2.9/USB-CONNECT/RXD2*         PAD19         | N/A  | N/A                N/A
  ------------------------------ --------------+------+------------------- ---------------------------
  P0.21/RI1/RD1                  P0[21]        |  23  | PIO0_6-USB_CONNECT VBUS via J14 and transistor
  P0.5/I2SRX-WS/TD2/CAP2.1       P0[5]         |  39  | PIO0_3-VBUS_SENSE  VBUS via J12
  ------------------------------ --------------+------+------------------- ---------------------------

  *P2.9 connects to a transistor driven USB-D+ pullup on the LPCXpresso board.

96x64 White OLED with I2C/SPI interface
---------------------------------------
  The OLED display can be connected either to the SPI-bus or the I2C-bus.

  Jumper Settings:

    - For the SPI interface (default), insert jumpers in J42, J43, J45 pin1-2
      and J46 pin 1-2.
    - For I2C interface, insert jumpers in J45 pin 2-3, J46 pin 2-3 and J47.

    In either case insert a jumper in J44 in order to allow PIO1_10 to control
    the OLED-voltage.

  Jumper Signal Control:

    J42: Short: SPI Open: I2C (Default: inserted)

    J44: Allow control of OLED voltage (Default: inserted)

      PIO1_10-------->J44 ---------->FAN5331

    Common Reset:

      PIO0_0-RESET ---------------> RES#

    J43: Select OLED chip select
    J58: For embed (Default: not inserted)

      PIO0_2--------------->J43 ---->CS#
      PIO2_7--------->J58 ->J43 ---->D/C#
      PIO0_8-MISO --------^

    J45: Select SPI or I2C clock (Default: SPI clock)

      PIO2_11-SCK---->J45 ----------> D0
      PIO0_4-SCL------------^

    J46: Select serial data input (Default: SPI MOSI)

      PIO0_9-MOSI---->J46 ----------> D1
      I2C_SDA---------------^

    J47: Allow I2C bi-directional communications (Default: SPI unidirectional)

      PIO0_5-SDA---->J47 ----------> D2

    LPCXpresso Signals

      ----------------------------+-------+-------------- ----------------------------------------
      LPC1758 Pin                 | J4/6  | Base Board    Description
      ----------------------------+-------+-------------- ----------------------------------------
      P2.1/PWM1.2/RXD1            |  43   | PIO1_10       FAN5331 Power Control (SHDN#)
      RESET_N                     |   4   | PIO0_0-RESET  OLED reset (RES#) -- Resets EVERYTHING
      P0.6/I2SRX-SDA/SSEL1/MAT2.0 |   8   | PIO0_2        OLED chip select (CS#)
      P2.7/RD2/RTS1               |  49   | PIO2_7        OLED command/data (D/C#)
      P0.7/I2STX-CLK/SCK1/MAT2.1  |   7   | PIO2_11-SCK   OLED clock (D0)
      P0.9/I2STX-SDA/MOSI1/MAT2.3 |   5   | PIO0_9-MOSI   OLED data in (D1)
      ----------------------------+-------+-------------- ----------------------------------------

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

  1. The Code Red GNU toolchain
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain,
  4. The NuttX buildroot Toolchain (see below).

  All testing has been conducted using the Code Red toolchain and the
  make system is setup to default to use the Code Red Linux toolchain.  To use
  the other toolchain, you simply need add one of the following configuration
  options to your .config (or defconfig) file:

    CONFIG_LPC17_CODESOURCERYW=y   : CodeSourcery under Windows
    CONFIG_LPC17_CODESOURCERYL=y   : CodeSourcery under Linux
    CONFIG_LPC17_DEVKITARM=y       : devkitARM under Windows
    CONFIG_LPC17_BUILDROOT=y       : NuttX buildroot under Linux or Cygwin (default)
    CONFIG_LPC17_CODEREDW=n        : Code Red toolchain under Windows
    CONFIG_LPC17_CODEREDL=y        : Code Red toolchain under Linux

  You may also have to modify the PATH in the setenv.h file if your make cannot
  find the tools.

  NOTE: the CodeSourcery (for Windows), devkitARM, and Code Red (for Windoes)
  are Windows native toolchains.  The CodeSourcey (for Linux), Code Red (for Linux)
  and NuttX buildroot toolchains are Cygwin and/or Linux native toolchains. There
  are several limitations to using a Windows based toolchain in a Cygwin
  environment.  The three biggest are:

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

Code Red IDE
^^^^^^^^^^^^

  NuttX is built using command-line make.  It can be used with an IDE, but some
  effort will be required to create the project.
  
  Makefile Build
  --------------
  Under Linux Eclipse, it is pretty easy to set up an "empty makefile project" and
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

  Using Code Red GNU Tools from Cygwin
  ------------------------------------

  Under Cygwin, the Code Red command line tools (e.g., arm-non-eabi-gcc) cannot
  be executed because the they only have execut privileges for Administrators.  I
  worked around this by:
  
  Opening a native Cygwin RXVT as Administrator (Right click, "Run as administrator"),
  then executing 'chmod 755 *.exe' in the following directories:

  /cygdrive/c/nxp/lpcxpreeso_3.6/bin, and
  /cygdrive/c/nxp/lpcxpreeso_3.6/Tools/bin

  Command Line Flash Programming
  ------------------------------

  If using LPCLink as your debug connection, first of all boot the LPC-Link using
  the script:

    bin\Scripts\bootLPCXpresso type

  where type = winusb for Windows XP, or type = hid for Windows Vista / 7.

  Now run the flash programming utility with the following options

    flash_utility wire -ptarget -flash-load[-exec]=filename [-load-base=base_address]

  Where flash_utility is one of:

    crt_emu_lpc11_13 (for LPC11xx or LPC13xx parts)
    crt_emu_cm3_nxp (for LPC17xx parts)
    crt_emu_a7_nxp (for LPC21/22/23/24 parts)
    crt_emu_a9_nxp (for LPC31/32 and LPC29xx parts)
    crt_emu_cm3_lmi (for TI Stellaris parts)

  wire is one of:

    (empty) (for Red Probe+, Red Probe, RDB1768v1, or TI Stellaris evaluation boards)
    -wire=hid (for RDB1768v2 without upgraded firmware)
    -wire=winusb (for RDB1768v2 with upgraded firmware)
    -wire=winusb (for LPC-Link on Windows XP)
    -wire=hid (for LPC-Link on Windows Vista/ Windows 7)

  target is the target chip name. For example LPC1343, LPC1114/301, LPC1768 etc.

  filename is the file to flash program. It may be an executable (axf) or a binary
  (bin) file. If using a binary file, the base_address must be specified.

  base_address is the base load address when flash programming a binary file. It
  should be specified as a hex value with a leading 0x.

  Note:
  - flash-load will leave the processor in a stopped state
  - flash-load-exec will start execution of application as soon as download has
    completed.

  Examples
    To load the executable file app.axf and start it executing on an LPC1758
    target using Red Probe, use the following command line:

      crt_emu_cm3_nxp -pLPC1758 -flash-load-exec=app.axf

    To load the binary file binary.bin to address 0x1000 to an LPC1343 target
    using LPC-Link on Windows XP, use the following command line:

      crt_emu_lpc11_13_nxp -wire=hid -pLPC1343 -flash-load=binary.bin -load-base=0x1000

  tools/flash.sh
  --------------

  All of the above steps are automated in the bash script flash.sh that can
  be found in the configs/lpcxpresso/tools directory.

NuttX EABI "buildroot" Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/projects/nuttx/files/).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh lpcxpresso-lpc1768/<sub-dir>

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

  If CONFIG_ARCH_LEDS is defined, then support for the LPCXpresso LEDs will be
  included in the build.  See:

  - configs/lpcxpresso-lpc1768/include/board.h - Defines LED constants, types and
    prototypes the LED interface functions.

  - configs/lpcxpresso-lpc1768/src/lpcxpresso_internal.h - GPIO settings for the LEDs.

  - configs/lpcxpresso-lpc1768/src/up_leds.c - LED control logic.

  The LPCXpresso LPC1768 has a single LEDs (there are more on the Embedded Artists
  base board, but those are not controlled by NuttX).  Usage this single LED by NuttX
  is as follows:

  - The LED is not illuminated until the LPCXpresso completes initialization.
  
    If the LED is stuck in the OFF state, this means that the LPCXpresso did not 
    complete intialization.

  - Each time the OS enters an interrupt (or a signal) it will turn the LED OFF and
    restores its previous stated upon return from the interrupt (or signal).

    The normal state, after initialization will be a dull glow.  The brightness of
    the glow will be inversely related to the proportion of time spent within interrupt
    handling logic.  The glow may decrease in brightness when the system is very
    busy handling device interrupts and increase in brightness as the system becomes
    idle.

    Stuck in the OFF state suggests that that the system never completed
    initialization;  Stuck in the ON state would indicated that the system
    intialialized, but is not takint interrupts.

  - If a fatal assertion or a fatal unhandled exception occurs, the LED will flash
    strongly as a slow, 2Hz rate.

LPCXpresso Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

  General Architecture Settings:

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

       CONFIG_ARCH_CHIP_LPC1768=y

    CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
       hence, the board that supports the particular chip or SoC.

       CONFIG_ARCH_BOARD=lpcxpresso-lpc1768

    CONFIG_ARCH_BOARD_name - For use in C code

       CONFIG_ARCH_BOARD_LPCEXPRESSO=y

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
    CONFIG_CAN_TSEG2 = the number of CAN time quanta in segment 2. Default: 7

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
 
  LPC17xx USB Host Configuration (the LPCXpresso does not support USB Host)

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

Configurations
^^^^^^^^^^^^^^

Each LPCXpresso configuration is maintained in a sudirectory and can be
selected as follow:

    cd tools
    ./configure.sh lpcxpresso-lpc1768/<subdir>
    cd -
    . ./setenv.sh

Where <subdir> is one of the following:

  dhcpd:
    This builds the DCHP server using the apps/examples/dhcpd application
    (for execution from FLASH.) See apps/examples/README.txt for information
    about the dhcpd example.

    Jumpers: Nothing special.  Use the default base board jumper
    settings.

  nsh:
    Configures the NuttShell (nsh) located at apps/examples/nsh.  The
    Configuration enables both the serial and telnet NSH interfaces.
    Support for the board's SPI-based MicroSD card is included
    (but not passing tests as of this writing).

    NOTE: At present, the value for the SD SPI frequency is too
    high and the SD will fail.  Setting that frequency to 400000
    removes the problem. TODO:  Tune this frequency to some optimal
    value.

    Jumpers: J55 must be set to provide chip select PIO1_11 signal as
    the SD slot chip select.

  nx:
    And example using the NuttX graphics system (NX).  This example
    uses the UG-9664HSWAG01 driver.

    Jumpers:  There are several jumper settings needed by the OLED.
    All are the default settings:
    
    J42: Close to select the SPI interface (Default: closed)
    J43: Close to support OLED command/data select (Default: closed)
    J44: Close to allow control of OLED voltage (Default: closed)
    J45: Close to select SPI clock (Default: closed)
    J46: Close SPI data input (MOSI) (Default:closed)

  ostest:
    This configuration directory, performs a simple OS test using
    apps/examples/ostest.
 
    Jumpers: Nothing special.  Use the default base board jumper
    settings.

  thttpd:
    This builds the THTTPD web server example using the THTTPD and
    the apps/examples/thttpd application.

    NOTE:  You will need to build the NXFLAT toolchain as described
    above in order to use this example.

    See also note above with regard to the EABI/OABI buildroot
    toolchains.  This example can only be built using the older
    OABI toolchain.

    Jumpers: Nothing special.  Use the default base board jumper
    settings.

  usbstorage:
    This configuration directory exercises the USB mass storage
    class driver at apps/examples/usbstorage.  See apps/examples/README.txt
    for more information.

    NOTE: At present, the value for the SD SPI frequency is too
    high and the SD will fail.  Setting that frequency to 400000
    removes the problem. TODO:  Tune this frequency to some optimal
    value.
 
    Jumpers: J55 must be set to provide chip select PIO1_11 signal as
    the SD slot chip select.

